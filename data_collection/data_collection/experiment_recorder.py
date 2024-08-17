import os
import sys
import time
import threading

import rclpy
import rclpy.executors

from rosidl_runtime_py import message_to_yaml

import numpy as np
import pandas as pd
import argparse

from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import ExecuteTrajectory

from data_collection.robot_interface import RobotInterfaceNode
from data_collection.camera_recorder import CameraRecorderNode
from data_collection.kuka_wrench_recorder import WrenchSubscriberNode
from data_collection.submodules.ati_sensor_socket import NetFTSensor

from data_collection.submodules.utils import get_offset_pose, frange

from datetime import datetime  
from scipy.ndimage import generic_filter  
from scipy.signal import find_peaks 

def flat_region_filter(data: np.array, window_size = 200): 
    stddev = generic_filter(data, np.std, size=window_size) 
    movmean_stddev = pd.Series(stddev).rolling(window=window_size).mean() 
    idx_min_std = np.argmin(movmean_stddev) 
    idx_flat = np.arange(max([0,idx_min_std-window_size]), idx_min_std).astype(int) 
    data_flat_filt = data[idx_flat]  
    return idx_flat, data_flat_filt 

def filter_wrench_flat(data: pd.DataFrame, window_size=200) -> np.array: 
    FZ = data.loc[:,'FZ'].to_numpy()
    idx_flat, _ = flat_region_filter(FZ, window_size) 
    wrench_filtered = data.loc[idx_flat,['FX','FY','FZ','TX','TY','TZ']].to_numpy()  
    wrench_filtered_mean = np.mean(wrench_filtered, axis=0)
    return wrench_filtered_mean 

def clip_past_contact(data: pd.DataFrame) -> [pd.DataFrame, int, list]: 
    FZ = data.loc[:,'FZ'].to_numpy()
    # idx_peaks, properties = find_peaks(np.diff(np.array(-data.loc[:,'FZ'])), distance=500)
    idx_peaks, properties = find_peaks((np.array(-data.loc[:,'FZ'])), distance=500, height=20)
    properties['peak_heights'] if 'peak_heights' in properties else np.diff(np.array(-data.loc[idx_peaks,'FZ']))
    num_peaks = min([2,len(idx_peaks)])
    min_samples = 250 
    
    if len(idx_peaks) != 0: 
        idx_clip = idx_peaks[-num_peaks]  
    else: 
        idx_clip = len(data)-min_samples 

    idx_clip = min(idx_clip, len(data)-min_samples) # have at least min num of samples 
    data_clipped = data.loc[idx_clip:,:] 
    data_clipped = data_clipped.reset_index(drop=True)

    return data_clipped, idx_clip, idx_peaks 

class wrench_data_container:

    def __init__(self, sensor, sensor_frequency):

        self.sensor = sensor
        self.sensor_frequency = sensor_frequency
        self.running = False
        self.data_thread = None

        self.signal_averaged = np.zeros(6)
        self.complete_signal = []

        self.wrench_timestamp = None  # will store timestamp of last packet

    def start_filling(self):

        if not self.running:

            self.running = True  # Thread Active
            self.data_thread = threading.Thread(target=self._store_data)
            self.data_thread.start()

        else:
            print("DATA THREAD IS ALREADY RUNNING")


    def _store_data(self):

        while self.running:
            
            try:
                time.sleep(1/self.sensor_frequency)  # To match data publishing rate of ATI

                _wrench_packet = self.sensor.get_most_recent_data()

                # print("Force-Torque Data:", np.array2string(_wrench_packet.force_torque_data, precision=3, separator=', ', suppress_small=True))

                if _wrench_packet != "No data received yet." :

                    try: 
                        _wrench = _wrench_packet.get_force_torque_array()
                        self.wrench_timestamp = _wrench_packet.get_ft_timestamp()
                        self.complete_signal.append(_wrench)
                       
                    except Exception as e:
                        print("Exception error occured!")
            
            except:
                
                print("Exception error occured in store_data !")

    def stop_filling(self):

        if self.running:
            # print("Stopping DATA THREAD")
            self.running=False

            if self.data_thread:
                self.data_thread.join()

        self.signal_averaged = np.mean(self.complete_signal, axis=0)

        return self.complete_signal, self.signal_averaged, self.wrench_timestamp

        

class ExperimentRecorder:
    DATA_FOLDER = "/home/aero/data/impedence_control_data"

    attempt_height = 0.1
    sensor_height = 0.0958
    ati_sensor_frequency = 500

    def __init__(self, trial_num: int) -> None:
        self.robot_interface_node = RobotInterfaceNode()
        self.camera_recorder_node = CameraRecorderNode()
        self.kuka_wrench_recorder_node = WrenchSubscriberNode()

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.camera_recorder_node)
        self.executor.add_node(self.kuka_wrench_recorder_node)

        self.wrench_sensor = NetFTSensor("192.168.10.100")
        self.wrench_sensor.start_streaming()

        self.rgb_folder = f"{self.DATA_FOLDER}/trial_{trial_num}/rgb"
        self.csv_folder = f"{self.DATA_FOLDER}/trial_{trial_num}/csv"
        if not os.path.exists(self.rgb_folder):
            os.makedirs(self.rgb_folder)
        if not os.path.exists(self.csv_folder):
            os.makedirs(self.csv_folder)

        self.csv_result_name = f"{self.csv_folder}/result.csv"
        self.csv_tare_name = f"{self.csv_folder}/tare.csv"

        self.tool_dict = {
            "X": 0,
            "Y": 0,
            "Z": self.sensor_height,
            "A": 0,
            "B": 0,
            "C": 0,
        }
        self.inverse_tool_dict = {
            "X": 0,
            "Y": 0,
            "Z": -self.sensor_height,
            "A": 0,
            "B": 0,
            "C": 0,
        }
        self.eef_dict = {
            "X": 0,
            "Y": 0,
            "Z": self.sensor_height,
            "A": 90,
            "B": 0,
            "C": 180,
        }
        self.attempt_dict = {
            "X": 0,
            "Y": 0,
            "Z": -self.attempt_height,
            "A": 0,
            "B": 0,
            "C": 0,
        }

    def plan_and_execute(self, target_pose: Pose, linear: bool, scaling_factor: float = 0.08): # NOTE: TODO lowered scaling_factor (velocity) from 0.1 to 0.01 
        traj = self.robot_interface_node.get_motion_plan(
            target_pose=target_pose, linear=linear, scaling_factor=scaling_factor
        )
        if not traj:
            self.robot_interface_node.get_logger().error("Failed to plan trajectory")
            return False

        if len(traj.joint_trajectory.points) > 0:
            client = self.robot_interface_node.get_motion_execute_client()
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = traj

            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.robot_interface_node, future)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.robot_interface_node.get_logger().error(
                    "Failed to execute trajectory"
                )
                return False

            result_future = goal_handle.get_result_async()

            expect_duration = traj.joint_trajectory.points[-1].time_from_start
            expect_time = time.time() + expect_duration.sec + 1

            while not result_future.done() and time.time() < expect_time:
                time.sleep(0.01)

            previous_robot_joint_state = self.robot_interface_node.get_joint_state()
            robot_reached_target =False
            while not robot_reached_target:
                time.sleep(0.1)

                current_robot_joint_state = self.robot_interface_node.get_joint_state()
                if (
                    np.sum(np.square(np.subtract(current_robot_joint_state.position, previous_robot_joint_state.position))) < 0.01
                    and np.sum(np.square(current_robot_joint_state.velocity)) < 0.01
                ):
                    robot_reached_target = True

                previous_robot_joint_state = current_robot_joint_state

        self.robot_interface_node.get_logger().info("Trajectory executed")
        return True

    def data_record(self, snap: bool = False):

        _final_wrench_time_stamp = None
        camera_timestamp = None
        # final_wrench_list = []

        # final_wrench_list_kuka = [] # TODO(dhanush) : Need to do averaging
        # time.sleep(0.1) # add sleep in order to not record peak of contact wrench  

        # t_record = 0.25 
        # for idx in range(int(t_record * self.ati_sensor_frequency)):

        #     time.sleep(1 / self.ati_sensor_frequency)

        #     _wrench_packet = self.wrench_sensor.get_most_recent_data()  # DATA PACKET OBJ

        #     if idx %  5 == 0 :
        #         print("Force-Torque Data:", np.array2string(_wrench_packet.force_torque_data, precision=3, separator=', ', suppress_small=True))

        #     if _wrench_packet != "No data received yet." :
        #         try:
        #             _final_wrench = _wrench_packet.get_force_torque_array()
        #             _final_wrench_time_stamp = _wrench_packet.get_ft_timestamp()
        #             final_wrench_list.append(_final_wrench)
        #         except Exception as e:
        #             print("Exception error occured!")

        # DEBUGGING: save timeseries wrench as a csv for testing filters 
        # final_wrench_df = pd.DataFrame(np.array(final_wrench_list), columns=['FX','FY','FZ','TX','TY','TZ']) 
        # now = datetime.now() # get current date and time for unique file-naming 
        # final_wrench_df.to_csv("./wrench_timeseries_data/" + "wrench_" + now.strftime("%Y%m%d_%H%M%S") + ".csv") 
    
        # FILTER WRENCH DATA 
        # final_wrench = filter_wrench_flat(final_wrench_df, window_size=50) 

        # if not(final_wrench_list): 
            # print("No ATI data received")

        # comment out block below if filtering data 
        # if final_wrench_list: 
        #     print("Computing final wrench by averaging (no filter applied).\n") 
        #     final_wrench = np.mean(final_wrench_list, axis=0)
        # else: 
        #     print("No ATI data received.\n")
        #     final_wrench = np.zeros(6)  # Default value if no data was received

        # self.robot_interface_node.get_logger().info(f"Final wrench: {final_wrench}") 
        # print("FINAL Force-Torque Data:", np.array2string(final_wrench, precision=3, separator=', ', suppress_small=True))

        if snap:
            camera_timestamp = self.camera_recorder_node.camera_record(self.rgb_folder)

        flange_pose = self.robot_interface_node.get_fk()
        # eef_pose = get_offset_pose(flange_pose, self.eef_dict) # commenting out extra rotation from flange frame to peg frame   
        eef_pose = flange_pose # define eef_pose to be flange frame 
        kuka_wrench = self.kuka_wrench_recorder_node.get_latest_kuka_wrench() 

        return eef_pose, camera_timestamp, kuka_wrench


def main(argv=None):
    # NOTE(dhanush) : Feel free to use this to note anything important : https://www.notion.so/Data_Collection-Script-4438ac3e1c6949aaafaeabd87b7d840a?pvs=4
    _tare_frequency = 10  # NOTE(abhay) : How often we want to tare

    # Perfectly Aligned pose - { POSE OF FLANGE | ROBOT BASE }
    # aligned_pose = Pose(
    #     position=Point(x=0.66599, y=-0.02113, z=0.19188),
    #     orientation=Quaternion(x=-0.0006837, y=0.9999783, z=-0.0023519, w=0.0061102 ),
    # )

    # TODO: Please replace aligned pose with the pendant data
    # TODO: Please replace aligned orientation with the pendant data
    # --------------------- #
    # CROSS PEG
    # aligned_pose = Pose(
    #     position=Point(x=0.65968, y=-0.03372, z=0.18784), 
    #     orientation=Quaternion( x=0.0023562, y=-0.9999826, z=-0.0000127, w=-0.0054105  ),
    # )

    # home_pose = Pose(
    #     position=Point(x=0.65914, y=-0.03378, z=0.3),
    #     orientation=Quaternion( x=0.0023562, y=-0.9999826, z=-0.0000127, w=-0.0054105  ),
    # )
    # --------------------- #
    # SLOTTED CIRCLE PEG
    # aligned_pose = Pose(
    #     position=Point(x=0.65870, y=-0.03536, z=0.19330),   # NEED TO CONFIRM Z THEN START EXPERIMENT 
    #     orientation=Quaternion( x=0.0020944, y=-0.9999822, z=-0.0000117, w=-0.005585  ),
    # )
    # --------------------- #
    # # GEAR PEG 
    # aligned_pose = Pose(
    #     position=Point(x=0.65901, y=-0.03140, z=0.18805), 
    #     orientation=Quaternion(  x=0.0021951, y=-0.9999428, z=-0.0013316, w=-0.0103816   ),
    # )

    # home_pose = Pose(
    #     position=Point(x=0.65970, y=-0.03131, z=0.3),
    #     orientation=Quaternion(  x=0.0021951, y=-0.9999428, z=-0.0013316, w=-0.0103816   ),
    # )
    # --------------------- #
    # EXTRUSION PEG 
    # aligned_pose = Pose(
    #     position=Point(x=0.65792, y=-0.03368, z=0.21247), 
    #     orientation=Quaternion(  x=0.0020905, y=-0.9999401, z=0.0003266, w=-0.0107343   ),
    # )

    #------ CROSS PEG VALIDATION TESTING --------#
    # aligned_pose = Pose(
    #     position=Point(x=0.65896, y=-0.03517, z=0.18818),
    #     orientation=Quaternion(x=0.9999283, y=-0.0099581, z=-0.0064417, w=0.0016349 ),
    # )

    # --------------------- #
    
    # cross peg vision CPE validation 
    aligned_pose = Pose(
        position=Point(x=0.65901, y=-0.03529, z=0.18911),
        orientation=Quaternion( x=-0.0027891, y=0.9999525, z=-0.000323, w=0.0093383 ),
    )

    home_pose = Pose(
        position=Point(x=0.65970, y=-0.03131, z=0.3),
        orientation=Quaternion( x=-0.0027891, y=0.9999525, z=-0.000323, w=0.0093383 ),
    )

    rclpy.init()
    experiment_recorder = ExperimentRecorder(int(sys.argv[1])) 


    # TODO : option to continue previous experiment 
    flag_continue_prev_exp = False    

    et = threading.Thread(target=experiment_recorder.executor.spin)
    et.start()

    # --------------------------------------------------------------------- #
    # --------------------MAIN - DATA - COLLECTION------------------------- #

    #------------ beginning of data collection offset generation ------------#

    # # Generate the ranges - data collection - TODO 
    # x_offsets = np.array(list(frange(-0.005, 0.0051, 0.002)))
    # y_offsets = np.array(list(frange(-0.005, 0.0051, 0.002))) 
    # a_offsets = np.array(list(frange(-10.0, 10.1, 4)))
    # b_offsets = np.array(list(frange(-10.0, 10.1, 4)))
    # c_offsets = np.array(list(frange(-10.0, 10.1, 4)))
    # z_offset = 0

    # # Create the meshgrid for all combinations
    # x, y, a, b, c = np.meshgrid(x_offsets, y_offsets, a_offsets, b_offsets, c_offsets, indexing='ij')

    # # Flatten the meshgrid arrays
    # x = x.flatten()
    # y = y.flatten()
    # a = a.flatten()
    # b = b.flatten()
    # c = c.flatten() 

    # # NISARA CHANGES START
    # # Generate the ranges - data collection - TODO 
    # x_offsets = np.array(list(frange(-0.005, 0.0051, 0.000428)))
    # y_offsets = np.array(list(frange(-0.005, 0.0051, 0.000428))) 
    # a_offsets = np.array(list(frange(-15.0, 15.1, 1.428)))
    # b_offsets = np.array(list(frange(-15.0, 10.1, 1.428)))
    # c_offsets = np.array(list(frange(-10.0, 15.1, 1.428)))
    # z_offset = 0

    # # # Create the meshgrid for all combinations
    # # x, y, a, b, c = np.meshgrid(x_offsets, y_offsets, a_offsets, b_offsets, c_offsets, indexing='ij')

    # # # Flatten the meshgrid arrays
    # # x = x.flatten()
    # # y = y.flatten()
    # # a = a.flatten()
    # # b = b.flatten()
    # # c = c.flatten() 

    # offsets = [x_offsets, y_offsets, a_offsets, b_offsets, c_offsets]
    # total_length = sum(len(offset) for offset in offsets)
    # x = np.zeros(total_length)
    # y = np.zeros(total_length)
    # a = np.zeros(total_length)
    # b = np.zeros(total_length)
    # c = np.zeros(total_length)
 
    # x[:len(x_offsets)] = x_offsets
    
    # y[len(x_offsets):len(x_offsets) + len(y_offsets)] = y_offsets

    # a[len(x_offsets) + len(y_offsets):len(x_offsets) + len(y_offsets) + len(a_offsets)] = a_offsets

    # b[len(x_offsets) + len(y_offsets) + len(a_offsets):len(x_offsets) + len(y_offsets) + len(a_offsets) + len(b_offsets)] = b_offsets

    # c[len(x_offsets) + len(y_offsets) + len(a_offsets) + len(b_offsets):] = c_offsets

    # # Flatten the meshgrid arrays
    # x = x.flatten()
    # y = y.flatten()
    # a = a.flatten()
    # b = b.flatten()
    # c = c.flatten() 

    # # NISARA CHANGES END

    # x = np.array([-0.001])
    # y = np.array([0])
    # a = np.array([0])
    # b = np.array([0])
    # c = np.array([0])

    #------------ end of data collection offset generation ------------#

    #------------ beginning of manual offset definition ------------#

    x = np.array([0.005,0.005,0.005,0.005,0.005])
    y = np.array([0,0,0,0,0])
    a = np.array([0,0,0,0,0])
    b = np.array([-1,-1,-1,-1,-1])
    c = np.array([0,0,0,0,0])
    z_offset = np.array([0,0,0,0,0])

    #------------ end of manual offset definition ------------#
    

    #------------ beginning of tolerance test offset generation ------------#

    # print("#------------------- PEG-HOLE TOLERANCE TEST -------------------#")

    # NOTE: CHANGE SPEED TO 0.01 FOR TOLERANCE TEST 

    # x_range = np.array(list(np.linspace(-0.002,0.004,31)))
    # y_range = np.array([0]) 
    # z_range = np.array([0]) 
    # a_range = np.array([0]) 
    # b_range = np.array([0])  
    # c_range = np.array([0]) 

    # range_list = [x_range, y_range, z_range, a_range, b_range, c_range] 
    # range_lengths = [len(x_range),len(y_range),len(z_range),len(a_range),len(b_range),len(c_range)]

    # offsets_list = np.empty((sum(range_lengths),6))  
    # for i, range in enumerate(range_list):
    #     offsets = np.zeros((len(range),6))  
    #     offsets[:,i] = range 
    #     offsets_list[sum(range_lengths[:i]):sum(range_lengths[:i+1]),:] = offsets   
        
    # x = offsets_list[:,0]
    # y = offsets_list[:,1]
    # z_offset = offsets_list[:,2]
    # a = offsets_list[:,3]
    # b = offsets_list[:,4]
    # c = offsets_list[:,5]

    # N_samples = 500
    # x = np.random.rand(N_samples) * 0.010 - 0.005 
    # y = np.random.rand(N_samples) * 0.010 - 0.005
    # a = np.random.rand(N_samples) * 20 - 10
    # b = np.random.rand(N_samples) * 20 - 10
    # c = np.random.rand(N_samples) * 20 - 10
    # z_offset = np.zeros(N_samples)  

    #------------ end of tolerance test offset generation ------------#
    

    # Create a DataFrame with all combinations
    data = {
        'X': x, 'Y': y, 'Z': z_offset, 'A': a, 'B': b, 'C': c,
        'Fx': np.nan, 'Fy': np.nan, 'Fz': np.nan, 'Tx': np.nan, 'Ty': np.nan, 'Tz': np.nan,

        'pose': 'nan',
        'rgb_timestamp': 'nan',
        'wrench_timestamp': np.nan,

        'KUKA_Fx': np.nan, 'KUKA_Fy': np.nan, 'KUKA_Fz': np.nan, 'KUKA_Tx': np.nan, 'KUKA_Ty': np.nan, 'KUKA_Tz': np.nan
    }

    result_dataframe = pd.DataFrame(data)

    tare_dataframe = pd.DataFrame(
        columns=["Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "pose", "wrench_timestamp"]
    )
    tare_dataframe_index = 0

    experiment_recorder.plan_and_execute(home_pose, linear=True)

    previous_wrench_timestamp = None
    current_offset_dataframe_index = 0

    # Shuffle data - TODO : uncomment for data collection, comment for tolerance test 
    #--------------------------------- 
    if flag_continue_prev_exp == False : 
        result_dataframe = result_dataframe.sample(frac=1).reset_index(drop=True) 
    else: 
        result_dataframe = pd.read_csv(experiment_recorder.csv_result_name) 
    #--------------------------------- 
    
    #------------- VALIDATE ALIGNED POSE BY EXECUTING INSERTION 
    offset_dict = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0,
            "A": 0.0,
            "B": 0.0,
            "C": 0.0,
        }
    current_target_pose = get_offset_pose(
            get_offset_pose(
                get_offset_pose(aligned_pose, experiment_recorder.tool_dict),
                offset_dict,
            ),
            experiment_recorder.inverse_tool_dict,
        ) 
    current_attempt_pose = get_offset_pose(
            current_target_pose, experiment_recorder.attempt_dict
        )
    # move to attempt pose 
    success = experiment_recorder.plan_and_execute(target_pose=current_attempt_pose, linear=True)
    if not success:
        print("Did not succeed to move to attempt pose.")

    # move to target pose 
    success = experiment_recorder.plan_and_execute(target_pose=current_target_pose, linear=True, scaling_factor=0.01)
    if not success:
        experiment_recorder.plan_and_execute(home_pose, linear=True)
        
    # move back to attempt pose
    retract_pose = get_offset_pose(
        experiment_recorder.robot_interface_node.get_fk(), experiment_recorder.attempt_dict
    )
    success = experiment_recorder.plan_and_execute(target_pose=retract_pose, linear=True, scaling_factor=0.075)
    print(f"Successs : {success}")
    if not success:
        experiment_recorder.plan_and_execute(home_pose, linear=True)

    for index, row in result_dataframe.iterrows():

        # if continuing prev exp, skip row if it's already been recorded 
        if flag_continue_prev_exp: 
            if isinstance(row['pose'], str): # if pose column is a string then it has already been written
                print('skipping trial index {}'.format(index)) 
                current_offset_dataframe_index += 1
                continue  

        experiment_recorder.robot_interface_node.get_logger().info(f"Current datapoint: {current_offset_dataframe_index}/{len(result_dataframe)}")
        current_offset_dataframe_index += 1

        # get offset pose for trial 
        offset_dict = {
            "X": row["X"],
            "Y": row["Y"],
            "Z": row["Z"],
            "A": row["A"],
            "B": row["B"],
            "C": row["C"],
        }

        # printout trial info 
        print("#----- EXECUTING TRIAL " + str(index) + " of " + str(result_dataframe.shape[0]) + "-----# \n") 

        # compute target pose using offset and true hole pose 
        current_target_pose = get_offset_pose(
            get_offset_pose(
                get_offset_pose(aligned_pose, experiment_recorder.tool_dict),
                offset_dict,
            ),
            experiment_recorder.inverse_tool_dict,
        )
        
        # compute attempt pose from target pose 
        current_attempt_pose = get_offset_pose(
            current_target_pose, experiment_recorder.attempt_dict
        )

        # move to attempt pose 
        success = experiment_recorder.plan_and_execute(target_pose=current_attempt_pose, linear=True)
        if not success:
            print("Did not succeed to move to attempt pose.")
            continue

        # start reading wrench - NEW FEAT
        ATI_data_container = wrench_data_container(sensor=experiment_recorder.wrench_sensor, 
                                                   sensor_frequency=experiment_recorder.ati_sensor_frequency)
        
        ATI_data_container.start_filling()

        # move to target pose 
        success = experiment_recorder.plan_and_execute(target_pose=current_target_pose, linear=True, scaling_factor=0.08) # TODO: change scaling factor 
        if not success:
            experiment_recorder.plan_and_execute(home_pose, linear=True)
            continue

        # record data at contact -> POSE, CAMERA TS, KUKA WRENCH 
        recorded_pose, camera_ts, kuka_wrench = experiment_recorder.data_record(True)

        # stop reading wrench 
        time.sleep(1.5) 
        ati_complete_signal, ati_signal_average, wrench_ts = ATI_data_container.stop_filling()

        # convert wrench data from list to df 
        final_wrench_df = pd.DataFrame(np.array(ati_complete_signal), columns=['FX','FY','FZ','TX','TY','TZ']) 
        # save wrench data as csv 
        now = datetime.now() # get current date and time for unique file-naming 
        final_wrench_df.to_csv("./wrench_timeseries_data/" + "wrench_" + now.strftime("%Y%m%d_%H%M%S") + ".csv")
        # clip the wrench data to only keep data past contact  
        wrench_clipped, _, _ = clip_past_contact(final_wrench_df) 
        # find wrench values by taking mean of flat region of clipped data 
        final_wrench = filter_wrench_flat(wrench_clipped, window_size=150) 

        # NOTE: moved record pose away away from here to earlier 

        # PREV data record used to give ATI now we have seperate method to do it
        recorded_wrench = final_wrench  # FIXME : filter and then pass as recorded wrench


        # check if sensor is disconnected by checking if measurement is stale 
        if previous_wrench_timestamp and wrench_ts == previous_wrench_timestamp:
            experiment_recorder.robot_interface_node.get_logger().error(
                "Received same force sensor timestamp, please check sensor connection"
            )
            break
        else:
            previous_wrench_timestamp = wrench_ts

        # STORING THE DATA FROM THE ATTEMPT
        row["Fx"] = recorded_wrench[0]
        row["Fy"] = recorded_wrench[1]
        row["Fz"] = recorded_wrench[2]
        row["Tx"] = recorded_wrench[3]
        row["Ty"] = recorded_wrench[4]
        row["Tz"] = recorded_wrench[5]

        # STORING KUKA DATA
        row["KUKA_Fx"] = kuka_wrench[0]
        row["KUKA_Fy"] = kuka_wrench[1]
        row["KUKA_Fz"] = kuka_wrench[2]
        row["KUKA_Tx"] = kuka_wrench[3]
        row["KUKA_Ty"] = kuka_wrench[4]
        row["KUKA_Tz"] = kuka_wrench[5]

        row["pose"] = message_to_yaml(recorded_pose)
        row["rgb_timestamp"] = camera_ts
        row["wrench_timestamp"] = wrench_ts

        result_dataframe.loc[index] = row

        # move back to attempt pose
        retract_pose = get_offset_pose(
            experiment_recorder.robot_interface_node.get_fk(), experiment_recorder.attempt_dict
        )
        success = experiment_recorder.plan_and_execute(target_pose=retract_pose, linear=True, scaling_factor=0.1)
        print(f"Successs : {success}")
        if not success:
            experiment_recorder.plan_and_execute(home_pose, linear=True)
            continue

        # NOTE : TARING | every m-th trial, record force/torque to tare 
        if index % _tare_frequency == 0:  # NOTE(dhanush) :  Time to tare !
            experiment_recorder.plan_and_execute(home_pose, linear=True)

            recorded_pose, _, _  = experiment_recorder.data_record()  # IMAGE AND KUKA WRENCH NOT NEEDED
            # NOTE : We seperated ATI stuff
            _recorded_wrench_packet = experiment_recorder.wrench_sensor.get_most_recent_data()
            recorded_wrench = _recorded_wrench_packet.get_force_torque_array()
            wrench_ts = _recorded_wrench_packet.get_ft_timestamp()

            tare_dataframe.loc[tare_dataframe_index] = [
                recorded_wrench[0],
                recorded_wrench[1],
                recorded_wrench[2],
                recorded_wrench[3],
                recorded_wrench[4],
                recorded_wrench[5],
                message_to_yaml(recorded_pose),
                wrench_ts,
            ]
            tare_dataframe_index += 1

        # save tare results to csv
        result_dataframe.to_csv(experiment_recorder.csv_result_name, index=False)
        tare_dataframe.to_csv(experiment_recorder.csv_tare_name, index=False)

    experiment_recorder.plan_and_execute(home_pose, linear=True)
    experiment_recorder.robot_interface_node.get_logger().info("Data collection finished")

    experiment_recorder.robot_interface_node.destroy_node()
    experiment_recorder.camera_recorder_node.destroy_node()
    et.join()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
