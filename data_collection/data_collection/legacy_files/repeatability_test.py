import os
import shutil
import sys
import time
import threading
import json
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

from data_collection.submodules.utils import get_offset_pose, frange, pose_to_position_and_euler

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
            print("Started Filling....")
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
                # print(f"Wrench Packet : {_wrench_packet}")
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
            print("Stopping DATA THREAD")
            self.running=False

            if self.data_thread:
                self.data_thread.join()

        self.signal_averaged = np.mean(self.complete_signal, axis=0)

        return self.complete_signal, self.signal_averaged, self.wrench_timestamp

        

class ExperimentRecorder:
    DATA_FOLDER = "/home/aero/data/inclined_surface"

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

    # --------------------- #
    
    # cross peg vision CPE validation 
    # aligned_pose = Pose(
    #     position=Point(x=0.65901, y=-0.03529, z=0.18911),
    #     orientation=Quaternion( x=-0.0027891, y=0.9999525, z=-0.000323, w=0.0093383 ),
    # )

    # aligned_pose = Pose(
    #     position=Point(x=0.65601, y=-0.03606, z=0.18811),
    #     orientation=Quaternion( x=-0.0027891, y=0.9999525, z=-0.000323, w=0.0093383 ),
    # )

    # home_pose = Pose(
    #     position=Point(x=0.65970, y=-0.03131, z=0.3),
    #     orientation=Quaternion( x=-0.0027891, y=0.9999525, z=-0.000323, w=0.0093383 ),
    # )

    aligned_pose = Pose(
        position=Point(x=0.520, y=-0.584, z=0.43651),
        orientation=Quaternion( x=0.0, y=1.0, z=0.0, w=0.0),
    )

    home_pose = Pose(
        position=Point(x=-0.31612, y=-0.585, z=0.33165),
        orientation=Quaternion( x=0.0, y=1.0, z=0.0, w=0.0 ),
    )

    rclpy.init()
    trial_number = int(sys.argv[1])
    experiment_recorder = ExperimentRecorder(trial_number) 


    # TODO : option to continue previous experiment 
    flag_continue_prev_exp = False    

    et = threading.Thread(target=experiment_recorder.executor.spin)
    et.start()



    x = np.array([0])
    y = np.array([0])
    a = np.array([0])
    b = np.array([0])
    c = np.array([0])
    z_offset = np.array([-0.149])


    

    # Create a DataFrame with all combinations
    data = {
        'X': x, 'Y': y, 'Z': z_offset, 'A': a, 'B': b, 'C': c,
        'Fx': np.nan, 'Fy': np.nan, 'Fz': np.nan, 'Tx': np.nan, 'Ty': np.nan, 'Tz': np.nan,

        'pose': 'nan',
        'rgb_timestamp': 'nan',
        'commanded_pose' : 'nan',
        'recorded_pose' : 'nan',
        'wrench_timestamp': np.nan,
        

        'KUKA_Fx': np.nan, 'KUKA_Fy': np.nan, 'KUKA_Fz': np.nan, 'KUKA_Tx': np.nan, 'KUKA_Ty': np.nan, 'KUKA_Tz': np.nan
    }

    result_dataframe = pd.DataFrame(data)

    tare_dataframe = pd.DataFrame(
        columns=["Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "pose", "wrench_timestamp"]
    )
    tare_dataframe_index = 0
    print("Sleeping for 5 Seconds")
    time.sleep(5)

    experiment_recorder.plan_and_execute(home_pose, linear=True)

    previous_wrench_timestamp = None
    current_offset_dataframe_index = 0

    s_init_pose = home_pose
    s_target_pose = Pose(
        position=Point(x=-0.31612, y=-0.585, z=0.290),
        orientation=Quaternion( x=0.0, y=1.0, z=0.0, w=0.0 ),
    )

    # Shuffle data - TODO : uncomment for data collection, comment for tolerance test 
    #--------------------------------- 
    # if flag_continue_prev_exp == False : 
    #     result_dataframe = result_dataframe.sample(frac=1).reset_index(drop=True) 
    # else: 
    #     result_dataframe = pd.read_csv(experiment_recorder.csv_result_name) 
    #--------------------------------- 
    
    # #------------- VALIDATE ALIGNED POSE BY EXECUTING INSERTION 
    # offset_dict = {
    #         "X": 0.0,
    #         "Y": 0.0,
    #         "Z": 0.0,
    #         "A": 0.0,
    #         "B": 0.0,
    #         "C": 0.0,
    #     }
    # current_target_pose = get_offset_pose(
    #         get_offset_pose(
    #             get_offset_pose(aligned_pose, experiment_recorder.tool_dict),
    #             offset_dict,
    #         ),
    #         experiment_recorder.inverse_tool_dict,
    #     ) 
    # print(current_target_pose)
    # current_attempt_pose = get_offset_pose(
    #         current_target_pose, experiment_recorder.attempt_dict
    #     )
    # print(current_attempt_pose)
    # # move to attempt pose 
    # success = experiment_recorder.plan_and_execute(target_pose=s_target_pose, linear=True)
    # if not success:
    #     print("Did not succeed to move to attempt pose.")

    # move to target pose 
    # success = experiment_recorder.plan_and_execute(target_pose=s_target_pose, linear=True, scaling_factor=0.08)
    # if not success:
    #     experiment_recorder.plan_and_execute(s_init_pose, linear=True)
        
    # # move back to attempt pose
    # retract_pose = get_offset_pose(
    #     experiment_recorder.robot_interface_node.get_fk(), experiment_recorder.attempt_dict
    # )
    # success = experiment_recorder.plan_and_execute(target_pose=retract_pose, linear=True, scaling_factor=0.075)
    # if not success:
    #     experiment_recorder.plan_and_execute(home_pose, linear=True)

    wtd_folder = f'{experiment_recorder.DATA_FOLDER}/wrench_timeseries_data/trail_{trial_number}'
    if os.path.exists(wtd_folder):
        shutil.rmtree(wtd_folder)
    os.makedirs(wtd_folder)

    for index, row in result_dataframe.iterrows():

        # # if continuing prev exp, skip row if it's already been recorded 
        # if flag_continue_prev_exp: 
        #     if isinstance(row['pose'], str): # if pose column is a string then it has already been written
        #         print('skipping trial index {}'.format(index)) 
        #         current_offset_dataframe_index += 1
        #         continue  

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
        # print(f'Offset X: {offset_dict["X"]}')
        # print(f'Offset Y: {offset_dict["Y"]}')
        # print(f'Offset Z: {offset_dict["Z"]}')
        # print(f'Offset A: {offset_dict["A"]}')
        # print(f'Offset B: {offset_dict["B"]}')
        # print(f'Offset C: {offset_dict["C"]}')

        # # compute target pose using offset and true hole pose 
        # current_target_pose = get_offset_pose(
        #     get_offset_pose(
        #         get_offset_pose(aligned_pose, experiment_recorder.tool_dict),
        #         offset_dict,
        #     ),
        #     experiment_recorder.inverse_tool_dict,
        # )
        
        # # compute attempt pose from target pose 
        # current_attempt_pose = get_offset_pose(
        #     current_target_pose, experiment_recorder.attempt_dict
        # )

        # # move to attempt pose 
        # success = experiment_recorder.plan_and_execute(target_pose=current_attempt_pose, linear=True)
        # if not success:
        #     print("Did not succeed to move to attempt pose.")
        #     continue

        # start reading wrench - NEW FEAT
        ATI_data_container = wrench_data_container(sensor=experiment_recorder.wrench_sensor, 
                                                   sensor_frequency=experiment_recorder.ati_sensor_frequency)
        
        ATI_data_container.start_filling()

        # move to target pose 
        success = experiment_recorder.plan_and_execute(target_pose=s_target_pose, linear=True, scaling_factor=0.1) # TODO: change scaling factor 
        time.sleep(2)
        print("Recording pose...")
        commanded_pose = experiment_recorder.robot_interface_node.get_fk()
        real_pose = experiment_recorder.robot_interface_node.get_fk()
        if not success:
            experiment_recorder.plan_and_execute(s_init_pose, linear=True)
            continue
        # record data at contact -> POSE, CAMERA TS, KUKA WRENCH 
        recorded_pose, camera_ts, kuka_wrench = experiment_recorder.data_record(True)

        # stop reading wrench 
        time.sleep(1.5) 
        ati_complete_signal, ati_signal_average, wrench_ts = ATI_data_container.stop_filling()
        # print("NO PROBLEM ....!")
        # print(np.array(ati_complete_signal))

        # convert wrench data from list to df 
        final_wrench_df = pd.DataFrame(np.array(ati_complete_signal), columns=['FX','FY','FZ','TX','TY','TZ']) 
        # save wrench data as csv 
        now = datetime.now() # get current date and time for unique file-naming 
        # f"{experiment_recorder.DATA_FOLDER}/wrench_timeseries_data/wrench_{now.strftime("%Y%m%d_%H%M%S")}.csv"
        # wtd_folder = f'{experiment_recorder.DATA_FOLDER}/wrench_timeseries_data/trail_{trial_number}'
        # if os.path.exists(wtd_folder):
        #     shutil.rmtree(wtd_folder)
        # os.makedirs(wtd_folder)
        final_wrench_df.to_csv(f'{wtd_folder}/{str(wrench_ts)}.csv')
        # final_wrench_df.to_csv("./wrench_timeseries_data/" + "wrench_" + now.strftime("%Y%m%d_%H%M%S") + ".csv")
        # clip the wrench data to only keep data past contact  
        wrench_clipped, _, _ = clip_past_contact(final_wrench_df) 
        wrench_clipped.to_csv(f'{wtd_folder}/{str(wrench_ts)}_clipped.csv')
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
        row["commanded_pose"] = json.dumps(pose_to_position_and_euler(commanded_pose))
        row["recorded_pose"] = json.dumps(pose_to_position_and_euler(real_pose))
        row["rgb_timestamp"] = camera_ts
        row["wrench_timestamp"] = wrench_ts

        result_dataframe.loc[index] = row

        # move back to home pose
        # retract_pose = get_offset_pose(
        #     experiment_recorder.robot_interface_node.get_fk(), experiment_recorder.attempt_dict
        # )
        # print("Retracting Back ...")
        success = experiment_recorder.plan_and_execute(target_pose=s_init_pose, linear=True, scaling_factor=0.1)
        if not success:
            experiment_recorder.plan_and_execute(s_init_pose, linear=True)
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
