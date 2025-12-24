import os
import sys
import time
import threading
import cv2
import rclpy
import rclpy.executors

from typing import List
from datetime import datetime

import numpy as np
import pandas as pd

from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import ExecuteTrajectory

from data_collection.robot_interface import RobotInterfaceNode
from data_collection.camera_recorder import CameraRecorderNode
from data_collection.force_wrench_recorder import ForceWrenchRecorder
from data_collection.podah_models.simulation.src.trajctory_generation_functions import invT

from data_collection.submodules.ati_sensor_socket import NetFTSensor
from data_collection.submodules.utils import (
    filter_wrench_flat,
    clip_past_contact,
    get_offset_pose,
    construct_offset_dict,
    construct_transform_matrix,
    get_peg_to_hole_transform,
)

from data_collection.podah_models.podah.podah_pipeline import podah     
from scipy.spatial.transform import Rotation as R 
from PIL import Image

import random 


def image_cropper(img):

    center_x, center_y = 675, 210  # Center coordinates for cropping
    crop_width, crop_height = 660, 370  # Dimensions for the crop
    resize_width, resize_height = 224, 224  # Dimensions for the resized image

    # Calculate the cropping box (left, upper, right, lower)
    left = center_x - crop_width / 2
    top = center_y - crop_height / 2
    right = center_x + crop_width / 2
    bottom = center_y + crop_height / 2

    # Ensure the cropping box is within the image boundaries
    left = max(0, left)
    top = max(0, top)
    right = min(img.width, right)
    bottom = min(img.height, bottom)

    # Crop the image
    cropped_image = img.crop((left, top, right, bottom))

    # Resize the cropped image
    resized_image = cropped_image.resize((resize_width, resize_height))

    return resized_image

### define functions and classes
class ExperimentExecutor:
    DATA_FOLDER = "/home/aero/exp_exec_data"

    # define experiment constants
    attempt_height = 0.1
    sensor_height = 0.0958
    ati_sensor_frequency = 500

    def __init__(self, trial_num: int) -> None:
        self.robot_interface_node = RobotInterfaceNode()
        self.camera_recorder_node = CameraRecorderNode()

        self.ft_sensor = NetFTSensor("192.168.10.100")
        self.ft_sensor.start_streaming()

        self.force_wrench_recorder = ForceWrenchRecorder(
            self.ft_sensor, self.ati_sensor_frequency
        )

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.camera_recorder_node)

        self.rgb_folder = f"{self.DATA_FOLDER}/trial_{trial_num}/rgb"
        self.csv_folder = f"{self.DATA_FOLDER}/trial_{trial_num}/csv"
        self.tf_folder = f"{self.DATA_FOLDER}/trial_{trial_num}/tf"
        if not os.path.exists(self.rgb_folder):
            os.makedirs(self.rgb_folder)
        if not os.path.exists(self.csv_folder):
            os.makedirs(self.csv_folder)
        if not os.path.exists(self.tf_folder):
            os.makedirs(self.tf_folder)

        self.csv_result_name = f"{self.csv_folder}/result.csv"
        self.csv_tare_name = f"{self.csv_folder}/tare.csv"

    def plan_and_execute(
        self,
        target_pose: Pose,
        linear: bool,
        force_wrench_record: bool = False,
        scaling_factor: float = 0.08,
    ) -> bool:
        traj = self.robot_interface_node.get_motion_plan(
            target_pose, linear, scaling_factor
        )
        if force_wrench_record:
            self.force_wrench_recorder.start_filling()

        if traj is None:
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
            robot_reached_target = False
            while not robot_reached_target:
                time.sleep(0.1)

                current_robot_joint_state = self.robot_interface_node.get_joint_state()
                if (
                    np.sum(
                        np.square(
                            np.subtract(
                                current_robot_joint_state.position,
                                previous_robot_joint_state.position,
                            )
                        )
                    )
                    < 0.01
                    and np.sum(np.square(current_robot_joint_state.velocity)) < 0.01
                ):
                    robot_reached_target = True

                previous_robot_joint_state = current_robot_joint_state

        self.robot_interface_node.get_logger().info("Trajectory executed")
        return True

    def data_record(self, snap: bool = False):
        camera_timestamp = (
            self.camera_recorder_node.camera_record(self.rgb_folder) if snap else None
        )
        force_wrench = self.__record_force_wrench__()
        flange_pose = self.robot_interface_node.get_fk()

        return camera_timestamp, force_wrench, flange_pose
    
    def __record_force_wrench__(self, timeout: float = 1.5):
        time.sleep(timeout)
        ati_complete_signal, ati_signal_average, wrench_ts = (
            self.force_wrench_recorder.stop_filling()
        )

        ft_df = pd.DataFrame(
            np.array(ati_complete_signal), columns=["FX", "FY", "FZ", "TX", "TY", "TZ"]
        )
        ft_df.to_csv(f"{self.tf_folder}/{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        wrench_clipped, _, _ = clip_past_contact(ft_df)

        ft = filter_wrench_flat(wrench_clipped, window_size=150)

        return ft
    
def package_single_observation(image_path, force_wrench, flange_pose, target_pose, tare_val, ground_truth_tare_val): 
    force_wrench = np.delete(force_wrench, obj=2) # remove FZ 
    img = Image.open(image_path).convert('RGB')
    # img.show()  # DEBUG
    T_F_P = construct_transform_matrix(
        rotation=np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]]),
        translation=np.array([0, 0, 0.0958]),
    )

    T_B_Fc = construct_transform_matrix(
        R.from_quat([flange_pose.orientation.x, flange_pose.orientation.y, flange_pose.orientation.z, flange_pose.orientation.w]).as_matrix(),
        translation=(np.array([flange_pose.position.x, flange_pose.position.y, flange_pose.position.z]))
    )

    T_B_Ft = construct_transform_matrix(
        R.from_quat([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]).as_matrix(),
        translation=(np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z]))
    )

    T_P_F = invT(T_F_P) 
    T_Ft_B = invT(T_B_Ft)  
    
    T_Pt_Pc = T_P_F @ T_Ft_B @ T_B_Fc @ T_F_P 
    X,Y,Z = T_Pt_Pc[:3,3] * 1e3 
    C,B,A = R.from_matrix(T_Pt_Pc[:3,:3]).as_euler('xyz',degrees=True)  

    # tare data and transform to match sim sensor coordinates used for model training 
    force_wrench = force_wrench - tare_val + ground_truth_tare_val 

    # transform data 
    Fx = force_wrench[0]
    Fy = force_wrench[1]
    Tx = force_wrench[2]
    Ty = force_wrench[3]
    Tz = force_wrench[4]
    force_wrench = np.array([Fy, Fx, Ty, Tx, -Tz])  
    
    # import pdb; pdb.set_trace() 

    raw_observation_dict = {'raw_wrench': force_wrench,
                                'raw_image': img,
                                'compliance_vector': np.array([X,Y,A,B,C]),
                                'flange_pose':flange_pose}  # TODO(dhanush) : READ FROM CSV
    return raw_observation_dict

def package_multiple_observations(observations,sensor_transform_dict): 
    N_obs = len(observations)
    processed_contact_waypoint_observations_wrench = np.empty((N_obs,5))

    processed_contact_waypoint_observation_pixels = [] 
 
    compliance_vectors = np.empty((N_obs,5))

    poses_peg_wrt_base = np.empty((N_obs,6)) 

    # contact_waypoints_dict : A dictionary containing the "pose of  PEG w.r.t. to KUKA BASE (ANCHOR) in XYZABC format - "xyzabc" should be the assigned key
    # Number of observations in the CPE_input_dict must be equal to the number contact waypoints

    for i, obs in enumerate(observations):  
        
        processed_contact_waypoint_observations_wrench[i,:] = obs['raw_wrench'] 

        cropped_image = np.array(image_cropper(obs['raw_image']))
        # BEFORE WE JUST DID RESIZE 224, 224 RESNET INUT SIZe

        processed_contact_waypoint_observation_pixels.append(cropped_image)   #  IMAGE OBJECT

        compliance_vectors[i,:] = obs['compliance_vector']   

        flange_pose = obs['flange_pose'] 

        # T_B_P = T_B_F @ T_F_P 
        # pose_peg_wrt_base = get_offset_pose(flange_pose,sensor_transform_dict)  # NOTE :THIS IS IN METERS , PEG|KUKA BASE
        # X_B_P = pose_peg_wrt_base.position.x
        # Y_B_P = pose_peg_wrt_base.position.y
        # Z_B_P = pose_peg_wrt_base.position.z
        # C_B_P, B_B_P, A_B_P = R.from_quat([pose_peg_wrt_base.orientation.x,pose_peg_wrt_base.orientation.y,pose_peg_wrt_base.orientation.z,pose_peg_wrt_base.orientation.w]).as_euler('xyz',degrees=True) 

        T_B_F=construct_transform_matrix(
            rotation=R.from_quat([flange_pose.orientation.x,flange_pose.orientation.y,flange_pose.orientation.z,flange_pose.orientation.w]).as_matrix(),
            translation=np.array([flange_pose.position.x,flange_pose.position.y,flange_pose.position.z])
        )

        T_F_P = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0.0958],[0,0,0,1]]) # FIXME: define from argument sensor_transform_dict

        T_B_P = T_B_F @ T_F_P 

        X_B_P = T_B_P[0,3]
        Y_B_P = T_B_P[1,3]
        Z_B_P = T_B_P[2,3]
        C_B_P, B_B_P, A_B_P = R.from_matrix(T_B_P[:3,:3]).as_euler('xyz',degrees=True)

        poses_peg_wrt_base[i,:] = np.array([X_B_P, Y_B_P, Z_B_P, A_B_P, B_B_P, C_B_P]) 

    processed_contact_waypoint_observation_pixels = np.array(processed_contact_waypoint_observation_pixels) 

    CPE_input_dict = {
                        'raw_wrench': processed_contact_waypoint_observations_wrench, 
                        'raw_image': processed_contact_waypoint_observation_pixels, 
                        'compliance_vector': compliance_vectors, 
                     }

    contact_waypoints_dict = {'xyzabc': poses_peg_wrt_base}

    return CPE_input_dict, contact_waypoints_dict

def pose_to_xyzabc(pose):
    X = pose.position.x * 1e3
    Y = pose.position.y * 1e3
    Z = pose.position.z * 1e3 
    C,B,A = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_euler('xyz',degrees=True) 
    return X,Y,Z,A,B,C  

def pose_to_xyzabc_array(pose):
    # outputs in units of mm and deg 
    X = pose.position.x * 1e3
    Y = pose.position.y * 1e3
    Z = pose.position.z * 1e3 
    C,B,A = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_euler('xyz',degrees=True) 
    return np.array([X,Y,Z,A,B,C])  

def xyzabc_array_to_transform(xyzabc): 
    # NOTE: no unit changes in translation, ABC in degrees 
    X = xyzabc[0]
    Y = xyzabc[1]
    Z = xyzabc[2]
    A = xyzabc[3]
    B = xyzabc[4]
    C = xyzabc[5] 
    p = np.array([[X],[Y],[Z]])  
    r = R.from_euler('xyz', [C,B,A], degrees=True).as_matrix()  
    T = np.vstack((np.hstack((r,p)), np.array([0,0,0,1]))) 
    return T 

def pose_to_transform(pose): 
    X = pose.position.x
    Y = pose.position.y
    Z = pose.position.z
    p = np.array([[X],[Y],[Z]])  
    r = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()  
    T = np.vstack((np.hstack((r,p)), np.array([0,0,0,1]))) 
    return T 

def transform_to_pose(T):
    q = R.from_matrix(T[:3,:3]).as_quat() 
    pose = Pose(
        position=Point(x=T[0,3], y=T[1,3], z=T[2,3]),
        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) 
    )
    return pose 

def transform_to_xyzabc_array(T):
    q = R.from_matrix(T[:3,:3]).as_quat() 
    pose = Pose(
        position=Point(x=T[0,3], y=T[1,3], z=T[2,3]),
        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) 
    )
    return pose_to_xyzabc_array(pose) 

def waypoint_to_target_pose(waypoint,flange_pose,sensor_transform): 

    # add z offset 
    waypoint[2] = -25.0 # push down  

    T_P0_Pi = construct_transform_matrix(
                    rotation=R.from_euler('xyz', [waypoint[5],waypoint[4],waypoint[3]], degrees=True).as_matrix(),
                    translation=waypoint[:3] / 1e3 
                )
    T_B_F0 = construct_transform_matrix(
                    rotation=R.from_quat([flange_pose.orientation.x, flange_pose.orientation.y, flange_pose.orientation.z, flange_pose.orientation.w]).as_matrix(),
                    translation=np.array([flange_pose.position.x, flange_pose.position.y, flange_pose.position.z]) 
                )
    T_F_P = sensor_transform
    T_P_F = invT(sensor_transform)  
    T_B_Fi = T_B_F0 @ T_F_P @ T_P0_Pi @ T_P_F   
    q = R.from_matrix(T_B_Fi[:3,:3]).as_quat()  
    target_pose = Pose(
                    position=Point(x=T_B_Fi[0,3],y=T_B_Fi[1,3],z=T_B_Fi[2,3]),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                ) 
    
    return target_pose 

def main(argv=None):
    rclpy.init(args=argv)
    experiment_executor = ExperimentExecutor(int(sys.argv[1]))

    et = threading.Thread(target=experiment_executor.executor.spin)
    et.start()

    experiment_executor.robot_interface_node.get_logger().info("Execution started")

    retraction = True 

    attempt_dict = {
        "X": 0.0,
        "Y": 0.0,
        "Z": -experiment_executor.attempt_height,
        "A": 0.0,
        "B": 0.0,
        "C": 0.0,
    }

    short_retract = {
        "X": 0.0,
        "Y": 0.0,
        "Z": -0.1,
        "A": 0.0,
        "B": 0.0,
        "C": 0.0,
    }

    long_retract = {
        "X": 0.0,
        "Y": 0.0,
        "Z": -0.1,
        "A": 0.0,
        "B": 0.0,
        "C": 0.0,
    }

    sensor_transform = construct_transform_matrix(
        rotation=np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]]),
        translation=np.array([[0], [0], [0.0958]]),
    )
    T_F_P = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0.0958],[0,0,0,1]])
    T_P_F = invT(T_F_P) 

    X_F_P = sensor_transform[0,3] 
    Y_F_P = sensor_transform[1,3] 
    Z_F_P = sensor_transform[2,3] 
    C_F_P, B_F_P, A_F_P = R.from_matrix(sensor_transform[:3,:3]).as_euler('xyz',degrees=True)   
    sensor_transform_dict = { 
        "X":X_F_P,
        "Y":Y_F_P,
        "Z":Z_F_P,
        "A":A_F_P,
        "B":B_F_P,
        "C":C_F_P
    }
    
    inv_sensor_transform = invT(sensor_transform) 
    X_P_F = inv_sensor_transform[0,3] 
    Y_P_F = inv_sensor_transform[1,3] 
    Z_P_F = inv_sensor_transform[2,3] 
    C_P_F, B_P_F, A_P_F = R.from_matrix(inv_sensor_transform[:3,:3]).as_euler('xyz',degrees=True)   
    inv_sensor_transform_dict = { 
        "X":X_P_F,
        "Y":Y_P_F,
        "Z":Z_P_F,
        "A":A_P_F,
        "B":B_P_F,
        "C":C_P_F
    }
    
    # perfect_aligned_pose = Pose(
    #     position=Point(x=0.65960, y=-0.03502, z=0.18838),
    #     orientation=Quaternion(x=-0.0088863, y=0.9999415, z=-0.0025681, w=0.0056081 ),
    # )

    # perfect_aligned_pose = Pose(
    #     position=Point(x=0.65896, y=-0.03517, z=0.18818),
    #     orientation=Quaternion(x=0.9999283, y=-0.0099581, z=-0.0064417, w=0.0016349 ),
    # )

    # perfect_aligned_pose = Pose(
    #     position=Point(x=0.65966, y=-0.03292, z=0.18824),
    #     orientation=Quaternion(x=0.0078043, y=0.99992, z=0.0073651, w=0.0066901),
    # )

    # TARE POSE AND VALUES - DO NOT CHANGE 
    tare_pose = Pose(
        position=Point(x=0.65914, y=-0.03378, z=0.3),
        orientation=Quaternion( x=0.0023562, y=-0.9999826, z=-0.0000127, w=-0.0054105  ),
    )
    ground_truth_tare_val = [1.083104, 0.499240, 0.235060, 0.012017, 0.249937] # Fx, Fy, Tx, Ty, Tz 

    # go to tare pose and record tare 
    target_pose = tare_pose  

    attempt_pose = get_offset_pose(target_pose, attempt_dict)

    experiment_executor.plan_and_execute(attempt_pose, linear=True)
    experiment_executor.plan_and_execute(target_pose, linear=True, force_wrench_record=True) 

    recorded_pose, _, _  = experiment_executor.data_record()  # IMAGE AND KUKA WRENCH NOT NEEDED
    # NOTE : We seperated ATI stuff
    _recorded_wrench_packet = experiment_executor.ft_sensor.get_most_recent_data()
    recorded_wrench = _recorded_wrench_packet.get_force_torque_array()
    tare_val = recorded_wrench 
    tare_val = np.delete(tare_val,2) # remove Fz 

    perfect_aligned_pose = Pose(
        position=Point(x=0.65901, y=-0.03529, z=0.18911),
        orientation=Quaternion( x=-0.0027891, y=0.9999525, z=-0.000323, w=0.0093383 ),
    )

    z_hole = perfect_aligned_pose.position.z 

    T_B_Fs = pose_to_transform(perfect_aligned_pose) 
    T_Ps_H = np.eye(4) 
    T_B_H_true = T_B_Fs @ T_F_P @ T_Ps_H
    T_H_B_true = invT(T_B_H_true) 
    pose_B_H_true = transform_to_xyzabc_array(T_B_H_true) 
    
    # make first contact 
    # initial_offset_dict = {
        # "X": random.uniform(-0.005,+0.005),
        # "Y": random.uniform(-0.005,+0.005),
        # "Z": 0.0,
        # "A": random.uniform(-10.0,+10.0),
        # "B": random.uniform(-10.0,+10.0),
        # "C": random.uniform(-10.0,+10.0),
    # }

    # make first contact 
    # initial_offset_dict = {
    #     "X": -0.003,
    #     "Y": 0.003,
    #     "Z": 0.0,
    #     "A": 0.0,
    #     "B": 4.0,
    #     "C": 2.0,
    # }
    # target_pose = get_offset_pose(perfect_aligned_pose, initial_offset_dict) 

    # initial_offsets = [+0.0012 , +0.002, 0.000, 
    #                    +0.3000, -0.200, 0.000]
    
    initial_offsets = [+0.0032, +0.005, 0.000, 
                       +3.3000, -2.200, 3.000]

    initial_offset_dict = {
        "X": initial_offsets[0],
        "Y": initial_offsets[1],
        "Z": initial_offsets[2],
        "A": initial_offsets[3],
        "B": initial_offsets[4],
        "C": initial_offsets[5],
    }
    T_Htrue_Hoff = xyzabc_array_to_transform(initial_offsets) 
    T_B_Ho = T_B_H_true @ T_Htrue_Hoff 
    T_B_Ft = T_B_Ho @ T_P_F 
    target_pose = transform_to_pose(T_B_Ft)  

    attempt_pose = get_offset_pose(target_pose, attempt_dict)

    experiment_executor.plan_and_execute(attempt_pose, linear=True)
    experiment_executor.force_wrench_recorder.start_filling()
    experiment_executor.plan_and_execute(target_pose, linear=True, force_wrench_record=True)

    camera_timestamp, force_wrench, flange_pose = experiment_executor.data_record(
            snap=True
        )
    
    # retract after recording observation 
    retraction_pose = get_offset_pose(flange_pose, short_retract)
    experiment_executor.plan_and_execute(retraction_pose, linear=True)    

    image_path = experiment_executor.rgb_folder + "/" + camera_timestamp + "_rgb.png"

    raw_observation_dict = package_single_observation(image_path, force_wrench, flange_pose, target_pose, tare_val, ground_truth_tare_val)  

    contact_flange_pose = flange_pose 

    # get first pose estimate 
    basic_dict = {
                    'device': "cpu",
                    # TODO : FILL MODEL PATHS HERE

                    'image_model_path': "/home/aero/aero_ws/src/data_collection/data_collection/podah_models/models/MODELS_INFERENCE_JUN6/ckpt_49_model.pt",
                    'fusion_compliance_model_path': "/home/aero/aero_ws/src/data_collection/data_collection/podah_models/models/FUSION_REAL.pt",
                    'wrench_compliance_model_path': "/home/aero/aero_ws/src/data_collection/data_collection/podah_models/models/CORL_{REAL}_{COMPLIANCE_WRENCH}_{DLA_FLA}_NETWORK.pt",
                    'DLA_model_path': "/home/aero/aero_ws/src/data_collection/data_collection/podah_models/models/DLAA.pt",
                    # 'REAL_DATA_path': "/home/aero/aero_ws/src/data_collection/data_collection/podah_models/models/CORL_{COMBINED}_{SIM_REAL_PAIR}_CROSS.csv",
                    'REAL_DATA_path': '/home/aero/aero_ws/src/data_collection/data_collection/podah_models/models/MODELS_INFERENCE_JUN6/CORL_SUBMISSION_DLA_DATASET.csv' ,
                    
                    'SIM_DATA_path': None, 
                    'KUKA': True
                 }
    
    insertion_finished = False

    print("basic_dict", basic_dict)
    trial_counter = 0 
    df_CPE = pd.DataFrame(columns=['trial',
                                   'X_true','Y_true','Z_true','A_true','B_true','C_true',
                                     'X_CPE','Y_CPE','Z_CPE','A_CPE','B_CPE','C_CPE',
                                     'X_err','Y_err','Z_err','A_err','B_err','C_err']) 
    df_FPE = pd.DataFrame(columns=['X_true','Y_true','Z_true','A_true','B_true','C_true',
                                     'X_FPE','Y_FPE','Z_FPE','A_FPE','B_FPE','C_FPE',
                                     'X_err','Y_err','Z_err','A_err','B_err','C_err']) 
    
    df_metadata = pd.DataFrame([initial_offset_dict])
    df_metadata = df_metadata.rename(columns={'X':'X0','Y':'Y0','Z':'Z0','A':'A0','B':'B0','C':'C0'})   
    df_metadata.to_csv(experiment_executor.csv_folder+'/metadata.csv', index=False) 
 
    print('\n\n')
    print('OFFSETS: ', initial_offsets) 
    print('\n\n')

    while not insertion_finished:
        
        model = podah(basic_dict)
        _waypoints_dict = model.get_target_waypoints(raw_observation_dict=raw_observation_dict) # call simulation 

        # NOTE(dhanush) : waypoints is a dictioanry and we want to access the key --> 'optimal_target_delta_actions_xyzabc'

        waypoints = _waypoints_dict['optimal_target_delta_actions_xyzabc']  

        # waypoints = np.zeros([6,6])

        trajectory_observations = []  # NOTE : Used to store the observation along the trajectory for FPE

        # EXECUTE INFORMATION GATHERING TRAJECTORY 
        for i, waypoint in enumerate(waypoints[1:]): 

            print('\n')
            print(f"Executing waypoint {i}: {waypoint}") 
            print('\n')

            target_pose = waypoint_to_target_pose(waypoint,contact_flange_pose,sensor_transform) 
            attempt_pose = get_offset_pose(target_pose, long_retract)
            experiment_executor.plan_and_execute(attempt_pose, linear=True)

            experiment_executor.plan_and_execute(
                target_pose, linear=True, force_wrench_record=True
            )
            camera_timestamp, force_wrench, flange_pose = (
                experiment_executor.data_record(snap=True)
            )

            # t_HP = get_peg_to_hole_transform(
            #     perfect_aligned_pose, flange_pose, sensor_transform
            # )
            T_B_Fc = pose_to_transform(flange_pose)
            T_H_Pc = T_H_B_true @ T_B_Fc @ T_F_P 
            pose_H_Pc = transform_to_xyzabc_array(T_H_Pc) 

            image_path = experiment_executor.rgb_folder + '/' + camera_timestamp + "_rgb.png"
            raw_observation_dict = package_single_observation(image_path, force_wrench, flange_pose, target_pose, tare_val, ground_truth_tare_val) 
            trajectory_observations.append(raw_observation_dict)

            if retraction:
                retraction_pose = get_offset_pose(flange_pose, short_retract)
                experiment_executor.plan_and_execute(retraction_pose, linear=True) 

        # RETRACT 
        retraction_pose = get_offset_pose(flange_pose, attempt_dict) 
        experiment_executor.plan_and_execute(retraction_pose, linear=True) 
        
        CPE_input_dict, contact_waypoints_dict = package_multiple_observations(trajectory_observations,sensor_transform_dict)


        # CHOOSE MODALITY FROM THIS : {image_model_prediction}, {wrench_compliance_model_prediction}, 
        # {fusion_compliance_model_prediction}
        modality = 'image_model_prediction'  # ----> { RESULTS TUNING }

        FPE_pose_prediction, CPE_pose_predictions = model.get_fine_pose_estimate_real(CPE_input_dict, 
                                                                  contact_waypoints_dict, modality) # perform inference
        # HOLE w.r.t KUKA BASE
        
        # # TODO : SAVE THIS NP array of shape (num observations, 6) - (HOLE|KUKA BASE)
        # CPE_pose_predictions

        _pose_estimate = FPE_pose_prediction
        # import pdb; pdb.set_trace()
        T_B_H_est = construct_transform_matrix(
            rotation=R.from_euler('xyz',[_pose_estimate[5],_pose_estimate[4],_pose_estimate[3]], degrees=True).as_matrix(),
            translation=np.array([_pose_estimate[0],_pose_estimate[1],_pose_estimate[2]]),
        )
        pose_B_H_est = transform_to_xyzabc_array(T_B_H_est) 


        T_B_Pt = T_B_H_est 
        T_B_Ft = T_B_Pt @ T_P_F 
        q_B_Ft = R.from_matrix(T_B_Ft[:3,:3]).as_quat() 

        T_Htrue_Hest_FPE = T_H_B_true @ T_B_H_est  
        pose_H_err_FPE = transform_to_xyzabc_array(T_Htrue_Hest_FPE) 

        print('\n\n\n')
        print('FINE POSE ESTIMATE ERROR') 
        print(pose_H_err_FPE)  
        print('\n\n\n')

        # SAVE DATA 
        initial_len_df_CPE = len(df_CPE)   
        for i in range(len(CPE_pose_predictions)): 
            T_B_H_est_CPE = xyzabc_array_to_transform(CPE_pose_predictions[i,:]) 
            T_Htrue_Hest_CPE = T_H_B_true @ T_B_H_est_CPE 
            pose_H_err_CPE = transform_to_xyzabc_array(T_Htrue_Hest_CPE) 
            df_CPE.loc[initial_len_df_CPE+i] = np.hstack((np.array([trial_counter]), pose_B_H_true, CPE_pose_predictions[i,:], pose_H_err_CPE))

        # import pdb; pdb.set_trace() 

        FPE_data = np.hstack((pose_B_H_true, pose_B_H_est, pose_H_err_FPE)) 
        df_FPE.loc[trial_counter] = FPE_data 
        df_FPE.to_csv(experiment_executor.csv_folder+'/FPE.csv', index=False) 
        df_CPE.to_csv(experiment_executor.csv_folder+'/CPE.csv', index=False)  

        target_pose = Pose(
            position=Point(x=T_B_Ft[0,3],y=T_B_Ft[1,3],z=T_B_Ft[2,3]),
            orientation=Quaternion(x=q_B_Ft[0],y=q_B_Ft[1],z=q_B_Ft[2],w=q_B_Ft[3]) 
        ) 
        
        # target_pose = get_offset_pose(pred_pose,inv_sensor_transform_dict)
        attempt_pose = get_offset_pose(target_pose, attempt_dict) 

        # print("CHECK TARGET POSE!") 
        # import pdb; pdb.set_trace() 

        # ATTEMPT INSERTION 
        experiment_executor.plan_and_execute(attempt_pose, linear=True)
        experiment_executor.plan_and_execute(target_pose, linear=True, force_wrench_record=True, scaling_factor=0.04)

        camera_timestamp, force_wrench, flange_pose = experiment_executor.data_record(
            snap=True
        )

        raw_observation_dict = package_single_observation(image_path, force_wrench, flange_pose, target_pose, tare_val, ground_truth_tare_val)  

        contact_flange_pose = flange_pose

        # check if success 
        if flange_pose.position.z <= z_hole - 0.010: 
            insertion_finished = True
            print('\n\n\n') 
            print('#------------ SUCCESSFUL INSERTION! ------------#')
        else: 
            # RETRACT 
            retraction_pose = get_offset_pose(flange_pose, short_retract) 
            experiment_executor.plan_and_execute(retraction_pose, linear=True)

        trial_counter = trial_counter + 1 

    experiment_executor.robot_interface_node.get_logger().info("Execution finished")
    experiment_executor.robot_interface_node.destroy_node()
    experiment_executor.camera_recorder_node.destroy_node()
    experiment_executor.force_wrench_recorder.stop_filling()
    et.join()

    rclpy.shutdown() 
