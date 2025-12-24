import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import os
import sys
import time
import threading
import cv2
import rclpy
import rclpy.executors

import pdb 

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

from data_collection.submodules.transformation_utils import (
    transform_to_pose,
    pose_to_transform,
    pose_to_xyzabc,
    xyzabc_to_pose,
    xyzabc_to_transform,
    transform_to_xyzabc,
    inverse_transform
) 

from data_collection.podah_models.podah.podah_pipeline import podah     
from scipy.spatial.transform import Rotation as R 
from PIL import Image

import random 

class PIHStateMachine:
    DATA_FOLDER = "/home/aero/PIH_SIM_data"

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
    
    def spin(self):
        self.executor.spin()
        
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
    
    def plan_and_execute_transform(
        self,
        target_transform: np.array,
        linear: bool,
        force_wrench_record: bool = False,
        scaling_factor: float = 0.08,
    ) -> bool:
        target_pose = transform_to_pose(target_transform)  
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
    
    def move_flange(self, xyzabc, linear=True, scaling_factor=0.08):
        # NOTE: offsets are applied in flange frame 
        flange_pose = self.robot_interface_node.get_fk()
        T_B_Fnow = pose_to_transform(flange_pose) # current pose  
        T_Fnow_Fnext = xyzabc_to_transform(xyzabc)
        T_B_Fnext = T_B_Fnow @ T_Fnow_Fnext  
        next_pose = transform_to_pose(T_B_Fnext) 
        self.plan_and_execute(next_pose, linear=linear, scaling_factor=scaling_factor)    
        return  
    
    def retract_up_to_height(self, target_height_mm): 
        flange_pose = self.robot_interface_node.get_fk() 
        current_height_mm = flange_pose.position.z * 1e3 
        retract_distance = (target_height_mm - current_height_mm) 
        if retract_distance > 0: 
            self.move_flange([0,0,-retract_distance,0,0,0]) 
        else: 
            print('Negative retract distance.')
        return 
    
    def get_wrench(self): 
        self.force_wrench_recorder.start_filling() 
        camera_timestamp, force_wrench, flange_pose = self.data_record(
            snap=True
        )
        return force_wrench
    
    def get_flange_pose(self): 
        self.force_wrench_recorder.start_filling() 
        camera_timestamp, force_wrench, flange_pose = self.data_record(
            snap=True
        )
        return flange_pose

def main(argv=None):
    # Initialize rclpy
    rclpy.init()
    
    # Create PIHStateMachine instance
    pih_state_machine = PIHStateMachine(int(sys.argv[1]))
    
    
    # Create and start the thread
    et = threading.Thread(target=pih_state_machine.spin)
    et.start()

    T_F_P = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0.0958],[0,0,0,1]])
    T_P_F = inverse_transform(T_F_P) 

    perfect_aligned_pose = Pose(
        position=Point(x=0.65901, y=-0.03529, z=0.18911),
        orientation=Quaternion( x=-0.0027891, y=0.9999525, z=-0.000323, w=0.0093383 ),
    ) 
    T_B_Fs = pose_to_transform(perfect_aligned_pose)
    T_B_H = T_B_Fs @ T_F_P 

    # TODO: organize repeated lines of code into function calls
    initial_offsets = [-5,5,0,0,-15,0] 
    T_H_Ha = xyzabc_to_transform(initial_offsets)  
    T_B_Fa = T_B_H @ T_H_Ha @ T_P_F 
    attempt_pose = transform_to_pose(T_B_Fa) 
    T_B_Fabove = T_B_Fa @ xyzabc_to_transform([0,0,-40,0,0,0]) 
    above_pose = transform_to_pose(T_B_Fabove)
    
    # retract to safe height 
    pih_state_machine.retract_up_to_height(target_height_mm = 250) 
    # move to above pose 
    pih_state_machine.plan_and_execute(above_pose, linear=True) 
    # record base wrench 
    wrench_ref = pih_state_machine.get_wrench() 
    # move to attempt pose 
    pih_state_machine.plan_and_execute(attempt_pose, linear=True) 
    time.sleep(1)
    # record contact wrench 
    wrench_cont = pih_state_machine.get_wrench() 
    wrench_diff = wrench_cont - wrench_ref   

    # TODO: implement method to estimate Z, B, C 
    # assume we now have a good estimate of Z, B, C 
    
    # slide until partial insertion 
    search_traj = np.array([[5,0,3,0,0,0],
                           [-10,2,3,0,0,0],
                           [+10,4,3,0,0,0],
                           [-10,6,3,0,0,0],
                           [+10,8,3,0,0,0],
                           [-10,10,3,0,0,0],
                           [+10,12,3,0,0,0]]) 
    for i in range(len(search_traj)): 
        if i == 0: 
            # record z contact 
            flange_pose = pih_state_machine.get_flange_pose()
            z_cont = flange_pose.position.z  
        pih_state_machine.move_flange(search_traj[i,:].reshape(6))  
        flange_pose = pih_state_machine.get_flange_pose()
        z_now = flange_pose.position.z 
        w_now = pih_state_machine.get_wrench() 
        if z_now < z_cont - 0.005: 
            print("Partial Insertion Detected!")
            break    
        if z_now > z_cont + 0.005: 
            print("Moving up for some reason?")


    # once partially inserted, tilt to align 
    # flange_pose_now = pih_state_machine.get_flange_pose()
    # while z_now > perfect_aligned_pose.position.z + 0.010: 

    #     # momentum 
    #     if z_now < z_prev - 0.002:
    #         pass # use same action  
    #     else:  
    #         x_next = random.uniform(-1,+1)  
    #         y_next = random.uniform(-1,+1)
    #         z_next = 3 
    #         a_next = random.uniform(-1,+1)
    #         b_next = random.uniform(-1,+1)
    #         c_next = random.uniform(-1,+1) 
    #     pih_state_machine.move_flange([x_next, y_next, z_next, a_next, b_next, c_next])

    #     flange_pose_prev = flange_pose_now 
    #     z_prev = flange_pose_prev.position.z

    #     flange_pose_now = pih_state_machine.get_flange_pose() 
    #     z_now = flange_pose_now.position.z 

    # tilt in direction of reducing torque  
    # angle_tilt = 15
    # k_tilt =  1.0 
    # pih_state_machine.move_flange([0,0,0,0,-k_tilt*wrench_diff[3],-k_tilt*wrench_diff[3]])   
    

    print('wrench_diff: ', wrench_diff) 

    # retract to safe height 
    # print("Retracting")
    # pih_state_machine.retract_up_to_height(target_height_mm = 250) 

    # pdb.set_trace() 

    # To join the thread (wait for it to finish) before exiting
    et.join()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
