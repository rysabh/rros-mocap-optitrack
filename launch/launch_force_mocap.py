#Launch file when in the data streaming pane the Up Axis option is set to "Y up"
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import datetime


def generate_launch_description():
    log_level = "warn"

    ld = LaunchDescription()
    # Create the NatNet client node
    config = os.path.join(
        get_package_share_directory('mocap_optitrack_client'),
        'config',
        'natnetclient.yaml'
    )
    natnet_client = Node(
        package='mocap_optitrack_client',
        executable='mocap_optitrack_client',
        name='natnet_client',
        parameters = [config],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    
    
    # Create the world to base client
    config = os.path.join(
        get_package_share_directory('mocap_optitrack_w2b'),
        'config',
        'world_to_base_y_up.yaml'
    )
    world_to_base = Node(
        package='mocap_optitrack_w2b',
        executable='mocap_optitrack_w2b',
        name='world_to_base',
        parameters = [config],
        arguments=['--ros-args', '--log-level', log_level]
    )
    # Create the inverse kinematics node
    config = os.path.join(
        get_package_share_directory('mocap_optitrack_inv_kin'),
        'config',
        'inverse_kinematics.yaml'
    )
    inverse_kinematics = Node(
        package='mocap_optitrack_inv_kin',
        executable='mocap_optitrack_inv_kin',
        name='inverse_kinematics',
        parameters = [config],
        arguments=['--ros-args', '--log-level', log_level]
    )

        # Get the current time and format it
    # current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # read a text file to get the current take number and increment it
    take_dir = "takes"
    take_number_file = os.path.join(take_dir, "take_number.txt")

    with open(take_number_file, "r") as f:
        take_number = int(f.readline().strip())
    
    with open(take_number_file, "w") as f:
        f.write(str(take_number+1))
    
    
    format_take_number = f"{take_number:03}"
    print(f"\n\n======================\n\nTake number: {format_take_number}\n\n======================\n\n")
    
    ft_data_file = os.path.join(take_dir, f"ft_{format_take_number}.csv")

    sensor_parameters = [
        {'sensor_ip': '192.168.10.100'},  # Replace with your sensor IP
        {'output_file': ft_data_file},  # Dynamically generated file name
        {'sample_rate': '240'}  # Replace with your desired rate
    ]

    ati_sensor_node = Node(
            package='data_collection',
            executable='ati_wrench_publisher',
            name='ati_wrench_publisher',
            output='screen',
            parameters=sensor_parameters
        )


    # Add the nodes to the launch description and return it
    ld.add_action(natnet_client)
    # ld.add_action(world_to_base)
    # ld.add_action(inverse_kinematics)
    # ld.add_action(ati_sensor_node)
    return ld