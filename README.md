# rros-mocap-optitrack
This repository contains ROS 2 packages for OptiTrack (NatNet) motion capture and ATI NetFT force/torque sensing. It is designed to live inside a colcon workspace and to be launched alongside other sensor packages (for example RealSense) and a recorder.

## Package map
- `mocap_optitrack_client` (C++): NatNet client and ROS node that publishes `MotionCaptureData` and serves `get_mocap_data`.
- `ati_sensor_service` (Python): ATI NetFT driver node that publishes `ForceTorque` and `WrenchStamped` and serves `get_force_torque`.
- `data_collection` (Python): recorder and export tools. The recorder subscribes to MoCap, ATI, and camera topics and time-aligns them into a take directory.
- `mocap_service` (deprecated): old subscriber-to-service wrapper. Keep it only if you still depend on it.

## Naming: package, executable, node
In ROS 2 these are different concepts.
- The package name comes from `package.xml` and `project()` in CMake. It is the first argument to `ros2 run`.
- The executable name is the program that gets built or installed. In C++ it comes from `add_executable` in CMake. In Python it comes from `console_scripts` in `setup.py`.
- The node name is the runtime name you set in code with `rclcpp::Node(...)` or `rclpy.Node(...)`.

Example in this repo:
- Package: `mocap_optitrack_client` (CMake `project` name).
- Executable: `mocap_optitrack_client` (CMake `add_executable`).
- Node name: `natnet_client` (set in `MoCapPublisher.cpp`).

Example for ATI:
- Package: `ati_sensor_service` (`package.xml` and `setup.py`).
- Executable: `ati_service` (`console_scripts` entry).
- Node name: `ati_sensor` (set in `ati_service.py`).

## Why ATI has two files (service plus socket)
The code is split into hardware I/O and ROS wrapper layers.
- `ati_sensor_service/submodules/ati_sensor_socket.py` handles UDP packets from the ATI device.
- `ati_sensor_service/ati_service.py` is the ROS node that publishes topics and provides the service.

The MoCap side is split the same way, just across C++ files:
- `MoCapNatNetClient.cpp` handles NatNet networking.
- `MoCapPublisher.cpp` is the ROS node.

## Topics and services
MoCap:
- Topic: `mocap_Data` (type `mocap_optitrack_interfaces/msg/MotionCaptureData`).
- Service: `get_mocap_data` (type `mocap_optitrack_interfaces/srv/GetMotionCaptureData`).

ATI:
- Topic: `force_torque` (type `ati_sensor_interfaces/msg/ForceTorque`).
- Topic: `wrench` (type `geometry_msgs/msg/WrenchStamped`).
- Service: `get_force_torque` (type `ati_sensor_interfaces/srv/GetForceTorque`).

Recorder:
- Subscribes to `mocap_Data`, `wrench`, and per-camera image and camera_info topics.
- Services: `set_recording` (`std_srvs/SetBool`) and `capture_sample` (`std_srvs/Trigger`).

## Time synchronization model
Each sensor publishes messages with timestamps in their headers. The recorder buffers recent messages from each topic and, on each sampling tick, selects the closest message within `max_sync_slop_ms`. This produces a synchronous snapshot without needing a service for each sensor.

## Configuration and parameters
MoCap uses a YAML file installed with the package:
- `mocap_optitrack_client/config/natnetclient.yaml`
This file sets `server_address`, ports, `pub_topic`, and `record` and `take_name` defaults.

ATI parameters are declared in `ati_service.py`:
- `sensor_ip` (default `192.168.10.100`)
- `sample_rate` (default `240.0`)
- `frame_id` (default `ati_sensor`)

Recorder parameters are declared in `multi_sensor_recorder.py`:
- `output_dir`, `take_name`, `record_hz`, `record_enabled`
- `max_sync_slop_ms`, `mocap_topic`, `wrench_topic`, `camera_names`

If you use the full stack launch file, the single source of truth is `service_launch/config/sensors.yaml` in the workspace root. That file enables or disables sensors and sets the RealSense serial numbers.

## Configuration of NatNet server
Please read the OptiTrack Streaming Guide first:
https://v22.wiki.optitrack.com/index.php?title=Data_Streaming

Make sure that a static IP is used on the OptiTrack (Windows) workstation for the LAN connection with the OptiTrack switch. Windows Defender needs to be deactivated for all networks as well. Nominally, the static IP should be set to `192.168.4.31`, which is the same local network as the VTEM terminal.

In the streaming pane of Motive, switch on "Broadcast Frame Data" and select the local interface `192.168.4.31`. As transmission type select "Multicast".

On your target machine (usually the Ubuntu lab workstation), set a static IP to the local network as well (for example `192.168.4.20`) and try to ping the source workstation with `ping 192.168.4.31`. The IP address of the source also needs to be set in `mocap_optitrack_client/config/natnetclient.yaml`.

## Building and running
Build the relevant packages:
```bash
colcon build --packages-select mocap_optitrack_client ati_sensor_service data_collection service_launch
```
This compiles the C++ node and installs Python entry points so ROS 2 can find them.

Source the ROS 2 environment and your workspace overlay:
```bash
source /opt/ros/<ros_distro>/setup.bash
source /home/cam/Downloads/GitHub/multi_iiwa_ws/install/setup.bash
```
The first command gives you core ROS 2 tools. The second command registers the packages built in this workspace.

Run the full sensing stack (MoCap, ATI, RealSense, recorder):
```bash
ros2 launch service_launch record_take.launch.py
```
This reads `service_launch/config/sensors.yaml` and launches every enabled sensor.

### Run each node individually
MoCap only:
```bash
ros2 run mocap_optitrack_client mocap_optitrack_client \
  --ros-args \
  --params-file $(ros2 pkg prefix mocap_optitrack_client)/share/mocap_optitrack_client/config/natnetclient.yaml \
  -p record:=false -p take_name:=test
```
This starts the NatNet client with its YAML defaults and overrides `record` and `take_name` for safety.

ATI only:
```bash
ros2 run ati_sensor_service ati_service \
  --ros-args -p sensor_ip:=192.168.10.100 -p sample_rate:=240.0 -p frame_id:=ati_sensor
```
This starts the ATI driver and begins publishing force and wrench topics.

Recorder only (requires the sensor topics to exist):
```bash
ros2 run data_collection multi_sensor_recorder \
  --ros-args -p output_dir:=/tmp/take -p take_name:=test -p record_hz:=30.0
```
For multiple cameras, it is easier to pass a YAML parameters file with `camera_names` as a list.

### Quick sanity checks
```bash
ros2 node list
ros2 topic list
ros2 topic echo /mocap_Data --once
ros2 topic hz /wrench
ros2 service list
```
These commands show which nodes are alive, which topics exist, and whether data is streaming.

## Do you still need ati_client
No, not for recording or synchronization. The recorder subscribes to topics directly. The `ati_client` tool is only useful if you want a one-shot request to the `get_force_torque` service during debugging or scripting. If you never use service calls, you can remove it.
