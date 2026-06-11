# Gento ROS Packages

ROS 2 packages for Marvin/Gento robot descriptions, custom interfaces, and the
Gento SDK-backed robot control node.

## Packages

| Path | ROS package | Purpose |
| --- | --- | --- |
| `marvin_description` | `marvin_description` | URDF/Xacro, meshes, RViz config, MuJoCo XML files, and display launch files. |
| `marvin_msgs` | `marvin_msgs` | Custom messages, services, and actions used by the control stack. |
| `marvin_ros2_control` | `marvin_ros_control` | C++ robot control node, robot parameter YAML files, launch files, and a jog GUI script. |

> Note: the directory is named `marvin_ros2_control`, but the ROS package name is
> `marvin_ros_control`. Use `marvin_ros_control` in `colcon` and `ros2` commands.

## Requirements

- ROS 2 with `colcon`
- Standard ROS dependencies used by these packages, including:
  - `rclcpp`, `rclpy`
  - `std_msgs`, `std_srvs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`
  - `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`
  - `urdf_launch`
- Pinocchio and hpp-fcl development packages
- Gento SDK installed on the target machine:
  - `libGentoSDK.so`
  - headers under `include/gentosdk`, including `L1Robot/L1Robot.h`

The control package searches for the Gento SDK in this order:

1. `GENTO_SDK_ROOT` environment variable
2. `/usr/local/lib/libGentoSDK.so`
3. CMake variables passed at build time:
   - `GENTO_SDK_ROOT`
   - `GENTO_SDK_LIB`
   - `GENTO_SDK_INCLUDE_ROOT`

## Build

Clone this repository into a ROS 2 workspace `src` directory, then build from
the workspace root:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If the Gento SDK is not installed under `/usr/local`, pass its prefix:

```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DGENTO_SDK_ROOT=/opt/gentosdk
source install/setup.bash
```

To build only the interfaces and robot descriptions:

```bash
colcon build --symlink-install --packages-select marvin_msgs marvin_description
```

To build only the control package after its dependencies are available:

```bash
colcon build --symlink-install --packages-select marvin_ros_control
```

## Robot Description Launches

Display a robot model with RViz and the joint state publisher GUI:

```bash
ros2 launch marvin_description displayMarvinUltra.launch.py
ros2 launch marvin_description displayMarvinPro.launch.py
ros2 launch marvin_description displayGentoLuna.launch.py
ros2 launch marvin_description displayGentoSky.launch.py
```

Publish `robot_description` without the joint state publisher GUI:

```bash
ros2 launch marvin_description description_only_MarvinUltra.launch.py
ros2 launch marvin_description description_only_MarvinPro.launch.py
ros2 launch marvin_description description_only_GentoLuna.launch.py
ros2 launch marvin_description description_only_GentoSky.launch.py
```

Each `description_only_*` launch accepts:

- `model`: path to the URDF/Xacro model
- `rvizconfig`: path to an RViz config
- `use_rviz`: whether to start RViz

## Control Launches

Bring up the robot control node and matching description:

```bash
ros2 launch marvin_ros_control bringup_control_marvin_ultra.launch.py
ros2 launch marvin_ros_control bringup_control_marvin_max.launch.py
ros2 launch marvin_ros_control bringup_control_gento_luna.launch.py
ros2 launch marvin_ros_control bringup_control_gento_sky.launch.py
```

The control launch files load robot parameters from
`marvin_ros2_control/config/robot_param_*.yaml` and start
`marvin_robot_node`.

Launch the jog GUI remapped to `/control/jog_cmd`:

```bash
ros2 launch marvin_ros_control jog.launch.py
```

The repository also contains `bringup_control_m3.launch.py` and
`bringup_control_m6.launch.py`, but they currently reference description launch
files that are not present in `marvin_description/launch`.

## CAN Setup

The control code uses `vcan0` and `vcan1` interfaces. To install and start the
provided systemd service:

```bash
cd ~/ros2_ws/src/gento_ros_packages/marvin_ros2_control
sudo bash install_van.sh
ip link show vcan0
ip link show vcan1
```

## Main ROS Interfaces

`marvin_robot_node` publishes:

- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/info/joint_feedback` (`marvin_msgs/msg/Jointfeedback`)
- `/info/cmd_pos_A` (`marvin_msgs/msg/JointcmdArm`)
- `/info/cmd_pos_B` (`marvin_msgs/msg/JointcmdArm`)
- `/info/cmd_pos_body` (`marvin_msgs/msg/JointcmdBody`)
- `/info/cmd_pos_head` (`marvin_msgs/msg/JointcmdHead`)
- `/info/robot_state` (`std_msgs/msg/Int16MultiArray`)
- `/info/robot_cmd_state` (`std_msgs/msg/Int16MultiArray`)
- `/info/robot_info` (`marvin_msgs/msg/RobotInfo`, configurable)

It subscribes to:

- `/control/joint_cmd_A` (`marvin_msgs/msg/JointcmdArm`)
- `/control/joint_cmd_B` (`marvin_msgs/msg/JointcmdArm`)
- `/control/joint_cmd_body` (`marvin_msgs/msg/JointcmdBody`)
- `/control/joint_cmd_head` (`marvin_msgs/msg/JointcmdHead`)

It provides:

- `/control/set_mode` (`marvin_msgs/srv/Int`)
- `/control/clear_fault` (`std_srvs/srv/Trigger`)
- `/control/set_ready` (`std_srvs/srv/Trigger`)

## Repository Notes

- Meshes and generated robot model files are stored under
  `marvin_description/meshes`, `marvin_description/urdf`, and
  `marvin_description/mjcf`.
- Custom interfaces are generated from `marvin_msgs/msg`, `marvin_msgs/srv`,
  and `marvin_msgs/action`.
- Robot-specific control parameters are stored in
  `marvin_ros2_control/config`.
