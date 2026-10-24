# 🌐 S.P.H.E.R.E.
> Search & Positioning Hazard Engine for Rescue Exploration

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Simulator-Gazebo_Harmonic-orange.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Sphere-Scout is an AI-powered transforming spherical scout robot that rolls through collapsed urban rubble and localizes trapped survivors in real time. This repository is the ROS 2 Jazzy + Gazebo Harmonic simulation, plus the ESP32-S3 firmware for the physical robot: the sphere drives as a two-shell differential drive, stops and opens its shells to expose a 32 × 24 thermal camera, rotates in place to scan, and triangulates warm targets from multiple viewpoints. Built for Hackachusetts 2026.

> **Status: skeleton.** Every package, file, and interface exists, but the files are stubs with the spec written as `TODO`s. The team implements them during the event. The run commands below will work once the stubs are filled in.

## Setup check

Ubuntu 24.04 (WSL2), ROS 2 Jazzy at `/opt/ros/jazzy`, Gazebo Harmonic via `ros-jazzy-ros-gz`. Do not install another ROS or Gazebo version.

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep -xE 'ros_gz_sim|ros_gz_bridge|robot_localization|xacro|vision_msgs|cv_bridge|teleop_twist_keyboard'
gz sim --versions   # expect 8.x (Harmonic)
```

## Build

The repo root is the colcon workspace.

```bash
colcon build --symlink-install
source install/setup.bash
```

The firmware (`firmware/sphere_scout_firmware`, `COLCON_IGNORE`d) is built separately with PlatformIO.

## Run

```bash
# 1. Simulation: Gazebo, robot, bridge, EKF, RViz (add headless:=true rviz:=false on WSL without a GPU)
ros2 launch sphere_scout_bringup gazebo_world.launch.py

# 2. Perception
ros2 launch sphere_scout_ai perception.launch.py

# 2b. Operator side: target_localizer_node, thermal_viewer_node, RViz
ros2 launch sphere_scout_bringup operator.launch.py   # rviz:=false to skip RViz

# 2c. 3D operator view in a browser (building, survivor estimate, movable thermal-feed window),
#     external tool, see tools/xr_viewer/bridge.py --help. Needs thermal_viewer_node (2b) for the feed.
python3 tools/xr_viewer/bridge.py        # then open http://localhost:8765

# 3. Motor state machine
ros2 run sphere_scout_control motor_state_machine

# 4. Teleop (into the state machine, not straight to the robot)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop

# 5. Trigger a scan
ros2 service call /mission/scan std_srvs/srv/Trigger
```

## Topics

| Topic | Type | Publisher | Subscriber(s) |
|---|---|---|---|
| `/clock` | rosgraph_msgs/msg/Clock | Gazebo (bridge) | all nodes |
| `/robot_description` | std_msgs/msg/String | robot_state_publisher | Gazebo spawn, RViz |
| `/cmd_vel_teleop` | geometry_msgs/msg/Twist | teleop_twist_keyboard | motor_state_machine |
| `/cmd_vel` | geometry_msgs/msg/Twist | motor_state_machine | Gazebo DiffDrive (bridge) / firmware |
| `/shell/cmd` | std_msgs/msg/Float64 | motor_state_machine | Gazebo JointPositionController (bridge, gz `/shell_cmd`) / firmware |
| `/joint_states` | sensor_msgs/msg/JointState | Gazebo JointStatePublisher (bridge) | robot_state_publisher, motor_state_machine |
| `/odom` | nav_msgs/msg/Odometry | Gazebo DiffDrive (bridge) | ekf_node |
| `/imu/data` | sensor_msgs/msg/Imu | Gazebo IMU (bridge) / firmware | ekf_node |
| `/odometry/filtered` | nav_msgs/msg/Odometry | ekf_node (also TF odom → base_link) | RViz |
| `/thermal/image_raw` | sensor_msgs/msg/Image (16-bit, K × 100) | Gazebo thermal camera (bridge) / firmware | thermal_detector_node, thermal_viewer_node |
| `/detected_target` | vision_msgs/msg/Detection2DArray | thermal_detector_node | target_localizer_node, thermal_viewer_node |
| `/survivor/position` | geometry_msgs/msg/PointStamped (frame `odom`) | target_localizer_node | — |
| `/survivor/markers` | visualization_msgs/msg/MarkerArray | target_localizer_node | RViz |
| `/thermal/view` | sensor_msgs/msg/Image (rgb8, reliable, depth 1) | thermal_viewer_node | RViz |
| `/mission/state` | std_msgs/msg/String | motor_state_machine | — |
| `/mission/scan` | std_srvs/srv/Trigger (service) | motor_state_machine (server) | operator |
| `/target_localizer_node/reset` | std_srvs/srv/Trigger (service) | target_localizer_node (server) | operator |

`/thermal/view` and `/target_localizer_node/reset` are new. The localizer is a continuous passive estimator (no coupling to `/mission/state`, `/cmd_vel`, `/odom`); `/survivor/position` carries triangulated points only. Parameters: `src/sphere_scout_bringup/config/target_localizer.yaml` and `thermal_viewer.yaml` (they replace the stub's `width`, `height`, `min_ray_angle_deg`, `ground_truth_file`). `pixel_convention` must match across detector, localizer and viewer.

Only the EKF publishes the `odom → base_link` TF; DiffDrive's TF is not bridged.

## Package ownership

| Package | Contents | Owner |
|---|---|---|
| `sphere_scout_description` | URDF/xacro, `params.xacro`, meshes, RViz config | TODO(kickoff) |
| `sphere_scout_gazebo` | `apartment.sdf` world, `apartment` model (built from `Building/` with `tools/export_apartment.py`), ground truth | TODO(kickoff) |
| `sphere_scout_bringup` | `gazebo_world.launch.py`, `operator.launch.py` (localizer + viewer + RViz), bridge, EKF, localizer and viewer config | TODO(kickoff) |
| `sphere_scout_control` | `motor_state_machine` | TODO(kickoff) |
| `sphere_scout_ai` | `thermal_detector_node`, `target_localizer_node`, `thermal_viewer_node`, `perception.launch.py` | TODO(kickoff) |
| `firmware/sphere_scout_firmware` | ESP32-S3 micro-ROS firmware (PlatformIO) | TODO(kickoff) |

## `TODO(kickoff)` placeholders

Values to decide at kickoff. No physical value is invented in this repo.

| Placeholder | File |
|---|---|
| `shell_radius`, `rolling_gap`, `shell_extension_max` | `src/sphere_scout_description/urdf/params.xacro` |
| `core_mass`, `shell_mass`, `core_com_offset_z` | `src/sphere_scout_description/urdf/params.xacro` |
| thermal and IMU mount poses | `src/sphere_scout_description/urdf/params.xacro` |
| IMU noise stddevs (angular velocity, linear acceleration) | `src/sphere_scout_description/urdf/sphere.urdf.xacro` |
| survivor stand-in pose and size (static box, visual with the Thermal system at 310 K; not in the world yet) | `src/sphere_scout_gazebo/worlds/apartment.sdf` |
| survivor stand-in pose (must match the world) | `src/sphere_scout_gazebo/config/ground_truth.yaml` |
| `target_height_m`, `target_height_sigma_m` (enable the one-view PROVISIONAL estimate), `ground_truth_world`, `spawn_pose` | `src/sphere_scout_bringup/config/target_localizer.yaml` (commented out / default until Mech 2 gives values) |
| RViz layout `sphere_scout.rviz` (save from the RViz2 GUI: fixed frame `odom`, Image `/thermal/view` Reliable, MarkerArray `/survivor/markers`, TF) | `src/sphere_scout_description/rviz/` |
| spawn entry pose defaults | `src/sphere_scout_bringup/launch/gazebo_world.launch.py` |
| `rolling_gap_cmd`, `expanded_cmd`, `scan_step_deg`, `scan_steps`, `scan_hold_s`, `scan_yaw_rate` defaults | `src/sphere_scout_control/sphere_scout_control/motor_state_machine.py` |
| `t_low_c`, `t_high_c`, `min_pixels` defaults | `src/sphere_scout_ai/sphere_scout_ai/thermal_detector_node.py` |
| MLX90640, MPU-6050, DRV8833 pin numbers | `firmware/sphere_scout_firmware/src/main.cpp` |
| package owners and maintainers | this README, `AGENTS.md`, every `package.xml` / `setup.py`, file headers |

Also unresolved: the ESP32-S3 Mini board ID in `firmware/sphere_scout_firmware/platformio.ini` (`TODO: confirm ESP32-S3 Mini board ID`).
