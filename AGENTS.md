# AGENTS.md: Sphere-Scout project context for AI tools

## Concept

Sphere-Scout is a spherical search-and-rescue robot whose two hemispherical shells act as the wheels of a differential drive. To search, it stops, slides the shells apart on prismatic joints to expose a 32 × 24 thermal camera, and rotates in place to scan. Warm detections from several viewpoints are triangulated into a survivor position in the `odom` frame and shown in RViz against ground truth.

## Fixed stack (do not change)

- Ubuntu 24.04 on WSL2, ROS 2 Jazzy at `/opt/ros/jazzy`, Gazebo Harmonic via `ros-jazzy-ros-gz`.
- robot_localization (`ekf_node`), xacro, teleop_twist_keyboard, python3-opencv, cv_bridge, vision_msgs.
- Firmware: ESP32-S3, PlatformIO, Arduino framework, micro-ROS (Jazzy, Wi-Fi transport).
- Do not install another ROS or Gazebo version.

## Hard rules

- Gazebo Harmonic names only: `gz-sim-*-system` filenames, `gz::sim::systems::*` class names. No Gazebo Classic (`gazebo_ros_*`) or Fortress (`ignition-*`) names anywhere.
- No ros2_control.
- Every node and launch file sets `use_sim_time: true`.
- Python (rclpy) for all ROS nodes.
- Standard message types only. No custom interface package.
- Do not invent physical values. Every robot dimension lives in `src/sphere_scout_description/urdf/params.xacro`; world values live in `apartment.sdf` / `ground_truth.yaml`. Placeholders are marked `TODO(kickoff)` and listed in `README.md`.
- Only the EKF publishes `odom → base_link`. Do not bridge DiffDrive's TF.
- Don't make design decisions beyond the spec in the file docstrings/headers. If something is ambiguous, ask.

## Layout

The repo root is the colcon workspace: `src/` holds the five ROS packages, `firmware/sphere_scout_firmware` is PlatformIO (`COLCON_IGNORE`d). Build with `colcon build --symlink-install`. `tools/xr_viewer` (`COLCON_IGNORE`d) is an external, read-only 3D operator view: `bridge.py` (rclpy + stdlib) forwards `/survivor/markers`, `/thermal/view` (as JPEG, only while a browser is connected) and the `odom → base_link` TF to a browser app (vendored three.js) with a movable thermal-feed window. It adds no topic, type or frame.

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
| `/imu/data` | sensor_msgs/msg/Imu | Gazebo IMU (bridge) / firmware | ekf_node, motor_state_machine (swing damping) |
| `/odometry/filtered` | nav_msgs/msg/Odometry | ekf_node (also TF odom → base_link) | RViz |
| `/thermal/image_raw` | sensor_msgs/msg/Image (16-bit, K × 100) | Gazebo thermal camera (bridge) / firmware | thermal_celsius_node, thermal_detector_node, thermal_viewer_node |
| `/thermal/celsius` | sensor_msgs/msg/Image (32FC1, °C per pixel, 32 × 24) | thermal_celsius_node | human detection, print_thermal |
| `/thermal/image_color` | sensor_msgs/msg/Image (rgb8, fixed 5–40 °C colour scale, 320 × 268) | thermal_celsius_node | Gazebo thermal panel (bridge, ROS → Gazebo), RViz |
| `/camera/image_raw` | sensor_msgs/msg/Image (rgb8, 320 × 240, OV2640) | Gazebo RGB camera (bridge) | camera_jpeg republisher, RViz |
| `/camera/image_raw/compressed` | sensor_msgs/msg/CompressedImage (JPEG, quality 80, ~7 kB) | camera_jpeg (sim) / firmware (OV2640 JPEG) | XR headset |
| `/detected_target` | vision_msgs/msg/Detection2DArray | thermal_detector_node | target_localizer_node, thermal_viewer_node |
| `/survivor/position` | geometry_msgs/msg/PointStamped (frame `odom`) | target_localizer_node | — |
| `/survivor/markers` | visualization_msgs/msg/MarkerArray | target_localizer_node | RViz |
| `/thermal/view` | sensor_msgs/msg/Image (rgb8, reliable, depth 1) | thermal_viewer_node | RViz |
| `/mission/state` | std_msgs/msg/String | motor_state_machine | — |
| `/mission/scan` | std_srvs/srv/Trigger (service) | motor_state_machine (server) | operator |
| `/target_localizer_node/reset` | std_srvs/srv/Trigger (service) | target_localizer_node (server) | operator |

`/thermal/view` (rgb8 render for the driver) and the `~/reset` service are new. `target_localizer_node` is a continuous passive estimator: it does not subscribe to `/mission/state`, `/cmd_vel` or `/odom`, and `/survivor/position` carries triangulated points only. Its parameters are in `src/sphere_scout_bringup/config/target_localizer.yaml` (replacing the stub's `width`, `height`, `min_ray_angle_deg`, `ground_truth_file`); the viewer's are in `thermal_viewer.yaml`. `pixel_convention` must be the same in the detector, localizer and viewer. `target_height_m`, `target_height_sigma_m` and `ground_truth_world` stay unset until Mech 2 provides them.

Gazebo-side topic names differ only for `/shell/cmd` (gz `/shell_cmd`). The thermal image encoding is 16-bit kelvin × 100 (`temp_c = pixel * 0.01 - 273.15`) in both sim and firmware.

## Package ownership

| Package | Contents | Owner |
|---|---|---|
| `sphere_scout_description` | URDF/xacro, `params.xacro`, meshes, RViz config | TODO(kickoff) |
| `sphere_scout_gazebo` | `apartment.sdf` world, `apartment` model (built from `Building/` with `tools/export_apartment.py`), ground truth | TODO(kickoff) |
| `sphere_scout_bringup` | `gazebo_world.launch.py`, `operator.launch.py` (localizer + viewer + RViz), bridge, EKF, localizer and viewer config | TODO(kickoff) |
| `sphere_scout_control` | `motor_state_machine` | TODO(kickoff) |
| `sphere_scout_ai` | `thermal_detector_node`, `target_localizer_node`, `thermal_viewer_node`, `perception.launch.py` | TODO(kickoff) |
| `firmware/sphere_scout_firmware` | ESP32-S3 micro-ROS firmware (PlatformIO) | TODO(kickoff) |

## Interface changes

**Flag any interface change.** Topic names, message types, frame names, parameter names, the bridge table, and the thermal encoding are shared across packages and with the firmware. If a change touches any of them, say so explicitly and update this file and `README.md` in the same change.
