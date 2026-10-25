# 🌐 S.P.H.E.R.E.
> Search & Positioning Hazard Engine for Rescue Exploration

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Simulator-Gazebo_Harmonic-orange.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Sphere-Scout is an AI-powered, transforming spherical scout robot that rolls through collapsed urban rubble, finds trapped survivors by their body heat, and works out where they are. The two hemispherical shells are the wheels of a differential drive. To search, the robot stops, slides the shells apart to expose a 32 × 24 thermal camera, and rotates in place. A neural network spots people in the thermal image, and detections from several viewpoints are triangulated into a survivor position with a stated uncertainty.

This repository is the complete system in simulation: a physically modelled robot in a collapsed-building scene, the sensors, the AI detector, the position estimator, and two operator views.

## What it does

- **A full simulation.** A 9 cm, 141 g sphere built from the team's CAD, with a thermal camera (MLX90640 specification, 55° field of view, 8 Hz), an RGB camera with a headlight for the dark, and an IMU, driving through a night-time collapsed apartment in Gazebo Harmonic. Rescue Randy lies trapped in a rubble void with a 37 °C heat signature, visible only from the right place. A wheel-and-IMU EKF provides odometry. A state machine runs the stop, expand, 360° scan and retract cycle, with pendulum-swing damping so the core settles quickly after each move.
- **AI human detection.** Every thermal frame goes to a CNN (419,841 parameters) that decides human or no human and draws a box around the warm region. On its own held-out thermal dataset the model reports about 90 % human recall and 75.5 % balanced accuracy. Its decisions are published as standard `vision_msgs`, one message per frame.
- **Multi-view triangulation.** Each detection becomes a ray in the world frame using the camera pose at the exact image timestamp. Rays are grouped into viewpoints and intersected by weighted least squares, with RANSAC to reject bad views. The result is a position, a 2σ uncertainty ellipsoid, and a plain status: `BEARING_ONLY`, `LOW_CONFIDENCE` or `CONFIRMED`. In the no-Gazebo test fixture, a sideways drive converges to within about 4 cm of the true target with a 0.13 m depth sigma after 8 s.
- **Two ways to watch.** A browser 3D view shows the translucent building, the robot's trail, the estimate with its uncertainty and the rays behind it, and a movable live thermal window. A WebXR mode puts you inside the scene in first person on a headset. RViz shows the same data for ROS users.
- **One command to run it all.** `tools/run_all.sh` starts the model service, Gazebo, the controller, perception, the estimator, the viewers and keyboard teleop in one terminal, and stops everything it started on Ctrl+C.
- **Built to move to hardware.** The simulation publishes exactly the topics, message types and thermal encoding (16-bit, kelvin × 100) planned for the ESP32-S3 micro-ROS firmware, so the same nodes run unchanged on the real robot.

```
 Gazebo / ESP32-S3
   thermal camera 32×24 ──► /thermal/image_raw ──┬─► thermal_detector_node ──HTTP──► CNN model service
   IMU, wheel odometry ──► EKF ──► odom→base_link │              │
                                                  │              ▼
                                                  │        /detected_target
                                                  │              │
                                                  ├─► thermal_viewer_node ──► /thermal/view
                                                  ▼              ▼
                                        target_localizer_node (rays at image stamp + TF)
                                                  │
                                    /survivor/position, /survivor/markers
                                                  │
                                    RViz  ·  browser 3D view  ·  WebXR headset view
```

## Quick start

Ubuntu 24.04 (WSL2), ROS 2 Jazzy at `/opt/ros/jazzy`, Gazebo Harmonic via `ros-jazzy-ros-gz`. Do not install another ROS or Gazebo version.

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep -xE 'ros_gz_sim|ros_gz_bridge|robot_localization|xacro|vision_msgs|cv_bridge|teleop_twist_keyboard'
gz sim --versions   # expect 8.x (Harmonic)

colcon build --symlink-install     # the repo root is the colcon workspace
source install/setup.bash
```

The detection model is a separate service, set up once:

```bash
git clone https://github.com/andrew2719/thermal_rescue_ai_model external/thermal_rescue_ai_model
touch external/COLCON_IGNORE
python3 -m venv external/venv
external/venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
external/venv/bin/pip install -r external/thermal_rescue_ai_model/requirements.txt
```

Its trained checkpoint ships in the clone, and the versions used here are in `external/requirements-lock.txt`. The first Gazebo launch downloads Rescue Randy from Gazebo Fuel (about 12 MB), so it needs internet once.

Then run everything:

```bash
tools/run_all.sh                 # --headless: no Gazebo GUI, --rviz: also RViz, --no-bridge, --no-teleop
```

Open http://localhost:8765 for the browser view. Drive with the keyboard in that terminal, and trigger a scan from another one:

```bash
ros2 service call /mission/scan std_srvs/srv/Trigger
```

Logs are in `log/run_all/`. Ctrl+C stops what the script started, and only that. Every service it starts listens on `127.0.0.1` and has no authentication.

## Run the pieces separately

```bash
# 1. Simulation: Gazebo, robot, bridge, EKF (headless:=true for no GUI; rviz:=true once
#    mission_control.rviz is saved). model:=placeholder_cube.urdf.xacro gives a simple cube
#    robot, and world:=climb_test.sdf a course of ramps and steps.
ros2 launch sphere_scout_bringup gazebo_world.launch.py

# 2. Perception: the thermal camera in degC on /thermal/celsius, the colour 5-40 degC view for
#    the Gazebo "Thermal camera" panel, and /detected_target from the human-detection model.
tools/run_model_server.sh                 # port 8766, 127.0.0.1 only, no authentication
ros2 launch sphere_scout_ai perception.launch.py
ros2 run sphere_scout_ai print_thermal    # one 32 x 24 frame as a grid of degC

# 2b. Operator side: target_localizer_node, thermal_viewer_node, RViz
ros2 launch sphere_scout_bringup operator.launch.py   # rviz:=false to skip RViz

# 2c. 3D operator view in a browser (see tools/xr_viewer/bridge.py --help)
python3 tools/xr_viewer/bridge.py        # then open http://localhost:8765

# 3. Motor state machine
ros2 run sphere_scout_control motor_state_machine --ros-args -p use_sim_time:=true

# 4. Teleop (into the state machine, not straight to the robot)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop

# 5. Trigger a scan
ros2 service call /mission/scan std_srvs/srv/Trigger
```

To exercise the estimator without Gazebo, start the fake publisher first (it provides `/clock`) and then the operator launch:

```bash
python3 src/sphere_scout_ai/scripts/fake_detection_pub.py --demo --corners
ros2 launch sphere_scout_bringup operator.launch.py rviz:=false
```

## The operator views

**Browser 3D view** (`tools/xr_viewer`, read-only, nothing is published back to the robot): an orbit camera on the translucent apartment, with the robot's trail, the survivor estimate and its uncertainty ellipsoid, the rays from each viewpoint (rejected ones in red), the ground-truth marker, and the estimator's status text. The live thermal feed floats in a window you can drag, resize and snap to a corner. Keys: `X` solid or x-ray building, `C` clear the trail, `T` show or hide the thermal window.

**WebXR headset view**: the same scene in first person on a headset browser, served over HTTPS (`bridge.py --host 0.0.0.0 --https --cert-ip <LAN IP>`). It starts at the robot's spawn point, outside the structure, with thumbstick movement, snap turning and a tabletop "dollhouse" view. `--host 0.0.0.0` exposes the data to the whole network with no authentication, so use it on a trusted network only.

**RViz**: Fixed Frame `odom`; Image `/thermal/view` with Reliability set to Reliable; MarkerArray `/survivor/markers`; TF.

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

The model service behind `/detected_target` is called over HTTP by `thermal_detector_node`. Its parameters (`server_url`, `request_timeout_s`, `pixel_convention`, `resolution`) are in `src/sphere_scout_ai/config/thermal_detector.yaml`. The estimator is a continuous passive observer: it does not listen to `/mission/state`, `/cmd_vel` or `/odom`, and `/survivor/position` carries triangulated points only. Its parameters are in `src/sphere_scout_bringup/config/target_localizer.yaml`, and the thermal feed's are in `thermal_viewer.yaml`. `pixel_convention` must match across the detector, the estimator and the viewer.

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

## Placeholders

No physical value is invented in this repo.

| Placeholder | File |
|---|---|
| robot values still assumed: `shell_extension_max` (GS-1502 stroke), `ballast_mass`, `slider_mass`, IMU noise, drive limits, headlight power and beam (the rest come from the CAD and the mass calculator) | `src/sphere_scout_description/urdf/params.xacro` |
| `target_height_m`, `target_height_sigma_m` (enable the one-view PROVISIONAL estimate) | `src/sphere_scout_bringup/config/target_localizer.yaml` (commented out until Mech 2 gives values) |
| RViz layout `sphere_scout.rviz` (save from the RViz2 GUI: fixed frame `odom`, Image `/thermal/view` Reliable, MarkerArray `/survivor/markers`, TF) | `src/sphere_scout_description/rviz/` |
| MLX90640, MPU-6050, DRV8833 pin numbers | `firmware/sphere_scout_firmware/src/main.cpp` |
| package owners and maintainers | this README, `AGENTS.md`, every `package.xml` / `setup.py`, file headers |

Also open: the ESP32-S3 Mini board ID in `firmware/sphere_scout_firmware/platformio.ini`.

## The scenario

Rescue Randy (Gazebo Fuel, heat signature peaking at 310 K) is trapped at the south edge of the collapse, legs under the fallen floor and head in a low rubble void that only the robot can see into (`apartment.sdf`, `ground_truth.yaml`). The robot spawns outside at (5.0, −2.5) facing east (`gazebo_world.launch.py`), and that pose is the origin of the `odom` frame the position estimate is reported in.

## Tests

```bash
(cd src/sphere_scout_ai && python3 test/test_triangulation.py && python3 test/test_thermal_render.py && python3 test/test_msg_utils.py)
python3 tools/xr_viewer/test/test_bridge.py
```
