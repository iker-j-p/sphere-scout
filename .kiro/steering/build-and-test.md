---
inclusion: always
---

# Build, run and test

Run from the repo root (`/home/ikerj/Sphere-Scout/sphere-scout`, which is the colcon workspace).

## Environment
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash        # after a build
```
Installed on this machine: `python3`, `colcon`, ROS 2 Jazzy. NOT installed: Node (JS tests cannot run), PlatformIO (firmware cannot be built). Say so when a change needs either, and don't claim those checks passed.

## Build
```bash
colcon build --symlink-install                       # everything
colcon build --symlink-install --packages-select sphere_scout_ai   # one package
```
Rebuild after changing `setup.py`, `package.xml`, `CMakeLists.txt`, or adding a console script or launch/config file. With `--symlink-install`, edits to existing Python files take effect without a rebuild.

## Unit tests (no ROS graph needed)
```bash
(cd src/sphere_scout_ai && python3 test/test_triangulation.py && python3 test/test_thermal_render.py && python3 test/test_msg_utils.py)
python3 tools/xr_viewer/test/test_bridge.py
```
Each prints `ok <name>` per test. Any traceback is a failure. `model_client.py` and the nodes have no unit tests yet. JS tests (`tools/xr_viewer/test/test_{panel,frames,markers}.mjs`) need Node 20+.

## Running the system
```bash
# Full sim (Gazebo + robot + bridge + EKF); headless:=true for no GUI
ros2 launch sphere_scout_bringup gazebo_world.launch.py
# Perception (celsius node + detector). Needs the model server running.
tools/run_model_server.sh            # port 8766
ros2 launch sphere_scout_ai perception.launch.py
# Operator side: localizer + viewer + RViz (rviz:=false to skip)
ros2 launch sphere_scout_bringup operator.launch.py
# Browser 3D view
python3 tools/xr_viewer/bridge.py    # http://localhost:8765
# Control
ros2 run sphere_scout_control motor_state_machine --ros-args -p use_sim_time:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
ros2 service call /mission/scan std_srvs/srv/Trigger
```
The first Gazebo launch downloads Rescue Randy from Fuel (about 12 MB), so it needs internet once.

## Running without Gazebo
`fake_detection_pub.py` provides `/clock`, TF, a 16-bit thermal image and detections. Start it FIRST (it owns `/clock`), then the localizer launch.
```bash
python3 src/sphere_scout_ai/scripts/fake_detection_pub.py --demo --corners
ros2 launch sphere_scout_bringup operator.launch.py rviz:=false
```
Useful flags: `--path sideways|parked|parallel`, `--target X Y Z`, `--noise-px`, `--no-image-after S`, `--bad-image`. Its camera mount and target are test values, not robot dimensions.

## Isolation when testing
Use a private domain and port so the user's own demo is untouched:
```bash
ROS_DOMAIN_ID=87 python3 src/sphere_scout_ai/scripts/fake_detection_pub.py --demo
```
Run long orchestration from a script file under `/tmp`, then clean up by PID. Under `use_sim_time` with no `/clock`, timers don't tick, so watchdogs never fire.

## Reading results
- A launch that prints no errors can still publish nothing. Check with `ros2 topic hz <topic>` or `ros2 topic echo --once <topic>`.
- RViz: Fixed Frame `odom`; Image `/thermal/view` with Reliability = Reliable; MarkerArray `/survivor/markers`; TF.
