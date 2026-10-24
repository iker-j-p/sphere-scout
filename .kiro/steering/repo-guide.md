---
inclusion: always
---

# Sphere-Scout: repo guide

`AGENTS.md` (loaded automatically) is the source of truth for the fixed stack, hard rules and the topic table. This file is the map: what the system does, where things live, and how data flows. It avoids status claims that go stale. For current status read the file docstrings and the `TODO(kickoff)` table in `README.md`.

## What the system does

A spherical robot whose two hemispherical shells are the wheels of a differential drive. To search it stops, slides the shells apart to expose a 32 x 24 thermal camera, and rotates in place. Warm detections from several viewpoints are triangulated into one survivor position in the `odom` frame and shown against ground truth.

## Data flow

```
Gazebo thermal camera / ESP32-S3 (MLX90640)
   -> /thermal/image_raw (16-bit, kelvin x 100, 32 x 24)
        -> thermal_celsius_node   -> /thermal/celsius, /thermal/image_color
        -> thermal_detector_node  -> HTTP -> model server (:8766) -> /detected_target
        -> thermal_viewer_node    -> /thermal/view (rgb8 for the operator)
/detected_target + TF odom->base_link
        -> target_localizer_node  -> /survivor/position, /survivor/markers
              -> RViz, and tools/xr_viewer/bridge.py (:8765) -> browser 3D view

teleop -> /cmd_vel_teleop -> motor_state_machine -> /cmd_vel, /shell/cmd -> Gazebo / firmware
Gazebo /odom + /imu/data -> ekf_node -> /odometry/filtered + TF odom->base_link (EKF only)
```

The human-detection model is NOT part of this repo and not a ROS node. It is a separate HTTP service (`external/thermal_rescue_ai_model`, own venv, git-ignored). Port 8766 is the model, 8765 is the browser bridge. Do not swap them.

## Where things live

| Path | What |
|---|---|
| `src/sphere_scout_ai/sphere_scout_ai/` | Perception nodes. Pure, ROS-free, tested cores: `triangulation.py`, `thermal_render.py`, `msg_utils.py`, `model_client.py`. ROS glue: `*_node.py`. |
| `src/sphere_scout_ai/scripts/fake_detection_pub.py` | Test fixture that fakes `/clock`, TF, thermal images and detections. Run with `python3`, not installed. |
| `src/sphere_scout_ai/test/` | Plain-Python unit tests (run as scripts, not pytest). |
| `src/sphere_scout_bringup/` | `gazebo_world.launch.py`, `operator.launch.py`, and `config/` (`bridge.yaml`, `ekf.yaml`, `target_localizer.yaml`, `thermal_viewer.yaml`). |
| `src/sphere_scout_control/` | `motor_state_machine.py`: DRIVE, STOP, EXPAND, SCAN, RETRACT. |
| `src/sphere_scout_description/` | `urdf/params.xacro` (every robot dimension), `sphere.urdf.xacro`, `placeholder_cube.urdf.xacro`, meshes, RViz layouts. |
| `src/sphere_scout_gazebo/` | `worlds/apartment.sdf`, `worlds/climb_test.sdf`, `models/apartment/`, `config/ground_truth.yaml`. |
| `firmware/sphere_scout_firmware/` | PlatformIO + micro-ROS project (`COLCON_IGNORE`d). Must match the sim's interfaces exactly. |
| `tools/xr_viewer/` | Read-only browser 3D operator view (`bridge.py` + `web/`). `COLCON_IGNORE`d, adds no ROS interface. |
| `tools/` | `record_thermal.py` (JSONL thermal recorder), `export_apartment.py` (Blender FBX to GLB), `run_model_server.sh`. |
| `Building/` | Source FBX for the apartment. |
| `docs/` | Written notes, including a point-in-time session handoff (may be stale; code wins). |
| `external/` | Git-ignored clones and venvs. Do not edit the model repo from here. |
| `build/ install/ log/` | colcon output. Never edit. |

## Key facts worth knowing before touching things

- Frames: `base_link` +x forward. `odom` origin is the robot spawn pose and `odom` +x points into the structure. Localizer output: x = depth, y = lateral (+ left), z = height.
- Thermal encoding: 16-bit kelvin x 100 (`temp_c = pixel * 0.01 - 273.15`), identical in sim and firmware.
- `pixel_convention` (`index` or `edge`) must be the same in `thermal_detector.yaml`, `target_localizer.yaml` and `thermal_viewer.yaml`.
- The localizer is a passive estimator. It does not subscribe to `/mission/state`, `/cmd_vel` or `/odom` and never integrates the IMU.
- Operator-facing text states facts only (distances, sigma, status). No advice such as "move" or "rescan".
- Unset on purpose until the mechanical team provides them: `target_height_m`, `target_height_sigma_m`, `ground_truth_world`. Ask before assuming.
