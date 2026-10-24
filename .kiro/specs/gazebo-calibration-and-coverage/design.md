# Design: Gazebo calibration and coverage

## Approach

Measurement first, no behavioural change. Small helper scripts under `tools/` read the live ROS graph, record numbers, and print a summary. YAML values change only where R1 shows a mismatch. The localizer's maths (`triangulation.py`) is not touched.

```
gazebo_world.launch.py  (sim, headless ok)
perception.launch.py    (+ tools/run_model_server.sh)
operator.launch.py rviz:=false
        |
tools/calibration/*.py  -> subscribe, record, summarize -> docs/calibration_report.md
```

## Components

- `tools/calibration/inspect_interfaces.py` (R1): subscribes once to `/thermal/image_raw` and `/detected_target`, prints encoding, endianness, size, step, `frame_id`, stamps, and checks `tf2` for `odom -> frame_id` at the stamp. Reads the SDF/URDF FOV and axes by parsing the files.
- `tools/calibration/pose_jitter.py` (R3): samples TF `odom -> base_link` for N seconds while stationary and prints peak-to-peak x, y, z, yaw.
- `tools/calibration/bearing_check.py` (R4): reads the localizer's ray and compares with the bearing computed from `ground_truth.yaml` and the current pose.
- `tools/calibration/coverage_run.py` (R5): for each run, resets the localizer (`/target_localizer_node/reset`), triggers `/mission/scan`, waits for the scan to end (`/mission/state` back to DRIVE), then logs the final estimate, covariance and the Mahalanobis distance of the ground truth. Appends JSON lines to a results file.
- `docs/calibration_report.md` (R6): human-readable summary.

All are external helpers: they add no ROS interface, publish nothing except the existing reset and scan services, and set `use_sim_time`. They run under a private `ROS_DOMAIN_ID` only when the sim is also started on it.

## Decisions and open points

- Covariance source: the localizer publishes markers (2-sigma ellipsoid) but not a covariance topic. The coverage script reads the ellipsoid from `/survivor/markers` (namespace `uncertainty`). Adding a covariance topic would be an interface change and is out of scope.
- Run-to-run repeatability depends on EKF noise and sim physics. Report the seed or lack of one.
- The detector model is a separate process. Record its version string from the detector log.
- Open: whether the sim's thermal sensor noise matches the real MLX90640. Not part of this spec.

## Testing

The helpers are measurement tools. Verify each once against the no-Gazebo fixture (`fake_detection_pub.py`) where possible, then against Gazebo. Unit tests for any pure maths (Mahalanobis distance, bearing difference) can use the existing plain-script style.
