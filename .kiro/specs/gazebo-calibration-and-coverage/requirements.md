# Requirements: Gazebo calibration and coverage of the localizer

## Introduction

The localizer, viewer and detector were built and tested against a fake publisher. They have not been validated against the Gazebo thermal camera. This spec verifies the assumptions marked "unverified" and measures whether the reported uncertainty can be trusted. It changes no interface. Fixes found here go through `#interface-change` if they do.

Reference: Rescue Randy's head at world (13.17, -0.13, 0.18), which is (8.17, 2.37, 0.12) in `odom` for the default spawn (`sphere_scout_gazebo/config/ground_truth.yaml`).

## Requirements

### R1: Image and frame facts are confirmed
**User story:** As an integrator, I want the real sim image and frame properties recorded, so that the localizer's assumptions are checked instead of guessed.

1. WHEN `/thermal/image_raw` is inspected in the running sim THEN its encoding, endianness, width, height, step and `frame_id` SHALL be recorded.
2. WHEN `frame_id` of the image and of `/detected_target` is looked up in TF THEN the transform from `odom` SHALL exist at the image stamp.
3. WHEN the SDF thermal sensor block is read THEN its horizontal and vertical FOV SHALL be compared with `hfov_rad` and `vfov_rad` in `target_localizer.yaml`.
4. WHEN the camera frame axes are checked in URDF/SDF THEN `camera_convention` (`gz` or `optical`) SHALL be confirmed or corrected.
5. IF any recorded value differs from the YAML THEN the YAML SHALL be corrected and the difference reported.

### R2: Detector stamp and pixel convention are consistent
1. WHEN a detection is published THEN its header stamp SHALL equal the stamp of the image it came from.
2. WHEN a known hot blob is placed at a known pixel THEN the box centre SHALL fall on that pixel under the configured `pixel_convention` in the detector, localizer and viewer.

### R3: Pose jitter is below the view radius
1. WHILE the robot is stationary, the peak-to-peak change of `odom -> base_link` position SHALL be measured over at least 30 s.
2. IF the measured jitter is not clearly below `view_radius_m` (0.15 m) THEN the finding SHALL be reported before any parameter is changed.

### R4: Bearing calibration
1. WHEN the survivor is placed dead ahead of the stationary robot THEN the localizer's ray bearing SHALL agree with the true bearing within the angular size of one pixel.
2. WHEN the robot is rotated by a known angle THEN the ray SHALL rotate by the same angle with the correct sign (positive = left).

### R5: Coverage of the 2-sigma ellipsoid
1. WHEN the scan sequence is run at least 20 times from the default spawn THEN each run SHALL record the final status, the estimate, the covariance and the ground-truth head position.
2. The fraction of CONFIRMED runs whose ground truth lies inside the 2-sigma ellipsoid SHALL be reported with the run count.
3. IF that fraction is clearly below the expected ~95% THEN the report SHALL state that errors look like bias (EKF yaw drift, extrinsics, pixel convention) and SHALL NOT silently inflate `sigma_theta_rad`.

### R6: Reporting
1. The results SHALL be written to `docs/calibration_report.md` as facts and numbers, with what could not be verified listed explicitly.
2. No physical value SHALL be invented to make a check pass.
