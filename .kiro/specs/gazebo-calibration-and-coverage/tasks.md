# Tasks: Gazebo calibration and coverage

- [ ] 1. Bring up the sim and record interface facts (R1)
  - [ ] 1.1 Launch `gazebo_world.launch.py`, `tools/run_model_server.sh`, `perception.launch.py`, `operator.launch.py rviz:=false` and confirm `ros2 topic hz /thermal/image_raw` and `/detected_target`
  - [ ] 1.2 Write `tools/calibration/inspect_interfaces.py` and record encoding, endianness, size, step, `frame_id`, TF availability
  - [ ] 1.3 Compare SDF FOV and camera axes with `hfov_rad`, `vfov_rad`, `camera_convention`; correct the YAML where they differ and report it
  - _Requirements: R1_

- [ ] 2. Check detector stamp and pixel convention (R2)
  - [ ] 2.1 Confirm each `/detected_target` stamp equals its source image stamp
  - [ ] 2.2 Check a known hot pixel against the box centre under `pixel_convention` in all three YAMLs
  - _Requirements: R2_

- [ ] 3. Measure stationary pose jitter (R3)
  - [ ] 3.1 Write `tools/calibration/pose_jitter.py` and run it for at least 30 s
  - [ ] 3.2 Report the result against `view_radius_m` before changing anything
  - _Requirements: R3_

- [ ] 4. Bearing calibration (R4)
  - [ ] 4.1 Write `tools/calibration/bearing_check.py`
  - [ ] 4.2 Dead-ahead check and known-rotation sign check
  - _Requirements: R4_

- [ ] 5. Coverage test (R5)
  - [ ] 5.1 Write `tools/calibration/coverage_run.py` with a pure, plain-script-testable Mahalanobis function
  - [ ] 5.2 Run at least 20 scans from the default spawn
  - [ ] 5.3 Compute and report the coverage fraction and run count; diagnose bias if low
  - _Requirements: R5_

- [ ] 6. Write `docs/calibration_report.md` (R6)
  - List numbers, what passed, what could not be verified, and any YAML changes made
  - _Requirements: R6_
