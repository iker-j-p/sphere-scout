# Tasks: ESP32-S3 micro-ROS firmware

- [ ] 0. Get the blocking inputs from the user (board ID, pins, Wi-Fi/agent, drive geometry, actuator mapping)
  - Do not start 2 to 6 with placeholder values presented as real
  - _Requirements: all_

- [ ] 1. Project skeleton and connectivity
  - [ ] 1.1 Set the confirmed board ID in `platformio.ini`
  - [ ] 1.2 Add `config.h`, `secrets.example.h`, and a `.gitignore` entry for `secrets.h`
  - [ ] 1.3 Connect to Wi-Fi and the agent with the reconnect state machine
  - _Requirements: R1_

- [ ] 2. Thermal publisher
  - [ ] 2.1 Read the MLX90640 and convert to kelvin x 100
  - [ ] 2.2 Publish `/thermal/image_raw` with the sim's encoding, stamp and `frame_id`
  - [ ] 2.3 Confirm `decode_image` accepts a captured message from hardware
  - _Requirements: R2_

- [ ] 3. IMU publisher
  - [ ] 3.1 Read the MPU-6050 and publish `/imu/data`
  - [ ] 3.2 Check axes and signs against a known tilt
  - _Requirements: R3_

- [ ] 4. Drive and shell subscribers
  - [ ] 4.1 `/cmd_vel` to DRV8833 PWM with clamping and a timeout stop
  - [ ] 4.2 `/shell/cmd` to the shell actuator
  - _Requirements: R4_

- [ ] 5. Optional OV2640 JPEG on `/camera/image_raw/compressed`
  - _Requirements: R5_

- [ ] 6. Parity check and report
  - Compare topics, types, frames and encodings with the `AGENTS.md` table; state whether the firmware was compiled and what was tested on hardware
  - _Requirements: R6_
