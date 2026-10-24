# Requirements: ESP32-S3 micro-ROS firmware

## Introduction

`firmware/sphere_scout_firmware/src/main.cpp` is a skeleton of TODO comments. The firmware must expose the same ROS interfaces as the simulation so the rest of the pipeline runs unchanged on hardware. Pin numbers, the board ID and Wi-Fi details are not decided and must not be guessed.

## Blocking inputs (ask the user, do not assume)

- ESP32-S3 Mini board ID for `platformio.ini`
- Pin numbers: MLX90640 (I2C bus 0), MPU-6050 (I2C bus 1), DRV8833 (PWM), shell actuator signal
- Wi-Fi SSID/password and micro-ROS agent IP/port (kept out of git)
- Wheel geometry and drive limits for `/cmd_vel` conversion (from `params.xacro`)
- Shell actuator mapping from `/shell/cmd` (metres) to the actuator command (GS-1502 stroke)

## Requirements

### R1: Connectivity
1. WHEN powered THEN the firmware SHALL connect to Wi-Fi and to the micro-ROS agent, and retry on loss without rebooting.
2. Credentials SHALL come from a git-ignored header or build flags, never from tracked source.

### R2: Thermal image publisher
1. The firmware SHALL publish `sensor_msgs/msg/Image` on `/thermal/image_raw`, 32 x 24, 16-bit, kelvin x 100, so that `temp_c = pixel * 0.01 - 273.15`.
2. The encoding string and byte order SHALL match what the simulation publishes, so `thermal_render.decode_image` accepts both.
3. The header stamp SHALL be taken from the synchronized agent time, and `frame_id` SHALL be the camera frame present in TF.
4. IF a frame read fails THEN no message SHALL be published for that frame.

### R3: IMU publisher
1. The firmware SHALL publish `sensor_msgs/msg/Imu` on `/imu/data` with orientation covariance and units consistent with the simulation IMU.

### R4: Command subscribers
1. The firmware SHALL subscribe to `/cmd_vel` (`geometry_msgs/msg/Twist`) and drive the DRV8833 as a differential drive.
2. The firmware SHALL subscribe to `/shell/cmd` (`std_msgs/msg/Float64`) and move the shells.
3. IF no `/cmd_vel` arrives for a timeout THEN the motors SHALL stop.
4. Commands SHALL be clamped to the drive limits in `params.xacro`.

### R5: Optional camera
1. IF the OV2640 is fitted THEN the firmware MAY publish JPEG on `/camera/image_raw/compressed`.

### R6: Interface parity and honesty
1. Topic names, types and frames SHALL equal the `AGENTS.md` table. Any deviation is an interface change.
2. WHEN PlatformIO is not installed THEN the report SHALL state the firmware was not compiled.
