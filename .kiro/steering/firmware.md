---
inclusion: fileMatch
fileMatchPattern: ["firmware/**"]
---

# Firmware conventions (ESP32-S3, PlatformIO, micro-ROS)

- Arduino framework, micro-ROS for Jazzy over Wi-Fi (`board_microros_distro = jazzy`, `board_microros_transport = wifi`). Do not change them.
- The firmware must expose exactly the simulation's interfaces:
  - publishes `/imu/data` (`sensor_msgs/msg/Imu`) and `/thermal/image_raw` (`sensor_msgs/msg/Image`, 32 x 24, 16-bit, kelvin x 100)
  - subscribes `/cmd_vel` (`geometry_msgs/msg/Twist`) and `/shell/cmd` (`std_msgs/msg/Float64`)
  - optionally `/camera/image_raw/compressed` (JPEG, OV2640) as in `AGENTS.md`
- Pixel values must decode with `temp_c = pixel * 0.01 - 273.15`. Keep the byte order and `encoding` string identical to what the sim publishes so `decode_image` accepts both.
- Image stamps: set the header stamp from the synchronized agent time, so the localizer's TF lookup at the image stamp works.
- Hardware: MLX90640 on I2C bus 0 (400 kHz), MPU-6050 on I2C bus 1 (100 kHz), DRV8833 via PWM. Pin numbers are `TODO(kickoff)`. Never guess a pin or the board ID. Ask.
- No credentials in source. Wi-Fi SSID/password and the agent IP go in a git-ignored header or `platformio.ini` build flags from the environment.
- PlatformIO is not installed on the dev machine. State plainly that the firmware was not compiled when that is the case.
- Any change to a published or subscribed topic, type or encoding is an interface change (`#interface-change`).
