# Design: ESP32-S3 micro-ROS firmware

## Structure

Arduino framework with `micro_ros_platformio` (Jazzy, Wi-Fi transport), as in `platformio.ini`. A single `main.cpp` stays readable by delegating to small headers under `src/`:

| File | Responsibility |
|---|---|
| `main.cpp` | `setup()`/`loop()`, micro-ROS init, executor, reconnect state machine |
| `config.h` | Pin numbers (`TODO(kickoff)`), limits, timeouts. Only non-secret values |
| `secrets.h` | Wi-Fi and agent address. Git-ignored; `secrets.example.h` is tracked |
| `thermal.h/.cpp` | MLX90640 read, conversion to kelvin x 100 (uint16) |
| `imu.h/.cpp` | MPU-6050 read, `Imu` message fill |
| `drive.h/.cpp` | `Twist` to left/right PWM, timeout stop, clamping |
| `shell.h/.cpp` | `Float64` shell command to actuator |

## Behaviour

- Agent connection: a small state machine (WAITING_AGENT, AGENT_AVAILABLE, CONNECTED, DISCONNECTED) that creates entities on connect and destroys them on loss, the pattern from the micro-ROS Arduino examples.
- Time: sync with the agent (`rmw_uros_sync_session`) and stamp messages with `rmw_uros_epoch_nanos`.
- Thermal frame: MLX90640 returns 768 pixels in degrees Celsius. Convert each to `round((T + 273.15) * 100)` in a `uint16_t` and publish as little-endian 16-bit with the same `encoding` string as the sim. Check the exact string with `ros2 topic echo --once --field encoding /thermal/image_raw` in the sim.
- Message memory: pre-allocate the image `data` buffer (768 x 2 bytes); no allocation in the loop.
- Drive: `v` and `w` to wheel speeds with the wheel radius and track from `params.xacro`; clamp; stop after a no-command timeout.
- Rates: thermal at the sim's rate (the sim records about 8 Hz), IMU at the rate the EKF config expects.

## Decisions needing the user

- Pin map and board ID (not decided).
- Actuator mapping for `/shell/cmd`.
- Whether the OV2640 JPEG stream is in scope for this milestone.

## Verification

- Compile with PlatformIO (not installed on the dev machine: state this if it stays so).
- On hardware, with the agent running: `ros2 topic hz /thermal/image_raw`, `ros2 run sphere_scout_ai print_thermal`, and feed `/thermal/image_raw` to `thermal_viewer_node` to compare against the sim's look.
- Check `/imu/data` orientation signs against a known tilt.
