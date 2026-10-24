// Sphere-Scout firmware (ESP32-S3, micro-ROS over Wi-Fi).
// Owner: TODO(kickoff)
//
// Interfaces must match the simulation exactly.

#include <Arduino.h>

// TODO: Wi-Fi credentials and micro-ROS agent IP/port; connect to the agent.

// TODO: publishers
//   /imu/data           sensor_msgs/msg/Imu
//   /thermal/image_raw  sensor_msgs/msg/Image, 32 x 24, 16-bit, kelvin x 100 (same as sim L16, resolution 0.01)

// TODO: subscribers
//   /cmd_vel    geometry_msgs/msg/Twist
//   /shell/cmd  std_msgs/msg/Float64

// TODO: MLX90640 thermal camera on I2C bus 0 (400 kHz). Pins: TODO(kickoff)
// TODO: MPU-6050 IMU on I2C bus 1 (100 kHz). Pins: TODO(kickoff)
// TODO: DRV8833 motor driver, PWM. Pins: TODO(kickoff)

void setup() {
  // TODO: implement
}

void loop() {
  // TODO: implement
}
