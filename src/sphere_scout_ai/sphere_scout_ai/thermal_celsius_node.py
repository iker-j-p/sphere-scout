"""thermal_celsius_node: the thermal camera in degrees Celsius.

Subscribes  /thermal/image_raw (sensor_msgs/Image, mono16, kelvin x 100: the simulation's L16
            output and the firmware's format)
Publishes   /thermal/celsius (sensor_msgs/Image, 32FC1): the same 32 x 24 frame with one float
            per pixel in degrees C, and the same header (stamp, thermal_link frame)

Parameter   resolution (0.01): kelvin per raw count, as set in the ThermalSensor plugin

Run in simulation with --ros-args -p use_sim_time:=true (perception.launch.py does).
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

KELVIN_AT_0C = 273.15


class ThermalCelsius(Node):

    def __init__(self):
        super().__init__('thermal_celsius_node')
        self.declare_parameter('resolution', 0.01)
        self.pub = self.create_publisher(Image, '/thermal/celsius', 10)
        self.create_subscription(Image, '/thermal/image_raw', self.on_image, 10)

    def on_image(self, msg):
        if msg.encoding not in ('mono16', '16UC1'):
            self.get_logger().warn(f'expected a mono16 thermal image, got {msg.encoding}',
                                   throttle_duration_sec=5.0)
            return
        raw = np.frombuffer(bytes(msg.data), dtype='>u2' if msg.is_bigendian else '<u2')
        raw = raw.reshape(msg.height, msg.step // 2)[:, :msg.width]
        celsius = raw.astype(np.float32) * self.get_parameter('resolution').value - KELVIN_AT_0C
        self.get_logger().info(
            f'first frame {msg.width} x {msg.height}: {celsius.min():.1f} to {celsius.max():.1f} degC',
            once=True)
        self.pub.publish(Image(header=msg.header, height=msg.height, width=msg.width,
                               encoding='32FC1', is_bigendian=0, step=msg.width * 4,
                               data=celsius.tobytes()))


def main():
    rclpy.init()
    node = ThermalCelsius()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
