"""thermal_celsius_node: the thermal camera in degrees Celsius, plus a colour view of it.

Subscribes  /thermal/image_raw (sensor_msgs/Image, mono16, kelvin x 100: the simulation's L16
            output and the firmware's format)
Publishes   /thermal/celsius (sensor_msgs/Image, 32FC1): the same 32 x 24 frame with one float
            per pixel in degrees C, and the same header (stamp, thermal_link frame)
            /thermal/image_color (sensor_msgs/Image, rgb8): the frame coloured on a fixed
            color_min_c..color_max_c scale (inferno: dark = cold, yellow = hot), enlarged
            color_scale times with square pixels, with a scale bar and the hottest reading.
            A fixed scale, unlike an auto-scaled view, keeps a scene with nothing warm in it
            dark instead of stretching 0.1 degC of noise to black-and-white.

Parameters  resolution (0.01): kelvin per raw count, as set in the ThermalSensor plugin
            color_min_c (5.0), color_max_c (40.0), color_scale (10)

Run in simulation with --ros-args -p use_sim_time:=true (perception.launch.py does).
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

KELVIN_AT_0C = 273.15
BAR_HEIGHT = 28


def colourize(celsius, t_min, t_max, scale):
    """RGB view of a degC frame on a fixed scale, with a scale bar underneath."""
    to_byte = lambda c: (np.clip((c - t_min) / (t_max - t_min), 0.0, 1.0) * 255).astype(np.uint8)
    view = cv2.applyColorMap(to_byte(celsius), cv2.COLORMAP_INFERNO)
    view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    width = view.shape[1]
    ramp = np.linspace(t_min, t_max, width, dtype=np.float32)[None, :].repeat(10, axis=0)
    bar = np.zeros((BAR_HEIGHT, width, 3), np.uint8)
    bar[:10] = cv2.applyColorMap(to_byte(ramp), cv2.COLORMAP_INFERNO)
    font, white = cv2.FONT_HERSHEY_SIMPLEX, (255, 255, 255)
    cv2.putText(bar, f'{t_min:.0f} C', (2, 25), font, 0.4, white, 1, cv2.LINE_AA)
    label = f'{t_max:.0f} C'
    (w, _), _ = cv2.getTextSize(label, font, 0.4, 1)
    cv2.putText(bar, label, (width - w - 2, 25), font, 0.4, white, 1, cv2.LINE_AA)
    hottest = f'max {celsius.max():.1f} C'
    (w, _), _ = cv2.getTextSize(hottest, font, 0.4, 1)
    cv2.putText(bar, hottest, ((width - w) // 2, 25), font, 0.4, white, 1, cv2.LINE_AA)
    return cv2.cvtColor(np.vstack([view, bar]), cv2.COLOR_BGR2RGB)


class ThermalCelsius(Node):

    def __init__(self):
        super().__init__('thermal_celsius_node')
        self.declare_parameter('resolution', 0.01)
        self.declare_parameter('color_min_c', 5.0)
        self.declare_parameter('color_max_c', 40.0)
        self.declare_parameter('color_scale', 10)
        self.pub = self.create_publisher(Image, '/thermal/celsius', 10)
        self.color_pub = self.create_publisher(Image, '/thermal/image_color', 10)
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
        if self.color_pub.get_subscription_count() == 0:
            return
        rgb = colourize(celsius, self.get_parameter('color_min_c').value,
                        self.get_parameter('color_max_c').value,
                        int(self.get_parameter('color_scale').value))
        self.color_pub.publish(Image(header=msg.header, height=rgb.shape[0], width=rgb.shape[1],
                                     encoding='rgb8', is_bigendian=0, step=rgb.shape[1] * 3,
                                     data=rgb.tobytes()))


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
