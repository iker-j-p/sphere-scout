"""print_thermal: print one /thermal/celsius frame as a grid of degrees C, then exit.

Rows are top to bottom and columns left to right, as the camera sees them.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = Node('print_thermal')
    frame = {}
    node.create_subscription(Image, '/thermal/celsius', lambda m: frame.setdefault('msg', m), 1)
    node.get_logger().info('waiting for /thermal/celsius (is thermal_celsius_node running?)')
    while 'msg' not in frame and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    msg = frame['msg']
    celsius = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
    hot = np.unravel_index(np.argmax(celsius), celsius.shape)
    print(f'{msg.width} x {msg.height} pixels in degC, frame {msg.header.frame_id}: '
          f'{celsius.min():.1f} to {celsius.max():.1f}, hottest at row {hot[0]} column {hot[1]}')
    print('     ' + ''.join(f'{c:>4d}' for c in range(msg.width)))
    for r, row in enumerate(celsius):
        print(f'{r:>4d} ' + ''.join(f'{v:4.0f}' for v in row))
    node.destroy_node()
    rclpy.try_shutdown()
