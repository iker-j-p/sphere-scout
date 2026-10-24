"""Record the thermal camera to JSON Lines, one frame per line, for testing detection models.

Usage (with the simulation running; Ctrl+C to stop):
  source ~/ss_ws/install/setup.bash && python3 tools/record_thermal.py thermal_run.jsonl

Each line is one 32 x 24 frame:
  {"t": 41.375,                       sim time of the frame (s)
   "frame": 12,                       index in this recording
   "width": 32, "height": 24,
   "celsius": [[...32 values...], ...24 rows],   top row first, degC, 2 decimals
   "min_c": 12.1, "max_c": 37.8,
   "pose": {"x": 3.1, "y": 1.6, "yaw_deg": 24.0},  robot in odom (spawn = 0, 0), or null
   "state": "DRIVE"}                  /mission/state, or null if the state machine isn't running

Read it back with:
  frames = [json.loads(line) for line in open("thermal_run.jsonl")]
  grid = numpy.array(frames[0]["celsius"])   # shape (24, 32)
"""
import json
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Recorder(Node):

    def __init__(self, path):
        super().__init__('record_thermal', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.out = open(path, 'w')
        self.count = 0
        self.pose = None
        self.state = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, '/thermal/image_raw', self.on_image, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.create_subscription(String, '/mission/state', self.on_state, latched)
        self.get_logger().info(f'recording /thermal/image_raw to {path} (Ctrl+C to stop)')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = {'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw_deg': round(math.degrees(yaw), 1)}

    def on_state(self, msg):
        self.state = msg.data

    def on_image(self, msg):
        raw = np.frombuffer(bytes(msg.data), dtype='>u2' if msg.is_bigendian else '<u2')
        raw = raw.reshape(msg.height, msg.step // 2)[:, :msg.width]
        celsius = raw.astype(np.float64) * 0.01 - 273.15   # kelvin x 100 -> degC
        self.out.write(json.dumps({
            't': round(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 3),
            'frame': self.count, 'width': msg.width, 'height': msg.height,
            'celsius': np.round(celsius, 2).tolist(),
            'min_c': round(float(celsius.min()), 2), 'max_c': round(float(celsius.max()), 2),
            'pose': self.pose, 'state': self.state}) + '\n')
        self.count += 1
        if self.count % 40 == 0:
            self.out.flush()
            self.get_logger().info(f'{self.count} frames (latest max {celsius.max():.1f} degC)')


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    rclpy.init()
    node = Recorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.out.close()
        print(f'saved {node.count} frames to {sys.argv[1]}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
