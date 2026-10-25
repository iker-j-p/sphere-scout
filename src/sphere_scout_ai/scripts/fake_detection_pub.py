#!/usr/bin/env python3
"""Test fixture, no Gazebo: fake clock, moving camera TF, thermal image and detections.

Run it standalone, then start target_localizer_node / thermal_viewer_node with their normal
YAML (use_sim_time: true). This script keeps its own time base and publishes /clock, so the
nodes under test run exactly as in the sim.

  python3 scripts/fake_detection_pub.py --path sideways

All numbers below (target position, camera mount, noise) are TEST FIXTURE values, not robot
dimensions. The camera mount here is NOT the URDF.

Paths:   sideways  camera drives along +y at --speed, then holds
         parked    camera does not move
         parallel  camera drives along +x (toward the target), rays stay near-parallel
Demo:    --demo   self-running 30 s loop with announced phases (see DEMO below)
Corners: --corners  the fake "model" emits four corner pixels, converted with corners_to_bbox
Faults:  --no-image-after S   stop the image (and keep detections) after S seconds
         --det-lag S          detection stamp = image stamp - S
         --bad-image          publish rgb8 on /thermal/image_raw instead of 16UC1
         --corrupt-y LO HI    add a 6 px bearing error while the camera y is in [LO, HI]
"""
import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from vision_msgs.msg import Detection2D, Detection2DArray

from sphere_scout_ai.msg_utils import corners_to_bbox

T0 = 100.0   # sim time at start, seconds (non-zero on purpose)


def to_stamp(t):
    sec = int(math.floor(t))
    return sec, int(round((t - sec) * 1e9))


class Fake(Node):
    def __init__(self, a):
        super().__init__('fake_detection_pub')
        self.a = a
        self.wall0 = time.time()
        self.rng = np.random.default_rng(1)
        self.W, self.H = 32, 24
        self.fx = (self.W / 2) / math.tan(a.hfov / 2)
        self.cx, self.cy = self.W / 2 - 0.5, self.H / 2 - 0.5     # 'index' convention
        self.cam_off = np.array([0.1, 0.0, 0.15])                 # fixture mount, base_link -> camera
        self.target = np.array(a.target, float)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.img_pub = self.create_publisher(Image, '/thermal/image_raw', 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/detected_target', 10)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self._static_tf()
        self.reset_cli = self.create_client(Trigger, '/target_localizer_node/reset')
        self.last_phase, self.last_cycle = None, None
        self.create_timer(0.01, self.tick_clock)
        self.create_timer(0.02, self.tick_tf)
        self.create_timer(1.0 / 8.0, self.tick_frame)

    # time and pose ---------------------------------------------------------------
    def now_s(self):
        return T0 + (time.time() - self.wall0)

    # demo timeline: (start second within the cycle, name, what you should see)
    DEMO = [
        (0.0, 'PARKED', 'localizer: BEARING_ONLY, "views 1: single view"; no /survivor/position'),
        (5.0, 'DRIVING SIDEWAYS', 'LOW_CONFIDENCE then CONFIRMED; ellipsoid shrinks, rays fan out'),
        (13.0, 'HOLD', 'CONFIRMED stays, estimate does not move'),
        (19.0, 'DROPOUT (no image, no detections)',
         'viewer: NO FEED banner with growing age; markers: estimate persists, "last detection" age grows'),
        (24.0, 'FEED BACK', 'banner clears, box returns, last detection age back to ~0'),
    ]
    DEMO_CYCLE = 30.0

    def demo_phase(self, t):
        c = (t - T0) % self.DEMO_CYCLE
        name = self.DEMO[0]
        for ph in self.DEMO:
            if c >= ph[0]:
                name = ph
        return c, name

    def base_pos(self, t):
        e = t - T0
        a = self.a
        if a.demo:
            c, _ = self.demo_phase(t)
            return np.array([0.0, a.y0 + min(max(0.0, c - 5.0) * a.speed, a.span), 0.0])
        if a.path == 'sideways':
            return np.array([0.0, min(a.y0 + a.speed * e, a.y0 + a.span), 0.0])
        if a.path == 'parallel':
            return np.array([min(a.speed * e, a.span), 0.0, 0.0])
        return np.array([0.0, a.y0, 0.0])

    def _static_tf(self):
        m = TransformStamped()
        m.header.stamp.sec, m.header.stamp.nanosec = to_stamp(T0)
        m.header.frame_id, m.child_frame_id = 'base_link', self.a.camera_frame
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = self.cam_off
        m.transform.rotation.w = 1.0
        self.static.sendTransform(m)

    def tick_clock(self):
        c = Clock()
        c.clock.sec, c.clock.nanosec = to_stamp(self.now_s())
        self.clock_pub.publish(c)

    def tick_tf(self):
        t = self.now_s()
        m = TransformStamped()
        m.header.stamp.sec, m.header.stamp.nanosec = to_stamp(t)
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        p = self.base_pos(t)
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = p
        m.transform.rotation.w = 1.0
        self.tf.sendTransform(m)

    # frame -----------------------------------------------------------------------
    def project(self, t):
        cam = self.base_pos(t) + self.cam_off
        v = self.target - cam
        if v[0] <= 0.1:
            return None
        u = self.cx + self.fx * (-v[1] / v[0])
        w = self.cy + self.fx * (-v[2] / v[0])
        size = max(2.0, self.fx * 0.5 / v[0])
        return u, w, size, cam

    def demo_step(self, t):
        """Announce phase changes, reset the localizer at each cycle start. True = feed is down."""
        c, ph = self.demo_phase(t)
        cycle = int((t - T0) // self.DEMO_CYCLE)
        if ph[1] != self.last_phase or cycle != self.last_cycle:
            if ph[0] == 0.0:
                if self.reset_cli.service_is_ready():
                    self.reset_cli.call_async(Trigger.Request())
                    note = ' (localizer reset requested)'
                else:
                    note = ' (localizer reset service not available; run the launch first)'
            else:
                note = ''
            self.get_logger().info(f'cycle {cycle} | {ph[1]}{note}\n    expect: {ph[2]}')
            self.last_phase, self.last_cycle = ph[1], cycle
        return ph[1].startswith('DROPOUT')

    def tick_frame(self):
        t = self.now_s()
        a = self.a
        if a.demo and self.demo_step(t):
            return                      # dropout: neither image nor detections
        pr = self.project(t)
        s_img = to_stamp(t)
        s_det = to_stamp(t - a.det_lag)
        arr = Detection2DArray()
        arr.header.stamp.sec, arr.header.stamp.nanosec = s_det
        arr.header.frame_id = a.camera_frame
        blob = None
        if pr is not None:
            u, v, size, cam = pr
            u += self.rng.normal(0, a.noise_px)
            v += self.rng.normal(0, a.noise_px)
            if a.corrupt_y is not None and a.corrupt_y[0] <= cam[1] <= a.corrupt_y[1]:
                u += 6.0      # gross bearing error while the camera is inside this y window
            if 0.0 <= u <= self.W - 1 and 0.0 <= v <= self.H - 1:
                blob = (u, v, size)
                d = Detection2D()
                d.header = arr.header
                s = float(round(size))
                if a.corners:
                    # stand-in for the model: four corner pixels (index convention, inclusive),
                    # converted at the "detector boundary" with the same helper the detector will use
                    h = (s - 1.0) / 2.0
                    corners = [(u - h, v - h), (u + h, v - h), (u + h, v + h), (u - h, v + h)]
                    u, v, sx, sy = corners_to_bbox(corners, 'index')
                else:
                    sx = sy = s
                d.bbox.center.position.x, d.bbox.center.position.y = float(u), float(v)
                d.bbox.size_x, d.bbox.size_y = float(sx), float(sy)
                arr.detections.append(d)
        self.det_pub.publish(arr)

        stop = a.no_image_after is not None and (t - T0) > a.no_image_after
        if stop:
            return
        img = Image()
        img.header.stamp.sec, img.header.stamp.nanosec = s_img
        img.header.frame_id = a.camera_frame
        img.height, img.width = self.H, self.W
        if a.bad_image:
            img.encoding, img.step = 'rgb8', self.W * 3
            img.data = bytes(self.H * self.W * 3)
        else:
            k = np.full((self.H, self.W), 295.0) + self.rng.normal(0, 0.05, (self.H, self.W))
            if blob is not None:
                u, v, size = blob
                r = int(round(size / 2))
                k[max(0, int(round(v)) - r):int(round(v)) + r + 1,
                  max(0, int(round(u)) - r):int(round(u)) + r + 1] = 310.0
            img.encoding, img.is_bigendian, img.step = '16UC1', 0, self.W * 2
            img.data = np.rint(k / 0.01).astype('<u2').tobytes()
        self.img_pub.publish(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--demo', action='store_true',
                    help='30 s loop: parked, drive, hold, feed dropout, feed back. Announces each phase '
                         'and resets the localizer at every cycle start. Ignores --path.')
    ap.add_argument('--path', choices=['sideways', 'parked', 'parallel'], default='sideways')
    ap.add_argument('--target', type=float, nargs=3, default=[4.0, 0.3, 0.3])
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--y0', type=float, default=-0.8)
    ap.add_argument('--span', type=float, default=1.6)
    ap.add_argument('--noise-px', type=float, default=0.2)
    ap.add_argument('--hfov', type=float, default=0.96)
    ap.add_argument('--camera-frame', default='thermal_link')
    ap.add_argument('--no-image-after', type=float, default=None)
    ap.add_argument('--det-lag', type=float, default=0.0)
    ap.add_argument('--corners', action='store_true',
                    help='emit the blob as four corner pixels and convert with msg_utils.corners_to_bbox')
    ap.add_argument('--bad-image', action='store_true')
    ap.add_argument('--corrupt-y', type=float, nargs=2, default=None, metavar=('Y_LO', 'Y_HI'))
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Fake(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
