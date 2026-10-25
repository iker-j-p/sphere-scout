"""thermal_viewer_node: the driver's live thermal feed. ROS glue only.

Owner: TODO(kickoff)

Rendering lives in thermal_render.py (tested without ROS). This node:
  - subscribes /thermal/image_raw (sensor_msgs/Image, 16-bit mono, kelvin = value x 0.01),
    sensor-data QoS, decoded with numpy (no cv_bridge)
  - subscribes /detected_target (vision_msgs/Detection2DArray), overlay only: boxes whose stamp
    is within overlay_max_age_s of the image stamp
  - publishes /thermal/view (sensor_msgs/Image, rgb8, reliable, depth 1), header copied from
    the source image
  - watchdog at watchdog_hz: if images stop for stale_s, re-publishes the last frame with a
    NO FEED banner and a growing age; before the first image, a black frame with a
    WAITING banner

It draws what the detector found and facts about the feed. It highlights nothing of its own and
never advises. pixel_convention must equal the localizer's and the detector's.

Parameters: src/sphere_scout_bringup/config/thermal_viewer.yaml (all declared below with the
same defaults). fixed_range_k is unset = auto range.
"""
import numpy as np
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from sphere_scout_ai.msg_utils import bbox_center
from sphere_scout_ai.thermal_render import (
    COLORMAPS, Box, RangeTracker, decode_image, format_hud, render)

DEFAULTS = {
    'scale': 20,
    'interp': 'nearest',
    'colormap': 'inferno',
    'kelvin_per_count': 0.01,
    'min_span_k': 15.0,
    'range_alpha': 0.2,
    'overlay_max_age_s': 0.25,
    'stale_s': 0.5,
    'watchdog_hz': 2.0,
    'pixel_convention': 'index',
    'show_colorbar': True,
    'show_crosshair': True,
}
FALLBACK_W, FALLBACK_H = 32, 24      # Blueprint thermal image size, used only for placeholder frames


def _stamp_is_zero(stamp):
    return stamp.sec == 0 and stamp.nanosec == 0


def _stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ThermalViewerNode(Node):

    def __init__(self):
        super().__init__('thermal_viewer_node')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('fixed_range_k', Parameter.Type.DOUBLE_ARRAY)
        p = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.scale = int(p['scale'])
        self.interp, self.colormap = p['interp'], p['colormap']
        self.kelvin_per_count = p['kelvin_per_count']
        self.overlay_max_age_s = p['overlay_max_age_s']
        self.stale_s = p['stale_s']
        self.pixel_convention = p['pixel_convention']
        self.show_colorbar = bool(p['show_colorbar'])
        self.show_crosshair = bool(p['show_crosshair'])
        if self.colormap not in COLORMAPS:
            raise ValueError(f'colormap must be one of {sorted(COLORMAPS)}, got {self.colormap!r}')
        if self.interp not in ('nearest', 'cubic'):
            raise ValueError(f"interp must be 'nearest' or 'cubic', got {self.interp!r}")
        if self.pixel_convention not in ('index', 'edge'):
            raise ValueError(f"pixel_convention must be 'index' or 'edge', got {self.pixel_convention!r}")
        fixed = self._opt('fixed_range_k')
        if fixed is not None and (len(fixed) != 2 or not fixed[0] < fixed[1]):
            raise ValueError('fixed_range_k must be [lo, hi] with lo < hi')
        self.tracker = RangeTracker(p['range_alpha'], p['min_span_k'], fixed)

        self.det_boxes = []          # [(stamp_s, Box)] from the newest /detected_target message
        self.last = None             # newest frame, kept for the watchdog
        self.last_rx = None          # node-clock time of the newest image
        self.bad_warned = False

        self.pub = self.create_publisher(
            Image, '/thermal/view', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Image, '/thermal/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/detected_target', self.on_det, 10)
        self.create_timer(1.0 / p['watchdog_hz'], self.on_watchdog)
        self.get_logger().info(
            f'thermal_viewer_node up: scale {self.scale}, {self.interp}, {self.colormap}, '
            f'{"auto" if self.tracker.auto else "fixed"} range, pixel_convention={self.pixel_convention}')

    def _opt(self, name):
        try:
            return self.get_parameter(name).value
        except ParameterUninitializedException:
            return None

    # ------------------------------------------------------------------ callbacks
    def on_det(self, msg):
        boxes = []
        for d in msg.detections:
            stamp = d.header.stamp if not _stamp_is_zero(d.header.stamp) else msg.header.stamp
            cx, cy = bbox_center(d.bbox)
            boxes.append((_stamp_s(stamp), Box(cx, cy, d.bbox.size_x, d.bbox.size_y)))
        self.det_boxes = boxes

    def on_image(self, msg):
        now = self.get_clock().now()
        self.last_rx = now
        try:
            kelvin = decode_image(msg.data, msg.width, msg.height, msg.step,
                                  bool(msg.is_bigendian), msg.encoding, self.kelvin_per_count)
        except ValueError as e:
            if not self.bad_warned:
                self.get_logger().warn(f'bad image on /thermal/image_raw: {e}')
                self.bad_warned = True
            w, h = msg.width, msg.height
            if not (0 < w <= 1024 and 0 < h <= 1024):
                w, h = FALLBACK_W, FALLBACK_H
            self.last = {'bad': True, 'kelvin': np.zeros((h, w), np.float32), 'lo': 0.0, 'hi': 1.0,
                         'boxes': [], 'peak': 0.0, 'header': msg.header}
            self.publish_frame(self.last, 'BAD IMAGE', now)
            return
        self.bad_warned = False
        lo, hi = self.tracker.update(kelvin)
        t_img = _stamp_s(msg.header.stamp)
        boxes = [b for t, b in self.det_boxes if abs(t - t_img) <= self.overlay_max_age_s]
        self.last = {'bad': False, 'kelvin': kelvin, 'lo': lo, 'hi': hi, 'boxes': boxes,
                     'peak': float(kelvin.max()), 'header': msg.header}
        self.publish_frame(self.last, None, now)

    def on_watchdog(self):
        now = self.get_clock().now()
        if self.last_rx is None:
            blank = {'bad': True, 'kelvin': np.zeros((FALLBACK_H, FALLBACK_W), np.float32),
                     'lo': 0.0, 'hi': 1.0, 'boxes': [], 'peak': 0.0, 'header': None}
            self.publish_frame(blank, 'WAITING FOR /thermal/image_raw', now)
            return
        if (now - self.last_rx).nanoseconds * 1e-9 <= self.stale_s:
            return
        age = self._age(self.last, now)
        self.publish_frame(self.last, f'NO FEED {age:.1f} s', now)

    # --------------------------------------------------------------------- output
    @staticmethod
    def _age(frame, now):
        return (now - Time.from_msg(frame['header'].stamp)).nanoseconds * 1e-9

    def publish_frame(self, frame, banner, now):
        if frame['bad']:
            hud, colorbar, crosshair = (), False, False
        else:
            hud = format_hud(frame['peak'], frame['lo'], frame['hi'], self.tracker.auto,
                             self._age(frame, now), len(frame['boxes']))
            colorbar, crosshair = self.show_colorbar, self.show_crosshair
        rgb = render(frame['kelvin'], frame['lo'], frame['hi'], frame['boxes'], self.scale,
                     self.colormap, self.interp, self.pixel_convention, hud, banner,
                     colorbar, crosshair)
        out = Image()
        if frame['header'] is not None:
            out.header = frame['header']
        else:
            out.header.stamp = now.to_msg()
        out.height, out.width = int(rgb.shape[0]), int(rgb.shape[1])
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = rgb.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalViewerNode()
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
