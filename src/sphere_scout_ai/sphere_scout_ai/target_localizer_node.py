"""target_localizer_node: continuous, passive survivor estimator. ROS glue only.

Owner: TODO(kickoff)

All geometry lives in triangulation.py (tested without ROS). This node:
  - subscribes /detected_target (vision_msgs/Detection2DArray), continuous, empty = nothing seen
  - looks up TF (target_frame <- camera frame) AT THE DETECTION'S IMAGE STAMP, never blocking:
    frames whose TF is not available yet wait in a pending queue until pending_timeout_s
  - turns each accepted frame into an odom-frame ray and feeds a ViewTracker
  - at publish_rate_hz recomputes the estimate from all views and publishes
      /survivor/position (geometry_msgs/PointStamped, frame odom, TRIANGULATED points only)
      /survivor/markers  (visualization_msgs/MarkerArray, frame odom, facts only)
  - offers the private Trigger service ~/reset (clears views and the pending queue)

It does not subscribe to /mission/state, /cmd_vel or /odom and publishes nothing that moves
the robot. Camera pose comes only from TF (EKF + robot_state_publisher); the IMU is never
integrated here. Status text states facts (numbers); it never advises.

Detector contract: the Detection2DArray (or each Detection2D) carries the source image
stamp; bbox centre and size are in pixels of the 32 x 24 image; frame_id is the camera frame.

Parameters: src/sphere_scout_bringup/config/target_localizer.yaml (all declared below with
the same defaults). target_height_m, target_height_sigma_m and ground_truth_world are unset
on purpose until Mech 2 provides them.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from sphere_scout_ai.msg_utils import bbox_center
from sphere_scout_ai.triangulation import (
    Gates, HeightPrior, ViewTracker, estimate, intrinsics, pixel_to_ray_cam,
    quat_to_rot, rot_to_quat, rotation_angle_deg, size_prior_range, world_to_odom)

BASE_FRAME = 'base_link'   # AGENTS.md: the EKF publishes odom -> base_link; +x is forward

DEFAULTS = {
    'image_width': 32,
    'image_height': 24,
    'hfov_rad': 0.96,
    'vfov_rad': 0.0,
    'pixel_convention': 'index',
    'camera_frame': '',
    'camera_convention': 'gz',
    'target_frame': 'odom',
    'view_radius_m': 0.15,
    'min_frames_per_view': 4,
    'ray_gate_deg': 4.0,
    'max_views': 200,
    'sigma_theta_rad': 0.01,
    'min_baseline_m': 0.5,
    'min_parallax_deg': 1.0,
    'max_chi2_per_dof': 9.0,
    'inlier_sigma': 3.0,
    'ransac_pairs': 100,
    'max_reject_frac': 0.5,
    'max_sigma_view_m': 0.5,
    'publish_low_confidence': True,
    'publish_rate_hz': 5.0,
    'pending_timeout_s': 1.0,
    'max_angular_rate_dps': 0.0,
    'max_speed_mps': 0.0,
    'motion_dt_s': 0.1,
    'prior_min_elev_deg': 2.0,
    'use_size_prior': True,
    'target_extent_min_m': 0.3,
    'target_extent_max_m': 1.7,
    'ellipsoid_sigma': 2.0,
    'ray_display_length_m': 8.0,
    'spawn_pose': [0.0, 0.0, 0.0, 0.0],
}

GREEN = (0.0, 1.0, 0.0)
AMBER = (1.0, 0.75, 0.0)
CYAN = (0.0, 1.0, 1.0)
RED = (1.0, 0.1, 0.1)
BLUE = (0.2, 0.4, 1.0)
GREY = (0.75, 0.75, 0.75)
STATUS_COLOR = {'CONFIRMED': GREEN, 'LOW_CONFIDENCE': AMBER, 'PROVISIONAL': CYAN}
BORDER_TOL_PX = 0.1


def _rgba(c, a=1.0):
    return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(a))


def _pt(v):
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def _stamp_is_zero(stamp):
    return stamp.sec == 0 and stamp.nanosec == 0


def _stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class TargetLocalizerNode(Node):

    def __init__(self):
        super().__init__('target_localizer_node')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('target_height_m', Parameter.Type.DOUBLE)
        self.declare_parameter('target_height_sigma_m', Parameter.Type.DOUBLE)
        self.declare_parameter('ground_truth_world', Parameter.Type.DOUBLE_ARRAY)
        self._load_params()

        self.tracker = ViewTracker(self.view_radius_m, self.max_views)
        self.pending = []
        self.last_stamp_key = None
        self.last_det_stamp = None      # builtin_interfaces/Time of the newest message
        self.last_status = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_pos = self.create_publisher(PointStamped, '/survivor/position', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/survivor/markers', 10)
        self.create_subscription(Detection2DArray, '/detected_target', self.on_det, 10)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)
        self.get_logger().info(
            f'target_localizer_node up: target_frame={self.target_frame}, '
            f'pixel_convention={self.pixel_convention}, camera_convention={self.camera_convention}, '
            f'height prior {"on" if self.prior is not None else "off"}, '
            f'ground truth {len(self.truth_odom)} point(s)')

    # ----------------------------------------------------------------- parameters
    def _opt(self, name):
        try:
            return self.get_parameter(name).value
        except ParameterUninitializedException:
            return None

    def _load_params(self):
        p = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.image_width, self.image_height = int(p['image_width']), int(p['image_height'])
        self.pixel_convention = p['pixel_convention']
        self.camera_convention = p['camera_convention']
        if self.pixel_convention not in ('index', 'edge'):
            raise ValueError(f"pixel_convention must be 'index' or 'edge', got {self.pixel_convention!r}")
        if self.camera_convention not in ('gz', 'optical'):
            raise ValueError(f"camera_convention must be 'gz' or 'optical', got {self.camera_convention!r}")
        self.camera_frame = p['camera_frame']
        self.target_frame = p['target_frame']
        self.fx, self.fy, self.cx, self.cy = intrinsics(
            self.image_width, self.image_height, p['hfov_rad'], p['vfov_rad'], self.pixel_convention)
        self.view_radius_m = p['view_radius_m']
        self.max_views = int(p['max_views'])
        self.min_frames = int(p['min_frames_per_view'])
        self.ray_gate_rad = math.radians(p['ray_gate_deg'])
        self.sigma_theta = p['sigma_theta_rad']
        self.gates = Gates(
            min_baseline_m=p['min_baseline_m'], min_parallax_deg=p['min_parallax_deg'],
            max_chi2_per_dof=p['max_chi2_per_dof'], max_sigma_view_m=p['max_sigma_view_m'],
            max_reject_frac=p['max_reject_frac'], inlier_sigma=p['inlier_sigma'],
            ransac_pairs=int(p['ransac_pairs']))
        self.publish_low_confidence = bool(p['publish_low_confidence'])
        self.publish_rate_hz = p['publish_rate_hz']
        self.pending_timeout_s = p['pending_timeout_s']
        self.max_ang_dps = p['max_angular_rate_dps']
        self.max_speed = p['max_speed_mps']
        self.motion_dt_s = p['motion_dt_s']
        self.motion_gate = self.max_ang_dps > 0.0 or self.max_speed > 0.0
        self.prior_min_elev_deg = p['prior_min_elev_deg']
        self.use_size_prior = bool(p['use_size_prior'])
        self.extent_min_m = p['target_extent_min_m']
        self.extent_max_m = p['target_extent_max_m']
        self.ellipsoid_sigma = p['ellipsoid_sigma']
        self.ray_len = p['ray_display_length_m']

        h, hs = self._opt('target_height_m'), self._opt('target_height_sigma_m')
        self.prior = None
        if h is not None and hs is not None:
            self.prior = HeightPrior(float(h), float(hs), self.prior_min_elev_deg)
        elif h is not None or hs is not None:
            self.get_logger().warn(
                'target_height_m and target_height_sigma_m are both required; height prior stays off')

        spawn = [float(v) for v in p['spawn_pose']]
        if len(spawn) != 4:
            raise ValueError('spawn_pose must be [x, y, z, yaw]')
        self.spawn = tuple(spawn)
        gt = self._opt('ground_truth_world')
        self.truth_odom = []
        if gt:
            if len(gt) % 3 != 0:
                raise ValueError('ground_truth_world must be a flat list x1, y1, z1, x2, ...')
            self.truth_odom = [world_to_odom(gt[i:i + 3], self.spawn) for i in range(0, len(gt), 3)]

    # -------------------------------------------------------------------- service
    def on_reset(self, request, response):
        self.tracker.clear()
        self.pending.clear()
        self.last_stamp_key = None
        self.last_det_stamp = None
        self.get_logger().info('reset: views and pending queue cleared')
        response.success = True
        response.message = 'views and pending queue cleared'
        return response

    # ----------------------------------------------------------------- detections
    def _touches_border(self, u, v, sx, sy):
        if self.pixel_convention == 'index':
            x_lo, x_hi = -0.5, self.image_width - 0.5
            y_lo, y_hi = -0.5, self.image_height - 0.5
        else:
            x_lo, x_hi = 0.0, float(self.image_width)
            y_lo, y_hi = 0.0, float(self.image_height)
        t = BORDER_TOL_PX
        return (u - sx / 2 <= x_lo + t or u + sx / 2 >= x_hi - t
                or v - sy / 2 <= y_lo + t or v + sy / 2 >= y_hi - t)

    def on_det(self, msg):
        # "last detection" is the newest message of any kind, empty arrays included
        stamp = msg.header.stamp
        if not msg.detections:
            self.last_det_stamp = stamp
            return
        det = max(msg.detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
        if len(msg.detections) > 1:
            self.get_logger().warn(
                f'{len(msg.detections)} blobs in one frame; using the largest (single survivor)',
                throttle_duration_sec=5.0)
        if not _stamp_is_zero(det.header.stamp):
            stamp = det.header.stamp
        self.last_det_stamp = stamp
        if _stamp_is_zero(stamp):
            self.get_logger().warn(
                'detection has a zero stamp; TF would be looked up at "latest" and duplicate '
                'stamps are dropped. The detector must copy the image stamp.',
                throttle_duration_sec=5.0)
        key = (stamp.sec, stamp.nanosec)
        if key == self.last_stamp_key:
            return
        self.last_stamp_key = key
        u, v = bbox_center(det.bbox)
        sx, sy = det.bbox.size_x, det.bbox.size_y
        if self._touches_border(u, v, sx, sy):
            return
        frame = self.camera_frame or det.header.frame_id or msg.header.frame_id
        if not frame:
            self.get_logger().warn('no camera frame: set camera_frame or fill the message frame_id',
                                   throttle_duration_sec=5.0)
            return
        self.pending.append({'stamp': stamp, 'frame': frame, 'u': u, 'v': v,
                             'extent': max(sx, sy), 'rx': self.get_clock().now()})
        self.process_pending()

    # -------------------------------------------------------------- pending queue
    def _lookup(self, frame, time):
        """Non-blocking: no timeout, so the executor keeps receiving TF."""
        return self.tf_buffer.lookup_transform(self.target_frame, frame, time)

    @staticmethod
    def _pose(tf):
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), quat_to_rot(q.x, q.y, q.z, q.w)

    def _try_add(self, it):
        """True when the frame is consumed (added or deliberately skipped), False if TF is missing."""
        stamp_t = Time.from_msg(it['stamp'])
        try:
            origin, R = self._pose(self._lookup(it['frame'], stamp_t))
            if self.motion_gate:
                prev = Time(nanoseconds=max(0, stamp_t.nanoseconds - int(self.motion_dt_s * 1e9)))
                origin0, R0 = self._pose(self._lookup(it['frame'], prev))
        except TransformException:
            return False
        if self.motion_gate:
            dt = self.motion_dt_s
            if self.max_ang_dps > 0.0 and rotation_angle_deg(R0, R) / dt > self.max_ang_dps:
                return True
            if self.max_speed > 0.0 and float(np.linalg.norm(origin - origin0)) / dt > self.max_speed:
                return True
        direction = R @ pixel_to_ray_cam(it['u'], it['v'], self.fx, self.fy, self.cx, self.cy,
                                         self.camera_convention)
        self.tracker.add_ray(origin, direction, it['extent'], _stamp_s(it['stamp']))
        return True

    def process_pending(self):
        now = self.get_clock().now()
        keep = []
        for it in self.pending:
            if self._try_add(it):
                continue
            if (now - it['rx']).nanoseconds * 1e-9 > self.pending_timeout_s:
                self.get_logger().warn(
                    f"dropped a frame: no TF {self.target_frame} <- {it['frame']} at its stamp "
                    f'within {self.pending_timeout_s:.1f} s',
                    throttle_duration_sec=5.0)
                continue
            keep.append(it)
        self.pending = keep

    # ---------------------------------------------------------------------- timer
    def on_timer(self):
        self.process_pending()
        rays = self.tracker.rays(self.ray_gate_rad, self.min_frames, self.sigma_theta)
        sol = estimate(rays, self.gates, self.prior)
        now = self.get_clock().now()
        if sol.status != self.last_status:
            self.get_logger().info(f'{self.last_status} -> {sol.status}: {sol.reason or "-"} '
                                   f'(views {sol.n_views}, used {sol.n_used}, rejected {sol.n_rejected})')
            self.last_status = sol.status
        if sol.status == 'CONFIRMED' or (sol.status == 'LOW_CONFIDENCE' and self.publish_low_confidence):
            ps = PointStamped()
            ps.header.stamp = now.to_msg()
            ps.header.frame_id = self.target_frame
            ps.point = _pt(sol.point)
            self.pub_pos.publish(ps)
        self.pub_markers.publish(self.build_markers(sol, now))

    # -------------------------------------------------------------------- markers
    def _marker(self, ns, mid, mtype, stamp):
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _robot_relative(self, point):
        """(3D distance m, horizontal bearing deg, + = left of the robot's heading) or None."""
        try:
            origin, R = self._pose(self._lookup(BASE_FRAME, Time()))
        except TransformException:
            return None
        d = point - origin
        yaw = math.atan2(R[1, 0], R[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        fwd, left = c * d[0] + s * d[1], -s * d[0] + c * d[1]
        return float(np.linalg.norm(d)), math.degrees(math.atan2(left, fwd))

    @staticmethod
    def _side(deg):
        return 'left' if deg >= 0 else 'right'

    def status_text(self, sol, now):
        lines = []
        p = sol.point
        if p is not None:
            lines.append(f'depth {p[0]:.2f} m  lateral {p[1]:+.2f} m  height {p[2]:.2f} m')
        if sol.status in ('CONFIRMED', 'LOW_CONFIDENCE'):
            lines.append(f'depth sigma {sol.sigma_view:.2f} m along view  {sol.status}')
            lines.append(f'views {sol.n_used} used, {sol.n_rejected} rejected  '
                         f'baseline {sol.baseline_m:.1f} m  parallax {sol.parallax_deg:.0f} deg')
        elif sol.status == 'PROVISIONAL':
            lines.append(f'PROVISIONAL  prior-based, height {self.prior.height_m:.2f} '
                         f'+- {self.prior.sigma_m:.2f} m')
            lines.append(f'triangulation unavailable: views {sol.n_views}: {sol.reason}')
        else:   # BEARING_ONLY
            lines.append(f'azimuth {sol.azimuth_deg:+.1f} deg ({self._side(sol.azimuth_deg)})  '
                         f'elevation {sol.elevation_deg:+.1f} deg')
            if self.use_size_prior and sol.ref_ray is not None:
                lo, hi = size_prior_range(sol.ref_ray.extent_px, self.fx,
                                          self.extent_min_m, self.extent_max_m)
                lines.append(f'range prior {lo:.1f} to {hi:.1f} m from blob size')
            lines.append(f'views {sol.n_views}: {sol.reason}')
        if p is not None:
            rel = self._robot_relative(p)
            if rel is not None:
                lines.append(f'from robot now: {rel[0]:.1f} m, bearing {rel[1]:+.0f} deg '
                             f'{self._side(rel[1])}')
            if self.truth_odom:
                t = min(self.truth_odom, key=lambda q: float(np.linalg.norm(q - p)))
                e = p - t
                lines.append(f'error dx {e[0]:+.2f} dy {e[1]:+.2f} dz {e[2]:+.2f} '
                             f'norm {float(np.linalg.norm(e)):.2f} m')
        if self.last_det_stamp is not None:
            age = (now - Time.from_msg(self.last_det_stamp)).nanoseconds * 1e-9
            lines.append(f'last detection {age:.1f} s ago')
        return '\n'.join(lines)

    def build_markers(self, sol, now):
        stamp = now.to_msg()
        arr = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)

        if sol.status != 'NO_TARGET':
            m = self._marker('status', 0, Marker.TEXT_VIEW_FACING, stamp)
            if sol.point is not None:
                anchor = sol.point + np.array([0.0, 0.0, 0.5])
            else:
                anchor = sol.ref_ray.origin + sol.ref_ray.direction * 1.0 + np.array([0.0, 0.0, 0.3])
            m.pose.position = _pt(anchor)
            m.scale.z = 0.12
            m.color = _rgba((1.0, 1.0, 1.0))
            m.text = self.status_text(sol, now)
            arr.markers.append(m)

        if sol.point is not None:
            col = STATUS_COLOR[sol.status]
            m = self._marker('estimate', 0, Marker.SPHERE, stamp)
            m.pose.position = _pt(sol.point)
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color = _rgba(col)
            arr.markers.append(m)

            w, V = np.linalg.eigh(sol.cov)
            if np.linalg.det(V) < 0:
                V[:, 0] *= -1.0
            m = self._marker('uncertainty', 0, Marker.SPHERE, stamp)
            m.pose.position = _pt(sol.point)
            q = rot_to_quat(V)
            m.pose.orientation.x, m.pose.orientation.y = float(q[0]), float(q[1])
            m.pose.orientation.z, m.pose.orientation.w = float(q[2]), float(q[3])
            sx, sy, sz = (max(0.02, 2.0 * self.ellipsoid_sigma * math.sqrt(max(float(e), 0.0))) for e in w)
            m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
            m.color = _rgba(col, 0.3)
            arr.markers.append(m)

        if sol.rays:
            m = self._marker('rays', 0, Marker.LINE_LIST, stamp)
            m.scale.x = 0.01
            m.color = _rgba(GREY)
            for ray, used in zip(sol.rays, sol.used):
                end = sol.point if (sol.point is not None and used) else ray.origin + ray.direction * self.ray_len
                col = GREY if used else RED
                m.points.append(_pt(ray.origin))
                m.points.append(_pt(end))
                m.colors.append(_rgba(col, 0.7))
                m.colors.append(_rgba(col, 0.7))
            arr.markers.append(m)

        if sol.status == 'BEARING_ONLY' and self.use_size_prior and sol.ref_ray is not None:
            r = sol.ref_ray
            lo, hi = size_prior_range(r.extent_px, self.fx, self.extent_min_m, self.extent_max_m)
            hi = min(hi, self.ray_len)
            if lo < hi:
                m = self._marker('size_prior', 0, Marker.LINE_STRIP, stamp)
                m.scale.x = 0.04
                m.color = _rgba(AMBER)
                m.points.append(_pt(r.origin + r.direction * lo))
                m.points.append(_pt(r.origin + r.direction * hi))
                arr.markers.append(m)

        for i, t in enumerate(self.truth_odom):
            m = self._marker('ground_truth', i, Marker.SPHERE, stamp)
            m.pose.position = _pt(t)
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color = _rgba(BLUE)
            arr.markers.append(m)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = TargetLocalizerNode()
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
