"""thermal_detector_node: asks the human-detection model about every thermal frame.

Owner: TODO(kickoff)

Subscribes  /thermal/image_raw (sensor_msgs/Image, 16-bit, kelvin x 100; mono16 or 16UC1)
Publishes   /detected_target (vision_msgs/Detection2DArray), one message per answered frame:
              header and every Detection2D header = the source image header (stamp, frame_id)
              bbox centre and size in pixels of the 32 x 24 image, in `pixel_convention`
              results[0].hypothesis = class_id 'human', score = the model's human_score
            An EMPTY array means the model answered "no human" (or found no region). Nothing is
            published for a frame the model did not answer (server down, timeout, bad reply),
            so a dead server is never reported as "nothing seen".
Parameters  src/sphere_scout_ai/config/thermal_detector.yaml
              server_url          where the model server listens
              request_timeout_s   per-frame HTTP timeout
              pixel_convention    index | edge, must equal target_localizer.yaml / thermal_viewer.yaml
              resolution          kelvin per raw count (0.01, as in the ThermalSensor plugin)

The model is NOT part of this repo and not a ROS node. It is
https://github.com/andrew2719/thermal_rescue_ai_model, started separately with
    python -m sphere_scout serve --port 8766
(8765 is the default of that server but also of tools/xr_viewer/bridge.py). See model_client.py
for the request/response contract and the box convention. The server is unauthenticated: keep
it on 127.0.0.1 unless the network is trusted.

Frames are handed to a worker thread through a one-slot mailbox, so a slow server drops old
frames (counted in the log) instead of queueing them or blocking the executor.
Run in simulation with use_sim_time: true (perception.launch.py does).
"""
import threading

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from sphere_scout_ai.model_client import HEIGHT, WIDTH, ModelClient, ModelError
from sphere_scout_ai.thermal_render import decode_image

KELVIN_AT_0C = 273.15
STATS_PERIOD_S = 30.0


class ThermalDetectorNode(Node):
    def __init__(self):
        super().__init__('thermal_detector_node')
        self.declare_parameters('', [
            ('server_url', 'http://127.0.0.1:8766'),
            ('request_timeout_s', 0.5),
            ('pixel_convention', 'index'),
            ('resolution', 0.01),
        ])
        self.resolution = float(self.get_parameter('resolution').value)
        self.client = ModelClient(self.get_parameter('server_url').value,
                                  self.get_parameter('request_timeout_s').value,
                                  self.get_parameter('pixel_convention').value)
        self.pub = self.create_publisher(Detection2DArray, '/detected_target', 10)
        self.create_subscription(Image, '/thermal/image_raw', self.on_image, qos_profile_sensor_data)
        self.cond = threading.Condition()
        self.slot = None            # newest unprocessed (header, celsius, frame_id)
        self.stop = False
        self.server_ok = None       # None until the first answer or failure
        self.counts = {'answered': 0, 'failed': 0, 'dropped': 0, 'human': 0}
        self.worker = threading.Thread(target=self.work, name='model_requests', daemon=True)
        self.worker.start()
        self.create_timer(STATS_PERIOD_S, self.log_stats)
        self.get_logger().info(
            f'thermal_detector_node up: model server {self.client.base_url}, '
            f'pixel_convention={self.client.pixel_convention}')

    # --------------------------------------------------------------- ROS callback
    def on_image(self, msg):
        if (msg.width, msg.height) != (WIDTH, HEIGHT):
            self.get_logger().error(
                f'image is {msg.width} x {msg.height}, the model needs {WIDTH} x {HEIGHT}',
                throttle_duration_sec=10.0)
            return
        try:
            kelvin = decode_image(msg.data, msg.width, msg.height, msg.step, msg.is_bigendian,
                                  msg.encoding, self.resolution)
        except ValueError as exc:
            self.get_logger().error(f'bad image: {exc}', throttle_duration_sec=10.0)
            return
        celsius = kelvin.astype(np.float64) - KELVIN_AT_0C
        self.get_logger().debug(f'frame min {celsius.min():.1f} C, max {celsius.max():.1f} C')
        frame_id = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        with self.cond:
            if self.slot is not None:
                self.counts['dropped'] += 1
            self.slot = (msg.header, celsius, frame_id)
            self.cond.notify()

    # ------------------------------------------------------------- worker thread
    def work(self):
        while True:
            with self.cond:
                while self.slot is None and not self.stop:
                    self.cond.wait()
                if self.stop:
                    return
                header, celsius, frame_id = self.slot
                self.slot = None
            try:
                result = self.client.predict(celsius, frame_id)
            except (ModelError, ValueError) as exc:
                self.counts['failed'] += 1
                if self.server_ok is not False:
                    self.get_logger().warn(
                        f'no answer from the model server at {self.client.base_url} ({exc}). '
                        'Start it with `python -m sphere_scout serve --port 8766` from the model '
                        'repo. /detected_target stays silent until it answers.')
                self.server_ok = False
                continue
            if self.server_ok is not True:
                self.get_logger().info(
                    f'model server answering ({result.model_version or "unknown version"})')
            self.server_ok = True
            self.counts['answered'] += 1
            self.counts['human'] += int(result.human)
            for w in result.warnings:
                self.get_logger().debug(f'model warning: {w}')
            self.pub.publish(self.to_message(header, result))

    def to_message(self, header, result):
        out = Detection2DArray()
        out.header = header
        for cx, cy, sx, sy in result.boxes:
            det = Detection2D()
            det.header = header
            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.size_x = float(sx)
            det.bbox.size_y = float(sy)
            det.results.append(self.hypothesis(result.score))
            out.detections.append(det)
        return out

    @staticmethod
    def hypothesis(score):
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = 'human'
        h.hypothesis.score = float(score)
        return h

    def log_stats(self):
        c = self.counts
        self.get_logger().info(
            f'last {STATS_PERIOD_S:.0f} s: answered {c["answered"]} (human {c["human"]}), '
            f'failed {c["failed"]}, dropped {c["dropped"]}')
        for k in c:
            c[k] = 0

    def shutdown(self):
        with self.cond:
            self.stop = True
            self.cond.notify()
        self.worker.join(timeout=2.0)


def main():
    rclpy.init()
    node = ThermalDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
