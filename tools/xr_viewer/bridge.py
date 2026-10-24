#!/usr/bin/env python3
"""xr_bridge: read-only bridge from ROS 2 to the browser viewer (desktop 3D or WebXR headset).

Owner: TODO(kickoff)

External tool. It lives outside src/ (COLCON_IGNOREd), adds no ROS interface and publishes
nothing. It only listens and serves:

  subscribes  /survivor/markers   (visualization_msgs/MarkerArray, frame odom)   existing topic
              /thermal/view       (sensor_msgs/Image rgb8, from thermal_viewer_node) existing topic
  TF          odom -> base_link   (EKF)                                          existing frame
  serves      GET /               the web app in tools/xr_viewer/web/
              GET /config.json    {spawn_pose, frame, mesh}  (spawn_pose is read from
                                  target_localizer.yaml, the single source of truth, so the
                                  viewer's world<->odom alignment cannot drift from the localizer)
              GET /models/apartment.glb   the Gazebo visual mesh (Z-up, same file Gazebo loads)
              GET /events         server-sent events: {"type":"markers",...} at the localizer
                                  rate, {"type":"robot","p":[x,y,z],"q":[x,y,z,w]} at 20 Hz,
                                  {"type":"thermal","jpg":<base64 JPEG>,"w":..,"h":..,"t":<send time>} at the
                                  /thermal/view rate (only encoded while a browser is connected;
                                  never replayed to a new client, so an old frame cannot pass
                                  for a live one)

Run (workspace sourced, localizer + something publishing markers running):
  python3 tools/xr_viewer/bridge.py                      # this PC only: http://localhost:8765
  python3 tools/xr_viewer/bridge.py --host 0.0.0.0 --https --cert-ip <LAN IP of the PC>
                                                          # Quest browser over Wi-Fi (see below)

WebXR needs a secure context. http://localhost counts as secure (desktop browser, or a
Quest Link / Air Link session using the PC's browser). A headset opening http://<LAN-IP>
does not, so for the standalone Quest browser serve HTTPS with --https; the certificate is
self-signed, so the headset shows a warning once that you must accept. --cert-ip must be the
address the headset types in. Under WSL2 that is the Windows host's LAN address, and Windows
must forward the port to WSL (netsh interface portproxy, or WSL mirrored networking).

Security: no authentication. Everything is GET-only and read-only, and only the web/ folder
plus the one mesh file are served. The default bind is 127.0.0.1; --host 0.0.0.0 exposes
the data (survivor estimate, robot pose, building mesh) to everyone on the network.
"""
import argparse
import base64
import json
import mimetypes
import queue
import socket
import ssl
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
WEB = HERE / 'web'
MESH = REPO / 'src/sphere_scout_gazebo/models/apartment/meshes/apartment.glb'
LOCALIZER_YAML = REPO / 'src/sphere_scout_bringup/config/target_localizer.yaml'
CERT_DIR = HERE / '.cert'
FRAME = 'odom'
BASE_FRAME = 'base_link'
ROBOT_HZ = 20.0

mimetypes.add_type('text/javascript', '.js')
mimetypes.add_type('model/gltf-binary', '.glb')


def r4(v):
    return round(float(v), 4)


JPEG_QUALITY = 80
MAX_SIDE_PX = 4096


def image_to_jpeg(data, width, height, step, encoding, quality=JPEG_QUALITY):
    """sensor_msgs/Image fields (rgb8) -> JPEG bytes. Raises ValueError on anything else."""
    if encoding != 'rgb8':
        raise ValueError(f'/thermal/view must be rgb8, got {encoding!r}')
    if not (0 < width <= MAX_SIDE_PX and 0 < height <= MAX_SIDE_PX):
        raise ValueError(f'unreasonable image size {width}x{height}')
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if step < width * 3 or buf.size < height * step:
        raise ValueError(f'image data too short: {buf.size} bytes for {height} rows of step {step}')
    rgb = buf[:height * step].reshape(height, step)[:, :width * 3].reshape(height, width, 3)
    ok, enc = cv2.imencode('.jpg', np.ascontiguousarray(rgb[:, :, ::-1]),
                           [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return enc.tobytes()


# ------------------------------------------------------------------------------- hub
class Hub:
    """Fans messages out to every connected browser; a slow client drops old messages."""

    def __init__(self):
        self.lock = threading.Lock()
        self.clients = set()
        self.latest = {}                    # type -> json string, replayed to new clients

    def add(self):
        q = queue.Queue(maxsize=64)
        with self.lock:
            self.clients.add(q)
            for s in self.latest.values():
                q.put_nowait(s)
        return q

    def remove(self, q):
        with self.lock:
            self.clients.discard(q)

    def has_clients(self):
        with self.lock:
            return bool(self.clients)

    def publish(self, obj, replay=True):
        s = json.dumps(obj, separators=(',', ':'))
        with self.lock:
            if replay:
                self.latest[obj['type']] = s
            for q in list(self.clients):
                try:
                    q.put_nowait(s)
                except queue.Full:
                    try:
                        q.get_nowait()
                        q.put_nowait(s)
                    except (queue.Empty, queue.Full):
                        pass


# ------------------------------------------------------------------------- ROS node
class Bridge(Node):
    def __init__(self, hub):
        super().__init__('xr_bridge', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.hub = hub
        self.scene = {}                     # (ns, id) -> marker json; replace semantics like RViz
        self.warned_frame = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(MarkerArray, '/survivor/markers', self.on_markers, 10)
        self.warned_thermal = False
        self.create_subscription(Image, '/thermal/view', self.on_thermal,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / ROBOT_HZ, self.on_robot)

    def on_markers(self, msg):
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                self.scene.clear()
                continue
            key = (m.ns, m.id)
            if m.action in (Marker.DELETE,):
                self.scene.pop(key, None)
                continue
            if m.header.frame_id and m.header.frame_id != FRAME:
                if not self.warned_frame:
                    self.get_logger().warn(f'marker in frame {m.header.frame_id!r}, only {FRAME!r} is drawn')
                    self.warned_frame = True
                continue
            pts = [r4(v) for p in m.points for v in (p.x, p.y, p.z)]
            cols = []
            if m.points and len(m.colors) == len(m.points):
                cols = [r4(v) for c in m.colors for v in (c.r, c.g, c.b, c.a)]
            self.scene[key] = {
                'ns': m.ns, 'id': m.id, 'type': m.type,
                'p': [r4(m.pose.position.x), r4(m.pose.position.y), r4(m.pose.position.z)],
                'q': [r4(m.pose.orientation.x), r4(m.pose.orientation.y),
                      r4(m.pose.orientation.z), r4(m.pose.orientation.w)],
                's': [r4(m.scale.x), r4(m.scale.y), r4(m.scale.z)],
                'c': [r4(m.color.r), r4(m.color.g), r4(m.color.b), r4(m.color.a)],
                'pts': pts, 'cols': cols, 'text': m.text,
            }
        self.hub.publish({'type': 'markers', 'markers': list(self.scene.values())})

    def on_thermal(self, msg):
        if not self.hub.has_clients():
            return                          # nobody watching: skip the JPEG encode
        try:
            jpg = image_to_jpeg(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        except ValueError as e:
            if not self.warned_thermal:
                self.get_logger().warn(f'thermal frame skipped: {e}')
                self.warned_thermal = True
            return
        self.warned_thermal = False
        self.hub.publish({'type': 'thermal', 'jpg': base64.b64encode(jpg).decode('ascii'),
                          'w': int(msg.width), 'h': int(msg.height), 't': round(time.time(), 3)},
                         replay=False)

    def on_robot(self):
        try:
            tf = self.tf_buffer.lookup_transform(FRAME, BASE_FRAME, rclpy.time.Time())
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        self.hub.publish({'type': 'robot', 'p': [r4(t.x), r4(t.y), r4(t.z)],
                          'q': [r4(q.x), r4(q.y), r4(q.z), r4(q.w)]})


# --------------------------------------------------------------------------- HTTP
def make_handler(hub, config):
    config_bytes = json.dumps(config).encode()

    class Handler(BaseHTTPRequestHandler):
        server_version = 'xr_bridge'

        def log_message(self, fmt, *args):      # keep the terminal quiet
            pass

        def _send(self, code, body, ctype, cache='no-cache'):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', cache)
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(body)

        def _file(self, path, cache='no-cache'):
            try:
                body = path.read_bytes()
            except OSError:
                return self._send(404, b'not found', 'text/plain')
            ctype = mimetypes.guess_type(str(path))[0] or 'application/octet-stream'
            self._send(200, body, ctype, cache)

        def do_HEAD(self):
            self.do_GET()

        def do_GET(self):
            route = self.path.split('?', 1)[0]
            if route == '/events':
                return self._events()
            if route == '/config.json':
                return self._send(200, config_bytes, 'application/json')
            if route == '/models/apartment.glb':
                return self._file(MESH, 'max-age=300')
            rel = 'index.html' if route in ('', '/') else route.lstrip('/')
            target = (WEB / rel).resolve()
            if WEB.resolve() not in target.parents or not target.is_file():
                return self._send(404, b'not found', 'text/plain')
            self._file(target)

        def _events(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            q = hub.add()
            try:
                while True:
                    try:
                        s = q.get(timeout=10.0)
                        self.wfile.write(f'data: {s}\n\n'.encode())
                    except queue.Empty:
                        self.wfile.write(b': keepalive\n\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ssl.SSLError, OSError):
                pass
            finally:
                hub.remove(q)

    return Handler


# ------------------------------------------------------------------------- startup
def read_spawn(override):
    if override:
        return [float(v) for v in override]
    try:
        doc = yaml.safe_load(LOCALIZER_YAML.read_text())
        spawn = doc['target_localizer_node']['ros__parameters']['spawn_pose']
        return [float(v) for v in spawn]
    except (OSError, KeyError, TypeError, ValueError) as e:
        sys.exit(f'cannot read spawn_pose from {LOCALIZER_YAML}: {e}. Pass --spawn X Y Z YAW.')


def lan_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))        # no packet is sent; just picks the outbound interface
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ips)


def ensure_cert(extra_ips):
    sans = ['DNS:localhost', 'IP:127.0.0.1'] + [f'IP:{ip}' for ip in extra_ips]
    CERT_DIR.mkdir(exist_ok=True)
    cert, key, stamp = CERT_DIR / 'cert.pem', CERT_DIR / 'key.pem', CERT_DIR / 'san.txt'
    if not (cert.exists() and key.exists() and stamp.exists() and stamp.read_text() == ','.join(sans)):
        subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '365',
                        '-keyout', str(key), '-out', str(cert), '-subj', '/CN=sphere-scout-xr',
                        '-addext', 'subjectAltName=' + ','.join(sans)],
                       check=True, capture_output=True)
        key.chmod(0o600)
        stamp.write_text(','.join(sans))
        print(f'generated self-signed certificate for {", ".join(sans)}')
    return cert, key


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--host', default='127.0.0.1', help='bind address (0.0.0.0 = whole network, no auth)')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--https', action='store_true', help='serve HTTPS with a self-signed certificate')
    ap.add_argument('--cert-ip', action='append', default=[],
                    help='address the headset will use (repeatable); added to the certificate')
    ap.add_argument('--spawn', type=float, nargs=4, metavar=('X', 'Y', 'Z', 'YAW'),
                    help='override spawn_pose from target_localizer.yaml')
    args, ros_args = ap.parse_known_args()

    if not MESH.is_file():
        sys.exit(f'mesh not found: {MESH}')
    config = {'spawn_pose': read_spawn(args.spawn), 'frame': FRAME, 'mesh': '/models/apartment.glb'}

    hub = Hub()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(hub, config))
    server.daemon_threads = True
    scheme = 'http'
    if args.https:
        cert, key = ensure_cert(args.cert_ip or [ip for ip in lan_ips()])
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(str(cert), str(key))
        server.socket = ctx.wrap_socket(server.socket, server_side=True, do_handshake_on_connect=False)
        scheme = 'https'
    threading.Thread(target=server.serve_forever, daemon=True).start()

    print(f'xr_bridge: spawn_pose (world->odom) {config["spawn_pose"]}')
    print(f'  open {scheme}://localhost:{args.port}/')
    if args.host not in ('127.0.0.1', 'localhost'):
        print(f'  listening on {args.host}:{args.port} with NO authentication (read-only data)')
        for ip in (args.cert_ip or lan_ips()):
            print(f'  headset URL (if reachable): {scheme}://{ip}:{args.port}/')

    rclpy.init(args=ros_args)
    node = Bridge(hub)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
