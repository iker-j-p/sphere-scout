"""Run: python3 tools/xr_viewer/test/test_bridge.py  (ROS sourced; no ROS graph needed)"""
import base64
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from bridge import Hub, image_to_jpeg  # noqa: E402

W, H = 640, 480


def rgb_frame():
    img = np.zeros((H, W, 3), np.uint8)
    img[:, :, 0] = np.linspace(0, 255, W, dtype=np.uint8)[None, :]          # red ramp
    img[100:200, 100:200] = (0, 255, 0)                                      # a green "box" patch
    return img


def decode(jpg):
    return cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)[:, :, ::-1]   # -> rgb


def test_jpeg_roundtrip_keeps_size_and_colours():
    img = rgb_frame()
    jpg = image_to_jpeg(img.tobytes(), W, H, W * 3, 'rgb8')
    out = decode(jpg)
    assert out.shape == (H, W, 3) and jpg[:2] == b'\xff\xd8'
    assert len(jpg) < 120_000, len(jpg)
    patch = out[120:180, 120:180].reshape(-1, 3).mean(axis=0)
    assert patch[1] > 230 and patch[0] < 30 and patch[2] < 30, patch        # green stays green, not blue/red swapped
    assert abs(float(out[300, 500, 0]) - float(img[300, 500, 0])) < 12      # red channel ramp preserved


def test_row_padding_is_honoured():
    img = rgb_frame()
    pad = 12
    padded = b''.join(img[r].tobytes() + b'\x00' * pad for r in range(H))
    a = decode(image_to_jpeg(padded, W, H, W * 3 + pad, 'rgb8'))
    b = decode(image_to_jpeg(img.tobytes(), W, H, W * 3, 'rgb8'))
    assert np.abs(a.astype(int) - b.astype(int)).max() <= 2


def test_bad_input_raises():
    good = rgb_frame().tobytes()
    for args in ((good, W, H, W * 3, 'mono16'), (good[:-50], W, H, W * 3, 'rgb8'),
                 (good, W, H, W * 2, 'rgb8'), (good, 0, H, W * 3, 'rgb8'), (good, 99999, H, W * 3, 'rgb8')):
        try:
            image_to_jpeg(*args)
        except ValueError:
            continue
        raise AssertionError(f'expected ValueError for {args[1:]}')


def test_hub_replay_rules_and_backpressure():
    hub = Hub()
    assert not hub.has_clients()
    hub.publish({'type': 'markers', 'markers': []})
    hub.publish({'type': 'thermal', 'jpg': 'old'}, replay=False)
    q = hub.add()
    assert hub.has_clients()
    first = [json.loads(q.get_nowait())['type'] for _ in range(q.qsize())]
    assert first == ['markers'], first                       # markers replayed, the old thermal frame is not
    for i in range(200):                                     # slow client: never blocks, keeps the newest
        hub.publish({'type': 'thermal', 'jpg': str(i)}, replay=False)
    assert q.qsize() == 64
    last = None
    while not q.empty():
        last = json.loads(q.get_nowait())
    assert last['jpg'] == '199'
    hub.remove(q)
    assert not hub.has_clients()


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
