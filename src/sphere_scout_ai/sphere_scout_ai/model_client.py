"""Client for the human-detection model server. No ROS imports, so it runs in plain pytest.

The model lives in its own repository (https://github.com/andrew2719/thermal_rescue_ai_model)
and runs as a separate HTTP process (`python -m sphere_scout serve`), which keeps torch out of
the ROS environment. This module only speaks that server's documented contract:

  POST /predict   {"frame_id": <str>, "frame": [24 rows of 32 floats, degrees Celsius]}
  response        {"human_detected": bool, "human_score": float, "frame_id": <echo>,
                   "detections": [{"bbox_xyxy_pixels": [left, top, right, bottom], ...}],
                   "warnings": [...], ...}

Box convention of the server: pixel BOUNDARY coordinates, frame[y][x] with (0, 0) the top-left
corner of the top-left pixel, right and bottom EXCLUSIVE (pixel i spans [i, i + 1]). That is this
repo's 'edge' convention. parse_response converts to whichever pixel_convention the localizer and
the viewer use, through msg_utils.corners_to_bbox:
  'edge' : corners are the boundary coordinates as sent
  'index': corners are the indices of the extreme pixels, inclusive (right - 1, bottom - 1)
The server returns at most one box, and only when its CNN says HUMAN.
"""
import json
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from sphere_scout_ai.msg_utils import corners_to_bbox

MAX_RESPONSE_BYTES = 262144
WIDTH, HEIGHT = 32, 24


class ModelError(RuntimeError):
    """The server could not be reached, refused the frame, or answered something unusable."""


@dataclass
class ModelResult:
    human: bool
    score: float
    boxes: List[Tuple[float, float, float, float]] = field(default_factory=list)  # cx, cy, sx, sy
    model_version: str = ''
    frame_id: str = ''
    warnings: List[str] = field(default_factory=list)


def build_request(celsius: np.ndarray, frame_id: str) -> bytes:
    """JSON body for POST /predict from a (24, 32) array in degrees Celsius."""
    arr = np.asarray(celsius, dtype=np.float64)
    if arr.shape != (HEIGHT, WIDTH):
        raise ValueError(f'expected a ({HEIGHT}, {WIDTH}) array, got {arr.shape}')
    if not np.isfinite(arr).all():
        raise ValueError('frame has NaN or infinite values')
    return json.dumps({'frame_id': frame_id, 'frame': np.round(arr, 3).tolist()},
                      allow_nan=False).encode('utf-8')


def _number(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value != value \
            or abs(value) == float('inf'):
        raise ModelError(f'{what} is not a finite number: {value!r}')
    return float(value)


def parse_response(resp, pixel_convention='index') -> ModelResult:
    """Server JSON -> ModelResult, with boxes as (cx, cy, size_x, size_y) in pixel_convention."""
    if not isinstance(resp, dict):
        raise ModelError(f'response is not a JSON object: {type(resp).__name__}')
    if 'human_detected' not in resp or 'human_score' not in resp:
        raise ModelError('response lacks human_detected / human_score')
    if pixel_convention not in ('index', 'edge'):
        raise ValueError(f"pixel_convention must be 'index' or 'edge', got {pixel_convention!r}")
    human = resp['human_detected']
    if not isinstance(human, bool):
        raise ModelError(f'human_detected is not a boolean: {human!r}')
    score = _number(resp['human_score'], 'human_score')
    boxes = []
    for det in (resp.get('detections') or []) if human else []:
        raw = det.get('bbox_xyxy_pixels') if isinstance(det, dict) else None
        if not isinstance(raw, (list, tuple)) or len(raw) != 4:
            raise ModelError(f'bbox_xyxy_pixels is not four numbers: {raw!r}')
        left, top, right, bottom = (_number(v, 'bbox_xyxy_pixels') for v in raw)
        if not (0.0 <= left < right <= WIDTH and 0.0 <= top < bottom <= HEIGHT):
            raise ModelError(f'box outside the {WIDTH} x {HEIGHT} image: {raw!r}')
        if pixel_convention == 'index':
            right, bottom = right - 1.0, bottom - 1.0
        boxes.append(corners_to_bbox(
            [(left, top), (right, top), (right, bottom), (left, bottom)], pixel_convention))
    warnings = [str(w) for w in resp.get('warnings') or []]
    return ModelResult(human=human, score=score, boxes=boxes,
                       model_version=str(resp.get('model_version', '')),
                       frame_id=str(resp.get('frame_id', '')), warnings=warnings)


class ModelClient:
    """Blocking HTTP client. Call it from a worker thread, never from a ROS callback."""

    def __init__(self, base_url, timeout_s=0.5, pixel_convention='index'):
        parts = urllib.parse.urlsplit(base_url)
        if parts.scheme not in ('http', 'https') or not parts.netloc:
            raise ValueError(f'server_url must be an http(s) URL, got {base_url!r}')
        self.base_url = base_url.rstrip('/')
        self.timeout_s = float(timeout_s)
        self.pixel_convention = pixel_convention

    def _call(self, request):
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as r:
                body = r.read(MAX_RESPONSE_BYTES + 1)
        except urllib.error.HTTPError as exc:       # the server answered with an error body
            detail = exc.read(2048).decode('utf-8', 'replace')
            raise ModelError(f'server returned HTTP {exc.code}: {detail}') from None
        except (urllib.error.URLError, OSError) as exc:
            raise ModelError(f'cannot reach {self.base_url}: {getattr(exc, "reason", exc)}') from None
        if len(body) > MAX_RESPONSE_BYTES:
            raise ModelError('response too large')
        try:
            return json.loads(body)
        except ValueError:
            raise ModelError('response is not valid JSON') from None

    def health(self):
        """GET /health -> dict (raises ModelError)."""
        return self._call(urllib.request.Request(self.base_url + '/health'))

    def predict(self, celsius, frame_id) -> ModelResult:
        request = urllib.request.Request(
            self.base_url + '/predict', data=build_request(celsius, frame_id),
            headers={'Content-Type': 'application/json'}, method='POST')
        result = parse_response(self._call(request), self.pixel_convention)
        if result.frame_id != frame_id:
            raise ModelError(f'answer is for frame {result.frame_id!r}, asked for {frame_id!r}')
        return result
