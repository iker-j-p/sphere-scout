"""Rendering for the driver's thermal feed. No ROS imports, so it runs in plain pytest.

The viewer node only does message plumbing. Everything that decides what the driver
sees lives here and is tested:
  decode_image  raw sensor_msgs/Image bytes -> kelvin array
  RangeTracker  temperature window (auto with fast-attack / slow-decay, or fixed)
  render        kelvin + window + detector boxes -> upscaled rgb image with overlays
  format_hud    the facts printed on the image
This module draws what the detector found and facts about the feed. It decides nothing.
"""
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

KELVIN_PER_COUNT = 0.01          # Blueprint: L16, 0.01 K per count
BOX_RGB = (0, 255, 0)
COLORMAPS = {
    'inferno': cv2.COLORMAP_INFERNO,
    'hot': cv2.COLORMAP_HOT,
    'jet': cv2.COLORMAP_JET,
    'bone': cv2.COLORMAP_BONE,
}


# --------------------------------------------------------------------- decoding
def decode_image(data, width, height, step, is_bigendian, encoding,
                 kelvin_per_count=KELVIN_PER_COUNT) -> np.ndarray:
    """sensor_msgs/Image fields -> float32 kelvin array (height, width).

    Honours row padding (step > width * 2) and byte order. No cv_bridge needed.
    """
    if encoding not in ('16UC1', 'mono16'):
        raise ValueError(f'unsupported encoding {encoding!r}, expected 16UC1 or mono16')
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if buf.size < height * step or step < width * 2:
        raise ValueError(f'image data too short: {buf.size} bytes for {height} rows of step {step}')
    rows = buf[:height * step].reshape(height, step)[:, :width * 2]
    counts = np.ascontiguousarray(rows).view('>u2' if is_bigendian else '<u2')
    return counts.reshape(height, width).astype(np.float32) * kelvin_per_count


# ---------------------------------------------------------------- range tracking
class RangeTracker:
    """Temperature window mapped to the colormap.

    Auto mode behaves like an automatic gain control:
      - expands instantly, so a hot target appearing is never clipped;
      - relaxes slowly (EMA), so the picture does not flicker as the scene changes;
      - never narrower than min_span_k, so sensor noise on a flat scene is not
        stretched into false contrast.
    """

    def __init__(self, alpha=0.2, min_span_k=15.0, fixed: Optional[Sequence[float]] = None):
        self.alpha = alpha
        self.min_span_k = min_span_k
        self.fixed = None if fixed is None else (float(fixed[0]), float(fixed[1]))
        self.lo: Optional[float] = None
        self.hi: Optional[float] = None

    @property
    def auto(self) -> bool:
        return self.fixed is None

    def update(self, kelvin: np.ndarray) -> Tuple[float, float]:
        if self.fixed is not None:
            return self.fixed
        lo, hi = float(kelvin.min()), float(kelvin.max())
        if self.lo is None:
            self.lo, self.hi = lo, hi
        else:
            self.lo = lo if lo < self.lo else self.lo + self.alpha * (lo - self.lo)
            self.hi = hi if hi > self.hi else self.hi + self.alpha * (hi - self.hi)
        lo_o, hi_o = self.lo, self.hi
        if hi_o - lo_o < self.min_span_k:
            c = 0.5 * (lo_o + hi_o)
            lo_o, hi_o = c - self.min_span_k / 2.0, c + self.min_span_k / 2.0
        return lo_o, hi_o


# ----------------------------------------------------------------------- drawing
@dataclass
class Box:
    """Detector blob in image pixels: centre (cx, cy) and size (w, h)."""
    cx: float
    cy: float
    w: float
    h: float


def _text(img, s, org, fs, color=(255, 255, 255)):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, fs, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, fs, color, 1, cv2.LINE_AA)


def _colorbar(img, lo, hi, cmap, fs):
    H, W = img.shape[:2]
    bw = max(8, W // 50)
    x1 = W - 6
    x0 = x1 - bw
    y0, y1 = int(H * 0.12), int(H * 0.88)
    grad = np.linspace(255, 0, y1 - y0).astype(np.uint8).reshape(-1, 1)
    bar = cv2.applyColorMap(np.repeat(grad, bw, axis=1), cmap)[:, :, ::-1]
    img[y0:y1, x0:x1] = bar
    cv2.rectangle(img, (x0 - 1, y0 - 1), (x1, y1), (255, 255, 255), 1)
    for label, y in ((f'{hi:.0f} K', y0 + 8), (f'{lo:.0f} K', y1)):
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, 1)[0][0]
        _text(img, label, (x0 - tw - 8, y), fs)


def render(kelvin: np.ndarray, lo: float, hi: float, boxes: Sequence[Box] = (),
           scale: int = 20, colormap: str = 'inferno', interp: str = 'nearest',
           pixel_convention: str = 'index', hud: Sequence[str] = (),
           banner: Optional[str] = None, colorbar: bool = True,
           crosshair: bool = True) -> np.ndarray:
    """kelvin (h, w) -> rgb uint8 (h*scale, w*scale, 3).

    interp 'nearest' keeps the true pixels visible (nothing invented); 'cubic' is smoother.
    pixel_convention must match the detector: 'index' = pixel i centred on i, 'edge' = pixel i
    spans [i, i+1].
    """
    if colormap not in COLORMAPS:
        raise ValueError(f'unknown colormap {colormap!r}')
    if interp not in ('nearest', 'cubic'):
        raise ValueError(f'unknown interp {interp!r}')
    h, w = kelvin.shape
    g = np.clip((kelvin - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    g8 = np.rint(g * 255).astype(np.uint8)
    flag = cv2.INTER_NEAREST if interp == 'nearest' else cv2.INTER_CUBIC
    g8 = cv2.resize(g8, (w * scale, h * scale), interpolation=flag)
    img = np.ascontiguousarray(cv2.applyColorMap(g8, COLORMAPS[colormap])[:, :, ::-1])
    H, W = img.shape[:2]
    fs = 0.5 * scale / 20.0

    off = 0.5 if pixel_convention == 'index' else 0.0
    for b in boxes:
        x0 = int(round((b.cx - b.w / 2 + off) * scale))
        x1 = int(round((b.cx + b.w / 2 + off) * scale)) - 1
        y0 = int(round((b.cy - b.h / 2 + off) * scale))
        y1 = int(round((b.cy + b.h / 2 + off) * scale)) - 1
        cv2.rectangle(img, (x0, y0), (x1, y1), BOX_RGB, 2)
    if crosshair:
        cx, cy, arm = W // 2, H // 2, max(6, scale // 2)
        cv2.line(img, (cx - arm, cy), (cx + arm, cy), (255, 255, 255), 1)
        cv2.line(img, (cx, cy - arm), (cx, cy + arm), (255, 255, 255), 1)
    if colorbar:
        _colorbar(img, lo, hi, COLORMAPS[colormap], fs)
    line_h = int(cv2.getTextSize('Ag', cv2.FONT_HERSHEY_SIMPLEX, fs, 1)[0][1] + 10)
    for i, s in enumerate(reversed(list(hud))):
        _text(img, s, (8, H - 10 - i * line_h), fs)
    if banner:
        band = img.copy()
        cv2.rectangle(band, (0, H // 2 - 3 * line_h), (W, H // 2 + 3 * line_h), (0, 0, 0), -1)
        img = cv2.addWeighted(band, 0.65, img, 0.35, 0)
        big = fs * 2.0
        tw = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, big, 2)[0][0]
        cv2.putText(img, banner, ((W - tw) // 2, H // 2 + line_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, big, (255, 70, 70), 2, cv2.LINE_AA)
    return img


def format_hud(peak_k: float, lo: float, hi: float, auto: bool, age_s: float,
               n_detections: int) -> list:
    """Facts printed on the image. Never instructions."""
    return [
        f'peak {peak_k:.1f} K',
        f'range {lo:.0f} to {hi:.0f} K ({"auto" if auto else "fixed"})',
        f'feed age {age_s:.2f} s',
        f'detections {n_detections}',
    ]
