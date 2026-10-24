"""Run with `python3 test_thermal_render.py` or `pytest`. Needs numpy and cv2 only."""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sphere_scout_ai'))
from thermal_render import (BOX_RGB, COLORMAPS, Box, RangeTracker, decode_image,  # noqa: E402
                            format_hud, render)

W, H, S = 32, 24, 20
RNG = np.random.default_rng(1)


def frame(ambient=295.0, blob=None):
    k = np.full((H, W), ambient, np.float32)
    if blob:
        (c, r), t = blob
        k[r, c] = t
    return k


def lut(name='inferno'):
    return cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1),
                             COLORMAPS[name])[:, 0, ::-1]


def px(img, col, row):
    """Colour at the centre of source pixel (col, row)."""
    return tuple(int(v) for v in img[row * S + S // 2, col * S + S // 2])


PLAIN = dict(hud=(), colorbar=False, crosshair=False)


def test_decode_little_big_endian_and_padding():
    counts = np.full((H, W), 29500, np.uint16)           # 295.00 K
    counts[10, 20] = 31000                                # 310.00 K
    for big in (False, True):
        raw = counts.astype('>u2' if big else '<u2').tobytes()
        k = decode_image(raw, W, H, W * 2, big, '16UC1')
        assert k.shape == (H, W) and abs(k[0, 0] - 295.0) < 1e-3 and abs(k[10, 20] - 310.0) < 1e-3
    pad = 6                                               # row padding: step > width * 2
    rows = [counts[r].astype('<u2').tobytes() + b'\xff' * pad for r in range(H)]
    k = decode_image(b''.join(rows), W, H, W * 2 + pad, False, 'mono16')
    assert abs(k[10, 20] - 310.0) < 1e-3 and abs(k[23, 31] - 295.0) < 1e-3


def test_decode_rejects_bad_input():
    good = np.zeros((H, W), '<u2').tobytes()
    for args in ((good, W, H, W * 2, False, 'rgb8'), (good[:-10], W, H, W * 2, False, '16UC1')):
        try:
            decode_image(*args)
        except ValueError:
            continue
        raise AssertionError('expected ValueError')


def test_range_min_span_and_fixed():
    t = RangeTracker(alpha=0.2, min_span_k=15.0)
    lo, hi = t.update(frame())
    assert abs((hi - lo) - 15.0) < 1e-9 and abs((lo + hi) / 2 - 295.0) < 1e-9
    f = RangeTracker(fixed=(285, 320))
    assert f.update(frame()) == (285.0, 320.0) and not f.auto


def test_range_fast_attack_slow_decay():
    t = RangeTracker(alpha=0.2, min_span_k=5.0)
    for _ in range(5):
        t.update(frame(295.0))
    _, hi = t.update(frame(295.0, ((3, 3), 320.0)))
    assert hi == 320.0                                    # hot target appears: never clipped
    _, hi1 = t.update(frame(295.0))
    assert 320.0 > hi1 > 305.0                            # target gone: relaxes gradually
    for _ in range(40):
        _, hi = t.update(frame(295.0))
    assert hi < 300.0                                     # and eventually settles


def test_render_shape_dtype_and_exact_colours():
    k = frame(295.0, ((20, 10), 310.0))
    img = render(k, 295.0, 310.0, **PLAIN)
    assert img.shape == (H * S, W * S, 3) and img.dtype == np.uint8
    assert px(img, 20, 10) == tuple(lut()[255]) and px(img, 5, 5) == tuple(lut()[0])
    # nearest upscale keeps true pixels: every 20x20 block is one flat colour
    blk = img[200:220, 400:420].reshape(-1, 3)
    assert (blk == blk[0]).all()
    # cubic is allowed to blend, so it must differ at the block edge
    cub = render(k, 295.0, 310.0, interp='cubic', **PLAIN)
    assert not np.array_equal(img, cub)


def test_box_overlay_position_and_conventions():
    k = frame(295.0, ((20, 10), 310.0))
    box = Box(cx=20, cy=10, w=1, h=1)                     # index convention: spans x 400..419
    img = render(k, 295.0, 310.0, boxes=[box], pixel_convention='index', **PLAIN)
    on_left_edge = [tuple(img[210, x]) for x in (399, 400, 401)]
    assert BOX_RGB in on_left_edge
    assert px(img, 20, 10) == tuple(lut()[255])           # blob still visible inside the box
    edge = render(k, 295.0, 310.0, boxes=[box], pixel_convention='edge', **PLAIN)
    assert BOX_RGB in [tuple(edge[210, x]) for x in (389, 390, 391)]   # shifted half a pixel
    assert not np.array_equal(img, edge)


def test_flat_noisy_scene_is_not_stretched():
    k = (295.0 + RNG.normal(0, 0.1, (H, W))).astype(np.float32)
    lo, hi = RangeTracker(min_span_k=15.0).update(k)
    def noise(img):   # per-channel std across pixels, so colormap channel offsets do not count
        return img[::S, ::S].astype(float).reshape(-1, 3).std(axis=0).mean()
    calm = noise(render(k, lo, hi, **PLAIN))
    stretched = noise(render(k, float(k.min()), float(k.max()), **PLAIN))
    print(f'noise std in displayed colours: with min span {calm:.1f}, without {stretched:.1f}')
    assert calm < 0.3 * stretched


def test_overlays_change_pixels_and_hud_strings():
    k = frame(295.0, ((20, 10), 310.0))
    base = render(k, 295.0, 310.0, **PLAIN)
    hud = format_hud(310.0, 295.0, 310.0, True, 0.12, 1)
    full = render(k, 295.0, 310.0, hud=hud, banner='NO FEED 3.2 s')
    assert full.shape == base.shape and not np.array_equal(full, base)
    assert any('feed age 0.12 s' in s for s in hud) and any('(auto)' in s for s in hud)
    assert not any(w in ' '.join(hud).lower() for w in ('move', 'drive', 'turn', 'should'))


def test_bad_options_raise():
    for kw in ({'colormap': 'nope'}, {'interp': 'nope'}):
        try:
            render(frame(), 285, 300, **kw)
        except ValueError:
            continue
        raise AssertionError('expected ValueError')


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
