"""Run with `python3 test_msg_utils.py` or `pytest`. Fake objects, plus the real message if available."""
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sphere_scout_ai'))
from msg_utils import bbox_center, corners_to_bbox  # noqa: E402
from triangulation import intrinsics, pixel_to_ray_cam  # noqa: E402

import numpy as np  # noqa: E402


def test_pose2d_layout():
    bbox = NS(center=NS(position=NS(x=12.5, y=7.0), theta=0.0), size_x=3.0, size_y=2.0)
    assert bbox_center(bbox) == (12.5, 7.0)


def test_point2d_layout():
    bbox = NS(center=NS(x=4.0, y=9.5), size_x=3.0, size_y=2.0)
    assert bbox_center(bbox) == (4.0, 9.5)


def test_corners_index_and_edge():
    sq = [(5, 3), (9, 3), (9, 7), (5, 7)]                    # pixel indices, inclusive
    assert corners_to_bbox(sq, 'index') == (7.0, 5.0, 5.0, 5.0)
    sq_edge = [(5, 3), (10, 3), (10, 8), (5, 8)]            # same blob as boundaries
    assert corners_to_bbox(sq_edge, 'edge') == (7.5, 5.5, 5.0, 5.0)


def test_corners_order_and_tilt():
    a = corners_to_bbox([(9, 7), (5, 3), (5, 7), (9, 3)], 'index')
    assert a == corners_to_bbox([(5, 3), (9, 3), (9, 7), (5, 7)], 'index')
    cx, cy, sx, sy = corners_to_bbox([(7, 2), (11, 5), (7, 8), (3, 5)], 'edge')   # diamond -> hull
    assert (cx, cy, sx, sy) == (7.0, 5.0, 8.0, 6.0)


def test_corners_same_blob_gives_same_ray_in_both_conventions():
    """The same physical blob must give the same bearing whichever convention is used,
    provided each side uses the matching intrinsics."""
    rays = []
    for conv, sq in (('index', [(20, 10), (24, 10), (24, 14), (20, 14)]),
                     ('edge', [(20, 10), (25, 10), (25, 15), (20, 15)])):
        cx, cy, *_ = corners_to_bbox(sq, conv)
        fx, fy, px, py = intrinsics(32, 24, 0.96, pixel_convention=conv)
        rays.append(pixel_to_ray_cam(cx, cy, fx, fy, px, py))
    assert np.allclose(rays[0], rays[1], atol=1e-12)


def test_corners_bad_input():
    bad = [([(0, 0), (1, 1), (2, 2)], 'index'),                       # 3 corners
           ([(0, 0), (1, 0), (1, 1), (float('nan'), 1)], 'index'),    # non-finite
           ([(3, 3)] * 4, 'edge'),                                    # no area
           ([(0, 0), (1, 0), (1, 1), (0, 1)], 'bogus')]               # unknown convention
    for corners, conv in bad:
        try:
            corners_to_bbox(corners, conv)
        except ValueError:
            continue
        raise AssertionError(f'expected ValueError for {corners} {conv}')


def test_real_message_if_available():
    try:
        from vision_msgs.msg import BoundingBox2D
    except ImportError:
        return
    b = BoundingBox2D()
    b.center.position.x, b.center.position.y = 15.5, 11.5
    assert bbox_center(b) == (15.5, 11.5)


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
