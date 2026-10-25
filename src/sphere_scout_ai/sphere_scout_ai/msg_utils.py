"""Small message helpers shared by target_localizer_node and thermal_viewer_node.

No ROS imports, so it runs in plain pytest with fake objects.
"""


def corners_to_bbox(corners, pixel_convention='index'):
    """Four (x, y) corner points in image pixels -> (cx, cy, size_x, size_y).

    Use this where the model's output enters the system (thermal_detector_node), so the
    published vision_msgs BoundingBox2D stays the same: centre + size, in pixels of the image.
    Corner order does not matter. A tilted quadrilateral is reduced to its axis-aligned
    bounding box (rotation is ignored downstream).

    pixel_convention must match the localizer and the viewer:
      'index': corners are the pixel INDICES of the extreme pixels, inclusive
               (pixel columns 5..9 -> size 5, centre 7)
      'edge' : corners are continuous boundary coordinates, pixel i spans [i, i+1]
               (boundaries 5 and 10 -> size 5, centre 7.5)
    Raises ValueError on a bad corner count, non-finite values or an empty box.
    """
    pts = [(float(x), float(y)) for x, y in corners]
    if len(pts) != 4:
        raise ValueError(f'expected 4 corners, got {len(pts)}')
    if pixel_convention not in ('index', 'edge'):
        raise ValueError(f"pixel_convention must be 'index' or 'edge', got {pixel_convention!r}")
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    if not all(v == v and abs(v) != float('inf') for v in xs + ys):
        raise ValueError('corners must be finite')
    pad = 1.0 if pixel_convention == 'index' else 0.0
    sx, sy = max(xs) - min(xs) + pad, max(ys) - min(ys) + pad
    if sx <= 0.0 or sy <= 0.0:
        raise ValueError('corners enclose no area')
    return (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0, sx, sy


def bbox_center(bbox):
    """(x, y) of a vision_msgs BoundingBox2D centre, in pixels.

    Handles both layouts: `bbox.center.position.x/y` (Pose2D, verified on this Jazzy install)
    and `bbox.center.x/y` (older Point2D layout).
    """
    c = bbox.center
    pos = getattr(c, 'position', None)
    if pos is not None:
        return float(pos.x), float(pos.y)
    return float(c.x), float(c.y)
