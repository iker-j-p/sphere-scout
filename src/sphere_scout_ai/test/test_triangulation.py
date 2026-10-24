"""Run with `python3 test_triangulation.py` or `pytest`. Needs only numpy."""
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sphere_scout_ai'))
from triangulation import (Gates, HeightPrior, ViewRay, ViewTracker, bearing_deg,  # noqa: E402
                           estimate, intrinsics, pixel_to_ray_cam, plane_point,
                           quat_to_rot, rot_to_quat, rotation_angle_deg,
                           size_prior_range, triangulate, world_to_odom)

RNG = np.random.default_rng(0)
CHI2_3_95 = 7.815  # chi-square, 3 dof, 95%


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def perturb(d, sigma):
    """Rotate unit vector d by a random small angle (sigma per transverse axis)."""
    a = np.cross(d, [0, 0, 1.0])
    if np.linalg.norm(a) < 1e-6:
        a = np.cross(d, [0, 1.0, 0])
    e1 = unit(a)
    e2 = np.cross(d, e1)
    return unit(d + RNG.normal(0, sigma) * e1 + RNG.normal(0, sigma) * e2)


def mk(o, d, t=0.0, sigma=0.01):
    return ViewRay(np.array(o, float), unit(d), sigma, 10, 4.0, t)


def test_noise_free_recovery():
    target = np.array([4.0, 0.3, 0.4])
    O = np.array([[0, 0, 0.1], [0, 0.6, 0.1], [0.3, -0.5, 0.1]])
    D = np.array([unit(target - o) for o in O])
    x, cov, t, chi2_i, dof = triangulate(O, D, [0.01] * 3)
    assert np.allclose(x, target, atol=1e-6), x
    assert chi2_i.sum() < 1e-9 and dof == 3 and np.all(t > 0)


def test_predicted_sigma_matches_monte_carlo():
    target = np.array([4.0, 0.0, 0.1])
    O = np.array([[0, -0.25, 0.1], [0, 0.25, 0.1]])  # 0.5 m baseline, perpendicular to view
    s = 0.01
    est, preds = [], []
    for _ in range(3000):
        D = np.array([perturb(unit(target - o), s) for o in O])
        x, cov, *_ = triangulate(O, D, [s, s])
        est.append(x)
        preds.append(math.sqrt(cov[0, 0]))   # cov depends on the noisy range estimate: average it
    emp_depth = np.array(est)[:, 0].std()
    pred_depth = float(np.mean(preds))
    closed_form = math.sqrt(2) * 4.0 ** 2 * s / 0.5
    print(f'depth sigma: empirical {emp_depth:.3f}, covariance {pred_depth:.3f}, '
          f'closed form {closed_form:.3f}')
    assert abs(emp_depth - pred_depth) / pred_depth < 0.15
    assert abs(pred_depth - closed_form) / closed_form < 0.15


def test_gates_and_statuses():
    g = Gates()
    assert estimate([], g).status == 'NO_TARGET'
    assert estimate([mk([0, 0, 0], [1, 0, 0])], g).status == 'BEARING_ONLY'
    # parallel rays 0.6 m apart: baseline ok, parallax 0
    sol = estimate([mk([0, 0, 0], [1, 0, 0]), mk([0, 0.6, 0], [1, 0, 0])], g)
    assert sol.status == 'BEARING_ONLY' and 'parallax' in sol.reason
    # intersection behind the cameras
    sol = estimate([mk([0, 0, 0], [1, -0.2, 0]), mk([0, 1, 0], [1, 0.2, 0])], g)
    assert sol.status == 'BEARING_ONLY' and 'behind' in sol.reason, sol.reason
    # good geometry, tight
    tgt = np.array([2.0, 0.3, 0.2])
    rays = [mk(o, tgt - o) for o in ([0, -0.4, 0.1], [0, 0.5, 0.1])]
    assert estimate(rays, g).status == 'CONFIRMED'
    # far target, same baseline: valid point, depth too uncertain, still reported
    tgt = np.array([9.0, 0.3, 0.2])
    rays = [mk(o, tgt - o) for o in ([0, -0.25, 0.1], [0, 0.25, 0.1])]
    sol = estimate(rays, g)
    assert sol.status == 'LOW_CONFIDENCE' and sol.point is not None
    # KNOWN LIMITATION: two survivors, one seen per view. With 2 views the coplanar rays
    # always intersect, so nothing can flag it. With a third view the rays disagree:
    a, b = np.array([3.0, 1.5, 0.2]), np.array([3.0, -1.5, 0.2])
    o1, o2, o3 = np.array([0, -0.3, 0.1]), np.array([0, 0.3, 0.1]), np.array([0, 0.9, 0.1])
    rays = [mk(o1, a - o1), mk(o2, b - o2), mk(o3, a - o3)]
    assert estimate(rays, g).status == 'BEARING_ONLY'


def test_robust_rejection_of_one_bad_view():
    tgt = np.array([3.0, 0.2, 0.3])
    origins = [[0, y, 0.1] for y in (-0.6, -0.3, 0.0, 0.3, 0.6, 0.9)]
    rays = [mk(o, tgt - o, t=i) for i, o in enumerate(origins)]
    rays[2] = mk(origins[2], [3.0, 2.0, 0.2], t=2)     # one view saw something else
    sol = estimate(rays, Gates())
    assert sol.status in ('CONFIRMED', 'LOW_CONFIDENCE'), (sol.status, sol.reason)
    assert sol.n_rejected == 1 and sol.used[2] is False
    assert np.linalg.norm(sol.point - tgt) < 0.15, sol.point


def test_one_view_height_prior():
    tgt = np.array([2.0, 0.3, 0.25])
    o = np.array([0, 0, 0.5])
    ray = mk(o, tgt - o)
    g = Gates()
    assert estimate([ray], g).status == 'BEARING_ONLY'              # no prior: bearing only
    sol = estimate([ray], g, HeightPrior(0.25, 0.05))
    assert sol.status == 'PROVISIONAL' and sol.reason == 'single view'
    assert np.allclose(sol.point, tgt, atol=1e-6)
    # level bearing: prior refuses instead of returning a point at infinity
    sol = estimate([mk([0, 0, 0.25], [1, 0, 0])], g, HeightPrior(0.25, 0.05))
    assert sol.status == 'BEARING_ONLY' and 'unusable' in sol.reason


def test_plane_point_covariance_matches_monte_carlo():
    tgt = np.array([2.5, 0.4, 0.2])
    o = np.array([0, 0, 0.6])
    d0 = unit(tgt - o)
    s, sh = 0.01, 0.05
    pts = []
    for _ in range(4000):
        d = perturb(d0, s)
        p, _ = plane_point(o, d, 0.2 + RNG.normal(0, sh), s, sh)
        pts.append(p)
    emp = np.cov(np.array(pts).T)
    _, pred = plane_point(o, d0, 0.2, s, sh)
    for k in range(3):
        e, p = math.sqrt(emp[k, k]), math.sqrt(pred[k, k])
        print(f'plane axis {k}: empirical {e:.3f}, predicted {p:.3f}')
        assert abs(e - p) / p < 0.2


def test_reasons_never_give_advice():
    g = Gates()
    bad = re.compile(r'move|rescan|widen|try|should|recommend|suggest', re.I)
    cases = [
        [mk([0, 0, 0], [1, 0, 0])],
        [mk([0, 0, 0], [1, 0, 0]), mk([0, 0.6, 0], [1, 0, 0])],
        [mk([0, 0, 0], [1, -0.2, 0]), mk([0, 1, 0], [1, 0.2, 0])],
        [mk(o, np.array([9.0, 0.3, 0.2]) - o) for o in ([0, -0.25, 0.1], [0, 0.25, 0.1])],
        [mk([0, 0, 0], [1, 0, 0]), mk([0, 0.1, 0], [1, 0, 0])],
    ]
    for rays in cases:
        sol = estimate(rays, g, HeightPrior(0.2, 0.05))
        assert not bad.search(sol.reason), sol.reason


def test_view_grouping_and_outliers():
    tr = ViewTracker(radius_m=0.15)
    d0 = unit([1, 0.05, 0])
    for i in range(12):
        tr.add_ray([0, 0, 0.1 + 0.001 * i], perturb(d0, 0.003), 4.0, t=i)
    tr.add_ray([0, 0, 0.1], unit([1, -0.6, 0]), 4.0, t=12)         # outlier frame
    for i in range(8):
        tr.add_ray([0, 0.6, 0.1], perturb(unit([1, -0.1, 0]), 0.003), 4.0, t=20 + i)
    assert len(tr.views) == 2
    rays = tr.rays(gate_rad=math.radians(3), min_frames=5, sigma_theta=0.01)
    assert len(rays) == 2 and rays[0].n == 12 and rays[1].t_last == 27
    assert math.degrees(math.acos(rays[0].direction @ d0)) < 0.5
    tr2 = ViewTracker()
    for _ in range(3):
        tr2.add_ray([0, 0, 0], d0, 4.0)
    assert tr2.rays(math.radians(3), 5, 0.01) == []


def test_stationary_robot_is_never_split_into_two_views():
    tr = ViewTracker(radius_m=0.15)
    for i in range(200):                                  # jitter around a grid-like boundary
        tr.add_ray([0.15 + RNG.normal(0, 0.003), 0.0, 0.1], unit([1, 0, 0]), 4.0, t=i)
    assert len(tr.views) == 1


def test_eviction_keeps_newest_views():
    tr = ViewTracker(radius_m=0.1, max_views=5)
    for i in range(12):
        tr.add_ray([0.0, 0.3 * i, 0.1], unit([1, 0, 0]), 4.0, t=i)
    assert len(tr.views) == 5
    assert min(v.t_last for v in tr.views) == 7


def test_continuous_drive_end_to_end_and_coverage():
    """Robot drives sideways past a static survivor at 0.2 m/s, detector runs at 8 Hz.
    Error model: per-view common-mode error (the part that does not average) plus a
    small per-frame error. Reported 2-sigma ellipsoid should cover the truth ~95%."""
    tgt = np.array([4.0, 0.3, 0.3])
    g = Gates(max_sigma_view_m=1.0)
    hits, sig_early, sig_late, n_ok = 0, [], [], 0
    trials = 150
    for _ in range(trials):
        tr = ViewTracker(radius_m=0.15)
        common, anchors = {}, []
        for k in range(8 * 8):                            # 8 s of driving, 1.6 m
            t = k / 8.0
            o = np.array([0.0, -0.8 + 0.2 * t, 0.15])
            dists = [np.linalg.norm(a - o) for a in anchors]   # mirror the tracker's clustering
            if not dists or min(dists) > 0.15:
                anchors.append(o)
                vid = len(anchors) - 1
            else:
                vid = int(np.argmin(dists))
            if vid not in common:
                common[vid] = RNG.normal(0, 0.01, 2)
            d0 = unit(tgt - o)
            e1 = unit(np.cross(d0, [0, 0, 1.0]))
            e2 = np.cross(d0, e1)
            d = unit(d0 + common[vid][0] * e1 + common[vid][1] * e2
                     + RNG.normal(0, 0.003) * e1 + RNG.normal(0, 0.003) * e2)
            tr.add_ray(o, d, 4.0, t)
            if k in (31, 63):
                sol = estimate(tr.rays(math.radians(4), 4, 0.01), g)
                if sol.point is not None:
                    (sig_early if k == 31 else sig_late).append(sol.sigma_view)
        sol = estimate(tr.rays(math.radians(4), 4, 0.01), g)
        if sol.point is None:
            continue
        n_ok += 1
        err = sol.point - tgt
        m2 = float(err @ np.linalg.inv(sol.cov) @ err)
        hits += m2 <= CHI2_3_95
    cov = hits / n_ok
    print(f'continuous drive: {n_ok}/{trials} solved, 95% ellipsoid coverage {cov:.2f}, '
          f'median depth sigma after 4 s {np.median(sig_early):.2f} m, after 8 s '
          f'{np.median(sig_late):.2f} m')
    assert n_ok > 0.9 * trials
    assert 0.85 <= cov <= 1.0
    assert np.median(sig_late) < np.median(sig_early)


def test_camera_model():
    fx, fy, cx, cy = intrinsics(32, 24, 0.96)
    assert abs(fx - 30.73) < 0.01 and fx == fy and cx == 15.5 and cy == 11.5
    fx2, _, cx2, cy2 = intrinsics(32, 24, 0.96, pixel_convention='edge')
    assert cx2 == 16.0 and cy2 == 12.0
    d = pixel_to_ray_cam(cx, cy, fx, fy, cx, cy)
    assert np.allclose(d, [1, 0, 0])
    d = pixel_to_ray_cam(-0.5, cy, fx, fy, cx, cy)       # left image edge = hfov/2 to the left
    assert abs(math.atan2(d[1], d[0]) - 0.48) < 1e-9 and d[1] > 0
    assert pixel_to_ray_cam(cx, cy + 5, fx, fy, cx, cy)[2] < 0
    do = pixel_to_ray_cam(-0.5, cy, fx, fy, cx, cy, 'optical')
    assert do[0] < 0 and do[2] > 0


def test_quaternions_and_rotation_angle():
    for _ in range(200):
        q = RNG.normal(size=4)
        q /= np.linalg.norm(q)
        R = quat_to_rot(*q)
        assert np.allclose(R @ R.T, np.eye(3)) and abs(np.linalg.det(R) - 1) < 1e-9
        q2 = np.array(rot_to_quat(R))
        assert min(np.abs(q - q2).max(), np.abs(q + q2).max()) < 1e-9
    Rz = quat_to_rot(0, 0, math.sin(math.radians(5)), math.cos(math.radians(5)))  # 10 deg yaw
    assert abs(rotation_angle_deg(np.eye(3), Rz) - 10.0) < 1e-9


def test_bearing_and_world_to_odom():
    az, el = bearing_deg(unit([1, 1, 0]))
    assert abs(az - 45) < 1e-9 and abs(el) < 1e-9          # +y is left of the +x entry axis
    p = world_to_odom([1.0, 3.0, 0.1], (1.0, 2.0, 0.1, math.pi / 2))
    assert np.allclose(p, [1.0, 0.0, 0.0], atol=1e-9)


def test_size_prior():
    fx = intrinsics(32, 24, 0.96)[0]
    lo, hi = size_prior_range(fx * 0.5 / 3.0, fx, 0.3, 1.7)  # 0.5 m target at 3 m
    print(f'size prior for 3 m target: {lo:.1f} to {hi:.1f} m')
    assert lo < 3.0 < hi


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
