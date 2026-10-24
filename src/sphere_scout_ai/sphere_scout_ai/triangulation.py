"""Geometry core for target_localizer_node. No ROS imports, so it runs in plain pytest.

Continuous passive estimator. Inputs are only rays (camera origin + direction in odom).
Nothing here commands, recommends or schedules anything.

Pipeline: pixel -> camera ray -> (TF) odom ray -> per-VIEW collapsed ray
          -> multi-view weighted least squares with robust view rejection
          -> point + covariance + descriptive status.
A VIEW is a cluster of frames whose camera origins lie within view_radius of an anchor.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

EYE3 = np.eye(3)


# ---------------------------------------------------------------- camera model
def intrinsics(width, height, hfov, vfov=0.0, pixel_convention='index'):
    """Return fx, fy, cx, cy in pixels.

    vfov <= 0 means square pixels (fy = fx).
    pixel_convention:
      'index': pixel i has its centre at coordinate i (cv2 centroids), so cx = (W-1)/2
      'edge' : pixel i spans [i, i+1] (bbox from stats left+width/2), so cx = W/2
    """
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    fy = fx if vfov <= 0.0 else (height / 2.0) / math.tan(vfov / 2.0)
    off = 0.5 if pixel_convention == 'index' else 0.0
    return fx, fy, width / 2.0 - off, height / 2.0 - off


def pixel_to_ray_cam(u, v, fx, fy, cx, cy, convention='gz'):
    """Unit ray in the camera frame.

    'gz'     : x forward, y left, z up (Gazebo sensor frame). Image u right, v down.
    'optical': z forward, x right, y down.
    """
    a = (u - cx) / fx
    b = (v - cy) / fy
    if convention == 'optical':
        d = np.array([a, b, 1.0])
    elif convention == 'gz':
        d = np.array([1.0, -a, -b])
    else:
        raise ValueError(f'unknown camera convention {convention!r}')
    return d / np.linalg.norm(d)


# ------------------------------------------------------------------ rotations
def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s,
                (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
            0.25 * s, (R[1, 0] - R[0, 1]) / s)


def rotation_angle_deg(Ra, Rb):
    """Angle between two orientations. Used by the optional motion gate."""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def world_to_odom(p, spawn):
    """World-frame point -> odom frame. spawn = (x, y, z, yaw) of base_link at spawn.

    odom origin is the spawn pose, so this is a translation and a yaw rotation.
    """
    x0, y0, z0, yaw = spawn
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p[0] - x0, p[1] - y0
    return np.array([c * dx + s * dy, -s * dx + c * dy, p[2] - z0])


def bearing_deg(d):
    """(azimuth, elevation) in odom, degrees. Azimuth 0 = +x (into the structure), + = left."""
    return (math.degrees(math.atan2(d[1], d[0])),
            math.degrees(math.asin(float(np.clip(d[2], -1.0, 1.0)))))


# ------------------------------------------------------------------ view logic
@dataclass
class ViewRay:
    origin: np.ndarray
    direction: np.ndarray
    sigma: float          # angular sigma, rad
    n: int                # inlier frames behind this ray
    extent_px: float      # median blob extent, for the size prior
    t_last: float = 0.0   # time of the newest frame, for "latest view" and ageing


class View:
    """All frames collected while the camera origin stayed within the view radius."""

    def __init__(self, anchor, max_frames=300):
        self.anchor = np.asarray(anchor, float)
        self.max_frames = max_frames
        self.origins, self.dirs, self.extents = [], [], []
        self.t_last = 0.0

    def add(self, origin, direction, extent_px, t=0.0):
        self.origins.append(np.asarray(origin, float))
        self.dirs.append(np.asarray(direction, float))
        self.extents.append(float(extent_px))
        self.t_last = max(self.t_last, t)
        if len(self.dirs) > self.max_frames:
            for lst in (self.origins, self.dirs, self.extents):
                lst.pop(0)

    def summarize(self, gate_rad, min_frames, sigma_theta):
        """Collapse the view to ONE ray. Frames from one spot share pose error and blob
        bias, so the ray keeps sigma_theta (or the measured scatter if larger), never
        sigma/sqrt(n). Repeated frames buy outlier rejection, not range."""
        if len(self.dirs) < min_frames:
            return None
        D = np.array(self.dirs)
        O = np.array(self.origins)
        m = np.median(D, axis=0)
        m /= np.linalg.norm(m)
        keep = np.ones(len(D), bool)
        for _ in range(2):
            ang = np.arccos(np.clip(D @ m, -1.0, 1.0))
            keep = ang <= gate_rad
            if keep.sum() < min_frames:
                return None
            m = D[keep].mean(axis=0)
            m /= np.linalg.norm(m)
        ang = np.arccos(np.clip(D[keep] @ m, -1.0, 1.0))
        scatter = float(math.sqrt(np.mean(ang ** 2)))
        ext = float(np.median(np.array(self.extents)[keep]))
        return ViewRay(O[keep].mean(axis=0), m, max(sigma_theta, scatter),
                       int(keep.sum()), ext, self.t_last)


class ViewTracker:
    """Clusters the continuous frame stream into views by camera position alone.

    Anchor-based, not grid-based: a stationary robot can never be split into two
    near-identical views by a cell boundary (that would double count its information).
    """

    def __init__(self, radius_m=0.15, max_views=200):
        self.radius = radius_m
        self.max_views = max_views
        self.views: List[View] = []

    def add_ray(self, origin, direction, extent_px, t=0.0):
        origin = np.asarray(origin, float)
        best, best_d = None, float('inf')
        for v in self.views:
            d = float(np.linalg.norm(v.anchor - origin))
            if d < best_d:
                best, best_d = v, d
        if best is None or best_d > self.radius:
            best = View(origin)
            self.views.append(best)
        best.add(origin, direction, extent_px, t)
        if len(self.views) > self.max_views:
            # drop the least recently updated view (never the one just updated)
            others = [v for v in self.views if v is not best]
            self.views.remove(min(others, key=lambda v: v.t_last))

    def rays(self, gate_rad, min_frames, sigma_theta):
        out = [v.summarize(gate_rad, min_frames, sigma_theta) for v in self.views]
        return [r for r in out if r is not None]

    def clear(self):
        self.views.clear()


# --------------------------------------------------------------- triangulation
def pairwise_geometry(O, D):
    """Largest angle between any two rays (deg) and largest origin separation (m)."""
    O, D = np.asarray(O, float), np.asarray(D, float)
    par = math.degrees(math.acos(float(np.clip(np.min(D @ D.T), -1.0, 1.0))))
    base = float(np.max(np.linalg.norm(O[:, None, :] - O[None, :, :], axis=2)))
    return par, base


def triangulate(O, D, S, iters=4, r_min=0.2):
    """Weighted least-squares intersection of rays.

    Each ray contributes information (I - d d^T) / (r * sigma)^2: transverse position
    error grows linearly with range r. r is re-estimated each iteration.
    Returns x, cov (3x3), t (signed range along each ray), chi2_i (per ray), dof.
    Raises numpy.linalg.LinAlgError if the geometry is singular.
    """
    O, D, S = np.asarray(O, float), np.asarray(D, float), np.asarray(S, float)
    n = len(D)
    P = EYE3 - np.einsum('ni,nj->nij', D, D)

    def info(r_vec):
        w = 1.0 / (r_vec * S) ** 2
        return (np.einsum('n,nij->ij', w, P),
                np.einsum('n,nij,nj->i', w, P, O))

    r = np.ones(n)
    x = None
    for _ in range(iters):
        A, b = info(r)
        x = np.linalg.solve(A, b)
        r = np.maximum(np.linalg.norm(x - O, axis=1), r_min)
    A, _ = info(r)
    cov = np.linalg.inv(A)
    t = np.einsum('ij,ij->i', D, x - O)
    res = np.einsum('nij,nj->ni', P, x - O)
    chi2_i = np.sum(res ** 2, axis=1) / (r * S) ** 2
    return x, cov, t, chi2_i, 2 * n - 3


# ------------------------------------------------------------- one-view prior
@dataclass
class HeightPrior:
    """Optional. Assumes the heat centroid lies on the plane z = height_m (odom frame).
    It is a prior, labelled as such, and never mixes into the triangulated solution."""
    height_m: float
    sigma_m: float
    min_elev_deg: float = 2.0     # guard: a near-level ray meets the plane very far away


def plane_point(origin, direction, height_m, sigma_theta, sigma_h, min_elev_deg=2.0):
    """Intersect a ray with the plane z = height_m. Returns (point, cov) or None.

    Covariance is propagated numerically from azimuth, elevation and height errors.
    """
    o = np.asarray(origin, float)
    d = np.asarray(direction, float)
    if abs(d[2]) < math.sin(math.radians(min_elev_deg)):
        return None
    if (height_m - o[2]) / d[2] <= 0:
        return None
    az, el = math.atan2(d[1], d[0]), math.asin(d[2])

    def pt(a, e, h):
        dd = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
        return o + dd * ((h - o[2]) / dd[2])

    eps = 1e-5
    p0 = np.array([az, el, height_m])
    J = np.zeros((3, 3))
    for k in range(3):
        dp = np.zeros(3)
        dp[k] = eps
        J[:, k] = (pt(*(p0 + dp)) - pt(*(p0 - dp))) / (2 * eps)
    sig = np.diag([(sigma_theta / math.cos(el)) ** 2, sigma_theta ** 2, sigma_h ** 2])
    return pt(*p0), J @ sig @ J.T


# --------------------------------------------------------------------- solving
@dataclass
class Gates:
    min_baseline_m: float = 0.5
    min_parallax_deg: float = 1.0     # singularity guard only; the real gate is max_sigma_view_m
    max_chi2_per_dof: float = 9.0
    max_sigma_view_m: float = 0.5     # CONFIRMED vs LOW_CONFIDENCE
    max_reject_frac: float = 0.5      # at most this share of views may be rejected as outliers
    inlier_sigma: float = 3.0         # a view is an inlier if its angular residual <= k * sigma
    ransac_pairs: int = 100           # candidate ray pairs tried when selecting the consensus


@dataclass
class Solution:
    # NO_TARGET | BEARING_ONLY | PROVISIONAL | LOW_CONFIDENCE | CONFIRMED
    status: str = 'NO_TARGET'
    reason: str = ''                  # descriptive facts only, never advice
    point: Optional[np.ndarray] = None
    cov: Optional[np.ndarray] = None
    sigma_view: float = float('nan')
    chi2_per_dof: float = float('nan')
    parallax_deg: float = 0.0
    baseline_m: float = 0.0
    n_views: int = 0
    n_used: int = 0
    n_rejected: int = 0
    azimuth_deg: float = float('nan')
    elevation_deg: float = float('nan')
    ref_ray: Optional[ViewRay] = None  # latest view, drawn when there is no point
    rays: List[ViewRay] = field(default_factory=list)
    used: List[bool] = field(default_factory=list)


def _geometry(O, D, g) -> Tuple[str, float, float]:
    par, base = pairwise_geometry(O, D)
    if base < g.min_baseline_m:
        return f'baseline {base:.2f} m < {g.min_baseline_m:.2f} m', par, base
    if par < g.min_parallax_deg:
        return f'parallax {par:.1f} deg < {g.min_parallax_deg:.1f} deg', par, base
    return '', par, base


def _pair_point(oi, di, oj, dj):
    """Closest-approach midpoint of two rays, or None if near-parallel or behind."""
    w0 = oi - oj
    b = float(di @ dj)
    d, e = float(di @ w0), float(dj @ w0)
    den = 1.0 - b * b
    if den < 1e-6:
        return None
    s_ = (b * e - d) / den
    t_ = (e - b * d) / den
    if s_ <= 0 or t_ <= 0:
        return None
    return 0.5 * ((oi + s_ * di) + (oj + t_ * dj))


def _residual_angles(O, D, x):
    """Angle (rad) between each ray and the direction from its origin to x. pi if behind."""
    v = x - O
    u = v / np.linalg.norm(v, axis=1, keepdims=True)
    c = np.einsum('ij,ij->i', D, u)
    ang = np.arccos(np.clip(c, -1.0, 1.0))
    ang[c <= 0] = math.pi
    return ang


def _consensus(O, D, S, g):
    """RANSAC over ray pairs. Returns the largest inlier mask found, or None.

    A plain 'drop the worst contributor' loop fails here: one gross outlier drags the
    least-squares point toward the cameras and every view then looks bad."""
    n = len(D)
    if n * (n - 1) // 2 <= g.ransac_pairs:
        pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]
    else:
        rng = np.random.default_rng(0)          # fixed seed: same input, same output
        pairs = [(int(i), int(j)) for i, j in
                 zip(rng.integers(0, n, g.ransac_pairs), rng.integers(0, n, g.ransac_pairs))
                 if i != j]
    best, best_k = None, 0
    for i, j in pairs:
        x = _pair_point(O[i], D[i], O[j], D[j])
        if x is None:
            continue
        inl = _residual_angles(O, D, x) <= g.inlier_sigma * S
        k = int(inl.sum())
        if k > best_k:
            best, best_k = inl, k
    return best


def _triangulate_views(sol, O, D, S, g) -> str:
    """Fill sol on success and return ''. Otherwise return a factual reason."""
    n = len(D)
    if n < 2:
        return 'single view'
    why, sol.parallax_deg, sol.baseline_m = _geometry(O, D, g)
    if why:
        return why
    used = np.ones(n, bool)
    if n >= 4:
        # With <= 3 views a "consistent pair" is trivially consistent (coplanar rays always
        # meet), so outlier rejection needs a consensus of at least 3 views.
        need = max(3, math.ceil((1.0 - g.max_reject_frac) * n))
        inl = _consensus(O, D, S, g)
        if inl is None or inl.sum() < need:
            best = 0 if inl is None else int(inl.sum())
            return f'no consistent subset of views (best {best} of {n})'
        used = inl
        try:                                     # one refinement pass on the consensus
            x, *_ = triangulate(O[used], D[used], S[used])
            again = _residual_angles(O, D, x) <= g.inlier_sigma * S
            if again.sum() >= need:
                used = again
        except np.linalg.LinAlgError:
            pass
    idx = np.flatnonzero(used)
    try:
        x, cov, t, chi2_i, dof = triangulate(O[idx], D[idx], S[idx])
    except np.linalg.LinAlgError:
        return 'singular geometry'
    cpd = float(chi2_i.sum() / dof)
    sol.used = used.tolist()
    sol.n_used, sol.n_rejected, sol.chi2_per_dof = len(idx), n - len(idx), cpd
    if cpd > g.max_chi2_per_dof:
        return f'rays inconsistent (chi2/dof {cpd:.1f}, {n - len(idx)} views rejected)'
    why, sol.parallax_deg, sol.baseline_m = _geometry(O[idx], D[idx], g)
    if why:
        return why + ' after rejecting views'
    if np.any(t <= 0):
        return 'intersection behind a camera'
    v = x - O[idx].mean(axis=0)
    v /= np.linalg.norm(v)
    sol.point, sol.cov = x, cov
    sol.sigma_view = float(math.sqrt(v @ cov @ v))
    sol.status = 'CONFIRMED' if sol.sigma_view <= g.max_sigma_view_m else 'LOW_CONFIDENCE'
    if sol.status == 'LOW_CONFIDENCE':
        sol.reason = f'depth sigma {sol.sigma_view:.2f} m > {g.max_sigma_view_m:.2f} m'
    return ''


def estimate(rays: List[ViewRay], g: Gates, prior: Optional[HeightPrior] = None) -> Solution:
    """Pure function of the current set of view rays. Called at a fixed rate."""
    sol = Solution(rays=list(rays), n_views=len(rays), used=[True] * len(rays))
    if not rays:
        return sol
    ref = max(rays, key=lambda r: r.t_last)
    sol.ref_ray = ref
    sol.azimuth_deg, sol.elevation_deg = bearing_deg(ref.direction)
    sol.status = 'BEARING_ONLY'
    O = np.array([r.origin for r in rays])
    D = np.array([r.direction for r in rays])
    S = np.array([r.sigma for r in rays])
    why = _triangulate_views(sol, O, D, S, g)
    if sol.point is not None:
        return sol
    sol.reason = why
    if prior is not None:
        res = plane_point(ref.origin, ref.direction, prior.height_m, ref.sigma,
                          prior.sigma_m, prior.min_elev_deg)
        if res is None:
            sol.reason += f'; height prior unusable (elevation {sol.elevation_deg:.1f} deg)'
        else:
            sol.point, sol.cov = res
            v = sol.point - ref.origin
            v /= np.linalg.norm(v)
            sol.sigma_view = float(math.sqrt(v @ sol.cov @ v))
            sol.status = 'PROVISIONAL'
    return sol


# ------------------------------------------------------------------ size prior
def size_prior_range(extent_px, fx, w_min_m, w_max_m, quant_px=1.0):
    """Range interval (m) from blob extent. Display only, not a measurement.

    Interval grows from two sources: unknown target size [w_min, w_max] and +-quant_px
    of blob quantisation, which dominates at 32x24.
    """
    lo = fx * w_min_m / (extent_px + quant_px)
    hi = fx * w_max_m / max(extent_px - quant_px, 0.5)
    return lo, hi
