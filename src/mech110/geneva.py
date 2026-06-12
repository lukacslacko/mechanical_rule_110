"""Geneva mechanism geometry: wheel silhouette and the driver's locking-ring
crescent, computed numerically from the same kinematics the sweep uses.

Wheel: center (SPROCKET_X, SPROCKET_Z), axis along Y. At reference (theta=0)
slot 0 points at world angle WHEEL_SLOT0_ANGLE (=90 deg, straight up at the
incoming driver pin); slots every 60 deg; concave locking arcs (r = LOCK_RING_R
+ 0.3) centered at distance = center_distance, at angles WHEEL_ARC_ANGLE0+k*60.
Wheel pose during the cycle: rotation by -psi_n(theta) (world angles decrease).
"""

import math
import numpy as np

from .params import P
from . import layout as L
from . import camprofiles as cam

WHEEL_R = P.geneva.wheel_radius          # 41.57
ARC_R = L.LOCK_RING_R + 0.3              # 14.3
D = P.geneva.center_distance             # 48
SLOT_HALF_W = L.WHEEL_SLOT_W / 2         # 2.75
N = P.geneva.slots


def _slot_dirs():
    return [math.radians(L.WHEEL_SLOT0_ANGLE + k * 360 / N) for k in range(N)]


def _arc_centers():
    return [(D * math.cos(math.radians(L.WHEEL_ARC_ANGLE0 + k * 360 / N)),
             D * math.sin(math.radians(L.WHEEL_ARC_ANGLE0 + k * 360 / N)))
            for k in range(N)]


def _in_slot(p):
    for a in _slot_dirs():
        u = (math.cos(a), math.sin(a))
        along = p[0] * u[0] + p[1] * u[1]
        across = -p[0] * u[1] + p[1] * u[0]
        if abs(across) <= SLOT_HALF_W and along >= L.WHEEL_SLOT_INNER_R:
            return True
        # slot end circle
        ex, ey = L.WHEEL_SLOT_INNER_R * u[0], L.WHEEL_SLOT_INNER_R * u[1]
        if (p[0] - ex) ** 2 + (p[1] - ey) ** 2 <= SLOT_HALF_W ** 2:
            return True
    return False


def _in_arc(p):
    for cx, cy in _arc_centers():
        if (p[0] - cx) ** 2 + (p[1] - cy) ** 2 <= ARC_R ** 2:
            return True
    return False


def wheel_boundary_points(step_deg=0.1, wall_step=0.1):
    """Boundary sample cloud of the wheel silhouette, wheel-local XZ coords
    (x=cos component, y=sin component of world XZ angle), at reference pose."""
    pts = []
    for i in np.arange(0, 360, step_deg):
        a = math.radians(i)
        p = (WHEEL_R * math.cos(a), WHEEL_R * math.sin(a))
        if not _in_slot(p) and not _in_arc(p):
            pts.append(p)
    for a in _slot_dirs():
        u = (math.cos(a), math.sin(a))
        v = (-u[1], u[0])
        for r in np.arange(L.WHEEL_SLOT_INNER_R, WHEEL_R, wall_step):
            for s in (-SLOT_HALF_W, SLOT_HALF_W):
                p = (r * u[0] + s * v[0], r * u[1] + s * v[1])
                if math.hypot(*p) <= WHEEL_R and not _in_arc(p):
                    pts.append(p)
    for cx, cy in _arc_centers():
        for i in np.arange(0, 360, 0.25):
            a = math.radians(i)
            p = (cx + ARC_R * math.cos(a), cy + ARC_R * math.sin(a))
            if math.hypot(*p) <= WHEEL_R and not _in_slot(p):
                pts.append(p)
    return np.array(pts)


def crescent_min_radius(bins=360):
    """For each driver-local angle bin, the minimum distance from the
    camshaft axis to any wheel material over the DRIVE window (theta in
    [-8, 128]). Used to carve the locking ring so the wheel never hits it
    while indexing."""
    pts = wheel_boundary_points()
    minr = np.full(bins, np.inf)
    cx = L.SPROCKET_X - L.CAMSHAFT_X      # wheel center rel. driver axis: -24
    cz = L.SPROCKET_Z - L.CAMSHAFT_Z      # -41.57
    for theta in np.arange(-8.0, 128.0, 0.5):
        psi = cam.geneva_psi(theta % 360.0)
        rot = math.radians(-psi)          # wheel world rotation
        c, s = math.cos(rot), math.sin(rot)
        x = pts[:, 0] * c - pts[:, 1] * s + cx
        z = pts[:, 0] * s + pts[:, 1] * c + cz
        # driver-local: undo driver rotation (+theta shifts world->local)
        ang = (np.degrees(np.arctan2(z, x)) - theta) % 360.0
        r = np.hypot(x, z)
        keep = r < L.LOCK_RING_R + 6.0
        b = (ang[keep] / (360.0 / bins)).astype(int) % bins
        np.minimum.at(minr, b, r[keep])
    # smear two bins each way (angular discretization safety)
    sm = minr
    for k in (1, -1, 2, -2):
        sm = np.minimum(sm, np.roll(minr, k))
    return sm


def ring_outer_profile():
    """Locking ring outer radius as a function of driver-local angle (deg)."""
    minr = crescent_min_radius()
    bins = len(minr)

    def outer(gamma):
        i = int((gamma % 360.0) / (360.0 / bins)) % bins
        return max(6.0, min(L.LOCK_RING_R, minr[i] - 0.4))
    return outer
