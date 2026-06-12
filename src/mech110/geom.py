"""Shared geometry helpers (build123d, algebra mode). All parts are modeled
in WORLD coordinates at the reference state theta=0, bits=(0,0,0).

Cam convention (must match kinematics):
- camshaft axis along Y through (CAMSHAFT_X, CAMSHAFT_Z);
- cam profiles are built in a local XY frame, extruded +Z, then mapped by
  cam_locate(y_top) = Location((CX, y_top, CZ)) * Rotation(90,0,0), which
  sends local +Z to world -Y (part spans world y in [y_top-h, y_top]) and
  local polar angle gamma to world XZ angle gamma (measured +X toward +Z);
- the pose rotation R_y(-theta) moves local angle gamma to gamma+theta, so
  a follower hanging at world bottom (angle -90) touches the rib at
  gamma = -90 - theta. A cam enforcing slider position s(theta) has

      rho_c(gamma) = CAMSHAFT_Z - s(theta = (-90 - gamma) mod 360).
"""

import math
import numpy as np
from build123d import (
    Polygon, extrude, Location, Rotation, Box, Cylinder, Pos, Rectangle,
    RegularPolygon, Circle, loft, Part, Sketch, export_stl,
)

from . import layout as L


# ---------------------------------------------------------------- basics

def union_all(parts) -> Part:
    """Robust n-ary fuse (the algebra '+' stops fusing once an intermediate
    result is a multi-solid compound)."""
    total = parts[0]
    for s in parts[1:]:
        total = total.fuse(s)
    return total.clean()


def cut_all(base, cutters) -> Part:
    for c in cutters:
        base = base.cut(c)
    return base.clean()


def box_at(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        x1 - x0, y1 - y0, z1 - z0)


def cyl_y(x, z, y0, y1, d) -> Part:
    return (Pos(x, (y0 + y1) / 2, z) * Rotation(90, 0, 0) *
            Cylinder(d / 2, y1 - y0))


def cyl_x(y, z, x0, x1, d) -> Part:
    return (Pos((x0 + x1) / 2, y, z) * Rotation(0, 90, 0) *
            Cylinder(d / 2, x1 - x0))


def cyl_z(x, y, z0, z1, d) -> Part:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0)


def hex_y(x, z, y0, y1, across_flats) -> Part:
    r = across_flats / math.sqrt(3)
    p = extrude(RegularPolygon(r, 6), amount=(y1 - y0))
    return Pos(x, y1, z) * Rotation(90, 0, 0) * p


def pyramid_tip(d, h) -> Part:
    """4-sided piercing tip, apex DOWN at local z=0, square d x d at z=h."""
    return loft([Pos(0, 0, 0.01) * Rectangle(0.12, 0.12).face(),
                 Pos(0, 0, h) * Rectangle(d, d).face()])


# ------------------------------------------------------------------ cams

def cam_locate(y_top) -> Location:
    return (Location((L.CAMSHAFT_X, y_top, L.CAMSHAFT_Z)) *
            Rotation(90, 0, 0))


def rho_from_slider(slider_z_fn):
    def rho(gamma):
        theta = (-90.0 - gamma) % 360.0
        return L.CAMSHAFT_Z - slider_z_fn(theta)
    return rho


def polar_ring(rho_outer_fn, rho_inner_fn, n=240) -> Sketch:
    def pts(fn):
        out = []
        for i in range(n):
            g = 360.0 * i / n
            r = fn(g)
            out.append((r * math.cos(math.radians(g)),
                        r * math.sin(math.radians(g))))
        return out
    return Polygon(*pts(rho_outer_fn), align=None) - Polygon(
        *pts(rho_inner_fn), align=None)


def _normal_offset_points(rho_c, offset, n=360):
    """Closed curve at constant NORMAL offset from the polar pitch curve
    rho_c(gamma). Radial offsets pinch the follower window at steep
    sections (a tilted rib is vertically thicker); normal offsets keep the
    boss/key clearance constant."""
    pts = []
    dg = math.radians(360.0 / n)
    for i in range(n):
        g = 360.0 * i / n
        r = rho_c(g)
        drdg = (rho_c((g + 0.5) % 360) - rho_c((g - 0.5) % 360)) / math.radians(1.0)
        a = math.radians(g)
        # position and unit normal of the polar curve
        px, py = r * math.cos(a), r * math.sin(a)
        # tangent: d/da (r cos a, r sin a) = (r' cos - r sin, r' sin + r cos)
        tx = drdg * math.cos(a) - r * math.sin(a)
        ty = drdg * math.sin(a) + r * math.cos(a)
        tl = math.hypot(tx, ty)
        nx, ny = ty / tl, -tx / tl          # outward normal (radial-ish)
        if nx * px + ny * py < 0:
            nx, ny = -nx, -ny
        pts.append((px + offset * nx, py + offset * ny))
    return pts


def rib_solid(slider_z_fn, y0, y1, n=720) -> Part:
    """Annular cam rib enforcing window-center position slider_z_fn(theta),
    spanning world y0..y1.

    Walls are ENVELOPES of the follower boss circles (the boss centers ride
    at radial offsets -+BOSS_OFF from the window center). Computed with
    shapely buffers: naive normal-offset curves cusp/self-intersect where
    the path curvature radius < offset, producing garbage solids."""
    from shapely.geometry import Polygon as SPoly
    rho_c = rho_from_slider(slider_z_fn)
    gap = L.BOSS_R + 0.3

    def path(off):
        return [(r * math.cos(math.radians(g)), r * math.sin(math.radians(g)))
                for g in (i * 360.0 / n for i in range(n))
                for r in ((rho_c(g) + off),)]

    from shapely.geometry.polygon import orient
    outer_region = SPoly(path(+L.BOSS_OFF)).buffer(-gap, quad_segs=8)
    inner_region = SPoly(path(-L.BOSS_OFF)).buffer(+gap, quad_segs=8)
    ring_region = orient(outer_region.difference(inner_region), 1.0)
    ext = list(ring_region.exterior.coords)[:-1]
    holes = [list(h.coords)[:-1] for h in ring_region.interiors]
    ring = Polygon(*ext, align=None)
    for h in holes:
        ring -= Polygon(*h, align=None)
    return cam_locate(y1) * extrude(ring, amount=(y1 - y0))


def disc_solid(radius, y0, y1, bore=0.0) -> Part:
    sk = Circle(radius)
    if bore > 0:
        sk -= Circle(bore / 2)
    return cam_locate(y1) * extrude(sk, amount=(y1 - y0))


# ------------------------------------------------- follower window voids

def window_void_polygon(slider_z_fn, margin=0.3, dx_max=12.0, step=0.5):
    """Follower-local outline (dx, dz from window center) of everything the
    rib sweeps near the window over a full revolution, + margin."""
    rho_c = rho_from_slider(slider_z_fn)
    xs = np.arange(-dx_max, dx_max + 1e-9, step)
    lower = np.full(xs.shape, np.inf)
    upper = np.full(xs.shape, -np.inf)
    for theta in np.arange(0.0, 360.0, 1.0):
        zc = slider_z_fn(theta)
        d_axis = L.CAMSHAFT_Z - zc          # axis height above window center
        base = -90.0 - theta
        for i, dx in enumerate(xs):
            g = base
            for _ in range(2):              # fixed-point for gamma(dx)
                r = rho_c(g % 360.0)
                s = max(-1.0, min(1.0, dx / max(r, 1e-6)))
                g = base + math.degrees(math.asin(s))
            r = rho_c(g % 360.0)
            for R in (r - L.RIB_HALF, r + L.RIB_HALF):
                if R * R >= dx * dx:
                    z = d_axis - math.sqrt(R * R - dx * dx)
                    lower[i] = min(lower[i], z)
                    upper[i] = max(upper[i], z)
    ok = np.isfinite(lower)
    return xs[ok], lower[ok] - margin, upper[ok] + margin


def window_cutout(slider_z_fn, y0, y1, center_x, center_z_at_ref,
                  margin=0.3, dx_max=12.0) -> Part:
    """World-positioned void solid to subtract from a follower plate
    (plate built at reference state; center_z_at_ref = window center world z
    at theta=0)."""
    xs, lo, up = window_void_polygon(slider_z_fn, margin, dx_max)
    pts = [(float(x), float(l)) for x, l in zip(xs, lo)]
    pts += [(float(x), float(u)) for x, u in zip(xs[::-1], up[::-1])]
    sk = Polygon(*pts, align=None)
    p = extrude(sk, amount=(y1 - y0))
    # local XY -> world XZ at the window center, local +Z -> world -Y
    return (Location((center_x, y1, center_z_at_ref)) *
            Rotation(90, 0, 0)) * p
