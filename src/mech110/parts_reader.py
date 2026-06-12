"""Reader-station parts: pins, collars, comb bridge, top guide, AND3
guillotine, OR rockers, rocker axle and the axle L-columns.

All parts are modeled in WORLD coordinates (mm), build123d algebra mode.

Attitudes:
- "pin" and "collar" are built at the REFERENCE state (theta=0, bail up):
  pin tip apex at z = camprofiles.PIN_TIP_UP = 63.1.
- "and3", "rocker_c", "rocker_r" are built at READ-REST attitude: pin tips
  on paper + 0.1 witness (tip at 60.2), collar top at 82.7 nominal
  (L.AND3_REF_COLLAR_TOP + 0.1); resting pads/feet bottoms at 82.7.
- everything else is static.

The assembly layer poses the parts; only one pin/collar is built (the other
two are posed copies at x = L.PIN_XS).

Deviations from the routing spec are flagged with "DEV:" comments.
"""

import math

from build123d import Part, Polygon, Pos, Sphere, extrude

from . import layout as L
from . import camprofiles as cam
from . import kinematics as kin
from .geom import box_at, cyl_x, cyl_z

TIP_REF = cam.PIN_TIP_UP                  # 63.1 reference tip apex
PIN_X = L.PIN_XS["C"]                     # 24 (build the C pin only)
PIN_Y = L.PIN_Y                           # 32


def _annular_sector(cx, cy, r_in, r_out, a0_deg, a1_deg, z0, z1, n=24) -> Part:
    """Annular sector (deg, CCW from +X) around (cx, cy), extruded z0..z1."""
    pts = []
    for i in range(n + 1):
        a = math.radians(a0_deg + (a1_deg - a0_deg) * i / n)
        pts.append((cx + r_out * math.cos(a), cy + r_out * math.sin(a)))
    for i in range(n + 1):
        a = math.radians(a1_deg - (a1_deg - a0_deg) * i / n)
        pts.append((cx + r_in * math.cos(a), cy + r_in * math.sin(a)))
    return Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), amount=z1 - z0)


# ------------------------------------------------------------------- pin

def _pin() -> Part:
    # one revolved lathe profile (tangent sphere+cylinder fusions leave
    # degenerate seams -> non-watertight STLs -> fcl phantom contacts)
    import numpy as _np
    from build123d import revolve, Axis, Polygon as BPoly
    z0 = TIP_REF
    prof = [(0.0, 0.0)]
    prof += [(1.75 * math.sin(t), 1.75 * (1 - math.cos(t)))
             for t in _np.linspace(0.05, math.pi / 2, 16)]   # nose arc
    prof += [(1.75, 4.0), (2.0, 4.6),
             (2.0, L.COLLAR_Z0), (1.7, L.COLLAR_Z0),       # clip groove
             (1.7, L.COLLAR_Z1), (2.0, L.COLLAR_Z1),
             (2.0, L.PIN_LEN), (0.0, L.PIN_LEN)]
    pin2d = BPoly(*prof, align=None)
    p = revolve(pin2d, Axis.Y, 360)
    from build123d import Rotation as BRot
    return Pos(PIN_X, PIN_Y, z0) * BRot(90, 0, 0) * p


# ----------------------------------------------------------------- collar

def _collar() -> Part:
    z0, z1 = TIP_REF + 20.05, TIP_REF + 22.45
    c = cyl_z(PIN_X, PIN_Y, z0, z1, L.COLLAR_D)                      # d7
    c -= cyl_z(PIN_X, PIN_Y, z0 - 0.1, z1 + 0.1, 3.5)                # bore
    # C-opening: slot 3.2 wide (x) from pin center to the -Y edge (snaps on
    # from -Y, onto the d3.4 groove core).
    c -= box_at(PIN_X - 1.6, PIN_X + 1.6, PIN_Y - 4.5, PIN_Y,
                z0 - 0.1, z1 + 0.1)
    return c


# ------------------------------------------------------------ comb bridge

def _comb_bridge() -> Part:
    # body only over the reader zone (cover A owns x +-10.5..15.5)
    p = box_at(14.0, L.COMB_X1, L.COMB_BODY_Y0, L.COMB_BODY_Y1,
               L.COMB_Z0, L.COMB_Z1)                       # body 61.2..70
    # top straps over the tape-edge gaps, HIGH (z>=64.4 clears cover A)
    p += box_at(14.0, 48, 13, 26, 64.4, 70)
    p += box_at(14.0, 48, 38, 40, 64.4, 70)
    # feet east of cover A's bridge (x>=18)
    p += box_at(18.0, 48, L.COMB_FOOT_S_Y0, L.COMB_FOOT_S_Y1, 60, 70)
    p += box_at(18.0, 48, L.COMB_FOOT_N_Y0, 43.8, 60, 70)
    for px, py in ((20, 10.75), (44, 10.75)):
        p += cyl_z(px, py, 56, 60.0, 5.0)
    for px, py in ((20, 41.6), (44, 41.6)):
        p += cyl_z(px, py, 56, 60.0, 4.5)
    # high bridge carrying the west tower over cover A (x>=4.8 clears the
    # sprocket shaft which crosses all y at x +-4.3, z 63.4..72)
    p += box_at(4.8, 15.0, 26.8, 34.5, 64.4, 70)
    # pin guide bosses (replace the former top_guide part)
    for px in L.PIN_XS.values():
        p += cyl_z(px, PIN_Y, 70.0, 73.9, 7.0)
    # rail notches where the feet cross the bed rails
    p -= box_at(13, 49, 12.6, 14.0, 59.0, 62.3)
    p -= box_at(13, 49, 38.9, 40.5, 59.0, 62.3)
    # END TOWERS guide BOTH the bail (groove A, y 28.9..30.65) and the and3
    # (groove B, y 32.2..33.8) via end tabs; no posts, no rail columns.
    p += box_at(4.8, 8.8, 26.8, 34.5, 69.9, 96.0)
    p += box_at(43.5, 47.5, 26.8, 34.5, 69.9, 96.0)
    # pin bores (through body and bosses)
    for px in L.PIN_XS.values():
        p -= cyl_z(px, PIN_Y, 59, 74.5, L.BED_PIN_HOLE_D)
    # grooves (1.75 wide, 1.6 deep, full height, open top)
    for y0, y1 in ((26.85, 28.6), (29.4, 31.15)):
        p -= box_at(7.2, 8.9, y0, y1, 71.0, 96.1)
        p -= box_at(43.4, 45.1, y0, y1, 71.0, 96.1)
    return p


# ------------------------------------------------------------------- and3

def _arm_edge_finger():
    """Finger center: 0.1 witness from the swing-arm bar's -Y edge when the
    arm is at BLOCKED_ANGLE. Computed from the bar outline (width 7)."""
    bx, by = L.BLOCK_OUT
    px, py = L.PIVOT_X, L.PIVOT_Y
    ux, uy = bx - px, by - py
    n = math.hypot(ux, uy)
    ux, uy = ux / n, uy / n
    # -Y edge direction normal (the bar's leading edge when swinging in)
    nx, ny = uy, -ux
    if ny > 0:
        nx, ny = -nx, -ny
    x_f = 50.5
    # edge point at OUT with x = x_f: solve px + t*ux + 3.5*nx = x_f
    t = (x_f - px - 3.5 * nx) / ux
    ex, ey = px + t * ux + 3.5 * nx, py + t * uy + 3.5 * ny
    a = math.radians(kin.BLOCKED_ANGLE)
    # edge point rotated to the blocked attitude
    rx, ry = ex - px, ey - py
    ebx = px + rx * math.cos(a) - ry * math.sin(a)
    eby = py + rx * math.sin(a) + ry * math.cos(a)
    # edge line direction at blocked
    ca, sa = math.cos(a), math.sin(a)
    ux2, uy2 = ux * ca - uy * sa, ux * sa + uy * ca
    nx2, ny2 = uy2, -ux2
    if ny2 > 0:
        nx2, ny2 = -nx2, -ny2
    return (ebx + (L.FINGER_D / 2 + 0.1) * nx2,
            eby + (L.FINGER_D / 2 + 0.1) * ny2)


def _and3() -> Part:
    """Read-rest attitude. Plate + tower tabs + crescent feet + blocker arm
    (S12 descender x 27..31, low run S3/S3b z 79..81.7, riser S4 at
    x 48.5..52.5, high run S5 z 99..102, finger near (50.5, 64))."""
    fx, fy = _arm_edge_finger()
    adds = [
        box_at(9.5, 43.4, 28.6, 31.0, 86.0, 94.0),
        # tower tabs (groove B at y 32.2..33.95)
        box_at(7.4, 9.7, 29.55, 31.0, 86.0, 94.0),
        box_at(43.3, 44.9, 29.55, 31.0, 86.0, 94.0),
        # blocker arm
        box_at(9.0, 12.2, 30.5, 37.6, 79.0, 97.0),      # S12 descender
        box_at(9.0, 12.2, 36.0, 58.2, 79.0, 81.6),      # S3 low run (west)
        box_at(9.0, 52.5, 56.4, 58.2, 79.0, 81.7),      # S3b cross run
        box_at(48.5, 52.5, 56.4, 58.2, 79.0, 102.0),    # S4 riser
        box_at(48.5, 52.5, 56.4, fy + 0.6, 99.0, 102.0),  # S5 high run
        cyl_z(fx, fy, 99.3, 102.0, L.FINGER_D),         # blocker finger
    ]
    # crescent feet on the collar tops (-Y sector) + low side webs
    for px in L.PIN_XS.values():
        adds.append(_annular_sector(px, PIN_Y, 2.8, 3.5, 200, 340,
                                    82.7, 85.0))
        adds.append(box_at(px - 3.3, px - 2.3, 30.9, 31.9, 84.0, 86.9))
        adds.append(box_at(px + 2.3, px + 3.3, 30.9, 31.9, 84.0, 86.9))
    p = adds[0]
    for q in adds[1:]:
        p = p.fuse(q)
    p = p.clean()
    # notch for the bail-key insertion path (key + head pass over at
    # z>=93.4 abs while the and3 sits at its theta=0 pose, +3.0)
    p -= box_at(21.0, 27.0, L.AND3_PLATE_Y0 - 1, L.AND3_PLATE_Y1 + 1,
                90.2, 94.1)
    # S12 must keep clear of the bail key head zone in y (head y 31..33):
    # it starts at y 34.4 -- nothing to cut. Pin-top clearance bores:
    for px in L.PIN_XS.values():
        p -= cyl_z(px, PIN_Y, 85.3, 94.1, L.BED_PIN_HOLE_D)
    return p


# ---------------------------------------------------------------- rockers

def _rocker_c() -> Part:
    """Pad on collar C -> +X run -> turn -> hub -> +Y run -> east arm to
    the tip ball at (TIP_X~70.8, 65.5, 87.45). Read-rest attitude."""
    adds = [
        box_at(22.5, 25.5, 32.4, 36.7, 83.4, 84.6),      # ledge pad
        box_at(22.5, 25.5, 34.4, 35.1, 82.8, 83.6),      # contact foot rib
        box_at(22.5, 25.5, 36.5, 38.2, 82.5, 85.2),      # descender
        # ballast: pad side must outweigh the tip side so the pad follows
        # its pin collar down by gravity (one-way contact, no spring)
        box_at(20.3, 27.7, 36.1, 38.4, 82.5, 90.2),
        box_at(22.5, 54.5, 37.5, 39.0, 82.5, 85.2),      # +X run
        box_at(52.0, 56.0, 38.5, 48.0, 83.0, 86.0),      # turn toward axle
        cyl_x(L.AXLE_Y, L.AXLE_Z, *L.ROCKER_HUB_C, 10.0),   # hub
        box_at(53.0, 56.5, 50.0, 64.9, 84.9, 86.3),      # +Y run (thin)
        box_at(53.0, 71.6, 64.2, 66.2, 84.6, 86.0),      # east arm (thin)
        cyl_z(L.TIP_X, L.TIP_YS[0], 85.5, 86.6, 2.0),    # ball post
        Pos(L.TIP_X, L.TIP_YS[0], 87.45) * Sphere(1.25), # tip ball
    ]
    p = adds[0]
    for q in adds[1:]:
        p = p.fuse(q)
    p = p.clean()
    p -= cyl_x(L.AXLE_Y, L.AXLE_Z, L.ROCKER_HUB_C[0] - 1,
               L.ROCKER_HUB_C[1] + 1, 5.6)               # axle bore
    p -= box_at(L.ROCKER_HUB_C[0] - 1, L.ROCKER_HUB_C[1] + 1,
                L.AXLE_Y - 2.3, L.AXLE_Y + 2.3, L.AXLE_Z, L.AXLE_Z + 6)
    p -= cyl_z(L.PIN_XS["C"], PIN_Y, 82.0, 85.5, 5.2)    # pad pin notch
    return p


def _rocker_r() -> Part:
    """Pad on collar R -> drop connector -> LOW +X run/diagonal under the
    cam hubs -> hub -> LOW +Y run (z 76..77.8, in the free airspace over
    tape B) -> tip riser -> east arm -> ball at (TIP_X, 68.5, 87.45)."""
    adds = [
        box_at(14.5, 17.5, 32.4, 36.7, 83.4, 84.6),      # ledge pad
        box_at(14.5, 17.5, 34.4, 35.1, 82.8, 83.6),      # contact foot rib
        box_at(14.5, 17.5, 36.5, 39.4, 78.6, 84.7),      # drop connector
        box_at(12.6, 19.4, 36.1, 38.4, 82.5, 90.5),      # ballast (see C)
        box_at(14.5, 32.2, 38.2, 39.4, 78.6, 80.1),      # low strip east
        box_at(31.4, 53.0, 38.0, 43.6, 78.6, 80.1),      # diagonal under hubs
        box_at(51.5, 61.0, 42.0, 50.5, 78.6, 80.6),      # to below the hub
        cyl_x(L.AXLE_Y, L.AXLE_Z, *L.ROCKER_HUB_R, 10.0),   # hub
        box_at(57.0, 61.0, 44.0, 50.0, 79.0, 84.0),      # hub riser web
        box_at(57.5, 61.0, 50.0, 52.3, 76.0, 84.0),      # drop to low run
        box_at(57.5, 61.0, 52.0, 71.4, 76.0, 77.8),      # LOW +Y run
        box_at(57.5, 59.5, 68.8, 71.4, 76.0, 86.3),      # tip riser (north
        # of the C arm band: deep-z y-shifts under rotation reach +-1.75)
        box_at(57.5, 71.6, 67.0, 69.3, 84.6, 86.0),      # east arm (thin)
        cyl_z(L.TIP_X, L.TIP_YS[1], 85.5, 86.6, 2.0),
        Pos(L.TIP_X, L.TIP_YS[1], 87.45) * Sphere(1.25),
    ]
    p = adds[0]
    for q in adds[1:]:
        p = p.fuse(q)
    p = p.clean()
    p -= cyl_x(L.AXLE_Y, L.AXLE_Z, L.ROCKER_HUB_R[0] - 1,
               L.ROCKER_HUB_R[1] + 1, 5.6)
    p -= box_at(L.ROCKER_HUB_R[0] - 1, L.ROCKER_HUB_R[1] + 1,
                L.AXLE_Y - 2.3, L.AXLE_Y + 2.3, L.AXLE_Z, L.AXLE_Z + 6)
    p -= cyl_z(L.PIN_XS["R"], PIN_Y, 82.0, 85.5, 5.2)
    return p


# ------------------------------------------------------------------- axle

def _axle() -> Part:
    # single-head rod: threads westward through cradles+hubs from +X
    p = cyl_x(L.AXLE_Y, L.AXLE_Z, 47.6, 65.4, L.AXLE_D)
    p += cyl_x(L.AXLE_Y, L.AXLE_Z, 65.4, 66.4, 8.0)      # head, +X end
    return p


# -------------------------------------------------------------- L-columns

def _lcol(x0, x1, peg_x) -> Part:
    p = box_at(x0, x1, 45.0, 47.5, 60.0, 84.0)           # column
    p += box_at(x0, x1, 45.0, 54.5, 81.5, 89.0)          # cradle arm
    p += cyl_z(peg_x, 46.0, 56.0, 60.0, 5.0)             # bed peg
    # axle cradle: open-top U at (y AXLE_Y, z AXLE_Z)
    p -= box_at(x0 - 1, x1 + 1, L.AXLE_Y - 2.75, L.AXLE_Y + 2.75,
                L.AXLE_Z, 89.5)
    p -= cyl_x(L.AXLE_Y, L.AXLE_Z, x0 - 1, x1 + 1, 5.5)
    return p


def _lcol_left() -> Part:
    return _lcol(48.0, 51.0, 49.5)


def _lcol_right() -> Part:
    return _lcol(62.0, 65.0, 63.5)


# ------------------------------------------------------------------ build

def build() -> dict[str, Part]:
    return {
        "pin": _pin(),
        "collar": _collar(),
        "comb_bridge": _comb_bridge(),
        "and3": _and3(),
        "rocker_c": _rocker_c(),
        "rocker_r": _rocker_r(),
        "axle": _axle(),
        "lcol_left": _lcol_left(),
        "lcol_right": _lcol_right(),
    }
