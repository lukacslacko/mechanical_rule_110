"""Cam-system parts: camshaft (with Geneva driver), Geneva wheel, sprocket
shaft, crank, the three cam followers (bail / guide-punch slider / hammer)
and their slide-in window keys. World coordinates, reference state theta=0.

Cam stack along Y (final): geneva driver | bail cam (hub 19.5-25, rib 25-28,
plate 28.5-31) | gp cam (SPOKED hub 39.8-44.6, rib 44.6-47.8, plate
48.3-51.3) | ham cam (SPOKED hub 52.5-57, rib 57-60.5, plate 61-63.2) |
crank hex. Both punch cams sit SOUTH of the swing-arm/pad region (y 62-71),
whose airspace below z~95 their ribs would otherwise sweep.
"""

import math
from build123d import (
    Polygon, extrude, Location, Rotation, Pos, Circle, RegularPolygon,
    Sphere, scale, Part,
)

from .params import P
from . import layout as L
from . import camprofiles as cam
from . import geneva
from .geom import (
    box_at, cyl_x, cyl_y, cyl_z, hex_y, pyramid_tip, rib_solid, disc_solid,
    cam_locate, polar_ring, union_all, cut_all,
)

HEX_AF = L.HEX_AF


def _spoked_hub(radius, y0, y1) -> Part:
    """Annulus + 4 spokes at 45+k*90 (local) + small center boss: the
    slide-in keys thread the 6-o'clock gap at the assembly angle."""
    sk = Circle(radius) - Circle(17.0)
    for k in range(3):       # none near 270deg: the keys thread there
        sk += Rotation(0, 0, 90 + k * 120) * Polygon(
            (0, -4), (17.5, -4), (17.5, 4), (0, 4), align=None)
    # no center boss: the d8 shaft core passes through; the keys transit
    # the hub bands at radius 7.3 during insertion
    return cam_locate(y1) * extrude(sk, amount=y1 - y0)


def _wheel_locate(y_top):
    return (Location((L.SPROCKET_X, y_top, L.SPROCKET_Z)) *
            Rotation(90, 0, 0))


def build_camshaft() -> Part:
    p = cyl_y(L.CAMSHAFT_X, L.CAMSHAFT_Z, -13.0, 102.0, L.SHAFT_D)
    # geneva driver disc + pin + locking ring (pin at local angle -180)
    p += disc_solid(28.5, L.DRIVER_DISC_Y0, L.DRIVER_DISC_Y1 - 0.4)
    pin_local = (P.geneva.crank_radius * math.cos(math.radians(L.GENEVA_PIN_GAMMA)),
                 P.geneva.crank_radius * math.sin(math.radians(L.GENEVA_PIN_GAMMA)))
    pin = extrude(Pos(*pin_local) * Circle(L.GENEVA_PIN_D / 2),
                  amount=L.GENEVA_WHEEL_Y1 - L.GENEVA_WHEEL_Y0 + 0.4)
    p += cam_locate(L.GENEVA_WHEEL_Y1) * pin
    # (no root boss: anything wider than the 5.5 slot jams the wheel)
    outer = geneva.ring_outer_profile()
    ring = extrude(Polygon(*[(outer(g) * math.cos(math.radians(g)),
                              outer(g) * math.sin(math.radians(g)))
                             for g in range(0, 360, 1)], align=None),
                   amount=L.GENEVA_WHEEL_Y1 - L.GENEVA_WHEEL_Y0 + 0.4)
    p += cam_locate(L.GENEVA_WHEEL_Y1) * ring
    # cams
    p += disc_solid(L.CAM_BAIL["hub_r"], *L.CAM_BAIL["hub"])
    p += rib_solid(L.bail_window_z, L.CAM_BAIL["rib"][0] - 0.2, L.CAM_BAIL["rib"][1])
    p += _spoked_hub(L.CAM_GP["hub_r"], *L.CAM_GP["hub"])
    p += rib_solid(L.gp_window_z, L.CAM_GP["rib"][0] - 0.2, L.CAM_GP["rib"][1])
    p += _spoked_hub(L.CAM_HAM["hub_r"], *L.CAM_HAM["hub"])
    p += rib_solid(L.ham_window_z, L.CAM_HAM["rib"][0] - 0.2, L.CAM_HAM["rib"][1])
    # crank hex
    p += hex_y(L.CAMSHAFT_X, L.CAMSHAFT_Z, 102.0, 114.0, HEX_AF)
    return p


def build_geneva_wheel() -> Part:
    sk = Circle(geneva.WHEEL_R)
    for a in geneva._slot_dirs():
        u = (math.cos(a), math.sin(a))
        v = (-u[1], u[0])
        r0, r1, w = L.WHEEL_SLOT_INNER_R, geneva.WHEEL_R + 2, geneva.SLOT_HALF_W
        sk -= Polygon((r0 * u[0] - w * v[0], r0 * u[1] - w * v[1]),
                      (r1 * u[0] - w * v[0], r1 * u[1] - w * v[1]),
                      (r1 * u[0] + w * v[0], r1 * u[1] + w * v[1]),
                      (r0 * u[0] + w * v[0], r0 * u[1] + w * v[1]),
                      align=None)
        sk -= Pos(r0 * u[0], r0 * u[1]) * Circle(w)
    for cx, cy in geneva._arc_centers():
        sk -= Pos(cx, cy) * Circle(geneva.ARC_R)
    body = _wheel_locate(L.GENEVA_WHEEL_Y1) * extrude(
        sk, amount=L.GENEVA_WHEEL_Y1 - L.GENEVA_WHEEL_Y0)
    hub = _wheel_locate(-1.6) * extrude(
        Circle(9.0), amount=-1.6 - L.WHEEL_HUB_Y0)   # ends 0.4 clear of the
    # sprocket-left cap pin head at y -1.2
    p = body + hub
    bore = _wheel_locate(L.WHEEL_HUB_Y1 + 0.5) * extrude(
        RegularPolygon((HEX_AF + 0.3) / math.sqrt(3), 6),
        amount=(L.WHEEL_HUB_Y1 - L.GENEVA_WHEEL_Y0) + 1.0)
    return p - bore


def _tooth(y_c, ang_deg) -> Part:
    # prolate spheroid (revolve of a half-ellipse about the radial axis):
    # clean single-surface BRep -> watertight STLs (a scaled+fused Sphere
    # leaves tangent seams that break mesh booleans / confuse fcl)
    from build123d import Plane, Spline, Polyline, make_face, revolve, Axis
    import numpy as _np
    pts = [(3.0 * math.cos(t), 1.4 * math.sin(t))
           for t in _np.linspace(0.0, math.pi, 32)]      # upper half-ellipse
    prof = Polygon(*(pts + [(-3.0, 0.0), (3.0, 0.0)][:1]), align=None)
    t = revolve(prof, Axis.X, 360)
    t = Rotation(0, -ang_deg, 0) * t
    a = math.radians(ang_deg)
    cxz = (L.SPROCKET_X + L.SPK_ROOT_R * math.cos(a),
           L.SPROCKET_Z + L.SPK_ROOT_R * math.sin(a))
    return Pos(cxz[0], y_c, cxz[1]) * t


def build_sprocket_shaft() -> Part:
    p = cyl_y(L.SPROCKET_X, L.SPROCKET_Z, -1.0, 101.0, L.SHAFT_D)
    p += hex_y(L.SPROCKET_X, L.SPROCKET_Z, -13.0, -1.0, HEX_AF)
    p += cyl_y(L.SPROCKET_X, L.SPROCKET_Z, -16.0, -13.0, 6.5)   # clip stub
    p -= (cyl_y(L.SPROCKET_X, L.SPROCKET_Z, -14.7, -13.8, 12.0) -
          cyl_y(L.SPROCKET_X, L.SPROCKET_Z, -14.8, -13.7, 5.0))
    for y_c in (L.GUIDE_A_Y, L.GUIDE_B_Y):
        p += cyl_y(L.SPROCKET_X, L.SPROCKET_Z, y_c - L.SPK_BODY_HALF_Y,
                   y_c + L.SPK_BODY_HALF_Y, 2 * L.SPK_ROOT_R)
        for k in range(6):
            p += _tooth(y_c, -90.0 + k * 60.0)
    return p


def build_spk_clip() -> Part:
    c = cyl_y(L.SPROCKET_X, L.SPROCKET_Z, -14.6, -13.9, 9.0)
    c -= cyl_y(L.SPROCKET_X, L.SPROCKET_Z, -15.0, -13.5, 5.1)
    c -= box_at(L.SPROCKET_X - 1.6, L.SPROCKET_X + 1.6,
                -15.0, -13.5, L.SPROCKET_Z, L.SPROCKET_Z + 6.0)
    return c


def build_crank() -> Part:
    p = cyl_y(L.CAMSHAFT_X, L.CAMSHAFT_Z, 102.0, 114.0, 22.0)
    p += box_at(L.CAMSHAFT_X, L.CAMSHAFT_X + 59.0, 106.0, 114.0,
                L.CAMSHAFT_Z - 4.0, L.CAMSHAFT_Z + 4.0)
    p += cyl_y(L.CAMSHAFT_X + 55.0, L.CAMSHAFT_Z, 114.0, 144.0, 12.0)
    p -= hex_y(L.CAMSHAFT_X, L.CAMSHAFT_Z, 101.0, 115.0, HEX_AF + 0.3)
    return p


# ------------------------------------------------------------ followers

def build_bail() -> Part:
    """Plate at y 27..28.4 (in FRONT of the collars at 28.5+, behind nothing
    else: pin shafts span y 30..34). End tabs ride the comb-tower grooves
    (y 27.1..28.75). Fingers cantilever +Y under the collars."""
    ref = L.bail_window_z(0.0)                  # 93.1
    adds = [
        box_at(9.5, 43.4, 26.8, 28.2, 80.0, 101.5),
        box_at(7.4, 9.7, 26.95, 28.4, 80.0, 101.5),       # left tab
        box_at(43.3, 44.8, 26.95, 28.4, 80.0, 101.5),     # right tab
        cyl_y(L.CAMSHAFT_X, ref - L.BOSS_OFF,
              L.CAM_BAIL["rib_contact"][0], 28.2, 2 * L.BOSS_R),
    ]
    for px in L.PIN_XS.values():
        adds.append(box_at(px - 3.5, px + 3.5, 26.8, 34.3, 80.5, 83.0))
    adds.append(box_at(9.5, 20.0, 26.8, 28.2, 80.0, 83.0))  # finger-R root
    p = union_all(adds)
    cuts = [cyl_y(L.CAMSHAFT_X, ref + L.BOSS_OFF, 26.4, 28.6, 5.0)]  # key press hole
    for px in L.PIN_XS.values():
        cuts.append(box_at(px - 2.3, px + 2.3, 31.5, 34.5,
                           80.0, 83.5))
        cuts.append(cyl_z(px, 31.5, 80.0, 83.5, 4.6))
    return cut_all(p, cuts)


def build_key_bail() -> Part:
    """Flush press-fit pin (d5 in a d5.0 hole + a drop of glue): no head or
    barb fits anywhere around the and3/collar cluster."""
    ref = L.bail_window_z(0.0) + L.BOSS_OFF
    return cyl_y(L.CAMSHAFT_X, ref, L.CAM_BAIL["rib_contact"][0], 28.2, 5.0)


def build_gp_slider() -> Part:
    pl = L.CAM_GP["plate"]                      # 48.3..51.3
    p = box_at(17.0, 31.0, pl[0], pl[1], 85.0, L.GP_PLATE_ZTOP)
    p -= cyl_y(L.CAMSHAFT_X, L.KEY_GP_Z, pl[0] - 1, pl[1] + 1, 5.2)
    p += cyl_y(L.CAMSHAFT_X, L.KEY_GP_Z - 2 * L.BOSS_OFF,
               L.CAM_GP["rib_contact"][0] + 0.2, pl[1], 2 * L.BOSS_R)
    p += box_at(19.0, 26.6, pl[0], 55.5, 78.0, 85.5)        # arm to shank
    p += box_at(15.0, 22.0, 46.0, 49.5, 78.0, 81.0)         # dummy arm
    p += cyl_z(L.GP_SHANK_X, L.GP_Y, 66.4, 78.5, 4.0)
    p += Pos(L.GP_SHANK_X, L.GP_Y, cam.PUNCH_TIP_UP) * pyramid_tip(3.2, 2.5)
    p += cyl_z(17.0, 47.5, 64.0, 78.5, 4.0)                 # dummy shank
    return p


def build_key_gp() -> Part:
    """Headless snap pin from -Y (starts inside the y 33..44 corridor)."""
    pl = L.CAM_GP["plate"]
    return (cyl_y(L.CAMSHAFT_X, L.KEY_GP_Z,
                  L.CAM_GP["rib_contact"][0] + 0.1, pl[1], 5.0) +
            cyl_y(L.CAMSHAFT_X, L.KEY_GP_Z, pl[1], pl[1] + 0.7, 5.8))


def build_hammer() -> Part:
    pl = L.CAM_HAM["plate"]                     # 61.0..63.2
    p = union_all([
        box_at(16.0, 32.0, pl[0], pl[1], 89.7, 105.0),       # fork plate
        cyl_y(L.CAMSHAFT_X, L.KEY_HAM_Z - 2 * L.BOSS_OFF,
              L.CAM_HAM["rib_contact"][0] + 0.2, pl[1], 2 * L.BOSS_R),
        box_at(L.HAM_WALL_X0, L.HAM_WALL_X1, 63.5, 69.5, 77.6, 96.0),
        box_at(L.HAM_WALL_X0, L.HAM_WALL_X1, pl[0], 65.0, 89.7, 96.0),
        box_at(L.HAM_WALL_X1, 27.5, 63.5, 69.5,
               L.PAD_FACE_REST, L.PAD_SLAB_Z1),              # pad slab
        cyl_z(24.0, L.DP_Y, L.LIP_TOP - 1.0, L.LIP_TOP, 11.0),
        box_at(11.0, 17.2, 65.0, 69.0, 88.0, 91.0),          # west arm
        cyl_z(L.HAM_SHANK_XS[0], L.DP_Y, 70.0, 90.0, 5.0),
        box_at(27.0, 36.0, 63.5, 66.0, 90.7, 93.5),          # east arm
        cyl_z(L.HAM_SHANK_XS[1], L.DP_Y, 70.0, 92.0, 5.0),
    ])
    p = cut_all(p, [
        cyl_y(L.CAMSHAFT_X, L.KEY_HAM_Z, pl[0] - 1, pl[1] + 1, 5.2),

        box_at(21.7, 26.3, 66.6, 69.6, L.PAD_FACE_REST - 1,
               L.PAD_SLAB_Z1 + 1),                           # stem slot
        cyl_z(24.0, 66.6, L.PAD_FACE_REST - 1, L.PAD_SLAB_Z1 + 1, 4.6),
        cyl_z(24.0, L.DP_Y, L.LIP_TOP - 1.5, L.LIP_TOP + 0.5, 5.2),
        box_at(21.7, 26.3, L.DP_Y + 2.5, L.DP_Y + 7.0,
               L.LIP_TOP - 1.5, L.LIP_TOP + 0.5),            # lip opening
    ])
    # key retention: entry ridge ring (d4.8 throat) inside the key hole
    ridge = (cyl_y(L.CAMSHAFT_X, L.KEY_HAM_Z, 61.0, 61.4, 5.4) -
             cyl_y(L.CAMSHAFT_X, L.KEY_HAM_Z, 60.9, 61.5, 4.8))
    return p.fuse(ridge).clean()


def build_key_ham() -> Part:
    """Snap pin from -Y, fully inside the hammer plate hole: shaft d5, neck
    d4.6 through the hole's entry ridge (d4.8 at y 61.0..61.4), barb d5.15
    expands in the d5.2 section behind it. Nothing protrudes past y 62.2
    (the swing-arm bar edge sweeps at y 63.5)."""
    return (cyl_y(L.CAMSHAFT_X, L.KEY_HAM_Z,
                  L.CAM_HAM["rib_contact"][0] + 0.1, 60.9, 5.0) +
            cyl_y(L.CAMSHAFT_X, L.KEY_HAM_Z, 60.9, 61.5, 4.6) +
            cyl_y(L.CAMSHAFT_X, L.KEY_HAM_Z, 61.5, 62.2, 5.15))


def build() -> dict:
    return {
        "camshaft": build_camshaft(),
        "geneva_wheel": build_geneva_wheel(),
        "sprocket_shaft": build_sprocket_shaft(),
        "spk_clip": build_spk_clip(),
        "crank": build_crank(),
        "bail": build_bail(),
        "key_bail": build_key_bail(),
        "gp_slider": build_gp_slider(),
        "key_gp": build_key_gp(),
        "hammer": build_hammer(),
        "key_ham": build_key_ham(),
    }
