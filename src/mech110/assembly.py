"""Part registry + pose computation.

Every part is built in WORLD coordinates at its declared build attitude;
pose(name, theta, bits) returns the 4x4 transform to apply on top.

Build attitudes:
- theta=0 attitude: camshaft, crank, geneva wheel, sprocket shaft, spk_clip,
  tapes, bail+key, gp_slider+key, hammer+key, pins, collars, data_punch,
  return_rocker (punch at rest), block & stem_clip (swing OUT, z rest),
  swing_arm (OUT).
- read-rest attitude (pins on paper): and3, rocker_c, rocker_r.
- static: everything else.
"""

import math
import numpy as np

from . import layout as L
from . import camprofiles as cam
from . import kinematics as kin
from .kinematics import _T, _rot_y, _rot_x
from .rule110 import rule110


def _rot_z(deg, cx, cy):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    R = np.array([[c, -s, 0.0, 0.0],
                  [s, c, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])
    return _T(cx, cy, 0) @ R @ _T(-cx, -cy, 0)


I4 = np.eye(4)

# cap retention pins: built at left-cam lower hole; right-side instances are
# 180deg z-rotations so the head lands on the OUTER plate face.
_CPIN = {
    "cp_cl1": I4, "cp_cl2": _T(z=4.0),
    "cp_cr1": None, "cp_cr2": None,
    "cp_sl1": _T(x=-24.0, z=-41.5703), "cp_sl2": _T(x=-24.0, z=-37.5703),
    "cp_sr1": None, "cp_sr2": None,
}


_CPIN["cp_cr1"] = _rot_z(180.0, 24.0, 50.0)
_CPIN["cp_cr2"] = _T(z=4.0) @ _rot_z(180.0, 24.0, 50.0)
_CPIN["cp_sr1"] = _T(z=-41.5703) @ _rot_z(180.0, 12.0, 50.0)
_CPIN["cp_sr2"] = _T(z=-37.5703) @ _rot_z(180.0, 12.0, 50.0)


def poses(theta: float, bits=(0, 0, 0)) -> dict[str, np.ndarray]:
    l, c, r = bits
    en = kin.punch_enabled(theta, bits)
    out = {}

    cam_rot = _rot_y(-theta, L.CAMSHAFT_X, L.CAMSHAFT_Z)
    out["camshaft"] = cam_rot
    out["crank"] = cam_rot
    psi = cam.geneva_psi(theta)
    spk_rot = _rot_y(+psi, L.SPROCKET_X, L.SPROCKET_Z)
    out["geneva_wheel"] = spk_rot
    out["sprocket_shaft"] = spk_rot
    out["spk_clip"] = spk_rot

    dx = L.TAPE_DIR * cam.tape_x(theta)
    out["tape_a"] = _T(x=dx)
    out["tape_b"] = _T(x=dx)

    bail_dz = cam.bail_tip_z(theta) - cam.PIN_TIP_UP
    out["bail"] = _T(z=bail_dz)
    out["key_bail"] = out["bail"]

    # pins/collars: instances pin_l/pin_c/pin_r of the single "pin"/"collar"
    # geometry built at pin C (x=24); instance offset in x, DOF in z.
    for name, bit, xoff in (("pin_l", l, 8.0), ("pin_c", c, 0.0),
                            ("pin_r", r, -8.0)):
        dz = kin.pin_tip_z(theta, bit) - cam.PIN_TIP_UP
        out[name] = _T(x=xoff, z=dz)
        out["collar_" + name[-1]] = out[name]

    # and3 built at read-rest: feet bottom 82.7 = collar top 82.6 + 0.1
    max_collar = max(kin.pin_tip_z(theta, b) for b in bits) + L.COLLAR_Z1
    out["and3"] = _T(z=max_collar + 0.1 - 82.7)

    out["rocker_c"] = _rot_x(kin.rocker_angle(theta, bits, "C"),
                             L.AXLE_Y, L.AXLE_Z)
    out["rocker_r"] = _rot_x(kin.rocker_angle(theta, bits, "R"),
                             L.AXLE_Y, L.AXLE_Z)

    arm = _rot_z(kin.pendulum_angle(theta, bits), L.PIVOT_X, L.PIVOT_Y)
    out["swing_arm"] = arm
    blk_dz = cam.block_bottom_z(theta, en) - L.BLOCK_Z0
    out["block"] = arm @ _T(z=blk_dz)

    gp_dz = cam.guide_punch_tip_z(theta) - cam.PUNCH_TIP_UP
    out["gp_slider"] = _T(z=gp_dz)
    out["key_gp"] = out["gp_slider"]

    ham_dz = cam.hammer_face_z(theta) - L.PAD_FACE_REST
    out["hammer"] = _T(z=ham_dz)
    out["key_ham"] = out["hammer"]

    dp_tip = cam.data_punch_tip_z(theta, en)
    out["data_punch"] = _T(z=dp_tip - cam.PUNCH_TIP_UP)

    # return rocker follows the flange bottom (one-way support)
    flange_bottom = dp_tip + L.DP_LEN - L.DP_FLANGE_H
    ang = math.degrees(math.asin(
        max(-1.0, min(1.0, (79.1 - flange_bottom) / L.RR_ARM))))
    out["return_rocker"] = _rot_x(ang, L.RR_PIVOT_Y, L.RR_PIVOT_Z)

    return out


# ----------------------------------------------------------- registry
# (base part name, instance name, instance offset transform or None)
INSTANCES = {
    "cap_pin3": [(k, v) for k, v in _CPIN.items()],
    "pin": [("pin_l", _T(x=8.0)), ("pin_c", I4), ("pin_r", _T(x=-8.0))],
    "collar": [("collar_l", _T(x=8.0)), ("collar_c", I4),
               ("collar_r", _T(x=-8.0))],
}

STATIC = {
    "base", "plate_left", "plate_right", "bed", "cover_a", "cover_b",
    "cap_cam_left", "cap_cam_right", "cap_spk_left", "cap_spk_right",
    "comb_bridge", "lcol_left", "lcol_right", "axle",
    "punch_block", "gallows", "pivot_clip", "rr_axle",
} | set(_CPIN)

MOVING_FROM_POSES = {
    "camshaft", "crank", "geneva_wheel", "sprocket_shaft", "spk_clip",
    "tape_a", "tape_b", "bail", "key_bail", "pin_l", "pin_c", "pin_r",
    "collar_l", "collar_c", "collar_r", "and3", "rocker_c", "rocker_r",
    "swing_arm", "block", "gp_slider", "key_gp",
    "hammer", "key_ham", "data_punch", "return_rocker",
}


def build_all(bits=(0, 0, 0)) -> dict:
    """name -> build123d Part for every physical part instance (built
    geometry, instance offsets applied for multi-instance parts)."""
    from . import parts_cam, parts_frame, parts_reader, parts_punch
    from build123d import Location

    raw = {}
    raw.update(parts_cam.build())
    raw.update(parts_frame.build(bits=bits))
    raw.update(parts_reader.build())
    raw.update(parts_punch.build())

    parts = {}
    for name, solid in raw.items():
        if name in INSTANCES:
            for iname, T in INSTANCES[name]:
                if np.allclose(T, I4):
                    parts[iname] = solid
                elif np.allclose(T[:3, :3], np.eye(3)):
                    x, y, z = T[0, 3], T[1, 3], T[2, 3]
                    parts[iname] = Location((x, y, z)) * solid
                else:
                    from build123d import Rotation as BRot
                    # general: rotation about z through a point + translation
                    moved = (Location((T[0, 3] + 24.0 + 24.0 * T[0, 0],
                                       0, 0)))  # placeholder, see below
                    # robust path: apply as matrix via OCP
                    from OCP.gp import gp_Trsf
                    tr = gp_Trsf()
                    tr.SetValues(T[0,0],T[0,1],T[0,2],T[0,3],
                                 T[1,0],T[1,1],T[1,2],T[1,3],
                                 T[2,0],T[2,1],T[2,2],T[2,3])
                    parts[iname] = solid.moved(Location(tr))
        else:
            parts[name] = solid
    return parts


def pose_of(name: str, theta: float, bits=(0, 0, 0),
            _cache={}) -> np.ndarray:
    key = (round(theta, 4), bits)
    if key not in _cache:
        if len(_cache) > 4000:
            _cache.clear()
        _cache[key] = poses(theta, bits)
    p = _cache[key]
    return p.get(name, I4)


# assembly-step prefix for exported STL filenames (see ASSEMBLY.md)
ASSEMBLY_ORDER = {
    "base": 1, "plate_left": 2, "plate_right": 2, "bed": 3,
    "tape_a": 4, "tape_b": 4, "cover_a": 5, "cover_b": 5,
    "comb_bridge": 6, "pin_l": 7, "pin_c": 7, "pin_r": 7, "bail": 8,
    "collar_l": 9, "collar_c": 9, "collar_r": 9, "punch_block": 10,
    "lcol_left": 11, "lcol_right": 11, "rocker_c": 12, "rocker_r": 12,
    "axle": 13, "and3": 14, "gallows": 15, "swing_arm": 16, "block": 16,
    "pivot_clip": 18, "return_rocker": 19, "rr_axle": 20,
    "hammer": 21, "data_punch": 21, "gp_slider": 22, "camshaft": 23,
    "key_bail": 24, "key_gp": 24, "key_ham": 24, "sprocket_shaft": 25,
    "cap_spk_left": 26, "cap_spk_right": 26,
    "cp_sl1": 26, "cp_sl2": 26, "cp_sr1": 26, "cp_sr2": 26,
    "geneva_wheel": 27, "spk_clip": 28,
    "cap_cam_left": 29, "cap_cam_right": 29,
    "cp_cl1": 29, "cp_cl2": 29, "cp_cr1": 29, "cp_cr2": 29,
    "crank": 30,
}


def stl_name(name: str) -> str:
    base = name.split("_")[0] if name.startswith("tape_a_") else name
    step = ASSEMBLY_ORDER.get("tape_a" if name.startswith("tape_a_")
                              else name, 0)
    return f"{step:02d}_{name}"


def strip_step_prefix(stem: str) -> str:
    import re
    return re.sub(r"^\d{2}_", "", stem)


# pairs that intentionally touch/pierce; (allowed penetration mm)
WHITELIST = {
    frozenset(("gp_slider", "tape_b")): 99.0,      # punches paper
    frozenset(("data_punch", "tape_b")): 99.0,     # punches paper
    frozenset(("sprocket_shaft", "tape_a")): 0.3,  # tooth flank chord effect
    frozenset(("sprocket_shaft", "tape_b")): 0.3,
    frozenset(("rocker_c", "swing_arm")): 0.1,     # ball-on-flank drive
    frozenset(("rocker_r", "swing_arm")): 0.1,
    frozenset(("geneva_wheel", "sprocket_shaft")): 0.2,  # keyed together
    frozenset(("spk_clip", "sprocket_shaft")): 0.2,      # clip in groove
    frozenset(("crank", "camshaft")): 0.2,               # keyed together
    frozenset(("hammer", "block")): 0.1,           # stem stop / pad press
    frozenset(("block", "data_punch")): 0.1,       # block on flange (drive)
}
