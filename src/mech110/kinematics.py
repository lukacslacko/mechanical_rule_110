"""Pose of every moving part as a function of camshaft angle theta and the
three input bits (l, c, r) under the reader pins.

All parts are modeled in WORLD coordinates at their reference state
(theta = 0, all bits = 0); pose(theta, bits) returns a 4x4 transform to
apply on top of that. Quasi-static: every position is an explicit algebraic
function of theta — no integration, no springs. Gravity-settled parts sit a
WITNESS_GAP above their support so the nominal model has zero contact;
the collision sweep then treats ANY mesh intersection as a design error.
"""

import math
import numpy as np

from .params import P
from . import camprofiles as cam

WITNESS_GAP = 0.10   # modeled hover above a physical resting contact

# ---------------------------------------------------------------- helpers

def _T(x=0.0, y=0.0, z=0.0):
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def _rot_y(deg, cx, cz):
    """Rotation about a Y-parallel axis through (cx, *, cz)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    R = np.array([[c, 0.0, s, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [-s, 0.0, c, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])
    return _T(cx, 0, cz) @ R @ _T(-cx, 0, -cz)


def _rot_x(deg, cy, cz):
    """Rotation about an X-parallel axis through (*, cy, cz)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    R = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, c, -s, 0.0],
                  [0.0, s, c, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])
    return _T(0, cy, cz) @ R @ _T(0, -cy, -cz)


# ------------------------------------------------------------ layout const
CAMSHAFT_X = P.reader_center_x                     # -24
CAMSHAFT_Z = None  # filled below
SPROCKET_X = P.sprocket_x
SPROCKET_Z = None

SPROCKET_Z = P.sprocket_axis_z
# center distance must be exact: vertical offset from horizontal offset
_dz = math.sqrt(P.geneva.center_distance ** 2 - (CAMSHAFT_X - SPROCKET_X) ** 2)
CAMSHAFT_Z = SPROCKET_Z + _dz

# ----------------------------------------------------- logic geometry
# The interposer ("swing arm") rotates about a VERTICAL pivot, so the block
# enters the hammer/punch gap at constant height. Rocker tips ride a 45-deg
# V-groove on the arm's foot ring: rising tips push the arm IN, descending
# tips pull it OUT (positive both ways, no springs).
# Rockers are see-saws on a common X-axis axle at y=50: ledge side rests on
# the pin collar (one-way), tip side rises into the foot groove.
ROCKER_ARM_TAB = 15.25    # rocker pivot to foot-rib center (y 34.75)
ROCKER_ARM_TIP_C = 15.5   # rocker C pivot to its groove tip (mm)
ROCKER_ARM_TIP_R = 18.5   # rocker R pivot to its groove tip (mm)
TIP_LAG = 1.5             # vertical gap tip-to-groove-flank at rest
FOOT_GAIN = 4.7           # mm of tip rise per radian of arm swing (r*cos)
BLOCK_ARM = 42.0          # arm pivot to interposer block center (mm)
BLOCK_ZONE_START = 0.8    # AND3 descent at which the blocker enters the arc
BLOCKED_TRAVEL = 1.11     # block tangential travel where the blocker stops it

# Derived (exact):
FULL_DROP = P.pin_drop                                     # 2.5
_RISE_C = ROCKER_ARM_TIP_C / ROCKER_ARM_TAB                # tip rise per drop
_RISE_R = ROCKER_ARM_TIP_R / ROCKER_ARM_TAB
EFF_LIFT_C = FULL_DROP * _RISE_C - TIP_LAG                 # 0.975
EFF_LIFT_R = FULL_DROP * _RISE_R - TIP_LAG                 # 1.297
# Hard stop at the angle the WEAKER rocker (C) can just reach; the R rocker
# stalls against the stop (gravity-scale forces, nothing breaks).
IN_ANGLE = math.degrees(math.atan2(EFF_LIFT_C, FOOT_GAIN))           # 14.80
IN_TRAVEL = BLOCK_ARM * math.sin(math.radians(IN_ANGLE))             # 10.73
BLOCKED_ANGLE = math.degrees(math.asin(BLOCKED_TRAVEL / BLOCK_ARM))  # 1.51
# physical hard stop: the block's stem meets the hammer-pad slot end 0.1mm
# before perfect alignment; the model caps the swing at the same angle.
STOP_ANGLE = IN_ANGLE - math.degrees(0.1 / BLOCK_ARM)


# ------------------------------------------------------------- pin levels

def pin_tip_z(theta: float, bit: int) -> float:
    """Reader pin tip height: rides the bail (one-way) or rests on its stop."""
    stop = cam.PIN_TIP_IN_HOLE if bit else cam.PIN_TIP_ON_PAPER
    return max(cam.bail_tip_z(theta), stop + WITNESS_GAP)


def pin_drop(theta: float, bit: int) -> float:
    """Descent below paper-rest level (>=0)."""
    return cam.PIN_TIP_ON_PAPER + WITNESS_GAP - pin_tip_z(theta, bit)


# ------------------------------------------------------------- logic train

def and3_descent(theta: float, bits) -> float:
    """AND3 guillotine descent = min of the three pin drops (rests on the
    highest tab)."""
    return min(pin_drop(theta, b) for b in bits)


def blocker_engaged(theta: float, bits) -> bool:
    return and3_descent(theta, bits) >= BLOCK_ZONE_START


def pendulum_angle(theta: float, bits) -> float:
    """Swing-arm angle (deg, 0 = OUT). Driven by whichever rocker tip rose
    higher in the foot groove, capped by the IN hard stop, and by the AND3
    blocker when engaged."""
    l, c, r = bits
    lift = max(pin_drop(theta, c) * _RISE_C,
               pin_drop(theta, r) * _RISE_R) - TIP_LAG
    if lift <= 0:
        return 0.0
    ang = math.degrees(math.atan2(lift, FOOT_GAIN))
    ang = min(ang, STOP_ANGLE)                    # hard stop (stem on slot end)
    if blocker_engaged(theta, bits):
        ang = min(ang, BLOCKED_ANGLE)             # veto
    return ang


def rocker_angle(theta: float, bits, which: str) -> float:
    """Rocker rotation (deg) about its X-axis pivot. Follows its pin collar
    down by gravity, but hangs on the swing-arm groove if that has stopped
    (one-way at both contacts; the collar simply descends away underneath)."""
    l, c, r = bits
    bit = c if which == "C" else r
    rise_ratio = _RISE_C if which == "C" else _RISE_R
    drop = pin_drop(theta, bit)
    free = math.degrees(math.atan2(drop, ROCKER_ARM_TAB))
    # tip rise cannot exceed lag + groove flank position (arm may be stopped)
    arm = pendulum_angle(theta, bits)
    max_rise = TIP_LAG + FOOT_GAIN * math.tan(math.radians(arm))
    stall = math.degrees(math.atan2(max_rise / rise_ratio, ROCKER_ARM_TAB))
    return min(free, stall)


def punch_enabled(theta: float, bits) -> bool:
    """Interposer block fully in the hammer-punch gap (at the stop)?"""
    return pendulum_angle(theta, bits) >= STOP_ANGLE - 1e-9


# ------------------------------------------------------------- tape motion

def tape_dx(theta: float) -> float:
    return cam.tape_x(theta)


# --------------------------------------------------------------- pose API

def poses(theta: float, bits=(0, 0, 0)) -> dict[str, np.ndarray]:
    """4x4 transforms (vs reference state) for every moving part."""
    l, c, r = bits
    out = {}
    out["camshaft"] = _rot_y(-theta, CAMSHAFT_X, CAMSHAFT_Z)
    out["crank"] = out["camshaft"]
    psi = cam.geneva_psi(theta)
    out["sprocket_shaft"] = _rot_y(-psi, SPROCKET_X, SPROCKET_Z)
    dx = tape_dx(theta)
    out["tape_a"] = _T(x=dx)
    out["tape_b"] = _T(x=dx)
    out["bail"] = _T(z=cam.bail_tip_z(theta) - cam.PIN_TIP_UP)
    for name, bit in (("pin_l", l), ("pin_c", c), ("pin_r", r)):
        out[name] = _T(z=pin_tip_z(theta, bit) - cam.PIN_TIP_UP)
    out["and3"] = _T(z=-and3_descent(theta, bits))
    # rockers and pendulum get their pivot positions from geometry module at
    # build time; kinematics exposes angles, assembly.py turns them into 4x4s.
    out["_rocker_c_angle"] = rocker_angle(theta, bits, "C")
    out["_rocker_r_angle"] = rocker_angle(theta, bits, "R")
    out["_pendulum_angle"] = pendulum_angle(theta, bits)
    out["guide_punch"] = _T(z=cam.guide_punch_tip_z(theta) - cam.PUNCH_TIP_UP)
    out["hammer"] = _T(z=cam.hammer_face_z(theta) - cam.hammer_face_z(0.0))
    en = punch_enabled(theta, bits)
    out["data_punch"] = _T(z=cam.data_punch_tip_z(theta, en) - cam.PUNCH_TIP_UP)
    return out


def machine_punches(bits) -> bool:
    """End-of-cycle outcome for a full revolution with these bits: does the
    data punch reach through the paper? (Evaluated at punch bottom.)"""
    theta_bottom = sum(P.phases.punch) / 2
    en = punch_enabled(theta_bottom, bits)
    tip = cam.data_punch_tip_z(theta_bottom, en)
    return tip < P.tape_z   # below tape underside = pierced
