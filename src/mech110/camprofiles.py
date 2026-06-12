"""Analytic motion functions — the single source of truth.

Cam solids are lofted from these functions and follower poses are computed
from them, so the geometry and the simulation cannot disagree.

Conventions:
- theta: camshaft angle in degrees, [0, 360). One revolution = one cycle.
- All lift functions return *vertical position of the follower contact*
  in machine Z (mm), already anchored to layout heights from params.
- Geneva: psi(theta) = sprocket shaft angle (degrees) within one cycle,
  0 at the dwell before the advance; total advance = 360/slots per cycle.
"""

import math
from .params import P

TWO_PI = 2 * math.pi


def smoothstep(t: float) -> float:
    """C1 ramp 0->1 for t in [0,1] (clamped)."""
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


class Segment:
    """Lift segment: dwell at v0 until a0, ramp to v1 over [a0,a1]."""
    def __init__(self, a0, a1, v0, v1):
        self.a0, self.a1, self.v0, self.v1 = a0, a1, v0, v1

    def value(self, a):
        if a <= self.a0:
            return self.v0
        if a >= self.a1:
            return self.v1
        return self.v0 + (self.v1 - self.v0) * smoothstep((a - self.a0) / (self.a1 - self.a0))


class Profile:
    """Piecewise lift profile over [0,360), cyclic: dwell-ramp-dwell chain."""
    def __init__(self, segments, base):
        self.segments = segments   # ordered, non-overlapping Segments
        self.base = base           # value before the first ramp

    def __call__(self, theta):
        a = theta % 360.0
        v = self.base
        for s in self.segments:
            if a < s.a0:
                break
            v = s.value(a)         # ramp if inside, v1 if past
        return v


def _profile(points):
    """points = [(angle, value), ...] sorted; value holds (dwell) until next
    ramp start; consecutive points define smoothstep ramps."""
    segs = []
    for (a0, v0), (a1, v1) in zip(points, points[1:]):
        if v0 != v1:
            segs.append(Segment(a0, a1, v0, v1))
    return Profile(segs, points[0][1])


# ----------------------------------------------------------------- layout Z
PAPER_TOP = P.tape_z + P.tape.thickness          # 60.1

# Reader pin tip levels
PIN_TIP_UP = PAPER_TOP + P.pin_lift              # bailed up
PIN_TIP_ON_PAPER = PAPER_TOP                     # bit = 0
PIN_TIP_IN_HOLE = PAPER_TOP - P.pin_drop         # bit = 1

# Bail support level is expressed as "pin tip height the bail enforces":
# bail finger under the pin collar; we track everything in tip coordinates.
BAIL_OVERTRAVEL = 0.8     # bail keeps going below the lowest pin stop
BAIL_TIP_LOW = PIN_TIP_IN_HOLE - BAIL_OVERTRAVEL

# Punch tip levels (both punches)
PUNCH_TIP_UP = PAPER_TOP + 4.0
PUNCH_TIP_DOWN = PAPER_TOP - 3.1                 # through paper into die

# Data punch drive chain: hammer face -> interposer block -> punch flange.
# The block rides a shelf rail (OUT) that hands off flush to the punch
# flange top (IN); its stem slides in the swing-arm bore (no clip).
PEND_BLOCK_H = 8.0         # interposer block height
HAMMER_GAP = PEND_BLOCK_H + 0.6   # hammer face to flange top at rest
HAMMER_STROKE = (PUNCH_TIP_UP - PUNCH_TIP_DOWN) + 0.4  # drive + gap pickup


# Data punch pin: head at tip + length
DATA_PUNCH_LEN = 18.0      # tip to head top


def bail_tip_z(theta: float) -> float:
    """Pin-tip height enforced by the bail (pins rest ON it via collars,
    one-way: an individual pin may stop higher on paper)."""
    return _bail(theta)


_bail = _profile([
    (0.0, PIN_TIP_UP),
    (P.phases.pins_lower[0], PIN_TIP_UP),
    (P.phases.pins_lower[1], BAIL_TIP_LOW),
    (P.phases.pins_lift[0], BAIL_TIP_LOW),
    (P.phases.pins_lift[1], PIN_TIP_UP),
    (360.0, PIN_TIP_UP),
])

_punch = _profile([
    (0.0, 0.0),
    (P.phases.punch[0], 0.0),
    ((P.phases.punch[0] + P.phases.punch[1]) / 2, 1.0),   # full depth mid-window
    (P.phases.punch[1], 0.0),
    (360.0, 0.0),
])


def guide_punch_tip_z(theta: float) -> float:
    """Guide punch slider tip height (slider = follower + punch in one part)."""
    return PUNCH_TIP_UP - (PUNCH_TIP_UP - PUNCH_TIP_DOWN) * _punch(theta)


def hammer_face_z(theta: float) -> float:
    """Data hammer underside height. Rest: HAMMER_GAP above punch head rest."""
    rest = PUNCH_TIP_UP + DATA_PUNCH_LEN + HAMMER_GAP
    return rest - HAMMER_STROKE * _punch(theta)


def block_bottom_z(theta: float, enabled: bool) -> float:
    """Interposer block underside. At rest it hangs on its stem collar 0.3
    above the flange; when the hammer face presses it (0.1 model witness),
    it descends with the punch."""
    rest = PUNCH_TIP_UP + DATA_PUNCH_LEN + 0.1
    if not enabled:
        return rest
    return min(rest, hammer_face_z(theta) - PEND_BLOCK_H - 0.1)


def data_punch_tip_z(theta: float, enabled: bool) -> float:
    """Punch pin tip height: rides 0.1 below the block underside when the
    block presses it; held at rest by the return rocker otherwise."""
    if not enabled:
        return PUNCH_TIP_UP
    flange_rest_top = PUNCH_TIP_UP + DATA_PUNCH_LEN
    drive = (flange_rest_top + 0.1) - block_bottom_z(theta, enabled)
    return PUNCH_TIP_UP - max(0.0, drive)


# ------------------------------------------------------------------ Geneva
def geneva_psi(theta: float) -> float:
    """Sprocket-shaft rotation (deg) within one cycle, 0 before the advance.

    Drive window: theta in [0, 2*drive_half_angle] (=[0,120] for 6 slots).
    Mid-drive at theta=60: pin points along the center line, slot radial.
    Wheel turns from -30 to +30 about the center line => normalize to 0..60.
    """
    g = P.geneva
    half = g.drive_half_angle                    # 60
    a = theta % 360.0
    if a >= 2 * half:
        return 360.0 / g.slots                   # dwell, advanced position
    phi = math.radians(a - half)                 # -60..+60 driver angle
    r = g.crank_radius
    d = g.center_distance
    psi = math.degrees(math.atan2(r * math.sin(phi), d - r * math.cos(phi)))
    return psi + 180.0 / g.slots                 # shift -30..30 -> 0..60


def tape_x(theta: float) -> float:
    """Tape displacement within one cycle (0 -> pitch P)."""
    r_p = P.sprocket.pitch_radius(P.tape.pitch)
    return r_p * math.radians(geneva_psi(theta))


# Sanity values importable by tests
ADVANCE_PER_CYCLE = P.tape.pitch
