"""Punch-station parts for MECH-110: punch block (bridge over tape B),
data punch pin, interposer block + stem clip, swing arm (vertical-pivot
interposer carrier), gallows post + pivot clip, and the data-punch
return rocker with its axle.

All parts are modeled in WORLD coordinates (mm) at the reference state
(theta = 0, bits = (0,0,0)): swing arm and block at OUT, punches up,
return rocker at punch-rest. build123d algebra mode; helpers from .geom.

The punch block is a BRIDGE over tape B: feet in the corridors
y 44..48.5 / 75.5..80, nothing solid between z 60..62.5 over the tape
except the stripper pad underside hovering at z 61.2.
"""

import math

from build123d import Box, Part, Plane, Polygon, Pos, Rotation, extrude

from . import kinematics as kin
from . import layout as L
from .geom import box_at, cyl_x, cyl_z, pyramid_tip

# ----------------------------------------------------- local constants
PEG_D, PEG_Z0 = 5.0, 56.0
PB_PEGS = ((12.0, 46.25), (34.0, 46.25), (12.0, 77.75), (34.0, 77.75))
GP_BORE_D = 4.5                       # guide-punch slider shank bores
DP_BORE_D = 4.6                       # data punch shank bore
HAM_BORE_D = 5.6                      # hammer guide shank bores
EAR_XS = ((20.0, 22.5), (25.5, 28.0))  # return-rocker pivot ears
# spec'd ears (y 82.5..85.5, z 70..81) are severed by their own d4.2 axle
# hole (4.2 > 3.0 y-width); widened/raised for 0.9 / 1.4 mm hole walls
EAR_Y0, EAR_Y1, EAR_Z0, EAR_Z1 = 81.0, 87.0, 70.0, 82.0
RR_AXLE_HOLE_D = 4.2

DP_TIP_H = 3.0                        # pyramid tip height
DP_FLANGE_Z1 = L.PUNCH_TIP_UP + L.DP_LEN              # 82.1
DP_FLANGE_Z0 = DP_FLANGE_Z1 - L.DP_FLANGE_H           # 79.1

BLOCK_Z1 = L.BLOCK_Z0 + L.BLOCK_H                     # 90.4
STEM_TOP = 99.5
STEM_GROOVE_Z0, STEM_GROOVE_Z1 = 98.7, 99.4
STEM_GROOVE_D = 3.2

PIN_GROOVE_Z0, PIN_GROOVE_Z1 = 93.6, 94.3
PIN_GROOVE_D = 3.2

# swing-arm foot ring / inclined tip slots
SKIRT_Z0, SKIRT_Z1 = 89.0, 95.5
SKIRT_RO, SKIRT_RI = 6.6, 3.4
SLOT_R0, SLOT_R1 = 3.2, 6.8           # radial extent of slot channel
TIP_REST_Z = 87.45                    # ball CENTER z at read-rest
BALL_R = L.TIP_BALL_D / 2
# tangential extent: +4.5 per spec; the -t end wall is trimmed to the
# ball's swept reach (arm-frame travel r*IN_ANGLE, plus ball radius and
# clearance) instead of -4.5: the two slots sit only ~3 mm apart (35 deg
# at r 5) and a full -4.5 channel, whose ceiling rises toward -t, would
# wipe out the NEIGHBOR slot's push flank entirely (verified: ball R
# then never contacts a flank and the R bit could not swing the arm).
SLOT_T_NEG = -(L.FOOT_R * math.radians(kin.IN_ANGLE) + BALL_R + 0.16)
SLOT_T_POS = 4.5                      # ~-2.70 .. +4.5
SLOT_VGAP = 4.2                       # vertical flank gap (normal ~3.0)
# Push flank plane height at t=0. The spec prose says "ball top + lag"
# (= 89.85), but kinematics consumes the FULL 1.15 TIP_LAG of vertical
# rise before the arm starts turning; a d2.5 sphere meets a 45-deg plane
# 1.25*(sqrt(2)-1) = 0.518 earlier than its top point does, so at 89.85
# every kin-posed drive would penetrate the flank ~0.34 normal. Adding
# the sphere correction puts first contact at exactly TIP_LAG rise and
# the kin-posed swing rides the flank with a 0..0.05 hover (the project
# convention for one-way contacts).
SLOT_FLANK_Z0 = (TIP_REST_Z + BALL_R + kin.TIP_LAG +
                 (math.sqrt(2) - 1) * BALL_R)         # 90.37
SLOT_Z_CLIP = 95.4                    # never cut the plate above

RR_CW_Y0, RR_CW_Y1 = 84.0, 90.0       # counterweight
RR_HUB_X0, RR_HUB_X1 = 22.7, 25.3


# ------------------------------------------------------ 1. punch block

def _punch_block() -> Part:
    add = [
        # bridge body over tape + stripper pad (underside = stripper, 61.2)
        box_at(L.PB_X0, L.PB_X1, 49.0, 74.0, 62.5, L.PB_Z1),
        box_at(18.0, 30.0, 50.0, 71.0, L.PB_Z0, L.PB_Z1),
        # feet in the corridors, straps closing the gaps over the rails
        box_at(L.PB_X0, L.PB_X1, L.PB_FOOT_S_Y0, L.PB_FOOT_S_Y1,
               60.0, L.PB_Z1),
        # west stripper finger: holds tape B down east of sprocket B
        box_at(L.PB_X0, 18.0, 49.0, 59.0, 61.4, L.PB_Z1),
        box_at(L.PB_X0, L.PB_X1, L.PB_FOOT_N_Y0, L.PB_FOOT_N_Y1,
               60.0, L.PB_Z1),
        box_at(L.PB_X0, L.PB_X1, L.PB_FOOT_S_Y1, 49.0, 62.5, L.PB_Z1),
        box_at(L.PB_X0, L.PB_X1, 74.0, L.PB_FOOT_N_Y0, 62.5, L.PB_Z1),
        # north extension + pivot ears for the return rocker
        box_at(18.0, 30.0, 80.0, 86.0, 60.0, 72.0),
    ]
    add += [box_at(x0, x1, EAR_Y0, EAR_Y1, EAR_Z0, EAR_Z1)
            for x0, x1 in EAR_XS]
    add += [cyl_z(x, y, PEG_Z0, 60.0, PEG_D) for x, y in PB_PEGS]
    # shelf rail: supports the interposer block at OUT, top flush with the
    # flange top + witness (82.1); column on the N foot, cantilever south
    add += [box_at(27.2, 29.2, 75.5, 77.4, 60.0, 82.1),
            box_at(27.2, 29.2, 72.0, 75.7, 79.0, 82.1)]
    cut = [
        # rocker axle hole through both ears
        cyl_x(L.RR_PIVOT_Y, L.RR_PIVOT_Z, 19.0, 29.0, RR_AXLE_HOLE_D),
        # vertical guide bores, through everything
        cyl_z(L.GP_SHANK_X, L.GP_Y, 60.0, 79.0, GP_BORE_D),
        cyl_z(17.0, 47.5, 56.0, 79.0, GP_BORE_D),   # dummy dive bore (corridor)
        cyl_z(L.PUNCH_X, L.DP_Y, 60.0, 79.0, DP_BORE_D),
        # well pockets (open top); guidance kept: gp bore 61.2..70.5,
        # dp bore 61.2..69.3, hammer shanks full length (outside wells)
        box_at(15.9, 30.2, 61.3, 72.7, 69.3, 78.5),       # hammer well
        box_at(14.5, 33.5, 45.4, 59.8, 70.5, 78.5),       # gp well
        box_at(21.3, 26.7, 44.8, 45.5, 76.4, 78.6),       # gp boss dip
        # rail notch: the S foot crosses the bed rail at y 47.8..48.8
        box_at(L.PB_X0 - 1, L.PB_X1 + 1, 47.5, 49.1, 59.0, 62.3),
        # relief channels: the and3 low runs / rocker R diagonal descend to
        # ~76.4 at full pin drop
        box_at(8.6, 12.6, 38.0, 58.6, 76.2, 78.6),   # and3 S3 (west)
        box_at(8.6, 53.5, 56.0, 58.6, 76.2, 78.6),   # and3 S3b
        box_at(31.0, 53.5, 43.6, 45.2, 76.2, 78.6),  # rocker R diagonal
        # return-rocker arm channel (y0 72.6 not 73.0: meets the hammer
        # well at y 72.7 so the dipping rocker arm crosses no 0.3 sliver)
        box_at(21.0, 27.0, 72.6, 80.5, 69.5, 78.5),
    ]
    cut += [cyl_z(x, L.DP_Y, 60.0, 79.0, HAM_BORE_D) for x in L.HAM_SHANK_XS]
    return add[0] + add[1:] - cut


# ------------------------------------------------------- 2. data punch

def _data_punch() -> Part:
    x, y = L.PUNCH_X, L.DP_Y
    tip = Pos(x, y, L.PUNCH_TIP_UP) * pyramid_tip(L.DP_SHANK_D, DP_TIP_H)
    # trim the square's corners to the shank cylinder so the tip passes
    # the d4.6 guide bore (square half-diagonal 2.83 > bore r 2.3)
    tip &= cyl_z(x, y, L.PUNCH_TIP_UP - 0.1, L.PUNCH_TIP_UP + DP_TIP_H,
                 L.DP_SHANK_D)
    p = tip
    p += cyl_z(x, y, L.PUNCH_TIP_UP + DP_TIP_H, DP_FLANGE_Z0, L.DP_SHANK_D)
    p += cyl_z(x, y, DP_FLANGE_Z0, DP_FLANGE_Z1, L.DP_FLANGE_D)
    return p


# ------------------------------------- 3. interposer block (at OUT pose)

def _block() -> Part:
    # No clip: the block rides the shelf rail (OUT) / flange top (IN); the
    # stem reaches z 105 so it stays engaged in the arm bore through the
    # full 7.3 punch descent (camshaft core bottom is 105.31).
    bx, by = L.BLOCK_OUT
    p = box_at(bx - L.BLOCK_W / 2, bx + L.BLOCK_W / 2,
               by - L.BLOCK_L / 2, by + L.BLOCK_L / 2, L.BLOCK_Z0, BLOCK_Z1)
    p += cyl_z(bx, by, BLOCK_Z1, 105.0, L.BLOCK_STEM_D)
    return p


# ----------------------------------------------------------- 4/7. clips

def _c_clip(x, y, z0, z1, od, bore_d) -> Part:
    p = cyl_z(x, y, z0, z1, od)
    p -= cyl_z(x, y, z0 - 0.1, z1 + 0.1, bore_d)
    # C opening: 3.0-wide slot from center toward +Y (snap onto d3.2 groove)
    p -= box_at(x - 1.5, x + 1.5, y, y + od, z0 - 0.1, z1 + 0.1)
    return p


# ---------------------------------------- 5. swing arm (at OUT attitude)

def _tip_slot_void(tip_x, tip_y) -> Part:
    """Inclined channel through the foot skirt for one rocker tip ball.

    Local frame at azimuth a: x = tangent t_hat = (-sin a, cos a), the
    direction material moves for + (CCW, IN-swing) arm rotation; extrude
    along the outward radial. Both flanks descend with dz/dt = -1, so a
    rising ball pushes the arm IN and a descending ball pulls it OUT.
    The lower flank stays below the skirt bottom (z 89) over the whole
    t range, so the channel is open to the bottom face (ball loads from
    below at assembly).
    """
    a = math.atan2(tip_y - L.PIVOT_Y, tip_x - L.PIVOT_X)
    t_hat = (-math.sin(a), math.cos(a), 0.0)
    rad = (math.cos(a), math.sin(a), 0.0)
    z0 = SLOT_FLANK_Z0                                    # 90.37
    t0, t1 = SLOT_T_NEG, SLOT_T_POS
    pts = [(t0, z0 - t0), (t1, z0 - t1),
           (t1, z0 - t1 - SLOT_VGAP), (t0, z0 - t0 - SLOT_VGAP)]
    origin = (L.PIVOT_X + SLOT_R0 * rad[0],
              L.PIVOT_Y + SLOT_R0 * rad[1], 0.0)
    sk = Plane(origin=origin, x_dir=t_hat, z_dir=rad) * Polygon(
        *pts, align=None)
    void = extrude(sk, amount=SLOT_R1 - SLOT_R0, dir=rad)
    # never cut into the arm plate above (no-op here: void top 94.35)
    return void & box_at(L.PIVOT_X - 10, L.PIVOT_X + 10,
                         L.PIVOT_Y - 10, L.PIVOT_Y + 10, 75.0, SLOT_Z_CLIP)


def _swing_arm(cut_slots: bool = True) -> Part:
    bx, by = L.BLOCK_OUT
    px, py = L.PIVOT_X, L.PIVOT_Y
    z0, z1 = L.ARM_PLATE_Z0, L.ARM_PLATE_Z1
    # hub, stem boss, and the straight bar between them (slot-shaped plan:
    # bar box + the two d9 cylinders give the rectangle-with-round-ends);
    # foot ring skirt overlaps the plate band 95..95.5 to weld
    ang = math.degrees(math.atan2(by - py, bx - px))
    # stem boss is d7 (not d9): at swing-IN it reaches y 63.5, which must
    # clear the hammer fork plate ending at y 63.2
    p = cyl_z(px, py, z0, z1, 9.0) + [
        cyl_z(bx, by, z0, z1, 7.0),
        Pos((px + bx) / 2, (py + by) / 2, (z0 + z1) / 2) *
        Rotation(0, 0, ang) *
        Box(math.hypot(bx - px, by - py), 7.0, z1 - z0),
        cyl_z(px, py, SKIRT_Z0, SKIRT_Z1, 2 * SKIRT_RO) -
        cyl_z(px, py, SKIRT_Z0 - 0.1, SKIRT_Z1 + 0.1, 2 * SKIRT_RI),
    ]
    # bores (again, through the welded plate, to be safe)
    p -= cyl_z(px, py, z0 - 0.5, z1 + 0.5, L.PIVOT_PIN_D + 0.6)   # 4.6
    p -= cyl_z(bx, by, z0 - 0.5, z1 + 0.5, 4.4)   # stem slides vertically
    if cut_slots:
        for ty in L.TIP_YS:
            p -= _tip_slot_void(L.TIP_X, ty)
    # no IN hard-stop tab: kinematics caps the swing at IN_ANGLE; the
    # physical stop comes from the rocker/groove geometry (see report)
    return p


# ----------------------------------------------------------- 6. gallows

def _gallows() -> Part:
    # column down to the bed doubles as the foot (same 62..70 x 76..84
    # footprint, z 60..103); pegs into the bed
    p = box_at(62.0, 70.0, 76.0, 84.0, 60.0, 103.0)
    for x in (64.0, 68.0):
        p += cyl_z(x, 80.0, PEG_Z0, 60.0, PEG_D)
    # arm over the pivot, pivot pin hanging from its underside
    p += box_at(62.0, 70.0, 64.0, 80.0, L.GALLOWS_ARM_Z0, L.GALLOWS_ARM_Z1)
    p += cyl_z(L.PIVOT_X, L.PIVOT_Y, 92.0, L.GALLOWS_ARM_Z0 + 1.0,
               L.PIVOT_PIN_D)
    ring = (cyl_z(L.PIVOT_X, L.PIVOT_Y, PIN_GROOVE_Z0, PIN_GROOVE_Z1, 6.0) -
            cyl_z(L.PIVOT_X, L.PIVOT_Y, PIN_GROOVE_Z0, PIN_GROOVE_Z1,
                  PIN_GROOVE_D))
    return p - ring


# ---------------------------------- 8. return rocker (punch-rest attitude)

def _return_rocker() -> Part:
    y, z = L.RR_PIVOT_Y, L.RR_PIVOT_Z
    p = cyl_x(y, z, RR_HUB_X0, RR_HUB_X1, 9.0)            # hub
    p += box_at(RR_HUB_X0, RR_HUB_X1, 72.5, 84.0, 76.8, 79.2)     # arm
    p += box_at(RR_HUB_X0, RR_HUB_X1, L.RR_TIP_Y - 1.0, 72.5,
                77.5, 79.0)   # tip pad; top 79.0 = flange 79.1 - witness
    # counterweight kept to the hub's x band (not 21..27) so it clears
    # the pivot ears (x 20..22.5 / 25.5..28) at every rocker angle
    # counterweight sized so CM sits BEHIND the pivot (tip biased up)
    p += box_at(RR_HUB_X0, RR_HUB_X1, 84.5, 87.0, 72.3, 81.8)
    p += box_at(20.5, 27.5, 87.0, 91.0, 73.0, 80.0)
    # closed bore for the separate axle pin (no C-opening)
    p -= cyl_x(y, z, RR_HUB_X0 - 3.0, RR_HUB_X1 + 4.0, 4.4)
    return p


def _rr_axle() -> Part:
    y, z = L.RR_PIVOT_Y, L.RR_PIVOT_Z
    p = cyl_x(y, z, 19.2, 29.0, 4.0)
    p += cyl_x(y, z, 17.7, 19.2, 7.0)                     # head, -X end
    return p


# --------------------------------------------------------------- build

def build() -> dict[str, Part]:
    bx, by = L.BLOCK_OUT
    return {
        "punch_block": _punch_block(),
        "data_punch": _data_punch(),
        "block": _block(),
        "swing_arm": _swing_arm(),
        "gallows": _gallows(),
        "pivot_clip": _c_clip(L.PIVOT_X, L.PIVOT_Y, 93.65, 94.25, 6.6, 3.3),
        "return_rocker": _return_rocker(),
        "rr_axle": _rr_axle(),
    }
