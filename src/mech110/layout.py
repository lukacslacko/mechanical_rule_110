"""World-coordinate anchors for every feature. Single source of truth for
geometry modules AND pose computation. All mm.

X = tape travel axis (tape moves -X), Y = across machine (shaft axes), Z = up.

Vertical zoning (hard rules discovered during layout):
- y in tape bands (14..39 and 49..74): NOTHING attached to the bed may exist
  between z=60 and z=62.5 except the tapes; all blocks are BRIDGES anchored
  in the corridors y<13, 40..48.5, y>75.5.
- routing layers above the tape B station:
  z 79..82   rocker doglegs ledge->tip runs (later rising to ~89 at tips)
  z 86..94   AND3 guillotine plate (at reader)
  z 90.7..94.2  hammer pad slab (swings/strokes)
  z 95..98.5 swing-arm plate (horizontal interposer arm)
  z >=99     AND3 blocker arm high run + gallows
"""

import math
from .params import P
from . import kinematics as kin
from . import camprofiles as cam

# ---------------------------------------------------------------- Z stack
BASE_Z0, BASE_Z1 = 0.0, 4.0
BED_Z0, BED_Z1 = 52.0, 60.0
PAPER_TOP = cam.PAPER_TOP            # 60.1
STRIPPER_Z = 61.2

SPROCKET_Z = kin.SPROCKET_Z          # 67.74
CAMSHAFT_Z = kin.CAMSHAFT_Z          # 109.31
CAMSHAFT_X = kin.CAMSHAFT_X          # 24
SPROCKET_X = kin.SPROCKET_X          # 0

# ------------------------------------------------------------------- Y map
PLATE_L_Y0, PLATE_L_Y1 = 0.0, 6.0
PLATE_R_Y0, PLATE_R_Y1 = 94.0, 100.0
TAPE_A_Y0, TAPE_A_Y1 = P.tapeA_y0, P.tapeA_y0 + P.tape.width   # 14..39
TAPE_B_Y0, TAPE_B_Y1 = P.tapeB_y0, P.tapeB_y0 + P.tape.width   # 49..74
GUIDE_A_Y = TAPE_A_Y0 + P.tape.guide_track_y    # 19
DATA_A_Y = TAPE_A_Y0 + P.tape.data_track_y      # 32
GUIDE_B_Y = TAPE_B_Y0 + P.tape.guide_track_y    # 54
DATA_B_Y = TAPE_B_Y0 + P.tape.data_track_y      # 67

# ------------------------------------------------------------ reader pins
PIN_XS = {"L": 32.0, "C": 24.0, "R": 16.0}
PIN_Y = DATA_A_Y                                 # 32
PIN_SHAFT_D = P.pin_shaft_d                      # 4
PIN_TIP_D = P.pin_tip_d                          # 3.5
PIN_LEN = 26.0                                   # tip apex .. top
COLLAR_Z0, COLLAR_Z1 = 20.0, 22.5                # local, from tip apex
COLLAR_D = 7.0

COMB_Z0, COMB_Z1 = 61.2, 70.0                    # comb bridge body
TOPGUIDE_Z0, TOPGUIDE_Z1 = 72.0, 75.0
BED_PIN_HOLE_D = 4.6
# comb bridge footprint: body over tape, feet in corridors
COMB_X0, COMB_X1 = 0.0, 48.0
COMB_BODY_Y0, COMB_BODY_Y1 = 26.0, 38.0
COMB_FOOT_S_Y0, COMB_FOOT_S_Y1 = 8.5, 13.0
COMB_FOOT_N_Y0, COMB_FOOT_N_Y1 = 40.0, 44.0

# ------------------------------------------------------------------- bail
BAIL_PLATE_Y0, BAIL_PLATE_Y1 = 26.8, 28.2   # in front of collars (28.5+)
BAIL_X0, BAIL_X1 = 4.0, 44.0
BAIL_FINGER_Y0, BAIL_FINGER_Y1 = 28.5, 35.5      # fingers at z<=83.2 (low)
BAIL_WINDOW_DROP = 30.0
BAIL_POST_XS = (6.0, 42.0)
BAIL_POST_Y = 30.2          # post bosses on the bail are clipped y 28.4..31
BAIL_POST_D = 5.0

# ------------------------------------------------------------------ cams
# standard construction per cam: hub disc backs the rib; follower plate sits
# BESIDE the rib band, bottom boss cantilevered into it, top boss = key pin
# inserted through the plate along Y.
RIB_HALF = 1.5
BOSS_R = 2.5
BOSS_OFF = RIB_HALF + BOSS_R + 0.3   # 4.3 boss center offset from window ctr

CAM_BAIL = dict(hub=(18.0, 23.5), rib=(23.5, 26.5), hub_r=24.3,
                rib_contact=(23.7, 26.5),
                plate=(27.0, 28.4))               # plate on +Y side
CAM_GP = dict(hub=(39.8, 44.6), rib=(44.6, 47.8), hub_r=27.3,
              rib_contact=(44.8, 47.8),
              plate=(48.3, 51.3))                 # plate on +Y side, SPOKED hub
CAM_HAM = dict(hub=(52.5, 57.0), rib=(57.0, 60.5), hub_r=20.6,
               rib_contact=(57.5, 60.5),
               plate=(61.0, 63.2))                # plate on +Y side, SPOKED hub
GP_WINDOW_DROP = 27.5
# >= 7.0 so the hammer's window boss clears the interposer block top in
# every phase (boss-bottom - block-top = RISE - 6.8 + 0.1)
HAM_WINDOW_RISE = 7.0
# both slide-in keys insert from -Y at x=24 (gp hub is SPOKED so the key
# paths thread between its spokes at the assembly angle); values at the
# bottom of this file.

# ---------------------------------------------------------------- punches
PUNCH_X = P.punch_x                              # 24
GP_Y = GUIDE_B_Y                                 # 54
DP_Y = DATA_B_Y                                  # 67
PUNCH_TIP_UP = cam.PUNCH_TIP_UP                  # 64.1
PUNCH_TIP_DOWN = cam.PUNCH_TIP_DOWN              # 57.0
DP_LEN = cam.DATA_PUNCH_LEN                      # 18: tip..flange top
DP_FLANGE_D, DP_FLANGE_H = 8.0, 3.0
DP_SHANK_D = 4.0
BLOCK_W, BLOCK_L, BLOCK_H = 6.0, 6.0, cam.PEND_BLOCK_H
BLOCK_Z0 = PUNCH_TIP_UP + DP_LEN + 0.1           # 82.2 rest (bottom)
PAD_FACE_REST = BLOCK_Z0 + BLOCK_H + 0.5         # 90.7
BLOCK_STEM_D = 4.0
PAD_SLAB_Z1 = PAD_FACE_REST + 3.5                # 94.2
LIP_TOP = PUNCH_TIP_UP + DP_LEN - DP_FLANGE_H - 0.5   # 78.6
HAM_WALL_X0, HAM_WALL_X1 = 16.7, 19.7            # -X side wall/web
HAM_SHANK_XS = (14.0, 35.0)                      # dual guide shanks, d5
GP_SHANK_X, GP_DUMMY_X = 24.0, 31.0              # gp slider shanks, d4

# punch block = bridge over tape B
PB_X0, PB_X1 = 10.5, 38.5   # west edge clears sprocket B orbit (10.1)
PB_Z0, PB_Z1 = 61.2, 78.0
PB_FOOT_S_Y0, PB_FOOT_S_Y1 = 44.0, 48.5
PB_FOOT_N_Y0, PB_FOOT_N_Y1 = 75.5, 80.0
DIE_GP_D, DIE_DP_D = 5.0, 5.9   # > pyramid tip diagonals (4.53 / 5.66)

# data punch return rocker (idle support): axle along X on north extension
RR_PIVOT_Y, RR_PIVOT_Z = 84.0, 78.5
RR_TIP_Y = 71.5                                  # pad under flange +Y edge
RR_ARM = RR_PIVOT_Y - RR_TIP_Y                   # 12.5

# ------------------------------------------------------------ logic train
AXLE_Y, AXLE_Z = 50.0, 86.0          # rocker axle (along X)
AXLE_D = 5.0
AXLE_X0, AXLE_X1 = 49.0, 64.0        # axle rod span (+X of the cam stack)
AXLE_POST_XS = (49.5, 63.5)          # post centers; columns y 44.6..47.5
ROCKER_HUB_C = (52.0, 56.0)          # hub x-bands on the axle
ROCKER_HUB_R = (57.0, 61.0)
ROCKER_LEDGE_Y0, ROCKER_LEDGE_Y1 = 32.4, 35.2    # pads on collar +Y half
# rockers route +X around the entire cam stack (y 39.8..60.5 is impassable
# below z~95 within x -4..52 once the cams sweep):
# pad -> run +X at y 33..36 (C) / dogleg (R) -> turn +Y at x 53.5..60.5
# -> tips at (61.3, 65.5/68.5)
AND3_FOOT_Y0, AND3_FOOT_Y1 = 30.9, 32.0          # pads on collar -Y half
DOGLEG_Z0, DOGLEG_Z1 = 83.5, 86.5    # rocker arm running band (reader side)
DOGLEG_LOW_Z0, DOGLEG_LOW_Z1 = 79.0, 82.0   # dive band across y 44..58
TIP_C_Y = AXLE_Y + kin.ROCKER_ARM_TIP_C          # 65.3
TIP_R_Y = AXLE_Y + kin.ROCKER_ARM_TIP_R          # 68.7

# swing arm (interposer): plate z 95..98.5, hangs block on stem
ARM_PLATE_Z0, ARM_PLATE_Z1 = 95.0, 98.5
PIVOT_X, PIVOT_Y = 66.0, 67.0
PIVOT_PIN_D = 4.0
FOOT_R = 5.0                          # inclined-slot ring radius
TIP_X = PIVOT_X + math.sqrt(FOOT_R**2 - 1.5**2)  # ~70.8 (east side)
TIP_BALL_D = 2.5
TIP_YS = (PIVOT_Y - 1.5, PIVOT_Y + 1.5)          # C at 65.5, R at 68.5
BLOCK_IN = (PUNCH_X, DP_Y)
_a = math.radians(kin.IN_ANGLE)


def rot_about_pivot(px, py, ang_rad):
    dx, dy = px - PIVOT_X, py - PIVOT_Y
    c, s = math.cos(ang_rad), math.sin(ang_rad)
    return (PIVOT_X + c * dx - s * dy, PIVOT_Y + s * dx + c * dy)


BLOCK_OUT = rot_about_pivot(*BLOCK_IN, -_a)      # ~ (24.9, 75.5)
GALLOWS_XY = (66.0, 80.0)            # column outside tape B; arm to pivot
GALLOWS_ARM_Z0, GALLOWS_ARM_Z1 = 99.5, 102.5

# AND3 guillotine
AND3_PLATE_Y0, AND3_PLATE_Y1 = 31.5, 34.5
AND3_X0, AND3_X1 = 2.0, 46.0
AND3_PLATE_Z0, AND3_PLATE_Z1 = 86.0, 94.0        # at read-rest reference!
AND3_RAIL_XS = (2.0, 46.0)
AND3_REF_COLLAR_TOP = cam.PIN_TIP_ON_PAPER + COLLAR_Z1   # 82.6 (read-rest)
# blocker arm route (all at read-rest z; whole part moves with the plate):
# S1: x 36..40, y 34.5..46, z 92..97
# S2: x 36..40, y 44..48, z 79..97 (descend ahead of axle)
# S3: x 36..40, y 48..57.5, z 79..82 (duck under axle & gp swept zone)
# S4: x 34..38, y 60..63, z 79..102 (riser between stations)
# S5: x 34..38, y 63..68, z 99..102 (high run over swing arm)
# finger: d4 at (36, ~66.9), tip down to 96.9 when fully engaged
FINGER_D = 4.0

# ----------------------------------------------------------------- geneva
GENEVA_WHEEL_Y0, GENEVA_WHEEL_Y1 = -13.0, -9.0
DRIVER_DISC_Y0, DRIVER_DISC_Y1 = -18.0, -13.0
GENEVA_PIN_D = 5.0
LOCK_RING_R = 14.0
WHEEL_HUB_Y0, WHEEL_HUB_Y1 = -9.0, -1.0
WHEEL_SLOT_W = GENEVA_PIN_D + 0.5
WHEEL_SLOT_INNER_R = 21.0
HEX_AF = 7.0                          # shaft hex stubs across flats

# ------------------------------------------------------------------ frame
PLATE_X0, PLATE_X1 = -42.0, 82.0
PLATE_Z1 = 127.0
BED_X0, BED_X1 = -40.0, 78.0
BED_Y0, BED_Y1 = 6.0, 94.0
BASE_X0, BASE_X1 = -46.0, 86.0
BASE_Y0, BASE_Y1 = -22.0, 104.0      # extends under geneva wheel side
SHAFT_D = P.shaft_d                  # 8
SLOT_W = 8.5

SPK_PITCH_R = P.sprocket.pitch_radius(P.tape.pitch)      # 7.64
SPK_TIP_R = 9.8
SPK_ROOT_R = 6.8
SPK_BODY_HALF_Y = 3.0
BED_GROOVE_W = 3.4
BED_GROOVE_X0, BED_GROOVE_X1 = -12.0, 12.0
BED_GROOVE_Z = 57.4

# tape rails on bed (z 60..62) just outside each tape edge
RAILS_Y = ((12.8, 13.8), (39.2, 40.2), (47.8, 48.8), (74.4, 75.4))

TAPE_LEN_X0, TAPE_LEN_X1 = -38.0, 76.0

# -------------------------------------------------- direction & geneva
TAPE_DIR = -1.0   # tape travels -X (Geneva/crank chirality)
CENTER_LINE_ANGLE = math.degrees(math.atan2(SPROCKET_Z - CAMSHAFT_Z,
                                            SPROCKET_X - CAMSHAFT_X))  # -120
GENEVA_PIN_GAMMA = CENTER_LINE_ANGLE - 60.0                            # -180
WHEEL_SLOT0_ANGLE = math.degrees(math.atan2(CAMSHAFT_Z - SPROCKET_Z,
                                            CAMSHAFT_X - SPROCKET_X)) + 30.0  # 90
WHEEL_ARC_ANGLE0 = WHEEL_SLOT0_ANGLE - 30.0      # concave locking arcs at 60+k*60


def bail_window_z(theta: float) -> float:
    return cam.bail_tip_z(theta) + BAIL_WINDOW_DROP


def gp_window_z(theta: float) -> float:
    return cam.guide_punch_tip_z(theta) + GP_WINDOW_DROP


def ham_window_z(theta: float) -> float:
    return cam.hammer_face_z(theta) + HAM_WINDOW_RISE


KEY_GP_Z = gp_window_z(0.0) + BOSS_OFF      # 96.4
KEY_HAM_Z = ham_window_z(0.0) + BOSS_OFF    # 102.0
# free corridor checks for the ham key's 3-segment insertion path:
GP_PLATE_ZTOP = 98.9                        # ham key passes over at z>=99.5
