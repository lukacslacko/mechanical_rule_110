"""Frame parts for MECH-110: base plate, frame plates, bearing caps and
their pins, tape bed (with legs/rails), sprocket hold-down covers, and the
two paper tapes.

All parts are modeled in WORLD coordinates (mm) at the reference state
(theta = 0). build123d algebra mode; geometry helpers from .geom.
"""

import math

from build123d import Part

from . import layout as L
from .geom import box_at, cyl_x, cyl_y, cyl_z

# ----------------------------------------------------- frame-local consts
TAB_XCS = (-30.0, 20.0, 70.0)     # tab / mortise x-centers
TAB_LEN, TAB_W = 20.0, 6.0        # tab size (x, y)
MORTISE_CL = 0.15                 # mortise clearance per side

CAP_PLUG_W = 8.2                  # cap width across the slot (x)
CAP_Y_CL = 0.3                    # total y clearance of cap in plate band
PIN_HOLE_D = 4.2                  # cap-pin clearance holes (along X)
PIN_D, PIN_HALF_LEN = 4.0, 11.0   # cap pin: d4 x 22 long
PIN_HEAD_S, PIN_HEAD_T = 6.0, 1.5
PIN_SPAN = 12.0                   # half-span (x) of plate pin holes
CAM_PIN_ZS = (116.0, 123.0)
SPK_PIN_ZS = (75.0, 95.0)

RAIL_Z1 = 62.0                    # tape rail top
LEG_X, LEG_Y = 10.0, 8.0          # bed leg cross-section
PEG_HOLE_D = 5.4
PEG_HOLE_Z0 = 55.0
PEG_HOLES = (
    (20.0, 10.75), (44.0, 10.75), (20.0, 41.6), (44.0, 41.6),    # comb feet
    (17.0, 47.5),                                                # gp dummy dive
    (12.0, 46.25), (34.0, 46.25), (12.0, 77.75), (34.0, 77.75),  # punch blk
    (49.5, 46.0), (63.5, 46.0),                                  # axle posts
    (64.0, 80.0), (68.0, 80.0),                                  # gallows
    (-13.0, 11.2), (13.0, 11.2), (-13.0, 41.4), (13.0, 41.4),    # cover A
    (-13.0, 46.6), (-6.0, 46.6),                                 # cover B
)
DIE_CBORE_D, DIE_CBORE_H = 9.0, 4.0

COVER_Z0, COVER_Z1 = 61.4, 64.0
BEAM_Z1 = 63.4   # beams cross x=0: sprocket shaft underside is 63.44
COVER_PEG_D, COVER_PEG_Z0 = 4.5, 55.2
COVER_BRIDGE_XS = ((-15.5, -10.5), (10.5, 15.5))
COVER_A_FEET = ((9.5, 13.0), (40.0, 43.5))
COVER_A_PEGS = ((-13.0, 11.2), (13.0, 11.2), (-13.0, 41.4), (13.0, 41.4))
# cover B = WEST bridge only (the punch block's stripper + a west finger
# hold tape B down on the east side; hammer shanks descend at y=67)
COVER_B_FEET = ((44.3, 48.0),)
COVER_B_PEGS = ((-13.0, 46.6), (-6.0, 46.6))
COVER_B_SPAN_Y1 = 62.0
COVER_B_BRIDGE_XS = ((-15.5, -10.5),)
COVER_B_BEAM_X = (-15.5, -3.0)

TAPE_Z0, TAPE_Z1 = 60.02, 60.12
GUIDE_HOLE_D, DATA_HOLE_D = 3.2, 4.0
ROW_PITCH = 8.0


# ------------------------------------------------------------------- base

def _base() -> Part:
    p = box_at(L.BASE_X0, L.BASE_X1, L.BASE_Y0, L.BASE_Y1,
               L.BASE_Z0, L.BASE_Z1)
    for xc in TAB_XCS:
        for y0, y1 in ((L.PLATE_L_Y0, L.PLATE_L_Y1),
                       (L.PLATE_R_Y0, L.PLATE_R_Y1)):
            p -= box_at(xc - TAB_LEN / 2 - MORTISE_CL,
                        xc + TAB_LEN / 2 + MORTISE_CL,
                        y0 - MORTISE_CL, y1 + MORTISE_CL,
                        L.BASE_Z0 - 1, L.BASE_Z1 + 1)
    return p


# ----------------------------------------------------------- frame plates

def _u_slot(p: Part, x, z, y0, y1) -> Part:
    """Open-top U-slot of width SLOT_W ending in a half circle at (x, z),
    with a T-pocket above (the bearing cap is a T-bar slid in along Y)."""
    p -= box_at(x - L.SLOT_W / 2, x + L.SLOT_W / 2, y0 - 1, y1 + 1,
                z, L.PLATE_Z1 + 1)
    p -= cyl_y(x, z, y0 - 1, y1 + 1, L.SLOT_W)
    # cap retention pin holes (d3.2 along Y, pins enter from the outer face)
    for dz in (8.6, 12.6):
        p -= cyl_y(x, z + dz, y0 - 1, y1 + 1, 3.2)
    return p


def _plate(y0, y1) -> Part:
    p = box_at(L.PLATE_X0, L.PLATE_X1, y0, y1, L.BASE_Z1, L.PLATE_Z1)
    for xc in TAB_XCS:                               # bottom tabs
        p += box_at(xc - TAB_LEN / 2, xc + TAB_LEN / 2, y0, y1,
                    0.0, L.BASE_Z1)
    p = _u_slot(p, L.CAMSHAFT_X, L.CAMSHAFT_Z, y0, y1)
    p = _u_slot(p, L.SPROCKET_X, L.SPROCKET_Z, y0, y1)
    return p


# ------------------------------------------------------------ bearing caps

def _cap(xc, z0, y0, y1, pin_zs) -> Part:
    """T-bar: plug fills the U-slot, flanges ride the T-pocket; slides in
    along Y from the machine's inner span. Retained by the shaft load
    direction (up into the plug) + light fit."""
    ym = (y0 + y1) / 2
    half_t = (y1 - y0 - CAP_Y_CL) / 2
    p = box_at(xc - CAP_PLUG_W / 2, xc + CAP_PLUG_W / 2,
               ym - half_t, ym + half_t, z0, L.PLATE_Z1)
    p -= cyl_y(xc, z0, y0 - 1, y1 + 1, L.SLOT_W)    # half-bore for shaft
    for dz in (8.6, 12.6):                          # retention pin holes
        p -= cyl_y(xc, z0 + dz, y0 - 1, y1 + 1, 3.2)
    return p


def _cap_pin() -> Part:
    """One pin design (used 8x): modeled in place for the LEFT plate cam
    position at z=116. Shaft d4 x 22 along X, 6x6x1.5 head at -X end."""
    ym = (L.PLATE_L_Y0 + L.PLATE_L_Y1) / 2
    z = CAM_PIN_ZS[0]
    x0 = L.CAMSHAFT_X - PIN_HALF_LEN
    p = cyl_x(ym, z, x0, L.CAMSHAFT_X + PIN_HALF_LEN, PIN_D)
    p += box_at(x0 - PIN_HEAD_T, x0,
                ym - PIN_HEAD_S / 2, ym + PIN_HEAD_S / 2,
                z - PIN_HEAD_S / 2, z + PIN_HEAD_S / 2)
    return p


# -------------------------------------------------------------------- bed

def _bed() -> Part:
    p = box_at(L.BED_X0, L.BED_X1, L.BED_Y0, L.BED_Y1, L.BED_Z0, L.BED_Z1)
    for y0, y1 in L.RAILS_Y:                        # tape rails on top
        p += box_at(L.BED_X0, L.BED_X1, y0, y1, L.BED_Z1, RAIL_Z1)
    for xc in (L.BED_X0 + 5, L.BED_X1 - 5):         # 4 legs down to base
        for yc in (L.BED_Y0 + 4, L.BED_Y1 - 4):
            p += box_at(xc - LEG_X / 2, xc + LEG_X / 2,
                        yc - LEG_Y / 2, yc + LEG_Y / 2,
                        L.BASE_Z1, L.BED_Z0)
    for gy in (L.GUIDE_A_Y, L.GUIDE_B_Y):           # sprocket grooves
        p -= box_at(L.BED_GROOVE_X0, L.BED_GROOVE_X1,
                    gy - L.BED_GROOVE_W / 2, gy + L.BED_GROOVE_W / 2,
                    L.BED_GROOVE_Z, L.BED_Z1 + 2)
    for x in L.PIN_XS.values():                     # pin dive holes
        p -= cyl_z(x, L.PIN_Y, L.BED_Z0 - 1, L.BED_Z1 + 1, L.BED_PIN_HOLE_D)
    for y, d in ((L.GP_Y, L.DIE_GP_D), (L.DP_Y, L.DIE_DP_D)):   # die holes
        p -= cyl_z(L.PUNCH_X, y, L.BED_Z0 - 1, L.BED_Z1 + 1, d)
        p -= cyl_z(L.PUNCH_X, y, L.BED_Z0 - 1, L.BED_Z0 + DIE_CBORE_H,
                   DIE_CBORE_D)                     # counterbore from below
    for x, y in PEG_HOLES:                          # peg holes (through any
        p -= cyl_z(x, y, PEG_HOLE_Z0, RAIL_Z1 + 0.6, PEG_HOLE_D)  # rails too)
    return p


# ----------------------------------------------------- hold-down covers

def _cover(feet, pegs, span_y1=None, bridge_xs=None, beam_x=None) -> Part:
    s0, s1 = feet[0]
    y_end = feet[-1][1] if len(feet) > 1 else span_y1
    bridge_xs = bridge_xs or COVER_BRIDGE_XS
    bx0 = beam_x[0] if beam_x else COVER_BRIDGE_XS[0][0]
    bx1 = beam_x[1] if beam_x else COVER_BRIDGE_XS[1][1]
    parts = []
    for i, (x0, x1) in enumerate(bridge_xs):
        # the EAST bridge of cover A stops at y 25.6: the comb bridge body
        # (x>=14, holding the tape at the reader) owns y 26..38 there
        ye = 25.6 if (len(bridge_xs) > 1 and i == 1) else y_end
        parts.append(box_at(x0, x1, s0, ye, COVER_Z0, COVER_Z1))
    parts += [box_at(bx0, bx1, y0, y1, COVER_Z0, BEAM_Z1)
              for y0, y1 in feet]                   # side beams at the feet
    parts += [cyl_z(x, y, COVER_PEG_Z0, COVER_Z0, COVER_PEG_D)
              for x, y in pegs]
    p = parts[0]
    for q in parts[1:]:
        p += q
    # clearance notches over the bed rails
    for y0, y1 in L.RAILS_Y:
        p -= box_at(COVER_BRIDGE_XS[0][0] - 0.5, COVER_BRIDGE_XS[1][1] + 0.5,
                    y0 - 0.3, y1 + 0.3, COVER_Z0 - 1.0, 62.3)
    return p


# ------------------------------------------------------------------ tapes

def _row_range():
    k0 = math.ceil(L.TAPE_LEN_X0 / ROW_PITCH)
    k1 = math.floor(L.TAPE_LEN_X1 / ROW_PITCH)
    return range(k0, k1 + 1)


def _tape_hole(x, y, d) -> Part:
    return cyl_z(x, y, TAPE_Z0 - 0.1, TAPE_Z1 + 0.1, d)


def _tape_a(bits) -> Part:
    p = box_at(L.TAPE_LEN_X0, L.TAPE_LEN_X1, L.TAPE_A_Y0, L.TAPE_A_Y1,
               TAPE_Z0, TAPE_Z1)
    bit_rows = {40.0: bits[0], 32.0: bits[1], 24.0: bits[2]}
    for k in _row_range():
        x = ROW_PITCH * k
        p -= _tape_hole(x, L.GUIDE_A_Y, GUIDE_HOLE_D)   # every row
        punched = bit_rows[x] if x in bit_rows else (k % 2 == 0)
        if punched:
            p -= _tape_hole(x, L.DATA_A_Y, DATA_HOLE_D)
    return p


def _tape_b() -> Part:
    p = box_at(L.TAPE_LEN_X0, L.TAPE_LEN_X1, L.TAPE_B_Y0, L.TAPE_B_Y1,
               TAPE_Z0, TAPE_Z1)
    for k in _row_range():
        x = ROW_PITCH * k
        if x <= 24.0:                                   # rows >=32 virgin
            p -= _tape_hole(x, L.GUIDE_B_Y, GUIDE_HOLE_D)
        if x <= 16.0 and k % 2 == 1:
            p -= _tape_hole(x, L.DATA_B_Y, DATA_HOLE_D)
    return p


# ------------------------------------------------------------------ build

def _cap_pin3() -> Part:
    """d3 x 8.5 retention pin + d5 head; built at the LEFT plate cam cap
    lower hole; other 7 are posed instances."""
    z = L.CAMSHAFT_Z + 8.6
    return (cyl_y(L.CAMSHAFT_X, z, 0.0, 5.9, 3.0) +
            cyl_y(L.CAMSHAFT_X, z, -1.2, 0.0, 3.8))


def build(bits=(0, 0, 0)) -> dict[str, Part]:
    return {
        "base": _base(),
        "plate_left": _plate(L.PLATE_L_Y0, L.PLATE_L_Y1),
        "plate_right": _plate(L.PLATE_R_Y0, L.PLATE_R_Y1),
        "cap_cam_left": _cap(L.CAMSHAFT_X, L.CAMSHAFT_Z,
                             L.PLATE_L_Y0, L.PLATE_L_Y1, CAM_PIN_ZS),
        "cap_cam_right": _cap(L.CAMSHAFT_X, L.CAMSHAFT_Z,
                              L.PLATE_R_Y0, L.PLATE_R_Y1, CAM_PIN_ZS),
        "cap_spk_left": _cap(L.SPROCKET_X, L.SPROCKET_Z,
                             L.PLATE_L_Y0, L.PLATE_L_Y1, SPK_PIN_ZS),
        "cap_spk_right": _cap(L.SPROCKET_X, L.SPROCKET_Z,
                              L.PLATE_R_Y0, L.PLATE_R_Y1, SPK_PIN_ZS),
        "bed": _bed(),
        "cap_pin3": _cap_pin3(),
        "cover_a": _cover(COVER_A_FEET, COVER_A_PEGS),
        "cover_b": _cover(COVER_B_FEET, COVER_B_PEGS, COVER_B_SPAN_Y1,
                          COVER_B_BRIDGE_XS, COVER_B_BEAM_X),
        "tape_a": _tape_a(bits),
        "tape_b": _tape_b(),
    }
