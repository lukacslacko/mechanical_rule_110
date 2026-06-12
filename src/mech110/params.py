"""Single source of truth for every dimension in MECH-110 (mm, degrees).

Coordinate system: X = tape travel, Y = across tape / shaft axes, Z = up.
Origin: X=0 at the sprocket-shaft axis, Y=0 at outer face of left frame
plate, Z=0 at the bottom of the base.
"""

from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class Clearances:
    run: float = 0.25      # running fit, per side (shaft in bushing, slider)
    loc: float = 0.15      # locational fit (tabs, keys)
    press: float = 0.05    # press fit
    # collision sweep: any pair closer than `warn` (and not touching) is a
    # warning; any intersection is an error.
    warn: float = 0.12


@dataclass(frozen=True)
class Tape:
    width: float = 25.0
    thickness: float = 0.1
    pitch: float = 8.0            # row pitch P along tape
    guide_hole_d: float = 3.2
    data_hole_d: float = 4.0
    guide_track_y: float = 5.0    # from tape reference edge
    data_track_y: float = 18.0    # from tape reference edge


@dataclass(frozen=True)
class Geneva:
    slots: int = 6
    center_distance: float = 48.0     # camshaft axis to sprocket axis (vertical)

    @property
    def crank_radius(self) -> float:          # driver pin radius
        return self.center_distance * math.sin(math.pi / self.slots)

    @property
    def wheel_radius(self) -> float:
        return self.center_distance * math.cos(math.pi / self.slots)

    @property
    def drive_half_angle(self) -> float:      # camshaft degrees of engagement /2
        return 90.0 - 180.0 / self.slots      # 60° for n=6  → drive lasts 120°


@dataclass(frozen=True)
class Sprocket:
    teeth: int = 6                 # one tooth per Geneva index step

    def pitch_radius(self, tape_pitch: float) -> float:
        return self.teeth * tape_pitch / (2 * math.pi)


@dataclass(frozen=True)
class Phases:
    """Camshaft angle windows (degrees). Geneva engagement is centered on
    theta=60 (drive from 0 to 120 for a 6-slot wheel)."""
    advance: tuple = (0.0, 120.0)
    pins_lower: tuple = (130.0, 170.0)
    logic: tuple = (170.0, 205.0)
    punch: tuple = (210.0, 280.0)     # down at 245, back up by 280
    pins_lift: tuple = (290.0, 355.0)


@dataclass(frozen=True)
class Params:
    cl: Clearances = field(default_factory=Clearances)
    tape: Tape = field(default_factory=Tape)
    geneva: Geneva = field(default_factory=Geneva)
    sprocket: Sprocket = field(default_factory=Sprocket)
    phases: Phases = field(default_factory=Phases)

    # --- printer ---
    bed_x: float = 220.0
    bed_y: float = 220.0
    bed_z: float = 250.0
    min_wall: float = 1.2

    # --- global layout (Z heights) ---
    base_z: float = 4.0               # base plate thickness
    tape_z: float = 60.0              # top surface of tape bed = tape underside
    # sprocket axis: pitch radius above tape so teeth mesh through the paper
    @property
    def sprocket_axis_z(self) -> float:
        return self.tape_z + self.tape.thickness + self.sprocket.pitch_radius(self.tape.pitch)

    @property
    def camshaft_z(self) -> float:
        return self.sprocket_axis_z + self.geneva.center_distance

    # --- layout (X) ---
    # The tape travels in -X (set by the Geneva/crank chirality), so the
    # punch station sits at +X (upstream): machine-punched guide holes reach
    # the output sprocket. Camshaft is directly above the stations.
    sprocket_x: float = 0.0           # sprocket shaft axis
    reader_center_x: float = 24.0     # center pin (C); L at +32, R at +16
    punch_x: float = 24.0             # punch station, 3·P upstream of sprocket B

    # --- layout (Y): two tape paths between frame plates ---
    plate_t: float = 6.0              # frame plate thickness
    tapeA_y0: float = 14.0            # reference edge of input tape path
    tapeB_y0: float = 49.0            # reference edge of output tape path
    inner_w: float = 88.0             # inner span between plates

    @property
    def right_plate_y0(self) -> float:
        return self.plate_t + self.inner_w   # inner face at plate_t + inner_w

    # --- pins / reader ---
    pin_tip_d: float = 3.5
    pin_shaft_d: float = 4.0
    pin_drop: float = 2.5             # how far a pin sinks into a hole
    pin_lift: float = 3.0             # clearance above paper when bailed up

    # --- punches ---
    punch_data_d: float = 4.0
    punch_guide_d: float = 3.2
    punch_stroke: float = 7.0         # from rest (4 above paper) to 3 into die

    # --- shafts ---
    shaft_d: float = 8.0


P = Params()
