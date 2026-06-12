import itertools
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mech110.params import P
from mech110 import camprofiles as cam
from mech110 import kinematics as kin
from mech110.rule110 import rule110

ALL_BITS = list(itertools.product((0, 1), repeat=3))
THETAS = np.arange(0.0, 360.0, 0.25)


def test_machine_output_equals_rule110():
    for bits in ALL_BITS:
        assert kin.machine_punches(bits) == bool(rule110(*bits)), bits


def test_punch_depth_when_enabled():
    th = sum(P.phases.punch) / 2
    tip = cam.data_punch_tip_z(th, True)
    assert abs(tip - cam.PUNCH_TIP_DOWN) < 1e-9
    assert cam.data_punch_tip_z(th, False) == cam.PUNCH_TIP_UP


def test_tape_advances_exactly_one_pitch():
    assert abs(cam.tape_x(0.0)) < 1e-9
    assert abs(cam.tape_x(200.0) - P.tape.pitch) < 1e-9
    xs = [cam.tape_x(t) for t in THETAS]
    assert all(b - a > -1e-9 for a, b in zip(xs, xs[1:]))  # monotonic


def test_tape_moves_only_while_pins_and_punches_clear():
    for th in THETAS:
        moving = 1e-6 < cam.tape_x(th + 0.25) - cam.tape_x(th)
        if moving:
            # pins fully up
            for bit in (0, 1):
                assert kin.pin_tip_z(th, bit) >= cam.PAPER_TOP + P.pin_lift - 1e-6, th
            # punches fully up
            assert cam.guide_punch_tip_z(th) >= cam.PUNCH_TIP_UP - 1e-9, th
            assert cam.data_punch_tip_z(th, True) >= cam.PUNCH_TIP_UP - 1e-9, th


def test_punch_strokes_only_during_dwell():
    for th in THETAS:
        if cam.guide_punch_tip_z(th) < cam.PUNCH_TIP_UP - 0.05:
            assert cam.tape_x(th) > P.tape.pitch - 1e-6, th  # advance finished


def test_veto_engages_before_pendulum_could_pass():
    """Case 111: at every theta the block travel must stay at/below the
    blocked travel; and when the pendulum first starts moving, the blocker
    must already be engaged."""
    bits = (1, 1, 1)
    for th in THETAS:
        ang = kin.pendulum_angle(th, bits)
        travel = kin.BLOCK_ARM * math.sin(math.radians(ang))
        assert travel <= kin.BLOCKED_TRAVEL + 1e-9, th
        if ang > 1e-9:
            assert kin.blocker_engaged(th, bits), th


def test_pendulum_fully_in_before_punch_for_punching_cases():
    for bits in ALL_BITS:
        if rule110(*bits):
            for th in np.arange(P.phases.punch[0], P.phases.punch[1], 0.5):
                assert kin.punch_enabled(th, bits), (bits, th)


def test_pendulum_out_for_nonpunching_cases():
    for bits in ALL_BITS:
        if not rule110(*bits):
            for th in THETAS:
                assert not kin.punch_enabled(th, bits), (bits, th)


def test_continuity_of_all_dofs():
    """No DOF may jump: catches cam-profile and phase-window bugs."""
    for bits in ALL_BITS:
        prev = None
        for th in THETAS:
            state = {
                "bail": cam.bail_tip_z(th),
                "pin_l": kin.pin_tip_z(th, bits[0]),
                "pin_c": kin.pin_tip_z(th, bits[1]),
                "pin_r": kin.pin_tip_z(th, bits[2]),
                "and3": kin.and3_descent(th, bits),
                "rock_c": kin.rocker_angle(th, bits, "C"),
                "rock_r": kin.rocker_angle(th, bits, "R"),
                "pend": kin.pendulum_angle(th, bits),
                "gp": cam.guide_punch_tip_z(th),
                "hammer": cam.hammer_face_z(th),
                "dp": cam.data_punch_tip_z(th, rule110(*bits) == 1),
                "tape": cam.tape_x(th),
                "psi": cam.geneva_psi(th),
            }
            if prev is not None:
                for k in state:
                    lim = 1.5 if k in ("rock_c", "rock_r", "pend", "psi") else 0.6
                    assert abs(state[k] - prev[k]) < lim, (bits, th, k)
            prev = state


def test_cycle_closes():
    """State at theta=360-eps must match theta=0 (cyclic machine)."""
    for bits in ALL_BITS:
        a, b = 359.999, 0.0
        assert abs(cam.bail_tip_z(a) - cam.bail_tip_z(b)) < 1e-3
        assert abs(kin.pendulum_angle(a, bits) - kin.pendulum_angle(b, bits)) < 1e-3
        assert abs(cam.geneva_psi(a) % 60 - cam.geneva_psi(b) % 60) < 1e-3


def test_geneva_geometry():
    g = P.geneva
    assert abs(g.crank_radius - 24.0) < 1e-9
    assert abs(g.wheel_radius - 48 * math.cos(math.pi / 6)) < 1e-9
    # smooth start/stop (no-jerk property of Geneva at entry/exit)
    assert abs(cam.geneva_psi(0.01) - 0.0) < 0.01
    assert abs(cam.geneva_psi(119.99) - 60.0) < 0.01
