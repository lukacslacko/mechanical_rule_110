import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mech110.rule110 import rule110, mech_output, step, step_mech


def test_mechanism_equals_rule110_all_neighborhoods():
    for l, c, r in itertools.product((0, 1), repeat=3):
        assert mech_output(l, c, r) == rule110(l, c, r), (l, c, r)


def test_known_table():
    table = {(1,1,1): 0, (1,1,0): 1, (1,0,1): 1, (1,0,0): 0,
             (0,1,1): 1, (0,1,0): 1, (0,0,1): 1, (0,0,0): 0}
    for k, v in table.items():
        assert rule110(*k) == v


def test_multigeneration_tape():
    tape = [0] * 20 + [1] + [0] * 3   # single seed near right edge, grows left
    a, b = tape[:], tape[:]
    for _ in range(15):
        # tape grows by one cell of blank leader each generation, like real paper
        a = step([0] + a)
        b = step_mech([0] + b)
        assert a == b


def test_blank_stays_blank():
    assert step([0] * 10) == [0] * 10
