"""Force-closure regression: parts that must return by weight balance.

The geometric sweeps cannot catch a rocker whose pad fails to follow its
pin because the tip side outweighs it; these tests pin the sign AND a
margin (in mm^4 = volume*lever; PLA ~1.24e-3 g/mm^3, so 1600 mm^4 ~ 2 g.mm,
several times the estimated axle-bore friction).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def _ycm(part):
    from build123d import CenterOf
    return part.center(CenterOf.MASS).Y, part.volume


def test_or_rockers_are_pad_heavy():
    from mech110 import parts_reader
    for builder in (parts_reader._rocker_c, parts_reader._rocker_r):
        p = builder()
        y, v = _ycm(p)
        moment = v * (y - 50.0)          # axle at y=50; pad side is y<50
        assert moment < -1500.0, (builder.__name__, moment)


def test_return_rocker_is_counterweight_heavy():
    from mech110 import parts_punch
    p = parts_punch._return_rocker()
    y, v = _ycm(p)
    moment = v * (y - 84.0)              # pivot y=84; tip pad is y<84
    assert moment > +250.0, moment
