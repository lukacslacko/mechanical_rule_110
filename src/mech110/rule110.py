"""Rule 110 reference implementation + discrete model of the mechanism's
logic train (pins -> bars -> rocker -> interposer -> punch).

The mechanism model is deliberately written in terms of *mechanical* facts
(who rests on whom, who pushes whom, who blocks whom), so that if the
geometry changes, this stays the executable spec the CAD must satisfy.
"""

RULE = 110


def rule110(l: int, c: int, r: int) -> int:
    """Reference: bit (l,c,r) of the Wolfram code 110."""
    return (RULE >> ((l << 2) | (c << 1) | r)) & 1


def mech_output(l: int, c: int, r: int) -> int:
    """What the mechanism punches, derived from mechanical behavior.

    Pin drop: pin i drops fully (depth=1) iff there is a hole (bit=1).
    AND3 bar rests on tabs of all three pins -> its descent = min of drops.
    OR rocker ledges sit under tabs of C and R; either dropping pin pushes
    it -> its travel = max of those drops.
    Interposer swings in iff the rocker pushed it AND the AND3 blocker
    finger is not fully down. Punch fires iff interposer is in.
    """
    drop = {"L": l, "C": c, "R": r}          # 1 = dropped into hole
    and3_descent = min(drop.values())        # bar rests on all three tabs
    rocker_travel = max(drop["C"], drop["R"])  # one-way pushes
    blocker_engaged = and3_descent == 1
    interposer_in = (rocker_travel == 1) and not blocker_engaged
    return 1 if interposer_in else 0


def step(cells: list[int]) -> list[int]:
    """One generation on a finite tape with 0-padding (blank paper) ends."""
    padded = [0] + cells + [0]
    return [rule110(padded[i - 1], padded[i], padded[i + 1])
            for i in range(1, len(padded) - 1)]


def step_mech(cells: list[int]) -> list[int]:
    padded = [0] + cells + [0]
    return [mech_output(padded[i - 1], padded[i], padded[i + 1])
            for i in range(1, len(padded) - 1)]
