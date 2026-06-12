"""Build every part and export STLs to out/parts/ (plus the 8 tape_a hole
patterns as tape_a_<lcr>.stl). Run: .venv/bin/python scripts/build_stls.py"""

import sys, time, itertools
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from build123d import export_stl
from mech110 import assembly

OUT = Path(__file__).resolve().parents[1] / "out" / "parts"
OUT.mkdir(parents=True, exist_ok=True)

t0 = time.time()
parts = assembly.build_all(bits=(0, 0, 0))
for name, solid in sorted(parts.items()):
    path = OUT / f"{name}.stl"
    export_stl(solid, str(path), tolerance=0.03, angular_tolerance=0.25)
    ns = len(solid.solids())
    tag = "" if ns == 1 else f"  [WARN {ns} solids]"
    print(f"{name:16s} vol={solid.volume:10.1f}{tag}")

# tape_a per input case (the three read rows differ)
from mech110 import parts_frame
for bits in itertools.product((0, 1), repeat=3):
    t = parts_frame.build(bits=bits)["tape_a"]
    export_stl(t, str(OUT / f"tape_a_{bits[0]}{bits[1]}{bits[2]}.stl"),
               tolerance=0.03, angular_tolerance=0.25)
    print(f"tape_a_{bits[0]}{bits[1]}{bits[2]} ok")

print(f"done in {time.time()-t0:.0f}s -> {OUT}")
