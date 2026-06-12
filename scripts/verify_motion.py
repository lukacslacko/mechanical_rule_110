"""Full-cycle motion interference sweep.

For each of the 8 input cases and theta in 0..360 step DTH: pose every part,
report ANY mesh intersection (excluding whitelisted intended contacts up to
their allowed penetration). Also coarse minimum-clearance survey.

Run: .venv/bin/python scripts/verify_motion.py [--fast]
"""

import sys, time, itertools, argparse
from pathlib import Path
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mech110 import assembly
from mech110.assembly import WHITELIST, STATIC

ap = argparse.ArgumentParser()
ap.add_argument("--fast", action="store_true", help="2deg steps, 2 cases")
ap.add_argument("--cases", type=str, default=None, help="e.g. 000,111")
args = ap.parse_args()

DTH = 2.0 if args.fast else 1.0
PARTS_DIR = ROOT / "out" / "parts"

def load_clean(path):
    m = trimesh.load(str(path), force="mesh")
    m.merge_vertices(merge_tex=True, merge_norm=True)
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    return m

meshes = {assembly.strip_step_prefix(f.stem): load_clean(f)
          for f in PARTS_DIR.glob("*.stl")}
print(f"loaded {len(meshes)} meshes")

def stl_key(n):
    if n in ("pin_l", "pin_r"):
        return "pin_c"
    if n in ("collar_l", "collar_r"):
        return "collar_c"
    return n

names = [n for n in meshes
         if not n.startswith("tape_a") and n not in
         ("pin_l", "pin_r", "collar_l", "collar_r")] + [
         "pin_l", "pin_r", "collar_l", "collar_r", "tape_a"]
if args.cases:
    cases = [tuple(int(c) for c in s) for s in args.cases.split(",")]
elif args.fast:
    cases = [(0, 0, 0), (1, 1, 1)]
else:
    cases = list(itertools.product((0, 1), repeat=3))

mgr = trimesh.collision.CollisionManager()
for n in names:
    key = "tape_a_000" if n == "tape_a" else stl_key(n)
    mgr.add_object(n, meshes[key])

errors = {}
t0 = time.time()
for bits in cases:
    # swap in the right tape_a mesh
    mgr.remove_object("tape_a")
    mgr.add_object("tape_a", meshes[f"tape_a_{bits[0]}{bits[1]}{bits[2]}"])
    for th in np.arange(0.0, 360.0, DTH):
        P = assembly.poses(th, bits)
        for n in names:
            if n in P:
                mgr.set_transform(n, P[n])
            else:
                mgr.set_transform(n, np.eye(4))
        hit, contacts = mgr.in_collision_internal(return_names=False,
                                                  return_data=True)
        if hit:
            for c in contacts:
                pair = frozenset(c.names)
                allowed = WHITELIST.get(pair, 0.0)
                if c.depth > allowed + 1e-6:
                    key = tuple(sorted(pair)) if len(pair) == 2 else tuple(pair)
                    rec = errors.setdefault(key, [0, 0.0, None, None])
                    rec[0] += 1
                    if c.depth > rec[1]:
                        rec[1], rec[2], rec[3] = c.depth, th, bits
    print(f"case {bits} done ({time.time()-t0:.0f}s)")

print()


def stl_key(n):
    if n in ("pin_l", "pin_r"):
        return "pin_c"
    if n in ("collar_l", "collar_r"):
        return "collar_c"
    return n


def boolean_vol(a, b, th, bits):
    P = assembly.poses(th, bits)
    ka = "tape_a_" + "".join(map(str, bits)) if a == "tape_a" else stl_key(a)
    kb = "tape_a_" + "".join(map(str, bits)) if b == "tape_a" else stl_key(b)
    ma, mb = meshes[ka].copy(), meshes[kb].copy()
    ma.apply_transform(P.get(a, np.eye(4)))
    mb.apply_transform(P.get(b, np.eye(4)))
    try:
        inter = trimesh.boolean.intersection([ma, mb], engine="manifold")
        return 0.0 if inter.is_empty else inter.volume
    except Exception:
        return -1.0


real = []
for pair, (cnt, depth, th, bits) in sorted(errors.items(),
                                           key=lambda kv: -kv[1][1]):
    vol = boolean_vol(pair[0], pair[1], th, bits)
    wl = WHITELIST.get(frozenset(pair), 0.0)
    # face contacts -> vol ~ 0; whitelisted drive contacts allowed a small
    # contact-patch volume; everything else with real volume is an ERROR
    allowed_vol = 0.5 if wl > 0 else 5e-3
    status = "REAL" if (vol > allowed_vol or vol < 0) else "ok"
    if status == "REAL":
        real.append(pair)
    print(f"  [{status:4s}] {pair[0]:16s} x {pair[1]:16s} fcl={depth:6.3f} "
          f"vol={vol:8.3f} worst@{th:6.1f} bits={bits}")
if real:
    print(f"=== {len(real)} REAL interferences ===")
    sys.exit(1)
print("=== NO real interference across all sampled states ===")

# coarse clearance survey
print("\nclearance survey (5 deg, case 010):")
mgr.remove_object("tape_a")
mgr.add_object("tape_a", meshes["tape_a_010"])
worst = (1e9, None, None)
for th in np.arange(0.0, 360.0, 5.0):
    P = assembly.poses(th, (0, 1, 0))
    for n in names:
        mgr.set_transform(n, P.get(n, np.eye(4)))
    d, pair = mgr.min_distance_internal(return_names=True)
    if d < worst[0]:
        worst = (d, pair, th)
print(f"  min clearance {worst[0]:.3f}mm between {worst[1]} at theta={worst[2]}")
