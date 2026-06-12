"""Assembly-order feasibility check.

Each step inserts a rigid GROUP of parts: every member has a stage pose;
the group moves along a shared piecewise-linear offset path (offsets applied
on the left, ending at identity). Sampled waypoints are collision-checked
against everything already placed. Repose steps move already-placed parts
the same way. Snap-fits get a small allowed penetration.

Run: .venv/bin/python scripts/verify_assembly.py
"""

import sys
from pathlib import Path
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mech110 import assembly, layout as L
from mech110.kinematics import _T
from mech110.assembly import _rot_z

PARTS_DIR = ROOT / "out" / "parts"
meshes = {assembly.strip_step_prefix(f.stem): trimesh.load(str(f), force="mesh")
          for f in PARTS_DIR.glob("*.stl")}

P0 = assembly.poses(0.0, (0, 0, 0))
I = np.eye(4)


def stl_key(name):
    if name in ("pin_l", "pin_r"):
        return "pin_c"
    if name in ("collar_l", "collar_r"):
        return "collar_c"
    if name.startswith("cp_"):
        return "cp_cl1"
    if name == "tape_a":
        return "tape_a_000"
    return name


def pose0(name):
    if name.startswith("cp_"):
        rel = dict(assembly.INSTANCES["cap_pin3"])
        base = dict(assembly.INSTANCES["cap_pin3"])["cp_cl1"]
        return rel[name] @ np.linalg.inv(base)
    return P0.get(name, I)


def step(names, path_offsets, allow=0.0, n=12, check=True, repose=False,
         poses=None):
    if isinstance(names, str):
        names = [names]
    return dict(names=names, path=path_offsets, allow=allow, n=n,
                check=check, repose=repose, poses=poses or {})


def lin(names, vec, dist, **kw):
    v = np.array(vec, float)
    v /= np.linalg.norm(v)
    return step(names, [_T(*(v * dist)), I], **kw)


UP = (0, 0, 1)
TILT_DEG = -34.0          # block rises through the free column at y~90
TILT = _rot_z(TILT_DEG, L.PIVOT_X, L.PIVOT_Y)


def kin_rot(theta):
    from mech110.kinematics import _rot_y
    return _rot_y(-theta, L.CAMSHAFT_X, L.CAMSHAFT_Z)

STEPS = [
    lin("base", UP, 30),
    lin("plate_left", UP, 60), lin("plate_right", UP, 60),
    lin("bed", UP, 60),
    step(["tape_a", "tape_b"], [I], check=False),     # threading: subset arg
    lin("cover_a", UP, 40), lin("cover_b", UP, 40),
    lin("comb_bridge", UP, 50),
    lin("pin_l", UP, 45), lin("pin_c", UP, 45), lin("pin_r", UP, 45),
    lin("bail", UP, 45),
    lin("collar_l", (0, 1, 0), 18, allow=0.8),        # snap over groove
    lin("collar_c", (0, 1, 0), 18, allow=0.8),
    lin("collar_r", (0, 1, 0), 18, allow=0.8),
    lin("punch_block", UP, 50),
    lin("lcol_left", UP, 40), lin("lcol_right", UP, 40),
    lin("rocker_c", UP, 25), lin("rocker_r", UP, 25),
    lin("axle", (1, 0, 0), 24),               # rod threads westward
    lin("and3", UP, 45),
    lin("gallows", UP, 50),
    lin(["swing_arm", "block"], UP, 8,
        poses={n: TILT @ pose0(n) for n in ("swing_arm", "block")}),
    step(["swing_arm", "block"],
         [_rot_z(a, L.PIVOT_X, L.PIVOT_Y) @ np.linalg.inv(TILT)
          for a in np.linspace(TILT_DEG, 0.0, 18)],
         repose=True, n=2,
         poses={n: pose0(n) for n in ("swing_arm", "block")}),
    lin("pivot_clip", UP, 25, allow=0.35),    # rises inside the skirt bore
    lin("return_rocker", UP, 25),
    lin("rr_axle", (-1, 0, 0), 25, allow=0.2),
    lin(["hammer", "data_punch"], UP, 55),    # punch held at rest height
    lin("gp_slider", UP, 45),
    lin("camshaft", UP, 80),
    lin("key_bail", (0, 1, 0), 12),
    lin("key_gp", (0, -1, 0), 10, allow=0.45),
    lin("key_ham", (0, -1, 0), 22, allow=0.45),
    # rotate camshaft to theta=180 so the geneva pin is clear of the
    # sprocket stub's drop path, insert shaft+wheel, rotate back
    step("camshaft", [np.eye(4)], repose=True, check=False,
         poses={"camshaft": kin_rot(180.0)}),
    lin("sprocket_shaft", UP, 60),
    lin("cap_spk_left", UP, 30), lin("cap_spk_right", UP, 30),
    lin("cp_sl1", (0, -1, 0), 14), lin("cp_sl2", (0, -1, 0), 14),
    lin("cp_sr1", (0, 1, 0), 14), lin("cp_sr2", (0, 1, 0), 14),
    step("geneva_wheel", [_T(y=-30.0), I], n=24),
    lin("spk_clip", (0, 1, 0), 12, allow=0.8),
    step("camshaft", [np.eye(4)], repose=True, check=False,
         poses={"camshaft": pose0("camshaft")}),
    lin("cap_cam_left", UP, 30), lin("cap_cam_right", UP, 30),
    lin("cp_cl1", (0, -1, 0), 8), lin("cp_cl2", (0, -1, 0), 8),
    lin("cp_cr1", (0, 1, 0), 14), lin("cp_cr2", (0, 1, 0), 14),
    step("crank", [_T(y=28.0), I]),
]

def main():
    placed = {}
    mgr = trimesh.collision.CollisionManager()
    failures = []

    for i, st in enumerate(STEPS):
        names = st["names"]
        stage = {n: st["poses"].get(n, pose0(n)) for n in names}
        if st["repose"]:
            for n in names:
                mgr.remove_object(n)
                del placed[n]
        ok = True
        if st["check"]:
            # sample along the offset path
            samples = []
            path = st["path"]
            for w0, w1 in zip(path, path[1:]):
                for t in np.linspace(0.0, 1.0, st["n"], endpoint=False):
                    samples.append(w0 + (w1 - w0) * t)
            samples.append(path[-1])
            for off in samples:
                for n in names:
                    m = meshes[stl_key(n)].copy()
                    m.apply_transform(off @ stage[n])
                    hit, data = mgr.in_collision_single(
                        m, return_names=False, return_data=True)
                    if hit:
                        depth = max(c.depth for c in data)
                        if depth > st["allow"] + 1e-6:
                            others = sorted({x for c in data for x in c.names
                                             if x != "__external"})
                            failures.append((i, n, depth, others[:4]))
                            ok = False
                if not ok:
                    break
        final_off = st["path"][-1]
        for n in names:
            placed[n] = final_off @ stage[n]
            mgr.add_object(n, meshes[stl_key(n)], transform=placed[n])
        print(f"{i:2d} {'+'.join(names):42s} {'OK' if ok else 'FAIL'}")

    print()
    if failures:
        print(f"=== {len(failures)} candidate failures (fcl) ===")
        seen = set()
        for i, name, depth, others in failures:
            key = (name, tuple(others))
            if key in seen:
                continue
            seen.add(key)
            print(f"  step{i:2d} {name:16s} depth={depth:6.3f} vs {others}")
        sys.exit(1)
    print("=== assembly sequence feasible ===")


if __name__ == "__main__":
    main()
