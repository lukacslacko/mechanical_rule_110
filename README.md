# MECH-110 — a 3D-printable mechanical Rule 110 machine

A hand-cranked, fully 3D-printable machine that computes the
[Rule 110](https://en.wikipedia.org/wiki/Rule_110) cellular automaton —
which is Turing complete — on punched paper tape. One crank revolution
reads three neighboring cells from the input tape, decides the next-generation
cell mechanically, and punches it (plus a fresh sprocket hole) into the
output tape. No springs, no electronics, no metal parts.

The entire mechanism was designed *and verified* in software before printing:
every part is generated parametrically, the full cycle is collision-swept in
all 8 input states, and the assembly order is checked step by step — so
nothing has to be assembled "through the fourth dimension".

## The trick: no 8-way decoder needed

Rule 110 factors into

```
new = (C OR R) AND NOT (L AND C AND R)
```

so instead of an 8-position code wheel, the logic train is just:

- three sensing **pins** that drop into tape holes,
- an **AND3 guillotine** plate resting on all three pin collars (descends
  fully only for 1-1-1) carrying a blocker finger,
- two **OR rockers** (one per pin C, R) whose tips drive
- a horizontal **swing-arm interposer** that slides a block between the
  cam-driven hammer and the data punch.

The 111-veto wins its race by construction: pins ride a cam-timed bail down,
so all drops are rate-synchronized; the blocker enters the swing path before
the arm can start moving.

## Highlights

- **Geneva drive**: 6-slot Geneva + locking ring indexes both tapes exactly
  one 8 mm pitch per revolution; the locking crescent is computed numerically
  from the swept wheel silhouette.
- **Rib cams + slide-in keys**: positive-return cams that still allow the
  one-piece camshaft to drop radially into open bearings — the window
  followers are closed by snap-in keys after the shaft is seated.
- **Single source of truth**: cam lift functions, the kinematic model and the
  cam solids all derive from the same code (`camprofiles.py`), so geometry
  and simulation cannot disagree.

## Verification pipeline

| stage | command | what it proves |
|---|---|---|
| logic & kinematics | `pytest tests/` | the mechanism model computes Rule 110 in all 8 cases; phase discipline; veto race; cycle closure |
| motion sweep | `python scripts/verify_motion.py` | 8 cases × 360° × 1°: no part-pair interference (FCL + exact mesh booleans) |
| assembly order | `python scripts/verify_assembly.py` | a 51-step insertion sequence with swept approach paths |
| viewer | `python scripts/export_viewer.py` | animated three.js model → `out/viewer.html` |

## Quick start

```sh
uv venv .venv && uv pip install --python .venv/bin/python \
    build123d trimesh python-fcl manifold3d shapely scipy rtree networkx pytest
.venv/bin/python -m pytest tests/ -q
.venv/bin/python scripts/build_stls.py        # writes out/parts/*.stl
.venv/bin/python scripts/verify_motion.py
.venv/bin/python scripts/export_viewer.py     # open out/viewer.html
```

Open `out/viewer.html` in a browser: drag the crank-angle slider, pick the
(L,C,R) input case, press play.

## Layout of the repo

- `DESIGN.md` — full mechanism architecture and the design decisions.
- `STATUS.md` — current verification status and open items.
- `src/mech110/` — parameters, cam profiles, kinematics, world layout, the
  parametric part builders (build123d), Geneva geometry, assembly registry.
- `scripts/` — STL export, motion sweep, assembly check, viewer export.
- `out/parts/` — generated STLs; `out/viewer.html` — the animated model.

## Status

Motion-certified (zero interference across all sampled states); assembly
sequence verified with a handful of residual snap-fit contacts still being
classified — see `STATUS.md`. Not yet printed: tolerance/printability review
is the next step. This is a "Turing complete in principle" demonstration
piece, not a practical computer — exactly as intended.

## License

MIT
