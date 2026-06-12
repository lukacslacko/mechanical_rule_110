# MECH-110 status (end of session 1)

## VERIFIED
- Logic+kinematics: 19 tests green (mechanism == Rule 110 for all 8 cases,
  phase discipline, 111-veto race, cycle closure).
- MOTION: full sweep CLEAN - 8 cases x 360deg x 1deg, all 44 parts, fcl +
  manifold-boolean second pass: zero real interference
  (scripts/verify_motion.py).
- ASSEMBLY (scripts/verify_assembly.py): 51-step sequence runs; structural
  blockers all resolved (axle threads along X after rockers; camshaft turned
  to theta=180 for sprocket-shaft drop; cap pins from outer faces before
  wheel/crank; swing-arm enters tilted -34deg, 8mm rise). 8 residual
  candidate contacts, believed noise/snap-fits but NOT yet boolean-classified:
  key_gp 0.57 (snap barb squeeze ~0.3 by design), key_ham 1.29 (recheck after
  the hub-boss removal!), sprocket_shaft 2.11 & geneva_wheel 1.74 vs camshaft
  (slide-on profile - diagnose with boolean bbox), spk_clip 1.51 (deep snap),
  cp_cl 0.2 (ok), crank 0.045 (ok).
- out/viewer.html: animated three.js model (crank slider, 8 input cases).
- out/parts/*.stl: all printable parts, single solids, watertight after
  vertex merge.

## NEXT
1. Boolean-classify the 8 assembly residuals (reuse verify_motion's bvol).
2. Re-run verify_assembly after the spoked-hub boss removal.
3. Printability pass (bed fit OK by construction; check min walls: gp key
   hole web 0.4, pillar x21..21.5 in and3, rocker foot ribs).
4. Tolerance review for FDM (0.25 running fits assumed).
5. Hand-punch jig for tape leaders (not yet modeled).
6. DESIGN.md final sync (tower guides, east-tip rockers, shelf rail,
   3-spoke hubs, T-cap reversion to pinned plug caps).

Run: .venv/bin/python scripts/{build_stls,verify_motion,verify_assembly,
export_viewer}.py ; tests: -m pytest tests/ -q
