# MECH-110 — a 3D-printable Rule 110 machine on punched paper tape

One crank revolution = one machine cycle = one cell of the next generation
computed and punched. Hole = 1, no hole = 0.

## 0. The logic simplification (no 8-way code wheel needed)

Rule 110: new(i) = f(L, C, R) with table
111→0, 110→1, 101→1, 100→0, 011→1, 010→1, 001→1, 000→0.

f is 0 exactly for {111, 100, 000}. Since 100 and 000 are exactly the patterns
with C=0 and R=0:

    new = (C OR R) AND NOT (L AND C AND R)

Mechanically:
- **AND3** = a guided "guillotine" plate whose three feet rest on the collars
  of all three sensing pins. It descends fully only if *all three* pins drop
  into holes (min function), and carries a blocker finger.
- **OR** = two independent gravity-follower rockers (one for pin C, one for
  pin R), see-saws on a common axle: the ledge side rests on its pin's collar
  (one-way), so the tip side rises when *either* pin drops (max function at
  the contact). One-way resting contacts mean a stalled rocker can never
  prevent a pin from completing its drop — the logic is fully decoupled.
- The rocker tips ride in a 45° V-groove on the foot of a **swing-arm
  interposer** rotating about a *vertical* pivot: rising tips swing the
  interposer block horizontally (at constant height!) into the gap between
  the punch hammer and the data punch head; descending tips pull it back out
  (positive both ways, no springs, no gravity bias needed).
- The AND3 finger, when fully down, stands in the swing path and stops the
  arm after ~1 mm of block travel (= veto). Veto wins the race by
  construction: pins don't free-fall, they ride the cam-controlled bail down,
  so all drops are rate-synchronized; the finger enters the swing path at
  0.8 mm of descent while the V-groove lag delays arm motion until ~1.1 mm.

So: 3 pins + 1 plate + 2 rockers + 1 swing arm replace the 8-position wheel.

## 1. Tape format

- Paper strip, width 25 mm (cut from 57 mm adding-machine roll, or any paper).
- Row pitch P = 8 mm along the tape.
- Track 1 (guide/sprocket): ⌀3.2 mm holes, every row, near reference edge.
- Track 2 (data): ⌀4.0 mm holes, hole = 1.
- Two parallel tape paths through the machine, side by side:
  path A = input (reader), path B = output (punches).
  Loop mode: the tape leaving B U-turns outside the machine and re-enters A.

## 2. Coordinate system & layout

X = tape travel direction, Y = across tape (all shaft axes along Y), Z = up.

- Tape bed (with die holes, sprocket grooves, chad chute) at mid height.
- **Sprocket shaft** above the tape, teeth pointing down through the guide
  holes into bed grooves: two 6-tooth sprockets (pitch radius = 6P/2π ≈ 7.64 mm),
  one per tape path, on one shaft → both tapes advance in lockstep.
- **Main camshaft** higher up, parallel. Carries all cams + Geneva driver +
  crank. Vertical center distance to sprocket shaft = Geneva center distance d.
- **Geneva (6-slot)**: driver crank (radius 0.5·d) on the camshaft's left end,
  wheel (radius 0.866·d) keyed to the sprocket shaft's left end, both outside
  the left frame plate. 60° of wheel per rev = exactly one tooth = one pitch P.
  Locking ring on the driver keeps tapes registered during dwell.
- **Crank handle** on the camshaft's right end.

## 3. Stations

**Reader (over path A)**: three pins at X-spacing P over the data track,
rounded ⌀3.5 tips, sliding in a comb block. A lift bail under the pin collars
(one-way: lift only, lost motion downward) is driven by a groove cam — pins are
held 3 mm above the paper during tape advance, released to settle by gravity
during the read dwell. A pin over a hole drops ~2.5 mm into a bed groove;
a pin over paper rests on it (a few grams — cannot pierce).

**Logic (beside/above reader)**: AND3 bar, OR rocker, pendulum interposer,
as in §0. All gravity-returned; no metal springs anywhere in the machine.

**Punch station (over path B)**: punches guided in a block with integral
stripper; die holes in the bed; chads fall through to the base.
- Guide punch (⌀3.2-hole) — slider with integral punch tip, positively
  driven *both ways* by its cam (window follower). Punches every cycle.
- Data punch (⌀4-hole) — cam-driven hammer; the hammer's face pad presses
  the interposer block which presses the punch-head flange (drive), and a
  C-lip under the flange yanks the punch back out of the paper on retract
  (positive extraction — pierced paper grips the pin too hard for gravity).
  When the interposer is out, the face pad's edge passes clear of both the
  blocked block and the flange: the hammer strokes empty. A small
  counterweighted return rocker holds the idle punch at rest height.
Punch tips are pyramidal piercing points (printable); the design leaves a
drop-in option for steel pins (ground 4 mm nail) if PLA edges dull.

**Cam-follower construction**: every cam is an annular RIB (radial band of
varying center radius, walls = pitch curve ±1.5 mm) protruding axially from
its hub disc; each follower is a slider with a window straddling the rib —
lower boss outside the rib, upper boss inside the ring. The upper boss is a
separate slide-in KEY (inserted along Y after the camshaft is seated), which
is what makes radial drop-in assembly of the camshaft possible at all.
Punch station X is an exact integer multiple of P upstream of sprocket B's
top-dead-center, so machine-punched guide holes land on the teeth.
To start a raw tape, ~3 guide holes are hand-punched with a printed jig.

## 4. Cycle (one camshaft revolution, θ = 0…360°)

| θ | action |
|---|--------|
| 0–120° | Geneva engaged → both tapes advance exactly P. Pins held up, punches up. |
| 130–170° | bail lowers; pins settle into holes / onto paper |
| 170–205° | logic settles: AND3 bar lands, OR rocker pushes, interposer swings in (or is vetoed / not pushed) |
| 210–280° | punch stroke down + return (guide always, data iff interposer in) |
| 290–355° | bail lifts pins; bars/rocker/interposer reset by gravity |

Tapes only move while pins and punches are clear; punching only happens during
Geneva dwell (locking ring engaged).

## 5. Frame & assembly strategy

Two side plates + bed + top bearing bridges, joined by printed tab-and-wedge
joints (no screws required; M3 holes provided as backup). All shaft bearings
are **split pillow blocks**: shafts (printed in one piece with their cams /
sprockets) drop in radially, then a printed cap closes the bearing — nothing
has to be threaded axially through closed holes. Geneva wheel and crank key
onto hex shaft ends after the shafts are seated.

## 6. Verification plan (all in software, before printing)

1. **Logic model** (`rule110.py`): discrete simulation of pins → bars →
   interposer → punch for all 8 neighborhoods, asserted equal to Rule 110;
   plus multi-generation tape simulation against a reference implementation.
2. **Single source of truth for motion** (`camprofiles.py`): each cam's lift
   function is defined once, analytically; the cam *solid* is lofted from it
   and the follower *pose* is computed from it ⇒ cam/follower agreement by
   construction. All intended contacts are modeled with explicit running
   clearance (≥0.15 mm), so the nominal mechanism has **zero** intersections.
3. **Motion collision sweep** (`verify/collide.py`): θ sampled at ≤1° over a
   full cycle × all 8 input cases (and tape-with-holes solids in the loop);
   pairwise mesh collision via FCL. *Any* intersection = design error.
   Near-miss report (<0.15 mm clearance) as warnings.
4. **Assembly check** (`verify/assemble.py`): every part has an ordered
   insertion path (sequence of linear moves); each step is sweep-checked
   against the union of already-placed parts. Catches "needs 4th dimension".
5. **Printability** (`verify/printability.py`): per part — bed fit
   (220×220×250 default), min wall ≥ 1.2 mm, chosen print orientation noted,
   overhang sanity.
6. **Animated viewer** (`export/viewer.py`): self-contained three.js HTML,
   θ slider + play, selectable input case, exploded-view assembly animation.

## 7. Print parameters (defaults, all in `params.py`)

PLA, 0.4 nozzle. Running fits 0.25 mm/side, locational 0.15, press 0.05.
Shafts ⌀8 printed. Bed 220×220×250.

## 8. Registration & bookkeeping

Output row is punched k·P offset from the input row being read (k = fixed
station offset). In two-tape mode this is just indexing; in loop mode it adds
a constant spatial shift per generation, which composes harmlessly with the CA.
