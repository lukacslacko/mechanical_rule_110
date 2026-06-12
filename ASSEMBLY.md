# MECH-110 assembly guide

Step-by-step assembly in the **verified order** — this exact sequence (including insertion directions and the camshaft rotation in step 25) is collision-checked by `scripts/verify_assembly.py`; the images are rendered from the same data by `scripts/render_assembly_guide.py`.

New parts in each step are shown **orange**, slightly offset along their insertion direction (arrow). The near frame plate is drawn translucent so you can see inside.

No screws, springs or metal parts. Snap fits are printed; keep cyanoacrylate handy for the two press-fit pins noted below.

## Bill of parts

| part | qty | note |
|---|---|---|
| base, bed, plate_left/right | 1 ea | print flat |
| comb_bridge, punch_block, gallows, lcol_left/right | 1 ea | bed-peg mounted bridges |
| cover_a, cover_b | 1 ea | tape hold-downs |
| pin / collar | 3 / 3 | identical; collars are snap C-clips |
| bail + key_bail | 1 + 1 | key is a d5 press pin |
| and3, rocker_c, rocker_r, axle | 1 ea | the logic train |
| swing_arm, block, pivot_clip | 1 ea | interposer |
| hammer + key_ham, gp_slider + key_gp | 1+1 ea | cam followers |
| data_punch, return_rocker, rr_axle | 1 ea | data punch set |
| camshaft, crank | 1 ea | print camshaft vertically |
| sprocket_shaft, geneva_wheel, spk_clip | 1 ea | print shaft vertically |
| cap_cam_* , cap_spk_* | 2 + 2 | bearing caps |
| cap_pin3 | 8 | d3 retention pins |

---

## Step 1 — Base plate

![Step 1: Base plate](docs/assembly/step_01_base.png)

Lay the base flat. The rectangular mortises take the frame-plate tabs; everything else stacks on top.

## Step 2 — Frame plates

![Step 2: Frame plates](docs/assembly/step_02_plates.png)

Drop both plates straight down so their three bottom tabs enter the base mortises. The open-top U-slots are the shaft bearings; the small side holes near the slots take the bearing-cap retention pins later.

## Step 3 — Tape bed

![Step 3: Tape bed](docs/assembly/step_03_bed.png)

Lower the bed between the plates until its four legs sit on the base. Top side up: tape rails, sprocket grooves, the three reader-pin holes and the two punch die holes. All later peg-mounted parts plug into the bed's peg holes.

## Step 4 — Paper tapes (reference)

![Step 4: Paper tapes (reference)](docs/assembly/step_04_tapes.png)

Shown for reference only: the tapes are threaded LAST, by cranking — the sprocket teeth carry them in (hand-punch ~3 guide holes into each leader first). Input tape A runs in the near lane, output tape B in the far lane; both travel toward -X (to the left here).

## Step 5 — Sprocket hold-down covers

![Step 5: Sprocket hold-down covers](docs/assembly/step_05_covers.png)

Press the two covers' pegs into the bed holes flanking the sprocket station. They keep the paper wrapped on the sprocket teeth.

## Step 6 — Reader comb bridge

![Step 6: Reader comb bridge](docs/assembly/step_06_comb.png)

Press the comb bridge down onto its four bed pegs. It spans tape A as a bridge: its underside is the stripper, the three bores guide the reader pins, and the two end towers carry the vertical grooves that guide the bail and the AND3 plate.

## Step 7 — Reader pins

![Step 7: Reader pins](docs/assembly/step_07_pins.png)

Drop the three identical pins tip-first into the comb bores. They fall through into the bed holes for now; lift them while clipping on the collars in the next step.

## Step 8 — Lift bail

![Step 8: Lift bail](docs/assembly/step_08_bail.png)

Slide the bail down: its two end tabs ride the front grooves of the comb towers. The three U-slotted fingers pass around the pin shafts — they will lift the pins by their collars.

## Step 9 — Pin collars

![Step 9: Pin collars](docs/assembly/step_09_collars.png)

Hold each pin raised and snap a C-collar sideways (from the front) into its groove. The collar rests on the bail finger below; the AND3 feet and rocker pads will ride on its top face.

## Step 10 — Punch block

![Step 10: Punch block](docs/assembly/step_10_punch_block.png)

Press the punch block onto its four bed pegs. It bridges tape B: stripper underside, guide bores for both punches and the hammer shanks, the return-rocker ears at the back, and the shelf rail that parks the interposer block.

## Step 11 — Rocker axle columns

![Step 11: Rocker axle columns](docs/assembly/step_11_lcols.png)

Press the two L-columns into their bed holes in the corridor between the tapes. Their open cradles will hold the rocker axle.

## Step 12 — OR rockers

![Step 12: OR rockers](docs/assembly/step_12_rockers.png)

Set the two rockers in place: each ledge pad rests on its pin collar (C and R), the long arms route around the cam stack to the east side of the machine. Their hubs line up with the axle cradles.

## Step 13 — Rocker axle

![Step 13: Rocker axle](docs/assembly/step_13_axle.png)

Thread the axle rod westward (from the right) through the right cradle, both rocker hubs and the left cradle until the head seats. A drop of glue at the plain end keeps it from walking.

## Step 14 — AND3 guillotine

![Step 14: AND3 guillotine](docs/assembly/step_14_and3.png)

Lower the AND3 plate: its end tabs ride the rear grooves of the comb towers, the three crescent feet land around the pin collars, and the long blocker arm snakes west and over to the punch station. It must slide freely — it is lifted by whichever pin sits highest.

## Step 15 — Swing-arm gallows

![Step 15: Swing-arm gallows](docs/assembly/step_15_gallows.png)

Press the gallows onto its two bed pegs behind tape B. The vertical pin hanging from its arm is the swing-arm pivot.

## Step 16 — Interposer arm (tilted entry)

![Step 16: Interposer arm (tilted entry)](docs/assembly/step_16_swing_in.png)

Hang the block's stem in the arm's end bore first (off the machine). Then, with the arm rotated about 34 degrees clockwise from its rest attitude, raise the assembly so the hub slides onto the gallows pin. The tilt routes the block up through the free column behind the return-rocker ears — straight-in does not fit.

## Step 17 — Interposer arm (rotate home)

![Step 17: Interposer arm (rotate home)](docs/assembly/step_17_swing_home.png)

Rotate the arm counterclockwise to its rest position: the block settles onto the shelf rail, ready to swing over the punch flange.

## Step 18 — Pivot clip

![Step 18: Pivot clip](docs/assembly/step_18_pivot_clip.png)

Push the small C-clip up inside the foot-ring bore and snap it into the groove near the pivot pin's tip — it keeps the arm from lifting off.

## Step 19 — Punch return rocker

![Step 19: Punch return rocker](docs/assembly/step_19_return_rocker.png)

Set the return rocker between the ears on the punch block's rear extension, counterweight toward the back, tip pad forward under where the punch flange will sit.

## Step 20 — Return-rocker axle

![Step 20: Return-rocker axle](docs/assembly/step_20_rr_axle.png)

Push the small axle pin eastward through the ears and the rocker hub until the head seats against the west ear.

## Step 21 — Hammer + data punch

![Step 21: Hammer + data punch](docs/assembly/step_21_hammer.png)

Off the machine, slide the data punch sideways into the hammer's C-lip through its front opening (flange above the lip). Lower both together: the hammer's two shanks and the punch enter their bores; the punch flange comes to rest on the return-rocker pad.

## Step 22 — Guide punch slider

![Step 22: Guide punch slider](docs/assembly/step_22_gp_slider.png)

Lower the guide-punch slider: punch shank and the short dummy shank enter their bores; the fork plate hangs where the cam rib will run.

## Step 23 — Camshaft

![Step 23: Camshaft](docs/assembly/step_23_camshaft.png)

With every follower resting in place, drop the one-piece camshaft straight down into the open plate bearings. The three cam ribs land 0.3 mm above their follower bosses — nothing should touch. Set it to the reference angle (Geneva pin pointing straight down at the sprocket slot).

## Step 24 — Follower keys (camshaft ghosted)

![Step 24: Follower keys (camshaft ghosted)](docs/assembly/step_24_keys.png)

Close each follower window with its key so the cams drive both ways: the bail key presses in from the front through the bail plate (glue it); the guide-punch and hammer keys are snap pins inserted from the front corridor — they thread between the spokes of the cam hubs (keep the camshaft at the reference angle) and click into their plate holes.

## Step 25 — Sprocket shaft (camshaft at 180°, ghosted)

![Step 25: Sprocket shaft (camshaft at 180°, ghosted)](docs/assembly/step_25_sprocket.png)

FIRST turn the crank-end of the camshaft half a revolution (the Geneva pin now points up) — otherwise the pin blocks the drop path. Then lower the sprocket shaft into its bearings: both sprockets' teeth dip into the bed grooves.

## Step 26 — Sprocket bearing caps

![Step 26: Sprocket bearing caps](docs/assembly/step_26_spk_caps.png)

Drop the two tall caps into the plate slots over the sprocket journals and push the four d3 retention pins in from the OUTER plate faces (do this before the Geneva wheel blocks the left side).

## Step 27 — Geneva wheel

![Step 27: Geneva wheel](docs/assembly/step_27_wheel.png)

Slide the wheel onto the left hex stub (any of the 6 orientations works — pick the one where a slot points up at the driver pin and the locking ring nests in a concave arc). Snap the C-clip onto the stub groove.

## Step 28 — Geneva wheel clip

![Step 28: Geneva wheel clip](docs/assembly/step_28_spk_clip.png)

Snap the retaining C-clip into the stub groove in front of the wheel hub. Now turn the camshaft back to the reference angle.

## Step 29 — Camshaft bearing caps

![Step 29: Camshaft bearing caps](docs/assembly/step_29_cam_caps.png)

Drop the two short caps into the camshaft slots and push the four retention pins in from the outer faces. These caps take the punch reaction, so seat the pins fully.

## Step 30 — Crank

![Step 30: Crank](docs/assembly/step_30_crank.png)

Push the crank onto the right hex stub. Done — crank clockwise (looking from the crank side): tape advances, pins read, logic decides, punches fire at the bottom of the cycle.

## Done

![Assembled](docs/assembly/assembled.png)

Thread the tapes (leaders pre-punched with ~3 guide holes) by cranking, and the machine is ready to compute.
