"""Render the verified assembly sequence into step-by-step images and an
ASSEMBLY.md guide.

Walks scripts/verify_assembly.py's STEPS (the collision-checked sequence),
maintains the placed-parts state (including the camshaft-at-180 repose), and
renders one image per logical group: already-placed parts muted, new parts
highlighted orange and offset along their insertion direction with an arrow.

Run: .venv/bin/python scripts/render_assembly_guide.py
"""

import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import verify_assembly as VA

OUT = ROOT / "docs" / "assembly"
OUT.mkdir(parents=True, exist_ok=True)

COLORS = {
    "camshaft": "#d4a017", "crank": "#d4a017", "geneva_wheel": "#c0392b",
    "sprocket_shaft": "#c0392b", "spk_clip": "#c0392b",
    "bail": "#2980b9", "key_bail": "#5dade2",
    "and3": "#8e44ad", "rocker_c": "#e67e22", "rocker_r": "#e67e22",
    "swing_arm": "#d35400", "block": "#c0392b",
    "gp_slider": "#27ae60", "key_gp": "#2ecc71",
    "hammer": "#2c3e50", "key_ham": "#34495e", "data_punch": "#c0392b",
    "return_rocker": "#7f8c8d", "tape_a": "#efe8d8", "tape_b": "#efe8d8",
    "bed": "#c8ced3", "base": "#aab4b8", "plate_left": "#bcc6cb",
    "plate_right": "#bcc6cb", "comb_bridge": "#8d9aa5",
    "punch_block": "#8d9aa5", "gallows": "#8d9aa5",
    "lcol_left": "#8d9aa5", "lcol_right": "#8d9aa5", "axle": "#7f8c8d",
    "cover_a": "#d5dde0", "cover_b": "#d5dde0",
}
HILITE = "#ff4500"

# decimated render meshes
try:
    import fast_simplification  # noqa: F401
    HAVE_DECIM = True
except ImportError:
    HAVE_DECIM = False

_rm = {}


def rmesh(stem):
    if stem not in _rm:
        m = VA.meshes[stem].copy()
        budget = 14000 if stem == "camshaft" else 4000
        if HAVE_DECIM and len(m.faces) > budget:
            m = m.simplify_quadric_decimation(face_count=budget)
        _rm[stem] = m
    return _rm[stem]


LIGHT = np.array([0.35, -0.5, 0.8])
LIGHT = LIGHT / np.linalg.norm(LIGHT)


def part_tris(name, pose):
    m = rmesh(VA.stl_key(name)).copy()
    m.apply_transform(pose)
    tris = m.vertices[m.faces]
    n = m.face_normals
    shade = 0.55 + 0.45 * np.clip(n @ LIGHT, 0, 1)
    return tris, shade


def hex2rgb(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)])


def render(placed, highlight, arrows, path, title, view=(32, -58),
           translucent=()):
    fig = plt.figure(figsize=(11, 8.5), dpi=110)
    ax = fig.add_subplot(111, projection="3d")
    all_tris, all_cols = [], []
    for name, pose in placed.items():
        if name in highlight or name.startswith("tape_a_"):
            continue
        tris, shade = part_tris(name, pose)
        base = hex2rgb(COLORS.get(VA.stl_key(name),
                                  COLORS.get(name, "#b8b8b8")))
        alpha = 0.13 if name in translucent else 1.0
        cols = np.empty((len(tris), 4))
        cols[:, :3] = base[None, :] * shade[:, None]
        cols[:, 3] = alpha
        all_tris.append(tris)
        all_cols.append(cols)
    for name, pose in highlight.items():
        tris, shade = part_tris(name, pose)
        base = hex2rgb(HILITE)
        cols = np.empty((len(tris), 4))
        cols[:, :3] = base[None, :] * shade[:, None]
        cols[:, 3] = 1.0
        all_tris.append(tris)
        all_cols.append(cols)
    if all_tris:
        tris = np.concatenate(all_tris)
        cols = np.concatenate(all_cols)
        pc = Poly3DCollection(tris, facecolors=cols, edgecolors="none",
                              zsort="average")
        ax.add_collection3d(pc)
    for (p0, p1) in arrows:
        v = np.array(p1) - np.array(p0)
        ax.quiver(*p0, *v, color="#ff4500", arrow_length_ratio=0.18,
                  linewidth=2.2)
    ax.set_xlim(-44, 84)
    ax.set_ylim(-22, 118)
    ax.set_zlim(0, 140)
    ax.set_box_aspect((128, 140, 140))
    ax.view_init(elev=view[0], azim=view[1])
    ax.set_axis_off()
    ax.set_title(title, fontsize=13, pad=0)
    fig.tight_layout(pad=0.1)
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)


# ---------------------------------------------------------------- groups
# (slug, title, caption, options)
GROUPS = [
    ("base", ["base"], "Base plate",
     "Lay the base flat. The rectangular mortises take the frame-plate "
     "tabs; everything else stacks on top."),
    ("plates", ["plate_left", "plate_right"], "Frame plates",
     "Drop both plates straight down so their three bottom tabs enter the "
     "base mortises. The open-top U-slots are the shaft bearings; the small "
     "side holes near the slots take the bearing-cap retention pins later."),
    ("bed", ["bed"], "Tape bed",
     "Lower the bed between the plates until its four legs sit on the base. "
     "Top side up: tape rails, sprocket grooves, the three reader-pin holes "
     "and the two punch die holes. All later peg-mounted parts plug into "
     "the bed's peg holes."),
    ("tapes", ["tape_a", "tape_b"], "Paper tapes (reference)",
     "Shown for reference only: the tapes are threaded LAST, by cranking — "
     "the sprocket teeth carry them in (hand-punch ~3 guide holes into each "
     "leader first). Input tape A runs in the near lane, output tape B in "
     "the far lane; both travel toward -X (to the left here)."),
    ("covers", ["cover_a", "cover_b"], "Sprocket hold-down covers",
     "Press the two covers' pegs into the bed holes flanking the sprocket "
     "station. They keep the paper wrapped on the sprocket teeth."),
    ("comb", ["comb_bridge"], "Reader comb bridge",
     "Press the comb bridge down onto its four bed pegs. It spans tape A "
     "as a bridge: its underside is the stripper, the three bores guide the "
     "reader pins, and the two end towers carry the vertical grooves that "
     "guide the bail and the AND3 plate."),
    ("pins", ["pin_l", "pin_c", "pin_r"], "Reader pins",
     "Drop the three identical pins tip-first into the comb bores. They "
     "fall through into the bed holes for now; lift them while clipping on "
     "the collars in the next step."),
    ("bail", ["bail"], "Lift bail",
     "Slide the bail down: its two end tabs ride the front grooves of the "
     "comb towers. The three U-slotted fingers pass around the pin shafts — "
     "they will lift the pins by their collars."),
    ("collars", ["collar_l", "collar_c", "collar_r"], "Pin collars",
     "Hold each pin raised and snap a C-collar sideways (from the front) "
     "into its groove. The collar rests on the bail finger below; the AND3 "
     "feet and rocker pads will ride on its top face."),
    ("punch_block", ["punch_block"], "Punch block",
     "Press the punch block onto its four bed pegs. It bridges tape B: "
     "stripper underside, guide bores for both punches and the hammer "
     "shanks, the return-rocker ears at the back, and the shelf rail that "
     "parks the interposer block."),
    ("lcols", ["lcol_left", "lcol_right"], "Rocker axle columns",
     "Press the two L-columns into their bed holes in the corridor between "
     "the tapes. Their open cradles will hold the rocker axle."),
    ("rockers", ["rocker_c", "rocker_r"], "OR rockers",
     "Set the two rockers in place: each ledge pad rests on its pin collar "
     "(C and R), the long arms route around the cam stack to the east side "
     "of the machine. Their hubs line up with the axle cradles."),
    ("axle", ["axle"], "Rocker axle",
     "Thread the axle rod westward (from the right) through the right "
     "cradle, both rocker hubs and the left cradle until the head seats. "
     "A drop of glue at the plain end keeps it from walking."),
    ("and3", ["and3"], "AND3 guillotine",
     "Lower the AND3 plate: its end tabs ride the rear grooves of the comb "
     "towers, the three crescent feet land around the pin collars, and the "
     "long blocker arm snakes west and over to the punch station. It must "
     "slide freely — it is lifted by whichever pin sits highest."),
    ("gallows", ["gallows"], "Swing-arm gallows",
     "Press the gallows onto its two bed pegs behind tape B. The vertical "
     "pin hanging from its arm is the swing-arm pivot."),
    ("swing_in", ["swing_arm", "block"], "Interposer arm (tilted entry)",
     "Hang the block's stem in the arm's end bore first (off the machine). "
     "Then, with the arm rotated about 34 degrees clockwise from its rest "
     "attitude, raise the assembly so the hub slides onto the gallows pin. "
     "The tilt routes the block up through the free column behind the "
     "return-rocker ears — straight-in does not fit."),
    ("swing_home", ["swing_arm", "block"], "Interposer arm (rotate home)",
     "Rotate the arm counterclockwise to its rest position: the block "
     "settles onto the shelf rail, ready to swing over the punch flange.",
     dict(repose=True)),
    ("pivot_clip", ["pivot_clip"], "Pivot clip",
     "Push the small C-clip up inside the foot-ring bore and snap it into "
     "the groove near the pivot pin's tip — it keeps the arm from lifting "
     "off."),
    ("return_rocker", ["return_rocker"], "Punch return rocker",
     "Set the return rocker between the ears on the punch block's rear "
     "extension, counterweight toward the back, tip pad forward under "
     "where the punch flange will sit."),
    ("rr_axle", ["rr_axle"], "Return-rocker axle",
     "Push the small axle pin eastward through the ears and the rocker hub "
     "until the head seats against the west ear."),
    ("hammer", ["hammer", "data_punch"], "Hammer + data punch",
     "Off the machine, slide the data punch sideways into the hammer's "
     "C-lip through its front opening (flange above the lip). Lower both "
     "together: the hammer's two shanks and the punch enter their bores; "
     "the punch flange comes to rest on the return-rocker pad."),
    ("gp_slider", ["gp_slider"], "Guide punch slider",
     "Lower the guide-punch slider: punch shank and the short dummy shank "
     "enter their bores; the fork plate hangs where the cam rib will run."),
    ("camshaft", ["camshaft"], "Camshaft",
     "With every follower resting in place, drop the one-piece camshaft "
     "straight down into the open plate bearings. The three cam ribs land "
     "0.3 mm above their follower bosses — nothing should touch. Set it to "
     "the reference angle (Geneva pin pointing straight down at the "
     "sprocket slot)."),
    ("keys", ["key_bail", "key_gp", "key_ham"], "Follower keys (camshaft ghosted)",
     "Close each follower window with its key so the cams drive both ways: "
     "the bail key presses in from the front through the bail plate "
     "(glue it); the guide-punch and hammer keys are snap pins inserted "
     "from the front corridor — they thread between the spokes of the cam "
     "hubs (keep the camshaft at the reference angle) and click into their "
     "plate holes.",
     dict(xtl={"camshaft"})),
    ("sprocket", ["sprocket_shaft"], "Sprocket shaft (camshaft at 180°, ghosted)",
     "FIRST turn the crank-end of the camshaft half a revolution (the "
     "Geneva pin now points up) — otherwise the pin blocks the drop path. "
     "Then lower the sprocket shaft into its bearings: both sprockets' "
     "teeth dip into the bed grooves.",
     dict(xtl={"camshaft"})),
    ("spk_caps", ["cap_spk_left", "cap_spk_right",
                  "cp_sl1", "cp_sl2", "cp_sr1", "cp_sr2"],
     "Sprocket bearing caps",
     "Drop the two tall caps into the plate slots over the sprocket "
     "journals and push the four d3 retention pins in from the OUTER plate "
     "faces (do this before the Geneva wheel blocks the left side)."),
    ("wheel", ["geneva_wheel"], "Geneva wheel",
     "Slide the wheel onto the left hex stub (any of the 6 orientations "
     "works — pick the one where a slot points up at the driver pin and "
     "the locking ring nests in a concave arc). Snap the C-clip onto the "
     "stub groove."),
    ("spk_clip", ["spk_clip"], "Geneva wheel clip",
     "Snap the retaining C-clip into the stub groove in front of the "
     "wheel hub. Now turn the camshaft back to the reference angle."),
    ("cam_caps", ["cap_cam_left", "cap_cam_right",
                  "cp_cl1", "cp_cl2", "cp_cr1", "cp_cr2"],
     "Camshaft bearing caps",
     "Drop the two short caps into the camshaft slots and push the four "
     "retention pins in from the outer faces. These caps take the punch "
     "reaction, so seat the pins fully.",
     dict(post_camshaft_home=True)),
    ("crank", ["crank"], "Crank",
     "Push the crank onto the right hex stub. Done — crank clockwise "
     "(looking from the crank side): tape advances, pins read, logic "
     "decides, punches fire at the bottom of the cycle."),
]


def main():
    placed = {}
    gi = 0
    md = []
    step_iter = iter(enumerate(VA.STEPS))
    images = []

    for slug, names, title, caption, *opt in [
            (g[0], g[1], g[2], g[3], *(g[4:] or [{}])) for g in GROUPS]:
        opts = opt[0] if opt else {}
        repose = opts.get("repose", False)
        remaining = set(names)
        arrows = []
        highlight = {}
        # consume steps until this group's parts are all (re)placed
        while remaining:
            i, st = next(step_iter)
            stage = {n: st["poses"].get(n, VA.pose0(n)) for n in st["names"]}
            off = st["path"][0][:3, 3].copy()
            for n in st["names"]:
                placed[n] = st["path"][-1] @ stage[n]
                if n in remaining:
                    remaining.discard(n)
                    if repose or not np.linalg.norm(off):
                        highlight[n] = placed[n]
                    else:
                        d = off / max(np.linalg.norm(off), 1e-9)
                        disp = d * min(np.linalg.norm(off), 26.0)
                        T = np.eye(4)
                        T[:3, 3] = disp
                        highlight[n] = T @ placed[n]
                        m = rmesh(VA.stl_key(n)).copy()
                        m.apply_transform(highlight[n])
                        c = m.bounds.mean(axis=0)
                        arrows.append((c, c - disp * 0.92))
                elif st.get("check") is False and len(st["names"]) == 1:
                    pass  # silent repose (camshaft rotations)

        gi += 1
        fname = f"step_{gi:02d}_{slug}.png"
        view = opts.get("view", (32, -58))
        translucent = ({"plate_left", "plate_right"} if gi > 2 else set()
                       ) | opts.get("xtl", set())
        render(placed, highlight, arrows, OUT / fname,
               f"Step {gi}: {title}", view=view, translucent=translucent)
        images.append(fname)
        md.append(f"## Step {gi} — {title}\n\n"
                  f"![Step {gi}: {title}](docs/assembly/{fname})\n\n"
                  f"{caption}\n")
        print(f"rendered {fname}")

    # hero shot
    render(placed, {}, [], OUT / "assembled.png", "MECH-110 assembled",
           view=(28, -58), translucent={"plate_left", "plate_right"})

    bom = """\
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
"""

    text = (
        "# MECH-110 assembly guide\n\n"
        "Step-by-step assembly in the **verified order** — this exact "
        "sequence (including insertion directions and the camshaft "
        "rotation in step 25) is collision-checked by "
        "`scripts/verify_assembly.py`; the images are rendered from the "
        "same data by `scripts/render_assembly_guide.py`.\n\n"
        "New parts in each step are shown **orange**, slightly offset "
        "along their insertion direction (arrow). The near frame plate is "
        "drawn translucent so you can see inside.\n\n"
        "No screws, springs or metal parts. Snap fits are printed; keep "
        "cyanoacrylate handy for the two press-fit pins noted below.\n\n"
        "## Bill of parts\n\n" + bom + "\n---\n\n" + "\n".join(md) +
        "\n## Done\n\n![Assembled](docs/assembly/assembled.png)\n\n"
        "Thread the tapes (leaders pre-punched with ~3 guide holes) by "
        "cranking, and the machine is ready to compute.\n"
    )
    (ROOT / "ASSEMBLY.md").write_text(text)
    print("wrote ASSEMBLY.md +", len(images) + 1, "images")


if __name__ == "__main__":
    main()
