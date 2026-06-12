"""Bake meshes + pose tracks into a single self-contained HTML viewer
(three.js via CDN). Slider = camshaft angle, dropdown = input case (L,C,R),
play button cranks continuously. Run after build_stls.py:
    .venv/bin/python scripts/export_viewer.py
"""

import sys, json, base64, itertools
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mech110 import assembly

PARTS_DIR = ROOT / "out" / "parts"
OUT = ROOT / "out" / "viewer.html"
DTH = 2.0
THETAS = np.arange(0.0, 360.0 + DTH, DTH)

COLORS = {
    "camshaft": "#d4a017", "crank": "#d4a017", "geneva_wheel": "#c0392b",
    "sprocket_shaft": "#c0392b", "spk_clip": "#c0392b",
    "bail": "#2980b9", "key_bail": "#5dade2", "pin": "#1abc9c",
    "collar": "#16a085", "and3": "#8e44ad", "rocker_c": "#e67e22",
    "rocker_r": "#e67e22", "swing_arm": "#d35400", "block": "#e74c3c",
    "stem_clip": "#e74c3c", "gp_slider": "#27ae60", "key_gp": "#2ecc71",
    "hammer": "#2c3e50", "key_ham": "#34495e", "data_punch": "#c0392b",
    "return_rocker": "#7f8c8d", "tape_a": "#f7f1e3", "tape_b": "#f7f1e3",
    "bed": "#bdc3c7", "base": "#95a5a6", "plate_left": "#aab7b8",
    "plate_right": "#aab7b8", "comb_bridge": "#85929e",
    "top_guide": "#85929e", "punch_block": "#85929e", "gallows": "#85929e",
    "lcol_left": "#85929e", "lcol_right": "#85929e", "axle": "#7f8c8d",
    "cover_a": "#d5dbdb", "cover_b": "#d5dbdb",
}


def b64(arr):
    return base64.b64encode(np.ascontiguousarray(arr).tobytes()).decode()


def mesh_entry(m, color):
    return {"v": b64(m.vertices.astype(np.float32)),
            "f": b64(m.faces.astype(np.uint32)),
            "nv": len(m.vertices), "nf": len(m.faces), "c": color}


def main():
    parts = {}
    for f in sorted(PARTS_DIR.glob("*.stl")):
        stem = assembly.strip_step_prefix(f.stem)
        if stem.startswith("tape_a_"):
            continue
        m = trimesh.load(str(f), force="mesh")
        base = stem
        key = ("pin" if base == "pin" else
               "collar" if base == "collar" else base)
        parts[base] = mesh_entry(m, COLORS.get(key, COLORS.get(
            base.rstrip("_lcr").rstrip("_"), "#cccccc")))
    tapes_a = {}
    for bits in itertools.product((0, 1), repeat=3):
        f = PARTS_DIR / ("04_tape_a_%d%d%d.stl" % bits)
        m = trimesh.load(str(f), force="mesh")
        tapes_a["".join(map(str, bits))] = mesh_entry(m, COLORS["tape_a"])

    # instances: viewer object name -> mesh key
    objects = {}
    for name in assembly.STATIC | assembly.MOVING_FROM_POSES:
        key = ("pin" if name.startswith("pin_") else
               "collar" if name.startswith("collar_") else
               "cap_pin" if name.startswith("cap_pin_") else name)
        if key in parts or name == "tape_a":
            objects[name] = key

    # pose tracks
    tracks = {}
    for bits in itertools.product((0, 1), repeat=3):
        case = "".join(map(str, bits))
        frames = {}
        for name in objects:
            frames[name] = []
        inst_off = {}
        for base, insts in assembly.INSTANCES.items():
            for iname, T in insts:
                inst_off[iname] = T
        for th in THETAS:
            P = assembly.poses(th, bits)
            for name in objects:
                T = P.get(name, np.eye(4))
                if name.startswith("cap_pin_"):
                    T = inst_off[name]
                pos = T[:3, 3]
                quat = R.from_matrix(T[:3, :3]).as_quat()  # x,y,z,w
                frames[name].append(np.concatenate([pos, quat]))
        tracks[case] = {n: b64(np.array(v, dtype=np.float32))
                        for n, v in frames.items()}

    data = {"dth": DTH, "n": len(THETAS), "parts": parts, "tapesA": tapes_a,
            "objects": objects, "tracks": tracks}

    html = HTML.replace("__DATA__", json.dumps(data))
    OUT.write_text(html)
    print(f"wrote {OUT} ({OUT.stat().st_size/1e6:.1f} MB)")


HTML = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>MECH-110 — Rule 110 machine</title>
<style>
 body{margin:0;background:#1a1a2e;color:#eee;font-family:system-ui;overflow:hidden}
 #ui{position:absolute;top:10px;left:10px;background:#0008;padding:10px 14px;
     border-radius:8px;z-index:5;max-width:330px}
 #parts{max-height:42vh;overflow-y:auto;font-size:12px;column-count:2}
 label{display:block;font-size:12px;margin:2px 0}
 input[type=range]{width:300px}
 .lbl{display:inline-block;width:80px}
 button,select{margin:2px}
</style></head><body>
<div id="ui">
 <div><b>MECH-110</b> — mechanical Rule 110</div>
 <div><span class="lbl">crank θ</span><input id="th" type="range" min="0" max="360" step="0.5" value="0">
      <span id="thv">0°</span></div>
 <div><span class="lbl">input L,C,R</span><select id="case"></select>
      <span id="rule"></span></div>
 <div><button id="play">▶ play</button>
      <button id="x2">speed ×2</button>
      <span id="phase"></span></div>
 <div id="parts"></div>
</div>
<script type="importmap">{"imports":{"three":"https://unpkg.com/three@0.160.0/build/three.module.js","three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const D = __DATA__;
const dec=(s,T)=>new T(Uint8Array.from(atob(s),c=>c.charCodeAt(0)).buffer);
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x1a1a2e);
const cam3=new THREE.PerspectiveCamera(45,innerWidth/innerHeight,1,2000);
cam3.position.set(160,-160,200); cam3.up.set(0,0,1);
const ren=new THREE.WebGLRenderer({antialias:true});
ren.setSize(innerWidth,innerHeight); document.body.appendChild(ren.domElement);
const ctl=new OrbitControls(cam3,ren.domElement); ctl.target.set(20,50,80);
scene.add(new THREE.AmbientLight(0xffffff,.45));
const dl=new THREE.DirectionalLight(0xffffff,1.2); dl.position.set(1,-2,3); scene.add(dl);
const dl2=new THREE.DirectionalLight(0x8888ff,.5); dl2.position.set(-2,1,-1); scene.add(dl2);
function geom(p){const g=new THREE.BufferGeometry();
 g.setAttribute('position',new THREE.BufferAttribute(dec(p.v,Float32Array),3));
 g.setIndex(new THREE.BufferAttribute(dec(p.f,Uint32Array),1));
 g.computeVertexNormals(); return g;}
const geos={}; for(const k in D.parts) geos[k]=geom(D.parts[k]);
const tapeGeos={}; for(const k in D.tapesA) tapeGeos[k]=geom(D.tapesA[k]);
const meshes={}, ui=document.getElementById('parts');
for(const name in D.objects){
 const key=D.objects[name];
 const g=(name==='tape_a')?tapeGeos['000']:geos[key];
 const mat=new THREE.MeshStandardMaterial({color:D.parts[key]?D.parts[key].c:'#ccc',
   metalness:.1,roughness:.65});
 if(name.startsWith('tape')) {mat.side=THREE.DoubleSide;}
 const m=new THREE.Mesh(g,mat); m.matrixAutoUpdate=false; scene.add(m); meshes[name]=m;
 const l=document.createElement('label');
 const c=document.createElement('input'); c.type='checkbox'; c.checked=true;
 c.onchange=()=>m.visible=c.checked; l.appendChild(c);
 l.appendChild(document.createTextNode(' '+name)); ui.appendChild(l);
}
const tracks={}; for(const cs in D.tracks){tracks[cs]={};
 for(const n in D.tracks[cs]) tracks[cs][n]=dec(D.tracks[cs][n],Float32Array);}
const sel=document.getElementById('case');
const RULE={'111':0,'110':1,'101':1,'100':0,'011':1,'010':1,'001':1,'000':0};
for(const cs of ['000','001','010','011','100','101','110','111']){
 const o=document.createElement('option'); o.value=cs;
 o.text=cs+' → '+RULE[cs]; sel.appendChild(o);}
sel.value='011';
function phase(th){
 if(th<120)return 'advance tape'; if(th<130)return 'dwell';
 if(th<170)return 'pins settle'; if(th<205)return 'logic decides';
 if(th<280)return 'punch'; if(th<290)return 'dwell'; if(th<355)return 'reset';
 return 'dwell';}
const thEl=document.getElementById('th');
function setPose(th){
 const cs=sel.value;
 meshes['tape_a'].geometry=tapeGeos[cs];
 const i=Math.min(Math.floor(th/D.dth),D.n-2), f=th/D.dth-i;
 for(const n in meshes){const tr=tracks[cs][n]; if(!tr) continue;
  const a=i*7,b=(i+1)*7;
  const px=tr[a]+(tr[b]-tr[a])*f, py=tr[a+1]+(tr[b+1]-tr[a+1])*f,
        pz=tr[a+2]+(tr[b+2]-tr[a+2])*f;
  const qa=new THREE.Quaternion(tr[a+3],tr[a+4],tr[a+5],tr[a+6]);
  const qb=new THREE.Quaternion(tr[b+3],tr[b+4],tr[b+5],tr[b+6]);
  qa.slerp(qb,f);
  meshes[n].matrix.compose(new THREE.Vector3(px,py,pz),qa,new THREE.Vector3(1,1,1));
 }
 document.getElementById('thv').textContent=th.toFixed(0)+'°';
 document.getElementById('phase').textContent='  ['+phase(th%360)+']';
 document.getElementById('rule').textContent=' punches: '+RULE[sel.value];
}
let playing=false,speed=40; // deg per second
document.getElementById('play').onclick=e=>{playing=!playing;
 e.target.textContent=playing?'⏸ pause':'▶ play';};
document.getElementById('x2').onclick=()=>speed=speed>=160?20:speed*2;
sel.onchange=()=>setPose(parseFloat(thEl.value));
thEl.oninput=()=>{playing=false;document.getElementById('play').textContent='▶ play';
 setPose(parseFloat(thEl.value));};
let last=performance.now();
function tick(t){requestAnimationFrame(tick);
 if(playing){let th=parseFloat(thEl.value)+(t-last)/1000*speed;
  if(th>360)th-=360; thEl.value=th; setPose(th);}
 last=t; ctl.update(); ren.render(scene,cam3);}
setPose(0); tick(last);
onresize=()=>{cam3.aspect=innerWidth/innerHeight;cam3.updateProjectionMatrix();
 ren.setSize(innerWidth,innerHeight);};
</script></body></html>
"""

if __name__ == "__main__":
    main()
