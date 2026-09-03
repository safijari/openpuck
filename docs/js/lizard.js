import { $, log } from './util.js';

export const LZ_MAX = 32;
export const LZO = {NONE:0, KBD:1, MBTN:2, AXIS:3, SCROLL:4, CONSUMER:5};
const LZ_OUT_LABELS = {0:"(disabled)", 1:"Keyboard key", 2:"Mouse button", 3:"Mouse move", 4:"Scroll wheel", 5:"Media key"};
const LZ_BTNS = [
  [0x1,"A"],[0x2,"B"],[0x4,"X"],[0x8,"Y"],
  [0x10,"QAM (• • •)"],[0x40,"View"],[0x4000,"Menu"],[0x10000,"Steam"],
  [0x20,"R3 (stick click)"],[0x8000,"L3 (stick click)"],
  [0x80,"R4 (back upper-right)"],[0x100,"R5 (back lower-right)"],
  [0x20000,"L4 (back upper-left)"],[0x40000,"L5 (back lower-left)"],
  [0x200,"RB (bumper)"],[0x80000,"LB (bumper)"],
  [0x800000,"R2 (trigger pull)"],[0x8000000,"L2 (trigger pull)"],
  [0x2000,"D-pad Up"],[0x400,"D-pad Down"],[0x1000,"D-pad Left"],[0x800,"D-pad Right"],
  [0x400000,"Right pad click"],[0x4000000,"Left pad click"],
  [0x200000,"Right pad touch"],[0x2000000,"Left pad touch"],
  [0x10000000,"L-stick → right"],[0x20000000,"L-stick → left"],[0x40000000,"L-stick → down"],[0x80000000,"L-stick → up"],
];
const LZ_MODS = [[0x01,"Ctrl"],[0x02,"Shift"],[0x04,"Alt"],[0x08,"Win/⌘"]];
const LZ_KEYS = [[0,"— none —"]];
"ABCDEFGHIJKLMNOPQRSTUVWXYZ".split("").forEach((c,i)=>LZ_KEYS.push([0x04+i,c]));
"1234567890".split("").forEach((c,i)=>LZ_KEYS.push([0x1e+i,c]));
[[0x28,"Enter"],[0x29,"Esc"],[0x2a,"Backspace"],[0x2b,"Tab"],[0x2c,"Space"],
 [0x4f,"Arrow Right"],[0x50,"Arrow Left"],[0x51,"Arrow Down"],[0x52,"Arrow Up"],
 [0x4a,"Home"],[0x4d,"End"],[0x4b,"Page Up"],[0x4e,"Page Down"],[0x49,"Insert"],[0x4c,"Delete"],
 [0x2d,"- _"],[0x2e,"= +"],[0x46,"Print Screen"]].forEach(k=>LZ_KEYS.push(k));
for(let i=0;i<12;i++) LZ_KEYS.push([0x3a+i,"F"+(i+1)]);
const LZ_MBTNS = [[1,"Left click"],[2,"Right click"],[4,"Middle click"]];
const LZ_AXIS_SRC = [[0,"Right trackpad"],[1,"Left stick"],[2,"Gyro"]];
const LZ_GYRO_ACT = [[0,"Always"],[1,"While right pad touched"],[2,"While left stick deflected"],[3,"While hold-button held"]];
const LZ_CONSUMER = [[1,"Volume +"],[2,"Volume −"]];

export function lzBtnLabel(mask){ const m=LZ_BTNS.find(b=>b[0]===(mask>>>0)); return m?m[1]:(mask?("0x"+(mask>>>0).toString(16)):""); }
export let lizardBindings = [];
export let lizardBusy = false;
let lizardLoaded = false;
export function getLizardLoaded(){ return lizardLoaded; }
export function setLizardLoaded(v){ lizardLoaded=v; }
export function lizardCapable(lastP){ return !!(lastP && lastP[0]>=16); }

async function lizardExchange(fn, inflight){
  lizardBusy=true;
  for(let i=0;i<25 && inflight();i++) await new Promise(r=>setTimeout(r,20));
  try{ return await fn(); }
  catch(e){ log("lizard err: "+e.message); return null; }
  finally{ lizardBusy=false; }
}

export async function readLizard(dev, epIn){
  let acc=new Uint8Array(0);
  for(let guard=0; guard<64; guard++){
    const r=await dev.transferIn(epIn,128);
    if(r.status!=="ok") break;
    const d=new Uint8Array(r.data.buffer);
    const m=new Uint8Array(acc.length+d.length); m.set(acc); m.set(d,acc.length); acc=m;
    let i=0; while(i<acc.length && acc[i]!==0xAA) i++;
    if(i>0) acc=acc.slice(i);
    if(acc.length<2) continue;
    const count=acc[1], total=2+count*16;
    if(acc.length>=total){
      const out=[];
      for(let b=0;b<count;b++){
        const q=2+b*16;
        out.push({
          outType:acc[q],
          od:[...acc.slice(q+1,q+8)],
          trig:((acc[q+8])|(acc[q+9]<<8)|(acc[q+10]<<16)|(acc[q+11]<<24))>>>0,
          hold:((acc[q+12])|(acc[q+13]<<8)|(acc[q+14]<<16)|(acc[q+15]<<24))>>>0,
        });
      }
      return out;
    }
  }
  return null;
}

export async function loadLizard(dev, epIn, send, inflight){
  await lizardExchange(async()=>{
    await send([0x11]);
    const b=await readLizard(dev, epIn);
    if(b){ lizardBindings=b; renderLizard(); log(`lizard map loaded — ${b.length} bindings`); }
  }, inflight);
}

export async function saveLizard(dev, epIn, send, inflight, lastP){
  if(!dev || !lizardCapable(lastP)) return;
  const digital=b=>b.outType===LZO.KBD||b.outType===LZO.MBTN||b.outType===LZO.CONSUMER;
  const valid=lizardBindings.filter(b=> b.outType!==LZO.NONE && (!digital(b) || (b.trig>>>0)||(b.hold>>>0)) );
  const dropped=lizardBindings.length-valid.length;
  await lizardExchange(async()=>{
    await send([0x13, valid.length&0xff]);
    for(let i=0;i<valid.length;i++){
      const b=valid[i];
      const od=(b.od||[]).slice(0,7); while(od.length<7) od.push(0);
      const t=b.trig>>>0, h=b.hold>>>0;
      await send([0x12, i, b.outType, ...od.map(x=>x&0xff),
        t&0xff,(t>>>8)&0xff,(t>>>16)&0xff,(t>>>24)&0xff,
        h&0xff,(h>>>8)&0xff,(h>>>16)&0xff,(h>>>24)&0xff]);
    }
    await send([0x14]);
    const b=await readLizard(dev, epIn); if(b){ lizardBindings=b; renderLizard(); }
    log(`lizard map saved — ${valid.length} bindings`+(dropped?` (skipped ${dropped} incomplete: no trigger/hold)`:""));
  }, inflight);
}

export async function resetLizard(dev, epIn, send, inflight, lastP){
  if(!dev || !lizardCapable(lastP)) return;
  if(!confirm("Reset the lizard map to the built-in defaults? Unsaved edits are lost.")) return;
  await lizardExchange(async()=>{
    await send([0x15]);
    const b=await readLizard(dev, epIn); if(b){ lizardBindings=b; renderLizard(); }
    log("lizard map reset to defaults");
  }, inflight);
}

function mkSel(opts, val, onchange){
  const s=document.createElement("select"); s.style.flex="0 1 auto"; s.style.minWidth="0";
  for(const [v,l] of opts){ const o=document.createElement("option"); o.value=v; o.textContent=l; s.appendChild(o); }
  s.value=val; s.addEventListener("change",()=>onchange(+s.value));
  return s;
}
export function renderLizard(){
  const host=$("#lizardList"); host.innerHTML="";
  lizardBindings.forEach((b,idx)=>host.appendChild(buildLizardRow(b,idx)));
  $("#lzStatus").textContent = `${lizardBindings.length} / ${LZ_MAX} bindings`;
  $("#lzAdd").disabled = lizardBindings.length>=LZ_MAX;
}
function buildLizardRow(b, idx){
  const row=document.createElement("div");
  row.style.cssText="display:flex;align-items:center;gap:8px;flex-wrap:wrap;background:#0e1017;border:1px solid #232a3a;border-radius:8px;padding:8px 10px;margin:8px 0";
  const analog = (b.outType===LZO.AXIS || b.outType===LZO.SCROLL);

  if(!analog){
    const when=document.createElement("span"); when.className="note"; when.textContent="When"; when.style.flex="0 0 auto"; row.appendChild(when);
    row.appendChild(mkSel([[0,"— pick input —"],...LZ_BTNS], b.trig, v=>{ b.trig=v>>>0; }));
    const plus=document.createElement("span"); plus.className="note"; plus.textContent="+ hold"; plus.style.flex="0 0 auto"; row.appendChild(plus);
    row.appendChild(mkSel([[0,"(none)"],...LZ_BTNS], b.hold, v=>{ b.hold=v>>>0; }));
  } else {
    const lab=document.createElement("span"); lab.className="note"; lab.textContent="Analog source"; lab.style.flex="0 0 auto"; row.appendChild(lab);
  }

  const arrow=document.createElement("span"); arrow.textContent="→"; arrow.style.color="var(--mut)"; row.appendChild(arrow);
  row.appendChild(mkSel(Object.entries(LZ_OUT_LABELS).map(([v,l])=>[+v,l]), b.outType, v=>{
    b.outType=v;
    if(v===LZO.AXIS){ b.od=[0,0,0,0,0,0,0]; }
    else if(v===LZO.SCROLL){ b.od=[0,0,0,0,0,0,0]; }
    else if(v===LZO.MBTN){ b.od=[1,0,0,0,0,0,0]; }
    else if(v===LZO.CONSUMER){ b.od=[1,0,0,0,0,0,0]; }
    else if(v===LZO.KBD){ b.od=[0,0,0,0,0,0,0]; }
    renderLizard();
  }));

  if(b.outType===LZO.KBD){
    for(const [bit,name] of LZ_MODS){
      const w=document.createElement("label"); w.style.cssText="flex:0 0 auto;display:flex;align-items:center;gap:3px;color:var(--fg)";
      const cb=document.createElement("input"); cb.type="checkbox"; cb.checked=!!(b.od[0]&bit);
      cb.addEventListener("change",()=>{ b.od[0]=cb.checked?(b.od[0]|bit):(b.od[0]&~bit); });
      w.appendChild(cb); w.appendChild(document.createTextNode(name)); row.appendChild(w);
    }
    row.appendChild(mkSel(LZ_KEYS, b.od[1]||0, v=>{ b.od[1]=v; for(let k=2;k<7;k++) b.od[k]=0; }));
  } else if(b.outType===LZO.MBTN){
    row.appendChild(mkSel(LZ_MBTNS, b.od[0]||1, v=>{ b.od[0]=v; }));
  } else if(b.outType===LZO.AXIS){
    row.appendChild(mkSel(LZ_AXIS_SRC, b.od[0]||0, v=>{ b.od[0]=v; renderLizard(); }));
    if((b.od[0]||0)===2){
      const g=document.createElement("span"); g.className="note"; g.textContent="when"; g.style.flex="0 0 auto"; row.appendChild(g);
      row.appendChild(mkSel(LZ_GYRO_ACT, b.od[1]||0, v=>{ b.od[1]=v; }));
    }
  } else if(b.outType===LZO.SCROLL){
    const s=document.createElement("span"); s.className="note"; s.textContent="Left trackpad → scroll"; row.appendChild(s);
  } else if(b.outType===LZO.CONSUMER){
    row.appendChild(mkSel(LZ_CONSUMER, b.od[0]||1, v=>{ b.od[0]=v; }));
  }

  const del=document.createElement("button"); del.textContent="✕"; del.title="Delete binding";
  del.style.cssText="margin-left:auto;flex:0 0 auto;padding:4px 10px";
  del.onclick=()=>{ lizardBindings.splice(idx,1); renderLizard(); };
  row.appendChild(del);
  return row;
}
