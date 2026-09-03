const DPAD = {12:"D-pad Up",13:"D-pad Down",14:"D-pad Left",15:"D-pad Right"};
export const TYPE_DEFS = [
  {key:"XBOX", name:"Xbox", labels:{1:"A",2:"B",3:"X",4:"Y",5:"LB",6:"RB",7:"L3",8:"R3",9:"Back",10:"Start",11:"Guide",19:"LT",20:"RT",...DPAD}},
  {key:"SWITCH", name:"Switch", labels:{1:"A",2:"B",3:"X",4:"Y",5:"L",6:"R",7:"L-Stick",8:"R-Stick",9:"Minus",10:"Plus",11:"Home",19:"ZL",20:"ZR",...DPAD,18:"Capture / Screenshot"}},
  {key:"DS4", name:"DS4", labels:{1:"Cross",2:"Circle",3:"Square",4:"Triangle",5:"L1",6:"R1",7:"L3",8:"R3",9:"Create",10:"Options",11:"PS",19:"L2",20:"R2",...DPAD,16:"Touchpad Click"}},
  {key:"DS5", name:"DS5", labels:{1:"Cross",2:"Circle",3:"Square",4:"Triangle",5:"L1",6:"R1",7:"L3",8:"R3",9:"Create",10:"Options",11:"PS",19:"L2",20:"R2",...DPAD,16:"Touchpad Click",17:"Mute"}},
];
export function etypeForMode(m){ switch(m){case 1:case 10:return 0; case 2:case 4:return 1; case 6:case 8:case 9:return 2; case 5:case 7:return 3; default:return -1;} }

export const PAD_STICK_FIELD0 = 80;
export const PAD_STICK_OPTS = [[0,"Off (touchpad)"],[1,"Left stick"],[2,"Right stick"]];
const PAD_STICK_LABELS = ["Left trackpad → stick","Right trackpad → stick"];
const BACK_LABELS = ["L4 (back upper-left)","R4 (back upper-right)","L5 (back lower-left)","R5 (back lower-right)"];

export const typeEls = [];
export let curTab=0, tabInited=false;
export function resetTabInited(){ tabInited=false; }
export function markTabInited(){ tabInited=true; }
export function setTab(et){
  curTab=et;
  typeEls.forEach((rec,i)=>{
    rec.sec.style.display = (i===et) ? "" : "none";
    rec.tab.classList.toggle("active", i===et);
  });
}

function mkSelect(def, includeNone){
  const sel=document.createElement("select");
  if(includeNone){ const o=document.createElement("option"); o.value=0; o.textContent="— none —"; sel.appendChild(o); }
  else { const o=document.createElement("option"); o.value=0; o.textContent="Default (per-mode)"; sel.appendChild(o); }
  for(const c of Object.keys(def.labels).map(Number).sort((a,b)=>a-b)){
    const o=document.createElement("option"); o.value=c; o.textContent=def.labels[c]; sel.appendChild(o); }
  return sel;
}

export function buildTypeCfgs(setField){
  const host=document.getElementById("typeCfgs");
  const bar=document.createElement("div"); bar.style.cssText="display:flex;gap:6px;flex-wrap:wrap;margin-bottom:12px";
  host.appendChild(bar);
  TYPE_DEFS.forEach((def,et)=>{
    const tab=document.createElement("button"); tab.className="slot-tab"; tab.style.cursor="pointer";
    tab.innerHTML='<span>'+def.name+'</span><span class="active-dot" style="display:none;font-size:11px;color:var(--acc)"> ●</span>';
    tab.onclick=()=>setTab(et); bar.appendChild(tab);
    const sec=document.createElement("div"); sec.style.cssText="background:#0e1017;border:1px solid #232a3a;border-radius:8px;padding:12px 14px;display:none";
    const rec={sec,tab,activeDot:tab.querySelector(".active-dot"),back:[],qam:null,abSwap:null,pad:null,led:null,ledV:null,rumble:null,padStick:[]};
    const tog=document.createElement("div"); tog.className="row";
    tog.innerHTML='<label>A/B + X/Y swap</label>';
    const ab=document.createElement("button"); ab.textContent="off"; tog.appendChild(ab);
    const padLbl=document.createElement("label"); padLbl.textContent="Trackpad haptics"; padLbl.style.marginLeft="14px"; tog.appendChild(padLbl);
    const pad=document.createElement("button"); pad.textContent="on"; tog.appendChild(pad);
    const rumbleLbl=document.createElement("label"); rumbleLbl.textContent="Rumble"; rumbleLbl.style.marginLeft="14px"; tog.appendChild(rumbleLbl);
    const rumble=document.createElement("button"); rumble.textContent="on"; tog.appendChild(rumble);
    sec.appendChild(tog);
    rec.abSwap=ab; rec.pad=pad; rec.rumble=rumble;
    ab.onclick=()=>{ const on=ab.classList.contains("active"); setField(40+et*9+5, on?0:1); };
    pad.onclick=()=>{ const on=pad.classList.contains("active"); setField(40+et*9+6, on?0:1); };
    rumble.onclick=()=>{ const on=rumble.classList.contains("active"); setField(40+et*9+8, on?0:1); };
    for(let i=0;i<4;i++){
      const row=document.createElement("div"); row.className="row";
      const lab=document.createElement("label"); lab.textContent=BACK_LABELS[i]; row.appendChild(lab);
      const sel=mkSelect(def,true); row.appendChild(sel); sec.appendChild(row);
      sel.addEventListener("change",()=>setField(40+et*9+i, +sel.value));
      rec.back.push(sel);
    }
    { const row=document.createElement("div"); row.className="row";
      const lab=document.createElement("label"); lab.textContent="QAM (3 dots)"; row.appendChild(lab);
      const sel=mkSelect(def,false); row.appendChild(sel); sec.appendChild(row);
      sel.addEventListener("change",()=>setField(40+et*9+4, +sel.value));
      rec.qam=sel; }
    { const row=document.createElement("div"); row.className="row";
      const lab=document.createElement("label"); lab.textContent="LED brightness"; row.appendChild(lab);
      const sl=document.createElement("input"); sl.type="range"; sl.min=0; sl.max=100; sl.step=5; row.appendChild(sl);
      const vspan=document.createElement("span"); vspan.className="val"; row.appendChild(vspan);
      function fmtLed(v){ return (+v===0)?"Auto":v+"%"; }
      sl.addEventListener("input",()=>{ vspan.textContent=fmtLed(sl.value); });
      sl.addEventListener("change",()=>setField(40+et*9+7, +sl.value));
      sec.appendChild(row);
      rec.led=sl; rec.ledV=vspan; }
    for(let pad=0; pad<2; pad++){
      const row=document.createElement("div"); row.className="row";
      const lab=document.createElement("label"); lab.textContent=PAD_STICK_LABELS[pad]; row.appendChild(lab);
      const sel=document.createElement("select");
      for(const [v,lbl] of PAD_STICK_OPTS){ const o=document.createElement("option"); o.value=v; o.textContent=lbl; sel.appendChild(o); }
      row.appendChild(sel); sec.appendChild(row);
      sel.addEventListener("change",()=>setField(PAD_STICK_FIELD0+et*2+pad, +sel.value));
      rec.padStick.push(sel);
    }
    host.appendChild(sec); typeEls.push(rec);
  });
  setTab(0);
}
