import { $, log } from './util.js';
import { dev, epIn, send, setCapturing } from './protocol.js';

function fmtCapEntry(e,maxMs){
  const hex=e.bytes.map(x=>x.toString(16).padStart(2,"0")).join(" ");
  const t = `-${String(maxMs-e.ms).padStart(6," ")}ms  `;
  if(e.slot===0xFD){ const k=e.bytes[0]; return `${t}════════ ${k===2?"RECONNECT (block+reinit armed)":k===1?"RF LINK UP":"RF LINK DOWN"} ════════`; }
  if(e.slot===0xFC){ return `${t}GET←host rid=${e.rid.toString(16).padStart(2,"0")}  n=${e.nb}:  ${hex}`; }
  if(e.slot===0xFB){ return `${t}→host    rid=${e.rid.toString(16).padStart(2,"0")}  n=${e.nb}:  ${hex}`; }
  const who = (e.slot===0xFE) ? "TX→ctlr " : `if${e.slot}     `;
  return `${t}${who} cmd=${e.rid.toString(16).padStart(2,"0")}  n=${e.nb}:  ${hex}`;
}
export function downloadCap(){
  const t=window._lastCap||$("#capOut").textContent||"";
  const a=document.createElement("a");
  a.href=URL.createObjectURL(new Blob([t],{type:"text/plain"}));
  a.download="puck-capture.txt"; a.click();
}

let liveTimer=null, draining=false, capLines=[];
function renderCap(){
  if(!capLines.length){ $("#capOut").textContent="(nothing captured yet)"; window._lastCap=""; return; }
  const maxMs=capLines[capLines.length-1].ms;
  const t=capLines.map(e=>fmtCapEntry(e,maxMs)).join("\n");
  $("#capOut").textContent=t; window._lastCap=t;
}
async function drainTick(){
  if(!dev||draining) return; draining=true;
  let acc=new Uint8Array(0);
  try{
    await send([0x06]);
    let done=false, guard=0;
    while(!done && guard++<400){
      const r=await dev.transferIn(epIn,64);
      if(r.status!=="ok") break;
      const d=new Uint8Array(r.data.buffer);
      const m=new Uint8Array(acc.length+d.length); m.set(acc); m.set(d,acc.length); acc=m;
      let i=0;
      while(i<acc.length){
        if(acc[i]!==0xA6){ i++; continue; }
        if(i+2>acc.length) break;
        const L=acc[i+1]; if(i+2+L>acc.length) break;
        const f=acc.slice(i+2,i+2+L); i+=2+L;
        if(f[0]===0){ done=true; break; }
        if(f[0]===1 && L>=8){
          const ms=((f[1])|(f[2]<<8)|(f[3]<<16)|(f[4]<<24))>>>0;
          capLines.push({ms, slot:f[5], rid:f[6], nb:f[7], bytes:[...f.slice(8,8+f[7])]});
        }
      }
      acc=acc.slice(i);
    }
  }catch(e){ log("drain err: "+e.message); }
  draining=false;
  if(liveTimer){ const el=$("#capOut"); const atBottom=el.scrollTop+el.clientHeight>=el.scrollHeight-4; renderCap(); if(atBottom) el.scrollTop=el.scrollHeight; }
}

export async function startCapture(){
  if(!dev) return;
  setCapturing(true); capLines=[];
  $("#capOut").textContent="(dumping ring from boot, then live — press Stop when done)";
  $("#capStart").disabled=true; $("#capStop").disabled=false;
  await send([0x05,1]);
  liveTimer=setInterval(drainTick,100);
}
export async function stopCapture(){
  if(liveTimer){ clearInterval(liveTimer); liveTimer=null; }
  await drainTick();
  try{ await send([0x05,0]); }catch(e){}
  setCapturing(false);
  $("#capStart").disabled=false; $("#capStop").disabled=true;
  renderCap(); log(`capture stopped — ${capLines.length} entries`);
}
