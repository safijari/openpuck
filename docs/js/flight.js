import { $, log } from './util.js';
import { dev, epIn, send, setFlightBusy } from './protocol.js';

let flightHdr=null, flightEvents=[];
const FR_EVT=["none","beat","SET","GET","relay","rf-up","rf-DN","HEAL!","mount","SUSPEND","resume","OFF","RINGF","save"];
function frEvtName(e){ return FR_EVT[e]!==undefined?FR_EVT[e]:("evt"+e); }

export async function loadFlightTrail(){
  if(!dev) return;
  setFlightBusy(true);
  $("#flightSummary").textContent="loading…";
  flightHdr=null; flightEvents=[];
  const STAGE_NAMES=["webusb","ctrl.task","serial","rfdiag","rflink","haptic","led","usbmount","usbtx"];
  try{
    let restart=true, done=false, guard=0, acc=new Uint8Array(0);
    while(!done && guard++<600){
      await send([0x10, restart?1:0]); restart=false;
      const r=await dev.transferIn(epIn,192);
      if(r.status!=="ok") break;
      const d=new Uint8Array(r.data.buffer);
      const m=new Uint8Array(acc.length+d.length); m.set(acc); m.set(d,acc.length); acc=m;
      let i=0;
      while(i<acc.length){
        if(acc[i]!==0xA8){ i++; continue; }
        if(i+2>acc.length) break;
        const L=acc[i+1]; if(i+2+L>acc.length) break;
        const f=acc.slice(i+2,i+2+L); i+=2+L;
        const T=f[0];
        if(T===0){ done=true; break; }
        else if(T===2 && L>=27){
          flightHdr={ count:f[1]|(f[2]<<8), total:f[3]|(f[4]<<8),
            loopPerSec:f[5]|(f[6]<<8), stallMs:f[7], stage:f[8],
            usbdStk:f[9]|(f[10]<<8), loopStk:f[11]|(f[12]<<8),
            heap:((f[13]|(f[14]<<8)|(f[15]<<16)|(f[16]<<24))>>>0),
            pollsps:f[17]|(f[18]<<8), relayps:f[19]|(f[20]<<8),
            crc:f[21], norx:f[22], heal:f[23]|(f[24]<<8), ringF:f[25]|(f[26]<<8) };
        } else if(T===1 && L>=9){
          const dt=((f[1]|(f[2]<<8)|(f[3]<<16)|(f[4]<<24))>>>0);
          flightEvents.push({dt, evt:f[5], stage:f[6], arg:f[7]|(f[8]<<8)});
        }
      }
      acc=acc.slice(i);
    }
  }catch(e){ log("flight err: "+e.message); }
  setFlightBusy(false);
  renderFlight(STAGE_NAMES);
}
function renderFlight(STAGE_NAMES){
  const h=flightHdr, out=$("#flightOut"), sum=$("#flightSummary"), wedge=$("#flightWedge");
  const stage=s=>(STAGE_NAMES[s]||("st"+s));
  if(!h || (h.count===0 && flightEvents.length===0)){
    sum.textContent="no trail — last boot wasn't a hang, or the recorder didn't survive this board's reset";
    wedge.textContent=""; out.textContent="—"; return;
  }
  const hx=v=>"0x"+v.toString(16).padStart(4,"0");
  sum.textContent="showing "+flightEvents.length+" of "+h.total+" events before the last hang";
  const usbdFlag=(h.usbdStk>0 && h.usbdStk<16)?" ⚠LOW":"";
  wedge.innerHTML=" &nbsp;<b>@wedge:</b> stuck in "+stage(h.stage)+", stall "+h.stallMs+"ms, loop "+h.loopPerSec+"/s"
    +" · usbdStk "+h.usbdStk+"w"+usbdFlag+" · loopStk "+h.loopStk+"w · heap "+h.heap+"B"
    +" · poll "+h.pollsps+" relay "+h.relayps+" crc "+h.crc+" norx "+h.norx+" heal "+h.heal+" ringF "+h.ringF;
  const pad=(s,n)=>(""+s).padEnd(n);
  out.textContent =
    pad("Δms",8)+pad("event",9)+pad("stage",10)+"arg\n"
    +"".padEnd(30,"─")+"\n"
    + flightEvents.map(e=>pad("-"+e.dt,8)+pad(frEvtName(e.evt),9)+pad(stage(e.stage),10)+hx(e.arg)).join("\n");
}
