import { $, log } from './util.js';
import { trailAdd } from './trail.js';

const WEDGE_STAGE=["webusb","ctrl.task","serial","rfdiag","rflink","haptic","led","usbmount","usbtx"];
export function onWedge(stage, stallMs){
  const name=WEDGE_STAGE[stage]||("stage "+stage);
  if(!window._wedgeEp) trailAdd("WEDGED @ "+name+" ("+stallMs+"ms — live 0xA9 report; watchdog reset imminent)");
  if(!window._wedgeEp || stallMs > (window._wedgePeak||0)+500){
    window._wedgeEp=true; window._wedgePeak=stallMs;
    log("🛑 LOOP WEDGED @ "+name+" (stuck "+stallMs+"ms) — this is where it hangs; watchdog reset imminent");
    const el=$("#flightWedge"); if(el) el.innerHTML=' &nbsp;<span class="pill dn">last wedge: '+name+' ('+stallMs+'ms)</span>';
  }
}

export function initHeartbeatWatchdog(getState){
  setInterval(()=>{
    const {dev, capturing, backupBusy, flightBusy} = getState();
    if(!dev || !window._lastBlobTs || capturing || backupBusy || flightBusy) return;
    const age=Date.now()-window._lastBlobTs;
    if(age>2500){
      const secs=Math.round(age/1000);
      $("#stLoopState").innerHTML='<span class="pill dn">NO HEARTBEAT '+secs+'s — hard wedge</span>';
      if(!window._hbLostEp){ window._hbLostEp=true;
        log("🛑 heartbeat lost — no status blob for "+secs+"s: hard wedge (USB stack dead too, so the live wedge reporter can't run) — check the flight trail after the reset");
        trailAdd("HEARTBEAT LOST — no status for "+secs+"s (hard wedge: USB silent, whole MCU likely stopped)");
        const el=$("#flightWedge"); if(el) el.innerHTML=' &nbsp;<span class="pill dn">last wedge: hard (no heartbeat)</span>';
      }
    } else if(window._hbLostEp){ window._hbLostEp=false;
      trailAdd("heartbeat back — status blobs resumed without a reset");
    }
  }, 1000);
}
