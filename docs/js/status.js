import { $, log, fmtDur, setSlider } from './util.js';
import { trailAdd } from './trail.js';
import { pendingHang, clearPendingHang, renderHangLog, hangLogPush } from './hang.js';
import { g_activeSlot, g_slotData, setActiveSlot, setSlotData, renderSlotTabs } from './slots.js';
import { TYPE_DEFS, etypeForMode, typeEls, tabInited, markTabInited, setTab } from './types.js';
import { getLizardLoaded, setLizardLoaded } from './lizard.js';
import { setLastP } from './protocol.js';

export const MODE_NAMES = ["Steam (puck)","Xbox 360","Switch (HORIPAD)","Lizard (always)","Switch Pro + gyro","PS5 DualSense","HID gyro (DS4)","PS5 (game/clean)","DS4 (game/clean)","PS3 (DualShock 3)","Original Xbox","DirectInput (sims)","SInput (SDL native)"];
export const CHORD_FIELD = [17, 18, 19];
export const CHORD_DPAD_FIELD = [34, 35, 36, 37];
const CHORD_DPAD_DEF = [9, 8, 7, 2];
export const RUMBLE_STYLES=[["Normal (default)",0],["Mono (both motors)",1],["Heavy (low only)",2],["Light (high only)",3],["Swapped motors",4],["Punchy (squared)",5],["Soft (boost weak)",6]];
export const RUMBLE_SCALES=[50,75,100,150,200,250,300,400,500];

let _statusHandlers = {};
export function setStatusHandlers(h){ _statusHandlers=h; }

export function applyBlob(p){
  setLastP(p);
  window._lastBlobTs=Date.now();
  if(!getLizardLoaded() && p[0]>=16){ setLizardLoaded(true); if(_statusHandlers.loadLizard) _statusHandlers.loadLizard(); }
  const mode=p[1], mDiv=p[2], mFric=p[3];
  const slot=p[10], up=p[11], f1=(p[12]|(p[13]<<8));
  const newps=(p.length>16?(p[15]|(p[16]<<8)):0), persistMode=(p.length>22?p[22]:0);
  const chord=(p.length>25)?[p[23],p[24],p[25]]:[3,1,4];
  const dpadCap=(p[0]>=18 && p.length>183);
  const chordD=dpadCap?[p[180],p[181],p[182],p[183]]:CHORD_DPAD_DEF;
  const polls=(p.length>27?(p[26]|(p[27]<<8)):0);
  const loopUs=(p.length>29?(p[28]|(p[29]<<8)):0);
  const worstIdx=(p.length>30?p[30]:0), worstUs=(p.length>32?(p[31]|(p[32]<<8)):0);
  const WORST=["webusb","ctrl.task","serial","rfdiag","rflink","haptic","led"];
  const pollUs=(p.length>34?(p[33]|(p[34]<<8)):0);
  const pollIntended=(p.length>14?p[14]*100:0);
  const logEnabled=(p.length>35?p[35]:0);
  for(const el of document.querySelectorAll(".logonly")) el.style.display = logEnabled ? "" : "none";
  for(const b of document.querySelectorAll(".modebtn")) b.classList.toggle("active", +b.dataset.mode===mode);
  $("#mouseCard").style.opacity = (mode===1||mode===3)?1:0.5;
  $("#lizardCard").style.display = (mode===3 && p[0]>=16)?"":"none";
  const battery=(p.length>36?p[36]:0);
  const gyroMapCap=(p[0]>=19 && p.length>184);
  const swGyroLegacy=gyroMapCap?p[184]:0;
  const rumbleCap=(p[0]>=21 && p.length>193);
  const rumbleScale=rumbleCap?p[51]*2:200, rumbleStyle=rumbleCap?p[193]:0;
  const rssi=(p.length>37?p[37]:0);

  const hasSlotsData = p.length >= 73;
  if (hasSlotsData) {
    const bondedN = p[60];
    let sd = [];
    for (let s=0; s<4; s++) {
      const base = 61 + s*3;
      if (p.length >= base+3) {
        const slotUp = !!p[base];
        const slotBatt = p[base+1];
        const slotRssi = p[base+2];
        sd[s] = {up: slotUp, battery: slotBatt, rssi: slotRssi};
      }
    }
    let bondedSlots = [];
    for (let s=0; s<4; s++) {
      if (sd[s] && (sd[s].up || sd[s].battery || sd[s].rssi)) bondedSlots.push(s);
    }
    if (bondedSlots.length === 0 && bondedN > 0) {
      for (let s=0; s<bondedN; s++) bondedSlots.push(s);
    }
    const bondedSet = new Set(bondedSlots);
    for (let s=0; s<4; s++) { if (!bondedSet.has(s)) sd[s] = null; }
    setSlotData(sd);
    if (!sd[g_activeSlot]) setActiveSlot(bondedSlots[0] || 0);
    if (p.length >= 179) {
      for (let s=0; s<4; s++) {
        if (!sd[s]) continue;
        const b = 143 + s*9;
        sd[s].stats = { polls: p[b]|(p[b+1]<<8), f1: p[b+2]|(p[b+3]<<8),
                        newps: p[b+4]|(p[b+5]<<8), crc: p[b+6], norx: p[b+7], relay: p[b+8] };
      }
    }
    renderSlotTabs(bondedN);
  } else {
    $("#slotTabs").style.display="none";
    $("#stLink").innerHTML = up? '<span class="pill up">up</span>' : '<span class="pill dn">down</span>';
    $("#stBatt").textContent = (up && battery)? battery+"%" : "—";
    $("#stRssi").textContent = (up && rssi)? ("-"+rssi+" dBm") : "—";
  }

  $("#stRate").textContent = up? f1+" /s" : "—";
  $("#stNew").textContent = up? newps+" /s" : "—";
  $("#stPolls").textContent = polls+" /s";
  { const crc=(p.length>117?p[117]:0), norx=(p.length>118?p[118]:0), heal=(p.length>119?p[119]:0);
    const rf=(p.length>130?(p[129]|(p[130]<<8)):0);
    $("#stRfFail").innerHTML = crc+" · "+norx+" · "+heal + (rf?' · <span class="pill dn">ring '+rf+'</span>':"");
    if(rf>(window._ringFault||0)) log("⚠ RING FAULT #"+rf+" recovered — caught a relay-ring desync/corruption (would have hung loop with IRQs off)");
    window._ringFault=rf; }
  { const relay=(p.length>121?(p[120]|(p[121]<<8)):null);
    $("#stRelay").textContent = relay==null ? "—" : (relay+" /s"); }
  if(p.length>125){
    const LF=["stopped","RC","xtal","synth"], HF=["RC","?","xtal","?"];
    const lf=p[122], hf=p[123], upm=(p[124]|(p[125]<<8));
    const lfBad=(lf!==2), hfBad=(hf!==2), upmBad=(upm && (upm<985||upm>1015));
    const tag=(t,bad)=> bad ? '<span class="pill dn">'+t+'</span>' : t;
    $("#stClock").innerHTML = tag(LF[lf]||lf, lfBad)+" / "+tag(HF[hf]||hf, hfBad);
    $("#stUsPerMs").innerHTML = upm ? tag(""+upm, upmBad) : "—";
  }
  if(p.length>128){
    const STAGE=["webusb","ctrl.task","serial","rfdiag","rflink","haptic","led","usbmount","usbtx"];
    const cur=p[127], stallMs=p[128]*40, name=(STAGE[cur]||("stage "+cur));
    if(stallMs>=200){
      $("#stLoopState").innerHTML='<span class="pill dn">STALLED @ '+name+' '+stallMs+'ms</span>';
      if(!window._stallEpisode) trailAdd("STALLED @ "+name+" ("+stallMs+"ms, soft wedge — usbd still alive)");
      if(!window._stallEpisode || stallMs>window._stallPeak+800){
        window._stallEpisode=true; window._stallPeak=stallMs;
        log("⚠ LOOP STALL @ "+name+" ("+stallMs+"ms) — watchdog reset imminent");
      }
    } else {
      if(window._stallEpisode) trailAdd("running again — stall recovered without a reset (peaked "+window._stallPeak+"ms)");
      window._stallEpisode=false; window._stallPeak=0;
      $("#stLoopState").innerHTML='<span class="pill up">running</span>';
    }
  }
  if(p.length>140){ const usbdW=(p[139]|(p[140]<<8));
    $("#stUsbdStack").innerHTML = usbdW<24 ? ('<span class="pill dn">'+usbdW+'</span>')
      : (usbdW<48 ? ('<span class="pill" style="background:#3a3216;color:#e5c76b">'+usbdW+'</span>') : (""+usbdW));
  }
  $("#stLoop").textContent = loopUs? loopUs+" µs" : "—";
  $("#stWorst").textContent = loopUs? (WORST[worstIdx]||worstIdx)+" "+worstUs+"µs" : "—";
  $("#stPollPer").textContent = up? (pollUs+" / "+pollIntended+" µs") : "—";
  const RR_NAMES = ["unknown","power-on","pin/replug","watchdog (hang)","CPU lockup",
                    "HARDFAULT","reboot","soft reset","wake-from-off"];
  const RR_FAULT = new Set([3,4,5]);
  if (p.length > 113) {
    const code = p[109];
    const raw = ((p[110]|(p[111]<<8)|(p[112]<<16)|(p[113]<<24))>>>0);
    const name = RR_NAMES[code] || ("code "+code);
    const STAGE_NAMES = ["webusb","ctrl.task","serial","rfdiag","rflink","haptic","led","usbmount","usbtx"];
    const hangStage = (p.length>126 ? p[126] : 0xFF);
    let label = name;
    if (hangStage!==0xFF) label += " @ "+(STAGE_NAMES[hangStage]||("stage "+hangStage));
    $("#stReset").innerHTML = RR_FAULT.has(code) ? ('<span class="pill dn">'+label+'</span>') : label;
    const hpc=(p.length>134?((p[131]|(p[132]<<8)|(p[133]<<16)|(p[134]<<24))>>>0):0);
    const hlr=(p.length>138?((p[135]|(p[136]<<8)|(p[137]<<16)|(p[138]<<24))>>>0):0);
    const hx=v=>"0x"+v.toString(16).padStart(8,"0");
    const el = $("#resetHist");
    const stabLastRun = _statusHandlers.getStabLastRun ? _statusHandlers.getStabLastRun() : null;
    if (el) el.textContent = "RESETREAS=0x"+raw.toString(16).padStart(8,"0")
      + (hangStage!==0xFF ? "  hung in: "+(STAGE_NAMES[hangStage]||hangStage) : "")
      + (hpc ? "  hang PC="+hx(hpc)+" LR="+hx(hlr) : "")
      + (stabLastRun ? "  ·  stability: stayed up "+fmtDur(stabLastRun.secs)+" before this reset" : "");
    if (hpc && hpc!==window._hangPC){ window._hangPC=hpc; log("⚑ HANG PC="+hx(hpc)+" LR="+hx(hlr)+" — send me this; I'll map it to the stuck function"); }
    if (pendingHang){
      const usbdW2=(p.length>140?(p[139]|(p[140]<<8)):0);
      hangLogPush({ time:new Date(pendingHang.t).toLocaleTimeString(), uptime:pendingHang.uptimeSecs,
        reason:name, stage:(hangStage!==0xFF)?(STAGE_NAMES[hangStage]||("stage"+hangStage)):"",
        pc:hpc?hx(hpc):"", lr:hlr?hx(hlr):"", usbd:usbdW2 });
      trailAdd("reconnected — last reset: "+name
        +(hangStage!==0xFF?(" @ "+(STAGE_NAMES[hangStage]||("stage "+hangStage))):"")
        +(hpc?(" PC="+hx(hpc)):"")
        +(pendingHang.uptimeSecs!=null?("  (up "+fmtDur(pendingHang.uptimeSecs)+" before it)"):""));
      try{ localStorage.setItem("opk_trail_rrfp", code+"/"+raw.toString(16)+"/"+hangStage+"/"+hpc.toString(16)); }catch(e){}
      clearPendingHang(); renderHangLog();
    }
    else if (RR_FAULT.has(code)){
      const fp=code+"/"+raw.toString(16)+"/"+hangStage+"/"+hpc.toString(16);
      if(localStorage.getItem("opk_trail_rrfp")!==fp){
        try{ localStorage.setItem("opk_trail_rrfp", fp); }catch(e){}
        trailAdd("connected — last boot was a "+label+(hpc?(" PC="+hx(hpc)):"")+" (page wasn't open to catch it live)");
      }
    }
    if (RR_FAULT.has(code) && !window._flightAuto){ window._flightAuto=true; window._autoFlight=true; }
  }

  const padStickCap=(p[0]>=20 && p.length>192);
  const gitDirty=(p.length>38?p[38]:0); let buildId="";
  for(let i=39;i<51 && i<p.length && p[i];i++) buildId+=String.fromCharCode(p[i]);
  $("#stBuild").innerHTML = buildId
    ? (buildId + (gitDirty ? ' <span class="pill dn">dirty</span>' : ' <span class="pill up">clean</span>'))
    : "—";
  $("#updInstalled").textContent = buildId || "—";
  if(_statusHandlers.updateVersionUI) _statusHandlers.updateVersionUI();
  if(_statusHandlers.updateFwGate) _statusHandlers.updateFwGate();
  if(_statusHandlers.checkUpdateNotice) _statusHandlers.checkUpdateNotice();
  setSlider("mDiv",mDiv); setSlider("mFric",mFric);
  $("#persistMode").textContent = persistMode? "on":"off"; $("#persistMode").classList.toggle("active",!!persistMode);
  if(p.length>=73+TYPE_DEFS.length*9){
    const activeEt=etypeForMode(mode);
    typeEls.forEach((rec,et)=>{
      const q=73+et*9;
      const back=[p[q],p[q+1],p[q+2],p[q+3]], qam=p[q+4], ab=p[q+5], pad=p[q+6], led=p[q+7], rum=p[q+8];
      rec.back.forEach((sel,i)=>{ if(document.activeElement!==sel) sel.value=back[i]; });
      if(document.activeElement!==rec.qam) rec.qam.value=qam;
      rec.abSwap.textContent=ab?"on":"off"; rec.abSwap.classList.toggle("active",!!ab);
      rec.pad.textContent=pad?"on":"off"; rec.pad.classList.toggle("active",!!pad);
      rec.rumble.textContent=rum?"on":"off"; rec.rumble.classList.toggle("active",!!rum);
      if(document.activeElement!==rec.led){ rec.led.value=led; rec.ledV.textContent=(led===0)?"Auto":led+"%"; }
      rec.padStick.forEach((sel,pad)=>{
        sel.parentNode.classList.toggle("hide", !padStickCap);
        if(padStickCap && document.activeElement!==sel) sel.value=p[185+et*2+pad];
      });
      rec.activeDot.style.display = (et===activeEt) ? "" : "none";
    });
    if(!tabInited){ markTabInited(); setTab(activeEt>=0?activeEt:0); }
  }
  if(p.length>=60){ const s16=(o)=>{ let v=p[o]|(p[o+1]<<8); return v>32767?v-65536:v; };
    const ax=s16(54),ay=s16(56),az=s16(58);
    const amag=Math.round(Math.sqrt(ax*ax+ay*ay+az*az));
    $("#stImu").textContent = `a=(${ax}, ${ay}, ${az})  |a|=${amag}`;
  }
  { const sel=$("#swGyroMap");
    $("#swGyroMapBlock").classList.toggle("hide", !gyroMapCap);
    $("#swGyroOld").classList.toggle("hide", gyroMapCap);
    if(gyroMapCap && document.activeElement!==sel) sel.value=swGyroLegacy; }
  { const sty=$("#rumbleStyle"), scl=$("#rumbleScale");
    $("#rumbleBlock").classList.toggle("hide", !rumbleCap);
    $("#rumbleOld").classList.toggle("hide", rumbleCap);
    if(rumbleCap){
      if(document.activeElement!==sty) sty.value=rumbleStyle;
      if(document.activeElement!==scl) scl.value=RUMBLE_SCALES.reduce((a,b)=>Math.abs(b-rumbleScale)<Math.abs(a-rumbleScale)?b:a);
    } }
  for(const sel of document.querySelectorAll("select.chord")){ if(document.activeElement!==sel) sel.value=chord[+sel.dataset.i]; }
  $("#dpadChords").classList.toggle("hide", !dpadCap);
  $("#dpadOld").classList.toggle("hide", dpadCap);
  if(dpadCap) for(const sel of document.querySelectorAll("select.chordD")){ if(document.activeElement!==sel) sel.value=chordD[+sel.dataset.i]; }
}
