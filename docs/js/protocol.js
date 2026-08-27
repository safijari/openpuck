import { $, log, fmtDur } from './util.js';
import { trailAdd } from './trail.js';
import { hangLog, pendingHang, setPendingHang, clearPendingHang, renderHangLog, hangLogPush } from './hang.js';
import { onWedge } from './wedge.js';
import { getLizardLoaded, setLizardLoaded } from './lizard.js';

export let dev=null, epIn=0, epOut=0, ifNum=0, polling=false, capturing=false, backupBusy=false;
export let isDongle=false;
export let lastP=null;
export function setLastP(v){ lastP=v; }
let stabArmed=false, stabStart=0, stabLastRun=null;
export let inflight=false;
export let flightBusy=false;
export function setFlightBusy(v){ flightBusy=v; }
export function setCapturing(v){ capturing=v; }
export function setBackupBusy(v){ backupBusy=v; }
export function getStabArmed(){ return stabArmed; }
export function setStabArmed(v){ stabArmed=v; }
export function getStabStart(){ return stabStart; }
export function setStabStart(v){ stabStart=v; }
export function getStabLastRun(){ return stabLastRun; }
export function setStabLastRun(v){ stabLastRun=v; }

let _handlers = {};
export function setHandlers(h){ _handlers=h; }
let _lizardLoader = null;
export function setLizardLoader(fn){ _lizardLoader=fn; }

const USB_FILTERS=[{vendorId:0x28DE},{vendorId:0x045E},{vendorId:0x057E},{vendorId:0x0F0D},{vendorId:0x054C},{vendorId:0x1209},{vendorId:0x2E8A}];

export function updateStabUI(){
  const btn=$("#stabBtn"), st=$("#stabStatus");
  if(!btn) return;
  btn.textContent = stabArmed ? "Stop stability test" : "Test stability";
  btn.classList.toggle("active", stabArmed);
  let txt="";
  if(stabArmed && dev && stabStart) txt="up "+fmtDur((Date.now()-stabStart)/1000);
  else if(stabArmed && !dev) txt="reconnecting…";
  if(stabLastRun) txt += (txt?"  ·  ":"")+"last run: "+fmtDur(stabLastRun.secs);
  st.textContent = txt||"—";
}

export async function send(bytes){
  if(!dev) return;
  try{ await dev.transferOut(epOut, new Uint8Array(bytes)); }
  catch(e){ log("write err: "+e.message); }
}

export async function readFrame(marker, minLen, readLen){
  try{
    const r = await dev.transferIn(epIn, readLen||256);
    if(r.status!=="ok"||r.data.byteLength<2) return null;
    const d=new Uint8Array(r.data.buffer);
    for(let w=0; w+4<d.length; w++){
      if(d[w]===0xA9 && d[w+1]===3){ onWedge(d[w+2], d[w+3]|(d[w+4]<<8)); break; }
    }
    let i=0; while(i<d.length && d[i]!==marker) i++;
    if(i+2>d.length) return null;
    const len=d[i+1];
    if(i+2+len>d.length) return null;
    const p=d.slice(i+2, i+2+len);
    return p.length>=(minLen||0) ? p : null;
  }catch(e){ if(dev) log("read err: "+e.message); return null; }
}

export async function readBlob(){ return readFrame(0xA5, 12); }

export async function setField(field,value){
  await send([0x02, field, value&0xff]); const p=await readBlob(); if(p && _handlers.applyBlob) _handlers.applyBlob(p);
}

export async function waitIdle(){ for(let i=0;i<60 && inflight;i++) await new Promise(r=>setTimeout(r,5)); }
export function getInflight(){ return inflight; }

async function refresh(){
  if(!dev||inflight||capturing||backupBusy||flightBusy) return;
  if(_handlers.isLizardBusy && _handlers.isLizardBusy()) return;
  inflight=true;
  try{
    if(isDongle){ await send([0x01]); const p=await readFrame(0xAC, 3, 256); if(p && _handlers.applyDongleStatus) _handlers.applyDongleStatus(p); }
    else { await send([0x01]); const p=await readBlob(); if(p && _handlers.applyBlob) _handlers.applyBlob(p); }
  }
  catch(e){ log("refresh err: "+e.message); }
  finally{ inflight=false; }
}

async function startPolling(){
  polling=true; await refresh();
  while(polling && dev){
    await new Promise(r=>setTimeout(r,600)); await refresh();
    if(window._autoFlight){ window._autoFlight=false;
      if(_handlers.loadFlightTrail) await _handlers.loadFlightTrail();
    }
  }
}

async function openDevice(d){
  try{
    dev=d;
    await dev.open();
    if(dev.configuration===null) await dev.selectConfiguration(1);
    let found=null;
    for(const itf of dev.configuration.interfaces){
      const a=itf.alternate;
      if(a.interfaceClass!==0xFF) continue;
      if(a.interfaceSubClass===0x5D) continue;
      let bin=0,bout=0;
      for(const e of a.endpoints){ if(e.type==="bulk"){ if(e.direction==="in")bin=e.endpointNumber; else bout=e.endpointNumber; } }
      if(bin&&bout){ found={ifNum:itf.interfaceNumber, epIn:bin, epOut:bout}; break; }
    }
    if(!found){ log("no WebUSB bulk vendor interface — reflash firmware with WebUSB enabled, or another app may be holding the device"); dev=null; return false; }
    ifNum=found.ifNum; epIn=found.epIn; epOut=found.epOut;
    await dev.claimInterface(ifNum);
    await dev.controlTransferOut({requestType:"class", recipient:"interface", request:0x22, value:0x01, index:ifNum});
    isDongle = (dev.productId === 0x1302);
    const kind = isDongle ? "ReversePuck" : "puck";
    $("#connState").textContent="connected"; $("#connState").className="pill up";
    $("#connState").title = dev.serialNumber ? (kind+" serial "+dev.serialNumber) : "";
    $("#connectBtn").textContent="Reconnect";
    $("#panel").classList.remove("hide");
    if(_handlers.applyDeviceProfile) _handlers.applyDeviceProfile();
    log(`connected — ${kind} ${dev.serialNumber||"?"} (VID ${dev.vendorId.toString(16)} PID ${dev.productId.toString(16)} iface ${ifNum})`);
    if(stabArmed && !isDongle){ await send([0x0F,1]); stabStart=Date.now(); log("stability test resumed — timing next run"); }
    updateStabUI();
    setLizardLoaded(false);
    startPolling();
    if(!isDongle && _handlers.loadReleases) _handlers.loadReleases(false);
    return true;
  }catch(e){ log("connect failed: "+e.message); dev=null; return false; }
}

export async function connect(){
  try{ const d = await navigator.usb.requestDevice({filters:USB_FILTERS}); await openDevice(d); }
  catch(e){ log("connect failed: "+e.message); }
}

export async function autoConnect(){
  if(dev) return true;
  try{
    const ds = await navigator.usb.getDevices();
    const d = ds.find(x=>USB_FILTERS.some(f=>f.vendorId===x.vendorId));
    if(d) return await openDevice(d);
  }catch(e){}
  return false;
}

export function onGone(){
  let up=null;
  if(stabArmed && stabStart){ up=(Date.now()-stabStart)/1000; stabLastRun={secs:up}; log("⏱ stability: stayed up "+fmtDur(up)+", then reset"); stabStart=0; }
  if(pendingHang){
    hangLogPush({ time:new Date(pendingHang.t).toLocaleTimeString(), uptime:pendingHang.uptimeSecs,
      reason:"unclassified", stage:"", pc:"", lr:"", usbd:null });
    renderHangLog();
    trailAdd("note: the previous reset was never classified — no full status blob arrived before this disconnect");
  }
  setPendingHang({ t: Date.now(), uptimeSecs: up });
  trailAdd("device disconnected"+(up!=null?(" after "+fmtDur(up)+" up"):"")+" — reset or replug; reason logged on reconnect");
  window._flightAuto=false;
  window._wedgeEp=false; window._wedgePeak=0;
  window._lastBlobTs=0; window._hbLostEp=false;
  polling=false; dev=null;
  if(_handlers.resetTabInited) _handlers.resetTabInited();
  if(_handlers.checkUpdateNotice) _handlers.checkUpdateNotice();
  $("#connState").textContent="reconnecting…"; $("#connState").className="pill dn";
  $("#panel").classList.add("hide");
  log("device disconnected (mode-switch / watchdog reboot) — auto-reconnecting when it returns");
  let tries=0;
  const iv=setInterval(async()=>{ if(dev||tries++>40){ clearInterval(iv); return; } if(await autoConnect()) clearInterval(iv); }, 500);
}
