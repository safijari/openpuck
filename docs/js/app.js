import { $, log, DEBUG_UI, BETA_UI, betaPopup, fmtSlider } from './util.js';
import { renderTrail, trailClear, trailAdd } from './trail.js';
import { hangLogClear, downloadHangCsv } from './hang.js';
import { initHeartbeatWatchdog } from './wedge.js';
import { buildTypeCfgs, resetTabInited } from './types.js';
import {
  dev, isDongle, send, connect, autoConnect, onGone, setField, updateStabUI,
  setHandlers, setLizardLoader, setStabArmed, getStabArmed, setStabStart, setStabLastRun, getStabLastRun,
  backupBusy, flightBusy, capturing, epIn, getInflight
} from './protocol.js';
import { applyBlob, setStatusHandlers, MODE_NAMES, CHORD_FIELD, CHORD_DPAD_FIELD, RUMBLE_STYLES, RUMBLE_SCALES } from './status.js';
import { applyDongleStatus, applyDeviceProfile } from './dongle.js';
import { loadLizard, saveLizard, resetLizard, renderLizard, lizardBindings, lizardBusy, LZ_MAX, LZO } from './lizard.js';
import { startCapture, stopCapture, downloadCap } from './capture.js';
import { exportBackup, importBackup } from './backup.js';
import { loadFlightTrail } from './flight.js';
import {
  loadReleases, checkUpdateNotice, updateFwGate, updateUf2UI, updateVersionUI,
  pickUf2, getSelectedUf2, runUpdate
} from './firmware.js';

// Wire cross-module callbacks (breaks circular deps)
setHandlers({
  applyBlob,
  applyDongleStatus: (p) => applyDongleStatus(p, isDongle),
  loadReleases,
  applyDeviceProfile: () => applyDeviceProfile(isDongle),
  checkUpdateNotice,
  resetTabInited,
  loadFlightTrail,
  isLizardBusy: () => lizardBusy,
});
setLizardLoader(() => loadLizard(dev, epIn, send, () => getInflight()));
setStatusHandlers({
  loadLizard: () => loadLizard(dev, epIn, send, () => getInflight()),
  updateVersionUI,
  updateFwGate,
  checkUpdateNotice,
  getStabLastRun,
});

// Init debug/beta UI
if(!DEBUG_UI) for(const el of document.querySelectorAll(".debugonly")) el.style.display="none";
$("#betaBtn").textContent = BETA_UI ? "Beta on" : "Beta";
$("#betaBtn").classList.toggle("active", BETA_UI);
$("#betaBtn").title = BETA_UI
  ? "Show beta-disable confirmation and return to the standard UI"
  : "Show beta warning and enable beta UI";
$('#mainTabs .slot-tab[data-tab="tabUpdate"]').classList.toggle("beta-gated", !BETA_UI);

// Beta toggle
$("#betaBtn").onclick=async()=>{
  const u=new URL(location.href);
  if(BETA_UI){
    const ok=await betaPopup({
      title:"Disable beta mode?",
      body:["Beta-only tabs and controls will be gated again after the page reloads.",
        "Let any firmware or config operation already in progress finish before leaving beta mode."],
      okText:"Disable beta"
    });
    if(!ok) return;
    u.searchParams.delete("beta");
    location.href=u.toString();
    return;
  }
  const ok=await betaPopup({
    title:"Enable beta mode?",
    body:["Beta features are still being tested.",
      "They may not work correctly, can disconnect the panel or puck, and may write invalid or difficult-to-recover configs.",
      "Export a backup first if the puck is already paired and configured."],
    okText:"Enable beta"
  });
  if(!ok) return;
  u.searchParams.set("beta","true");
  location.href=u.toString();
};

// Build per-type config UI
buildTypeCfgs(setField);

// Populate chord selects
for(const sel of document.querySelectorAll("select.chord")){
  MODE_NAMES.forEach((n,i)=>{ if(i===7||i===8) return;
    const o=document.createElement("option"); o.value=i; o.textContent=n; sel.appendChild(o); });
}
for(const sel of document.querySelectorAll("select.chordD")){
  MODE_NAMES.forEach((n,i)=>{
    const o=document.createElement("option"); o.value=i; o.textContent=n; sel.appendChild(o); });
}
{ const sel=document.getElementById("swGyroMap");
  for(const [lbl,v] of [["Corrected (default)",0],["Legacy (untrimmed)",1]]){
    const o=document.createElement("option"); o.value=v; o.textContent=lbl; sel.appendChild(o); } }
{ const sel=document.getElementById("rumbleStyle");
  for(const [lbl,v] of RUMBLE_STYLES){
    const o=document.createElement("option"); o.value=v; o.textContent=lbl; sel.appendChild(o); } }
{ const sel=document.getElementById("rumbleScale");
  for(const v of RUMBLE_SCALES){
    const o=document.createElement("option"); o.value=v; o.textContent=v+"%"+(v===200?" (default)":""); sel.appendChild(o); } }

// Trail init
renderTrail();

// Stability test timer
setInterval(updateStabUI, 500);

// Heartbeat watchdog
initHeartbeatWatchdog(() => ({ dev, capturing, backupBusy, flightBusy }));

// USB events
navigator.usb && navigator.usb.addEventListener("connect", () => { if(!dev) autoConnect(); });
navigator.usb && navigator.usb.addEventListener("disconnect", e => { if(e.device===dev) onGone(); });

// ---- Event handlers ----
$("#connectBtn").onclick=connect;
$("#trailClear").onclick=()=>{ trailClear(); log("loop-state trail cleared"); };
$("#backupExport").onclick=exportBackup;
$("#backupImport").onclick=()=>$("#backupFile").click();
$("#backupFile").onchange=async(e)=>{ const f=e.target.files[0]; if(f) await importBackup(f, setField, CHORD_DPAD_FIELD); e.target.value=""; };
$("#capDl").onclick=downloadCap;
$("#capStart").onclick=startCapture;
$("#capStop").onclick=stopCapture;
$("#lzAdd").onclick=()=>{ if(lizardBindings.length<LZ_MAX){ lizardBindings.push({outType:LZO.KBD,od:[0,0,0,0,0,0,0],trig:0,hold:0}); renderLizard(); } };
$("#lzSave").onclick=()=>saveLizard(dev, epIn, send, ()=>getInflight());
$("#lzReload").onclick=()=>loadLizard(dev, epIn, send, ()=>getInflight());
$("#lzReset").onclick=()=>resetLizard(dev, epIn, send, ()=>getInflight());
$("#stabBtn").onclick=async()=>{
  if(!dev){ log("connect first"); return; }
  const armed=!getStabArmed();
  setStabArmed(armed);
  if(armed){ await send([0x0F,1]); setStabStart(Date.now()); setStabLastRun(null); log("stability test STARTED — buzzing every 10s, timing uptime until reset"); trailAdd("stability test started (buzz every 10s)"); }
  else { await send([0x0F,0]); setStabStart(0); log("stability test stopped"); trailAdd("stability test stopped"); }
  updateStabUI();
};
$("#hangClear").onclick=hangLogClear;
$("#flightLoad").onclick=loadFlightTrail;
$("#hangCsv").onclick=downloadHangCsv;
$("#hapClear").onclick=async()=>{ if(dev){ await send([0x07]); log("haptic re-init sent (clear stuck buzz)"); } };
$("#ctlrOff").onclick=async()=>{ if(dev){ await send([0x08]); log("controller power-off attempt sent — watch link status");
  trailAdd("power-off sent to controller (panel button) — stress action, correlate with what follows"); } };
$("#debugCdc").onclick=async()=>{
  if(!dev){ log("not connected"); return; }
  if(!confirm("Reboot with the CDC serial console enabled?\n\nThe puck reboots and comes back with a serial port (115200 baud) instead of WebUSB — this panel will disconnect and NOT reconnect until the next reboot.\n\nConnect a serial monitor to capture logs, then replug (or reboot) to return to normal WebUSB mode. The debug console auto-reverts after one boot.")) return;
  await send([0x02,20,1]); log("debug-CDC reboot sent — device disconnecting; reconnect a serial monitor at 115200 baud");
};
$("#dfuSerial").onclick=async()=>{
  if(!dev){ log("not connected"); return; }
  if(!confirm("Reboot into serial DFU?\n\nThe puck will disconnect immediately. Flash with adafruit-nrfutil, then replug.")) return;
  await send([0x0B]); log("serial DFU reboot sent — device disconnecting");
};
$("#dfuUf2").onclick=async()=>{
  if(!dev){ log("not connected"); return; }
  if(!confirm("Reboot into UF2 bootloader?\n\nThe puck will disconnect and mount as a USB drive. Drag the .uf2 file onto it to flash.")) return;
  await send([0x0C]); log("UF2 bootloader reboot sent — device disconnecting");
};
$("#factoryErase").onclick=async()=>{
  if(!dev){ log("not connected"); return; }
  if(!confirm("Factory erase?\n\nThis wipes ALL persistent storage on the copycat:\n  • the paired-controller bond (you'll have to re-pair)\n  • every saved setting (mode, chords, back paddles, sensitivity)\n\nThe copycat reboots to factory defaults. This CANNOT be undone.")) return;
  if(!confirm("Are you absolutely sure?\n\nThere is no recovery. The controller bond and all settings will be gone.")) return;
  const typed=prompt('Final confirmation — type  ERASE  (all caps) to wipe everything:');
  if(typed!=="ERASE"){ log("factory erase cancelled (confirmation text did not match)"); return; }
  await send([0x0A,0x45,0x52,0x53]);
  log("FACTORY ERASE sent — copycat is reformatting and rebooting to defaults. Re-pair the controller, then reconnect.");
};
$("#wipeBoard").onclick=async()=>{
  if(!dev){ log("not connected"); return; }
  if(!confirm("WIPE THE ENTIRE BOARD?\n\nThis is NOT a factory reset. It erases:\n  • the OpenPuck FIRMWARE itself\n  • every setting (mode, chords, back paddles, sensitivity)\n  • the paired-controller bond\n\nThe board reboots with NO firmware and mounts as the UF2 bootloader drive. It will do so on EVERY boot until you flash OpenPuck (a .uf2) back onto it. This panel will disconnect and NOT reconnect until then.\n\nThis CANNOT be undone from software.")) return;
  if(!confirm("Are you absolutely sure?\n\nThe board will be blank. The ONLY way back is to drag a .uf2 firmware file onto the UF2 drive it mounts as.")) return;
  const typed=prompt('Final confirmation — type  WIPE  (all caps) to erase the whole board:');
  if(typed!=="WIPE"){ log("board wipe cancelled (confirmation text did not match)"); return; }
  await send([0x25,0x57,0x49,0x50,0x45]);
  log("FULL BOARD WIPE sent — the board is erasing firmware + all data (~15–20 s), then reboots as a blank UF2 drive. Flash OpenPuck (.uf2) to restore it.");
};
$("#persistMode").onclick=()=>{ const on=$("#persistMode").classList.contains("active"); setField(16, on?0:1); };
for(const id of ["mDiv","mFric"]){
  const field={mDiv:1,mFric:2}[id];
  $("#"+id).addEventListener("input", ()=>{ $("#"+id+"V").textContent=fmtSlider(id,$("#"+id).value); });
  $("#"+id).addEventListener("change", ()=>setField(field, +$("#"+id).value));
}
$("#rumbleTest").onclick=async()=>{ if(dev){ await send([0x16]); log("test rumble sent (style "+$("#rumbleStyle").value+", "+$("#rumbleScale").value+"%)"); } };
$("#rumbleStyle").addEventListener("change", ()=>setField(39, +$("#rumbleStyle").value));
$("#rumbleScale").addEventListener("change", ()=>setField(22, (+$("#rumbleScale").value)/2));
$("#swGyroMap").addEventListener("change", ()=>setField(38, +$("#swGyroMap").value));
for(const sel of document.querySelectorAll("select.chord")){
  sel.addEventListener("change", ()=>setField(CHORD_FIELD[+sel.dataset.i], +sel.value));
}
for(const sel of document.querySelectorAll("select.chordD")){
  sel.addEventListener("change", ()=>setField(CHORD_DPAD_FIELD[+sel.dataset.i], +sel.value));
}
for(const b of document.querySelectorAll(".modebtn")){
  b.onclick=()=>{ const m=+b.dataset.mode;
    const clean=(m===7||m===8||m===9);
    const msg="Switch to "+MODE_NAMES[m]+"? The copycat will reboot."+(clean?"\n\nNOTE: the game/clean PlayStation modes (and PS3) drop WebUSB + host-wake, so THIS PANEL WILL DISCONNECT and can't reach the device while it's in this mode. To get back, chord on the controller: hold all four back paddles (L4+R4+L5+R5) + A to return to Steam mode. The back4+D-pad chords are how you get back INTO these modes without the panel — check their assignments in the chords card first.":"");
    if(confirm(msg)){ send([0x03, m]); log("mode switch requested — device will reboot"); } };
}

// ---- top-level tabs ----
for(const t of document.querySelectorAll("#mainTabs .slot-tab")){
  t.onclick=async()=>{
    if(t.dataset.tab==="tabUpdate" && !BETA_UI){
      const ok=await betaPopup({
        title:"Firmware update is beta",
        body:["Firmware update is still being tested and may not work correctly.",
          "A bad image or interrupted recovery path can leave you needing UF2 DFU to recover.",
          "Enable beta mode before using this feature."],
        okText:"Enable beta"
      });
      if(ok){
        const u=new URL(location.href);
        u.searchParams.set("beta","true");
        location.href=u.toString();
      }
      return;
    }
    for(const x of document.querySelectorAll("#mainTabs .slot-tab")) x.classList.toggle("active",x===t);
    $("#tabMain").classList.toggle("hide", t.dataset.tab!=="tabMain");
    $("#tabUpdate").classList.toggle("hide", t.dataset.tab!=="tabUpdate");
    if(t.dataset.tab==="tabUpdate") loadReleases(false);
  };
}

// ---- UF2 file picker / drag-and-drop ----
$("#uf2File").onchange=async(e)=>{
  const f=e.target.files && e.target.files[0];
  e.target.value="";
  if(f) await pickUf2(f);
};
window.addEventListener("dragover",e=>e.preventDefault());
window.addEventListener("drop",e=>e.preventDefault());
{
  const dz=$("#uf2Drop");
  dz.onclick=()=>$("#uf2File").click();
  dz.ondragover=e=>{ e.preventDefault(); dz.classList.add("drag"); };
  dz.ondragleave=()=>dz.classList.remove("drag");
  dz.ondrop=async e=>{
    e.preventDefault(); dz.classList.remove("drag");
    const f=e.dataTransfer.files && e.dataTransfer.files[0];
    if(f) await pickUf2(f);
  };
}
$("#uf2Flash").onclick=()=>{
  const sel=getSelectedUf2();
  if(!sel) return;
  runUpdate("Flashing "+sel.name,
    "Flash "+sel.name+" ("+Math.round(sel.image.length/1024)+" KiB)?\n\n"
    +"The image is sent over WebUSB into spare flash (~15 s), verified on the puck, and applied on an automatic "
    +"reboot (~5 s dark). Nothing is armed until it verifies, so a failed or interrupted transfer leaves the "
    +"current firmware untouched. Worst case (power cut during the apply itself) the puck comes back as the "
    +"UF2BOOT drive for drag-and-drop recovery — it cannot end up half-flashed.",
    async()=>sel.image);
};
$("#relRefresh").onclick=()=>loadReleases(true);
$("#updAvail").onclick=()=>{ document.querySelector('#mainTabs .slot-tab[data-tab="tabUpdate"]').click(); };
$("#updClose").onclick=()=>$("#updModal").classList.add("hide");

// ---- init ----
updateUf2UI();
updateFwGate();
if(!("usb" in navigator)) $("#connectBtn").outerHTML='<b style="color:var(--bad)">WebUSB not supported — use Chrome or Edge.</b>';
else autoConnect();
