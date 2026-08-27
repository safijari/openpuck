import { $, log, BETA_UI } from './util.js';
import { dev, epIn, epOut, isDongle, lastP, send, readBlob } from './protocol.js';

let selectedUf2=null, dfuBusy=false;
let relCache=null, relManifest=null;

export function updateUf2UI(){
  const sel=$("#uf2Sel"), flash=$("#uf2Flash");
  if(!sel || !flash) return;
  sel.textContent = selectedUf2
    ? selectedUf2.name+" — app image "+Math.round(selectedUf2.image.length/1024)+" KiB, parsed OK"
      +(selectedUf2.updatable ? "" : " — ⚠ no panel-update support in this image (flashing it = drag-and-drop only afterwards)")
    : "no file selected";
  flash.disabled = dfuBusy || !selectedUf2;
  flash.title = selectedUf2
    ? "Send " + selectedUf2.name + " to the puck over WebUSB, verify it, and apply it on reboot"
    : "Pick a .uf2 file first";
}

export function getSelectedUf2(){ return selectedUf2; }

// ---- blocking modal ----
export function modalStage(txt,pct){
  const f=$("#updFill"), p=$("#updPct");
  $("#updStage").textContent=txt;
  f.classList.remove("bad");
  if(pct==null){ f.classList.add("pulse"); f.style.width="100%"; p.textContent=""; }
  else{ f.classList.remove("pulse"); f.style.width=pct+"%"; p.textContent=pct+"%"; }
}
export function modalOpen(title){
  $("#updTitle").textContent=title;
  $("#updDetail").textContent="";
  $("#updClose").classList.add("hide");
  $("#updModal").classList.remove("hide");
  modalStage("Preparing…",null);
}
export function modalDone(ok,msg,failTitle){
  const f=$("#updFill");
  f.classList.remove("pulse");
  f.style.width="100%";
  f.classList.toggle("bad",!ok);
  $("#updStage").textContent = ok ? "Done" : (failTitle||"Update failed");
  $("#updPct").textContent="";
  $("#updDetail").textContent=msg;
  $("#updClose").classList.remove("hide");
}

export function blobBuildId(p){
  if(!p) return "";
  let s=""; for(let i=39;i<51 && i<p.length && p[i];i++) s+=String.fromCharCode(p[i]);
  return s;
}

function verTriple(s){
  const m=/^(\d+)\.(\d+)\.(\d+)/.exec(s||"");
  return m ? [+m[1],+m[2],+m[3]] : null;
}

export function checkUpdateNotice(){
  const el=$("#updAvail");
  if(!BETA_UI){ el.classList.add("hide"); return; }
  const inst=verTriple(blobBuildId(lastP));
  const rel=(relCache||[]).find(r=>!r.prerelease && relAsset(r,false));
  const latest=rel ? verTriple(rel.tag_name) : null;
  const newer = !!(dev && inst && latest &&
    (latest[0]>inst[0] || (latest[0]===inst[0] &&
     (latest[1]>inst[1] || (latest[1]===inst[1] && latest[2]>inst[2])))));
  el.classList.toggle("hide", !newer);
  if(newer) el.textContent="⬆ update "+rel.tag_name+" available";
}

export function updateFwGate(){
  const ok = isDongle || !!(lastP && lastP[0]>=15);
  $("#updGate").classList.toggle("hide", ok);
  if(!ok) $("#updGateMsg").textContent =
    "This puck is running "+(blobBuildId(lastP)||"an unknown build")+" (status v"+(lastP?lastP[0]:"?")
    +"), which predates panel updates (needs v15+), so updating from this page is disabled. One manual flash "
    +"gets you back: click \"UF2 DFU\" in the top bar, then drag a panel-update-capable .uf2 onto the UF2BOOT "
    +"drive it mounts. Every update after that happens right here.";
  for(const c of document.querySelectorAll(".fwupcard")) c.classList.toggle("gated", !ok);
}

export function updateVersionUI(){
  const build=blobBuildId(lastP), proto=lastP ? lastP[0] : 0;
  const dirty=lastP && lastP.length>38 ? lastP[38] : 0;
  $("#stVersionBuild").innerHTML = build
    ? (build + (dirty ? ' <span class="pill dn">dirty</span>' : ' <span class="pill up">clean</span>'))
    : "—";
  $("#stVersionProto").textContent = proto ? ("v"+proto) : "—";
  $("#stVersionUpdate").textContent = proto >= 15 ? "supported" : (proto ? "manual UF2 only" : "—");
}

// ---- UF2 parsing ----
const UF2_MAGIC0=0x0A324655, UF2_MAGIC1=0x9E5D5157, UF2_MAGIC_END=0x0AB16F30, UF2_FAMILY=0xADA52840;
const APP_BASE=0x26000, FWUP_MAX_IMG=0x60000;
function uf2ToImage(buf){
  if(!buf.byteLength || buf.byteLength%512) throw new Error("not a UF2 (size is not a multiple of 512)");
  const dv=new DataView(buf), blocks=[]; let lo=Infinity, hi=0;
  for(let off=0; off<buf.byteLength; off+=512){
    if(dv.getUint32(off,true)!==UF2_MAGIC0 || dv.getUint32(off+4,true)!==UF2_MAGIC1 || dv.getUint32(off+508,true)!==UF2_MAGIC_END)
      throw new Error("bad UF2 block magic at offset "+off);
    const flags=dv.getUint32(off+8,true), addr=dv.getUint32(off+12,true), len=dv.getUint32(off+16,true);
    if(flags&0x00000001) continue;
    if((flags&0x00002000) && dv.getUint32(off+28,true)!==UF2_FAMILY)
      throw new Error("UF2 family 0x"+dv.getUint32(off+28,true).toString(16)+" is not nRF52840 (0xada52840)");
    if(len<1 || len>476) throw new Error("bad UF2 payload size "+len);
    blocks.push({addr,len,off}); lo=Math.min(lo,addr); hi=Math.max(hi,addr+len);
  }
  if(!blocks.length) throw new Error("UF2 contains no flash data");
  if(lo!==APP_BASE) throw new Error("image base 0x"+lo.toString(16)+" is not the app region (0x26000) — not an OpenPuck app UF2");
  if(hi-lo>FWUP_MAX_IMG) throw new Error("image is "+Math.round((hi-lo)/1024)+" KiB — over the 384 KiB staged-update cap; flash it via UF2 DFU + drag-and-drop");
  const img=new Uint8Array((hi-lo+3)&~3).fill(0xFF);
  for(const b of blocks) img.set(new Uint8Array(buf,b.off+32,b.len), b.addr-lo);
  return img;
}
function crc32(u8){
  let c=0xFFFFFFFF;
  for(let i=0;i<u8.length;i++){
    c^=u8[i];
    for(let k=0;k<8;k++) c=(c>>>1)^(0xEDB88320&-(c&1));
  }
  return (~c)>>>0;
}
const u32le=v=>[v&0xFF,(v>>>8)&0xFF,(v>>>16)&0xFF,(v>>>24)&0xFF];
const dfuSleep=ms=>new Promise(r=>setTimeout(r,ms));
function imagePanelUpdatable(image){
  return new TextDecoder("latin1").decode(image).includes("OPK-FWUP-v1");
}
const FWUP_ERR=["ok","command out of sequence","image too big for this puck's free flash","offset resync",
  "staged bytes failed CRC verify","staged image has no valid vector table"];

let fwupRead=null;
async function fwupAckWait(timeoutMs){
  const deadline=Date.now()+timeoutMs;
  for(;;){
    const remain=deadline-Date.now();
    if(remain<=0) return null;
    if(!fwupRead) fwupRead=dev.transferIn(epIn, 64);
    const r=await Promise.race([fwupRead, dfuSleep(remain).then(()=>"timeout")]);
    if(r==="timeout") return null;
    fwupRead=null;
    if(r.status!=="ok") continue;
    const d=new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength);
    let ack=null;
    for(let i=0;i+6<d.length;i++)
      if(d[i]===0xAB && d[i+1]===5)
        ack={status:d[i+2], off:(d[i+3]|(d[i+4]<<8)|(d[i+5]<<16)|(d[i+6]<<24))>>>0};
    if(ack) return ack;
  }
}
const fwupSend=cmd=>dev.transferOut(epOut, new Uint8Array(cmd));
async function fwupCtl(cmd, timeoutMs, label){
  for(let t=0;t<3;t++){
    await fwupSend(cmd);
    const a=await fwupAckWait(timeoutMs);
    if(a) return a;
  }
  throw new Error("no response to "+label);
}
async function fwupRun(image){
  const kb=Math.round(image.length/1024);
  fwupRead=null;
  while(await fwupAckWait(400)!==null){}
  log("firmware update: staging "+kb+" KiB into the puck's spare flash — input may stutter briefly during page erases");
  let a=await fwupCtl([0x20,...u32le(image.length),...u32le(crc32(image))],4000,"begin");
  if(a.status) throw new Error("begin rejected: "+(FWUP_ERR[a.status]||("code "+a.status)));
  let off=0, sends=0, lastShown=-1;
  const maxSends=Math.ceil(image.length/128)*2+64;
  while(off<image.length){
    const len=Math.min(128,image.length-off);
    await fwupSend([0x21,...u32le(off),len,...image.subarray(off,off+len)]);
    if(++sends>maxSends) throw new Error("transfer not converging at offset "+off);
    for(let reads=0;reads<8;reads++){
      a=await fwupAckWait(2500);
      if(a===null) break;
      if(a.status===3){ off=a.off; break; }
      if(a.status!==0) throw new Error("chunk rejected at offset "+off+": "+(FWUP_ERR[a.status]||("code "+a.status)));
      if(a.off>off){ off=a.off; break; }
    }
    const pct=Math.floor(off*50/image.length)*2;
    if(pct!==lastShown){ lastShown=pct; modalStage("Sending to the puck", pct); }
  }
  modalStage("Verifying on the puck", null);
  a=await fwupCtl([0x22],8000,"verify+commit");
  if(a.status) throw new Error("verify+commit rejected: "+(FWUP_ERR[a.status]||("code "+a.status)));
  log("image staged + CRC-verified on the puck — update armed");
  modalStage("Rebooting", null);
  try{ await fwupSend([0x23]); }catch(e){}
  log("rebooting to apply: the puck goes dark ~5 s while the new firmware is written, then re-enumerates — the panel reconnects itself");
}

const USB_FILTERS=[{vendorId:0x28DE},{vendorId:0x045E},{vendorId:0x057E},{vendorId:0x0F0D},{vendorId:0x054C},{vendorId:0x1209},{vendorId:0x2E8A}];
async function waitReconnect(t0,ms){
  const deadline=Date.now()+ms;
  let seenGone=!dev;
  while(Date.now()<deadline){
    if(dev && window._lastBlobTs>t0) return true;
    if(!dev) seenGone=true;
    if(seenGone && !dev){
      try{
        const ds=await navigator.usb.getDevices();
        if(ds.some(x=>USB_FILTERS.some(f=>f.vendorId===x.vendorId))) return "appeared";
      }catch(_e){}
    }
    await dfuSleep(400);
  }
  return false;
}

let _startPollingFn = null;
export function setStartPolling(fn){ _startPollingFn=fn; }

export async function runUpdate(title, confirmText, getImage){
  if(dfuBusy) return;
  if(!dev){ log("connect to the device first — the update travels over this WebUSB connection"); return; }
  if(!isDongle && (!lastP || lastP[0]<15)){
    log("this puck's firmware ("+(blobBuildId(lastP)||"?")+", status v"+(lastP?lastP[0]:"?")+") predates panel updates (needs v15+) — flash a panel-update-capable build once via UF2 DFU + drag-and-drop, then this works");
    return;
  }
  if(!confirm(confirmText)) return;
  dfuBusy=true; updateUf2UI(); modalOpen(title);
  // polling is managed via the protocol module — we just need the IN pipe
  await dfuSleep(800);
  const t0=Date.now(), oldBuild=blobBuildId(lastP);
  try{
    const image=await getImage();
    if(!imagePanelUpdatable(image) &&
       !confirm("⚠ DOWNGRADE WARNING\n\nThis image predates panel updates (releases up to 0.9.6 do). It will "
        +"flash fine, but the puck it leaves behind CANNOT be updated from this panel — getting off it again "
        +"means UF2 DFU + drag-and-drop.\n\nFlash it anyway?"))
      throw new Error("cancelled — the selected image doesn't support panel updates");
    await fwupRun(image);
    modalStage("Applying on the puck",null);
    $("#updDetail").textContent="the puck goes dark ~5 s while the new firmware is written, then re-enumerates";
    const result=await waitReconnect(t0,40000);
    if(result===true){
      const nb=blobBuildId(lastP);
      modalDone(true,"Update applied — the puck is back and running build "+(nb||"?")+(oldBuild&&nb!==oldBuild?" (was "+oldBuild+")":""));
    }else if(result==="appeared"){
      modalDone(true,"Update applied — the puck is back. Tap Connect to reconnect.");
    }else{
      modalDone(false,"The update was sent and armed, but the puck didn't reconnect within 40 s. Unplug/replug it. "
        +"If it mounts as a UF2BOOT drive, drag the .uf2 onto the drive to recover — the bootloader is intact.");
    }
  }catch(e){
    log("firmware update FAILED: "+e.message);
    modalDone(false,e.message+" — nothing was applied; the running firmware is untouched.");
    try{ await fwupSend([0x24]); }catch(_e){}
  }finally{
    dfuBusy=false; updateUf2UI();
  }
}

export async function pickUf2(f){
  try{
    const image=uf2ToImage(await f.arrayBuffer());
    selectedUf2={name:f.name, image, updatable:imagePanelUpdatable(image)};
    log("UF2 ready: "+f.name+" — app image "+Math.round(image.length/1024)+" KiB, CRC32 0x"+crc32(image).toString(16).padStart(8,"0"));
  }catch(err){
    selectedUf2=null;
    log("UF2 rejected: "+err.message);
  }
  updateUf2UI();
}

// ---- releases ----
const REL_REPO="safijari/openpuck";
export function relAsset(rel,factory){
  const suffix=factory ? "factory-reset" : "standard";
  return rel.assets.find(a=>a.name==="OpenPuck-"+rel.tag_name+"-"+suffix+".uf2")
      || rel.assets.find(a=>new RegExp("^OpenPuck-.*-"+suffix+"\\.uf2$").test(a.name)) || null;
}
function relPanelUpdatable(rel){
  if(!relManifest) return null;
  const known=[relAsset(rel,false),relAsset(rel,true)].filter(a=>a && relManifest[a.name]);
  if(!known.length) return null;
  return known.some(a=>relManifest[a.name].panelUpdate);
}
async function downloadAsset(asset){
  modalStage("Downloading "+asset.name,0);
  let resp=null;
  try{
    const r=await fetch("https://raw.githubusercontent.com/"+REL_REPO+"/firmware/"+asset.name,{cache:"no-store"});
    if(r.ok) resp=r;
  }catch(e){}
  if(!resp){
    try{
      const r=await fetch(asset.url,{headers:{Accept:"application/octet-stream"}});
      if(r.ok) resp=r;
    }catch(e){}
  }
  if(!resp){
    window.open(asset.browser_download_url,"_blank","noopener");
    throw new Error(asset.name+" isn't on the firmware mirror yet and GitHub's asset CDN blocks in-page downloads. "
      +"It's downloading in a new tab instead — drop the file on the local-file card above.");
  }
  const total=+resp.headers.get("content-length") || asset.size || 0;
  const reader=resp.body.getReader(), parts=[]; let got=0;
  for(;;){
    const {done,value}=await reader.read();
    if(done) break;
    parts.push(value); got+=value.length;
    if(total) modalStage("Downloading "+asset.name, Math.min(99,Math.floor(got*100/total)));
  }
  const buf=new Uint8Array(got); let o=0;
  for(const p of parts){ buf.set(p,o); o+=p.length; }
  return uf2ToImage(buf.buffer);
}
function flashRelease(rel,factory){
  const asset=relAsset(rel,factory);
  if(!asset){ log("release "+rel.tag_name+" has no "+(factory?"factory-reset":"standard")+" .uf2 asset"); return; }
  const kb=Math.round((asset.size||0)/1024);
  runUpdate("Updating to "+rel.tag_name+(factory?" (factory reset)":""),
    "Update the puck to "+rel.tag_name+(factory
      ? " using the FACTORY RESET build?\n\nOn its first boot it wipes ALL settings and the controller pairing — you must re-pair the controller afterwards."
      : "?")
    +"\n\n"+asset.name+" ("+kb+" KiB) is downloaded, sent over WebUSB, verified on the puck, and applied on an "
    +"automatic reboot. A failed or interrupted transfer leaves the current firmware untouched.",
    ()=>downloadAsset(asset));
}
function relNotesHtml(md){
  const esc=t=>t.replace(/&/g,"&amp;").replace(/</g,"&lt;").replace(/>/g,"&gt;");
  return esc((md||"").replace(/\r\n/g,"\n").trim())
    .replace(/`([^`\n]+)`/g,(m,c)=>"<code>"+c+"</code>")
    .replace(/\*\*([^*\n]+)\*\*/g,(m,b)=>"<b>"+b+"</b>")
    .replace(/^(#{1,6})\s+(.+)$/gm,(m,h,t)=>"<b>"+t+"</b>")
    .replace(/^\s*[-*]\s+/gm,"• ")
    .replace(/\[([^\]\n]+)\]\((https?:\/\/[^\s)]+)\)/g,
             (m,t,u)=>'<a href="'+u+'" target="_blank" rel="noopener">'+t+"</a>")
    .replace(/(^|[\s(])(https?:\/\/[^\s<)]+)/g,
             (m,pre,u)=>pre+'<a href="'+u+'" target="_blank" rel="noopener">'+u+"</a>");
}
function renderReleases(){
  const list=$("#relList");
  list.textContent="";
  let shown=0;
  for(const rel of relCache){
    const std=relAsset(rel,false), fac=relAsset(rel,true);
    if(!std && !fac) continue;
    shown++;
    const item=document.createElement("div"); item.className="rel-item";
    const row=document.createElement("div"); row.className="rel-row";
    const ver=document.createElement("span"); ver.className="ver"; ver.textContent=rel.tag_name;
    const date=document.createElement("span"); date.className="date";
    date.textContent=(rel.published_at||"").slice(0,10);
    const name=document.createElement("span"); name.className="grow note"; name.style.margin="0";
    name.textContent=rel.name && rel.name!==rel.tag_name ? rel.name : "";
    const lab=document.createElement("label");
    const cb=document.createElement("input"); cb.type="checkbox"; cb.disabled=!fac;
    lab.appendChild(cb); lab.appendChild(document.createTextNode("factory reset"));
    lab.title=fac ? "Flash the -factory-reset build: wipes settings + pairing once on first boot"
                  : "this release has no factory-reset build";
    const btn=document.createElement("button"); btn.textContent="Flash "+rel.tag_name;
    btn.onclick=()=>flashRelease(rel,cb.checked);
    row.append(ver,date,name);
    if(rel.prerelease){
      const pre=document.createElement("span"); pre.className="pill pre";
      pre.textContent="pre-release";
      pre.title="Marked as a pre-release on GitHub: a test build, not the recommended version. "
        +"It flashes like any other release, and the update notice never points at it.";
      row.appendChild(pre);
    }
    if(relPanelUpdatable(rel)===false){
      const pill=document.createElement("span"); pill.className="pill dn";
      pill.textContent="no panel updates";
      pill.title="This build predates panel-update support: it flashes fine from here, but a puck running it "
        +"can only be updated again via UF2 DFU + drag-and-drop.";
      row.appendChild(pill);
    }
    row.append(lab,btn);
    item.appendChild(row);
    if((rel.body||"").trim()){
      const det=document.createElement("details"); det.className="rel-notes";
      const sum=document.createElement("summary"); sum.textContent="release notes";
      const body=document.createElement("div"); body.className="body";
      body.innerHTML=relNotesHtml(rel.body);
      det.append(sum,body);
      item.appendChild(det);
    }
    list.appendChild(item);
  }
  if(!shown) list.textContent="no releases with OpenPuck .uf2 assets found";
}
export async function loadReleases(force){
  if(relCache && !force) return;
  const list=$("#relList");
  list.textContent="loading…";
  try{
    const [r,mr]=await Promise.all([
      fetch("https://api.github.com/repos/"+REL_REPO+"/releases?per_page=15",
            {cache:"no-store",headers:{Accept:"application/vnd.github+json"}}),
      fetch("https://raw.githubusercontent.com/"+REL_REPO+"/firmware/manifest.json",{cache:"no-store"})
        .catch(()=>null),
    ]);
    if(!r.ok) throw new Error("GitHub API "+r.status);
    try{ relManifest=(mr && mr.ok) ? await mr.json() : null; }catch(e){ relManifest=null; }
    relCache=(await r.json()).filter(x=>!x.draft);
    renderReleases();
    checkUpdateNotice();
  }catch(e){
    list.innerHTML='could not load releases ('+e.message.replace(/</g,"&lt;")
      +') — <a href="https://github.com/'+REL_REPO+'/releases" target="_blank" rel="noopener" style="color:var(--acc2)">open the releases page</a> and use the local-file card';
    relCache=null;
  }
}
