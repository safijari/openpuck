import { $, log } from './util.js';
import { dev, epIn, lastP, setLastP, send, readBlob, readFrame, setBackupBusy, waitIdle } from './protocol.js';
import { TYPE_DEFS, PAD_STICK_FIELD0 } from './types.js';
import { lizardBindings, lizardBusy, lizardCapable, getLizardLoaded, readLizard, LZ_MAX } from './lizard.js';
import { modalOpen, modalStage, modalDone } from './firmware.js';

function hexEnc(arr){ return [...arr].map(x=>x.toString(16).padStart(2,"0")).join(""); }
function hexDec(h){ const a=[]; if(!h) return a; for(let i=0;i+1<h.length;i+=2) a.push(parseInt(h.substr(i,2),16)); return a; }

function buildBackup(p, bp){
  const mask=bp[1], bonds=[];
  for(let s=0;s<4;s++){ const rec=bp.slice(2+s*24, 2+s*24+24);
    bonds.push({slot:s, used:!!((mask>>s)&1), rec:hexEnc(rec)}); }
  const types=[];
  for(let et=0; et<TYPE_DEFS.length; et++){ const q=73+et*9;
    const t={back:[p[q],p[q+1],p[q+2],p[q+3]], qam:p[q+4], abSwap:p[q+5], pad:p[q+6], led:p[q+7], rumble:p[q+8]};
    if(p[0]>=21 && p.length>192) t.padStick=[p[185+et*2],p[186+et*2]];
    types.push(t); }
  const cfg={ mode:p[1], mDiv:p[2], mFric:p[3], persistMode:p[22], chord:[p[23],p[24],p[25]], types };
  if(p[0]>=18 && p.length>183) cfg.chordD=[p[180],p[181],p[182],p[183]];
  if(p[0]>=19 && p.length>184) cfg.swGyroLegacy=p[184];
  if(getLizardLoaded() && lizardCapable(p) && !lizardBusy)
    cfg.lizardMap=lizardBindings.map(b=>({outType:b.outType,od:b.od.slice(),trig:b.trig>>>0,hold:b.hold>>>0}));
  return { magic:"openpuck-backup", version:(cfg.lizardMap?2:1), bonds, config:cfg };
}
function downloadBackup(obj){
  const ts=new Date().toISOString().slice(0,19).replace(/[:T]/g,"-");
  const a=document.createElement("a");
  a.href=URL.createObjectURL(new Blob([JSON.stringify(obj,null,2)],{type:"application/json"}));
  a.download="openpuck-backup-"+ts+".json"; a.click();
}

export async function exportBackup(){
  if(!dev){ log("not connected"); return; }
  setBackupBusy(true); await waitIdle();
  try{
    await send([0x01]); const cfg=await readBlob(); if(cfg) setLastP(cfg);
    if(!lastP){ log("export: no config snapshot yet — wait a second and retry"); return; }
    await send([0x09]);
    let bp=null;
    for(let t=0; t<8 && !bp; t++) bp=await readFrame(0xA7, 2+4*24);
    if(!bp){ log("export: no bond data (firmware too old for 0x09 export — reflash)"); return; }
    const backup=buildBackup(lastP, bp);
    downloadBackup(backup);
    const n=backup.bonds.filter(b=>b.used).length;
    log("exported backup — "+n+" bonded controller slot(s) + all settings");
  } finally { setBackupBusy(false); }
}

export async function importBackup(file, setField, CHORD_DPAD_FIELD){
  if(!dev){ log("not connected"); return; }
  let obj; try{ obj=JSON.parse(await file.text()); }
  catch(e){ log("import: not valid JSON"); return; }
  if(obj.magic!=="openpuck-backup" || !obj.bonds || !obj.config){ log("import: unrecognized backup file"); return; }
  const bondedN=obj.bonds.filter(b=>b.used).length;
  if(!confirm("Restore this backup onto the CONNECTED puck (serial "+(dev.serialNumber||"?")+")?\n\nMake sure this is the TARGET puck, not the one you exported from — the panel only talks to the puck you last connected.\n\nThis OVERWRITES its "+bondedN+" controller pairing(s) and ALL settings, then reboots it.\n\nResult: any controller paired to the original puck will connect to this one with no re-pairing.")) return;
  log("importing onto puck "+(dev.serialNumber||"?"));
  setBackupBusy(true); await waitIdle();
  modalOpen("Restore backup");
  const c=obj.config;
  const typeN=Array.isArray(c.types)?Math.min(c.types.length,4):0;
  const lizardN=(Array.isArray(c.lizardMap) && lizardCapable(lastP))?Math.min(c.lizardMap.length,LZ_MAX):0;
  const baseN=6 + (Array.isArray(c.chordD)?4:0) + (c.swGyroLegacy!==undefined?1:0);
  const padStickN=Array.isArray(c.types)?c.types.slice(0,typeN).filter(t=>Array.isArray(t.padStick)).length*2:0;
  const total=baseN + typeN*9 + padStickN + lizardN + 4 + 1;
  let step=0, stage="";
  const tick=()=>modalStage(stage, Math.min(99,Math.floor(step*100/total)));
  try{
    modalStage("Checking puck firmware", null);
    await send([0x09]);
    let probe=null; for(let t=0;t<8 && !probe;t++) probe=await readFrame(0xA7, 2+4*24);
    if(!probe){
      log("import ABORTED — the connected puck does not support import (old firmware).");
      modalDone(false, "This puck's firmware is too old to import a backup. Flash it with the latest "
        +"OpenPuck build (the same one that produced the export), reconnect, then import again.", "Import failed");
      return;
    }
    const w=async(bytes)=>{ await send(bytes); await readBlob(); step++; tick(); };
    const sf=async(f,v)=>{ await w([0x02, f, (v|0)&0xff]); };
    stage="Restoring settings"; tick();
    await sf(1,c.mDiv); await sf(2,c.mFric); await sf(16,c.persistMode?1:0);
    await sf(17,c.chord[0]); await sf(18,c.chord[1]); await sf(19,c.chord[2]);
    if(Array.isArray(c.chordD)) for(let i=0;i<4;i++) await sf(CHORD_DPAD_FIELD[i], c.chordD[i]);
    if(c.swGyroLegacy!==undefined) await sf(38, c.swGyroLegacy?1:0);
    if(typeN){
      for(let et=0; et<typeN; et++){ const t=c.types[et];
        for(let k=0;k<4;k++) await sf(40+et*9+k, t.back[k]);
        await sf(40+et*9+4, t.qam); await sf(40+et*9+5, t.abSwap?1:0);
        await sf(40+et*9+6, t.pad?1:0); await sf(40+et*9+7, t.led);
        await sf(40+et*9+8, t.rumble!==undefined?t.rumble:1);
        if(Array.isArray(t.padStick)){ await sf(PAD_STICK_FIELD0+et*2, t.padStick[0]); await sf(PAD_STICK_FIELD0+et*2+1, t.padStick[1]); } }
      log("settings replayed ("+(baseN+typeN*9)+" fields)");
    } else { log("settings replayed ("+baseN+" fields — no per-type config in this backup)"); }
    if(lizardN){
      stage="Restoring button map"; tick();
      const map=c.lizardMap.slice(0,LZ_MAX);
      await send([0x13, map.length&0xff]);
      for(let i=0;i<map.length;i++){
        const b=map[i], od=(b.od||[]).slice(0,7); while(od.length<7) od.push(0);
        const t=(b.trig||0)>>>0, h=(b.hold||0)>>>0;
        await send([0x12, i, (b.outType||0)&0xff, ...od.map(x=>(x||0)&0xff),
          t&0xff,(t>>>8)&0xff,(t>>>16)&0xff,(t>>>24)&0xff,
          h&0xff,(h>>>8)&0xff,(h>>>16)&0xff,(h>>>24)&0xff]);
        step++; tick();
      }
      await send([0x14]);
      await readLizard(dev, epIn); // drain the 0xAA echo
      log("lizard map restored — "+map.length+" bindings");
    }
    stage="Restoring controller pairings"; tick();
    let n=0;
    for(let s=0;s<4;s++){
      const b=obj.bonds.find(x=>x.slot===s) || {used:false, rec:""};
      const rec=hexDec(b.rec), r24=new Array(24).fill(0);
      for(let i=0;i<24 && i<rec.length;i++) r24[i]=rec[i];
      const used=(b.used && rec.length===24)?1:0; if(used) n++;
      await w([0x0D, s, used, ...r24]);
    }
    log("wrote "+n+" bond slot(s) — committing + rebooting");
    stage="Committing + rebooting"; step++; tick();
    await send([0x0E, (c.mode!==undefined?c.mode:0xFF)&0xff]);
    log("import sent — puck is cloning and rebooting. Reconnect after it returns.");
    modalDone(true, "Backup restored — the puck is cloning and rebooting. Reconnect after it returns.");
  } catch(e){
    log("import FAILED — "+e.message);
    modalDone(false, e.message+" — the import stopped partway; reconnect and retry.", "Import failed");
  } finally { setBackupBusy(false); }
}
