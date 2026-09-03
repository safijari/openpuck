import { $, fmtDur } from './util.js';

export let hangLog=[], pendingHang=null;
export function setPendingHang(v){ pendingHang=v; }
export function clearPendingHang(){ pendingHang=null; }
export function renderHangLog(){
  const out=$("#hangOut"), sum=$("#hangSummary"); if(!out) return;
  if(!hangLog.length){ out.textContent=""; sum.textContent="no resets logged yet"; return; }
  const ups=hangLog.filter(e=>e.uptime!=null).map(e=>e.uptime);
  const avg=ups.length?Math.round(ups.reduce((a,b)=>a+b,0)/ups.length):null;
  sum.textContent = hangLog.length+" reset(s)"+(avg!=null?("  ·  avg uptime "+fmtDur(avg)):"");
  const pad=(s,n)=>String(s).padEnd(n);
  out.textContent = pad("time",11)+pad("uptime",9)+pad("reason",16)+pad("stage",10)+pad("PC",11)+"usbd\n"
    + hangLog.map(e=>pad(e.time,11)+pad(e.uptime!=null?fmtDur(e.uptime):"-",9)+pad(e.reason,16)+pad(e.stage||"-",10)+pad(e.pc||"-",11)+(e.usbd!=null?e.usbd:"-")).join("\n");
}
export function hangLogPush(entry){ hangLog.unshift(entry); }
export function hangLogClear(){ hangLog=[]; renderHangLog(); }
export function downloadHangCsv(){
  const rows=[["time","uptime_s","reason","stage","pc","lr","usbd_free_words"]].concat(
    hangLog.map(e=>[e.time, e.uptime!=null?Math.round(e.uptime):"", e.reason, e.stage, e.pc, e.lr, e.usbd]));
  const csv=rows.map(r=>r.map(x=>'"'+String(x).replace(/"/g,'""')+'"').join(",")).join("\n");
  const a=document.createElement("a"); a.href=URL.createObjectURL(new Blob([csv],{type:"text/csv"}));
  a.download="openpuck-hanglog.csv"; a.click();
}
