export const $ = s=>document.querySelector(s);
export const DEBUG_UI = new URLSearchParams(location.search).get("debug")==="true";
export const BETA_UI = new URLSearchParams(location.search).get("beta")==="true";
export function log(m){ const l=$("#log"); l.textContent=(new Date().toLocaleTimeString()+"  "+m+"\n"+l.textContent).slice(0,2000); }
export function escHtml(s){
  return String(s).replace(/[&<>"']/g, c=>({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[c]));
}
export function fmtDur(s){ s=Math.round(s); return s>=60 ? (Math.floor(s/60)+"m "+(s%60)+"s") : (s+"s"); }
export function fmtSlider(id,v){ return id==="hapBlockS" ? v+" s" : ""+v; }
export function setSlider(id,v){ const el=$("#"+id); if(document.activeElement!==el){ el.value=v; } $("#"+id+"V").textContent=fmtSlider(id,el.value); }
export function betaPopup({title, body, okText="OK", cancelText="Cancel"}){
  return new Promise(resolve=>{
    let done=false;
    const shade=document.createElement("div");
    shade.style.cssText="position:fixed;inset:0;z-index:90;background:rgba(8,10,16,.55);display:flex;align-items:center;justify-content:center;padding:18px";
    shade.innerHTML = '<div style="background:var(--card);border:1px solid #80651c;border-radius:12px;padding:20px 22px;width:min(440px,94vw);box-shadow:0 18px 60px rgba(0,0,0,.45)">'
      +'<h3 style="margin:0 0 10px;font-size:16px">'+escHtml(title)+'</h3>'
      +body.map(p=>'<p class="note" style="margin:8px 0">'+escHtml(p)+'</p>').join("")
      +'<div class="row" style="justify-content:flex-end;margin:18px 0 0">'
      +'<button data-beta-cancel>'+escHtml(cancelText)+'</button>'
      +'<button class="beta active" data-beta-ok>'+escHtml(okText)+'</button>'
      +'</div></div>';
    const finish=ok=>{
      if(done) return;
      done=true;
      shade.remove();
      resolve(!!ok);
    };
    shade.querySelector("[data-beta-cancel]").onclick=()=>finish(false);
    shade.querySelector("[data-beta-ok]").onclick=()=>finish(true);
    shade.onclick=e=>{ if(e.target===shade) finish(false); };
    shade.onkeydown=e=>{ if(e.key==="Escape") finish(false); };
    document.body.appendChild(shade);
    shade.tabIndex=-1;
    shade.focus();
  });
}
