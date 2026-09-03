import { $ } from './util.js';

const TRAIL_KEY="opk_loop_trail", TRAIL_MAX=400;
export let loopTrail=[]; try{ loopTrail=JSON.parse(localStorage.getItem(TRAIL_KEY))||[]; }catch(e){ loopTrail=[]; }
export function trailAdd(m){
  loopTrail.unshift({t:Date.now(), m});
  if(loopTrail.length>TRAIL_MAX) loopTrail.length=TRAIL_MAX;
  try{ localStorage.setItem(TRAIL_KEY, JSON.stringify(loopTrail)); }catch(e){}
  renderTrail();
}
export function renderTrail(){
  const out=$("#trailOut"), sum=$("#trailSummary"); if(!out) return;
  sum.textContent = loopTrail.length ? (loopTrail.length+" event(s) — newest first") : "no events yet";
  out.textContent = loopTrail.length
    ? loopTrail.map(e=>new Date(e.t).toLocaleString()+"  "+e.m).join("\n") : "—";
}
export function trailClear(){ loopTrail=[]; try{ localStorage.removeItem(TRAIL_KEY); }catch(e){} renderTrail(); }
