import { $ } from './util.js';

export let g_activeSlot = 0;
export let g_slotData = [];
export function setActiveSlot(v){ g_activeSlot=v; }
export function setSlotData(v){ g_slotData=v; }

export function updateSlotDisplay(slotIdx) {
  const d = g_slotData[slotIdx];
  if (!d) { $("#stLink").innerHTML="—"; $("#stBatt").textContent="—"; $("#stRssi").textContent="—";
            $("#stSlotPolls").textContent="—"; $("#stSlotF1").textContent="—"; $("#stSlotFail").textContent="—"; return; }
  $("#stLink").innerHTML = d.up ? '<span class="pill up">up</span>' : '<span class="pill dn">down</span>';
  $("#stBatt").textContent = (d.up && d.battery) ? d.battery+"%" : "—";
  $("#stRssi").textContent = (d.up && d.rssi) ? ("-"+d.rssi+" dBm") : "—";
  if (d.stats) {
    $("#stSlotPolls").textContent = d.stats.polls+" /s";
    $("#stSlotF1").textContent = d.up ? (d.stats.f1+" /s ("+d.stats.newps+" new)") : "—";
    $("#stSlotFail").textContent = d.stats.crc+" · "+d.stats.norx+" · "+d.stats.relay;
  } else {
    $("#stSlotPolls").textContent="—"; $("#stSlotF1").textContent="—"; $("#stSlotFail").textContent="—";
  }
}

export function renderSlotTabs(bondedCount) {
  const bar = $("#slotTabs");
  if (bondedCount <= 1) { bar.style.display="none"; updateSlotDisplay(g_activeSlot); return; }
  bar.style.display="flex";
  bar.innerHTML="";
  for (let s=0; s<4; s++) {
    const d = g_slotData[s];
    if (!d) continue;
    const btn = document.createElement("button");
    btn.className = "slot-tab" + (s===g_activeSlot ? " active" : "");
    const dot = document.createElement("span");
    dot.className = "dot" + (d.up ? " up" : "");
    btn.appendChild(dot);
    btn.appendChild(document.createTextNode("Controller "+(s+1)));
    btn.onclick = () => { g_activeSlot=s; renderSlotTabs(bondedCount); };
    bar.appendChild(btn);
  }
  updateSlotDisplay(g_activeSlot);
}
