#include "haptics.h"
#include "bonds.h"
#include "config.h"
#include "rf_link.h"
#include <Arduino.h>
#include <string.h>

volatile bool    g_relayPend = false;
uint8_t          g_relayBuf[24];
volatile uint8_t g_relayN = 0;
uint8_t          g_relayOp  = 0xE3;   // E3 poll
// relay sub-TLV type byte = SET (confirmed on hardware: E3 + [len][05][rid][payload] makes the controller act
// on it -- haptics buzz + lizard-off lands). GET is 01.
uint8_t          g_relaySub = 0x05;
volatile uint8_t g_testHaptic = 0;
volatile uint8_t g_hapticStop = 0;
unsigned long    g_hapticBlockUntil = 0;   // drop Steam haptics briefly during reconnect settle

static unsigned long g_haptic82Ms = 0;     // millis of last 0x82 haptic OUTPUT relayed (Steam mode)
static bool          g_haptic82On = false; // a non-zero 0x82 haptic is currently active (awaiting host stop)

// diagnostic capture: a ring of the last OUTPUT reports Steam sends (rid/slot/bytes/ms), dumped with 'H'.
// Reproduce the whine, press Steam to stop it, then 'H' to see the ON stream + the exact OFF frame Steam sends.
struct HapLog { uint32_t ms; uint8_t slot, rid, n, b[12]; };
static HapLog  g_hapLog[28];
static uint8_t g_hapHead = 0;

void hapLogAdd(uint8_t slot, uint8_t rid, const uint8_t* b, uint16_t n){
  HapLog &e = g_hapLog[g_hapHead]; e.ms = millis(); e.slot = slot; e.rid = rid; e.n = (uint8_t)(n>255?255:n);
  for(int i=0;i<12;i++) e.b[i] = (i<(int)n) ? b[i] : 0;
  g_hapHead = (uint8_t)((g_hapHead+1) % (sizeof g_hapLog / sizeof g_hapLog[0]));
}
void hapticDumpLog(){   // dump the captured OUTPUT-report history (oldest->newest) for the haptic-whine hunt
  const uint8_t N=sizeof g_hapLog/sizeof g_hapLog[0]; uint32_t now=millis();
  Serial.printf("# --- haptic/OUTPUT history (now=%lu, connSlot=%d) ---\n",(unsigned long)now,g_connSlot);
  for(uint8_t i=0;i<N;i++){ HapLog &e=g_hapLog[(g_hapHead+i)%N]; if(!e.ms && !e.rid) continue;
    Serial.printf("# -%lums if%u rid=%02X n=%u:",(unsigned long)(now-e.ms),e.slot,e.rid,e.n);
    for(uint8_t j=0;j<12 && j<e.n;j++) Serial.printf(" %02X",e.b[j]); Serial.println(); }
  Serial.println("# --- end ---");
}

bool hapticLinkUp(){
  return g_connSlot>=0 && (millis()-g_connReplyMs) < 300;
}
bool haptic82Blocked(){
  return !hapticLinkUp() || (g_hapticBlockUntil && (int32_t)(millis()-g_hapticBlockUntil) < 0);
}
bool hapticRelaySlotOk(int slot){
  return g_connSlot>=0 && slot==g_connSlot;
}
static bool haptic82PayloadOn(const uint8_t* p, uint16_t n){
  if(n<3) return false;
  for(uint16_t i=2;i<n;i++) if(p[i]) return true;   // observed form is [01 01 gain], but treat any trailing non-zero as active
  return false;
}
static void hapticCancelPendingOn(){
  if(g_relayPend && g_relayBuf[0]==0x82 && haptic82PayloadOn(g_relayBuf+2, g_relayBuf[1]))
    g_relayPend = false;
}
void haptic82HostReport(const uint8_t* p, uint16_t n){
  if(n<3) return;
  g_haptic82Ms = millis();
  // Track on/off only. Do NOT synthesize a stop burst when Steam's own stop arrives: that stop is already
  // forwarded verbatim, so adding 0x82-zero frames on top just makes the controller see the stop several times
  // over. Each 0x82 is a discrete pad click, so the extra frames are exactly the spurious end-of-movement
  // "click"/buzz that the real puck never produces. (Connect-time clearing still runs in hapticTask().)
  g_haptic82On = haptic82PayloadOn(p,n);
}
// Queue a pending host haptic / test-haptic / stop relay (runs inside the poll cadence -- never at raw loop rate).
void rfConnQueueHapticRelay(){
  if(g_relayPend) return;
  if(g_testHaptic){
    g_relayBuf[0]=0x82; g_relayBuf[1]=3; g_relayBuf[2]=0x01; g_relayBuf[3]=0x01; g_relayBuf[4]=0xF7; g_relayPend=true; g_testHaptic--;
  } else if(g_hapticStop && !g_xbox){
    g_relayBuf[0]=0x82; g_relayBuf[1]=3; g_relayBuf[2]=0x01; g_relayBuf[3]=0x01; g_relayBuf[4]=0x00; g_relayPend=true; g_hapticStop--;
  }
}
void rfConnFlushRelay(uint8_t ch){
  if(!g_relayPend) return;
  uint8_t rid=g_relayBuf[0], rl=g_relayBuf[1]; if(rl>18)rl=18;
  uint8_t p[24]; p[0]=g_relayOp; p[1]=(uint8_t)(1+rl); p[2]=g_relaySub; p[3]=rid;
  for(uint8_t i=0;i<rl;i++) p[4+i]=g_relayBuf[2+i];
  rfConnTx(ch,0x07,p,(uint8_t)(4+rl));
  g_relayPend=false;
}

void hapticInit(){
  g_relayPend=false; g_haptic82On=false;
  g_hapticBlockUntil = millis() + HAPTIC_RECONNECT_BLOCK_MS;   // boot: block stale Steam 0x82 until link stable
  g_hapticStop = HAPTIC_STOP_BURST;
}
void hapticTask(){
  static bool wasHapticLinkUp=false;
  bool up=hapticLinkUp();
  if(up && !wasHapticLinkUp){
    g_hapticBlockUntil = millis() + HAPTIC_RECONNECT_BLOCK_MS;
    g_haptic82On = false;
    hapticCancelPendingOn();
    g_hapticStop = HAPTIC_STOP_BURST;
  }
  wasHapticLinkUp=up;
  // Steam-mode: host went quiet -> mark the 0x82 stream inactive. Do NOT synthesize a stop: trackpad haptics
  // are one-shot pulses, so firing a 0x82-zero ~HAPTIC_QUIET_MS after a swipe ends is the extra end-of-movement
  // click the real puck doesn't make. Steam forwards its own stop for any sustained haptic.
  if (!g_xbox && g_haptic82On && millis()-g_haptic82Ms > HAPTIC_QUIET_MS) g_haptic82On=false;
}
