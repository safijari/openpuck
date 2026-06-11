#include "haptics.h"
#include "bonds.h"
#include "config.h"
#include "rf_link.h"
#include <Arduino.h>
#include <string.h>

uint8_t          g_relayOp  = 0xE3;   // E3 poll
// relay sub-TLV type byte = SET (confirmed on hardware: E3 + [len][05][rid][payload] makes the controller act
// on it -- haptics buzz + lizard-off lands). GET is 01.
uint8_t          g_relaySub = 0x05;
volatile uint8_t g_testHaptic = 0;
volatile uint8_t g_hapticStop = 0;
unsigned long    g_hapticBlockUntil = 0;   // drop Steam haptics briefly during reconnect settle

static unsigned long g_haptic82Ms = 0;     // millis of last 0x82 haptic OUTPUT relayed (Steam mode)
static bool          g_haptic82On = false; // a non-zero 0x82 haptic is currently active (awaiting host stop)
static unsigned long g_reinitAt = 0;       // when to fire the next post-reconnect haptic re-init (0 = none scheduled)
static uint8_t       g_reinitLeft = 0;     // how many re-init shots remain in this connect window
static bool          g_hapClearArmed=false;// haptic activity happened -> arm a clear once it goes idle (catches a
                                           // latch that engaged during/after use, even seconds after connect)

// ---- relay ring: multi-producer (USB ISR + loop-context console/xinput), single consumer (poll flush) ----
// Producers serialize through a brief PRIMASK critical section (copy is <=62 bytes); the consumer only ever
// touches the tail entry, which no producer writes while the ring isn't full -- so a flush can't be torn.
struct RelayMsg { uint8_t rid, len; uint8_t data[RELAY_MAXP]; };
#define RELAY_QLEN 32          // deep enough to hold a full Steam settings/LED transaction burst without loss
static RelayMsg         g_rq[RELAY_QLEN];
static volatile uint8_t g_rqHead = 0, g_rqTail = 0;   // head=next write, tail=next read; empty when equal
static inline uint8_t rqNext(uint8_t i){ return (uint8_t)((i+1)%RELAY_QLEN); }

bool relayPending(){ return g_rqHead != g_rqTail; }
bool relayEnqueue(uint8_t rid, const uint8_t* payload, uint8_t plen){
  if (plen > RELAY_MAXP) plen = RELAY_MAXP;            // 60B is the RF frame ceiling; longer can't be relayed
  uint32_t pm = __get_PRIMASK(); __disable_irq();
  uint8_t h = g_rqHead, nx = rqNext(h);
  // Full -> evict the OLDEST entry, never the newest. Steam sends commands as bursts where the meaningful
  // frame comes LAST (a settings transaction ends with its commit; a haptic stream ends with its stop). The
  // old drop-newest policy threw away exactly those, so LED commits and haptic stops were the casualties --
  // brightness never applied and a glide's final stop never reached the controller (stuck buzz). Dropping the
  // oldest instead guarantees the most recent command always lands.
  if (nx == g_rqTail) g_rqTail = rqNext(g_rqTail);
  g_rq[h].rid = rid; g_rq[h].len = plen;
  if (plen) memcpy(g_rq[h].data, payload, plen);
  g_rqHead = nx;
  // Any haptic relay (Steam OR Xbox rumble OR test) arms the idle-clear and refreshes its timer -- so the
  // during-use buzz gets cleared in EVERY USB mode, not just Steam. The re-init's own 0x81/0x87 don't match.
  if (rid==0x82){ g_haptic82Ms = millis(); g_hapClearArmed = true; }
  __set_PRIMASK(pm); return true;
}

#if OPK_LOG
// diagnostic capture: a ring of the last OUTPUT reports Steam sends (rid/slot/bytes/ms), dumped with 'H'.
// Reproduce the whine, press Steam to stop it, then 'H' to see the ON stream + the exact OFF frame Steam sends.
struct HapLog { uint32_t ms; uint8_t slot, rid, n, b[16]; };   // 16 payload bytes: capture full 0x87 settings frames
// Big always-on ring: log EVERYTHING from boot (Steam writes, our TX-to-controller, link edges) so a rare
// reconnect-buzz can be caught after the fact -- the trigger happens moments after the puck boots, while this
// RAM is fresh, and we dump it once the panel reconnects. 4096 * 20B ~= 80KB (we have plenty of free RAM).
#define HAPLOG_N 4096
static HapLog   g_hapLog[HAPLOG_N];
static uint16_t g_hapHead = 0;
static uint16_t g_hapTail = 0;  // live/dump drain cursor (loop-context reader; chases g_hapHead)

void hapLogAdd(uint8_t slot, uint8_t rid, const uint8_t* b, uint16_t n){
  // Written from the USB SET ISR (handleSet) AND loop context (relay flush / link edges) -> guard g_hapHead.
  // Special slot markers for the diagnostic capture: 0xFE = a frame WE transmitted to the controller (TX
  // relay); 0xFD = a link state edge (b[0]=1 up, 0 down). Real Steam writes use the interface index (0..3).
  uint32_t pm = __get_PRIMASK(); __disable_irq();
  HapLog &e = g_hapLog[g_hapHead]; e.ms = millis(); e.slot = slot; e.rid = rid; e.n = (uint8_t)(n>255?255:n);
  for(int i=0;i<16;i++) e.b[i] = (i<(int)n) ? b[i] : 0;
  g_hapHead = (uint8_t)((g_hapHead+1) % (sizeof g_hapLog / sizeof g_hapLog[0]));
  __set_PRIMASK(pm);
}
void hapticDumpLog(){   // dump the captured history (oldest->newest) over CDC for the haptic-whine hunt
  const uint16_t N=HAPLOG_N; uint32_t now=millis();
  Serial.printf("# --- capture history (now=%lu, connSlot=%d) ---\n",(unsigned long)now,g_connSlot);
  for(uint16_t i=0;i<N;i++){ HapLog &e=g_hapLog[(uint16_t)((g_hapHead+i)%N)]; if(!e.ms && !e.rid) continue;
    Serial.printf("# -%lums if%u rid=%02X n=%u:",(unsigned long)(now-e.ms),e.slot,e.rid,e.n);
    for(uint8_t j=0;j<16 && j<e.n;j++) Serial.printf(" %02X",e.b[j]); Serial.println(); }
  Serial.println("# --- end ---");
}
// ---- drain cursor: stream entries to the WebUSB panel. resetDrain(false)=from "now" (live only);
//      resetDrain(true)=from the OLDEST entry (dump the whole ring from boot). pull skips empty slots. ----
void hapLogResetDrain(bool fromBoot){
  g_hapTail = fromBoot ? (uint16_t)((g_hapHead+1)%HAPLOG_N)   // oldest slot (the next-to-overwrite holds it)
                       : g_hapHead;                            // "now"
}
bool hapLogPull(uint32_t* logMs, uint8_t* slot, uint8_t* rid, uint8_t* n, uint8_t bytes16[16]){
  while(g_hapTail != g_hapHead){
    HapLog &e=g_hapLog[g_hapTail];
    g_hapTail=(uint16_t)((g_hapTail+1)%HAPLOG_N);
    if(!e.ms && !e.rid) continue;                            // skip empty slot (ring not yet full)
    // Return the ABSOLUTE log time (millis since boot), not "age now". The panel drains in batches 100ms
    // apart, so an age computed here would jump between batches; the panel computes age vs the newest entry.
    *logMs=e.ms; *slot=e.slot; *rid=e.rid; *n=(e.n>16)?16:e.n; memcpy(bytes16,e.b,16);
    return true;
  }
  return false;
}
#endif // OPK_LOG

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
static void hapticCancelPendingOn(){   // void queued 0x82-ON entries (stale Steam haptics across a reconnect)
  uint32_t pm = __get_PRIMASK(); __disable_irq();
  for(uint8_t i=g_rqTail; i!=g_rqHead; i=rqNext(i)){
    RelayMsg &m = g_rq[i];
    if(m.rid==0x82){ bool on=false; for(uint8_t j=2;j<m.len;j++) if(m.data[j]){on=true;break;} if(on) m.rid=0; }
  }
  __set_PRIMASK(pm);
}
void haptic82HostReport(const uint8_t* p, uint16_t n){
  if(n<3) return;
  g_haptic82Ms = millis();
  g_hapClearArmed = true;   // any haptic activity arms a clear when it next goes idle (kills a latch from this use)
  // Track on/off only. Do NOT synthesize a stop burst when Steam's own stop arrives: that stop is already
  // forwarded verbatim, so adding 0x82-zero frames on top just makes the controller see the stop several times
  // over. Each 0x82 is a discrete pad click, so the extra frames are exactly the spurious end-of-movement
  // "click"/buzz that the real puck never produces. (Connect-time clearing still runs in hapticTask().)
  g_haptic82On = haptic82PayloadOn(p,n);
}
// Queue a pending test-haptic / stop relay (runs inside the poll cadence -- never at raw loop rate).
void rfConnQueueHapticRelay(){
  if(relayPending()) return;            // host relays first; injectables wait for an idle cycle
  static const uint8_t HAP_ON[3]  = {0x01,0x01,0xF7};
  static const uint8_t HAP_OFF[3] = {0x01,0x01,0x00};
  if(g_testHaptic){
    if(relayEnqueue(0x82, HAP_ON, 3)) g_testHaptic--;
  } else if(g_hapticStop && !g_xbox){
    if(relayEnqueue(0x82, HAP_OFF, 3)) g_hapticStop--;
  }
}
void rfConnFlushRelay(uint8_t ch, uint8_t s1){
  while(g_rqTail != g_rqHead){
    RelayMsg &m = g_rq[g_rqTail];
    if(m.rid){                          // rid 0 = entry voided by hapticCancelPendingOn -> skip, take the next
      uint8_t rl = m.len; if(rl > RELAY_MAXP) rl = RELAY_MAXP;
      uint8_t p[4+RELAY_MAXP]; p[0]=g_relayOp; p[1]=(uint8_t)(1+rl); p[2]=g_relaySub; p[3]=m.rid;
      memcpy(p+4, m.data, rl);
      hapLogAdd(0xFE, m.rid, m.data, rl);   // log what we actually TX to the controller (slot 0xFE) for the buzz hunt
      g_rqTail = rqNext(g_rqTail);      // copied out -> release the slot before the TX
      // s1 carries a PID distinct from the GET poll that follows (caller cycles it), so the controller's ESB
      // dedup never mistakes the GET for a retransmit of this relay. 80us RX window: the relay is NO-ACK, the
      // controller sends nothing back, so don't burn the full ~1.2ms reply window (that was halving the poll
      // rate during haptics).
      rfConnTx(ch,s1,p,(uint8_t)(4+rl),80);
      return;                           // ONE relay per poll cycle (matches the real puck's pacing)
    }
    g_rqTail = rqNext(g_rqTail);
  }
}

// Haptic-subsystem RE-INIT: the exact sequence Steam sends (captured on hardware) when it (re)takes control,
// and which clears a stuck haptic script on the controller -- a 0x81 reset action plus 0x87 writes to the
// haptic registers (30/07/08/31/52, 18/2e/34/35). We replay it to recover from the latched-buzz the
// controller falls into across a reconnect. Brightness (0x87 reg 2d) is deliberately OMITTED so we don't
// stomp the LED. Enqueued onto the normal relay (drains in the poll cadence).
void hapticReinit(){
  // The FULL sequence Steam sends when it (re)takes control / on a Steam-button press -- captured on hardware
  // and proven to clear a stuck haptic. Replayed verbatim and in order (no trimming this time: the brightness
  // write 0x87[2d] and the two 7-byte 0x81 frames are part of what Steam sends, and dropping them may be why
  // the trimmed version didn't always clear the connect buzz).
  static const uint8_t H30[]={0x30,0x00,0x00,0x07,0x07,0x00,0x08,0x07,0x00,0x31,0x02,0x00,0x52,0x03,0x00};
  static const uint8_t H18[]={0x18,0x00,0x00,0x2e,0x00,0x00,0x34,0xff,0xff,0x35,0xff,0xff,0x34,0xff,0xff};
  static const uint8_t H35[]={0x35,0xff,0xff,0x2e,0x00,0x00};
  static const uint8_t T81A[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  static const uint8_t T81B[]={0x01,0x00,0x00,0x00,0x00,0x00,0x00};
  relayEnqueue(0x81, nullptr, 0);            // reset action (FUN_0001f554) -- Steam sends this first
  relayEnqueue(0x87, H30, sizeof H30);
  relayEnqueue(0x87, H18, sizeof H18);       // haptic config (enabled/amplifier/gain): the part that clears a latch
  relayEnqueue(0x87, H35, sizeof H35);
  relayEnqueue(0x81, T81A, sizeof T81A);
  relayEnqueue(0x81, T81B, sizeof T81B);
  // NOTE: the brightness write (0x87 2d) that Steam includes is deliberately OMITTED -- it's an LED setting,
  // not haptic, and this re-init now fires often (after every haptic-idle), so we must not flicker the LED.
}
void hapticInit(){
  g_rqHead = g_rqTail = 0; g_haptic82On=false;
  g_hapticBlockUntil = millis() + HAPTIC_RECONNECT_BLOCK_MS;   // boot: block stale Steam 0x82 until link stable
  // NO fabricated stop burst. USB capture proves Steam only ever sends 0x82 [01 01 f7] pulses -- never a
  // zero-gain [01 01 00] "stop". Injecting our invented stop frame (which the real puck never sends) at
  // boot/connect is unfaithful and, given the controller's "haptic script already active -> ignore" behavior,
  // a likelier cause of the connect-time buzz than a cure (the buzz happened WITH this burst in place). Be a
  // pure pass-through of Steam's haptics, like the real puck.
  g_hapticStop = 0;
}
// Arm the post-(re)connect haptic block + schedule the clearing re-init. Called from rf_link the moment a
// controller reply arrives after a gap (the reliable reconnect signal), and as a backup on hapticTask's
// link-up edge. Idempotent -- safe to call repeatedly.
void hapticOnReconnect(){
  g_hapticBlockUntil = millis() + HAPTIC_RECONNECT_BLOCK_MS;   // no haptics relayed for the next 3s
  g_haptic82On = false;
  hapticCancelPendingOn();                                     // drop any haptic ON queued before the link came up
  // Be PROACTIVE: start re-initing the haptic engine right after the link is up and repeat across the whole
  // settle window, instead of waiting for the block to end. The brief connect buzz engages early (during the
  // block, controller-internal), so a single late shot misses it -- frequent early resets cut it short or
  // preempt it. The re-init is settings only (no haptic play), and Steam haptics are blocked the whole time,
  // so repeating it can't itself buzz.
  g_reinitAt = millis() + 200u;                               // first reset ~200ms after (re)connect
  g_reinitLeft = HAPTIC_REINIT_SHOTS;                          // then every HAPTIC_REINIT_GAP_MS across the window
  uint8_t mk=2; hapLogAdd(0xFD, 0xEE, &mk, 1);                 // capture marker: RECONNECT detected (block+reinit armed)
}
void hapticTask(){
  static bool wasHapticLinkUp=false;
  bool up=hapticLinkUp();
  // Link-edge markers are diagnostic only now -- the block/re-init is armed reliably from rf_link
  // (hapticOnReconnect) on the first reply after a gap, which fires even when this 300ms edge doesn't.
  if(up && !wasHapticLinkUp){ uint8_t mk=1; hapLogAdd(0xFD,0xEE,&mk,1); hapticOnReconnect(); }
  if(!up && wasHapticLinkUp){ uint8_t mk=0; hapLogAdd(0xFD,0xEE,&mk,1); }
  wasHapticLinkUp=up;
  if(g_reinitAt && up && (int32_t)(millis()-g_reinitAt) >= 0){   // proactive haptic re-init across the connect window
    hapticReinit();
    g_reinitAt = (g_reinitLeft && --g_reinitLeft) ? (millis()+HAPTIC_REINIT_GAP_MS) : 0;
  }
  // Steam-mode: host went quiet -> mark the 0x82 stream inactive. Do NOT synthesize a stop: trackpad haptics
  // are one-shot pulses, so firing a 0x82-zero ~HAPTIC_QUIET_MS after a swipe ends is the extra end-of-movement
  // click the real puck doesn't make. Steam forwards its own stop for any sustained haptic.
  if (!g_xbox && g_haptic82On && millis()-g_haptic82Ms > HAPTIC_QUIET_MS) g_haptic82On=false;
  // Haptic activity has gone idle for a while -> fire one re-init to clear any latch it left behind (the buzz
  // that engages during/after use and won't self-clear, incl. after a mode switch). Fires only after a quiet
  // gap, so it never interrupts active haptics; the brightness-less re-init is silent (settings, no play, no
  // LED). Runs in ALL modes (the controller-side latch is mode-independent).
  if (g_hapClearArmed && (millis()-g_haptic82Ms) > HAPTIC_CLEAR_IDLE_MS){ g_hapClearArmed=false; hapticReinit(); }
}
