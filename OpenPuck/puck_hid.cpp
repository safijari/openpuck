#include "puck_hid.h"
#include "bonds.h"
#include "config.h"
#include "identity.h"
#include "haptics.h"
#include "rf_link.h"
#include "triton.h"
#include "mode_lizard.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

uint8_t g_fwdNewOnly = 1;
SteamPuckController g_steamPuck;

// ---- cloned puck HID report descriptor (verbatim): mouse(0x40)+keyboard(0x41)+vendor(FF00) with the 63-byte
//      FEATURE command reports on report id 1/2. Each of the 4 interfaces uses this. ----
static const uint8_t PUCK_HID_DESC[] = {
  0x05,0x01,0x09,0x02,0xA1,0x01,0x85,0x40,0x09,0x01,0xA1,0x00,0x05,0x09,0x19,0x01,
  0x29,0x02,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x02,0x81,0x02,0x75,0x06,0x95,0x01,
  0x81,0x01,0x05,0x01,0x09,0x30,0x09,0x31,0x15,0x81,0x25,0x7F,0x75,0x08,0x95,0x02,
  0x81,0x06,0x95,0x01,0x09,0x38,0x81,0x06,0x05,0x0C,0x0A,0x38,0x02,0x95,0x01,0x81,
  0x06,0xC0,0xC0,0x05,0x01,0x09,0x06,0xA1,0x01,0x85,0x41,0x05,0x07,0x19,0xE0,0x29,
  0xE7,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x08,0x81,0x02,0x81,0x01,0x19,0x00,0x29,
  0x65,0x15,0x00,0x25,0x65,0x75,0x08,0x95,0x06,0x81,0x00,0xC0,0x06,0x00,0xFF,0x09,
  0x01,0xA1,0x01,0x85,0x42,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x35,0x09,0x42,
  0x81,0x02,0x85,0x44,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x05,0x09,0x44,0x81,
  0x02,0x85,0x79,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x01,0x09,0x79,0x81,0x02,
  0x85,0x43,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x0E,0x09,0x43,0x81,0x02,0x85,
  0x7B,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x0C,0x09,0x7B,0x81,0x02,0x85,0x45,
  0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x2D,0x09,0x45,0x81,0x02,0x85,0x80,0x15,
  0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x09,0x09,0x80,0x91,0x02,0x85,0x81,0x15,0x00,
  0x26,0xFF,0x00,0x75,0x08,0x95,0x07,0x09,0x81,0x91,0x02,0x85,0x82,0x15,0x00,0x26,
  0xFF,0x00,0x75,0x08,0x95,0x03,0x09,0x82,0x91,0x02,0x85,0x83,0x15,0x00,0x26,0xFF,
  0x00,0x75,0x08,0x95,0x09,0x09,0x83,0x91,0x02,0x85,0x84,0x15,0x00,0x26,0xFF,0x00,
  0x75,0x08,0x95,0x08,0x09,0x84,0x91,0x02,0x85,0x85,0x15,0x00,0x26,0xFF,0x00,0x75,
  0x08,0x95,0x03,0x09,0x85,0x91,0x02,0x85,0x86,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,
  0x95,0x03,0x09,0x86,0x91,0x02,0x85,0x87,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,
  0x3F,0x09,0x87,0x91,0x02,0x85,0x89,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x3F,
  0x09,0x89,0x91,0x02,0x85,0x88,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x3F,0x09,
  0x88,0x91,0x02,0x85,0x01,0x95,0x3F,0x09,0x01,0xB1,0x02,0x85,0x02,0x95,0x3F,0x09,
  0x01,0xB1,0x02,0xC0
};

static Adafruit_USBD_HID hid[NSLOT];

// ===================== seamless LIZARD decision =====================
// Steam, while running, re-sends settings report 0x87 (lizard-off) every ~3s as a heartbeat (captured on HW),
// and ANY OUTPUT report likewise stamps g_steamAliveMs. When the heartbeat stops we fall back to lizard, so the
// controller drives desktop keyboard+mouse whenever Steam isn't running. MODE_LIZARD forces lizard always.
static unsigned long g_steamAliveMs = 0;   // millis of last Steam OUTPUT/settings write; 0 at boot => lizard until Steam appears
// Fall back to lizard this long after Steam's ~3s settings heartbeat stops. Keep >2x the cadence (7s): the
// haptic relay is gated by !lizardActive(), so a shorter window lets a single jittered/delayed heartbeat flip
// lizard mid-session while Steam is still running -> a haptic that arrives in that window gets gated out
// (dropped/missing haptic). 7s tolerates one missed beat. (This matches main; do not shorten without a lizard
// trigger that doesn't depend on the heartbeat timeout.)
#define LIZARD_WD_MS 7000u
static bool g_autoLizard = true;            // master switch; false => Steam mode always forwards 0x45
// Single source of truth, shared by the USB input path AND the haptic relay gate: if we ever relay a 0x82 to
// the controller while presenting lizard (Steam isn't reading 0x45 back), Steam loops the same haptic -> buzz.
static inline bool steamDrivingGamepad(){ return g_steamAliveMs && (millis()-g_steamAliveMs < LIZARD_WD_MS); }
static inline bool lizardActive(){ return modeIsPuck(g_usbMode) && (g_usbMode==MODE_LIZARD || (g_autoLizard && !steamDrivingGamepad())); }
// Right after the host resumes from suspend, MUTE input forwarding briefly. Otherwise the controller's input
// in that instant (a trackpad click/trigger, or residual button state from the wake gesture) gets forwarded
// as a real click/keypress into the just-woken desktop -- which was activating the highlighted Start tile
// (Edge) and closing the Start menu on every wake. Set when task() sees the suspended->active transition.
static unsigned long g_resumeMs = 0;
#define POST_RESUME_MUTE_MS 1500u

// ===================== puck feature command channel =====================
// `slot` is the interface index (interface N == bond slot N).
static void handleSet(int slot, uint8_t rid, hid_report_type_t type, uint8_t const *b, uint16_t n) {
  if (type == HID_REPORT_TYPE_OUTPUT) {   // Steam OUTPUT reports 0x80-0x89. The haptic/actuator reports (0x80-0x86)
    // are relayed to the controller, and ONLY when they arrive on the CONNECTED slot's interface. We have one
    // controller but expose 4 puck slots, and a report aimed at a DIFFERENT slot made the controller buzz at
    // random -> that is what the slot gate below fixes. (This used to also clamp to 0x82-ONLY, which silently
    // dropped the ping / grip / test haptics: those ride other report IDs such as 0x85/0x86, so they never
    // reached the controller.) The 63-byte settings/config reports 0x87/0x88/0x89 are NOT haptics and are not
    // pushed here -- 0x87 (lizard-off/settings) reaches the controller through the feature-0x01 passthrough path.
    if (rid >= 0x80 && rid <= 0x89) {
      hapLogAdd((uint8_t)slot, rid, b, n);   // capture ALL OUTPUT reports (even un-relayed) for the 'H' dump
      g_steamAliveMs = millis();   // ANY Steam OUTPUT report (not just the 0x87 heartbeat) means Steam is present and
                                   // driving -> leave lizard for gamepad NOW, so a haptic that arrives before the
                                   // first 0x87 doesn't get relayed while we're still presenting lizard (-> buzz loop).
    }
    // Post-resume mute also gates haptics: while onReport45 is muted Steam reads NO 0x45 back, which is the
    // exact condition under which Steam loops the same haptic command (-> connect/wake buzz loop). Same logic
    // as the lizard gate; the mute is short (1.5s), so a dropped wake chime is the cheap end of the trade.
    bool muted = g_resumeMs && millis()-g_resumeMs < POST_RESUME_MUTE_MS;
    if (rid >= 0x80 && rid <= 0x86 && n >= 1 && hapticRelaySlotOk(slot) && !lizardActive() && !muted) {  // wrap as a SET sub-TLV like the report-01 path
      if (!haptic82Blocked()) {
        relayEnqueue(rid, b, (uint8_t)(n > RELAY_MAXP ? RELAY_MAXP : n));
        // Track on/off from what was actually RELAYED (= the controller's believed state), not from every host
        // report: a BLOCKED stop must leave "on" set (the controller may be latched -> reconnect stop-burst),
        // and a blocked ON must not set it (nothing reached the controller -> no spurious burst clicks).
        if (rid == 0x82) haptic82HostReport(b, n);
      }
    }
    if (Serial.availableForWrite() > 80) {                      // log so we can see what Steam actually sends (e.g. glide haptics)
      Serial.printf("# OUT if%d rid=%02X n=%u:", slot, rid, n);
      for (uint16_t i = 0; i < n && i < 14; i++) Serial.printf(" %02X", b[i]);
      Serial.println();
    }
    return;
  }
  if (type != HID_REPORT_TYPE_FEATURE || n < 1) return;
  Slot &S = g_slot[slot];
  uint8_t cmd = b[0], len = (n > 1) ? b[1] : 0;
  const uint8_t *pl = b + 2; uint16_t pln = (n >= 2) ? n - 2 : 0;
  if (cmd >= 0x80 && cmd <= 0x89) g_steamAliveMs = millis();   // any Steam settings/haptic/LED report (incl. the 0x87 lizard-off heartbeat) -> Steam present, forward gamepad, suppress auto-lizard
  if (rid == 1 && n >= 2) {   // report 0x01 = raw passthrough -> queue for RF relay to the controller
    hapLogAdd((uint8_t)slot, cmd, b, n);   // capture EVERY relayed feature-1 command for the WebUSB capture view
                                           // (haptics, LED SET_LED_COLOR, 0x87 settings, 0x9F power-off). The log
                                           // shows cmd as "rid"; bytes start [cmd][len]...  Was gated to 0x80-0x89,
                                           // which hid exactly the LED/power-off frames we now need to see.
    bool haptic82 = (cmd == 0x82 && len <= pln);
    bool muted = g_resumeMs && millis()-g_resumeMs < POST_RESUME_MUTE_MS;   // see the OUTPUT path: no haptics while Steam can't read 0x45 back
    bool relayOk = hapticRelaySlotOk(slot) && !(haptic82 && (lizardActive() || muted));  // never push haptics to the controller while presenting lizard (Steam isn't reading 0x45 -> would buzz-loop)
    if (relayOk && (!haptic82 || !haptic82Blocked())) {
      // Relay the DECLARED length (up to the 60B RF frame ceiling), not a truncation: Steam's multi-register
      // 0x87 settings blocks (LED brightness) and calibration writes are longer than the old 18B cap, and the
      // chopped frames were why those settings never landed on the controller.
      uint8_t rl = (len <= pln) ? len : (uint8_t)pln;
      if (len > RELAY_MAXP && Serial.availableForWrite() > 60)
        Serial.printf("# RELAY TRUNC cmd=%02X len=%u>%u\n", cmd, len, (unsigned)RELAY_MAXP);
      relayEnqueue(cmd, pl, rl);
      if (haptic82) haptic82HostReport(pl, len);  // track from RELAYED frames only (see the OUTPUT path)
    }
  }
  if (Serial.availableForWrite() > 80) {   // log host feature writes (non-blocking)
    Serial.printf("# SET if%d rid=%02X cmd=%02X len=%u:", slot, rid, cmd, len);
    for (uint16_t i = 0; i < n && i < 14; i++) Serial.printf(" %02X", b[i]);
    Serial.println();
  }
  memset(S.resp, 0, sizeof S.resp); S.resp_len = 0;
  switch (cmd) {
    case 0x83:
      S.resp[0] = 0x83; S.resp[1] = sizeof ATTR83; memcpy(S.resp + 2, ATTR83, sizeof ATTR83); S.resp_len = 63; break;
    case 0xAE: {
      uint8_t idx = pln > 0 ? pl[0] : 1; const char *s = (idx == 0) ? g_board : (idx == 1) ? g_unit : "NA";
      S.resp[0] = 0xAE; S.resp[1] = 0x14; S.resp[2] = idx; memset(S.resp + 3, 0, 60); memcpy(S.resp + 3, s, strlen(s)); S.resp_len = 63; break; }
    case 0xB4:    // connection/version state per slot: value 0x02 = controller connected, 0x01 = not
      S.resp[0] = 0xB4; S.resp[1] = 0x01;
      S.resp[2] = (slot == g_connSlot && !g_xbox && (millis() - g_connReplyMs < 500)) ? 0x02 : 0x01;
      S.resp_len = 63; break;
    case 0xAD:
      g_pairing = (pln > 0 && pl[0] != 0); Serial.printf("# pairing %s\n", g_pairing ? "ON" : "off");
      S.resp[0] = 0xAD; S.resp[1] = 0; S.resp_len = 63; break;
    case 0xA2:                                   // write/clear THIS interface's slot
      if (len >= 24 && pln >= 24) {
        if (recEmpty(pl)) { S.used = false; memset(S.rec, 0, 24); }
        else { memcpy(S.rec, pl, 24); S.used = true; }
        g_dirty = true; Serial.printf("# slot %d %s\n", slot, recEmpty(pl) ? "cleared" : "bonded");
      }
      S.resp[0] = 0xA2; S.resp[1] = 0; S.resp_len = 63; break;
    case 0xA3:                                   // read THIS interface's slot
      S.resp[0] = 0xA3; S.resp[1] = 0x18; memset(S.resp + 2, 0, 24);
      if (S.used) memcpy(S.resp + 2, S.rec, 24); S.resp_len = 63; break;
    default:
      S.resp[0] = cmd; S.resp[1] = len; if (pln) memcpy(S.resp + 2, pl, pln > 60 ? 60 : pln); S.resp_len = 63; break;
  }
}
static uint16_t handleGet(int slot, uint8_t rid, hid_report_type_t type, uint8_t *buf, uint16_t reqlen) {
  (void)rid;
  if (type != HID_REPORT_TYPE_FEATURE) return 0;
  Slot &S = g_slot[slot];
  uint16_t n = S.resp_len ? S.resp_len : 63; if (n > reqlen) n = reqlen;
  memcpy(buf, S.resp, n); return n;
}

// one callback pair per interface (the Adafruit core routes by interface to the matching object)
#define SLOTCB(N) \
  static void setcb##N(uint8_t r, hid_report_type_t t, uint8_t const *b, uint16_t n) { handleSet(N, r, t, b, n); } \
  static uint16_t getcb##N(uint8_t r, hid_report_type_t t, uint8_t *bf, uint16_t rl) { return handleGet(N, r, t, bf, rl); }
SLOTCB(0) SLOTCB(1) SLOTCB(2) SLOTCB(3)
typedef uint16_t (*getcb_t)(uint8_t, hid_report_type_t, uint8_t *, uint16_t);
typedef void (*setcb_t)(uint8_t, hid_report_type_t, uint8_t const *, uint16_t);
static getcb_t GETCB[NSLOT] = { getcb0, getcb1, getcb2, getcb3 };
static setcb_t SETCB[NSLOT] = { setcb0, setcb1, setcb2, setcb3 };

// ===================== IController =====================
void SteamPuckController::begin(){
  USBDevice.setID(0x28DE, 0x1304);
  // Distinct bcdDevice so Windows keys a FRESH usbflags entry (cache is VID:PID:bcdDevice) and actually runs
  // MS OS 2.0 / WinUSB binding for the WebUSB vendor interface -- instead of reusing a stale "no WinUSB" entry
  // tied to the real Steam Controller (28DE:1304), which has no WebUSB interface. The normal (wake-mouse) and
  // one-shot debug (CDC) boots present DIFFERENT interface sets, so they use DIFFERENT bcdDevice values --
  // otherwise Windows would serve one's cached descriptor for the other when you reboot between them.
  USBDevice.setDeviceVersion(g_debugCdcThisBoot ? 0x0212 : 0x0211);
  USBDevice.setManufacturerDescriptor("Valve Software");
  USBDevice.setProductDescriptor("Steam Controller Puck");
  for (int i = 0; i < NSLOT; i++) {
    hid[i].setReportDescriptor(PUCK_HID_DESC, sizeof PUCK_HID_DESC);
    hid[i].setReportCallback(GETCB[i], SETCB[i]);
    hid[i].setPollInterval(1);       // 1ms USB poll (was default 10ms = 100/s cap -> choppy)
    hid[i].begin();
  }
}

// Forward the controller's report 0x45 to Steam, or drive lizard kb/mouse when Steam is closed. This is a
// PURELY USB-SIDE decision -- it changes nothing about the RF poll or the host->controller relay.
void SteamPuckController::onReport45(const uint8_t* rep, bool fresh, uint8_t bodyTlen){
  // Host asleep -> forward NOTHING. While suspended, every sendReport attempt (a lizard-mouse report from a
  // trackpad graze, a 0x45 forward) can translate into a host wake -- which made the PC wake on any controller
  // movement and nearly impossible to keep asleep. Waking is an explicit gesture only (Steam-button short press
  // / controller connect), handled in rf_link.cpp via the device-level USB resume signal.
  if (USBDevice.suspended()) return;
  // Just woke? Hold off forwarding for a beat so the wake gesture's residual controller input doesn't click /
  // type into the freshly-woken desktop (it was launching Edge from the Start menu on every wake).
  if (g_resumeMs && millis()-g_resumeMs < POST_RESUME_MUTE_MS) return;
  if (g_connSlot < 0 || g_connSlot >= NSLOT) return;
  if (lizardActive()){
    rfLizard(rep, &hid[g_connSlot], &hid[g_connSlot], 0x40, 0x41);
  } else {
    uint8_t blen = bodyTlen - 1; if(blen>45)blen=45;   // body after the 0x45 id byte
    // forward the puck's raw pad coords untouched -- Steam does its own interpolation/smoothing. Forward only
    // FRESH reports (the real puck dedupes -> Steam gets a clean unique stream; sending stale repeats makes
    // Steam's velocity/smoothing stair-step). g_fwdNewOnly toggles for A/B.
    if((fresh || !g_fwdNewOnly) && g_slot[g_connSlot].used && hid[g_connSlot].ready())
      hid[g_connSlot].sendReport(0x45, rep+1, blen);   // Steam/SDL: input report -> "connected"
  }
}

// ---- wake nudge: a bare USB resume signal is NOT enough to wake some hosts (Windows in particular) -- they
// only wake when actual mouse/keyboard input follows. So on a deliberate wake gesture we play a HARMLESS mouse
// JIGGLE (move a few px right, then back -- NET ZERO cursor, NO button): real mouse activity wakes the host,
// but it clicks nothing and doesn't close/activate anything (an open Start menu stays open). Earlier versions
// also sent a click / space / Ctrl; those either activated the focused item or weren't needed. Queued by
// wakeEvent() (rf_link, on a Steam short press / controller connect while suspended); reports can't cross a
// suspended bus, so it's delivered once the bus has resumed.
static uint8_t       g_nudgeStep = 0;      // 0=idle; 1=jiggle+, 2=jiggle-
static unsigned long g_nudgeMs = 0;
#define NUDGE_JIGGLE_PX 10
void SteamPuckController::wakeEvent(){
  g_nudgeStep = 1; g_nudgeMs = millis();
}
static void wakeNudgeTask(){
  if(!g_nudgeStep) return;
  if(millis()-g_nudgeMs > 5000){ g_nudgeStep=0; return; }            // bus never resumed -> drop the nudge
  if(USBDevice.suspended()) return;                                   // wait for resume; reports can't cross a suspended bus
  if(g_connSlot<0 || g_connSlot>=NSLOT || !hid[g_connSlot].ready()) return;
  static unsigned long stepMs=0;
  if(millis()-stepMs < 15) return;                                    // pace the edges
  stepMs=millis();
  hid_mouse_report_t m; m.buttons=0; m.x=(g_nudgeStep==1)?NUDGE_JIGGLE_PX:-NUDGE_JIGGLE_PX; m.y=0; m.wheel=0; m.pan=0;
  hid[g_connSlot].sendReport(0x40,&m,sizeof m);                       // jiggle right, then back
  g_nudgeStep = (g_nudgeStep>=2) ? 0 : (uint8_t)(g_nudgeStep+1);
}

// USB connection presentation (like the real dongle): report 0x79 = connection state (01=disc, 02=conn),
// edge-triggered, + periodic 0x7B status. Live-captured: this is what Steam reads to mark the controller
// connected. Without it Steam shows disconnected even though 0x45 input is streaming.
void SteamPuckController::task(){
  wakeNudgeTask();
  { static bool wasSusp=false; bool susp=USBDevice.suspended();   // stamp the suspended->active edge for the post-resume mute
    if(wasSusp && !susp) g_resumeMs=millis();
    wasSusp=susp; }
  if (USBDevice.suspended()) return;   // no periodic 0x79/0x7B while the host sleeps -- those sends can wake it too
  static bool usbConn=false; static unsigned long last79=0, last7B=0, connEdgeMs=0;
  bool conn = (g_connSlot>=0 && millis()-g_connReplyMs < 300);
  if (g_connSlot>=0 && g_connSlot<NSLOT && hid[g_connSlot].ready()){
    // 0x79 connection state: on edge, then repeated every 750ms ONLY until Steam reacts (its first OUTPUT/
    // settings write after the edge -- g_steamAliveMs). The real puck sends 0x79 ONCE, edge-triggered; the old
    // unconditional forever-resend could re-trigger Steam's connect handling (connect chime) every 750ms
    // before Steam started consuming 0x45 -> the connect-time loop of haptic buzzes that only stopped on the
    // first real input (the Steam button). Resending until acked still covers "Steam missed the edge".
    bool steamAcked = g_steamAliveMs && (int32_t)(g_steamAliveMs - connEdgeMs) >= 0;
    if (conn!=usbConn || (conn && !steamAcked && millis()-last79>=750)){
      if (conn && !usbConn) connEdgeMs = millis();
      uint8_t st=conn?0x02:0x01; hid[g_connSlot].sendReport(0x79,&st,1); usbConn=conn; last79=millis();
    } else if (conn && millis()-last7B>=2000){
      // 0x7B status, live-captured template. Byte 8 is the controller->puck signal strength as signed dBm
      // (capture showed 0xDD = -35; replaying it verbatim is why Steam pinned -35dBm forever) -- patch in the
      // smoothed RSSI the radio now samples on each controller reply (rf_link). 0 = no sample yet -> keep the
      // capture value rather than report garbage.
      //
      // CALIBRATION: our raw RSSISAMPLE reads ~RSSI_DBM_OFFSET dB lower than the real Valve puck at the same
      // distance (the Pro Micro's PCB-trace antenna vs Valve's tuned front-end). The 2Mbit ESB link has ~55dB
      // of margin, so a -75dBm reading still works across a house -- but Steam's bar maps raw dBm to "weak"
      // long before that. Adding the offset lines our close-range value up with the puck's captured -35 so the
      // bar tracks usable range instead of antenna gain. Clamp keeps it in a sane window (no wrap/garbage).
      uint8_t s7b[12]={0xF7,0x01,0x89,0x00,0x00,0x00,0x03,0x00,0xDD,0x00,0x3A,0x02};
      if (g_linkRssi){
        int mag = (int)g_linkRssi - RSSI_DBM_OFFSET;
        if (mag < 25) mag = 25; else if (mag > 95) mag = 95;
        s7b[8]=(uint8_t)(0u-(uint8_t)mag);
      }
      hid[g_connSlot].sendReport(0x7B,s7b,12); last7B=millis();
    }
  } else if(!conn) usbConn=false;
}
