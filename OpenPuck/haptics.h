// haptics.h -- host -> controller haptic relay + the watchdogs that stop a stuck buzz.
//
// Steam (and legacy XInput rumble) send OUTPUT reports the real dongle forwards to the controller as a SET
// sub-TLV inside the E3 poll. We do the same: handleSet()/the console queue a relay into g_relayBuf, and
// rfConnFlushRelay() emits it inside the poll cadence (rf_link.cpp), never at raw loop rate.
//
// The controller's haptic LATCHES until told to stop. If the host's stop is lost over RF (or the link drops
// mid-buzz) the actuator whines forever -- so we actively send 0x82-zero stop bursts on reconnect/silence and
// gate relays on link + slot + lizard state. The g_hapLog ring captures recent OUTPUT reports for the 'H' dump.
#pragma once
#include <stdint.h>

// after this much host silence, consider the current 0x82 haptic stream inactive
#define HAPTIC_QUIET_MS    300u
// drop stale host 0x82 after RF reconnect (Steam floods on attach)
#define HAPTIC_RECONNECT_BLOCK_MS 1500u
// 0x82-zero relays per stop event (sent at poll cadence -- not loop rate)
#define HAPTIC_STOP_BURST  4u

// ---- relay queue (written by puck_hid.cpp, mode_xinput.cpp, serial_console.cpp; drained by rf_link.cpp) ----
extern volatile bool    g_relayPend;
extern uint8_t          g_relayBuf[24];
extern volatile uint8_t g_relayN;
extern uint8_t          g_relayOp;   // relay frame opcode (E3 poll)
extern uint8_t          g_relaySub;  // relay sub-TLV type byte = SET
extern volatile uint8_t g_testHaptic;// 't<n>' injects n test haptics for the buzz hunt
extern volatile uint8_t g_hapticStop;// pending haptic-STOP frames to relay (kill a latched whine)
extern unsigned long    g_hapticBlockUntil;

void hapLogAdd(uint8_t slot, uint8_t rid, const uint8_t* b, uint16_t n);
void hapticDumpLog();   // 'H' console dump of the recent OUTPUT-report history

bool hapticLinkUp();
bool haptic82Blocked();
bool hapticRelaySlotOk(int slot);
void haptic82HostReport(const uint8_t* p, uint16_t n);

// queue + flush the pending host/test/stop relay inside the poll cadence (called from rf_link)
void rfConnQueueHapticRelay();
void rfConnFlushRelay(uint8_t ch);

void hapticInit();   // boot reset: clear relay/active flags, arm the reconnect block + initial stop burst
void hapticTask();   // per-loop upkeep: reconnect-block edge handling + steam 0x82 quiet timeout
