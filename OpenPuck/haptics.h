// haptics.h -- host -> controller relay queue (haptics + settings) + the watchdogs that stop a stuck buzz.
//
// Steam and translated host rumble send OUTPUT/feature reports the real dongle forwards to the controller as
// a SET sub-TLV inside the E3 poll. We do the same: handleSet()/the console enqueue with relayEnqueue(), and
// rfConnFlushRelay() emits ONE entry per poll cycle (rf_link.cpp), never at raw loop rate.
//
// The queue is a small ring, NOT a single buffer: producers run in USB-ISR context while the consumer may be
// mid-flush in loop context, and Steam writes settings/calibration as bursts of back-to-back feature reports.
// A single buffer LOSES reports arriving before the previous flush and can be TORN mid-copy by the ISR.
// Entries carry up to RELAY_MAXP payload bytes (the RF frame fits 60: [E3][len][05][rid] + payload <= MAXLEN
// 64); a smaller cap chops multi-register 0x87 settings blocks (LED brightness) and calibration writes.
//
// The controller's haptic LATCHES until told to stop. If the host's stop is lost over RF (or the link drops
// mid-buzz) the actuator whines forever -- so we send 0x82-zero stop bursts on reconnect, but ONLY when a
// haptic was actually active when the link dropped: every extra 0x82 frame is an audible click on the
// controller (captured), so an unconditional burst on each link-up edge is itself a buzz source on a flapping
// link. The g_hapLog ring captures recent OUTPUT reports for the 'H' dump.
#pragma once
#include <stdint.h>
#include "config.h" // OPK_LOG

// after this much host silence, consider the current 0x82 haptic stream inactive
#define HAPTIC_QUIET_MS 300u
// Block ALL haptic relays for this long after a (re)connect: a just-powered-on controller's haptic engine
// isn't ready, and feeding it haptics in that window leaves it in a degraded/latched state (slow/stuck/missing
// haptics until a re-init). 3s gives it time to initialize.
#define HAPTIC_RECONNECT_BLOCK_MS 3000u
// 0x82-zero relays per stop event (sent at poll cadence -- not loop rate)
#define HAPTIC_STOP_BURST 4u
// max relayed payload bytes per entry: RF frame = [E3][len][05][rid][payload] and MAXLEN=64 -> 60
#define RELAY_MAXP 60u
// Proactive post-(re)connect haptic re-init: this many shots, this far apart, starting ~200ms after the link
// comes up -- covers ~0.2s..3s so the brief controller-side connect buzz gets reset before it can sustain.
#define HAPTIC_REINIT_SHOTS 8u
#define HAPTIC_REINIT_GAP_MS 350u
// After haptic activity, if it's been idle this long, fire one clear-re-init -- kills a latch that engaged
// during use (a buzz that starts seconds after connect and won't self-clear). Long enough not to fire between
// rapid in-game haptics; short enough to clear a stuck buzz soon after the user pauses.
#define HAPTIC_CLEAR_IDLE_MS 1200u
// Controller power-off: hapticSendShutdown() relays Steam's confirmed "turn off controller" command (feature-0x01
// cmd 0x9F, payload "off!" -- captured from the real puck). Sent as a small burst because the RF relay is NO-ACK.
#define HAPTIC_SHUTDOWN_SHOTS 3u

// ---- relay queue (written by puck_hid.cpp, mode_*.cpp, serial_console.cpp; drained by rf_link.cpp) ----
// Enqueue one host->controller report: rid = report/command id, payload = the bytes AFTER [cmd][len] (what
// goes on the air). ISR-safe (brief PRIMASK critical section). Returns false (dropped) when the ring is full.
bool relayEnqueue(uint8_t rid, const uint8_t *payload, uint8_t plen);

// anything still queued (xinput uses it to pace rumble re-queues)
bool relayPending();
extern uint8_t g_relayOp; // relay frame opcode (E3 poll)
extern uint8_t g_relaySub; // relay sub-TLV type byte = SET
extern volatile uint8_t g_testHaptic; // 't<n>' injects n test haptics
// pending haptic-STOP frames to relay (kill a latched whine)
extern volatile uint8_t g_hapticStop;
extern unsigned long g_hapticBlockUntil;

// relay the controller power-off (0x9F "off!"), burst x3 (Steam 0x9F / host-suspend / test button)
void hapticSendShutdown();

// ---- diagnostic capture (compiled in only when OPK_LOG): a ring of recent host->controller commands +
//      link/TX markers, dumped over WebUSB. No-ops in a production build so call sites vanish. ----
#if OPK_LOG
void hapLogAdd(uint8_t slot, uint8_t rid, const uint8_t *b, uint16_t n);
void hapticDumpLog(); // 'H' console dump of the recent OUTPUT-report history
// WebUSB capture drain: resetDrain(true) rewinds to the OLDEST entry (dump the whole ring from boot);
// resetDrain(false) starts at "now" (live only). pull yields each entry once, oldest->newest, skipping empties.
void hapLogResetDrain(bool fromBoot);
bool hapLogPull(uint32_t *logMs, uint8_t *slot, uint8_t *rid, uint8_t *n,
		uint8_t bytes16[16]);
#else
static inline void hapLogAdd(uint8_t, uint8_t, const uint8_t *, uint16_t)
{
}
static inline void hapticDumpLog()
{
}
static inline void hapLogResetDrain(bool)
{
}
static inline bool hapLogPull(uint32_t *, uint8_t *, uint8_t *, uint8_t *,
			      uint8_t *)
{
	return false;
}
#endif

bool hapticLinkUp();
bool haptic82Blocked();
bool hapticRelaySlotOk(int slot);
void haptic82HostReport(const uint8_t *p, uint16_t n);
bool hapticSteamRumble(
	uint16_t lowFreq,
	uint16_t highFreq); // queue Steam/Triton output report 0x80 rumble

// queue + flush the pending host/test/stop relay inside the poll cadence (called from rf_link).
// rfConnFlushRelay's s1 must carry a PID distinct from the GET poll that follows it (rf_link cycles the shared
// PID counter), so the controller doesn't dedup the GET as a retransmit of the relay.
void rfConnQueueHapticRelay();
void rfConnFlushRelay(uint8_t ch, uint8_t s1);

// boot reset: clear relay/active flags, arm the reconnect block
void hapticInit();

// per-loop upkeep: link-edge markers + steam 0x82 quiet timeout + fires the scheduled re-init
void hapticTask();

// replay Steam's haptic-subsystem re-init to the controller -> clears a latched/stuck buzz
void hapticReinit();
// Called from rf_link the instant a controller (re)connect is detected (an F-reply after a gap): blocks haptic
// relays for HAPTIC_RECONNECT_BLOCK_MS and schedules a re-init just after, to keep the freshly-booted
// controller out of the degraded/latched haptic state. Reliable -- independent of hapticTask's link heuristic.
void hapticOnReconnect();
