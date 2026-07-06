// haptics.h -- host -> controller relay queue (haptics + settings) + the watchdogs that stop a stuck buzz.
//
// Steam and translated host rumble send OUTPUT/feature reports the real dongle forwards to the controller as
// a SET sub-TLV inside the E3 poll. We do the same: handleSet()/the console enqueue with relayEnqueue(), and
// rfConnFlushRelay() emits ONE entry per poll cycle (rf_link.cpp), never at raw loop rate.
//
// One ring per bond slot: the consumer (rfConnFlushRelay) only drains the current slot's queue, so commands
// addressed to other slots are never consumed out of turn. ISR producers write under PRIMASK; the consumer
// runs in loop context and never races the same slot's head pointer.
//
// The controller's haptic LATCHES until told to stop. If the host's stop is lost over RF (or the link drops
// mid-buzz) the actuator whines forever -- so we send 0x82-zero stop bursts on reconnect, but ONLY when a
// haptic was actually active when the link dropped: every extra 0x82 frame is an audible click on the
// controller (captured), so an unconditional burst on each link-up edge is itself a buzz source on a flapping
// link. The g_hapLog ring captures recent OUTPUT reports for the 'H' dump.
#pragma once
#include <stdint.h>
#include "config.h" // OPK_LOG
#include "bonds.h" // NSLOT

// after this much host silence, consider the current 0x82 haptic stream inactive
#define HAPTIC_QUIET_MS 300u
// Post-(re)connect haptic block default. While armed, ALL Steam haptic relays to that slot are dropped: a
// just-powered-on controller's haptic engine isn't ready, and feeding it haptics in that window leaves it in a
// degraded/latched state (slow/stuck/missing haptics until a re-init). Runtime-adjustable (g_hapticBlockMs) and
// toggleable (g_hapticBlockOn) from the WebUSB panel; this is the boot default.
#define HAPTIC_BLOCK_MS_DEFAULT 10000u
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
// Enqueue one host->controller report. `slot` = bond slot (0..NSLOT-1) or 0xFF to broadcast to every
// connected controller (used by hapticSendShutdown / hapticReinit / test haptics). ISR-safe (PRIMASK).
bool relayEnqueue(uint8_t rid, const uint8_t *payload, uint8_t plen,
		  uint8_t slot = 0xFF);

// id9=0 hold (MODE_STEAM only): land the controller's SET_SETTINGS index 9 (digital-mappings/lizard-active)
// at 0, once per LIZKEEP_MS per connected slot, like the real puck. This holds the controller's autonomous
// mapping/haptic engine OFF so it can't latch into the deep-inside buzz seen after repeated reconnects
// (capture-for-haptics.txt: the buzz is controller-internal; OpenPuck relays no haptics in that state). It
// also disables the controller's autonomous touchpad ticks (id9 gates the whole pad layer) -- fine in Steam
// mode (Steam owns haptics), so it is scoped to MODE_STEAM; pure MODE_LIZARD is left alone to keep its ticks.
#define LIZKEEP_MS 2000u
extern uint8_t
	g_lizKeep; // 1 = hold on (default, persisted); console 'u' toggles for A/B
// Experiment: land ALL relayed 0x87 SET_SETTINGS verbatim (real-puck relay) instead of the discard-whitelist.
// Default 0 (whitelist). Console "L87" toggles; persisted. See haptics.cpp for the buzz hypothesis it tests.
extern uint8_t g_landAll87;
// Land Steam's amp/haptic-config 0x87 (regs 0x18/0x2E/0x34/0x35, not gyro 0x30) so haptics play as clean
// ticks not a default-amp buzz. On by default; console "AMP" toggles.
extern uint8_t g_landAmp;
// Master enable for the puck->controller haptic relay (Steam 0x80-0x86 rumble/pad-feedback). Console "HR"
// toggles it to isolate the drag-smoothness cost of relaying Steam's trackpad haptics. See haptics.cpp.
extern bool g_hapticRelay;

// Post-connect haptic block (persisted, panel-controlled): when g_hapticBlockOn, Steam haptics are dropped for
// g_hapticBlockMs after a (re)connect so the controller's haptic engine settles before the first real haptic.
extern uint8_t
	g_hapticBlockOn; // 1 = block enabled, 0 = relay haptics immediately on connect (default)
extern uint16_t
	g_hapticBlockMs; // block duration in ms (default HAPTIC_BLOCK_MS_DEFAULT)

// anything still queued (xinput uses it to pace rumble re-queues)
bool relayPending();
extern uint8_t g_relayOp; // relay frame opcode (E3 poll)
extern uint8_t g_relaySub; // relay sub-TLV type byte = SET
extern volatile uint8_t g_testHaptic; // 't<n>' injects n test haptics
// pending haptic-STOP frames to relay (kill a latched whine)
extern volatile uint8_t g_hapticStop;
// Per-slot block: arm after a (re)connect, drop haptics aimed at the slot for g_hapticBlockMs (when g_hapticBlockOn).
extern unsigned long g_hapticBlockUntil[NSLOT];

// relay the controller power-off (0x9F "off!"), burst x3. Steam's per-interface 0x9F passes that slot so
// only that controller powers off; host-suspend / the panel test button keep the broadcast default (all off).
void hapticSendShutdown(uint8_t slot = 0xFF);

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

bool hapticLinkUp(int slot = -1);
bool haptic82Blocked(int slot = -1);
bool hapticRelaySlotOk(int slot);
void haptic82HostReport(const uint8_t *p, uint16_t n);
// queue a Steam/Triton 0x80 rumble frame. `slot` = bond slot of the originating controller (0..NSLOT-1);
// defaults to 0 for the legacy single-controller callers. Per-slot so each connected controller can have its
// own active rumble stream when the host presents multiple gamepads (e.g. 4 XInput devices).
bool hapticSteamRumble(uint16_t lowFreq, uint16_t highFreq, uint8_t slot = 0);

// queue + flush the pending host/test/stop relay inside the poll cadence (called from rf_link).
// rfConnFlushRelay's s1 must carry a PID distinct from the GET poll that follows it. g_relayPid
// is initialised 2 ahead of g_pollPid and both increment once per cycle, so the 2-bit PIDs stay
// 2 apart (mod 4) forever and never collide — keeping the controller from deduplicating the GET.
// Stability test: when g_stabTest (WebUSB cmd 0x0F), buzz all controllers every 10s to keep them awake for an
// unattended uptime-until-hang measurement. hapticStabTask() is called from loop().
extern bool g_stabTest;
void hapticStabTask();

void rfConnQueueHapticRelay();
// returns true if a relay frame was actually transmitted this call (queue had an entry), so the poll loop can
// count relay TXs separately from poll cycles.
bool rfConnFlushRelay(uint8_t ch, uint8_t s1);
// times a relay-ring drain hit its iteration cap (head/tail desync or corruption) -- non-zero means we caught
// and recovered from what would otherwise be an IRQ-off watchdog hang. Surfaced on the WebUSB panel.
extern volatile uint16_t g_ringFault;

// boot reset: clear relay/active flags, arm the reconnect block
void hapticInit();

// per-loop upkeep: link-edge markers + steam 0x82 quiet timeout + fires the scheduled re-init
void hapticTask();

// replay Steam's haptic-subsystem re-init to the controller -> clears a latched/stuck buzz.
// `slot` defaults to 0xFF (broadcast to all connected) -- the re-init is a settings-only reset and is
// harmless on healthy controllers, so it's worth re-initializing every slot the firmware knows about.
void hapticReinit(uint8_t slot = 0xFF);
// Called from rf_link the instant a controller (re)connect is detected (an F-reply after a gap): blocks haptic
// relays for g_hapticBlockMs and schedules a re-init just after, to keep the freshly-booted
// controller out of the degraded/latched haptic state. Reliable -- independent of hapticTask's link heuristic.
// Per-slot: only the slot that just reconnected is blocked, the others keep relaying.
void hapticOnReconnect(int slot);
// Write the controller's global trackpad-haptics enable setting (per active emulated type). slot 0xFF = broadcast.
void hapticSetPadEnabled(uint8_t slot, bool on);
