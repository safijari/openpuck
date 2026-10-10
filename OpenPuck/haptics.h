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
#include <math.h>
#include <stdint.h>
#include "config.h" // OPK_LOG
#include "bonds.h" // NSLOT

// Post-(re)connect haptic block default. While armed, ALL Steam haptic relays to that slot are dropped: a
// just-powered-on controller's haptic engine isn't ready, and feeding it haptics in that window leaves it in a
// degraded/latched state (slow/stuck/missing haptics until a re-init). Runtime-adjustable (g_hapticBlockMs) and
// toggleable (g_hapticBlockOn) from the WebUSB panel; this is the boot default.
#define HAPTIC_BLOCK_MS_DEFAULT 10000u
// Max relayed payload bytes per entry. OUTPUT 0x87/0x88 haptic sample streams fill a whole 63-byte report;
// the haptic frame [E3][len][05][rid][63] is 67 B, inside MAXLEN=96.
#define RELAY_MAXP 63u
// Feature-0x01 commands stay at 60: the controller copies the type-01 TLV into a 63-byte command buffer.
#define RELAY_CMD_MAXP 60u
// Controller power-off: hapticSendShutdown() relays Steam's confirmed "turn off controller" command (feature-0x01
// cmd 0x9F, payload "off!" -- captured from the real puck). Sent as a small burst because the RF relay is NO-ACK.
#define HAPTIC_SHUTDOWN_SHOTS 3u
// Rumble strength: percent of the decoded host amplitude applied in every translated (non-puck) mode. 200
// (double) is the default the panel's removed rumble-strength slider shipped with, so an untouched puck feels
// exactly as before. Runtime-adjustable and persisted -- console "RS<pct>".
#define RUMBLE_SCALE_PCT 200u
#define RUMBLE_SCALE_MIN 10u
#define RUMBLE_SCALE_MAX 500u
// Rumble style: how the decoded low/high motor amplitudes are shaped before the 0x80 frame is built. Applied
// BEFORE the strength scale, on the raw host amplitudes. Persisted -- console "RY<n>".
#define RUMBLE_STYLE_NORMAL \
	0 // as captured: low -> left speed, high -> right speed
#define RUMBLE_STYLE_MONO 1 // both motors at max(low,high) -- heavier, fuller
#define RUMBLE_STYLE_HEAVY 2 // low-frequency motor only (mute the buzzy one)
#define RUMBLE_STYLE_LIGHT \
	3 // high-frequency motor only (crisp, no deep rumble)
#define RUMBLE_STYLE_SWAP 4 // swap the two motors
#define RUMBLE_STYLE_PUNCHY \
	5 // squared curve: weak effects softer, strong ones untouched
#define RUMBLE_STYLE_SOFT \
	6 // sqrt curve: lifts weak effects so subtle rumble is felt
#define RUMBLE_STYLE_MAX 6
extern uint16_t g_rumbleScale; // percent, RUMBLE_SCALE_MIN..RUMBLE_SCALE_MAX
extern uint8_t g_rumbleStyle; // RUMBLE_STYLE_*
// Switch Pro HD rumble: trackpad tone strength, percent (0..500, 100 default). The grips follow g_rumbleScale.
extern uint16_t g_hdPadScale;
// Test buzz for the panel/console: a fixed mid-scale amplitude pushed through the SAME shaping path host
// rumble takes, so what you feel is what a game at that amplitude would feel like. Auto-stops in hapticTask()
// -- the controller's haptic LATCHES, so the stop is not optional.
#define RUMBLE_TEST_AMP 0x8000u
#define RUMBLE_TEST_MS 500u
void hapticTestRumble();

// ---- relay queue (written by puck_hid.cpp, mode_*.cpp, serial_console.cpp; drained by rf_link.cpp) ----
// Enqueue one host->controller report. `slot` = bond slot (0..NSLOT-1) or 0xFF to broadcast to every
// connected controller (used by hapticSendShutdown / hapticReinit / test haptics). ISR-safe (PRIMASK).
// `expectReply`: true iff the CALLER is relaying this specifically because it wants the controller's real
// answer back (a query like GET_ATTRIBUTES/GET_STRING_ATTRIBUTE/READ_SETTING -- see puck_hid.cpp
// handleSet's `relayQuery`), as opposed to a fire-and-forget action (haptics, settings writes, power-off)
// that nothing will ever wait on a reply for. rfConnFlushRelay uses this.
bool relayEnqueue(uint8_t rid, const uint8_t *payload, uint8_t plen,
		  bool isHaptic, uint8_t slot = 0xFF, bool expectReply = false);
// Same as relayEnqueue for one bond slot, but the message is sent next instead of last. For latency-sensitive
// one-shots (trackpad click pulse) that would otherwise wait behind queued PCM stream frames. ISR-safe.
bool relayEnqueueFront(uint8_t rid, const uint8_t *payload, uint8_t plen,
		       bool isHaptic, uint8_t slot);
// Drop everything queued for one bond slot. Called when a slot becomes BONDED (Steam's 0xA2 pairing write,
// the panel's bond import): whatever was queued while the slot was empty was aimed at a controller that no
// longer -- or never did -- live there, and an unbonded slot's ring is never flushed, so it would otherwise
// be handed straight to the freshly paired controller (a stale "off!" powered it back off). ISR-safe.
void relayClearSlot(uint8_t slot);

// id9 steering (EMULATED modes only): land the controller's SET_SETTINGS index 9 (digital-mappings /
// lizard-active) at 0 once per LIZKEEP_MS per connected slot to hold its autonomous mapping/haptic engine
// OFF, or at 1 once per connect episode to turn it on -- per the active type's g_padHaptics config. id9
// gates the whole autonomous pad layer, including the trackpad ticks, and holding it off also stops the
// engine latching the deep-inside buzz seen after repeated reconnects (capture-for-haptics.txt: that buzz is
// controller-internal; OpenPuck relays no haptics in that state). PUCK modes (STEAM/LIZARD) are not steered:
// Steam writes id9 itself there, and driving it from the puck side fought those writes.
#define LIZKEEP_MS 2000u
extern uint8_t
	g_lizKeep; // 1 = hold on (default, persisted); console 'u' toggles for A/B
// Autonomous controller power-off on host sleep (see hapticTask). 1 = power the controllers off once the
// USB suspend has persisted SUSPEND_OFF_MS (default -- what the real puck does, so the controllers don't sit
// awake draining while the host sleeps). 0 = leave them on.
//
// TRADE-OFF, deliberate: with this ON the controller is off while the host sleeps, so the short-Steam-press
// remote-wakeup gesture in rf_link (guarded on USBDevice.suspended()) can no longer be sent from it. Waking
// the host from the controller then goes through the OTHER path: pressing Steam on a powered-off controller
// turns it back on, the link comes up, and the reconnect-wake in rfConnStep issues the remote wakeup. Same
// physical gesture; the power-off fires once per suspend so the returning controller is not shut off again.
// Turn this OFF (console "SO") if a controller must stay awake through host sleep. Persisted.
extern uint8_t g_suspendOff;
// Master enable for the puck->controller haptic relay (Steam 0x80-0x89 rumble/pad-feedback). Console "HR"
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
// queue a Steam/Triton 0x80 rumble frame. `slot` = bond slot of the originating controller (0..NSLOT-1);
// defaults to 0 for the legacy single-controller callers. Per-slot so each connected controller can have its
// own active rumble stream when the host presents multiple gamepads (e.g. 4 XInput devices).
bool hapticSteamRumble(uint16_t lowFreq, uint16_t highFreq, uint8_t slot = 0);
// queue an audio-driven 0x80 haptic rumble frame (gated on g_audioHaptics rather than g_rumble).
bool hapticAudioRumble(uint16_t lowFreq, uint16_t highFreq, uint8_t slot = 0);
// queue an audio-driven 0x83 tone on one actuator (side 0 = left, 1 = right). The tone plays for durMs, so
// a stream that stops refreshing it ends on its own; gainDb -128 cuts a playing tone within ~25-50 ms.
bool hapticAudioTone(uint8_t side, int8_t gainDb, uint16_t freqHz,
		     uint16_t durMs, uint8_t slot = 0);
// Actuator select byte of OUTPUT 0x82 / 0x83 (as the controller firmware routes it): trackpads 0 left, 1 right,
// 2 both; grips 3 left, 4 right, 5 both. 0x81 swaps 0 and 1. 0x80 rumble always plays on both grips.
#define HSIDE_LPAD 0
#define HSIDE_RPAD 1
#define HSIDE_PADS 2
#define HSIDE_LGRIP 3
#define HSIDE_RGRIP 4
#define HSIDE_GRIPS 5
static inline uint8_t hsidePads(bool left, bool right)
{
	return (left && right) ? HSIDE_PADS : right ? HSIDE_RPAD : HSIDE_LPAD;
}

// PCM haptic stream (OUTPUT 0x86 mode, 0x88 stereo frame; measured on the controller's IMU). Each frame
// carries PCM_SAMPLES u-law samples per side at PCM_RATE_HZ; L/R drive the left/right grip actuators. The
// controller buffers ~40 ms before playing, rides out 120 ms of jitter, and falls silent on its own when
// frames stop. hapticPcmStart sets the format (it persists, so it is re-sent periodically, not torn down).
#define PCM_SAMPLES 31u
#define PCM_RATE_HZ 4000u
void hapticPcmStart(uint8_t slot);
// Sends the first n samples of each side (n <= PCM_SAMPLES).
bool hapticPcmSend(uint8_t slot, const uint8_t *left, const uint8_t *right,
		   uint8_t n);
// Samples in the next frame of a new stream at `rate` Hz, `queued` samples into it (callers stop counting past
// the threshold). The controller starts playing on the first frame to arrive once it holds more than 16 ms
// (rate * 2 / 125: 64 samples at 4 kHz), and that fill stays buffered for the rest of the stream. One short
// frame lands the crossing on threshold + 1, so playback starts 31 samples later: at 4 kHz 31, 31, 3 start it
// at 96 (24 ms) instead of 124 with full frames.
static inline uint8_t hapticPcmFrameLen(uint16_t queued, uint16_t rate)
{
	uint16_t threshold = (uint16_t)(rate * 2u / 125u);
	if (queued > threshold)
		return PCM_SAMPLES;
	uint16_t need = (uint16_t)(threshold + 1u - queued);
	return need < PCM_SAMPLES ? (uint8_t)need : PCM_SAMPLES;
}
// G.711 u-law byte for x in [-1, 1] (clamped).
uint8_t hapticUlaw(float x);
// PCM soft limit: linear up to the knee (g_hapticLimitKnee), then eases toward full scale, so peaks past it round
// off instead of clipping. A clipped peak plays on the grips as a pop. Knee 100 passes y through to the clamp.
static inline float hapticSoftLimit(float y)
{
	float k = g_hapticLimitKnee * 0.01f, a = fabsf(y);
	if (a <= k || k >= 1.0f)
		return y;
	float o = (a - k) / (1.0f - k);
	a = k + (1.0f - k) * o / (1.0f + o);
	return y < 0 ? -a : a;
}

// Switch Pro HD rumble: the latest decoded bands per side (left low/high, right low/high) and, for
// hapticSwitchPitch, their frequencies in Hz. Called from the USB callback; hapticTask renders each side's two
// bands as PCM on that side's grip and the high band as a 0x83 tone on that side's trackpad.
void hapticSwitchHd(uint8_t slot, uint16_t leftLow, uint16_t leftHigh,
		    uint16_t rightLow, uint16_t rightHigh);
void hapticSwitchPitch(uint8_t slot, uint16_t ll, uint16_t lh, uint16_t rl,
		       uint16_t rh, uint16_t lf, uint16_t hf, uint16_t rf,
		       uint16_t rhf);
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
// relay entries evicted unsent because a slot's ring was full (serial "# stat" drop=)
extern volatile uint16_t g_relayDrops;

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
