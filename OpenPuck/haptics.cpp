#include "haptics.h"
#include "rf_timing.h"
#include "bonds.h"
#include "config.h"
#include "rf_link.h"
#include "usb_tx.h" // usbTxBoost/Unboost -- flood-rate CDC prints share the dcd DMA claim window
#include "puck_hid.h" // puckNotePowerOff() -- hold a powered-off slot disconnected to Steam
#include "steam_commands.h"

#include "fault_diag.h" // faultDiagTrace() -- flight recorder
// USBDevice.suspended() -> autonomous controller power-off on host sleep
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

uint8_t g_relayOp = 0xE3; // E3 poll
// relay sub-TLV TYPE byte. Vestigial: rfConnFlushRelay derives the on-air type from the report id (0x05 for
// actuators/haptics <0x87, 0x01 for config/settings/LED >=0x87). Still exposed over WebUSB diagnostics.
uint8_t g_relaySub = 0x05;
volatile uint8_t g_testHaptic = 0;
volatile uint8_t g_hapticStop = 0;
// Post-connect haptic block (see haptics.h): enabled + duration, both persisted and panel-adjustable.
// Off by default -- opt in from the WebUSB panel if a controller's haptics come up degraded after connect.
uint8_t g_hapticBlockOn = 0;
uint16_t g_hapticBlockMs = HAPTIC_BLOCK_MS_DEFAULT;
// id9 steering (see hapticTask): ON by default. EMULATED modes only -- it lands id9=1 (autonomous pad layer,
// the trackpad-tick source, on) or holds id9=0 every LIZKEEP_MS per the per-type g_padHaptics config. Puck
// modes (STEAM/LIZARD) are left alone: Steam owns the controller's haptics there.
// Persisted; console 'u' toggles for A/B.
uint8_t g_lizKeep = 1;
// Power the controllers off when host sleep persists (see haptics.h for the wake trade-off). Persisted;
// console "SO" toggles.
uint8_t g_suspendOff = 1;
// Host-rumble shaping (persisted in cfg.bin; console "RS<pct>" / "RY<n>").
uint16_t g_rumbleScale = RUMBLE_SCALE_PCT;
uint8_t g_rumbleStyle = RUMBLE_STYLE_NORMAL;
// Master enable for the puck->controller haptic RELAY (Steam OUTPUT reports 0x80-0x86, incl. the trackpad
// texture-feedback stream Steam pushes WHILE you drag). Each relayed frame is an extra TX that precedes the
// E3 poll and steals its reply window, and the controller must stop to process it -- both can depress the
// input rate exactly during a drag. On by default; console "HR" toggles it so the drag-smoothness cost of
// haptics can be isolated on hardware (drag with it OFF vs ON). OFF only affects Steam-driven rumble/pad
// feedback; it does NOT touch settings/config (0x87) or power-off (0x9F) relays.
bool g_hapticRelay = true;
// Per-slot reconnect block. 0 = idle; non-zero = drop haptics aimed at this slot until millis() catches up.
unsigned long g_hapticBlockUntil[NSLOT] = { 0 };

// Controller power-off. CONFIRMED from a real Windows USB capture of the Valve
// puck: Steam's "turn off controller" is feature-0x01 command 0x9F with payload
// ASCII "off!" (6F 66 66 21). Steam's per-interface 0x9F targets only that
// controller; slot 0xFF remains the broadcast form used by host suspend and the
// panel/test power-off path. The controller returns transaction status, so relay
// this command once rather than enqueueing a duplicate burst.
void hapticSendShutdown(uint8_t slot)
{
	static const uint8_t OFF[4] = { 0x6f, 0x66, 0x66, 0x21 }; // "off!"
	// slot-targeted: Steam's 0x9F arrives on ONE controller's interface and must power off only that
	// controller (broadcasting it killed every connected controller when the user turned off one). The
	// broadcast default (0xFF) remains for the triggers that logically mean "all off": host suspend and
	// the panel/test power-off button.
	faultDiagTrace(FR_OFF, slot);
	relayEnqueue(IBEX_CMD_TURN_OFF_CONTROLLER, OFF, sizeof OFF, false,
		     slot);
	// Tell the puck presentation layer to show this slot (or all, for 0xFF broadcast) cleanly DISCONNECTED to
	// Steam and hold it there through the controller's post-off F1 tail -- otherwise the dying replies bounce
	// the connection state (phantom reconnect / never-removed). Covers every power-off path (Steam 0x9F, the
	// Steam+Y chord, the panel button, host suspend) since they all funnel through here.
	puckNotePowerOff(slot);
}

// Translated 0x80 rumble state is stored AFTER mode-specific shaping/scaling.
// SDL's Steam/Triton driver resends active rumble every 40 ms because the
// controller hardware safety timeout is about 50 ms.
#define RUMBLE_RESEND_INTERVAL_MS 40u
static unsigned long g_rumble80Ms[NSLOT] = {
	0
}; // last successfully queued 0x80 TX
static uint16_t g_rumble80Low[NSLOT] = { 0 };
static uint16_t g_rumble80High[NSLOT] = { 0 };
static bool g_rumble80On[NSLOT] = { false, false, false, false };

// A rumble STOP is relayed as this many copies over successive poll cycles (see hapticSteamRumble). An ON is
// self-healing because the active keepalive repeats it every 40 ms. A STOP is the dangerous final frame: if it
// is lost, the controller can remain latched rumbling with no later ON/STOP traffic to correct it. Three copies
// provide temporal diversity against a single-frame RF loss without changing steady-state traffic.
#define RUMBLE_STOP_REPS 3

// ---- relay rings: one per bond slot. Multi-producer (USB ISR + loop-context console/xinput), one consumer
// per slot (rfConnFlushRelay on that slot's poll turn). Producers serialize under PRIMASK.
struct RelayMsg {
	uint8_t rid, len;
	bool isHaptic;
	uint8_t data[RELAY_MAXP];
	// set by the caller at relayEnqueue() time (see haptics.h) -- true iff a real controller reply is
	// wanted for this specific message, i.e. whether rfConnFlushRelay appends the query-reply trailer.
	bool expectReply;
	// Correlates a queued response request with the host transaction that created it.
	uint32_t queryGeneration;
};
// deep enough to hold a full Steam settings/LED transaction burst without loss
#define RELAY_QLEN 32
static RelayMsg g_rq[NSLOT][RELAY_QLEN];
static volatile uint8_t g_rqHead[NSLOT];
static volatile uint8_t
	g_rqTail[NSLOT]; // head=next write, tail=next read; empty when equal
static inline uint8_t rqNext(uint8_t i)
{
	return (uint8_t)((i + 1) % RELAY_QLEN);
}
// Counts times a ring-drain loop hit its iteration cap = head/tail were desynced or corrupted (e.g. an
// out-of-range g_rqHead from a stray write/stack overflow). These loops run with IRQs DISABLED, so an
// unterminated one spins forever with interrupts off -> millis()/USB/SOF all freeze and only the hardware
// watchdog recovers (an invisible "watchdog (hang)" -- the live stall monitor can't see it because the SOF
// IRQ is dead). The cap turns that into a logged, recovered event instead of a hang. Surfaced on the panel.
volatile uint16_t g_ringFault = 0;

bool relayPending()
{
	// check the current slot's queue; called from rfConnQueueHapticRelay which runs with g_curSlot set
	int cur = (g_curSlot >= 0 && g_curSlot < NSLOT) ? g_curSlot : 0;
	return g_rqHead[cur] != g_rqTail[cur];
}
bool relayQueryPending(uint8_t slot)
{
	if (slot >= NSLOT)
		return false;
	bool found = false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t i = g_rqTail[slot], guard = RELAY_QLEN + 1;
	while (i != g_rqHead[slot] && guard--) {
		const RelayMsg &m = g_rq[slot][i];
		if (m.rid && m.expectReply) {
			found = true;
			break;
		}
		i = rqNext(i);
	}
	__set_PRIMASK(pm);
	return found;
}
bool relayEnqueue(uint8_t rid, const uint8_t *payload, uint8_t plen,
		  bool isHaptic, uint8_t slot, bool expectReply)
{
	if (plen > RELAY_MAXP)
		plen = RELAY_MAXP;
	if (slot != 0xFF && slot >= NSLOT)
		return false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	// slot=0xFF: broadcast -- enqueue into every BONDED slot's ring. Unbonded slots are skipped: only used
	// slots are ever flushed (rfConnStep/doPoll), so anything queued for an empty slot sits in its ring
	// indefinitely and is then delivered to whatever controller later pairs INTO that slot -- e.g. a host
	// suspend with one controller bonded left "off!" (0x9F) frames in the empty rings, and the next
	// controller paired there powered itself off on its first flush. puckNotePowerOff() already gates its
	// broadcast on used; this is the same rule for the wire side.
	// Full queue: evict the oldest entry, never the newest. Steam bursts end with the commit/stop, so
	// dropping the oldest keeps the most-recent (meaningful) frame.
	uint8_t s0 = (slot == 0xFF) ? 0 : slot;
	uint8_t s1 = (slot == 0xFF) ? NSLOT : slot + 1;
	for (uint8_t s = s0; s < s1; s++) {
		if (slot == 0xFF && !g_slot[s].used)
			continue;
		uint8_t h = g_rqHead[s], nx = rqNext(h);
		if (nx == g_rqTail[s])
			g_rqTail[s] = rqNext(g_rqTail[s]);
		g_rq[s][h].rid = rid;
		g_rq[s][h].len = plen;
		g_rq[s][h].expectReply = expectReply;
		g_rq[s][h].queryGeneration =
			expectReply ? g_slot[s].queryHostGeneration : 0u;
		g_rq[s][h].isHaptic = isHaptic;
		if (plen)
			memcpy(g_rq[s][h].data, payload, plen);
		g_rqHead[s] = nx;
	}

	__set_PRIMASK(pm);
	faultDiagTrace(FR_RELAY, (uint16_t)((slot << 8) | rid));
	return true;
}
void relayClearSlot(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	g_rqHead[slot] = g_rqTail[slot] = 0;
	__set_PRIMASK(pm);
}

#if OPK_LOG
// diagnostic capture: a ring of the last OUTPUT reports Steam sends (rid/slot/bytes/ms), dumped with 'H'.
struct HapLog {
	uint32_t ms;
	uint8_t slot, rid, n, b[16];
}; // 16 payload bytes: capture full 0x87 settings frames
// Big always-on ring: log EVERYTHING from boot (Steam writes, our TX-to-controller, link edges) so a rare
// reconnect-buzz can be caught after the fact -- the trigger happens moments after boot, while this RAM is
// fresh, and we dump it once the panel reconnects. 4096 * 20B ~= 80KB.
#define HAPLOG_N 4096
static HapLog g_hapLog[HAPLOG_N];
static uint16_t g_hapHead = 0;
static uint16_t g_hapTail =
	0; // live/dump drain cursor (loop-context reader; chases g_hapHead)

void hapLogAdd(uint8_t slot, uint8_t rid, const uint8_t *b, uint16_t n)
{
	// Written from the USB SET ISR (handleSet) AND loop context (relay flush / link edges) -> guard g_hapHead.
	// Special slot markers for the diagnostic capture: 0xFE = a frame WE transmitted to the controller (TX
	// relay); 0xFD = a link state edge (b[0]=1 up, 0 down). Real Steam writes use the interface index (0..3).
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	HapLog &e = g_hapLog[g_hapHead];
	e.ms = millis();
	e.slot = slot;
	e.rid = rid;
	e.n = (uint8_t)(n > 255 ? 255 : n);
	for (int i = 0; i < 16; i++)
		e.b[i] = (i < (int)n) ? b[i] : 0;
	// uint16_t, NOT uint8_t: HAPLOG_N is 4096, so a uint8_t cast truncated the head to 0..255 and only the
	// first 256 ring slots were ever used (capture lost ~94% of its depth).
	g_hapHead = (uint16_t)((g_hapHead + 1) %
			       (sizeof g_hapLog / sizeof g_hapLog[0]));
	__set_PRIMASK(pm);
}
void hapticDumpLog()
{
	const uint16_t N = HAPLOG_N;
	uint32_t now = millis();
	Serial.printf("# --- capture history (now=%lu, curSlot=%d) ---\n",
		      (unsigned long)now, g_curSlot);
	for (uint16_t i = 0; i < N; i++) {
		HapLog &e = g_hapLog[(uint16_t)((g_hapHead + i) % N)];
		if (!e.ms && !e.rid)
			continue;
		Serial.printf("# -%lums if%u rid=%02X n=%u:",
			      (unsigned long)(now - e.ms), e.slot, e.rid, e.n);
		for (uint8_t j = 0; j < 16 && j < e.n; j++)
			Serial.printf(" %02X", e.b[j]);
		Serial.println();
	}
	Serial.println("# --- end ---");
}
// ---- drain cursor: stream entries to the WebUSB panel. resetDrain(false)=from "now" (live only);
//      resetDrain(true)=from the OLDEST entry (dump the whole ring from boot). pull skips empty slots. ----
void hapLogResetDrain(bool fromBoot)
{
	// oldest slot (the next-to-overwrite holds it)
	g_hapTail = fromBoot ? (uint16_t)((g_hapHead + 1) % HAPLOG_N) :
			       // "now"
			       g_hapHead;
}
bool hapLogPull(uint32_t *logMs, uint8_t *slot, uint8_t *rid, uint8_t *n,
		uint8_t bytes16[16])
{
	while (g_hapTail != g_hapHead) {
		HapLog &e = g_hapLog[g_hapTail];
		g_hapTail = (uint16_t)((g_hapTail + 1) % HAPLOG_N);
		if (!e.ms && !e.rid)
			continue; // skip empty slot (ring not yet full)
		// Return the ABSOLUTE log time (millis since boot), not "age now". The panel drains in batches 100ms
		// apart, so an age computed here would jump between batches; the panel computes age vs the newest entry.
		*logMs = e.ms;
		*slot = e.slot;
		*rid = e.rid;
		*n = (e.n > 16) ? 16 : e.n;
		memcpy(bytes16, e.b, 16);
		return true;
	}
	return false;
}
#endif // OPK_LOG

// Per-slot helpers. slot==-1 (default) checks the CURRENT poll slot (g_curSlot), used by flush-time code
// paths that don't have a slot in hand. Callers with a real slot pass it in.
bool hapticLinkUp(int slot)
{
	int s = (slot >= 0) ? slot : g_curSlot;
	if (s < 0 || s >= NSLOT)
		return false;
	return g_slot[s].used && (millis() - g_connReplyMs[s]) < 300;
}
bool haptic82Blocked(int slot)
{
	int s = (slot >= 0) ? slot : g_curSlot;
	if (s < 0 || s >= NSLOT)
		return true;
	return !hapticLinkUp(s) ||
	       (g_hapticBlockUntil[s] &&
		(int32_t)(millis() - g_hapticBlockUntil[s]) < 0);
}
// "Is haptics from this USB interface's slot allowed through?" -- the slot must be currently connected.
bool hapticRelaySlotOk(int slot)
{
	return slot >= 0 && slot < NSLOT && hapticLinkUp(slot);
}

int hapticResolveRelaySlot(int requestedSlot, uint8_t *liveCount,
			   uint8_t *decision)
{
	uint8_t live = 0;
	int sole = -1;
	for (int s = 0; s < NSLOT; s++)
		if (hapticLinkUp(s)) {
			live++;
			sole = s;
		}
	uint8_t d = HROUTE_NO_LIVE;
	int out = -1;
	if (requestedSlot >= 0 && requestedSlot < NSLOT &&
	    hapticLinkUp(requestedSlot)) {
		d = HROUTE_EXACT;
		out = requestedSlot;
	} else if (live == 1) {
		d = HROUTE_SINGLE_FALLBACK;
		out = sole;
	} else if (live > 1)
		d = HROUTE_AMBIGUOUS;
	if (liveCount)
		*liveCount = live;
	if (decision)
		*decision = d;
	return out;
}
static void hapticCancelPendingOn(int slot)
{
	// void queued ON entries (stale haptics / rumble across a reconnect). Per-slot: only the reconnected
	// slot's queue is scrubbed -- another controller's pending haptics are its own business. slot=-1 keeps
	// the old scrub-everything behavior (boot).
	int c0 = (slot >= 0 && slot < NSLOT) ? slot : 0;
	int c1 = (slot >= 0 && slot < NSLOT) ? slot + 1 : NSLOT;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	for (int s = c0; s < c1; s++) {
		uint8_t guard = RELAY_QLEN + 1;
		for (uint8_t i = g_rqTail[s]; i != g_rqHead[s]; i = rqNext(i)) {
			if (!guard--) { // desynced/corrupt ring -> recover, don't spin IRQs-off
				g_rqHead[s] = g_rqTail[s] = 0;
				g_ringFault++;
				faultDiagTrace(FR_RINGF, g_ringFault);
				break;
			}
			RelayMsg &m = g_rq[s][i];
			if (m.rid == 0x82) {
				bool on = false;
				for (uint8_t j = 2; j < m.len; j++)
					if (m.data[j]) {
						on = true;
						break;
					}
				if (on)
					m.rid = 0;
			}
			if (m.rid == 0x80) {
				bool on = false;
				for (uint8_t j = 0; j < m.len; j++)
					if (m.data[j]) {
						on = true;
						break;
					}
				if (on)
					m.rid = 0;
			}
		}
	}
	__set_PRIMASK(pm);
}
// Integer square root (binary restoring). Used by RUMBLE_STYLE_SOFT; avoids pulling float math into the USB
// OUT callback that calls hapticSteamRumble().
static uint32_t isqrt32(uint32_t v)
{
	uint32_t r = 0, b = 1UL << 30;
	while (b > v)
		b >>= 2;
	while (b) {
		if (v >= r + b) {
			v -= r + b;
			r = (r >> 1) + b;
		} else {
			r >>= 1;
		}
		b >>= 2;
	}
	return r;
}

static bool hapticQueueRumble80(uint16_t lowFreq, uint16_t highFreq,
				uint8_t slot, bool stopBurst)
{
	// Match SDL's maintained Steam/Triton report 0x80 layout exactly:
	// type=0, intensity=0, left/right speed carry the two motor amplitudes,
	// and both gains are zero.
	uint8_t p[9] = { 0x00,
			 0x00,
			 0x00,
			 (uint8_t)(lowFreq & 0xFF),
			 (uint8_t)(lowFreq >> 8),
			 0x00,
			 (uint8_t)(highFreq & 0xFF),
			 (uint8_t)(highFreq >> 8),
			 0x00 };

	uint8_t reps = stopBurst ? RUMBLE_STOP_REPS : 1;
	bool queued = false;
	for (uint8_t i = 0; i < reps; i++)
		if (relayEnqueue(0x80, p, sizeof p, true, slot))
			queued = true;

	if (queued)
		g_rumble80Ms[slot] = millis();
	return queued;
}

bool hapticSteamRumble(uint16_t lowFreq, uint16_t highFreq, uint8_t slot)
{
	if (slot >= NSLOT)
		return false;
	// Shape the decoded host amplitudes: style first (which motor plays, and the response curve), then the
	// strength scale. Integer-only: host OUTPUT paths can invoke this from TinyUSB callback context, while
	// loop-context test/keepalive paths use the same shaping without introducing float math.
	{
		uint32_t l = lowFreq, h = highFreq, t;
		switch (g_rumbleStyle) {
		case RUMBLE_STYLE_MONO:
			l = h = (l > h) ? l : h;
			break;
		case RUMBLE_STYLE_HEAVY:
			h = 0;
			break;
		case RUMBLE_STYLE_LIGHT:
			l = 0;
			break;
		case RUMBLE_STYLE_SWAP:
			t = l;
			l = h;
			h = t;
			break;
		case RUMBLE_STYLE_PUNCHY:
			// x^2/FS: 0xFFFF*0xFFFF fits uint32 exactly, so no intermediate overflow
			l = l * l / 0xFFFF;
			h = h * h / 0xFFFF;
			break;
		case RUMBLE_STYLE_SOFT:
			l = isqrt32(l * 0xFFFF);
			h = isqrt32(h * 0xFFFF);
			break;
		default: // RUMBLE_STYLE_NORMAL
			break;
		}
		l = l * g_rumbleScale / 100;
		h = h * g_rumbleScale / 100;
		lowFreq = (l > 0xFFFF) ? 0xFFFF : (uint16_t)l;
		highFreq = (h > 0xFFFF) ? 0xFFFF : (uint16_t)h;
	}
	bool on = lowFreq || highFreq;
	// If rumble is disabled while active, turn the next ON request into a stop.
	if (on && !g_rumble) {
		lowFreq = highFreq = 0;
		on = false;
	}
	// Per-slot settle gate (the per-slot reconnect block + link-up check). 0x82 haptics in Steam mode use the
	// same gate; for XInput, the host only sends a stream while a controller is connected, so this also doubles
	// as "no controller here, no relay".
	if (on && haptic82Blocked(slot))
		return false;
	if (!on && !hapticLinkUp(slot)) {
		g_rumble80Low[slot] = 0;
		g_rumble80High[slot] = 0;
		g_rumble80On[slot] = false;
		g_rumble80Ms[slot] = 0;
		return false;
	}

	bool stopping = !on && g_rumble80On[slot];
	if (!hapticQueueRumble80(lowFreq, highFreq, slot, stopping))
		return false;

	// Save already-shaped/scaled values; keepalive must not apply shaping twice.
	g_rumble80Low[slot] = lowFreq;
	g_rumble80High[slot] = highFreq;
	g_rumble80On[slot] = on;
	return true;
}
// Queue a pending test-haptic / stop relay (runs inside the poll cadence -- never at raw loop rate). Test
// haptics broadcast to all connected slots (slot 0xFF); the stop frame is broadcast too (a stuck latch can
// affect any controller, and the haptic-engine clear-re-init is settings-only so it's harmless on healthy
// ones).
// Hardware-accepted production contract, derived from official Valve/Proteus RF captures and T4.2 testing:
//   * USB OUTPUT 0x87/0x88/0x89 body is exactly 63 bytes.
//   * RF uses E3 40 05 <rid> <63-byte body>; that E3 is the poll turn and returns normal F1.
//   * Short ordinary relay TLVs are appended behind PCM up to the existing 96-byte RF payload ceiling.
//   * During a live PCM session, a temporarily empty PCM queue does not allow a short relay to steal the turn;
//     it waits through a 24-ms grace for the next PCM body. Oversized relay heads still escape standalone.
#define PCM_BODY_LEN 63u
#define PCM_RF_PAYLOAD_LEN 67u
#define PCM_RF_MAX_PAYLOAD 96u
#define PCM_QUEUE_LEN 8u
#define PCM_ACTIVE_GRACE_MS 24u
struct PcmMsg {
	uint8_t rid;
	uint8_t body[PCM_BODY_LEN];
};
static PcmMsg g_pcmQ[NSLOT][PCM_QUEUE_LEN];
static volatile uint8_t g_pcmHead[NSLOT] = { 0 }, g_pcmTail[NSLOT] = { 0 };
static uint32_t g_pcmLastUsbMs[NSLOT] = { 0 };
static inline uint8_t pcmNext(uint8_t v)
{
	return (uint8_t)((v + 1u) % PCM_QUEUE_LEN);
}
bool pcmEnqueue(uint8_t rid, const uint8_t *payload, uint16_t plen,
		uint8_t slot)
{
	if (slot >= NSLOT || rid < 0x87 || rid > 0x89 || !payload ||
	    plen != PCM_BODY_LEN)
		return false;
	uint32_t now = millis();
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t h = g_pcmHead[slot], nx = pcmNext(h);
	if (nx == g_pcmTail[slot])
		g_pcmTail[slot] = pcmNext(g_pcmTail[slot]);
	g_pcmQ[slot][h].rid = rid;
	memcpy(g_pcmQ[slot][h].body, payload, PCM_BODY_LEN);
	g_pcmHead[slot] = nx;
	g_pcmLastUsbMs[slot] = now;
	__set_PRIMASK(pm);
	return true;
}
bool pcmSessionActive(uint8_t slot)
{
	if (slot >= NSLOT)
		return false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint32_t last = g_pcmLastUsbMs[slot];
	bool pending = (g_pcmTail[slot] != g_pcmHead[slot]);
	__set_PRIMASK(pm);
	if (pending)
		return true;
	return last != 0u && (uint32_t)(millis() - last) <= PCM_ACTIVE_GRACE_MS;
}
bool pcmAnyActive(void)
{
	for (uint8_t s = 0; s < NSLOT; s++)
		if (pcmSessionActive(s))
			return true;
	return false;
}
bool pcmRelayPending(uint8_t slot)
{
	if (slot >= NSLOT)
		return false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	bool pending = (g_rqTail[slot] != g_rqHead[slot]);
	__set_PRIMASK(pm);
	return pending;
}
bool pcmRelayNeedsStandalone(uint8_t slot)
{
	if (slot >= NSLOT)
		return false;
	bool needStandalone = false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t guard = RELAY_QLEN + 1;
	while (g_rqTail[slot] != g_rqHead[slot]) {
		if (!guard--) {
			g_rqHead[slot] = g_rqTail[slot] = 0;
			g_ringFault++;
			faultDiagTrace(FR_RINGF, g_ringFault);
			break;
		}
		RelayMsg &m = g_rq[slot][g_rqTail[slot]];
		if (!m.rid) {
			g_rqTail[slot] = rqNext(g_rqTail[slot]);
			continue;
		}
		// Feature queries and TURN_OFF_CONTROLLER are transaction/status boundaries.
		if (m.expectReply ||
		    (!m.isHaptic && m.rid == IBEX_CMD_TURN_OFF_CONTROLLER)) {
			needStandalone = true;
			break;
		}
		uint8_t rl = m.len;
		if (rl > RELAY_MAXP)
			rl = RELAY_MAXP;
		uint8_t need = (uint8_t)((m.isHaptic ? 3u : 4u) + rl);
		needStandalone =
			((uint16_t)PCM_RF_PAYLOAD_LEN + (uint16_t)need) >
			(uint16_t)PCM_RF_MAX_PAYLOAD;
		break;
	}
	__set_PRIMASK(pm);
	return needStandalone;
}
static bool pcmAppendRelayLocked(uint8_t cur, uint8_t *p, uint8_t *plen)
{
	uint8_t guard = RELAY_QLEN + 1;
	while (g_rqTail[cur] != g_rqHead[cur]) {
		if (!guard--) {
			g_rqHead[cur] = g_rqTail[cur] = 0;
			g_ringFault++;
			faultDiagTrace(FR_RINGF, g_ringFault);
			return false;
		}
		RelayMsg &m = g_rq[cur][g_rqTail[cur]];
		if (!m.rid) {
			g_rqTail[cur] = rqNext(g_rqTail[cur]);
			continue;
		}
		if (m.expectReply ||
		    (!m.isHaptic && m.rid == IBEX_CMD_TURN_OFF_CONTROLLER))
			return false; // status/transaction boundary
		uint8_t rl = m.len;
		if (rl > RELAY_MAXP)
			rl = RELAY_MAXP;
		uint8_t need = (uint8_t)((m.isHaptic ? 3u : 4u) + rl);
		if ((uint16_t)(*plen) + need > PCM_RF_MAX_PAYLOAD)
			return false;
		g_rqTail[cur] = rqNext(g_rqTail[cur]);
		uint8_t pos = *plen;
		if (!m.isHaptic) {
			p[pos++] = (uint8_t)(2u + rl);
			p[pos++] = 0x01;
			p[pos++] = m.rid;
			p[pos++] = rl;
		} else {
			p[pos++] = (uint8_t)(1u + rl);
			p[pos++] = 0x05;
			p[pos++] = m.rid;
		}
		if (rl) {
			memcpy(p + pos, m.data, rl);
			pos = (uint8_t)(pos + rl);
		}
		*plen = pos;
		return true;
	}
	return false;
}
bool rfConnFlushPcm(uint8_t ch, uint8_t s1, uint8_t *rxOut)
{
	int cur = (g_curSlot >= 0 && g_curSlot < NSLOT) ? g_curSlot : 0;
	PcmMsg msg;
	bool have = false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (g_pcmTail[cur] != g_pcmHead[cur]) {
		msg = g_pcmQ[cur][g_pcmTail[cur]];
		g_pcmTail[cur] = pcmNext(g_pcmTail[cur]);
		have = true;
	}
	__set_PRIMASK(pm);
	if (!have)
		return false;
	uint8_t p[PCM_RF_MAX_PAYLOAD];
	uint8_t plen = PCM_RF_PAYLOAD_LEN;
	p[0] = 0xE3;
	p[1] = 0x40;
	p[2] = 0x05;
	p[3] = msg.rid;
	memcpy(p + 4, msg.body, PCM_BODY_LEN);
	pm = __get_PRIMASK();
	__disable_irq();
	while (pcmAppendRelayLocked((uint8_t)cur, p, &plen)) {
	}
	__set_PRIMASK(pm);
	uint8_t rx = rfConnTx(ch, s1, p, plen, 0);
	if (rxOut)
		*rxOut = rx;
	hapLogAdd(0xFC, msg.rid, msg.body, 16);
	return true;
}
// Stability-test keepalive: while g_stabTest, buzz every connected controller for ~150ms once per 10s so an
// unattended uptime measurement isn't ended by a controller idle-sleeping. Toggled by WebUSB cmd 0x0F; the
// panel times uptime-until-reset. Reboots clear g_stabTest (the panel re-arms it on reconnect).
bool g_stabTest = false;
void hapticStabTask()
{
	if (!g_stabTest)
		return;
	static const uint8_t on[3] = { 0x01, 0x01, 0xF7 };
	static const uint8_t off[3] = { 0x01, 0x01, 0x00 };
	static unsigned long lastOn = 0, offAt = 0;
	unsigned long now = millis();
	// Buzz only slots whose controller is actually answering polls (same 300ms liveness the panel/0x79 use).
	// The old 0xFF broadcast kept stuffing a powered-off controller's relay ring -- pure eviction churn plus
	// wasted TX at a radio that can't hear it, right inside the power-off hang window (issue #72 repro).
	auto enqLive = [&](const uint8_t *p) {
		for (uint8_t s = 0; s < NSLOT; s++)
			if (g_slot[s].used && g_connReplyMs[s] &&
			    (unsigned long)(now - g_connReplyMs[s]) < 300u)
				relayEnqueue(0x82, p, 3, true, s);
	};
	if (lastOn == 0 || (uint32_t)(now - lastOn) >= 10000u) {
		lastOn = now;
		enqLive(on);
		offAt = now + 150;
	}
	if (offAt && (int32_t)(now - offAt) >= 0) {
		enqLive(off);
		offAt = 0;
	}
}

void rfConnQueueHapticRelay()
{
	if (relayPending())
		return; // host relays first; injectables wait for an idle cycle
	static const uint8_t HAP_ON[3] = { 0x01, 0x01, 0xF7 };
	static const uint8_t HAP_OFF[3] = { 0x01, 0x01, 0x00 };
	if (g_testHaptic) {
		if (relayEnqueue(0x82, HAP_ON, 3, true, 0xFF))
			g_testHaptic--;
	} else if (g_hapticStop && !g_xbox) {
		if (relayEnqueue(0x82, HAP_OFF, 3, true, 0xFF))
			g_hapticStop--;
	}
}
// rfConnFlushRelay(ch, s1): drain one entry from the current slot's relay queue and TX it. Each slot's queue
// is independent, so each controller only sees its own commands. Response-bearing feature commands execute
// in a standalone RF turn and retain one in-flight generation until its
// matching Type-4 response is consumed. Retrieval is ordered around ordinary E3
// polls so stale prior-command responses can drain without extending the 80-ms
// transaction deadline. Bounded replay paths cover: delayed retrieval, one deferred
// query retry, one deferred initial-fetch retry, stale-response replay, a no-Type4
// grace replay, one repeated-stale replay, silence recovery after either replay,
// same-PID zero-RX retries for the terminal replay calls, and one evidence-gated
// terminal FETCH. Every replay is generation-owned and capped so no path can loop.
static volatile uint8_t g_featureFetchPhase[NSLOT] = { 0 };
static volatile uint8_t g_featureTurnConsumed[NSLOT] = { 0 };
// Phase-ownership latch: one generation may authorize one existing poll.
static volatile uint32_t g_featureLatePollAuthGeneration[NSLOT] = { 0 };
// Same-generation pre-split authorization only; not a response cache or retry budget.
static volatile uint32_t g_featureCoframeGeneration[NSLOT] = { 0 };

// Cross-command quiescence begins only after a genuinely RF-armed generation is
// consumed. Unqueued fallback generations do not start this quiet window.
static volatile uint32_t g_featureLastArmedGeneration[NSLOT] = { 0 };
static volatile uint8_t g_featureLastArmedCmd[NSLOT] = { 0 };
static volatile uint32_t g_featureObservedConsumedGeneration[NSLOT] = { 0 };
static volatile uint8_t g_featureLastConsumedCmd[NSLOT] = { 0 };
static volatile uint32_t g_featureQuiesceStartMs[NSLOT] = { 0 };

// The original query arm time bounds the single delayed retrieval probe.
static volatile uint32_t g_featureQueryArmMs[NSLOT] = { 0 };
static volatile uint32_t g_featureDelayedFetchGeneration[NSLOT] = { 0 };

// A query with zero RX may retry once after exactly one scheduler opportunity.
// Retain the original payload and S1; clear the latch before retry I/O.
static uint32_t g_featureDeferredQueryRetryGeneration[NSLOT] = { 0 };
static uint8_t g_featureDeferredQueryRetryLen[NSLOT] = { 0 };
static uint8_t g_featureDeferredQueryRetryWaitTurns[NSLOT] = { 0 };
static uint8_t g_featureDeferredQueryRetryPayload[NSLOT][80] = { { 0 } };

// The normal initial FETCH may retry once after one scheduler opportunity.
// Delayed retrieval probes do not use this retry state.
static uint32_t g_featureDeferredFetchRetryGeneration[NSLOT] = { 0 };
static uint8_t g_featureDeferredFetchRetryWaitTurns[NSLOT] = { 0 };
static uint8_t g_featureDeferredFetchRetryPayload[NSLOT][4] = { { 0 } };

// Cache the exact active query so a structurally valid stale Type-4 mismatch can
// authorize one replay of the current generation.
static uint32_t g_featureQuerySnapshotGeneration[NSLOT] = { 0 };
static uint8_t g_featureQuerySnapshotLen[NSLOT] = { 0 };
static uint8_t g_featureQuerySnapshotPayload[NSLOT][80] = { { 0 } };
static uint32_t g_featureStaleReplayGeneration[NSLOT] = { 0 };
static uint32_t g_featureStaleReplayUsedGeneration[NSLOT] = { 0 };

// If the delayed probe still yields no Type-4, allow one ordinary grace poll
// before one exact current-query replay.
static uint8_t g_featureDelayedProbeSawType4[NSLOT] = { 0 };
static uint32_t g_featureNoType4GraceGeneration[NSLOT] = { 0 };
static uint8_t g_featureNoType4GraceState[NSLOT] = { 0 };
static uint32_t g_featureNoType4ReplayUsedGeneration[NSLOT] = { 0 };

// A new exact stale mismatch observed after the first stale replay can authorize
// one additional bounded replay for the same generation.
static uint32_t g_featureRepeatStaleGeneration[NSLOT] = { 0 };
static uint32_t g_featureRepeatStaleUsedGeneration[NSLOT] = { 0 };

// After a stale replay, track whether any Type-4 arrives. Pure silence may
// authorize one mutually-exclusive recovery replay.
static uint32_t g_featurePostStaleSilenceTrackGeneration[NSLOT] = { 0 };
static uint8_t g_featurePostStaleSilenceSawType4[NSLOT] = { 0 };
static uint32_t g_featurePostStaleSilenceReplayGeneration[NSLOT] = { 0 };
static uint32_t g_featurePostStaleSilenceUsedGeneration[NSLOT] = { 0 };
static void featureClearPostStaleSilenceTrack(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featurePostStaleSilenceTrackGeneration[slot] = 0u;
	g_featurePostStaleSilenceSawType4[slot] = 0u;
}
static void featureClearPostStaleSilenceReplay(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featurePostStaleSilenceReplayGeneration[slot] = 0u;
}
static void featureResetPostStaleSilence(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featurePostStaleSilenceUsedGeneration[slot] = 0u;
	featureClearPostStaleSilenceTrack(slot);
	featureClearPostStaleSilenceReplay(slot);
}

// Once the no-Type4 replay is spent, any later Type-4 invalidates the pure-silence
// premise used by the terminal recovery branch.
static uint32_t g_featurePostNoType4Type4SeenGeneration[NSLOT] = { 0 };
// Track no-Type4 replay evidence and ownership of the one terminal retrieval.
static volatile uint32_t g_featureNoType4ReplayActiveGeneration[NSLOT] = { 0 };
static volatile uint32_t g_featureNoType4CurrentResponseGeneration[NSLOT] = {
	0
};
static volatile uint32_t g_featureTerminalFetchGeneration[NSLOT] = { 0 };
static volatile uint32_t g_featureTerminalFetchUsedGeneration[NSLOT] = { 0 };
static uint32_t g_featurePostNoType4ReplayGeneration[NSLOT] = { 0 };
static uint32_t g_featurePostNoType4ReplayUsedGeneration[NSLOT] = { 0 };
static void featureClearPostNoType4Replay(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featurePostNoType4ReplayGeneration[slot] = 0u;
}
static void featureClearTerminalFetchEvidence(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featureNoType4ReplayActiveGeneration[slot] = 0u;
	g_featureNoType4CurrentResponseGeneration[slot] = 0u;
	g_featureTerminalFetchGeneration[slot] = 0u;
}
static void featureResetPostNoType4(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featurePostNoType4Type4SeenGeneration[slot] = 0u;
	g_featurePostNoType4ReplayUsedGeneration[slot] = 0u;
	featureClearPostNoType4Replay(slot);
}

// use arms one post-replay stale recovery. Any subsequent Type4 before service cancels it.
static uint32_t g_featurePostNoType4StaleGeneration[NSLOT] = { 0 };
static uint32_t g_featurePostNoType4StaleUsedGeneration[NSLOT] = { 0 };
static void featureClearPostNoType4StaleReplay(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featurePostNoType4StaleGeneration[slot] = 0u;
}
static void featureResetPostNoType4StaleReplay(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featurePostNoType4StaleUsedGeneration[slot] = 0u;
	featureClearPostNoType4StaleReplay(slot);
}

static void featureClearRepeatStaleReplay(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featureRepeatStaleGeneration[slot] = 0u;
}
static void featureResetRepeatStaleReplay(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featureRepeatStaleUsedGeneration[slot] = 0u;
	featureClearRepeatStaleReplay(slot);
}

static void featureClearNoType4Grace(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featureNoType4GraceGeneration[slot] = 0u;
	g_featureNoType4GraceState[slot] = 0u;
}
static void featureResetNoType4Grace(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featureDelayedProbeSawType4[slot] = 0u;
	g_featureNoType4ReplayUsedGeneration[slot] = 0u;
	featureClearNoType4Grace(slot);
}

static void featureClearStaleQueryReplay(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featureStaleReplayGeneration[slot] = 0u;
}
static void featureResetQuerySnapshot(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featureQuerySnapshotGeneration[slot] = 0u;
	g_featureQuerySnapshotLen[slot] = 0u;
	memset(g_featureQuerySnapshotPayload[slot], 0,
	       sizeof g_featureQuerySnapshotPayload[slot]);
	g_featureStaleReplayUsedGeneration[slot] = 0u;
	featureClearStaleQueryReplay(slot);
}

static void featureClearDeferredFetchRetry(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featureDeferredFetchRetryGeneration[slot] = 0u;
	g_featureDeferredFetchRetryWaitTurns[slot] = 0u;
	memset(g_featureDeferredFetchRetryPayload[slot], 0,
	       sizeof g_featureDeferredFetchRetryPayload[slot]);
}

static void featureClearDeferredQueryRetry(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_featureDeferredQueryRetryGeneration[slot] = 0u;
	g_featureDeferredQueryRetryLen[slot] = 0u;
	g_featureDeferredQueryRetryWaitTurns[slot] = 0u;
}

static void featureObserveConsumedGeneration(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	Slot &S = g_slot[slot];
	uint32_t consumed = S.queryConsumedGeneration;
	if (consumed == g_featureObservedConsumedGeneration[slot])
		return;
	g_featureObservedConsumedGeneration[slot] = consumed;
	// Only an RF-armed generation establishes the quiet-window source.
	if (consumed != 0u && consumed == g_featureLastArmedGeneration[slot]) {
		g_featureLastConsumedCmd[slot] = g_featureLastArmedCmd[slot];
		g_featureQuiesceStartMs[slot] = millis();
	}
}
void rfConnFeatureObserveType4(uint8_t slot, uint8_t seenCmd, uint8_t reason)
{
	// Stale replay is deliberately narrow: only an exact, structurally
	// valid stale-command mismatch may arm one late exact current-query replay.
	if (slot >= NSLOT || reason != 0x20u)
		return;
	Slot &S = g_slot[slot];
	uint32_t gen = S.pendingQueryGeneration;
	if (!S.pendingQueryCmd || !gen || S.pendingQueryCmd == seenCmd)
		return;
	if (g_featureQuerySnapshotGeneration[slot] != gen ||
	    !g_featureQuerySnapshotLen[slot])
		return;
	if (g_featureStaleReplayUsedGeneration[slot] == gen ||
	    g_featureStaleReplayGeneration[slot] == gen)
		return;
	g_featureStaleReplayGeneration[slot] = gen;
}

void rfConnFeatureObserveType4DuringDelayedProbe(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	uint32_t gen = g_slot[slot].pendingQueryGeneration;
	if (!gen)
		return;
	// Any Type4 observed during the delayed probe invalidates the no-Type4
	// premise. Current responses complete normally; stale responses are handled separately.
	if (g_featureDelayedFetchGeneration[slot] == gen)
		g_featureDelayedProbeSawType4[slot] = 1u;
	if (g_featureNoType4GraceGeneration[slot] == gen)
		featureClearNoType4Grace(slot);
}

void rfConnFeatureObserveType4AfterStaleReplay(uint8_t slot, uint8_t seenCmd,
					       uint8_t reason)
{
	// A repeated-stale replay requires a new structurally valid mismatch after
	// the first late-query replay budget has already been consumed.
	if (slot >= NSLOT || reason != 0x20u)
		return;
	Slot &S = g_slot[slot];
	uint32_t gen = S.pendingQueryGeneration;
	if (!S.pendingQueryCmd || !gen || S.pendingQueryCmd == seenCmd)
		return;
	if (g_featureQuerySnapshotGeneration[slot] != gen ||
	    !g_featureQuerySnapshotLen[slot])
		return;
	if (g_featureStaleReplayUsedGeneration[slot] != gen)
		return;
	if (g_featureNoType4ReplayUsedGeneration[slot] == gen)
		return;
	if (g_featureRepeatStaleUsedGeneration[slot] == gen ||
	    g_featureRepeatStaleGeneration[slot] == gen)
		return;
	g_featureRepeatStaleGeneration[slot] = gen;
}

void rfConnFeatureObserveType4DuringPostStaleSilence(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	Slot &S = g_slot[slot];
	uint32_t gen = S.pendingQueryGeneration;
	if (!S.pendingQueryCmd || !gen)
		return;
	// Any Type4 after a stale-query replay destroys the pure-silence premise.
	if (g_featurePostStaleSilenceTrackGeneration[slot] == gen)
		g_featurePostStaleSilenceSawType4[slot] = 1u;
	if (g_featurePostStaleSilenceReplayGeneration[slot] == gen)
		featureClearPostStaleSilenceReplay(slot);
}

void rfConnFeatureObserveType4AfterNoType4Replay(uint8_t slot, uint8_t seenCmd,
						 uint8_t reason)
{
	if (slot >= NSLOT)
		return;
	Slot &S = g_slot[slot];
	uint32_t gen = S.pendingQueryGeneration;
	if (!S.pendingQueryCmd || !gen)
		return;
	// Cache evidence only, never response bytes: a no-Type4 replay must be active
	// inside its RF call, with a FETCH-pending current command.
	if (g_featureNoType4ReplayActiveGeneration[slot] == gen &&
	    reason == 0x80u && seenCmd == S.pendingQueryCmd)
		g_featureNoType4CurrentResponseGeneration[slot] = gen;
	// Once the no-Type4 replay starts, any later Type4 destroys the pure-silence
	// premise and cancels a pending silence-based replay.
	if (g_featureNoType4ReplayUsedGeneration[slot] == gen) {
		g_featurePostNoType4Type4SeenGeneration[slot] = gen;
		if (g_featurePostNoType4ReplayGeneration[slot] == gen)
			featureClearPostNoType4Replay(slot);
	}
}

void rfConnFeatureObserveType4ForPostNoType4Stale(uint8_t slot, uint8_t seenCmd,
						  uint8_t reason)
{
	if (slot >= NSLOT)
		return;
	Slot &S = g_slot[slot];
	uint32_t gen = S.pendingQueryGeneration;
	if (!S.pendingQueryCmd || !gen)
		return;
	if (g_featureNoType4ReplayUsedGeneration[slot] != gen)
		return; // dedicated no-Type4 replay ownership
	if (g_featureTerminalFetchUsedGeneration[slot] == gen)
		return;
	// Once the post-replay stale branch is armed, any later Type4 cancels it.
	if (g_featurePostNoType4StaleGeneration[slot] == gen) {
		featureClearPostNoType4StaleReplay(slot);
		return;
	}
	if ((reason & 0x20u) == 0u || seenCmd == S.pendingQueryCmd)
		return;
	if (g_featureQuerySnapshotGeneration[slot] != gen ||
	    !g_featureQuerySnapshotLen[slot])
		return;
	// Do not create another recovery branch after a silence replay or after any
	// stale-response recovery branch has already consumed this generation.
	if (g_featurePostNoType4ReplayUsedGeneration[slot] == gen ||
	    g_featurePostNoType4StaleUsedGeneration[slot] == gen ||
	    g_featurePostStaleSilenceUsedGeneration[slot] == gen ||
	    g_featureRepeatStaleUsedGeneration[slot] == gen)
		return;
	g_featurePostNoType4StaleGeneration[slot] = gen;
	// Explicit stale handling owns this branch; cancel any pending pure-silence recovery.
	if (g_featurePostNoType4ReplayGeneration[slot] == gen)
		featureClearPostNoType4Replay(slot);
}

uint8_t rfConnFeatureFetchPhase(uint8_t slot)
{
	if (slot >= NSLOT)
		return 0;
	if (!g_slot[slot].pendingQueryCmd) {
		g_featureFetchPhase[slot] = 0;
		return 0;
	}
	return g_featureFetchPhase[slot];
}
bool rfConnFeatureFetchPending(uint8_t slot)
{
	uint8_t p = rfConnFeatureFetchPhase(slot);
	return p == FEATURE_FETCH_WAIT_POLL || p == FEATURE_FETCH_ELIGIBLE;
}
bool rfConnFeatureFetchEligible(uint8_t slot)
{
	return rfConnFeatureFetchPhase(slot) == FEATURE_FETCH_ELIGIBLE;
}
bool rfConnFeatureForceOrdinaryPoll(uint8_t slot)
{
	// The wait slot and deferred query retry own the first two opportunities after the initial query transmission.
	if (slot < NSLOT && g_featureDeferredQueryRetryGeneration[slot] != 0u)
		return false;
	// Suppress the forced follow-up poll while the initial FETCH wait/retry latch owns
	// the next two scheduler opportunities.
	if (slot < NSLOT && g_featureDeferredFetchRetryGeneration[slot] != 0u)
		return false;
	// A stale-triggered current-query replay owns the next scheduler RF opportunity.
	if (slot < NSLOT && g_featureStaleReplayGeneration[slot] != 0u)
		return false;
	// A repeated-stale replay owns the next scheduler RF opportunity.
	if (slot < NSLOT && g_featureRepeatStaleGeneration[slot] != 0u)
		return false;
	if (slot < NSLOT &&
	    g_featurePostStaleSilenceReplayGeneration[slot] != 0u)
		return false;
	if (slot < NSLOT && g_featurePostNoType4StaleGeneration[slot] != 0u)
		return false;
	if (slot < NSLOT && g_featurePostNoType4ReplayGeneration[slot] != 0u)
		return false;
	if (slot < NSLOT && g_featureTerminalFetchGeneration[slot] != 0u)
		return false;
	uint8_t p = rfConnFeatureFetchPhase(slot);
	return p == FEATURE_FETCH_WAIT_POLL || p == FEATURE_FETCH_FOLLOWUP_POLL;
}
bool rfConnFeatureCoframeWaitPollAuthBegin(uint8_t slot)
{
	if (slot >= NSLOT)
		return false;
	Slot &S = g_slot[slot];
	uint32_t gen = g_featureCoframeGeneration[slot];
	if (!gen || !S.pendingQueryCmd || S.pendingQueryGeneration != gen)
		return false;
	if (g_featureFetchPhase[slot] != FEATURE_FETCH_WAIT_POLL)
		return false;
	if (!S.pendingQueryDeadlineMs ||
	    (int32_t)(millis() - S.pendingQueryDeadlineMs) >= 0)
		return false;
	g_featureFetchPhase[slot] = FEATURE_FETCH_FOLLOWUP_POLL;

	return true;
}

void rfConnFeatureCoframeWaitPollAuthEnd(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	uint32_t gen = g_featureCoframeGeneration[slot];
	Slot &S = g_slot[slot];

	g_featureCoframeGeneration[slot] = 0u;
	if (!S.pendingQueryCmd) {
		rfConnFeaturePollCompleted(slot);
		return;
	}
	if (gen && S.pendingQueryGeneration == gen) {
		// Exactly the result of one unresolved WAIT_POLL ordinary poll:
		// split FETCH becomes eligible on the next existing feature turn.
		g_featureFetchPhase[slot] = FEATURE_FETCH_ELIGIBLE;

		return;
	}
	g_featureFetchPhase[slot] = 0u; // generation changed: fail closed
}

bool rfConnFeatureLateWaitPollAuthBegin(uint8_t slot)
{
	if (slot >= NSLOT)
		return false;
	Slot &S = g_slot[slot];
	if (!S.pendingQueryCmd || !S.pendingQueryGeneration)
		return false;
	uint32_t gen = S.pendingQueryGeneration;
	if (g_featureNoType4ReplayUsedGeneration[slot] != gen)
		return false;
	if (g_featureLatePollAuthGeneration[slot] == gen)
		return false;
	if (g_featureFetchPhase[slot] != FEATURE_FETCH_WAIT_POLL)
		return false;
	if (!S.pendingQueryDeadlineMs ||
	    (int32_t)(millis() - S.pendingQueryDeadlineMs) >= 0)
		return false;
	g_featureLatePollAuthGeneration[slot] = gen;
	// Phase-only authorization: the existing ordinary poll remains the only RF action.
	g_featureFetchPhase[slot] = FEATURE_FETCH_FOLLOWUP_POLL;

	return true;
}

void rfConnFeatureLateWaitPollAuthEnd(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	uint32_t gen = g_featureLatePollAuthGeneration[slot];
	if (!gen) {
		rfConnFeaturePollCompleted(slot);
		return;
	}
	Slot &S = g_slot[slot];

	g_featureLatePollAuthGeneration[slot] = 0u;
	if (!S.pendingQueryCmd) {
		// Preserve the normal completed-query cleanup.
		rfConnFeaturePollCompleted(slot);
		return;
	}
	if (S.pendingQueryGeneration == gen) {
		// This is exactly the state rfConnFeaturePollCompleted() would
		// produce after a WAIT_POLL ordinary poll that did not complete the query.
		g_featureFetchPhase[slot] = FEATURE_FETCH_ELIGIBLE;

		return;
	}
	// Generation changed unexpectedly: fail closed rather than authorizing a new owner.
	g_featureFetchPhase[slot] = 0u;
}

void rfConnFeaturePollCompleted(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	uint8_t completedPhase = g_featureFetchPhase[slot];
	if (!g_slot[slot].pendingQueryCmd) {
		g_featureLatePollAuthGeneration[slot] = 0u;
		g_featureCoframeGeneration[slot] = 0u;
		g_featureFetchPhase[slot] = 0;
		featureClearNoType4Grace(slot);
		g_featureDelayedProbeSawType4[slot] = 0u;
		featureClearPostStaleSilenceTrack(slot);
		featureClearPostStaleSilenceReplay(slot);
		g_featurePostNoType4Type4SeenGeneration[slot] = 0u;
		featureClearPostNoType4Replay(slot);
		featureResetPostNoType4StaleReplay(slot);
		featureClearTerminalFetchEvidence(slot);
		return;
	}
	uint32_t gen = g_slot[slot].pendingQueryGeneration;
	if (completedPhase == FEATURE_FETCH_WAIT_POLL)
		g_featureFetchPhase[slot] = FEATURE_FETCH_ELIGIBLE;
	else if (completedPhase == FEATURE_FETCH_FOLLOWUP_POLL)
		g_featureFetchPhase[slot] = 0;

	// Only the post-poll belonging to the already-spent delayed probe may arm a
	// no-Type4 replay, and only if no Type4 was observed during that probe/poll.
	if (completedPhase == FEATURE_FETCH_FOLLOWUP_POLL &&
	    g_featureDelayedFetchGeneration[slot] == gen) {
		bool arm = !g_featureDelayedProbeSawType4[slot] &&
			   g_featureDelayedFetchGeneration[slot] == gen &&
			   g_featureStaleReplayGeneration[slot] != gen &&
			   g_featureStaleReplayUsedGeneration[slot] != gen &&
			   g_featureNoType4ReplayUsedGeneration[slot] != gen;
		g_featureDelayedProbeSawType4[slot] = 0u;
		if (arm) {
			g_featureNoType4GraceGeneration[slot] = gen;
			g_featureNoType4GraceState[slot] = 1u;
		}
	} else if (completedPhase == 0u &&
		   g_featureNoType4GraceGeneration[slot] == gen &&
		   g_featureNoType4GraceState[slot] == 1u) {
		// The explicitly allowed ordinary grace poll completed without any Type4.
		g_featureNoType4GraceState[slot] = 2u;
	}

	if (completedPhase == FEATURE_FETCH_FOLLOWUP_POLL &&
	    g_featurePostStaleSilenceTrackGeneration[slot] == gen) {
		bool armPostStaleSilence =
			!g_featurePostStaleSilenceSawType4[slot] &&
			g_featureStaleReplayUsedGeneration[slot] == gen &&
			g_featureNoType4ReplayUsedGeneration[slot] != gen &&
			g_featureRepeatStaleGeneration[slot] != gen &&
			g_featureRepeatStaleUsedGeneration[slot] != gen &&
			g_featurePostStaleSilenceUsedGeneration[slot] != gen;
		featureClearPostStaleSilenceTrack(slot);
		if (armPostStaleSilence) {
			g_featurePostStaleSilenceReplayGeneration[slot] = gen;
		}
	}

	// The no-Type4 replay records ownership before RF I/O and also consumes the
	// shared first late-query replay budget. Its dedicated used-generation marker
	// is authoritative here. A completed follow-up poll with no later Type4 and no
	// prior silence recovery proves the terminal-silence condition.
	if (completedPhase == FEATURE_FETCH_FOLLOWUP_POLL &&
	    g_featureNoType4ReplayUsedGeneration[slot] == gen) {
		bool armPostNoType4Silence =
			g_featurePostNoType4Type4SeenGeneration[slot] != gen &&
			g_featurePostNoType4ReplayUsedGeneration[slot] != gen &&
			g_featurePostNoType4ReplayGeneration[slot] != gen &&
			g_featurePostNoType4StaleGeneration[slot] != gen &&
			g_featurePostNoType4StaleUsedGeneration[slot] != gen &&
			g_featurePostStaleSilenceReplayGeneration[slot] !=
				gen &&
			g_featurePostStaleSilenceUsedGeneration[slot] != gen &&
			g_featureRepeatStaleGeneration[slot] != gen &&
			g_featureRepeatStaleUsedGeneration[slot] != gen;
		if (armPostNoType4Silence) {
			g_featurePostNoType4ReplayGeneration[slot] = gen;
		}

		// A terminal FETCH may be armed only after the full post-replay poll cycle
		// fails to accept the generation and exact-current FETCH evidence was seen.
		if (completedPhase == FEATURE_FETCH_FOLLOWUP_POLL &&
		    g_featureNoType4ReplayUsedGeneration[slot] == gen &&
		    g_featureNoType4CurrentResponseGeneration[slot] == gen &&
		    g_featureTerminalFetchGeneration[slot] != gen &&
		    g_featureTerminalFetchUsedGeneration[slot] != gen &&
		    g_slot[slot].pendingQueryCmd &&
		    g_slot[slot].pendingQueryGeneration == gen &&
		    g_slot[slot].pendingQueryDeadlineMs &&
		    (int32_t)(millis() - g_slot[slot].pendingQueryDeadlineMs) <
			    0) {
			g_featureTerminalFetchGeneration[slot] = gen;
		}
	}
}
bool rfConnFeatureTurnConsumed(uint8_t slot)
{
	return slot < NSLOT && g_featureTurnConsumed[slot] != 0;
}
bool rfConnFlushRelay(uint8_t ch, uint8_t s1)
{
	int cur = (g_curSlot >= 0 && g_curSlot < NSLOT) ? g_curSlot : 0;
	bool quiesceDeferred = false;

	// Timeout retires transaction ownership only; resp remains the last valid value.
	Slot &queryState = g_slot[cur];
	featureObserveConsumedGeneration((uint8_t)cur);
	if (queryState.pendingQueryCmd && queryState.pendingQueryDeadlineMs &&
	    (int32_t)(millis() - queryState.pendingQueryDeadlineMs) >= 0) {
		uint32_t pm = __get_PRIMASK();
		__disable_irq();
		queryState.pendingQueryCmd = 0;
		queryState.pendingQueryGeneration = 0;
		queryState.pendingQueryDeadlineMs = 0;
		queryState.pendingQuerySelectorValid = 0;
		queryState.pendingQueryFailed = 1;
		__set_PRIMASK(pm);
	}
	// Snapshot one entry under a short critical section. relayEnqueue() (the producer) runs on the
	// high-priority usbd task and, when the ring is full, evicts the oldest by advancing g_rqTail itself --
	// the same variable this consumer reads/advances. Reading the entry while a producer could overwrite it
	// is a torn read (and dual-writing g_rqTail desyncs head/tail), so copy the entry out and consume the
	// slot atomically here, then do the (slow) RF TX on the copy with interrupts enabled.
	// FETCH becomes eligible only after one ordinary poll/status turn.
	g_featureTurnConsumed[cur] = 0;
	// After the first query returns zero RX, reserve exactly the first
	// next rfConnFlushRelay scheduler opportunity with no RF call and no post-poll
	// poll, then retry the exact current query once on the second opportunity. The
	// forced-poll helper remains suppressed while the generation latch is set.
	if (g_featureDeferredQueryRetryGeneration[cur] != 0u) {
		uint32_t retryGeneration =
			g_featureDeferredQueryRetryGeneration[cur];
		if (!queryState.pendingQueryCmd ||
		    queryState.pendingQueryGeneration != retryGeneration) {
			featureClearDeferredQueryRetry((uint8_t)cur);
		} else if (g_featureDeferredQueryRetryWaitTurns[cur] != 0u) {
			g_featureDeferredQueryRetryWaitTurns[cur]--;
			g_featureTurnConsumed[cur] = 1;

			// feature turn consumed; intentionally no rfConnTx() in the wait slot
			return false;
		} else {
			uint8_t retryLen = g_featureDeferredQueryRetryLen[cur];
			// Clear before RF I/O: even another rxlen==0 cannot schedule attempt #3.
			featureClearDeferredQueryRetry((uint8_t)cur);
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			bool coframeAuthDeferred =
				g_featureCoframeGeneration[cur] ==
					retryGeneration &&
				queryState.pendingQueryCmd &&
				queryState.pendingQueryGeneration ==
					retryGeneration;
			if (coframeAuthDeferred) {
				g_featureFetchPhase[cur] =
					FEATURE_FETCH_FOLLOWUP_POLL;
			}
			rfConnTx(ch, s1,
				 g_featureDeferredQueryRetryPayload[cur],
				 retryLen, 400);
			rfTimingEndFeatureResponse();
			if (coframeAuthDeferred) {
				if (!queryState.pendingQueryCmd ||
				    queryState.pendingQueryGeneration !=
					    retryGeneration) {
					g_featureCoframeGeneration[cur] = 0u;
					g_featureFetchPhase[cur] = 0u;
				} else {
					g_featureFetchPhase[cur] =
						FEATURE_FETCH_WAIT_POLL;
				}
			}
			return true;
		}
	}
	// Service only the NORMAL initial FETCH retry latch. The first later turn is
	// observation-only; the second later turn retries the exact FETCH once using
	// the scheduler-provided S1/PID. The post-poll phase remains armed for a later turn.
	if (g_featureDeferredFetchRetryGeneration[cur] != 0u) {
		uint32_t retryGeneration =
			g_featureDeferredFetchRetryGeneration[cur];
		if (!queryState.pendingQueryCmd ||
		    queryState.pendingQueryGeneration != retryGeneration) {
			featureClearDeferredFetchRetry((uint8_t)cur);
		} else if (g_featureDeferredFetchRetryWaitTurns[cur] != 0u) {
			g_featureDeferredFetchRetryWaitTurns[cur]--;
			g_featureTurnConsumed[cur] = 1;

			// reserved wait turn: no RF feature action
			return false;
		} else {
			uint8_t retryFetch[4];
			memcpy(retryFetch,
			       g_featureDeferredFetchRetryPayload[cur],
			       sizeof retryFetch);
			// Clear before RF I/O so another rxlen==0 can never schedule attempt #3.
			featureClearDeferredFetchRetry((uint8_t)cur);
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			rfConnTx(ch, s1, retryFetch, sizeof retryFetch, 400);
			rfTimingEndFeatureResponse();

			return true;
		}
	}
	// A valid stale Type4 proves prior-command response state while the current
	// generation is still pending. Reassert the original query once, then restart
	// only the existing poll/FETCH sequence without resetting its deadline or
	// delayed-probe ownership.
	if (g_featureStaleReplayGeneration[cur] != 0u) {
		uint32_t replayGeneration = g_featureStaleReplayGeneration[cur];
		if (!queryState.pendingQueryCmd ||
		    queryState.pendingQueryGeneration != replayGeneration ||
		    g_featureQuerySnapshotGeneration[cur] != replayGeneration ||
		    !g_featureQuerySnapshotLen[cur]) {
			featureClearStaleQueryReplay((uint8_t)cur);
		} else {
			uint8_t replay[80];
			uint8_t replayLen = g_featureQuerySnapshotLen[cur];
			memcpy(replay, g_featureQuerySnapshotPayload[cur],
			       replayLen);
			// The stale mismatch already passed structural validation. Track the
			// replay for a possible silence recovery unless that recovery budget was
			// already consumed for this generation.
			if (g_featurePostStaleSilenceUsedGeneration[cur] !=
				    replayGeneration &&
			    g_featureRepeatStaleUsedGeneration[cur] !=
				    replayGeneration) {
				g_featurePostStaleSilenceTrackGeneration[cur] =
					replayGeneration;
				g_featurePostStaleSilenceSawType4[cur] = 0u;
			} else {
				featureClearPostStaleSilenceTrack((uint8_t)cur);
			}
			// Clear and mark used before RF I/O: this generation can never receive
			// another stale-triggered current-query replay, even if the replay returns zero.
			featureClearStaleQueryReplay((uint8_t)cur);
			g_featureStaleReplayUsedGeneration[cur] =
				replayGeneration;
			g_featureFetchPhase[cur] = FEATURE_FETCH_WAIT_POLL;
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			rfConnTx(ch, s1, replay, replayLen, 400);
			rfTimingEndFeatureResponse();

			return true;
		}
	}
	if (g_featureNoType4GraceGeneration[cur] != 0u) {
		uint32_t pgen = g_featureNoType4GraceGeneration[cur];
		if (!queryState.pendingQueryCmd ||
		    queryState.pendingQueryGeneration != pgen ||
		    g_featureQuerySnapshotGeneration[cur] != pgen ||
		    !g_featureQuerySnapshotLen[cur] ||
		    g_featureStaleReplayGeneration[cur] == pgen ||
		    g_featureStaleReplayUsedGeneration[cur] == pgen ||
		    g_featureNoType4ReplayUsedGeneration[cur] == pgen) {
			featureClearNoType4Grace((uint8_t)cur);
		} else if (g_featureNoType4GraceState[cur] == 1u) {
			// Grace turn is observation-only from the feature layer: deliberately do
			// NOT consume the turn so the caller performs one ordinary RF poll.

			return false;
		} else if (g_featureNoType4GraceState[cur] == 2u) {
			uint8_t replay[80];
			uint8_t replayLen = g_featureQuerySnapshotLen[cur];
			memcpy(replay, g_featureQuerySnapshotPayload[cur],
			       replayLen);
			// The no-Type4 replay consumes the shared late-query reassertion budget
			// before RF I/O, so this generation cannot take another stale-response replay from the shared budget.
			featureClearNoType4Grace((uint8_t)cur);
			g_featureNoType4ReplayUsedGeneration[cur] = pgen;
			g_featureStaleReplayUsedGeneration[cur] = pgen;
			g_featureFetchPhase[cur] = FEATURE_FETCH_WAIT_POLL;
			g_featureTurnConsumed[cur] = 1;

			g_featureNoType4ReplayActiveGeneration[cur] = pgen;
			rfTimingBeginFeatureResponse();
			rfConnTx(ch, s1, replay, replayLen, 400);
			rfTimingEndFeatureResponse();
			g_featureNoType4ReplayActiveGeneration[cur] = 0u;

			return true;
		}
	}
	// After one stale-triggered current-query replay, a later exact stale mismatch is fresh
	// evidence that the stale tail persisted, so permit one additional
	// one additional bounded current-query replay on this scheduler opportunity.
	if (g_featureRepeatStaleGeneration[cur] != 0u) {
		uint32_t repeatedStaleGeneration =
			g_featureRepeatStaleGeneration[cur];
		if (!queryState.pendingQueryCmd ||
		    queryState.pendingQueryGeneration !=
			    repeatedStaleGeneration ||
		    g_featureQuerySnapshotGeneration[cur] !=
			    repeatedStaleGeneration ||
		    !g_featureQuerySnapshotLen[cur] ||
		    g_featureStaleReplayUsedGeneration[cur] !=
			    repeatedStaleGeneration ||
		    g_featureNoType4ReplayUsedGeneration[cur] ==
			    repeatedStaleGeneration ||
		    g_featureRepeatStaleUsedGeneration[cur] ==
			    repeatedStaleGeneration) {
			featureClearRepeatStaleReplay((uint8_t)cur);
		} else {
			uint8_t replay[80];
			uint8_t replayLen = g_featureQuerySnapshotLen[cur];
			memcpy(replay, g_featureQuerySnapshotPayload[cur],
			       replayLen);
			// Mark used before RF I/O so this generation cannot arm another replay.
			featureClearRepeatStaleReplay((uint8_t)cur);
			g_featureRepeatStaleUsedGeneration[cur] =
				repeatedStaleGeneration;
			g_featureFetchPhase[cur] = FEATURE_FETCH_WAIT_POLL;
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			rfConnTx(ch, s1, replay, replayLen, 400);
			rfTimingEndFeatureResponse();

			return true;
		}
	}
	// Repeated-stale recovery has priority. Silence recovery requires a prior
	// stale-triggered replay followed by a complete poll cycle with no Type4.
	// The stale replay originated from a reason-0x20 mismatch. Repeated-stale and
	// silence recovery share one additional-replay budget.
	if (g_featurePostStaleSilenceReplayGeneration[cur] != 0u) {
		uint32_t postStaleSilenceGeneration =
			g_featurePostStaleSilenceReplayGeneration[cur];
		if (!queryState.pendingQueryCmd ||
		    queryState.pendingQueryGeneration !=
			    postStaleSilenceGeneration ||
		    g_featureQuerySnapshotGeneration[cur] !=
			    postStaleSilenceGeneration ||
		    !g_featureQuerySnapshotLen[cur] ||
		    g_featureStaleReplayUsedGeneration[cur] !=
			    postStaleSilenceGeneration ||
		    g_featureNoType4ReplayUsedGeneration[cur] ==
			    postStaleSilenceGeneration ||
		    g_featureRepeatStaleGeneration[cur] ==
			    postStaleSilenceGeneration ||
		    g_featureRepeatStaleUsedGeneration[cur] ==
			    postStaleSilenceGeneration ||
		    g_featurePostStaleSilenceUsedGeneration[cur] ==
			    postStaleSilenceGeneration) {
			featureClearPostStaleSilenceReplay((uint8_t)cur);
		} else {
			uint8_t replay[80];
			uint8_t replayLen = g_featureQuerySnapshotLen[cur];
			memcpy(replay, g_featureQuerySnapshotPayload[cur],
			       replayLen);
			// Consume both additional recovery budgets before RF I/O.
			featureClearPostStaleSilenceReplay((uint8_t)cur);
			g_featurePostStaleSilenceUsedGeneration[cur] =
				postStaleSilenceGeneration;
			g_featureRepeatStaleUsedGeneration[cur] =
				postStaleSilenceGeneration;
			g_featureFetchPhase[cur] = FEATURE_FETCH_WAIT_POLL;
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			uint8_t postStaleSilenceRx =
				rfConnTx(ch, s1, replay, replayLen, 400);
			rfTimingEndFeatureResponse();

			// A zero RX is transport silence, not proof that the query was not received.
			// Reusing the exact same s1/PID makes this one ESB retransmission-safe attempt;
			// no new scheduler stage, query generation, or deadline is created.
			if (postStaleSilenceRx == 0 &&
			    queryState.pendingQueryCmd &&
			    queryState.pendingQueryGeneration ==
				    postStaleSilenceGeneration &&
			    queryState.pendingQueryDeadlineMs &&
			    (int32_t)(millis() -
				      queryState.pendingQueryDeadlineMs) < 0) {
				rfTimingBeginFeatureResponse();
				rfConnTx(ch, s1, replay, replayLen, 400);
				rfTimingEndFeatureResponse();
			}
			return true;
		}
	}
	// A stale mismatch after the no-Type4 replay is handled before pure-silence
	// recovery. Pure silence cannot arm this branch, and a generation that already
	// used silence recovery cannot acquire another recovery stage.
	if (g_featurePostNoType4StaleGeneration[cur] != 0u) {
		uint32_t postNoType4StaleGeneration =
			g_featurePostNoType4StaleGeneration[cur];
		if (!queryState.pendingQueryCmd ||
		    queryState.pendingQueryGeneration !=
			    postNoType4StaleGeneration ||
		    g_featureQuerySnapshotGeneration[cur] !=
			    postNoType4StaleGeneration ||
		    !g_featureQuerySnapshotLen[cur] ||
		    g_featureNoType4ReplayUsedGeneration[cur] !=
			    postNoType4StaleGeneration ||
		    g_featurePostNoType4ReplayUsedGeneration[cur] ==
			    postNoType4StaleGeneration ||
		    g_featurePostNoType4StaleUsedGeneration[cur] ==
			    postNoType4StaleGeneration ||
		    g_featurePostStaleSilenceReplayGeneration[cur] ==
			    postNoType4StaleGeneration ||
		    g_featurePostStaleSilenceUsedGeneration[cur] ==
			    postNoType4StaleGeneration ||
		    g_featureRepeatStaleGeneration[cur] ==
			    postNoType4StaleGeneration ||
		    g_featureRepeatStaleUsedGeneration[cur] ==
			    postNoType4StaleGeneration) {
			featureClearPostNoType4StaleReplay((uint8_t)cur);
		} else {
			uint8_t replay[80];
			uint8_t replayLen = g_featureQuerySnapshotLen[cur];
			memcpy(replay, g_featureQuerySnapshotPayload[cur],
			       replayLen);
			// Mark used before RF I/O so no later recovery branch can recurse.
			featureClearPostNoType4StaleReplay((uint8_t)cur);
			g_featurePostNoType4StaleUsedGeneration[cur] =
				postNoType4StaleGeneration;
			g_featureFetchPhase[cur] = FEATURE_FETCH_WAIT_POLL;
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			rfConnTx(ch, s1, replay, replayLen, 400);
			rfTimingEndFeatureResponse();

			return true;
		}
	}
	// The no-Type4 replay already consumed the shared first late-query budget.
	// Silence recovery therefore keys off its dedicated ownership marker and runs
	// only after the complete post-replay poll cycle remains silent.
	if (g_featurePostNoType4ReplayGeneration[cur] != 0u) {
		uint32_t postNoType4SilenceGeneration =
			g_featurePostNoType4ReplayGeneration[cur];
		if (!queryState.pendingQueryCmd ||
		    queryState.pendingQueryGeneration !=
			    postNoType4SilenceGeneration ||
		    g_featureQuerySnapshotGeneration[cur] !=
			    postNoType4SilenceGeneration ||
		    !g_featureQuerySnapshotLen[cur] ||
		    g_featureNoType4ReplayUsedGeneration[cur] !=
			    postNoType4SilenceGeneration ||
		    g_featurePostNoType4Type4SeenGeneration[cur] ==
			    postNoType4SilenceGeneration ||
		    g_featurePostNoType4ReplayUsedGeneration[cur] ==
			    postNoType4SilenceGeneration ||
		    g_featurePostStaleSilenceReplayGeneration[cur] ==
			    postNoType4SilenceGeneration ||
		    g_featurePostStaleSilenceUsedGeneration[cur] ==
			    postNoType4SilenceGeneration ||
		    g_featureRepeatStaleGeneration[cur] ==
			    postNoType4SilenceGeneration ||
		    g_featureRepeatStaleUsedGeneration[cur] ==
			    postNoType4SilenceGeneration) {
			featureClearPostNoType4Replay((uint8_t)cur);
		} else {
			uint8_t replay[80];
			uint8_t replayLen = g_featureQuerySnapshotLen[cur];
			memcpy(replay, g_featureQuerySnapshotPayload[cur],
			       replayLen);
			// Mark used before RF I/O so a silent attempt cannot recursively re-arm.
			featureClearPostNoType4Replay((uint8_t)cur);
			g_featurePostNoType4ReplayUsedGeneration[cur] =
				postNoType4SilenceGeneration;
			// Present FOLLOWUP_POLL only during the first silence-replay RF call so an
			// exact-current Type4 returned synchronously can still be accepted.
			g_featureFetchPhase[cur] = FEATURE_FETCH_FOLLOWUP_POLL;
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			uint8_t postNoType4SilenceRx =
				rfConnTx(ch, s1, replay, replayLen, 400);
			rfTimingEndFeatureResponse();

			if (queryState.pendingQueryCmd &&
			    queryState.pendingQueryGeneration ==
				    postNoType4SilenceGeneration) {
				g_featureFetchPhase[cur] =
					FEATURE_FETCH_WAIT_POLL;
			}
			// A zero RX is transport silence, not proof that the query was not received.
			// Reusing the exact same s1/PID makes this one ESB retransmission-safe attempt;
			// no new scheduler stage, query generation, or deadline is created.
			if (postNoType4SilenceRx == 0 &&
			    queryState.pendingQueryCmd &&
			    queryState.pendingQueryGeneration ==
				    postNoType4SilenceGeneration &&
			    queryState.pendingQueryDeadlineMs &&
			    (int32_t)(millis() -
				      queryState.pendingQueryDeadlineMs) < 0) {
				rfTimingBeginFeatureResponse();
				rfConnTx(ch, s1, replay, replayLen, 400);
				rfTimingEndFeatureResponse();
			}
			return true;
		}
	}
	if (g_featureTerminalFetchGeneration[cur] != 0u) {
		uint32_t terminalFetchGeneration =
			g_featureTerminalFetchGeneration[cur];
		if (!queryState.pendingQueryCmd ||
		    queryState.pendingQueryGeneration !=
			    terminalFetchGeneration ||
		    g_featureNoType4ReplayUsedGeneration[cur] !=
			    terminalFetchGeneration ||
		    g_featureNoType4CurrentResponseGeneration[cur] !=
			    terminalFetchGeneration ||
		    g_featureTerminalFetchUsedGeneration[cur] ==
			    terminalFetchGeneration ||
		    !queryState.pendingQueryDeadlineMs ||
		    (int32_t)(millis() - queryState.pendingQueryDeadlineMs) >=
			    0) {
			g_featureTerminalFetchGeneration[cur] = 0u;
		} else {
			uint8_t fetch[4] = { 0xE3, 0x01, 0x03, 0x00 };
			g_featureTerminalFetchGeneration[cur] = 0u;
			g_featureTerminalFetchUsedGeneration[cur] =
				terminalFetchGeneration;
			// FOLLOWUP_POLL makes rfConnFeatureFetchPending() false during this terminal
			// FETCH, matching the normal response-acceptance state.
			g_featureFetchPhase[cur] = FEATURE_FETCH_FOLLOWUP_POLL;
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			rfConnTx(ch, s1, fetch, sizeof fetch, 400);
			rfTimingEndFeatureResponse();

			return true;
		}
	}
	// After the normal FETCH/follow-up-poll sequence has fully
	// returned to phase 0, give this still-pending generation exactly one later
	// retrieval opportunity on the first scheduler turn at/after +24 ms from the original query.
	if (queryState.pendingQueryCmd &&
	    queryState.pendingQueryGeneration != 0u &&
	    g_featureFetchPhase[cur] == 0u && g_featureQueryArmMs[cur] != 0u &&
	    g_featureDelayedFetchGeneration[cur] !=
		    queryState.pendingQueryGeneration) {
		uint32_t delayedFetchAgeMs =
			(uint32_t)(millis() - g_featureQueryArmMs[cur]);
		if (delayedFetchAgeMs >= FEATURE_DELAYED_FETCH_DELAY_MS) {
			uint8_t fetch[4] = { 0xE3, 0x01, 0x03, 0x00 };
			g_featureDelayedFetchGeneration[cur] =
				queryState.pendingQueryGeneration;
			// Reuse FOLLOWUP_POLL so the next RF turn is guaranteed to be an
			// ordinary poll after the delayed retrieval probe as well.
			g_featureFetchPhase[cur] = FEATURE_FETCH_FOLLOWUP_POLL;
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			uint8_t fetchRx =
				rfConnTx(ch, s1, fetch, sizeof fetch, 400);
			rfTimingEndFeatureResponse();
			// Preserve one immediate same-PID retransmission for this FETCH invocation:
			// allow one retry after rxlen==0, but never another delayed probe.
			if (fetchRx == 0) {
				rfTimingBeginFeatureResponse();
				fetchRx = rfConnTx(ch, s1, fetch, sizeof fetch,
						   400);
				rfTimingEndFeatureResponse();
			}
			return true;
		}
	}
	if (rfConnFeatureFetchEligible((uint8_t)cur)) {
		if (!queryState.pendingQueryCmd) {
			g_featureFetchPhase[cur] = 0;
		} else {
			uint8_t fetch[4] = { 0xE3, 0x01, 0x03, 0x00 };
			// Clear the pre-FETCH guard before parsing this RF return. A matching Type-4
			// returned by FETCH itself is therefore eligible for the active generation.
			g_featureFetchPhase[cur] = FEATURE_FETCH_FOLLOWUP_POLL;
			g_featureTurnConsumed[cur] = 1;

			rfTimingBeginFeatureResponse();
			uint8_t fetchRx =
				rfConnTx(ch, s1, fetch, sizeof fetch, 400);
			rfTimingEndFeatureResponse();
			// A zero-length return from the initial FETCH reserves a no-RF wait turn
			// followed by one same-S1/PID retry. Snapshot the exact FETCH and reserve
			// one no-RF wait slot plus one deferred retry. The existing FOLLOWUP_POLL
			// phase remains armed throughout and the original query deadline is unchanged.
			if (fetchRx == 0) {
				g_featureDeferredFetchRetryGeneration[cur] =
					queryState.pendingQueryGeneration;
				g_featureDeferredFetchRetryWaitTurns[cur] = 1u;
				memcpy(g_featureDeferredFetchRetryPayload[cur],
				       fetch, sizeof fetch);
			}
			return true;
		}
	}
	RelayMsg msg;
	bool have = false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t guard = RELAY_QLEN + 1;
	while (g_rqTail[cur] != g_rqHead[cur]) {
		// desynced/corrupt ring -> recover, don't spin IRQs-off (watchdog-hang class)
		if (!guard--) {
			g_rqHead[cur] = g_rqTail[cur] = 0;
			g_ringFault++;
			faultDiagTrace(FR_RINGF, g_ringFault);
			break;
		}
		RelayMsg &m = g_rq[cur][g_rqTail[cur]];
		// The relay queue is also the feature-query FIFO. Leave the next expectReply
		// entry in place until the active controller query completes/fails.
		// A completed but unconsumed Type4 response still owns the feature slot.
		// Leave the next expectReply entry in the FIFO until the host consumes it.
		if (m.rid && m.expectReply &&
		    (g_slot[cur].pendingQueryCmd != 0 ||
		     g_slot[cur].queryResponseReady))
			break;
		if (m.rid && m.expectReply &&
		    g_featureLastConsumedCmd[cur] != 0u &&
		    m.rid != g_featureLastConsumedCmd[cur] &&
		    g_featureQuiesceStartMs[cur] != 0u) {
			uint32_t now = millis();
			uint32_t elapsed =
				(uint32_t)(now - g_featureQuiesceStartMs[cur]);
			if (elapsed < FEATURE_CROSS_COMMAND_QUIESCE_MS) {
				quiesceDeferred = true;

				// leave the response-bearing entry in the FIFO
				break;
			}
		}
		g_rqTail[cur] = rqNext(g_rqTail[cur]); // consume the slot
		// rid 0 = entry voided by hapticCancelPendingOn -> skip
		if (m.rid) {
			// copy out before the producer can reuse the slot
			msg = m;
			have = true;
			break;
		}
	}
	__set_PRIMASK(pm);
	if (quiesceDeferred) {
		// Caller performs the ordinary poll turn while the query stays queued.
		return false;
	}
	if (have) {
		RelayMsg &m = msg;
		{
			uint8_t rl = m.len;
			if (rl > RELAY_MAXP)
				rl = RELAY_MAXP;
			// Configuration commands require the type-01 + inner-length form
			// E3 [2+rl][01][rid][innerlen][data]. Haptic reports use the type-05 form below.

			// Same shape as the `[len][tag][value]` TLV grammar the F1 REPLY side
			// already uses (tags 0x02/0x04/0x06, docs/PROTOCOL.md sec 7.3): read as len=1, tag=3,
			// value=0. If the caller decides they want an answer, we add the queryTrailer.
			// Should check later if there's a way to somehow determine which types need the trailer and which ones don't.
			// Preserve co-framing as the primary query form.
			bool queryTrailer = m.expectReply;
			uint8_t p[5 + RELAY_MAXP + 3], plen;
			if (!m.isHaptic) {
				// Non-haptics use type 01
				p[0] = g_relayOp;
				p[1] = (uint8_t)(2 + rl);
				p[2] = 0x01;
				p[3] = m.rid;
				p[4] = rl;
				memcpy(p + 5, m.data, rl);
				plen = (uint8_t)(5 + rl);
				if (queryTrailer && plen <= sizeof p - 3u) {
					p[plen + 0] = 0x01;
					p[plen + 1] = 0x03;
					p[plen + 2] = 0x00;
					plen = (uint8_t)(plen + 3);
				}
			} else {
				// Haptics use type 05
				p[0] = g_relayOp;
				p[1] = (uint8_t)(1 + rl);
				p[2] = 0x05;
				p[3] = m.rid;
				memcpy(p + 4, m.data, rl);
				plen = (uint8_t)(4 + rl);
			}
			// Lifecycle log (CDC debug boot): EXACTLY what we TX to the controller, so a capture shows
			// whether OpenPuck (or Steam in the background) pokes the controller right before a buzz
			// latches -- the piece the I45 stream (controller->us) can't show. Low rate outside a haptic
			// burst; guarded so it never blocks the loop. cur = target slot.
			hapLogAdd(0xFE, m.rid, m.data, rl);
			if (Serial.availableForWrite() > 60) {
				// boosted: this print runs at relay-flood rate and CDC flush enters the same
				// TinyUSB DMA claim window as HID sends (the issue-72 livelock; see usb_tx.cpp)
				usbTxBoost();
				Serial.printf("# TX t=%lu slot%d %s rid=%02X:",
					      (unsigned long)millis(), cur,
					      m.isHaptic ? "L05" : "L01",
					      m.rid);
				for (uint8_t i = 0; i < rl && i < 8; i++)
					Serial.printf(" %02X", m.data[i]);
				Serial.println();
				usbTxUnboost();
			}
			// Transaction ownership starts at the actual RF transmission. This prevents a later
			// USB query from overwriting the command currently awaiting Type-4 data.
			if (m.expectReply) {
				uint32_t pm = __get_PRIMASK();
				__disable_irq();
				queryState.pendingQueryCmd = m.rid;
				queryState.pendingQueryGeneration =
					m.queryGeneration;
				queryState.pendingQueryFailed = 0;
				queryState.pendingQueryDeadlineMs =
					millis() + FEATURE_QUERY_TIMEOUT_MS;
				g_featureLastArmedGeneration[cur] =
					m.queryGeneration;
				g_featureLastArmedCmd[cur] = m.rid;
				g_featureQueryArmMs[cur] = millis();
				g_featureDelayedFetchGeneration[cur] = 0u;
				g_featureNoType4ReplayActiveGeneration[cur] =
					0u;
				g_featureNoType4CurrentResponseGeneration[cur] =
					0u;
				g_featureTerminalFetchGeneration[cur] = 0u;
				g_featureTerminalFetchUsedGeneration[cur] = 0u;
				featureClearDeferredQueryRetry((uint8_t)cur);
				featureClearDeferredFetchRetry((uint8_t)cur);
				featureResetQuerySnapshot((uint8_t)cur);
				featureResetNoType4Grace((uint8_t)cur);
				featureResetRepeatStaleReplay((uint8_t)cur);
				featureResetPostStaleSilence((uint8_t)cur);
				featureResetPostNoType4((uint8_t)cur);
				featureResetPostNoType4StaleReplay(
					(uint8_t)cur);
				g_featureQuerySnapshotGeneration[cur] =
					m.queryGeneration;
				g_featureQuerySnapshotLen[cur] = plen;
				memcpy(g_featureQuerySnapshotPayload[cur], p,
				       plen);
				queryState.pendingQuerySelectorValid =
					(m.rid ==
						 IBEX_CMD_GET_STRING_ATTRIBUTE &&
					 rl >= 1) ?
						1 :
						0;
				queryState.pendingQuerySelector =
					queryState.pendingQuerySelectorValid ?
						m.data[0] :
						0;
				__set_PRIMASK(pm);
				// bit0: 01 03 00 co-frame present
				rfTimingBeginFeatureResponse();
			}
			if (!m.isHaptic &&
			    m.rid == IBEX_CMD_TURN_OFF_CONTROLLER)
				queryState.shutdownStatusOwnerMs = millis();
			// slot was already consumed under the critical section above.
			// s1 carries a PID distinct from the GET poll (caller cycles it) so the controller's ESB
			// dedup never treats the GET as a retransmit of this relay.
			//
			// A relay can return the controller's current input in its ACK payload. Decode that reply
			// through the normal F1 path so relay-heavy periods do not discard usable input samples;
			// sequence deduplication prevents double-forwarding. The bounded receive window limits the
			// airtime cost when no reply is present.
			if (m.expectReply) {
				uint32_t coframeGeneration = m.queryGeneration;
				g_featureCoframeGeneration[cur] =
					coframeGeneration;
				g_featureFetchPhase[cur] =
					FEATURE_FETCH_FOLLOWUP_POLL;

				uint8_t queryRx =
					rfConnTx(ch, s1, p, plen, 400);
				rfTimingEndFeatureResponse();

				if (!queryState.pendingQueryCmd ||
				    queryState.pendingQueryGeneration !=
					    coframeGeneration) {
					g_featureCoframeGeneration[cur] = 0u;
					g_featureFetchPhase[cur] = 0u;
				} else {
					// The post-call fallback arm below remains authoritative.
					g_featureFetchPhase[cur] = 0u;
				}
				// Preserve the two-attempt maximum. Snapshot the
				// exact current-query bytes now, reserve one later scheduler opportunity with no RF,
				// and permit attempt #2 only on the following scheduler opportunity.
				if (queryRx == 0) {
					g_featureDeferredQueryRetryGeneration
						[cur] = m.queryGeneration;
					g_featureDeferredQueryRetryLen[cur] =
						plen;
					g_featureDeferredQueryRetryWaitTurns[cur] =
						1u;
					memcpy(g_featureDeferredQueryRetryPayload
						       [cur],
					       p, plen);
				}
			} else {
				rfConnTx(ch, s1, p, plen, 400);
			}
			// Preserve one-feature-turn ownership. Only unresolved
			// co-framed queries enter the normal split-FETCH fallback path.
			if (m.expectReply) {
				g_featureTurnConsumed[cur] = 1;
				if (queryState.pendingQueryCmd &&
				    queryState.pendingQueryGeneration ==
					    m.queryGeneration)
					g_featureFetchPhase[cur] =
						FEATURE_FETCH_WAIT_POLL;
				else
					g_featureFetchPhase[cur] = 0;
			}
		}
	}

	// true = a relay frame went out this cycle (its reply is harvested as input, above)
	return have;
}

void hapticReinit(uint8_t slot)
{
	// TODO: This function is only ever called from WebUSB in debug mode
	// when the user clicks the "Clear stuck buzz" button.
	// I believe with MR 230 the haptics issues should be gone,
	// so this can probably be removed?

	// clang-format off
	static const uint8_t haptic_reset_data_1[] = { 
			SETTING_LEFT_TRACKPAD_MODE, 0x07, 0x00, 
			SETTING_RIGHT_TRACKPAD_MODE, 0x07, 0x00, 
			SETTING_WIRELESS_PACKET_VERSION, 0x02, 0x00, 
			SETTING_COUNT, 0x03, 0x00 };
	
	static const uint8_t haptic_reset_data_2[] = { 
			SETTING_SMOOTH_ABSOLUTE_MOUSE, 0x00, 0x00, 
			SETTING_ENABLE_RAW_JOYSTICK, 0x00, 0x00, 
			SETTING_LEFT_TRACKPAD_CLICK_PRESSURE, 0xff, 0xff, 
			SETTING_RIGHT_TRACKPAD_CLICK_PRESSURE, 0xff, 0xff, 
			SETTING_LEFT_TRACKPAD_CLICK_PRESSURE, 0xff, 0xff };

	static const uint8_t haptic_reset_data_3[] = { 
			SETTING_RIGHT_TRACKPAD_CLICK_PRESSURE, 0xff, 0xff, 
			SETTING_ENABLE_RAW_JOYSTICK, 0x00, 0x00 };

	// clang-format on

	static const uint8_t T81A[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	static const uint8_t T81B[] = {
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	// reset action (FUN_0001f554) -- Steam sends this first
	relayEnqueue(0x81, nullptr, 0, true, slot);

	// This is probably split into three commands because you can only update
	// five parameters in one batch.
	relayEnqueue(IBEX_CMD_SET_SETTINGS_VALUES, haptic_reset_data_1,
		     sizeof haptic_reset_data_1, false, slot);
	relayEnqueue(IBEX_CMD_SET_SETTINGS_VALUES, haptic_reset_data_2,
		     sizeof haptic_reset_data_2, false, slot);
	relayEnqueue(IBEX_CMD_SET_SETTINGS_VALUES, haptic_reset_data_3,
		     sizeof haptic_reset_data_3, false, slot);

	// explicitly trigger an empty HAPTIC_PULSE: the part that clears a latch
	relayEnqueue(0x81, T81A, sizeof T81A, true, slot);
	relayEnqueue(0x81, T81B, sizeof T81B, true, slot);

	// Apply the configured LED brightness for the active emulated type. Steam sets the
	// brightness each session; emulated modes never do, so the controller comes up at
	// full brightness. 0 = no override (preserve controller default).
	// TODO: This has nothing to do with haptics, why is this here?
	if (g_etype < ET_COUNT && g_ledBright > 0) {
		uint8_t pl[3] = { SETTING_LED_USER_BRIGHTNESS, g_ledBright,
				  0x00 };
		relayEnqueue(IBEX_CMD_SET_SETTINGS_VALUES, pl, sizeof pl, false,
			     slot);
	}
}
void hapticInit()
{
	g_hapticStop = 0;
	for (int s = 0; s < NSLOT; s++) {
		g_rqHead[s] = g_rqTail[s] = 0;
		g_rumble80On[s] = false;
		g_rumble80Low[s] = 0;
		g_rumble80High[s] = 0;
		g_rumble80Ms[s] = 0;
		// post-connect haptic block is permanently disabled (not armed, not configurable)
		g_hapticBlockUntil[s] = 0;
	}
}
// Schedule the clearing re-init for ONE slot. Called on the reliable first-reply signal from rf_link and
// again from hapticTask's link-up edge detector. Strictly per-slot: only the reconnected controller gets
// re-inits / its pending haptics scrubbed -- the other controllers hear nothing about it.
void hapticOnReconnect(int slot)
{
	if (slot < 0 || slot >= NSLOT)
		return;
	// post-connect haptic block is permanently disabled -- relay haptics immediately on (re)connect
	g_hapticBlockUntil[slot] = 0;
	g_rumble80On[slot] = false;
	g_rumble80Low[slot] = 0;
	g_rumble80High[slot] = 0;
	g_rumble80Ms[slot] = 0;
	// Scrub haptics queued before the link came up (stale across the reconnect) -- this slot only.
	hapticCancelPendingOn(slot);
	if (g_usbMode == MODE_SW_PRO) {
		static const uint8_t RAW_IMU[] = {
			SETTING_IMU_MODE,
			SETTING_GYRO_MODE_SEND_RAW_ACCEL |
				SETTING_GYRO_MODE_SEND_RAW_GYRO,
			0x00
		};

		// Steam can leave the controller's persisted IMU mode off. Switch
		// Pro reports require the raw RF samples that setting suppresses.
		relayEnqueue(IBEX_CMD_SET_SETTINGS_VALUES, RAW_IMU,
			     sizeof RAW_IMU, false, (uint8_t)slot);
	}
	// NO automatic haptic re-init. hapticReinit lands 0x81 (CLEAR DIGITAL MAPPINGS -- rid < 0x87 so it
	// EXECUTES even on legacy framing). 0x81 is NON-IDEMPOTENT (ibex FUN_0001f554): EVERY call re-runs the
	// lizard-disable event + func_0x0001bbf0 (a hardware peripheral re-arm unique to the 0x81 path) -- so
	// firing 8 across the connect window = 8 audible clicks = the "repeated non-periodic clicks at connect"
	// the real puck (which clears ONCE then holds) never produces. The clearing re-init stays available on
	// demand (WebUSB "Clear stuck buzz" -> hapticReinit), just not sprayed at connect.
	uint8_t mk = 2;
	hapLogAdd(0xFD, 0xEE, &mk, 1);
}
// Panel/console rumble test: buzz every connected controller at RUMBLE_TEST_AMP through the normal
// hapticSteamRumble() path (so g_rumbleStyle / g_rumbleScale / the per-type rumble toggle all apply), and
// schedule the stop. Deadline 0 = idle; millis()+MS can be 0 only once every 49 days, so bias it off zero.
static unsigned long g_rumbleTestStop = 0;

void hapticTestRumble()
{
	for (uint8_t s = 0; s < NSLOT; s++)
		if (hapticLinkUp((int)s))
			hapticSteamRumble(RUMBLE_TEST_AMP, RUMBLE_TEST_AMP, s);
	g_rumbleTestStop = millis() + RUMBLE_TEST_MS;
	if (!g_rumbleTestStop)
		g_rumbleTestStop = 1;
}

void hapticTask()
{
	// stop the test buzz -- signed compare so the millis() rollover cannot strand a latched actuator
	if (g_rumbleTestStop && (long)(millis() - g_rumbleTestStop) >= 0) {
		g_rumbleTestStop = 0;
		for (uint8_t s = 0; s < NSLOT; s++)
			hapticSteamRumble(0, 0, s);
	}
	// id9 steering (SET_SETTINGS index 9 = digital-mappings / the controller's AUTONOMOUS mapping+haptic
	// engine, which is what generates the trackpad tick haptics). We decide per mode whether that autonomous
	// engine should be ON, then either land id9=1 ONCE per connect episode (engine on) or hold id9=0 every
	// LIZKEEP_MS (engine off). id9 gates the whole autonomous pad layer INCLUDING the trackpad ticks; 0x87 is
	// change-guarded in the controller so a repeated same-value write is silent (no 0x81-style click), and
	// re-landing id9=1 on (re)connect restores the pad layer immediately instead of waiting for the
	// controller's revert timer (which also resets all settings = audible pop).
	//
	// wantAuto = should the controller run its own pad layer (trackpad ticks) for the ACTIVE mode?
	//  - puck modes (STEAM/LIZARD): NOT STEERED AT ALL. Steam owns the controller's haptics in puck mode
	//    (it writes id9 itself), so the whole block below is skipped -- we used to drive id9 from
	//    puckLizardActive() here, which fought Steam's own writes.
	//  - emulated modes (Xbox/Switch/DS): follow the per-type trackpad-haptics config g_padHaptics (default
	//    ON; Switch defaults OFF). Holding id9=0 is how we turn the controller's autonomous trackpad
	//    haptics OFF for a type that doesn't want them.
	if (g_lizKeep) {
		static unsigned long lastKeep[NSLOT] = { 0 };
		static bool landedAuto[NSLOT] = { false };
		static const uint8_t DATA_LIZARD_OFF[3] = { SETTING_LIZARD_MODE,
							    0x00, 0x00 };
		static const uint8_t DATA_LIZARD_ON[3] = { SETTING_LIZARD_MODE,
							   0x01, 0x00 };

		// In puck mode Steam owns haptics; skip id9 steering.
		if (!modeIsPuck(g_usbMode)) {
			bool wantAuto = (g_padHaptics != 0);
			for (int s = 0; s < NSLOT; s++) {
				if (!g_slot[s].used || !hapticLinkUp(s)) {
					// re-land id9 on the next (re)connect: a fresh controller defaults to
					// autonomous, but one carrying our previous session's id9 does not
					landedAuto[s] = false;
					lastKeep[s] = 0;
					continue;
				}
				if (wantAuto) {
					if (!landedAuto[s]) {
						landedAuto[s] = true;
						relayEnqueue(
							IBEX_CMD_SET_SETTINGS_VALUES,
							DATA_LIZARD_ON,
							sizeof DATA_LIZARD_ON,
							false, (uint8_t)s);
					}
				} else {
					landedAuto[s] = false;
					if (lastKeep[s] &&
					    (uint32_t)(millis() - lastKeep[s]) <
						    LIZKEEP_MS)
						continue;
					lastKeep[s] = millis();
					relayEnqueue(
						IBEX_CMD_SET_SETTINGS_VALUES,
						DATA_LIZARD_OFF,
						sizeof DATA_LIZARD_OFF, false,
						(uint8_t)s);
				}
			}
		} // !modeIsPuck
	}
	// Per-slot link-edge detect (backup for hapticOnReconnect in rf_link).
	static bool wasHapticLinkUp[NSLOT] = { 0 };
	for (int s = 0; s < NSLOT; s++) {
		if (!g_slot[s].used)
			continue;
		bool up = hapticLinkUp(s);
		if (up && !wasHapticLinkUp[s]) {
			uint8_t mk = 1;
			hapLogAdd(0xFD, 0xEE, &mk, 1);
			hapticOnReconnect(s);
		}
		if (!up && wasHapticLinkUp[s]) {
			uint8_t mk = 0;
			hapLogAdd(0xFD, 0xEE, &mk, 1);
			// Lifecycle log (CDC debug boot): the 300ms link-up watchdog saw this slot go silent. Pairs
			// with the CONNECT/RECONNECT lines from rf_link so a session of cycles shows the full
			// down->up cadence -- how often the link actually drops (churn) vs stays up.
			if (Serial.availableForWrite() > 50)
				Serial.printf("# LC t=%lu slot%d link DOWN\n",
					      (unsigned long)millis(), s);
		}
		wasHapticLinkUp[s] = up;
	}
	// (No automatic re-init firing -- see hapticOnReconnect: the connect-time 0x81 storm was the click train
	// the real puck never makes. hapticReinit is on-demand only, via the WebUSB "Clear stuck buzz" button.)
	// Power-off on host sleep: only when VBUS is present (genuine sleep, not a cable unplug which also
	// trips the suspend edge briefly) AND the suspend has PERSISTED >= SUSPEND_OFF_MS. A brief USB
	// selective-suspend (host idle power-management) resumes in <1s; firing the power-off on its edge
	// powered the controllers off ourselves -> random drop/reconnect churn. Arm only on a genuine
	// resume->suspend edge (wasSusp=true at boot suppresses a false fire on boot-into-suspended).
	static bool wasSusp = true;
	static unsigned long suspSinceMs = 0;
	static bool suspArmed = false;
	bool susp = USBDevice.suspended();
	bool vbus = (NRF_POWER->USBREGSTATUS &
		     POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
	if (susp && !wasSusp) {
		suspSinceMs = millis();
		suspArmed = true;
		faultDiagTrace(FR_SUSP, 0);
	}
	if (!susp) {
		if (wasSusp)
			faultDiagTrace(FR_RESUME, 0);
		suspArmed = false;
	}
	// g_suspendOff: opt out of the power-off entirely -- keeps the controller awake through host sleep so its
	// short-Steam-press remote wakeup stays available (see haptics.h). Disarm either way so re-enabling it
	// mid-suspend can't fire late.
	if (suspArmed && vbus && (millis() - suspSinceMs) >= SUSPEND_OFF_MS) {
		if (g_suspendOff)
			hapticSendShutdown();
		suspArmed = false; // fire once per suspend
	}
	wasSusp = susp;
	// Translated-rumble keepalive. Triton hardware has an approximately
	// 50 ms safety timeout; SDL resends active rumble every 40 ms.
	for (int s = 0; s < NSLOT; s++) {
		if (!g_rumble80On[s] || !hapticLinkUp(s))
			continue;
		if ((uint32_t)(millis() - g_rumble80Ms[s]) >=
		    RUMBLE_RESEND_INTERVAL_MS)
			hapticQueueRumble80(g_rumble80Low[s], g_rumble80High[s],
					    (uint8_t)s, false);
	}
	// (No automatic idle-clear re-init either: same 0x81-click reason. A genuinely stuck buzz is cleared
	// on demand from the panel. Verbatim relay -- like the real puck -- is the steady-state behavior.)
}
