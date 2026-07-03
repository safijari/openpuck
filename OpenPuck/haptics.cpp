#include "haptics.h"
#include "bonds.h"
#include "config.h"
#include "rf_link.h"
#include "usb_tx.h" // usbTxBoost/Unboost -- flood-rate CDC prints share the dcd DMA claim window
#include "puck_hid.h" // puckLizardActive() -- gate the lizard-suppression keepalive

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
// id9=0 hold in MODE_STEAM (see hapticTask): ON by default -- keeps the controller's autonomous haptic engine
// off so it can't latch into the deep-inside buzz. Scoped to MODE_STEAM (pure MODE_LIZARD keeps its ticks).
// Persisted; console 'u' toggles for A/B.
uint8_t g_lizKeep = 1;
// EXPERIMENT (console "L87" toggles, persisted): land ALL relayed 0x87 SET_SETTINGS via type-01 framing so
// Steam's haptic/amp/IMU config actually reaches the controller (verbatim relay = real-puck behavior),
// instead of the whitelist discarding everything but LED(0x2D)/id9(0x09). Hypothesis: the buzz is the
// controller's haptic engine left MISCONFIGURED because Steam's 0x87 config is discarded; landing it should
// configure the engine correctly and stop the buzz. Default OFF (keeps the safe whitelist) because landing
// reg 0x30 was once blamed for freezing the gyro and 0x34/0x35 for the buzz -- but those were OpenPuck's own
// bad hapticReinit values, so Steam's real values may be fine. Flip it on hardware to test; flip back if the
// gyro freezes.
uint8_t g_landAll87 = 0;
// Land Steam's amp/haptic-config 0x87 blocks (regs 0x18/0x2E/0x34/0x35, NOT gyro 0x30) so the controller's
// amplifier is configured and haptics play as clean ticks instead of a default-amp buzz. On by default;
// console "AMP" toggles for A/B. See the land01 whitelist in rfConnFlushRelay.
uint8_t g_landAmp = 1;
// Per-slot reconnect block. 0 = idle; non-zero = drop haptics aimed at this slot until millis() catches up.
unsigned long g_hapticBlockUntil[NSLOT] = { 0 };

// Controller power-off. CONFIRMED from a real Windows USB capture of the Valve puck: Steam's "turn off
// controller" is the single feature-0x01 command 0x9F with payload ASCII "off!" (6F 66 66 21). The dongle
// forwards host feature reports verbatim, so the controller acts on report 0x9F directly -- we relay it the
// same way (E3 SET sub-TLV). The wire relay is NO-ACK, so send a small burst: a single lost frame must not
// leave the controller on.
void hapticSendShutdown(uint8_t slot)
{
	static const uint8_t OFF[4] = { 0x6f, 0x66, 0x66, 0x21 }; // "off!"
	// slot-targeted: Steam's 0x9F arrives on ONE controller's interface and must power off only that
	// controller (broadcasting it killed every connected controller when the user turned off one). The
	// broadcast default (0xFF) remains for the triggers that logically mean "all off": host suspend and
	// the panel/test power-off button.
	faultDiagTrace(FR_OFF, slot);
	for (uint8_t i = 0; i < HAPTIC_SHUTDOWN_SHOTS; i++)
		relayEnqueue(0x9F, OFF, sizeof OFF, slot);
}

// millis of last 0x82 haptic OUTPUT relayed (Steam mode)
static unsigned long g_haptic82Ms = 0;

// a non-zero 0x82 haptic is currently active (awaiting host stop)
static bool g_haptic82On = false;

// millis of last translated host rumble (0x80), per-slot (4 XInput interfaces each have their own stream)
static unsigned long g_rumble80Ms[NSLOT] = { 0 };

// Steam/Triton rumble is latched on until an explicit zero report; tracked per-slot so each controller's
// stuck-rumble watchdog is independent
static bool g_rumble80On[NSLOT] = { false, false, false, false };

// ---- relay rings: one per bond slot. Multi-producer (USB ISR + loop-context console/xinput), one consumer
// per slot (rfConnFlushRelay on that slot's poll turn). Producers serialize under PRIMASK.
struct RelayMsg {
	uint8_t rid, len;
	uint8_t data[RELAY_MAXP];
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
bool relayEnqueue(uint8_t rid, const uint8_t *payload, uint8_t plen,
		  uint8_t slot)
{
	if (plen > RELAY_MAXP)
		plen = RELAY_MAXP;
	if (slot != 0xFF && slot >= NSLOT)
		return false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	// slot=0xFF: broadcast -- enqueue into every slot's ring.
	// Full queue: evict the oldest entry, never the newest. Steam bursts end with the commit/stop, so
	// dropping the oldest keeps the most-recent (meaningful) frame.
	uint8_t s0 = (slot == 0xFF) ? 0 : slot;
	uint8_t s1 = (slot == 0xFF) ? NSLOT : slot + 1;
	for (uint8_t s = s0; s < s1; s++) {
		uint8_t h = g_rqHead[s], nx = rqNext(h);
		if (nx == g_rqTail[s])
			g_rqTail[s] = rqNext(g_rqTail[s]);
		g_rq[s][h].rid = rid;
		g_rq[s][h].len = plen;
		if (plen)
			memcpy(g_rq[s][h].data, payload, plen);
		g_rqHead[s] = nx;
	}
	// Track last-haptic time for the Steam-mode quiet timeout (marks the 0x82 stream inactive after silence).
	if (rid == 0x82 || rid == 0x80)
		g_haptic82Ms = millis();
	__set_PRIMASK(pm);
	faultDiagTrace(FR_RELAY, (uint16_t)((slot << 8) | rid));
	return true;
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
static bool haptic82PayloadOn(const uint8_t *p, uint16_t n)
{
	if (n < 3)
		return false;
	for (uint16_t i = 2; i < n; i++)
		if (p[i])
			// observed form is [01 01 gain], but treat any trailing non-zero as active
			return true;
	return false;
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
void haptic82HostReport(const uint8_t *p, uint16_t n)
{
	if (n < 3)
		return;
	g_haptic82Ms = millis();
	// Track on/off only. Do NOT synthesize a stop burst: Steam's own stop is forwarded verbatim, and each extra
	// 0x82 is a discrete pad click -- the spurious end-of-movement "click"/buzz the real puck never produces.
	g_haptic82On = haptic82PayloadOn(p, n);
}
bool hapticSteamRumble(uint16_t lowFreq, uint16_t highFreq, uint8_t slot)
{
	if (slot >= NSLOT)
		return false;
	// user rumble-strength scale (percent; 200 = double). Clamp to 16-bit.
	if (g_rumbleScale != 100) {
		uint32_t l = (uint32_t)lowFreq * g_rumbleScale / 100,
			 h = (uint32_t)highFreq * g_rumbleScale / 100;
		lowFreq = (l > 0xFFFF) ? 0xFFFF : (uint16_t)l;
		highFreq = (h > 0xFFFF) ? 0xFFFF : (uint16_t)h;
	}
	bool on = lowFreq || highFreq;
	// Per-slot settle gate (the per-slot reconnect block + link-up check). 0x82 haptics in Steam mode use the
	// same gate; for XInput, the host only sends a stream while a controller is connected, so this also doubles
	// as "no controller here, no relay".
	if (on && haptic82Blocked(slot))
		return false;
	if (!on && !hapticLinkUp(slot))
		return false;

	// SDL's current Steam/Triton structs define output report 0x80 as:
	//   type, uint16 intensity, {uint16 speed, int8 gain} left/right.
	// We map conventional gamepad low/high-frequency motors to left/right speeds and use max as intensity.
	uint16_t intensity = lowFreq > highFreq ? lowFreq : highFreq;
	uint8_t p[9];

	// haptic_type_t::HAPTIC_TYPE_RUMBLE; 0 is the off/zero report
	p[0] = on ? 0x04 : 0x00;
	p[1] = (uint8_t)(intensity & 0xFF);
	p[2] = (uint8_t)(intensity >> 8);
	p[3] = (uint8_t)(lowFreq & 0xFF);
	p[4] = (uint8_t)(lowFreq >> 8);
	p[5] = 0;
	p[6] = (uint8_t)(highFreq & 0xFF);
	p[7] = (uint8_t)(highFreq >> 8);
	p[8] = 0;
	if (!relayEnqueue(0x80, p, sizeof p, slot))
		return false;
	g_rumble80Ms[slot] = millis();
	g_rumble80On[slot] = on;
	return true;
}
// Queue a pending test-haptic / stop relay (runs inside the poll cadence -- never at raw loop rate). Test
// haptics broadcast to all connected slots (slot 0xFF); the stop frame is broadcast too (a stuck latch can
// affect any controller, and the haptic-engine clear-re-init is settings-only so it's harmless on healthy
// ones).
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
				relayEnqueue(0x82, p, 3, s);
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
		if (relayEnqueue(0x82, HAP_ON, 3, 0xFF))
			g_testHaptic--;
	} else if (g_hapticStop && !g_xbox) {
		if (relayEnqueue(0x82, HAP_OFF, 3, 0xFF))
			g_hapticStop--;
	}
}
// rfConnFlushRelay(ch, s1): drain one entry from the current slot's relay queue and TX it. Each slot's queue
// is independent, so each controller only sees its own commands. With N connected slots the per-slot relay
// rate is 1/N of the per-cycle rate; sustained buzz streams are still 1 packet/cycle/slot.
bool rfConnFlushRelay(uint8_t ch, uint8_t s1)
{
	int cur = (g_curSlot >= 0 && g_curSlot < NSLOT) ? g_curSlot : 0;
	// Snapshot one entry under a short critical section. relayEnqueue() (the producer) runs on the
	// high-priority usbd task and, when the ring is full, evicts the oldest by advancing g_rqTail itself --
	// the same variable this consumer reads/advances. Reading the entry while a producer could overwrite it
	// is a torn read (and dual-writing g_rqTail desyncs head/tail), so copy the entry out and consume the
	// slot atomically here, then do the (slow) RF TX on the copy with interrupts enabled.
	RelayMsg msg;
	bool have = false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t guard = RELAY_QLEN + 1;
	while (g_rqTail[cur] != g_rqHead[cur]) {
		if (!guard--) { // desynced/corrupt ring -> recover, don't spin IRQs-off (watchdog-hang class)
			g_rqHead[cur] = g_rqTail[cur] = 0;
			g_ringFault++;
			faultDiagTrace(FR_RINGF, g_ringFault);
			break;
		}
		RelayMsg &m = g_rq[cur][g_rqTail[cur]];
		g_rqTail[cur] = rqNext(g_rqTail[cur]); // consume the slot
		// rid 0 = entry voided by hapticCancelPendingOn -> skip
		if (m.rid) {
			msg = m; // copy out before the producer can reuse the slot
			have = true;
			break;
		}
	}
	__set_PRIMASK(pm);
	if (have) {
		RelayMsg &m = msg;
		{
			uint8_t rl = m.len;
			if (rl > RELAY_MAXP)
				rl = RELAY_MAXP;
			// On-air sub-TLV framing. CONFIRMED from real puck<->controller sniffs: a command LANDS on
			// the controller only with the type-01 + inner-len form E3 [2+rl][01][rid][innerlen][data];
			// the legacy form E3 [1+rl][05][rid][data] makes the controller DISCARD any 0x87+ command.
			//
			// Whitelist type-01 to exactly three commands: LED brightness (0x87 reg 0x2D), power-off
			// (0x9F), and the STANDALONE lizard-suppression keepalive (0x87 [09 00 00] -- rl==3 so a
			// multi-register block that merely STARTS with 0x09 can't smuggle other registers into
			// landing). Setting index 9 = digital-mappings/lizard-active (ibex FUN_0001f554 decomp);
			// the real puck lands id9=0 every 3s -- without it the controller's revert timer fires,
			// re-enables autonomous mode and resets ALL settings to defaults (audible amp pop +
			// autonomous touchpad ticks = the spurious mid-session buzz). Other 0x87 writes (e.g. reg
			// 0x30 IMU enable, 0x34/0x35 haptic amplitude) must stay on legacy form -- landing 0x30
			// freezes the gyro; landing 0x34/0x35 causes the connect buzz.
			// Amp/haptic-config registers (0x18/0x2E/0x34/0x35): LAND these so the controller's amplifier is
			// configured the way Steam intends -> haptics (0x82 ticks, connect cue) play as clean ticks
			// instead of the default-amp BUZZ we get when the config is discarded. Steam sends these grouped
			// in 0x87 blocks that START with one of those regs and do NOT contain 0x30 (the gyro reg, which
			// must NOT land -- it freezes the gyro), so gating on data[0] lands the amp block without 0x30.
			// (g_landAmp, console "AMP"; on by default. The old "landing 0x34/0x35 buzzes" note was really the
			// 0x81 storm, now removed.)
			bool ampReg = (m.data[0] == 0x18 || m.data[0] == 0x2E ||
				       m.data[0] == 0x34 || m.data[0] == 0x35);
			bool land01 =
				(m.rid == 0x9F) ||
				(m.rid == 0x87 && rl >= 1 &&
				 m.data[0] == 0x2D) ||
				(m.rid == 0x87 && rl == 3 &&
				 m.data[0] == 0x09) ||
				(g_landAmp && m.rid == 0x87 && rl >= 1 &&
				 ampReg) ||
				// experiment: land EVERY 0x87 verbatim (real-puck relay) when enabled
				(g_landAll87 && m.rid == 0x87);
			uint8_t p[5 + RELAY_MAXP], plen;
			if (land01) {
				p[0] = g_relayOp;
				p[1] = (uint8_t)(2 + rl);
				p[2] = 0x01;
				p[3] = m.rid;
				p[4] = rl;
				memcpy(p + 5, m.data, rl);
				plen = (uint8_t)(5 + rl);
			} else {
				p[0] = g_relayOp;
				p[1] = (uint8_t)(1 + rl);
				p[2] = 0x05;
				p[3] = m.rid;
				memcpy(p + 4, m.data, rl);
				plen = (uint8_t)(4 + rl);
			}
			hapLogAdd(0xFE, m.rid, m.data, rl);
			// Lifecycle log (CDC debug boot): EXACTLY what we TX to the controller, so a capture shows
			// whether OpenPuck (or Steam in the background) pokes the controller right before a buzz
			// latches -- the piece the I45 stream (controller->us) can't show. Low rate outside a haptic
			// burst; guarded so it never blocks the loop. cur = target slot.
			if (Serial.availableForWrite() > 60) {
				// boosted: this print runs at relay-flood rate and CDC flush enters the same
				// TinyUSB DMA claim window as HID sends (the issue-72 livelock; see usb_tx.cpp)
				usbTxBoost();
				Serial.printf("# TX t=%lu slot%d %s rid=%02X:",
					      (unsigned long)millis(), cur,
					      land01 ? "L01" : "l05", m.rid);
				for (uint8_t i = 0; i < rl && i < 8; i++)
					Serial.printf(" %02X", m.data[i]);
				Serial.println();
				usbTxUnboost();
			}
			// slot was already consumed under the critical section above.
			// s1 carries a PID distinct from the GET poll (caller cycles it) so the controller's ESB
			// dedup never treats the GET as a retransmit of this relay. 80us RX: relay is NO-ACK.
			rfConnTx(ch, s1, p, plen,
				 80); // one relay per poll cycle
		}
	}
	return have; // true = a relay frame went out this cycle (steals a reply window from the E3 poll)
}

// Haptic-subsystem re-init: the captured sequence Steam sends when it (re)takes control (0x81 reset + 0x87
// register writes). Brightness (reg 0x2D) is omitted to avoid stomping the LED. The 0x87 frames go out on
// legacy framing so the controller discards them; the three 0x81 frames are the effective reset. slot=0xFF
// broadcasts to all connected controllers (the re-init is settings-only and harmless on healthy controllers).
// One-shot SET_SETTINGS (0x87) write of the controller's GLOBAL trackpad-haptics enable. The controller plays
// touchpad haptic sequences autonomously (haptics-sequencer-touchpad) gated by settings/haptics/enabled; in the
// emulated (non-Steam) modes the host never sends SC haptic reports, so the only way to silence the pad buzz is
// to write this setting. Wire format [id][val u16 LE] -- confirmed from the controller's SET_SETTINGS handler
// (decomp FUN_0001f61c; id 0x30 == settings/sensors/imu/mode is the cross-check anchor in the same id space).
// The exact id for settings/haptics/enabled needs a one-time `scmd labels` HW read (candidates from the captured
// reinit: 0x18 / 0x2e / 0x34 / 0x35). Until it is set, SETTING_HAPTICS_ENABLED stays 0xFF and this is inert.
#define SETTING_HAPTICS_ENABLED 0xFF
void hapticSetPadEnabled(uint8_t slot, bool on)
{
#if SETTING_HAPTICS_ENABLED != 0xFF
	uint8_t pl[3] = { SETTING_HAPTICS_ENABLED, (uint8_t)(on ? 1 : 0), 0 };
	relayEnqueue(0x87, pl, 3, slot);
#else
	(void)slot;
	(void)on;
#endif
}
void hapticReinit(uint8_t slot)
{
	static const uint8_t H30[] = { 0x30, 0x00, 0x00, 0x07, 0x07,
				       0x00, 0x08, 0x07, 0x00, 0x31,
				       0x02, 0x00, 0x52, 0x03, 0x00 };
	static const uint8_t H18[] = { 0x18, 0x00, 0x00, 0x2e, 0x00,
				       0x00, 0x34, 0xff, 0xff, 0x35,
				       0xff, 0xff, 0x34, 0xff, 0xff };
	static const uint8_t H35[] = { 0x35, 0xff, 0xff, 0x2e, 0x00, 0x00 };
	static const uint8_t T81A[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	static const uint8_t T81B[] = {
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	// reset action (FUN_0001f554) -- Steam sends this first
	relayEnqueue(0x81, nullptr, 0, slot);
	relayEnqueue(0x87, H30, sizeof H30, slot);

	// haptic config (enabled/amplifier/gain): the part that clears a latch
	relayEnqueue(0x87, H18, sizeof H18, slot);
	relayEnqueue(0x87, H35, sizeof H35, slot);
	relayEnqueue(0x81, T81A, sizeof T81A, slot);
	relayEnqueue(0x81, T81B, sizeof T81B, slot);
	// Re-apply the active emulated type's trackpad-haptics preference last (after the haptic-config writes
	// above, which would otherwise re-enable it). Default-on types send "enable"; Switch (padHaptics=0)
	// disables. Inert until the setting id is captured.
	hapticSetPadEnabled(slot, g_padHaptics != 0);
	// Apply the configured LED brightness for the active emulated type. Steam sets the
	// brightness each session; emulated modes never do, so the controller comes up at
	// full brightness. 0 = no override (preserve controller default).
	if (g_etype < ET_COUNT && g_ledBright > 0) {
		uint8_t pl[3] = { 0x2D, g_ledBright, 0x00 };
		relayEnqueue(0x87, pl, sizeof pl, slot);
	}
}
void hapticInit()
{
	g_haptic82On = false;
	g_hapticStop = 0;
	for (int s = 0; s < NSLOT; s++) {
		g_rqHead[s] = g_rqTail[s] = 0;
		g_rumble80On[s] = false;
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
	g_haptic82On = false;
	g_rumble80On[slot] = false;
	g_rumble80Ms[slot] = 0;
	// Scrub haptics queued before the link came up (stale across the reconnect) -- this slot only.
	hapticCancelPendingOn(slot);
	// NO automatic haptic re-init. hapticReinit lands 0x81 (CLEAR DIGITAL MAPPINGS -- rid < 0x87 so it
	// EXECUTES even on legacy framing). 0x81 is NON-IDEMPOTENT (ibex FUN_0001f554): EVERY call re-runs the
	// lizard-disable event + func_0x0001bbf0 (a hardware peripheral re-arm unique to the 0x81 path) -- so
	// firing 8 across the connect window = 8 audible clicks = the "repeated non-periodic clicks at connect"
	// the real puck (which clears ONCE then holds) never produces. The clearing re-init stays available on
	// demand (WebUSB "Clear stuck buzz" -> hapticReinit), just not sprayed at connect.
	uint8_t mk = 2;
	hapLogAdd(0xFD, 0xEE, &mk, 1);
}
void hapticTask()
{
	// id9=0 hold in MODE_STEAM: land SET_SETTINGS index 9 = 0 on each connected controller every LIZKEEP_MS,
	// the way the real puck does (sniff1.json: id9=0 landed ~every few s). This holds the controller in
	// gamepad mode with its AUTONOMOUS mapping/haptic engine OFF -- which is what stops the controller's own
	// touchpad-haptic engine from latching into the "deep-inside" buzz seen after repeated reconnects (that
	// buzz is controller-internal: capture-for-haptics.txt showed the controller in autonomous mode, rid=40/
	// 41, with OpenPuck relaying no haptics at all -- so only holding id9=0 quiets it). Framing note: 0x87
	// (SET_SETTINGS) is idempotent -- a repeated same-value id9=0 is change-guarded in the controller, so it
	// does NOT re-fire the click-causing 0x81 side effects; holding via 0x87 is silent where a looped 0x81
	// was not. It DOES disable the controller's autonomous touchpad ticks (id9 gates the whole pad layer),
	// which is fine in Steam mode (Steam owns haptics) -- hence scoped to MODE_STEAM ONLY. Pure MODE_LIZARD
	// deliberately leaves id9 alone so its autonomous ticks keep working (and it's the mode with no buzz).
	if (g_lizKeep && g_usbMode == MODE_STEAM) {
		static unsigned long lastKeep[NSLOT] = { 0 };
		static const uint8_t KA[3] = { 0x09, 0x00, 0x00 };
		for (int s = 0; s < NSLOT; s++) {
			if (!g_slot[s].used || !hapticLinkUp(s))
				continue;
			if (lastKeep[s] &&
			    (uint32_t)(millis() - lastKeep[s]) < LIZKEEP_MS)
				continue;
			lastKeep[s] = millis();
			relayEnqueue(0x87, KA, sizeof KA, (uint8_t)s);
		}
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
	if (suspArmed && vbus && (millis() - suspSinceMs) >= SUSPEND_OFF_MS) {
		hapticSendShutdown();
		suspArmed = false; // fire once per suspend
	}
	wasSusp = susp;
	// Steam-mode quiet timeout: mark 0x82 stream inactive. No synthesized stop -- Steam forwards its own.
	if (!g_xbox && g_haptic82On &&
	    millis() - g_haptic82Ms > HAPTIC_QUIET_MS)
		g_haptic82On = false;
	// Per-slot stuck-rumble watchdog: force zero after 2.5s without a refresh.
	for (int s = 0; s < NSLOT; s++) {
		if (g_rumble80On[s] && millis() - g_rumble80Ms[s] > 2500u)
			hapticSteamRumble(0, 0, (uint8_t)s);
	}
	// (No automatic idle-clear re-init either: same 0x81-click reason. A genuinely stuck buzz is cleared
	// on demand from the panel. Verbatim relay -- like the real puck -- is the steady-state behavior.)
}
