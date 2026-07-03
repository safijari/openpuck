#include "puck_hid.h"
#include "bonds.h"
#include "config.h"
#include "identity.h"
#include "haptics.h"
#include "fault_diag.h"
#include "rf_link.h"
#include "triton.h"
#include "mode_lizard.h"
#include "wake_hid.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

uint8_t g_fwdNewOnly = 1;
SteamPuckController g_steamPuck;

// Cloned puck HID report descriptor: mouse(0x40)+keyboard(0x41)+vendor(FF00) with the 63-byte FEATURE
// command reports on report id 1/2. Each of the 4 interfaces uses this.
static const uint8_t PUCK_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x40, 0x09, 0x01, 0xA1, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
	0x95, 0x02, 0x81, 0x02, 0x75, 0x06, 0x95, 0x01, 0x81, 0x01, 0x05, 0x01,
	0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x02,
	0x81, 0x06, 0x95, 0x01, 0x09, 0x38, 0x81, 0x06, 0x05, 0x0C, 0x0A, 0x38,
	0x02, 0x95, 0x01, 0x81, 0x06, 0xC0, 0xC0, 0x05, 0x01, 0x09, 0x06, 0xA1,
	0x01, 0x85, 0x41, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25,
	0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x81, 0x01, 0x19, 0x00, 0x29,
	0x65, 0x15, 0x00, 0x25, 0x65, 0x75, 0x08, 0x95, 0x06, 0x81, 0x00, 0xC0,
	0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x42, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x35, 0x09, 0x42, 0x81, 0x02, 0x85, 0x44,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x05, 0x09, 0x44, 0x81,
	0x02, 0x85, 0x79, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x01,
	0x09, 0x79, 0x81, 0x02, 0x85, 0x43, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x0E, 0x09, 0x43, 0x81, 0x02, 0x85, 0x7B, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x0C, 0x09, 0x7B, 0x81, 0x02, 0x85, 0x45,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x2D, 0x09, 0x45, 0x81,
	0x02, 0x85, 0x80, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x09,
	0x09, 0x80, 0x91, 0x02, 0x85, 0x81, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x07, 0x09, 0x81, 0x91, 0x02, 0x85, 0x82, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x03, 0x09, 0x82, 0x91, 0x02, 0x85, 0x83,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x09, 0x09, 0x83, 0x91,
	0x02, 0x85, 0x84, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x08,
	0x09, 0x84, 0x91, 0x02, 0x85, 0x85, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x03, 0x09, 0x85, 0x91, 0x02, 0x85, 0x86, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x03, 0x09, 0x86, 0x91, 0x02, 0x85, 0x87,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F, 0x09, 0x87, 0x91,
	0x02, 0x85, 0x89, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F,
	0x09, 0x89, 0x91, 0x02, 0x85, 0x88, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x3F, 0x09, 0x88, 0x91, 0x02, 0x85, 0x01, 0x95, 0x3F, 0x09,
	0x01, 0xB1, 0x02, 0x85, 0x02, 0x95, 0x3F, 0x09, 0x01, 0xB1, 0x02, 0xC0
};

// Pure MODE_LIZARD only: same puck/lizard descriptor plus a tiny Consumer Control report for media keys.
// MODE_STEAM must keep PUCK_HID_DESC verbatim so Steam's controller/haptics path sees the normal puck shape.
static const uint8_t PUCK_LIZARD_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x40, 0x09, 0x01, 0xA1, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
	0x95, 0x02, 0x81, 0x02, 0x75, 0x06, 0x95, 0x01, 0x81, 0x01, 0x05, 0x01,
	0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x02,
	0x81, 0x06, 0x95, 0x01, 0x09, 0x38, 0x81, 0x06, 0x05, 0x0C, 0x0A, 0x38,
	0x02, 0x95, 0x01, 0x81, 0x06, 0xC0, 0xC0, 0x05, 0x01, 0x09, 0x06, 0xA1,
	0x01, 0x85, 0x41, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25,
	0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x81, 0x01, 0x19, 0x00, 0x29,
	0x65, 0x15, 0x00, 0x25, 0x65, 0x75, 0x08, 0x95, 0x06, 0x81, 0x00, 0xC0,
	0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x42, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x35, 0x09, 0x42, 0x81, 0x02, 0x85, 0x44,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x05, 0x09, 0x44, 0x81,
	0x02, 0x85, 0x79, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x01,
	0x09, 0x79, 0x81, 0x02, 0x85, 0x43, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x0E, 0x09, 0x43, 0x81, 0x02, 0x85, 0x7B, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x0C, 0x09, 0x7B, 0x81, 0x02, 0x85, 0x45,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x2D, 0x09, 0x45, 0x81,
	0x02, 0x85, 0x80, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x09,
	0x09, 0x80, 0x91, 0x02, 0x85, 0x81, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x07, 0x09, 0x81, 0x91, 0x02, 0x85, 0x82, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x03, 0x09, 0x82, 0x91, 0x02, 0x85, 0x83,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x09, 0x09, 0x83, 0x91,
	0x02, 0x85, 0x84, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x08,
	0x09, 0x84, 0x91, 0x02, 0x85, 0x85, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x03, 0x09, 0x85, 0x91, 0x02, 0x85, 0x86, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x03, 0x09, 0x86, 0x91, 0x02, 0x85, 0x87,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F, 0x09, 0x87, 0x91,
	0x02, 0x85, 0x89, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F,
	0x09, 0x89, 0x91, 0x02, 0x85, 0x88, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x3F, 0x09, 0x88, 0x91, 0x02, 0x85, 0x01, 0x95, 0x3F, 0x09,
	0x01, 0xB1, 0x02, 0x85, 0x02, 0x95, 0x3F, 0x09, 0x01, 0xB1, 0x02, 0xC0,
	0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x03, 0x15, 0x00, 0x25, 0x01,
	0x09, 0xE9, 0x09, 0xEA, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02, 0x75, 0x06,
	0x95, 0x01, 0x81, 0x01, 0xC0
};

static Adafruit_USBD_HID hid[NSLOT];

// Drop Steam's relayed 0x81 CLEAR_DIGITAL_MAPPINGS in Steam mode (console "S81" toggles; on by default). It's
// the confirmed amp-clicker in Steam's per-connect config and OpenPuck doesn't need it (own input translation
// + id9=0 keepalive). Reversible from the console if it regresses.
bool g_drop81 = true;

// Per-slot shadow of the controller's SET_SETTINGS_VALUES array (id-indexed u16, ids 0..0x52). Steam writes
// it with 0x87 and READS IT BACK with 0x89 to verify its config "took"; on the real puck the controller
// answers 0x89 from this array. OpenPuck can't relay-and-return over the NO-ACK RF link, so it maintains the
// shadow here and answers 0x89 from it -- WITHOUT this, Steam's verify never matches and it re-clears (0x81)
// + re-writes (0x87) forever = the connect config storm that buzzes the amp and floods the usbd task.
static uint16_t g_setShadow[NSLOT][0x53];

// ---- feature-command capture (diagnostic) ---------------------------------------------------------------
// Log the USB feature command channel (Steam's SET/GET) to serial to see the connect handshake -- WITHOUT a
// Serial.printf on the fragile 800B usbd task (that path can blow the stack -> LOCKUP; it's why production
// avoids printf here). handleSet/handleGet (usbd task) push a compact record under PRIMASK; puckCmdLogDrain()
// prints them from loop() context. When g_cmdCapture is on, the high-rate I45 input stream is also suppressed
// (rf_link) so the command sequence is readable. Toggle from the console with "FC".
bool g_cmdCapture = true;
struct FCmdRec {
	uint8_t dir; // 0 = SET (host->puck write), 1 = GET (host read)
	uint8_t iface; // HID interface index = bond slot the command hit
	uint8_t rid, cmd, len, b[10];
};
static FCmdRec g_fc[32];
static volatile uint8_t g_fcHead, g_fcTail;
static void fcPush(uint8_t dir, int iface, uint8_t rid, uint8_t cmd,
		   uint8_t len, const uint8_t *b, uint8_t n)
{
	if (!g_cmdCapture)
		return;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t h = g_fcHead, nx = (uint8_t)((h + 1) & 31);
	if (nx !=
	    g_fcTail) { // drop-newest when full (loop drains fast); never block the usbd task
		g_fc[h].dir = dir;
		g_fc[h].iface = (uint8_t)iface;
		g_fc[h].rid = rid;
		g_fc[h].cmd = cmd;
		g_fc[h].len = len;
		for (uint8_t i = 0; i < 10; i++)
			g_fc[h].b[i] = (i < n) ? b[i] : 0;
		g_fcHead = nx;
	}
	__set_PRIMASK(pm);
}
void puckCmdLogDrain(void)
{
	// boosted: runs at Steam's feature-storm rate and CDC flush enters the same TinyUSB DMA claim window
	// as HID sends (the issue-72 livelock; see usb_tx.cpp)
	usbTxBoost();
	while (g_fcHead != g_fcTail) {
		if (Serial.availableForWrite() < 70)
			break; // don't stall loop on CDC backpressure; resume next iteration
		FCmdRec r = g_fc[g_fcTail];
		g_fcTail = (uint8_t)((g_fcTail + 1) & 31);
		Serial.printf("# FC %s if=%u rid=%u cmd=%02X len=%u:",
			      r.dir ? "GET" : "SET", r.iface, r.rid, r.cmd,
			      r.len);
		for (uint8_t i = 0; i < 10; i++)
			Serial.printf(" %02X", r.b[i]);
		Serial.println();
	}
	usbTxUnboost();
}

// ===================== seamless LIZARD decision =====================
// Steam, while running, re-sends settings report 0x87 (lizard-off) every ~3s as a heartbeat (captured on HW),
// and ANY OUTPUT report likewise stamps g_steamAliveMs. When the heartbeat stops we fall back to lizard, so the
// controller drives desktop keyboard+mouse whenever Steam isn't running. MODE_LIZARD forces lizard always.

// millis of last Steam OUTPUT/settings write; 0 at boot => lizard until Steam appears
static unsigned long g_steamAliveMs = 0;
// Fall back to lizard this long after Steam's ~3s settings heartbeat stops. Keep >2x the cadence: the haptic
// relay is gated by !lizardActive(), so a shorter window lets a jittered/delayed heartbeat flip lizard
// mid-session while Steam is still running -> a haptic arriving in that window gets gated out (dropped). 7s
// tolerates one missed beat. Do not shorten without a lizard trigger independent of the heartbeat timeout.
#define LIZARD_WD_MS 7000u
static bool g_autoLizard =
	true; // master switch; false => Steam mode always forwards 0x45
// Single source of truth, shared by the USB input path AND the haptic relay gate: if we ever relay a 0x82 to
// the controller while presenting lizard (Steam isn't reading 0x45 back), Steam loops the same haptic -> buzz.
static inline bool steamDrivingGamepad()
{
	return g_steamAliveMs && (millis() - g_steamAliveMs < LIZARD_WD_MS);
}
static inline bool lizardActive()
{
	return modeIsPuck(g_usbMode) &&
	       (g_usbMode == MODE_LIZARD ||
		(g_autoLizard && !steamDrivingGamepad()));
}
// Public accessor for the haptic layer (haptics.cpp gates the lizard-suppression keepalive on this).
bool puckLizardActive()
{
	return lizardActive();
}
static inline void hostStampAlive()
{
	g_steamAliveMs = millis();
}
// Right after the host resumes from suspend, MUTE input forwarding briefly. Otherwise the controller's input
// in that instant (a trackpad click/trigger, or residual button state from the wake gesture) gets forwarded
// as a real click/keypress into the just-woken desktop. Set when task() sees the suspended->active transition.
static unsigned long g_resumeMs = 0;
#define POST_RESUME_MUTE_MS 1500u

// ===================== puck feature command channel =====================
// `slot` is the interface index (interface N == bond slot N).
static void handleSet(int slot, uint8_t rid, hid_report_type_t type,
		      uint8_t const *b, uint16_t n)
{
	// Steam OUTPUT reports 0x80-0x89. The haptic/actuator reports (0x80-0x86) are relayed to the controller,
	// and ONLY when they arrive on the CONNECTED slot's interface: we have one controller but expose 4 puck
	// slots, and a report aimed at a DIFFERENT slot made the controller buzz at random (the slot gate below
	// fixes that). Must NOT clamp to 0x82-only: ping/grip/test haptics ride other report IDs (0x85/0x86). The
	// 63-byte settings/config reports 0x87/0x88/0x89 are NOT haptics and reach the controller via the
	// feature-0x01 passthrough path instead.
	if (type == HID_REPORT_TYPE_OUTPUT) {
		if (rid >= 0x80 && rid <= 0x89) {
			// capture ALL OUTPUT reports (even un-relayed) for the 'H' dump
			hapLogAdd((uint8_t)slot, rid, b, n);

			// ANY Steam OUTPUT report (not just the 0x87 heartbeat) means Steam is present and driving ->
			// leave lizard for gamepad NOW, so a haptic arriving before the first 0x87 isn't relayed while
			// we're still presenting lizard (-> buzz loop).
			hostStampAlive();
		}
		// Post-resume mute also gates haptics: while onReport45 is muted Steam reads NO 0x45 back, which is the
		// exact condition under which Steam loops the same haptic command (-> connect/wake buzz loop).
		bool muted = g_resumeMs &&
			     millis() - g_resumeMs < POST_RESUME_MUTE_MS;
		// Drop the OUTPUT-report form of 0x81 too (Steam's haptic-reset action, a 7-byte report id 0x81 with
		// payloads 00../01..). Same amp-clicker as the feature-0x01 0x81; g_drop81 only covered the feature
		// path, so this OUTPUT form was still leaking through and clicking. OpenPuck doesn't need it.
		bool dropOut81 =
			(g_drop81 && g_usbMode == MODE_STEAM && rid == 0x81);
		if (rid >= 0x80 && rid <= 0x86 && !dropOut81 && n >= 1 &&
		    hapticRelaySlotOk(slot) && !lizardActive() && !muted) {
			if (!haptic82Blocked(slot)) {
				relayEnqueue(rid, b,
					     (uint8_t)(n > RELAY_MAXP ?
							       RELAY_MAXP :
							       n),
					     (uint8_t)slot);
				// Track on/off from what was actually RELAYED (= the controller's believed state): a
				// BLOCKED stop must leave "on" set (controller may be latched -> reconnect stop-burst),
				// a blocked ON must not set it (nothing reached the controller -> no spurious clicks).
				if (rid == 0x82)
					haptic82HostReport(b, n);
			}
		}

#if OPK_LOG
		// Per-report Serial echo: gated to the diagnostic build only. handleSet runs on the TinyUSB device
		// task, whose FreeRTOS stack is just 800 bytes; a Serial.printf there costs 200-500B and, stacked
		// under TinyUSB's control-transfer frames + a preempting USB ISR, can blow the stack -> the exact
		// SP-corruption-during-exception-entry that shows up as RR_LOCKUP. Production keeps this path
		// printf-free; the same data is already in the OPK_LOG capture ring (hapLogAdd) for the WebUSB panel.
		if (Serial.availableForWrite() > 80) {
			Serial.printf("# OUT if%d rid=%02X n=%u:", slot, rid,
				      n);
			for (uint16_t i = 0; i < n && i < 14; i++)
				Serial.printf(" %02X", b[i]);
			Serial.println();
		}
#endif
		return;
	}
	if (type != HID_REPORT_TYPE_FEATURE || n < 1)
		return;
	Slot &S = g_slot[slot];
	uint8_t cmd = b[0], len = (n > 1) ? b[1] : 0;
	const uint8_t *pl = b + 2;
	uint16_t pln = (n >= 2) ? n - 2 : 0;

	// Capture EVERY feature SET (the whole cmd channel: 0x83 attr, 0xAE strings, 0xB4 conn, 0xA2/A3 bond, the
	// feature-1 relay, AND any host battery query) to the WebUSB ring so the panel's capture view shows it.
	hapLogAdd((uint8_t)slot, cmd, b, n);
	// ...and to the serial feature-command capture (diagnostic; drained in loop, see puckCmdLogDrain).
	fcPush(0, slot, rid, cmd, len, pl, pln > 10 ? 10 : (uint8_t)pln);
	// ...and to the flight recorder (survives a watchdog reset). This runs ON the fragile usbd task, so the
	// command flood that (theory) overflows it lands in the post-mortem trail rather than being lost with USB.
	faultDiagTrace(FR_SET, (uint16_t)((rid << 8) | cmd));

	// settings/haptic/LED report (incl. 0x87 lizard-off heartbeat, SDL Triton lizard-disable)
	if (cmd >= 0x80 && cmd <= 0x89)
		hostStampAlive();
	// Controller power-off: Steam's "turn off controller" is feature-0x01 frame 9F 04 6F 66 66 21 ("off!"),
	// confirmed from a real puck capture. The feature-0x01 relay below forwards it once; hapticSendShutdown
	// bursts it for NO-ACK reliability. Slot-targeted: the command arrived on THIS controller's interface,
	// so only this controller powers off (broadcasting killed all connected controllers at once).
	if (rid == 1 && cmd == 0x9F)
		hapticSendShutdown((uint8_t)slot);

	// report 0x01 = raw passthrough -> queue for RF relay to the controller
	if (rid == 1 && n >= 2) {
		// (feature-1 commands -- haptics, LED, 0x87 settings, 0x9F power-off -- are captured by the
		// general feature-SET hapLogAdd above.)
		bool haptic82 = (cmd == 0x82 && len <= pln);

		bool muted = g_resumeMs &&
			     millis() - g_resumeMs < POST_RESUME_MUTE_MS;

		// Steam-mode: DROP the relayed 0x81 CLEAR_DIGITAL_MAPPINGS (g_drop81, console "S81"). It EXECUTES on
		// the controller (rid<0x87, legacy framing) and each one re-arms the haptic amp (non-idempotent, ibex
		// func_0x0001bbf0) = the connect click/buzz; Steam sends ~13 in its per-connect config. OpenPuck does
		// its OWN input translation and holds mappings cleared via the id9=0 keepalive, so it does NOT need the
		// controller's mapping engine -> Steam's 0x81 is pure downside here. The manual "Clear stuck buzz"
		// (hapticReinit) sends its own 0x81 via relayEnqueue directly, so that cure path is unaffected.
		bool drop =
			(g_drop81 && g_usbMode == MODE_STEAM && cmd == 0x81);
		// Do NOT relay commands OpenPuck answers LOCALLY (identity/bond/settings READS). Relaying them to the
		// controller is pointless (their reply can't come back over the NO-ACK RF link -- we answer from the
		// handleSet/handleGet switch), and 0x83 GET_ATTRIBUTES in particular is < 0x87 so it EXECUTES on the
		// controller -- captured as the remaining periodic CLICK, fired every time Steam re-polls identity
		// (0x83/0xAE on all 4 interfaces). Only actuator/config commands (0x80-0x82/0x84-0x88 haptics+config,
		// 0x9F power) should reach the controller.
		bool localAnswer = (cmd == 0x83 || cmd == 0x89 || cmd == 0xAE ||
				    cmd == 0xA2 || cmd == 0xA3 || cmd == 0xAD ||
				    cmd == 0xB4 || cmd == 0xED || cmd == 0xA4);
		// never push haptics while presenting lizard (Steam isn't reading 0x45 -> would buzz-loop)
		bool relayOk = hapticRelaySlotOk(slot) && !drop &&
			       !localAnswer &&
			       !(haptic82 && (lizardActive() || muted));
		if (relayOk && (!haptic82 || !haptic82Blocked(slot))) {
			// Relay the DECLARED length (up to the 60B RF frame ceiling), not a truncation: Steam's
			// multi-register 0x87 settings blocks (LED brightness) and calibration writes exceed the old
			// 18B cap, and the chopped frames were why those settings never landed on the controller.
			uint8_t rl = (len <= pln) ? len : (uint8_t)pln;
#if OPK_LOG
			if (len > RELAY_MAXP && Serial.availableForWrite() > 60)
				Serial.printf(
					"# RELAY TRUNC cmd=%02X len=%u>%u\n",
					cmd, len, (unsigned)RELAY_MAXP);
#endif
			relayEnqueue(cmd, pl, rl, (uint8_t)slot);

			// track from RELAYED frames only (see the OUTPUT path)
			if (haptic82)
				haptic82HostReport(pl, len);
		}
	}
#if OPK_LOG
	// Diagnostic-build only -- see the OUTPUT-path note: no Serial.printf on the 800B usbd-task stack in
	// production (LOCKUP mitigation). The feature SET is already captured to the OPK_LOG ring above.
	if (Serial.availableForWrite() > 80) {
		Serial.printf("# SET if%d rid=%02X cmd=%02X len=%u:", slot, rid,
			      cmd, len);
		for (uint16_t i = 0; i < n && i < 14; i++)
			Serial.printf(" %02X", b[i]);
		Serial.println();
	}
#endif
	memset(S.resp, 0, sizeof S.resp);
	S.resp_len = 0;
	switch (cmd) {
	case 0x83:
		S.resp[0] = 0x83;
		S.resp[1] = sizeof ATTR83;
		memcpy(S.resp + 2, ATTR83, sizeof ATTR83);
		// Report-id 1 = an attributes query RELAYED to the bonded CONTROLLER; it must report the CONTROLLER's
		// product id (0x1302), NOT the puck/dongle's (0x1304) -- else Steam sees "a controller with a dongle's
		// id" (HANDOFF.md) and drops to the LEGACY 0x81 CLEAR_DIGITAL_MAPPINGS init path, whose rapid re-arm
		// storm is the connect buzz; with the correct id Steam uses the modern quiet 0x87 id9=0 path. Report-id
		// 2 stays the puck (0x1304). ATTR83 stores product as u32 LE at offset 1, so only the low byte flips
		// (0x04 -> 0x02); everything else in the blob is identical.
		if (rid == 1)
			S.resp[2 + 1] =
				0x02; // product 0x1304 -> 0x1302 (controller)
		S.resp_len = 63;
		break;
	case 0xAE: {
		uint8_t idx = pln > 0 ? pl[0] : 1;
		// Report-id 1 = string attributes of the bonded CONTROLLER, not the puck. Steam matches the connected
		// controller to its bond by SERIAL: it reads the controller's serial here (rid 1) and compares to the
		// 16-byte serial in the bond record it reads via 0xA3. OpenPuck was returning the PUCK's serial (g_unit,
		// "FXB..."), which never matches any bond's controller serial ("FXA...") -> Steam can't associate the
		// controller with a puck -> "paired to" list is EMPTY -> Steam treats it as unconfigured and re-runs the
		// 0x81/0x87 config storm every connect (the buzz). So on rid 1, answer with the CONTROLLER's serial from
		// this interface's bond record (rec[8..24], the same 16-byte serial 0xA3 returns). Captured: Steam
		// hammered this read (AE x39 on rid1) exactly because the identity never matched. rid 2 = puck (unchanged).
		// idx 0/1/4 = board/unit/alt serial (real controller returns the SAME serial for 0 and 4; the clone
		// was returning "NA" for idx 4, which failed Steam's read -> retry).
		S.resp[0] = 0xAE;
		S.resp[1] = 0x14;
		S.resp[2] = idx;
		memset(S.resp + 3, 0, 60);
		if (rid == 1 && slot >= 0 && slot < NSLOT &&
		    g_slot[slot].used && (idx == 0 || idx == 1 || idx == 4)) {
			// THIS slot's paired-controller serial, straight from its bond record (rec[8..24], the same
			// 16-byte serial 0xA3 returns) -- per device, nothing hardcoded. Copied without a stack temp
			// (this runs on the fragile 800B usbd task; every byte off the stack helps under a Steam
			// re-enumeration burst).
			memcpy(S.resp + 3, g_slot[slot].rec + 8, 16);
		} else {
			// rid 2 = the puck's own board/unit serials (device-derived). Any other idx -> "NA".
			const char *s = (rid == 1)	       ? "NA" :
					(idx == 0 || idx == 4) ? g_board :
					(idx == 1)	       ? g_unit :
								 "NA";
			memcpy(S.resp + 3, s, strlen(s));
		}
		S.resp_len = 63;
		break;
	}

	// connection/version state per slot: value 0x02 = controller connected, 0x01 = not
	case 0xB4:
		// SDL Triton polls this on init; treat like Steam contact so we forward 0x45
		hostStampAlive();
		S.resp[0] = 0xB4;
		S.resp[1] = 0x01;
		S.resp[2] = (slot >= 0 && slot < NSLOT && !g_xbox &&
			     g_slot[slot].used &&
			     (millis() - g_connReplyMs[slot] < 500)) ?
				    0x02 :
				    0x01;
		S.resp_len = 63;
		break;
	case 0xAD:
		g_pairing = (pln > 0 && pl[0] != 0);
#if OPK_LOG
		Serial.printf("# pairing %s\n", g_pairing ? "ON" : "off");
#endif
		S.resp[0] = 0xAD;
		S.resp[1] = 0;
		S.resp_len = 63;
		break;
	case 0xA2: // write/clear THIS interface's slot
		if (len >= 24 && pln >= 24) {
			if (recEmpty(pl)) {
				S.used = false;
				memset(S.rec, 0, 24);
			} else {
				memcpy(S.rec, pl, 24);
				S.used = true;
			}
			g_dirty = true;
#if OPK_LOG
			Serial.printf("# slot %d %s\n", slot,
				      recEmpty(pl) ? "cleared" : "bonded");
#endif
		}
		S.resp[0] = 0xA2;
		S.resp[1] = 0;
		S.resp_len = 63;
		break;
	case 0xA3: // read THIS interface's slot
		S.resp[0] = 0xA3;
		S.resp[1] = 0x18;
		memset(S.resp + 2, 0, 24);
		if (S.used)
			memcpy(S.resp + 2, S.rec, 24);
		S.resp_len = 63;
		break;
	case 0x87:
		// SET_SETTINGS_VALUES: payload = N x [id][val16 LE]. Shadow each into g_setShadow[slot] so the 0x89
		// read-back below matches what Steam wrote -- this is THE fix that stops Steam's endless config-verify
		// retry (the 0x81/0x87 storm that buzzes the amp). (Still relayed to the controller by the feature-1
		// path above; this just maintains the dongle-side shadow the real controller would return on 0x89.)
		// ACK with [0x87][0], not the default payload echo (a clean success reply, like the real dongle).
		if (slot >= 0 && slot < NSLOT)
			for (uint16_t i = 0; i + 2 < pln; i += 3) {
				uint8_t id = pl[i];
				if (id < 0x53)
					g_setShadow[slot][id] =
						(uint16_t)(pl[i + 1] |
							   (pl[i + 2] << 8));
			}
		S.resp[0] = 0x87;
		S.resp[1] = 0;
		S.resp_len = 63;
		break;
	case 0x89: {
		// GET_SETTINGS_VALUES: Steam reads back setting `id` (payload[0]) from the id-indexed array. Answer
		// from the shadow 0x87 populated: [0x89][3][id][val16 LE]. (Steam didn't use 0x89 in the captured
		// session -- it provisions via 0xED below -- but this is the correct real-dongle behavior; harmless.)
		uint8_t id = (pln > 0) ? pl[0] : 0;
		uint16_t v = (slot >= 0 && slot < NSLOT && id < 0x53) ?
				     g_setShadow[slot][id] :
				     0;
		S.resp[0] = 0x89;
		S.resp[1] = 3;
		S.resp[2] = id;
		S.resp[3] = (uint8_t)v;
		S.resp[4] = (uint8_t)(v >> 8);
		S.resp_len = 63;
		break;
	}
	case 0xED: {
		// GET-SETTING-BY-PATH (keyed). Steam SETs the path string, then GETs the value; the REAL controller
		// returns the setting's value, but OpenPuck was ECHOING the path -> Steam sees the controller as
		// un-provisioned/un-bonded and re-runs its full LEGACY config every connect (0x81 CLEAR_DIGITAL_MAPPINGS
		// = the amp clicks/buzz; captured: 0xED reads of "esb/bond"/"user/wireless_transport" that got echoed).
		// Answer the paths Steam checks at connect so it treats the controller as provisioned + bonded:
		//   esb/bond          -> this slot's 24-byte bond record (the controller's record of ITS puck)
		//   esb/bond_2        -> a 2nd-puck bond; OpenPuck bonds one puck per slot -> absent (empty)
		//   user/wireless_transport -> 1 byte = the ACTIVE (connected) slot's transport code (see below)
		// Anything else -> empty ([0xED][0]) rather than a garbage path echo.
		const char *p = (const char *)pl;
		uint16_t pl_n = pln;
		auto pathIs = [&](const char *k) -> bool {
			uint16_t kl = (uint16_t)strlen(k);
			return pl_n >= kl && memcmp(p, k, kl) == 0 &&
			       (pl_n == kl || p[kl] == 0);
		};
		S.resp[0] = 0xED;
		if (slot >= 0 && slot < NSLOT && g_slot[slot].used &&
		    pathIs("esb/bond")) {
			S.resp[1] = 0x18;
			memcpy(S.resp + 2, g_slot[slot].rec, 24);
		} else if (pathIs("user/wireless_transport")) {
			// Steam reads this (a CONTROLLER setting, relayed rid1) to mark which bond slot is ACTIVE in the
			// pairing list, mapping active_slot = value XOR 2 (HANDOFF: slot0->0x02, slot1->0x03, slot2->0x00,
			// slot3->0x01). It's a single DONGLE-WIDE value = the currently-connected slot, NOT this
			// interface's own index (returning the raw index made Steam mark slot^2 active -> the live puck
			// showed "Inactive"). Report the connected slot's code so Steam marks the live puck active.
			int act = 0;
			unsigned long best = 0, now = millis();
			for (int s2 = 0; s2 < NSLOT; s2++)
				if (g_slot[s2].used && g_connReplyMs[s2] &&
				    (now - g_connReplyMs[s2]) < 1200u &&
				    (best == 0 || g_connReplyMs[s2] > best)) {
					best = g_connReplyMs[s2];
					act = s2;
				}
			S.resp[1] = 1;
			S.resp[2] = (uint8_t)(act ^ 2);
		} else {
			S.resp[1] =
				0; // absent/empty -- valid "no value" reply, not a path echo
		}
		S.resp_len = 63;
		break;
	}
	default:
		S.resp[0] = cmd;
		S.resp[1] = len;
		if (pln)
			memcpy(S.resp + 2, pl, pln > 60 ? 60 : pln);
		S.resp_len = 63;
		break;
	}
}
static uint16_t handleGet(int slot, uint8_t rid, hid_report_type_t type,
			  uint8_t *buf, uint16_t reqlen)
{
	(void)rid;
	if (type != HID_REPORT_TYPE_FEATURE)
		return 0;

	// Do NOT treat a feature GET as "Steam is driving." A bare read is weak evidence: on Linux the kernel
	// hid-steam driver (and any hidapi enumerator) issues GET_FEATURE probes even with Steam's window closed,
	// which pinned steamDrivingGamepad() true forever and blocked the Steam-closed lizard fallback (keyboard/
	// mouse dead, and the lizard-off writes those readers pair with the probe also killed the autonomous
	// touchpad haptics). Steam actually taking over is detected by its OUTPUT/SET WRITES (the 0x87 lizard-off
	// heartbeat ~every 3s, and 0x82 haptics), which stamp g_steamAliveMs on the handleSet paths. So drop the
	// GET-based stamp entirely; a read alone no longer suppresses lizard.
	Slot &S = g_slot[slot];
	uint16_t n = S.resp_len ? S.resp_len : 63;
	if (n > reqlen)
		n = reqlen;
	memcpy(buf, S.resp, n);
	// Flight recorder (every GET, un-deduped): a read STORM -- e.g. the "AE x39" identity hammering seen before
	// the identity fix -- is itself a wedge signal, so we want it filling the post-mortem trail if it happens.
	faultDiagTrace(FR_GET, (uint16_t)((rid << 8) | S.resp[0]));
	// Battery diagnostic: in gamepad (Steam) mode battery is read host-side via the feature channel, NOT the
	// forwarded 0x43 (that path is verbatim-identical to lizard mode, where battery works). Capture what Steam
	// GETs so a WebUSB-panel (or CDC) capture in Steam mode shows the report id it polls for battery (then we
	// answer it with g_battery in handleSet). De-duped by report id so the high-rate polling doesn't flood the
	// ring/console -- a freshly-requested id is logged once, then again only after 1s.
	{
		// Dedup by (rid, interface, response-cmd) so each interface's GET is captured (not collapsed to one
		// line across all 4 slots) -- the per-interface bond/identity responses are exactly what we need to
		// see -- while still not flooding on high-rate same-report polling.
		static uint8_t lastRid = 0xFF, lastSlot = 0xFF, lastCmd = 0xFF;
		static unsigned long lastMs = 0;
		if (rid != lastRid || (uint8_t)slot != lastSlot ||
		    S.resp[0] != lastCmd || millis() - lastMs > 1000) {
			lastRid = rid;
			lastSlot = (uint8_t)slot;
			lastCmd = S.resp[0];
			lastMs = millis();
			// ring marker 0xFC = "host feature GET" (panel renders it as "GET rid=.."); payload = what we returned
			hapLogAdd(0xFC, rid, S.resp, n);
			// serial feature-command capture: what Steam READ on this interface + our answer
			fcPush(1, slot, rid, S.resp[0], (uint8_t)n, S.resp + 2,
			       10);
#if OPK_LOG
			// Diagnostic build only -- no Serial.printf on the 800B usbd-task stack in production (LOCKUP
			// mitigation). The GET is captured to the OPK_LOG ring above for the WebUSB panel.
			if (Serial.availableForWrite() > 80)
				Serial.printf(
					"# GET if%d rid=%02X reqlen=%u -> %02X %02X %02X (batt=%u%%)\n",
					slot, rid, reqlen, S.resp[0], S.resp[1],
					S.resp[2],
					(slot >= 0 && slot < NSLOT) ?
						g_battery[slot] :
						0);
#endif
		}
	}
	return n;
}

// one callback pair per interface (the Adafruit core routes by interface to the matching object)
#define SLOTCB(N)                                                              \
	static void setcb##N(uint8_t r, hid_report_type_t t, uint8_t const *b, \
			     uint16_t n)                                       \
	{                                                                      \
		handleSet(N, r, t, b, n);                                      \
	}                                                                      \
	static uint16_t getcb##N(uint8_t r, hid_report_type_t t, uint8_t *bf,  \
				 uint16_t rl)                                  \
	{                                                                      \
		return handleGet(N, r, t, bf, rl);                             \
	}
// clang-format off
SLOTCB(0)
SLOTCB(1)
SLOTCB(2)
SLOTCB(3)
// clang-format on
typedef uint16_t (*getcb_t)(uint8_t, hid_report_type_t, uint8_t *, uint16_t);
typedef void (*setcb_t)(uint8_t, hid_report_type_t, uint8_t const *, uint16_t);
static getcb_t GETCB[NSLOT] = { getcb0, getcb1, getcb2, getcb3 };
static setcb_t SETCB[NSLOT] = { setcb0, setcb1, setcb2, setcb3 };

// ===================== IController =====================
void SteamPuckController::begin()
{
	USBDevice.setID(0x28DE, 0x1304);
	// Distinct bcdDevice so Windows keys a FRESH usbflags entry (cache is VID:PID:bcdDevice) and actually runs
	// MS OS 2.0 / WinUSB binding for the WebUSB vendor interface, instead of reusing a stale "no WinUSB" entry
	// tied to the real Steam Controller (28DE:1304, no WebUSB interface). The normal (wake-mouse) and one-shot
	// debug (CDC) boots present DIFFERENT interface sets, so they need DIFFERENT bcdDevice values or Windows
	// serves one's cached descriptor for the other across a reboot.
	USBDevice.setDeviceVersion(
		g_debugCdcThisBoot ?
			0x0212 :
			(g_usbMode == MODE_LIZARD ? 0x0213 : 0x0211));
	USBDevice.setManufacturerDescriptor("Valve Software");
	USBDevice.setProductDescriptor("Steam Controller Puck");
	const uint8_t *desc = (g_usbMode == MODE_LIZARD) ?
				      PUCK_LIZARD_HID_DESC :
				      PUCK_HID_DESC;
	const uint16_t descLen = (g_usbMode == MODE_LIZARD) ?
					 sizeof PUCK_LIZARD_HID_DESC :
					 sizeof PUCK_HID_DESC;
	for (int i = 0; i < NSLOT; i++) {
		hid[i].setReportDescriptor(desc, descLen);
		hid[i].setReportCallback(GETCB[i], SETCB[i]);

		// 1ms USB poll (was default 10ms = 100/s cap -> choppy)
		hid[i].setPollInterval(1);
		hid[i].begin();
	}
}

// Forward the controller's report 0x45 to Steam, or drive lizard kb/mouse when Steam is closed. PURELY a
// USB-side decision -- changes nothing about the RF poll or the host->controller relay. Per-slot: each
// controller's 0x45 goes to its OWN hid[slot], so Steam sees four independent inputs. Lizard is likewise
// per-slot now: each connected controller drives keyboard+mouse on its own hid[slot] (the OS merges the
// several HID mice/keyboards onto the one desktop), so any controller -- on any bond slot -- works.
void SteamPuckController::onReport45(int slot, const uint8_t *rep, bool fresh,
				     uint8_t bodyTlen)
{
	(void)fresh;
	(void)bodyTlen;
	// Host asleep -> forward NOTHING. While suspended, every sendReport attempt can translate into a host wake,
	// making the PC wake on any controller movement. Waking is an explicit gesture only (Steam-button short
	// press / controller connect), handled in rf_link.cpp via the device-level USB resume signal.
	if (USBDevice.suspended())
		return;
	// Just woke? Hold off forwarding so the wake gesture's residual controller input doesn't click/type into
	// the freshly-woken desktop.
	if (g_resumeMs && millis() - g_resumeMs < POST_RESUME_MUTE_MS)
		return;
	if (slot < 0 || slot >= NSLOT)
		return;
	if (!g_slot[slot].used)
		return;
	// Report id: 0x45 = legacy main input; 0x42 = new-firmware (SC2 beta ~2026-07) main input (53B). Forward
	// whichever the controller actually sent, verbatim, under its own id -- both are declared in PUCK_HID_DESC
	// and this is exactly what the real puck does. rep[0] is the id byte (rep points at it in the F1 TLV).
	const uint8_t rid = rep[0];
	if (lizardActive()) {
		// Every connected controller drives keyboard+mouse on ITS OWN slot interface; the OS merges the
		// multiple HID mice/keyboards onto the one desktop. (Was hardcoded to slot 0, which went silent
		// whenever the live controller bonded to a non-zero slot -- e.g. slot 0 holding a stale/phantom
		// bond from a cloned backup.) rfLizard keeps its glide/edge state per-slot so they don't clobber.
		// rfLizard reads 0x45 field offsets; report 0x42 is VERIFIED byte-identical over [0..45] (see the
		// rf_link decode note), so both ids drive lizard -- covers MODE_LIZARD and Steam-mode-with-Steam-
		// closed on new-firmware controllers.
		rfLizard(slot, rep, &hid[slot], &hid[slot], 0x40, 0x41);
	} else {
		// body length after the id byte, clamped to the descriptor's declared size for this report id
		// (0x42 = 53B vendor input, 0x45 = 45B input).
		uint8_t maxb = (rid == 0x42) ? 53 : 45;
		uint8_t blen = bodyTlen - 1;
		if (blen > maxb)
			blen = maxb;
		// forward the puck's raw pad coords untouched (Steam does its own interpolation/smoothing). Forward
		// only FRESH reports: the real puck dedupes, so stale repeats make Steam's velocity/smoothing
		// stair-step. g_fwdNewOnly toggles for A/B.
		if ((fresh || !g_fwdNewOnly) && hid[slot].ready())
			usbTxHid(
				&hid[slot], rid, rep + 1,
				blen); // Steam/SDL Triton: input report 0x45 (old) / 0x42 (new fw)
	}
}

// Forward the controller's NON-input status reports (0x43 power/battery, 0x44) to Steam verbatim -- the real
// puck does this and it's how Steam reads battery. Same host-asleep / post-resume gating as 0x45; no lizard
// path (status reports aren't input, so they forward regardless of the lizard decision). Per-slot: each
// controller's status goes to its own hid[slot].
void SteamPuckController::onAuxReport(int slot, uint8_t rid,
				      const uint8_t *data, uint8_t n)
{
	if (USBDevice.suspended())
		return;
	if (g_resumeMs && millis() - g_resumeMs < POST_RESUME_MUTE_MS)
		return;
	if (slot < 0 || slot >= NSLOT)
		return;
	// Forward the controller's status report VERBATIM (the real puck does this; it's how the host reads
	// battery). Padding the report to the descriptor-declared length broke battery in both lizard and
	// Steam, so it's reverted -- send exactly what the controller sent.
	if (g_slot[slot].used && hid[slot].ready()) {
		// capture the pushed status report (0x43 battery / 0x44) device->host for the WebUSB panel: this is
		// the channel Steam actually reads battery from; marker 0xFB = "->host push".
		hapLogAdd(0xFB, rid, data, n);
		usbTxHid(&hid[slot], rid, data, n);
	}
}

// wake nudge: a bare USB resume signal is NOT enough to wake some hosts (Windows in particular) -- they only
// wake when actual mouse/keyboard input follows. So on a deliberate wake gesture we play a HARMLESS mouse
// JIGGLE (move a few px right, then back -- NET ZERO cursor, NO button): real mouse activity wakes the host
// but clicks/activates nothing (an open Start menu stays open). Queued by wakeEvent() (rf_link, on a Steam
// short press / controller connect while suspended); delivered once the suspended bus has resumed. Per-slot
// so every connected interface gets the jiggle (Windows can credit any of them with the wake).
static uint8_t g_nudgeStep[NSLOT] = { 0 }; // 0=idle; 1=jiggle+, 2=jiggle-
static unsigned long g_nudgeMs[NSLOT] = { 0 };
#define NUDGE_JIGGLE_PX 10
void SteamPuckController::wakeEvent()
{
	// arm the jiggle on every connected slot -- any one of them is enough to wake Windows
	for (int s = 0; s < NSLOT; s++) {
		if (g_slot[s].used) {
			g_nudgeStep[s] = 1;
			g_nudgeMs[s] = millis();
		}
	}
}
static void wakeNudgeTask()
{
	if (USBDevice.suspended())
		return; // wait for resume; reports can't cross a suspended bus
	// Expire stale arms (bus never resumed) and see if any slot still wants a wake nudge.
	bool armed = false;
	for (int s = 0; s < NSLOT; s++) {
		if (!g_nudgeStep[s])
			continue;
		if (millis() - g_nudgeMs[s] > 5000) {
			g_nudgeStep[s] = 0;
			continue;
		}
		armed = true;
	}
	if (!armed)
		return;
	// Ride the BOOT MOUSE -- the interface Windows armed as the wake source; a gamepad-slot report does not
	// wake Modern Standby. A single jiggle on it wakes the host regardless of how many slots are connected.
	if (wakeHidPresent()) {
		if (!wakeHidReady())
			return;
		static unsigned long stepMs = 0;
		static uint8_t step = 1;
		if (millis() - stepMs < 15)
			return; // pace the edges
		stepMs = millis();
		wakeHidMove((step == 1) ? NUDGE_JIGGLE_PX : -NUDGE_JIGGLE_PX,
			    0);
		if (step >=
		    2) { // jiggle (right, then back) delivered -> disarm every slot
			step = 1;
			for (int s = 0; s < NSLOT; s++)
				g_nudgeStep[s] = 0;
		} else
			step++;
		return;
	}
	// Fallback (debug-CDC boot: no wake mouse): per-slot jiggle on the gamepad slot HIDs.
	static unsigned long stepMs[NSLOT] = { 0 };
	for (int s = 0; s < NSLOT; s++) {
		if (!g_nudgeStep[s])
			continue;
		if (!hid[s].ready())
			continue;
		if (millis() - stepMs[s] < 15)
			continue; // pace the edges
		stepMs[s] = millis();
		hid_mouse_report_t m;
		m.buttons = 0;
		m.x = (g_nudgeStep[s] == 1) ? NUDGE_JIGGLE_PX :
					      -NUDGE_JIGGLE_PX;
		m.y = 0;
		m.wheel = 0;
		m.pan = 0;
		usbTxHid(&hid[s], 0x40, &m,
			 sizeof m); // jiggle right, then back
		g_nudgeStep[s] = (g_nudgeStep[s] >= 2) ?
					 0 :
					 (uint8_t)(g_nudgeStep[s] + 1);
	}
}

// USB connection presentation (like the real dongle): report 0x79 = connection state (01=disc, 02=conn),
// edge-triggered, + periodic 0x7B status. Live-captured: this is what Steam reads to mark the controller
// connected. Without it Steam shows disconnected even though 0x45 input is streaming.
void SteamPuckController::task()
{
	wakeNudgeTask();
	{
		static bool wasSusp = false;

		// stamp the suspended->active edge for the post-resume mute
		bool susp = USBDevice.suspended();
		if (wasSusp && !susp)
			g_resumeMs = millis();
		wasSusp = susp;
	}

	// no periodic 0x79/0x7B while the host sleeps -- those sends can wake it too
	if (USBDevice.suspended())
		return;
	// Per-slot 0x79/0x7B: each connected slot reports its OWN edge and its OWN status. State arrays are
	// per-slot so each controller's "connected" edge fires once and is re-sent only until Steam acks THAT
	// slot. The real puck's per-slot edge-triggered 0x79 prevents re-triggering Steam's connect-chime loop.
	static bool usbConn[NSLOT] = { 0 };
	static unsigned long last79[NSLOT] = { 0 }, last7B[NSLOT] = { 0 },
			     connEdgeMs[NSLOT] = { 0 }, last43[NSLOT] = { 0 };
	for (int s = 0; s < NSLOT; s++) {
		if (!g_slot[s].used || !hid[s].ready())
			continue;
		bool conn = (millis() - g_connReplyMs[s] < 300);
		// 0x79 connection state: on edge, then repeated every 750ms ONLY until Steam reacts (its first OUTPUT/
		// settings write after the edge -- g_steamAliveMs). The real puck sends 0x79 ONCE, edge-triggered; an
		// unconditional forever-resend re-triggers Steam's connect handling (connect chime) every 750ms before
		// Steam consumes 0x45 -> a loop of connect-time haptic buzzes. Resending until acked still covers
		// "Steam missed the edge".
		bool steamAcked = g_steamAliveMs &&
				  (int32_t)(g_steamAliveMs - connEdgeMs[s]) >=
					  0;
		if (conn != usbConn[s] ||
		    (conn && !steamAcked && millis() - last79[s] >= 750)) {
			if (conn && !usbConn[s])
				connEdgeMs[s] = millis();
			uint8_t st = conn ? 0x02 : 0x01;
			hapLogAdd(0xFB, 0x79, &st, 1); // ->host push (capture)
			usbTxHid(&hid[s], 0x79, &st, 1);
			usbConn[s] = conn;
			last79[s] = millis();
		} else if (conn && millis() - last7B[s] >= 2000) {
			// 0x7B status, live-captured template. Byte 8 is the controller->puck signal strength as signed
			// dBm (capture showed 0xDD = -35) -- patch in the smoothed RSSI the radio samples on each
			// controller reply (rf_link). 0 = no sample yet -> keep the capture value rather than garbage.
			//
			// CALIBRATION: our raw RSSISAMPLE reads ~RSSI_DBM_OFFSET dB lower than the real Valve puck at the
			// same distance (Pro Micro PCB-trace antenna vs Valve's tuned front-end). The 2Mbit ESB link has
			// ~55dB margin, so a -75dBm reading still works across a house, but Steam's bar maps raw dBm to
			// "weak" long before that. The offset lines our close-range value up with the puck's captured -35
			// so the bar tracks usable range, not antenna gain. Clamp keeps it in a sane window.
			uint8_t s7b[12] = { 0xF7, 0x01, 0x89, 0x00, 0x00, 0x00,
					    0x03, 0x00, 0xDD, 0x00, 0x3A, 0x02 };
			if (g_linkRssi[s]) {
				int mag = (int)g_linkRssi[s] - RSSI_DBM_OFFSET;
				if (mag < 25)
					mag = 25;
				else if (mag > 95)
					mag = 95;
				s7b[8] = (uint8_t)(0u - (uint8_t)mag);
			}
			hapLogAdd(0xFB, 0x7B, s7b, 12); // ->host push (capture)
			usbTxHid(&hid[s], 0x7B, s7b, 12);
			last7B[s] = millis();
		}
		// Synthesized 0x43 = ID_TRITON_BATTERY_STATUS for SDL/Steam's gamepad driver. The verbatim forward of
		// the controller's own 0x43 (onAuxReport) is what the LIZARD/kernel path reads, but SDL's Triton driver
		// requires the FULL TritonBatteryStatus_t length (r >= 1 + 14) and lapses to the "wired" glyph without a
		// fresh one -- so we push a clean 14-byte report from g_battery every 2s. Body: [ucChargeState][ucBattery
		// Level][voltages/current/temp = 0]; SDL only reads the first two. Map unknown/reset state -> discharging
		// so it shows ON_BATTERY + % rather than UNKNOWN. Skipped until the controller has reported a level.
		if (conn && g_battery[s] && millis() - last43[s] >= 2000) {
			uint8_t st = g_batteryState[s];
			// EChargeState discharging -> SDL_POWERSTATE_ON_BATTERY
			if (st != 1 && st != 2 && st != 4)
				st = 1;
			uint8_t b43[14] = { 0 };
			b43[0] = st; // ucChargeState
			b43[1] = g_battery[s]; // ucBatteryLevel (percent)
			hapLogAdd(0xFB, 0x43, b43, 14); // ->host push (capture)
			usbTxHid(&hid[s], 0x43, b43, sizeof b43);
			last43[s] = millis();
		}
	}
	// Reset edge state for slots that are no longer used/ready (so a re-bond sees a fresh edge).
	for (int s = 0; s < NSLOT; s++)
		if (!g_slot[s].used || !hid[s].ready())
			usbConn[s] = false;
}
