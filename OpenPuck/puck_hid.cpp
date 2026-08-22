#include "puck_hid.h"
#include "bonds.h"
#include "config.h"
#include "identity.h"
#include "haptics.h"
#include "fault_diag.h"
#include "rf_link.h"
#include "triton.h"
#include "mode_lizard.h"
#include "steam_commands.h"
#include "wake_hid.h"
#include "build_info.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

uint8_t g_fwdNewOnly = 1;
// Content dedup for the Steam input forward: only forward a 0x45/0x42 report when its body EXCLUDING the
// counter byte differs from the last one forwarded on that slot. The controller's counter (rep[1]) FREE-RUNS
// -- it advances on every reply even with static input -- so the seq-based g_fwdNewOnly can't suppress true
// repeats, and a poll rate above the controller's ~250 Hz sample rate forwards identical inputs with only the
// counter bumped (host tester reads >250/s for a <=250 Hz controller). Content dedup is what the real puck
// effectively does. On by default; console "CD" toggles for A/B.
uint8_t g_fwdContentDedup = 1;
SteamPuckController g_steamPuck;

// millis() a power-off (0x9F) was last relayed to each slot; 0 = never. See puckNotePowerOff() / task().
static volatile unsigned long g_powerOffMs[NSLOT] = { 0 };
// Present a powered-off slot as DISCONNECTED to Steam for this long, to ride out the controller's post-off
// F1 tail (measured ~<=1s of dying replies after the "off!" command) without bouncing the connection state.
#define POWEROFF_HOLD_MS 2000u
void puckNotePowerOff(uint8_t slot)
{
	if (slot >=
	    NSLOT) { // 0xFF broadcast ("all off": host suspend, panel/test button)
		for (int s = 0; s < NSLOT; s++)
			if (g_slot[s].used)
				g_powerOffMs[s] = millis();
	} else {
		g_powerOffMs[slot] = millis();
	}
}
// True while `slot` is inside its post-power-off hold window (present it disconnected, ignore dying replies).
static inline bool slotPoweringOff(int slot)
{
	return slot >= 0 && slot < NSLOT && g_powerOffMs[slot] &&
	       (millis() - g_powerOffMs[slot] < POWEROFF_HOLD_MS);
}

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
// How long to keep re-asserting the 0x79 DISCONNECT edge to Steam after a controller drops, so a single
// lost report can't strand Steam in "connected" (see task()). Bounded to avoid forever-spamming disc.
#define DISC_RESEND_MS 6000u

// ===================== puck feature command channel =====================
// `slot` is the interface index (interface N == bond slot N).
static void handleSet(int slot, uint8_t rid, hid_report_type_t type,
		      uint8_t const *b, uint16_t n)
{
	// Steam OUTPUT reports 0x80-0x89. The haptic/actuator reports (0x80-0x86) are relayed to the controller,
	// and ONLY when they arrive on the CONNECTED slot's interface: we have one controller but expose 4 puck
	// slots, and a report aimed at a DIFFERENT slot made the controller buzz at random (the slot gate below
	// fixes that). Must NOT clamp to 0x82-only -- Steam drives the actuators through the WHOLE Triton OUTPUT
	// report space (ValveTritonOutReportMessageIDs, SDL steam/controller_structs.h; the ids match this
	// interface's descriptor payload sizes exactly):
	//   0x80 HAPTIC_RUMBLE(9)  0x81 HAPTIC_PULSE(7)   0x82 HAPTIC_COMMAND(3)
	//   0x83 HAPTIC_LFO_TONE(9) 0x84 HAPTIC_LOG_SWEEP(8) 0x85 HAPTIC_SCRIPT(3)  0x86 (3, unnamed)
	// These ids are NOT the feature-0x01 command ids (controller_constants.h) -- same numbers, different
	// meanings -- so a rule written for one channel must never be applied to the other.
	// The 63-byte settings/config reports 0x87/0x88/0x89 are NOT haptics and reach the controller via the
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
		// NOTE: report id 0x81 is NOT dropped here, unlike the feature-0x01 cmd 0x81. The two 0x81s live in
		// DIFFERENT id spaces (SDL steam/controller_structs.h vs controller_constants.h):
		//   OUTPUT  rid 0x81 = ID_OUT_REPORT_HAPTIC_PULSE, MsgHapticPulse {u8 side; u16 on_us; u16 off_us;
		//                      u16 repeat_count} = the 7-byte "00.."/"01.." (side=left/right) reports Steam
		//                      fires for trackpad CLICK feedback ("Regular Press"), the trigger FULL-PULL
		//                      click and GripSense cues.
		//   FEATURE cmd 0x81 = ID_CLEAR_DIGITAL_MAPPINGS (the mapping-engine reset, dropped below).
		// Dropping the OUTPUT form as if it were the same command is what made those haptics missing
		// (issues #163 / #166): "Soft Press" survived because it rides 0x82 ID_OUT_REPORT_HAPTIC_COMMAND,
		// while every pulse-based click was thrown away. Pulses are self-terminating (repeat_count in the
		// payload), so there is no latch to strand and no stop frame to lose.
		if (g_hapticRelay && rid >= 0x80 && rid <= 0x86 && n >= 1 &&
		    hapticRelaySlotOk(slot) && !lizardActive() && !muted) {
			if (!haptic82Blocked(slot)) {
				relayEnqueue(rid, b,
					     (uint8_t)(n > RELAY_MAXP ?
							       RELAY_MAXP :
							       n),
					     true, (uint8_t)slot);
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

	// Disable lizard if requested by host.
	if (cmd == IBEX_CMD_SET_SETTINGS_VALUES) {
		// Host sent a settings write. Check if this is a write with the lizard suppression tag.
		for (int off = 0; off <= len; off += 3) {
			uint8_t settings_key = pl[off];
			uint16_t settings_val = pl[off + 1] + 256 * pl[off + 2];
			if (settings_key == SETTING_LIZARD_MODE &&
			    settings_val == 0) {
				hostStampAlive();
				break;
			}
		}
	}

	// Controller power-off: Steam's "turn off controller" is feature-0x01 frame 9F 04 6F 66 66 21 ("off!"),
	// confirmed from a real puck capture. The feature-0x01 relay below forwards it once; hapticSendShutdown
	// bursts it for NO-ACK reliability. Slot-targeted: the command arrived on THIS controller's interface,
	// so only this controller powers off (broadcasting killed all connected controllers at once).

	// TODO: Why do we spam this? Doesn't the controller respond with a TAG4 upon command receival?
	if (rid == 1 && cmd == IBEX_CMD_TURN_OFF_CONTROLLER)
		hapticSendShutdown((uint8_t)slot);

	// report 0x01 = raw passthrough -> queue for RF relay to the controller
	if (rid == 1 && n >= 2) {
		// (feature-1 commands -- haptics, LED, 0x87 settings, 0x9F power-off -- are captured by the
		// general feature-SET hapLogAdd above.)

		bool muted = g_resumeMs &&
			     millis() - g_resumeMs < POST_RESUME_MUTE_MS;

		// Reports in relayQuery will be forwarded to the controller if rid==1 and may be answered locally
		// if it makes sense and rid==2.

		// Reports in localAnswer will always be answered by the puck and not be forwarded.

		// Forwarding uses the type-01 framing PLUS a trailing `01 03 00` (rfConnFlushRelay's `queryTrailer`,
		// haptics.cpp -- without it the controller only ACKs and never answers), the controller replies --
		// on a LATER poll -- with a tag-4 TLV carrying `[echoed cmd][len][payload]`, decoded in rf_link.cpp's
		// F1 walk and written into this slot's `resp` (pendingQueryCmd, bonds.h).

		// If rid==1:
		// ID in relayQuery: Sent to controller, expect a response.
		// ID in localAnswer: Not sent to controller, response generated by Puck.
		// ID in neither: Sent to controller, no response expected.

		// clang-format off

		bool relayQuery =
			(cmd == IBEX_CMD_GET_DIGITAL_MAPPINGS ||    // 0x82
			 cmd == IBEX_CMD_GET_ATTRIBUTES_VALUES ||   // 0x83
			 cmd == IBEX_CMD_GET_SETTINGS_VALUES ||     // 0x89
			 cmd == IBEX_CMD_GET_SETTINGS_MAXS ||       // 0x8B
			 cmd == IBEX_CMD_GET_SETTINGS_DEFAULTS ||   // 0x8C
			 cmd == IBEX_CMD_GET_DEVICE_INFO ||         // 0xA1
			 cmd == IBEX_CMD_GET_STRING_ATTRIBUTE ||    // 0xAE
			 cmd == IBEX_CMD_GET_CHIPID ||              // 0xBA
			 cmd == IBEX_CMD_TRITON_READ_SETTING ||     // 0xED
			 cmd == IBEX_CMD_GET_SYSTEM_INFO);          // 0xF2

		bool localAnswer =
			(cmd == IBEX_CMD_TRITON_A2_OBSERVED_PAIRING_RECORD || // 0xA2
			 cmd == IBEX_CMD_TRITON_A3_BOND_EVENT_OR_STATUS ||    // 0xA3
			 cmd == IBEX_CMD_ENABLE_PAIRING ||                    // 0xAD
			 cmd == IBEX_CMD_DONGLE_GET_WIRELESS_STATE ||         // 0xB4
			 cmd == 0xA4); // Not sure what that's for?

		// clang-format on

		bool queryArmed = false;
		bool relayOk = hapticRelaySlotOk(slot) && !localAnswer;
		if (relayOk) {
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
			relayEnqueue(cmd, pl, rl, false, (uint8_t)slot,
				     relayQuery);

			if (relayQuery && slot >= 0 && slot < NSLOT) {
				g_slot[slot].pendingQueryCmd = cmd;
				queryArmed = true;
			}
		}
		// No real controller answer is EVER coming for this SET: it wasn't a tracked query
		// (haptics/0x87/0x9F/localAnswer/...), or it was one but couldn't relay right now (link down,
		// blocked). `resp` must still be made to reflect CMD, not left holding whatever unrelated
		// command's data happened to be cached there from an earlier request -- otherwise a later
		// GET(rid=1), seeing pendingQueryCmd==0, would silently hand Steam stale/wrong data instead of
		// a defined answer. Same echo shape the old always-run `default:` switch case used.
		if (!queryArmed) {
			S.resp[0] = cmd;
			S.resp[1] = len;
			if (pln)
				memcpy(S.resp + 2, pl, pln > 60 ? 60 : pln);
			S.resp_len = 63;
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
	if (rid == 2) {
		memset(S.resp, 0, sizeof S.resp);
		S.resp_len = 0;
	} else {
		// If we forwarded the packet to the controller, do NOT clear the buffer.
		// The controller will respond and put its data into the buffer.
		return;
		// rid == 1 will be handled by the controller, no need to respond here.
	}
	switch (cmd) {
	case IBEX_CMD_GET_ATTRIBUTES_VALUES: // 0x83
		S.resp[0] = IBEX_CMD_GET_ATTRIBUTES_VALUES;
		if (g_isMachineInternal) {
			S.resp[1] = sizeof ATTR83_MACHINE;
			memcpy(S.resp + 2, ATTR83_MACHINE,
			       sizeof ATTR83_MACHINE);
			S.resp_len = 63;
		} else {
			S.resp[1] = sizeof ATTR83_PUCK;
			memcpy(S.resp + 2, ATTR83_PUCK, sizeof ATTR83_PUCK);
			S.resp_len = 63;
		}
		break;
	case IBEX_CMD_GET_STRING_ATTRIBUTE: { // 0xAE
		uint8_t idx = pln > 0 ? pl[0] : 1;
		// Report-id 1 = string attributes of the bonded CONTROLLER, not the puck. Not handled here, this request
		// will have been forwarded to the controller by earlier code.
		S.resp[0] = IBEX_CMD_GET_STRING_ATTRIBUTE;
		S.resp[1] = 0x14; // todo: is this correct?
		S.resp[2] = idx;
		memset(S.resp + 3, 0, 60);
		// Any other idx -> "NA".
		const char *s = (idx == 0 || idx == 4) ? g_board :
				(idx == 1)	       ? g_unit :
				(idx == 3)	       ? "OpenPuck " :
							 "NA";
		memcpy(S.resp + 3, s, strlen(s));
		if (idx == 3) {
			// The official puck has a 12-character GIT commit hash of the firmware in here.
			// Since I doubt any official process is ever going to parse / use this,
			// looks like the perfect place to put the OpenPuck version number.
			char *off = (char *)(S.resp + 3 + strlen(s));
			memcpy(off, OPK_BUILD_VERSION,
			       strlen(OPK_BUILD_VERSION));
			off += strlen(OPK_BUILD_VERSION);
			memcpy(off, " ", 1);
			memcpy(off + 1, OPK_GIT_HASH, strlen(OPK_GIT_HASH));
			if (OPK_GIT_DIRTY != 0) {
				off += 1 + strlen(OPK_GIT_HASH);
				memcpy(off, "-dirty", 6);
			}
		}
		S.resp_len = 63;
		break;
	}

	// connection/version state per slot: value 0x02 = controller connected, 0x01 = not
	case IBEX_CMD_DONGLE_GET_WIRELESS_STATE: // 0xB4
		// SDL Triton polls this on init; treat like Steam contact so we forward 0x45
		hostStampAlive();
		S.resp[0] = IBEX_CMD_DONGLE_GET_WIRELESS_STATE;
		S.resp[1] = 0x01;
		// Report disconnected during the post-power-off hold so B4 agrees with the 0x79 disconnect (they used
		// different windows -- 500ms here vs the 300ms conn/DOWN edge -- so right after a power-off Steam's B4
		// probe answered "still connected" and contradicted the disconnect we just pushed).
		S.resp[2] = (slot >= 0 && slot < NSLOT && !g_xbox &&
			     g_slot[slot].used && !slotPoweringOff(slot) &&
			     (millis() - g_connReplyMs[slot] < 500)) ?
				    0x02 :
				    0x01;
		S.resp_len = 63;
		break;
	case IBEX_CMD_ENABLE_PAIRING: // 0xAD
		// TODO: Check if this should be relayed?
		g_pairing = (pln > 0 && pl[0] != 0);
#if OPK_LOG
		Serial.printf("# pairing %s\n", g_pairing ? "ON" : "off");
#endif
		S.resp[0] = IBEX_CMD_ENABLE_PAIRING;
		S.resp[1] = 0;
		S.resp_len = 63;
		break;
	case IBEX_CMD_TRITON_A2_OBSERVED_PAIRING_RECORD: // 0xA2: write/clear THIS interface's slot
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
		S.resp[0] = IBEX_CMD_TRITON_A2_OBSERVED_PAIRING_RECORD;
		S.resp[1] = 0;
		S.resp_len = 63;
		break;
	case IBEX_CMD_TRITON_A3_BOND_EVENT_OR_STATUS: // 0xA3: read THIS interface's slot
		S.resp[0] = IBEX_CMD_TRITON_A3_BOND_EVENT_OR_STATUS;
		S.resp[1] = 0x18;
		memset(S.resp + 2, 0, 24);
		if (S.used)
			memcpy(S.resp + 2, S.rec, 24);
		S.resp_len = 63;
		break;
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

	// A feature-01 query (0x83/0xAE/0xED, relayed for real when rid==1 -- see handleSet's `relayQuery`)
	// is still waiting on the controller's actual reply: `resp` only holds the local placeholder for it
	// right now (pendingQueryCmd, bonds.h; cleared by rf_link.cpp when the real answer lands).
	// STALL this GET so the host retries shortly after (-EPIPE), like the real Puck does.
	//
	// Returning plain 0 does NOT stall here: this device's reports are all NUMBERED (report id 1/2), and
	// TinyUSB's callback (adafruit/Adafruit_TinyUSB_Arduino, src/class/hid/hid_device.c)
	// auto-prepends that report id byte into the reply BEFORE calling this callback --
	//   uint16_t xferlen = 0;
	//   if (report_id != HID_REPORT_TYPE_INVALID && req_len > 1) { *report_buf++ = report_id; xferlen++; }
	//   xferlen += tud_hid_get_report_cb(...);   // <- our return value lands here
	//   TU_ASSERT(xferlen > 0);                  // stalls the control transfer iff this is false
	// -- so xferlen is already 1 before we're even asked, and adding our 0 leaves it at 1: the assert
	// never fires and TinyUSB happily ships that lone prepended report-id byte as a "successful" 1-byte
	// reply.
	// This bug has been reported to upstream at https://github.com/hathach/tinyusb/issues/3814
	//
	// `xferlen` is uint16_t, so returning (uint16_t)-1 (0xFFFF) makes `1 + 0xFFFF` wrap to exactly 0
	// -- landing on the SAME TU_ASSERT(xferlen > 0) failure a genuinely-empty callback would have hit,

	// The additional check for rid is necessary so if the controller never replies, a new query for a
	// puck feature can get the puck un-stuck.

	// TODO: This is a very, very, very ugly solution and may break with library updates. Can we find a cleaner one?
	if (rid == 1 && S.pendingQueryCmd != 0 && reqlen > 1) {
		return 0xFFFF;
	}

	// rf_link.cpp will set pendingQueryCmd to 0 once the controller answered.

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
	if (g_isMachineInternal) {
		// Emulating a Steam Machine's internal receiver.
		USBDevice.setID(0x28DE, 0x1305);
	} else {
		// Emulating a Steam Controller Puck.
		USBDevice.setID(0x28DE, 0x1304);
	}
	USBDevice.setVersion(0x0201); // bcdUSB 2.01
	USBDevice.setDeviceVersion(2);

	// Distinct serial number so Windows keys a FRESH usbflags entry.
	if (g_debugCdcThisBoot) {
		snprintf(g_usbSerial, sizeof g_usbSerial, "%sC", g_unit);
		USBDevice.setSerialDescriptor(g_usbSerial);
	} else if (g_usbMode == MODE_LIZARD) {
		snprintf(g_usbSerial, sizeof g_usbSerial, "%sL", g_unit);
		USBDevice.setSerialDescriptor(g_usbSerial);
	} else {
		USBDevice.setSerialDescriptor(g_unit);
	}

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
// controller's 0x45 goes to its OWN hid[slot], so Steam sees four independent inputs. Lizard, by contrast,
// presents ONE mouse + keyboard (a desktop can only consume one): rfLizard merges every bonded controller's
// input (g_in[] across all used slots, populated by the per-slot decode) onto hid[0], so all connected
// controllers drive the same cursor/keys together -- and it works regardless of which bond slot is live.
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
		// All bonded controllers share ONE desktop mouse/keyboard (mounted on hid[0] regardless of
		// which RF slots are in use). rfLizard merges g_in[] across every used slot internally, so we
		// fire it ONCE per cycle -- gated on the lowest CONNECTED slot's report so it runs at a steady
		// cadence (not once per slot, which would multiply the output rate). g_in[] is populated by the
		// per-slot decode for BOTH report 0x45 and the beta 0x42 (byte-identical front section), so
		// lizard works on new-firmware controllers too without reading rep[] here.
		// Gate on the lowest RECENTLY-REPLYING slot, not merely the lowest bonded one: a bonded-but-
		// offline low slot (e.g. a stale/phantom bond from a cloned backup) must not starve lizard when
		// a higher slot is the live controller (main's #98 fix -- do not regress it). onReport45 only
		// runs for a slot that just decoded a report, so the reporting slot is itself connected.
		unsigned long now = millis();
		int lizSlot = -1;
		for (int s = 0; s < NSLOT; s++)
			if (g_slot[s].used && g_connReplyMs[s] &&
			    (now - g_connReplyMs[s]) < 1200u) {
				lizSlot = s;
				break;
			}
		if (slot == lizSlot)
			rfLizard(&hid[0], &hid[0], 0x40, 0x41);
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
		//
		// Do NOT gate on hid[slot].ready() here: usbTxHid enqueues into a ring buffer (drop-oldest)
		// that exists precisely to hold a report while the endpoint is briefly busy -- usbTxDrain
		// sends it the instant the host next polls. Gating on ready() at enqueue time DROPPED every
		// fresh report that landed while the endpoint was busy (~1 ms after each send), which at
		// 300+ captured reports/s silently discarded ~a third of them (measured: RF new/s ~313 but
		// host delivered ~180). Always enqueue; the ring paces delivery to the host without loss.
		bool send = (fresh || !g_fwdNewOnly);
		// Content dedup: the counter byte (rep[1]) free-runs, so drop a report whose body AFTER the
		// counter is byte-identical to the last one forwarded on this slot -- a true repeat (same
		// physical input, bumped counter), not a new sample. Caps delivery at the controller's real
		// distinct-report rate instead of the (higher) poll rate. See g_fwdContentDedup above.
		if (send && g_fwdContentDedup && blen >= 2) {
			static uint8_t lastBody[NSLOT][53];
			static uint8_t lastBodyLen[NSLOT] = { 0 };
			uint8_t clen =
				(uint8_t)(blen - 1); // bytes after the counter
			if (clen > sizeof lastBody[0])
				clen = sizeof lastBody[0];
			if (lastBodyLen[slot] == clen &&
			    memcmp(lastBody[slot], rep + 2, clen) == 0)
				send = false; // identical input, only the counter moved
			else {
				lastBodyLen[slot] = clen;
				memcpy(lastBody[slot], rep + 2, clen);
			}
		}
		if (send)
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

	// Lizard<->Steam handoff: release whatever the OUTGOING path was holding. lizardActive() is a pure
	// runtime decision -- Steam opening/closing (its ~3s settings heartbeat starting/stopping) flips it with
	// NO USB re-enumeration -- so a key/mouse-button held when Steam TAKES OVER, or a gamepad button held
	// when Steam CLOSES, is otherwise never released and sticks on the host until a reconnect or power-cycle
	// (the reported "stuck input after switching modes"). The link-drop neutral in rf_link only covers an RF
	// outage, not this handoff. Fire a neutral through the path we're leaving on each edge. Placed after the
	// suspend early-return: while suspended nothing can be sent, and a flip across a sleep is covered on
	// resume (this edge fires once, plus the post-resume input mute). MODE_LIZARD never leaves lizard, so
	// this is inert there.
	{
		static bool wasLizard = lizardActive();
		bool nowLizard = lizardActive();
		if (nowLizard != wasLizard) {
			if (wasLizard) {
				// leaving lizard -> release held desktop keyboard/mouse/consumer
				rfLizardRelease(&hid[0], &hid[0], 0x40, 0x41);
			} else {
				// leaving gamepad-forward -> release held 0x45 buttons/sticks/triggers on every slot
				static const uint8_t neutral45[45] = { 0 };
				for (int s = 0; s < NSLOT; s++)
					if (g_slot[s].used && hid[s].ready())
						usbTxHid(&hid[s], 0x45,
							 neutral45,
							 sizeof neutral45);
			}
			wasLizard = nowLizard;
		}
	}
	// Per-slot 0x79/0x7B: each connected slot reports its OWN edge and its OWN status. State arrays are
	// per-slot so each controller's "connected" edge fires once and is re-sent only until Steam acks THAT
	// slot. The real puck's per-slot edge-triggered 0x79 prevents re-triggering Steam's connect-chime loop.
	static bool usbConn[NSLOT] = { 0 };
	static unsigned long last79[NSLOT] = { 0 }, last7B[NSLOT] = { 0 },
			     connEdgeMs[NSLOT] = { 0 },
			     discEdgeMs[NSLOT] = { 0 }, last43[NSLOT] = { 0 },
			     poHandled[NSLOT] = { 0 };
	for (int s = 0; s < NSLOT; s++) {
		if (!g_slot[s].used || !hid[s].ready())
			continue;
		// Power-off just relayed to this slot -> force ONE clean disconnect to Steam now, regardless of the
		// prior edge state. The controller's noisy shutdown (it keeps streaming F1 for ~1s after "off!") can
		// leave usbConn desynced from what Steam shows, so the ordinary conn!=usbConn edge sometimes never
		// fired -> Steam kept the controller in its list (the "doesn't get removed" case). Anchor discEdgeMs
		// too so the bounded resend covers a dropped packet.
		if (g_powerOffMs[s] && g_powerOffMs[s] != poHandled[s]) {
			poHandled[s] = g_powerOffMs[s];
			uint8_t st = 0x01;
			hapLogAdd(0xFB, 0x79, &st, 1); // ->host push (capture)
			usbTxHid(&hid[s], 0x79, &st, 1);
			usbConn[s] = false;
			discEdgeMs[s] = millis();
			last79[s] = millis();
		}
		// Hold the slot DISCONNECTED through the controller's post-power-off F1 tail (see slotPoweringOff);
		// otherwise a stray dying reply bounces conn true -> a phantom 0x79=02 that Steam reads as a reconnect
		// and answers by re-running its connect config (the "reappears for a split second").
		bool conn = !slotPoweringOff(s) &&
			    (millis() - g_connReplyMs[s] < 300);
		// 0x79 connection state: on edge, then repeated every 750ms ONLY until Steam reacts (its first OUTPUT/
		// settings write after the edge -- g_steamAliveMs). The real puck sends 0x79 ONCE, edge-triggered; an
		// unconditional forever-resend re-triggers Steam's connect handling (connect chime) every 750ms before
		// Steam consumes 0x45 -> a loop of connect-time haptic buzzes. Resending until acked still covers
		// "Steam missed the edge".
		bool steamAcked = g_steamAliveMs &&
				  (int32_t)(g_steamAliveMs - connEdgeMs[s]) >=
					  0;
		// The DISCONNECT edge (0x79=01) also needs resending: it used to be sent exactly ONCE, so a single
		// dropped report left Steam believing the controller was still connected indefinitely -- observed as
		// "WebUSB panel shows the slot DOWN (g_connReplyMs stale) while Steam still lists it connected, and the
		// controller light is solid (it re-adopted the beacon) but no input flows." Resend disc every 750ms but
		// BOUNDED to DISC_RESEND_MS so a lost packet converges without the forever-spam the "real puck sends
		// 0x79 once" note warns against.
		bool discResend = !conn &&
				  (millis() - discEdgeMs[s] < DISC_RESEND_MS);
		if (conn != usbConn[s] ||
		    (conn && !steamAcked && millis() - last79[s] >= 750) ||
		    (discResend && millis() - last79[s] >= 750)) {
			if (conn && !usbConn[s])
				connEdgeMs[s] = millis();
			if (!conn &&
			    usbConn[s]) // connect->disc edge: anchor the resend window
				discEdgeMs[s] = millis();
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
