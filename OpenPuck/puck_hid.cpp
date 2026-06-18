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
		if (rid >= 0x80 && rid <= 0x86 && n >= 1 &&
		    hapticRelaySlotOk(slot) && !lizardActive() && !muted) {
			if (!haptic82Blocked()) {
				relayEnqueue(rid, b,
					     (uint8_t)(n > RELAY_MAXP ?
							       RELAY_MAXP :
							       n));
				// Track on/off from what was actually RELAYED (= the controller's believed state): a
				// BLOCKED stop must leave "on" set (controller may be latched -> reconnect stop-burst),
				// a blocked ON must not set it (nothing reached the controller -> no spurious clicks).
				if (rid == 0x82)
					haptic82HostReport(b, n);
			}
		}

		if (Serial.availableForWrite() > 80) {
			Serial.printf("# OUT if%d rid=%02X n=%u:", slot, rid,
				      n);
			for (uint16_t i = 0; i < n && i < 14; i++)
				Serial.printf(" %02X", b[i]);
			Serial.println();
		}
		return;
	}
	if (type != HID_REPORT_TYPE_FEATURE || n < 1)
		return;
	Slot &S = g_slot[slot];
	uint8_t cmd = b[0], len = (n > 1) ? b[1] : 0;
	const uint8_t *pl = b + 2;
	uint16_t pln = (n >= 2) ? n - 2 : 0;

	// settings/haptic/LED report (incl. 0x87 lizard-off heartbeat, SDL Triton lizard-disable)
	if (cmd >= 0x80 && cmd <= 0x89)
		hostStampAlive();
	// Controller power-off: Steam's "turn off controller" is feature-0x01 frame 9F 04 6F 66 66 21 ("off!"),
	// confirmed from a real puck capture. The feature-0x01 relay below forwards it once; hapticSendShutdown()
	// bursts it for NO-ACK reliability (the single hook the test button + host-suspend also drive).
	if (rid == 1 && cmd == 0x9F)
		hapticSendShutdown();

	// report 0x01 = raw passthrough -> queue for RF relay to the controller
	if (rid == 1 && n >= 2) {
		// capture EVERY relayed feature-1 command for the WebUSB capture view (haptics, LED SET_LED_COLOR,
		// 0x87 settings, 0x9F power-off). Log shows cmd as "rid"; bytes start [cmd][len]...
		hapLogAdd((uint8_t)slot, cmd, b, n);
		bool haptic82 = (cmd == 0x82 && len <= pln);

		bool muted = g_resumeMs &&
			     millis() - g_resumeMs < POST_RESUME_MUTE_MS;

		// never push haptics while presenting lizard (Steam isn't reading 0x45 -> would buzz-loop)
		bool relayOk = hapticRelaySlotOk(slot) &&
			       !(haptic82 && (lizardActive() || muted));
		if (relayOk && (!haptic82 || !haptic82Blocked())) {
			// Relay the DECLARED length (up to the 60B RF frame ceiling), not a truncation: Steam's
			// multi-register 0x87 settings blocks (LED brightness) and calibration writes exceed the old
			// 18B cap, and the chopped frames were why those settings never landed on the controller.
			uint8_t rl = (len <= pln) ? len : (uint8_t)pln;
			if (len > RELAY_MAXP && Serial.availableForWrite() > 60)
				Serial.printf(
					"# RELAY TRUNC cmd=%02X len=%u>%u\n",
					cmd, len, (unsigned)RELAY_MAXP);
			relayEnqueue(cmd, pl, rl);

			// track from RELAYED frames only (see the OUTPUT path)
			if (haptic82)
				haptic82HostReport(pl, len);
		}
	}
	if (Serial.availableForWrite() > 80) {
		Serial.printf("# SET if%d rid=%02X cmd=%02X len=%u:", slot, rid,
			      cmd, len);
		for (uint16_t i = 0; i < n && i < 14; i++)
			Serial.printf(" %02X", b[i]);
		Serial.println();
	}
	memset(S.resp, 0, sizeof S.resp);
	S.resp_len = 0;
	switch (cmd) {
	case 0x83:
		S.resp[0] = 0x83;
		S.resp[1] = sizeof ATTR83;
		memcpy(S.resp + 2, ATTR83, sizeof ATTR83);
		S.resp_len = 63;
		break;
	case 0xAE: {
		uint8_t idx = pln > 0 ? pl[0] : 1;
		const char *s = (idx == 0) ? g_board :
				(idx == 1) ? g_unit :
					     "NA";
		S.resp[0] = 0xAE;
		S.resp[1] = 0x14;
		S.resp[2] = idx;
		memset(S.resp + 3, 0, 60);
		memcpy(S.resp + 3, s, strlen(s));
		S.resp_len = 63;
		break;
	}

	// connection/version state per slot: value 0x02 = controller connected, 0x01 = not
	case 0xB4:
		// SDL Triton polls this on init; treat like Steam contact so we forward 0x45
		hostStampAlive();
		S.resp[0] = 0xB4;
		S.resp[1] = 0x01;
		S.resp[2] = (slot == g_connSlot && !g_xbox &&
			     (millis() - g_connReplyMs < 500)) ?
				    0x02 :
				    0x01;
		S.resp_len = 63;
		break;
	case 0xAD:
		g_pairing = (pln > 0 && pl[0] != 0);
		Serial.printf("# pairing %s\n", g_pairing ? "ON" : "off");
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
			Serial.printf("# slot %d %s\n", slot,
				      recEmpty(pl) ? "cleared" : "bonded");
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

	// host reading feature data => gamepad consumer active (Steam/SDL)
	if (g_usbMode == MODE_STEAM)
		hostStampAlive();
	Slot &S = g_slot[slot];
	uint16_t n = S.resp_len ? S.resp_len : 63;
	if (n > reqlen)
		n = reqlen;
	memcpy(buf, S.resp, n);
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
// USB-side decision -- changes nothing about the RF poll or the host->controller relay.
void SteamPuckController::onReport45(const uint8_t *rep, bool fresh,
				     uint8_t bodyTlen)
{
	// Host asleep -> forward NOTHING. While suspended, every sendReport attempt can translate into a host wake,
	// making the PC wake on any controller movement. Waking is an explicit gesture only (Steam-button short
	// press / controller connect), handled in rf_link.cpp via the device-level USB resume signal.
	if (USBDevice.suspended())
		return;
	// Just woke? Hold off forwarding so the wake gesture's residual controller input doesn't click/type into
	// the freshly-woken desktop.
	if (g_resumeMs && millis() - g_resumeMs < POST_RESUME_MUTE_MS)
		return;
	if (g_connSlot < 0 || g_connSlot >= NSLOT)
		return;
	if (lizardActive()) {
		rfLizard(rep, &hid[g_connSlot], &hid[g_connSlot], 0x40, 0x41);
	} else {
		uint8_t blen = bodyTlen - 1;
		if (blen > 45)
			blen = 45; // body after the 0x45 id byte
		// forward the puck's raw pad coords untouched (Steam does its own interpolation/smoothing). Forward
		// only FRESH reports: the real puck dedupes, so stale repeats make Steam's velocity/smoothing
		// stair-step. g_fwdNewOnly toggles for A/B.
		if ((fresh || !g_fwdNewOnly) && g_slot[g_connSlot].used &&
		    hid[g_connSlot].ready())
			hid[g_connSlot].sendReport(
				0x45, rep + 1,
				blen); // Steam/SDL Triton: input report 0x45
	}
}

// Forward the controller's NON-input status reports (0x43 power/battery, 0x44) to Steam verbatim -- the real
// puck does this and it's how Steam reads battery. Same host-asleep / post-resume gating as 0x45; no lizard
// path (status reports aren't input, so they forward regardless of the lizard decision).
void SteamPuckController::onAuxReport(uint8_t rid, const uint8_t *data,
				      uint8_t n)
{
	if (USBDevice.suspended())
		return;
	if (g_resumeMs && millis() - g_resumeMs < POST_RESUME_MUTE_MS)
		return;
	if (g_connSlot < 0 || g_connSlot >= NSLOT)
		return;
	if (g_slot[g_connSlot].used && hid[g_connSlot].ready())
		hid[g_connSlot].sendReport(rid, data, n);
}

// wake nudge: a bare USB resume signal is NOT enough to wake some hosts (Windows in particular) -- they only
// wake when actual mouse/keyboard input follows. So on a deliberate wake gesture we play a HARMLESS mouse
// JIGGLE (move a few px right, then back -- NET ZERO cursor, NO button): real mouse activity wakes the host
// but clicks/activates nothing (an open Start menu stays open). Queued by wakeEvent() (rf_link, on a Steam
// short press / controller connect while suspended); delivered once the suspended bus has resumed.
static uint8_t g_nudgeStep = 0; // 0=idle; 1=jiggle+, 2=jiggle-
static unsigned long g_nudgeMs = 0;
#define NUDGE_JIGGLE_PX 10
void SteamPuckController::wakeEvent()
{
	g_nudgeStep = 1;
	g_nudgeMs = millis();
}
static void wakeNudgeTask()
{
	if (!g_nudgeStep)
		return;
	if (millis() - g_nudgeMs > 5000) {
		g_nudgeStep = 0;
		return;
	} // bus never resumed -> drop the nudge
	if (USBDevice.suspended())
		return; // wait for resume; reports can't cross a suspended bus
	if (g_connSlot < 0 || g_connSlot >= NSLOT || !hid[g_connSlot].ready())
		return;
	static unsigned long stepMs = 0;
	if (millis() - stepMs < 15)
		return; // pace the edges
	stepMs = millis();
	hid_mouse_report_t m;
	m.buttons = 0;
	m.x = (g_nudgeStep == 1) ? NUDGE_JIGGLE_PX : -NUDGE_JIGGLE_PX;
	m.y = 0;
	m.wheel = 0;
	m.pan = 0;
	hid[g_connSlot].sendReport(0x40, &m,
				   sizeof m); // jiggle right, then back
	g_nudgeStep = (g_nudgeStep >= 2) ? 0 : (uint8_t)(g_nudgeStep + 1);
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
	static bool usbConn = false;
	static unsigned long last79 = 0, last7B = 0, connEdgeMs = 0;
	bool conn = (g_connSlot >= 0 && millis() - g_connReplyMs < 300);
	if (g_connSlot >= 0 && g_connSlot < NSLOT && hid[g_connSlot].ready()) {
		// 0x79 connection state: on edge, then repeated every 750ms ONLY until Steam reacts (its first OUTPUT/
		// settings write after the edge -- g_steamAliveMs). The real puck sends 0x79 ONCE, edge-triggered; an
		// unconditional forever-resend re-triggers Steam's connect handling (connect chime) every 750ms before
		// Steam consumes 0x45 -> a loop of connect-time haptic buzzes. Resending until acked still covers
		// "Steam missed the edge".
		bool steamAcked = g_steamAliveMs &&
				  (int32_t)(g_steamAliveMs - connEdgeMs) >= 0;
		if (conn != usbConn ||
		    (conn && !steamAcked && millis() - last79 >= 750)) {
			if (conn && !usbConn)
				connEdgeMs = millis();
			uint8_t st = conn ? 0x02 : 0x01;
			hid[g_connSlot].sendReport(0x79, &st, 1);
			usbConn = conn;
			last79 = millis();
		} else if (conn && millis() - last7B >= 2000) {
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
			if (g_linkRssi) {
				int mag = (int)g_linkRssi - RSSI_DBM_OFFSET;
				if (mag < 25)
					mag = 25;
				else if (mag > 95)
					mag = 95;
				s7b[8] = (uint8_t)(0u - (uint8_t)mag);
			}
			hid[g_connSlot].sendReport(0x7B, s7b, 12);
			last7B = millis();
		}
	} else if (!conn)
		usbConn = false;
}
