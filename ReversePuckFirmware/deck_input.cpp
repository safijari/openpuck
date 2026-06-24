#include "deck_input.h"
#include "triton.h"
#include "ctrl_bonds.h"
#include "ctrl_link.h"
#include "radio.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

static bool g_forwarding = false;
static unsigned long g_lastInputMs = 0;
static unsigned long g_lastStatusMs = 0;

bool deckForwarding()
{
	return g_forwarding;
}
unsigned long deckLastInputMs()
{
	return g_lastInputMs;
}

// ---- little-endian payload readers ----
static inline int16_t rd_s16(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t rd_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}
static inline uint16_t rd_u16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// INPUT payload = buttons u32, lx ly rx ry s16, lt rt u8, lpx lpy rpx rpy s16, lpp rpp u16, IMU 6h.
#define DECK_INPUT_LEN 38
static void applyInput(const uint8_t *p, uint8_t n)
{
	if (n < DECK_INPUT_LEN)
		return;
	g_in.buttons = rd_u32(p + 0);
	g_in.lx = rd_s16(p + 4);
	g_in.ly = rd_s16(p + 6);
	g_in.rx = rd_s16(p + 8);
	g_in.ry = rd_s16(p + 10);
	g_in.lt = p[12];
	g_in.rt = p[13];
	g_in.lpx = rd_s16(p + 14);
	g_in.lpy = rd_s16(p + 16);
	g_in.rpx = rd_s16(p + 18);
	g_in.rpy = rd_s16(p + 20);
	g_in.lpp = rd_u16(p + 22);
	g_in.rpp = rd_u16(p + 24);
	g_in.ax = rd_s16(p + 26);
	g_in.ay = rd_s16(p + 28);
	g_in.az = rd_s16(p + 30);
	g_in.gx = rd_s16(p + 32);
	g_in.gy = rd_s16(p + 34);
	g_in.gz = rd_s16(p + 36);
	g_lastInputMs = millis();
}

static void handleFrame(uint8_t type, const uint8_t *p, uint8_t n)
{
	switch (type) {
	case DECK_T_INPUT:
		applyInput(p, n);
		break;
	case DECK_T_CONTROL:
		if (n >= 2 && p[0] == 0x01)
			g_forwarding = (p[1] != 0);
		else if (n >= 2 &&
			 p[0] == 0x02) // clear-bond [slot] (GUI "remove")
			ctrlBondClear(p[1]);
		break;
	case DECK_T_REQ_STATUS:
		g_lastStatusMs = 0; // force an immediate status emit
		break;
	default:
		break;
	}
}

// ---- RX state machine: resync on the 2-byte sync word ----
void deckInputPoll()
{
	// state: 0 wait sync0, 1 wait sync1, 2 type, 3 len, 4 payload, 5 sum
	static uint8_t st = 0;
	static uint8_t type = 0, len = 0, idx = 0, sum = 0;
	static uint8_t buf[64];
	while (Serial.available()) {
		uint8_t c = (uint8_t)Serial.read();
		switch (st) {
		case 0:
			if (c == DECK_SYNC0)
				st = 1;
			break;
		case 1:
			st = (c == DECK_SYNC1) ? 2 : (c == DECK_SYNC0 ? 1 : 0);
			break;
		case 2:
			type = c;
			sum = c;
			st = 3;
			break;
		case 3:
			len = c;
			sum += c;
			idx = 0;
			if (len > sizeof buf) { // oversized -> drop, resync
				st = 0;
				break;
			}
			st = (len == 0) ? 5 : 4;
			break;
		case 4:
			buf[idx++] = c;
			sum += c;
			if (idx >= len)
				st = 5;
			break;
		case 5:
			if (c == sum)
				handleFrame(type, buf, len);
			st = 0;
			break;
		}
	}
}

// ---- frame TX ----
static void sendFrame(uint8_t type, const uint8_t *p, uint8_t n)
{
	// need room for the whole frame or skip (never block the RF loop on CDC backpressure)
	if ((int)Serial.availableForWrite() < n + 5)
		return;
	uint8_t hdr[4] = { DECK_SYNC0, DECK_SYNC1, type, n };
	uint8_t sum = (uint8_t)(type + n);
	for (uint8_t i = 0; i < n; i++)
		sum += p[i];
	Serial.write(hdr, 4);
	if (n)
		Serial.write(p, n);
	Serial.write(&sum, 1);
}

void deckText(const char *s)
{
	size_t n = strlen(s);
	if (n > 60)
		n = 60;
	sendFrame(DECK_T_TEXT, (const uint8_t *)s, (uint8_t)n);
}

// Latest relayed haptic, STAGED by deckForwardHaptic (called from the RF poll path) and flushed over CDC
// by deckHapticTask() at loop level. Staging-then-flushing (a) keeps the CDC write OUT of the RF receive
// path, and (b) COALESCES a burst to the newest state -- Steam streams rumble at the poll rate, and
// replaying every frame one-behind is what caused the lag; newest-wins removes the backlog.
static uint8_t g_hapPayload[8];
static volatile bool g_hapPending = false;
static unsigned long g_lastHapMs = 0;
// cap CDC haptic frames to ~60 Hz: enough for smooth rumble, but no flood of the CDC link / Deck USB.
#define HAPTIC_MIN_MS 16u

void deckForwardHaptic(uint8_t rid, const uint8_t *d, uint8_t n)
{
	// Only while forwarding -- otherwise the Deck isn't grabbed and there's nothing to buzz.
	if (!g_forwarding)
		return;
	uint16_t intensity = 0, left = 0, right = 0;
	uint8_t lgain = 0, rgain = 0;
	if (rid == 0x80 && n >= 9) {
		// SDL Triton rumble report 0x80: [type][intensity u16][left u16][lgain][right u16][rgain].
		// type 0 = off/zero report (everything stays 0).
		if (d[0] != 0) {
			intensity = (uint16_t)(d[1] | (d[2] << 8));
			left = (uint16_t)(d[3] | (d[4] << 8));
			lgain = d[5];
			right = (uint16_t)(d[6] | (d[7] << 8));
			rgain = d[8];
		}
	} else if (rid == 0x82 || rid == 0x8F) {
		// discrete haptic click/pulse (data ~ [01 01 F7]; trailing byte F7=on, 00=off). The Deck has no
		// per-pad clicker, so synthesize a short mid-strength buzz on both motors.
		bool on = (n >= 3) ? (d[2] != 0) : (n >= 1 && d[0] != 0);
		if (on) {
			intensity = 0x6000;
			left = right = 0x6000;
		}
	} else {
		return; // not a haptic the Deck can render
	}
	// stage newest (coalesce). The actual sendFrame happens in deckHapticTask, off the RF path.
	g_hapPayload[0] = (uint8_t)(intensity & 0xFF);
	g_hapPayload[1] = (uint8_t)(intensity >> 8);
	g_hapPayload[2] = (uint8_t)(left & 0xFF);
	g_hapPayload[3] = (uint8_t)(left >> 8);
	g_hapPayload[4] = (uint8_t)(right & 0xFF);
	g_hapPayload[5] = (uint8_t)(right >> 8);
	g_hapPayload[6] = lgain;
	g_hapPayload[7] = rgain;
	g_hapPending = true;
}

void deckHapticTask()
{
	if (!g_hapPending)
		return;
	if (millis() - g_lastHapMs < HAPTIC_MIN_MS)
		return; // rate-limit the newest staged haptic to ~60 Hz
	g_hapPending = false;
	g_lastHapMs = millis();
	sendFrame(DECK_T_HAPTIC, g_hapPayload, sizeof g_hapPayload);
}

// STATUS payload: [flags][linkSlot][sessCh][nbondMax][count] + per USED bond
//   [slotIdx][alive][puuid4][iuuid4][serial16] (26 B). Only used bonds are sent, so the typical 1-2-puck
// frame stays small; the slot index lets the GUI target a bond for removal.
void deckStatusTask()
{
	if (millis() - g_lastStatusMs < 250)
		return;
	g_lastStatusMs = millis();

	uint8_t p[5 + NBOND * 26];
	bool linkUp = ctrlLinkUp();
	p[0] = (uint8_t)((g_forwarding ? 1 : 0) | (linkUp ? 2 : 0));
	p[1] = (g_linkSlot >= 0) ? (uint8_t)g_linkSlot : 0xFF;
	p[2] = g_sessCh;
	p[3] = NBOND;
	uint8_t count = 0;
	uint8_t *q = p + 5;
	for (int i = 0; i < NBOND; i++) {
		CtrlBond &b = g_bond[i];
		if (!b.used)
			continue;
		q[0] = (uint8_t)i; // firmware slot index (for GUI removal)
		q[1] = (i == g_linkSlot && linkUp) ? 1 : 0;
		memcpy(q + 2, b.rec + 0, 4); // proteus_uuid
		memcpy(q + 6, b.rec + 4, 4); // ibex_uuid
		memcpy(q + 10, b.rec + 8, 16); // puck serial
		q += 26;
		count++;
	}
	p[4] = count;
	sendFrame(DECK_T_STATUS, p, (uint8_t)(5 + count * 26));
}
