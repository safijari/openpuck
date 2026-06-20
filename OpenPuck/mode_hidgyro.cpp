#include "mode_hidgyro.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

HidGyroController g_hidGyroCtl;

static const uint8_t GYRO_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09,
	0x31, 0x09, 0x32, 0x09, 0x35, 0x15, 0x00, 0x25, 0x07, 0x75, 0x04,
	0x95, 0x01, 0x81, 0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29,
	0x0E, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x0E, 0x81, 0x02,
	0x06, 0x00, 0xFF, 0x09, 0x20, 0x75, 0x06, 0x95, 0x01, 0x15, 0x00,
	0x25, 0x7F, 0x81, 0x02, 0x05, 0x01, 0x09, 0x33, 0x09, 0x34, 0x15,
	0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02, 0x06,
	0x00, 0xFF, 0x09, 0x21, 0x95, 0x36, 0x81, 0x02, 0x85, 0x05, 0x09,
	0x22, 0x95, 0x1F, 0x91, 0x02, 0x85, 0x04, 0x09, 0x23, 0x95, 0x24,
	0xB1, 0x02, 0x85, 0x02, 0x09, 0x24, 0x95, 0x24, 0xB1, 0x02, 0xC0
};
#define DS4_TOUCH_H 942
#define DS4_STATUS_USB 0x1B // cable + level 11 (full)
static unsigned long g_gyroLastMs[NSLOT] = { 0 };
// NSLOT HID instances: 4 DS4 gamepads on the wire, one per bond slot. The host enumerates each as a
// separate input device (hid-playstation, hid-sony, SDL, Steam Input).
static Adafruit_USBD_HID g_hidGyro[NSLOT];

// Per-slot MAC base: 4 distinct NICs so the host sees 4 different devices (real DS4s have unique MACs).
// OUI 0x001BDC is Sony's; vary the last byte per slot.
static const uint8_t DS4_MAC_BASE[5] = { 0x00, 0x1B, 0xDC, 0x4F, 0x55 };
static uint8_t g_ds4Mac[NSLOT][6];
static bool g_ds4MacInit = false;
static void initDs4Macs()
{
	if (g_ds4MacInit)
		return;
	for (int s = 0; s < NSLOT; s++) {
		memcpy(g_ds4Mac[s], DS4_MAC_BASE, 5);
		g_ds4Mac[s][5] = (uint8_t)(0x50 + s); // 0x50, 0x51, 0x52, 0x53
	}
	g_ds4MacInit = true;
}

// GET_FEATURE handler. Per-slot dispatch via per-instance callback (ps5Get##N passes the slot index).
// Sizes per drivers/hid/hid-playstation.c: 0x02=37, 0x12=16, 0xA3=49; legacy hid-sony MAC 0x81=7.
static uint16_t hidGyroGetCommon(uint8_t slot, uint8_t rid,
				 hid_report_type_t type, uint8_t *buf,
				 uint16_t reqlen)
{
	(void)slot;
	if (type != HID_REPORT_TYPE_FEATURE || !buf || reqlen == 0)
		return 0;
	memset(buf, 0, reqlen);
	switch (rid) {
	case 0x02: // motion calibration (37 incl id)
		if (reqlen < 36)
			return 0;
		psNeutralCalib(buf);
		return 36;
	case 0x12: // pairing info / MAC, hid-playstation (16 incl id)
		if (reqlen < 15)
			return 0;
		memcpy(buf, g_ds4Mac[slot], 6);
		return 15;
	case 0x81: // MAC, legacy hid-sony USB (7 incl id)
		if (reqlen < 6)
			return 0;
		memcpy(buf, g_ds4Mac[slot], 6);
		return 6;
	case 0xA3: // firmware / hardware info (49 incl id)
		if (reqlen < 48)
			return 0;
		// non-zero version (contents not validated over USB)
		buf[0] = 0x01;
		return 48;
	default:
		return 0;
	}
}
static void hidGyroSetCommon(uint8_t slot, uint8_t rid, hid_report_type_t type,
			     uint8_t const *b, uint16_t n)
{
	(void)slot;
	if (type != HID_REPORT_TYPE_OUTPUT || n < 1)
		return;
	uint8_t id;
	const uint8_t *p;
	uint16_t pn;
	if (rid == 0) {
		id = b[0];
		p = b + 1;
		pn = (uint16_t)(n - 1);
	} else {
		id = rid;
		p = b;
		pn = n;
	}
	// DS4 USB effects: report 0x05, magic at byte 0, effects block starts at byte 3.
	if (id != 0x05 || pn < 5)
		return;
	hapticSteamRumble((uint16_t)p[4] * 257u, (uint16_t)p[3] * 257u,
			  slot); // DS4: left=low, right=high
}
// Per-slot callback trampolines -- the Adafruit HID class has ONE _get/_set pair shared across instances,
// and the lib's tud_hid_*_cb doesn't pass the interface index to the user callback. Generate a
// per-instance trampoline that closes over the slot index, matching the pattern in puck_hid.cpp.
#define HIDGYROCB(N)                                                  \
	static uint16_t hidGyroGet##N(uint8_t r, hid_report_type_t t, \
				      uint8_t *bf, uint16_t rl)       \
	{                                                             \
		return hidGyroGetCommon(N, r, t, bf, rl);             \
	}                                                             \
	static void hidGyroSet##N(uint8_t r, hid_report_type_t t,     \
				  uint8_t const *b, uint16_t n)       \
	{                                                             \
		hidGyroSetCommon(N, r, t, b, n);                      \
	}
// clang-format off
HIDGYROCB(0)
HIDGYROCB(1)
HIDGYROCB(2)
HIDGYROCB(3)
// clang-format on
typedef uint16_t (*ds4_getcb_t)(uint8_t, hid_report_type_t, uint8_t *,
				uint16_t);
typedef void (*ds4_setcb_t)(uint8_t, hid_report_type_t, uint8_t const *,
			    uint16_t);
static ds4_getcb_t const DS4_GETCB[NSLOT] = { hidGyroGet0, hidGyroGet1,
					      hidGyroGet2, hidGyroGet3 };
static ds4_setcb_t const DS4_SETCB[NSLOT] = { hidGyroSet0, hidGyroSet1,
					      hidGyroSet2, hidGyroSet3 };

static void hidGyroBuild(uint8_t slot, uint8_t out[63])
{
	uint32_t b = psButtonsFromSteam(g_in[slot].buttons);
	bool lTouch = (b & TB_LPADT) || (b & TB_LPADC),
	     rTouch = (b & TB_RPADT) || (b & TB_RPADC);
	memset(out, 0, 63);
	out[0] = swStick(g_in[slot].lx, false);
	out[1] = swStick(g_in[slot].ly, true);
	out[2] = swStick(g_in[slot].rx, false);
	out[3] = swStick(g_in[slot].ry, true);
	out[4] = psHatNibble(b) | psFaceNibble(b);
	out[5] = psShouldersByte(b, g_in[slot].lt, g_in[slot].rt);
	static uint8_t ctr[NSLOT] = { 0 };
	out[6] = ((ctr[slot]++ & 0x0F) << 4) |
		 ((b & TB_TOUCH || b & TB_LPADC || b & TB_RPADC) ? 0x02 : 0) |
		 ((b & TB_STEAM) ? 0x01 : 0);
	out[7] = g_in[slot].lt;
	out[8] = g_in[slot].rt;
	out[12] = g_in[slot].gx & 0xFF;
	out[13] = g_in[slot].gx >> 8;
	out[14] = g_in[slot].gz & 0xFF;
	out[15] = g_in[slot].gz >> 8;
	out[16] = (-g_in[slot].gy) & 0xFF;
	out[17] = (-g_in[slot].gy) >> 8;
	out[18] = g_in[slot].ax & 0xFF;
	out[19] = g_in[slot].ax >> 8;
	out[20] = g_in[slot].ay & 0xFF;
	out[21] = g_in[slot].ay >> 8;
	out[22] = g_in[slot].az & 0xFF;
	out[23] = g_in[slot].az >> 8;
	out[29] = DS4_STATUS_USB;
	if (lTouch || rTouch) {
		uint16_t lx, ly, rx, ry;
		steamPadsToTouch(b, DS4_TOUCH_H, g_in[slot].lpx, g_in[slot].lpy,
				 g_in[slot].rpx, g_in[slot].rpy, &lx, &ly, &rx,
				 &ry);
		static uint8_t tstamp[NSLOT] = { 0 };
		out[32] = 1;
		out[33] = tstamp[slot]++;
		touchPackPads(out + 34, lTouch, rTouch, lx, ly, rx, ry);
	} else {
		out[32] = 0;
		touchPackPads(
			out + 34, false, false, 0, 0, 0,
			0); // contact 0x80 -- memset(0) reads as touch @0,0
	}
}

void HidGyroController::begin()
{
	USBDevice.setID(0x054C, 0x05C4);

	int n = bondedSlotCount();
	USBDevice.setDeviceVersion(
		(uint16_t)(0x0120 + (uint16_t)(n > 0 ? n - 1 : 0)));
	USBDevice.setManufacturerDescriptor("Sony Computer Entertainment");
	USBDevice.setProductDescriptor("Wireless Controller");
	initDs4Macs();
	for (int s = 0; s < NSLOT; s++) {
		if (n > 0 && !g_slot[s].used)
			continue;
		if (n == 0 && s > 0)
			break;
		g_hidGyro[s].enableOutEndpoint(true);
		g_hidGyro[s].setReportCallback(DS4_GETCB[s], DS4_SETCB[s]);
		g_hidGyro[s].setReportDescriptor(GYRO_HID_DESC,
						 sizeof GYRO_HID_DESC);
		g_hidGyro[s].setPollInterval(1);
		g_hidGyro[s].begin();
	}
}
void HidGyroController::task()
{
	for (int s = 0; s < NSLOT; s++) {
		if (!g_hidGyro[s].ready())
			continue;
		if (millis() - g_gyroLastMs[s] < USB_STREAM_MS)
			continue;
		g_gyroLastMs[s] = millis();
		uint8_t p[63];
		hidGyroBuild((uint8_t)s, p);
		g_hidGyro[s].sendReport(0x01, p, sizeof p);
	}
}
