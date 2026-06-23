#include "mode_ps5.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_mount.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

Ps5Controller g_ps5Ctl;

static const uint8_t PS5_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31,
	0x09, 0x32, 0x09, 0x35, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xFF,
	0x00, 0x75, 0x08, 0x95, 0x06, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x20,
	0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07,
	0x35, 0x00, 0x46, 0x3B, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81,
	0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0F, 0x15, 0x00, 0x25,
	0x01, 0x75, 0x01, 0x95, 0x0F, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x21,
	0x95, 0x0D, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x22, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x34, 0x81, 0x02, 0x85, 0x02, 0x09, 0x23,
	0x95, 0x2F, 0x91, 0x02, 0x85, 0x05, 0x09, 0x33, 0x95, 0x28, 0xB1, 0x02,
	0x85, 0x08, 0x09, 0x34, 0x95, 0x2F, 0xB1, 0x02, 0x85, 0x09, 0x09, 0x24,
	0x95, 0x13, 0xB1, 0x02, 0x85, 0x0A, 0x09, 0x25, 0x95, 0x1A, 0xB1, 0x02,
	0x85, 0x20, 0x09, 0x26, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x21, 0x09, 0x27,
	0x95, 0x04, 0xB1, 0x02, 0x85, 0x22, 0x09, 0x40, 0x95, 0x3F, 0xB1, 0x02,
	0x85, 0x80, 0x09, 0x28, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x81, 0x09, 0x29,
	0x95, 0x3F, 0xB1, 0x02, 0x85, 0x82, 0x09, 0x2A, 0x95, 0x09, 0xB1, 0x02,
	0x85, 0x83, 0x09, 0x2B, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0x84, 0x09, 0x2C,
	0x95, 0x3F, 0xB1, 0x02, 0x85, 0x85, 0x09, 0x2D, 0x95, 0x02, 0xB1, 0x02,
	0x85, 0xA0, 0x09, 0x2E, 0x95, 0x01, 0xB1, 0x02, 0x85, 0xE0, 0x09, 0x2F,
	0x95, 0x3F, 0xB1, 0x02, 0x85, 0xF0, 0x09, 0x30, 0x95, 0x3F, 0xB1, 0x02,
	0x85, 0xF1, 0x09, 0x31, 0x95, 0x3F, 0xB1, 0x02, 0x85, 0xF2, 0x09, 0x32,
	0x95, 0x0F, 0xB1, 0x02, 0x85, 0xF4, 0x09, 0x35, 0x95, 0x3F, 0xB1, 0x02,
	0x85, 0xF5, 0x09, 0x36, 0x95, 0x03, 0xB1, 0x02, 0xC0
};
#define PS5_TOUCH_H 1080
#define PS5_STATUS_USB 0x1A // charging + level 10 (~100%)
static unsigned long g_ps5LastMs[NSLOT] = { 0 };
// NSLOT DualSense HID instances, one per bond slot. The host enumerates each as a separate DualSense
// (hid-playstation on Linux/SteamOS, SDL on Windows).
static Adafruit_USBD_HID g_ps5[NSLOT];

// Per-slot MAC base: 4 distinct NICs. OUI 0x001BDC is Sony's; vary the last byte per slot.
static const uint8_t PS5_MAC_BASE[5] = { 0x00, 0x1B, 0xDC, 0x4F, 0x55 };
static uint8_t g_ps5Mac[NSLOT][6];
static bool g_ps5MacInit = false;
static void initPs5Macs()
{
	if (g_ps5MacInit)
		return;
	for (int s = 0; s < NSLOT; s++) {
		memcpy(g_ps5Mac[s], PS5_MAC_BASE, 5);
		g_ps5Mac[s][5] = (uint8_t)(0x60 + s); // 0x60, 0x61, 0x62, 0x63
	}
	g_ps5MacInit = true;
}

// GET_FEATURE handler. Per-slot dispatch via per-instance callback. Sizes per drivers/hid/hid-playstation.c:
// 0x05=41, 0x09=20, 0x20=64. TinyUSB writes the report id itself and hands us the buffer PAST it, so we
// fill only the PAYLOAD and return size-1.
static uint16_t ps5GetCommon(uint8_t slot, uint8_t rid, hid_report_type_t type,
			     uint8_t *buf, uint16_t reqlen)
{
	(void)slot;
	if (type != HID_REPORT_TYPE_FEATURE || !buf || reqlen == 0)
		return 0;
	memset(buf, 0, reqlen);
	switch (rid) {
	// capabilities: identify as DualSense-capable (SDL-only probe; hid-playstation never reads 0x03)
	case 0x03: {
		if (reqlen < 47)
			return 0;
		buf[0] = 0x00;
		buf[1] = 0x28;
		buf[2] = 0x01;
		buf[3] = 0x00;
		buf[4] = 0x0E; // sensors + lightbar + vibration capability bits
		return 47;
	}
	case 0x05: // motion calibration (41 incl id)
		if (reqlen < 40)
			return 0;
		psNeutralCalib(buf);
		return 40;
	case 0x09: // pairing info / MAC (20 incl id)
		if (reqlen < 19)
			return 0;
		// MAC at kernel buf[1..6] = payload[0..5]
		memcpy(buf, g_ps5Mac[slot], 6);
		return 19;
	case 0x20: // firmware info (64 incl id)
		if (reqlen < 63)
			return 0;
		buf[23] = 0x01; // hw_version (le32 @ kernel buf[24]) non-zero
		buf[27] = 0x01; // fw_version (le32 @ kernel buf[28]) non-zero
		return 63;
	default:
		return 0;
	}
}
static void ps5SetCommon(uint8_t slot, uint8_t rid, hid_report_type_t type,
			 uint8_t const *b, uint16_t n)
{
	if (type != HID_REPORT_TYPE_OUTPUT || n < 1)
		return;
	uint8_t id;
	const uint8_t *p;
	uint16_t pn;
	if (rid == 0) {
		id = b[0];
		p = b + 1;
		pn = (uint16_t)(n - 1);
	} else if (rid == 0x02 && b[0] == 0x02 && n >= 5) {
		id = rid;
		p = b + 1;
		pn = (uint16_t)(n - 1);
	} // some paths leave report id in b
	else {
		id = rid;
		p = b;
		pn = n;
	}
	if (id != 0x02 || pn < 4)
		return;
	// `slot` here is the USB slot the report arrived on -> route rumble to the bond slot it's mapped to.
	int bond = (slot < NSLOT) ? g_usbToBond[slot] : -1;
	if (bond < 0)
		return;
	hapticSteamRumble((uint16_t)p[3] * 257u, (uint16_t)p[2] * 257u,
			  (uint8_t)bond);
	// DualSense: left=low, right=high
}
#define PS5CB(N)                                                               \
	static uint16_t ps5Get##N(uint8_t r, hid_report_type_t t, uint8_t *bf, \
				  uint16_t rl)                                 \
	{                                                                      \
		return ps5GetCommon(N, r, t, bf, rl);                          \
	}                                                                      \
	static void ps5Set##N(uint8_t r, hid_report_type_t t,                  \
			      uint8_t const *b, uint16_t n)                    \
	{                                                                      \
		ps5SetCommon(N, r, t, b, n);                                   \
	}
// clang-format off
PS5CB(0)
PS5CB(1)
PS5CB(2)
PS5CB(3)
// clang-format on
typedef uint16_t (*ps5_getcb_t)(uint8_t, hid_report_type_t, uint8_t *,
				uint16_t);
typedef void (*ps5_setcb_t)(uint8_t, hid_report_type_t, uint8_t const *,
			    uint16_t);
static ps5_getcb_t const PS5_GETCB[NSLOT] = { ps5Get0, ps5Get1, ps5Get2,
					      ps5Get3 };
static ps5_setcb_t const PS5_SETCB[NSLOT] = { ps5Set0, ps5Set1, ps5Set2,
					      ps5Set3 };

// usbSlot drives the per-HID sequence counter; bond is the controller whose decoded input feeds the report.
static void ps5Build(uint8_t usbSlot, uint8_t slot, uint8_t out[63])
{
	uint32_t b = psButtonsFromSteam(g_in[slot].buttons);
	bool lTouch = (b & TB_LPADT) || (b & TB_LPADC),
	     rTouch = (b & TB_RPADT) || (b & TB_RPADC);
	memset(out, 0, 63);
	out[0] = swStick(g_in[slot].lx, false);
	out[1] = swStick(g_in[slot].ly, true);
	out[2] = swStick(g_in[slot].rx, false);
	out[3] = swStick(g_in[slot].ry, true);
	out[4] = g_in[slot].lt;
	out[5] = g_in[slot].rt;
	static uint8_t seq[NSLOT] = { 0 };
	out[6] = seq[usbSlot]++;
	out[7] = psHatNibble(b) | psFaceNibble(b);
	out[8] = psShouldersByte(b, g_in[slot].lt, g_in[slot].rt);
	out[9] = ((b & TB_STEAM) ? 0x01 : 0) |
		 ((b & TB_TOUCH || b & TB_LPADC || b & TB_RPADC) ? 0x02 : 0) |
		 ((b & TB_MUTE) ? 0x04 : 0);
	out[15] = g_in[slot].gx & 0xFF;
	out[16] = g_in[slot].gx >> 8;
	out[17] = g_in[slot].gz & 0xFF;
	out[18] = g_in[slot].gz >> 8;
	out[19] = (-g_in[slot].gy) & 0xFF;
	out[20] = (-g_in[slot].gy) >> 8;
	out[21] = g_in[slot].ax & 0xFF;
	out[22] = g_in[slot].ax >> 8;
	out[23] = g_in[slot].ay & 0xFF;
	out[24] = g_in[slot].ay >> 8;
	out[25] = g_in[slot].az & 0xFF;
	out[26] = g_in[slot].az >> 8;
	uint16_t lx, ly, rx, ry;
	steamPadsToTouch(b, PS5_TOUCH_H, g_in[slot].lpx, g_in[slot].lpy,
			 g_in[slot].rpx, g_in[slot].rpy, &lx, &ly, &rx, &ry);
	touchPackPads(out + 32, lTouch, rTouch, lx, ly, rx, ry);
	out[52] = PS5_STATUS_USB;
}

// Dynamic-mount mode: begin() is unused (setup() calls beginPool()+usbReenumerate instead).
void Ps5Controller::begin()
{
}
// HID budget: clean PS modes have no wake mouse (CFG_TUD_HID slots); normal PS5 keeps the wake mouse (1 HID).
uint8_t Ps5Controller::maxSlots() const
{
	uint8_t cap = modeIsCleanPS(g_usbMode) ? (uint8_t)CFG_TUD_HID :
						 (uint8_t)(CFG_TUD_HID - 1);
	return cap < NSLOT ? cap : (uint8_t)NSLOT;
}
void Ps5Controller::usbIdentity()
{
	USBDevice.setID(0x054C, 0x0CE6);
	USBDevice.setDeviceVersion(0x0110);
	USBDevice.setManufacturerDescriptor("Sony Interactive Entertainment");
	USBDevice.setProductDescriptor("DualSense Wireless Controller");
}
// One-time: create the DualSense HID pool and lock instance indices (wake mouse, if any, was begun first).
void Ps5Controller::beginPool()
{
	initPs5Macs();
	uint8_t pool = maxSlots();
	for (uint8_t s = 0; s < pool; s++) {
		g_ps5[s].enableOutEndpoint(true);
		g_ps5[s].setReportCallback(PS5_GETCB[s], PS5_SETCB[s]);
		g_ps5[s].setReportDescriptor(PS5_HID_DESC, sizeof PS5_HID_DESC);
		g_ps5[s].setPollInterval(1);
		g_ps5[s].begin();
	}
}
void Ps5Controller::mountSlots(uint8_t k)
{
	for (uint8_t u = 0; u < k; u++)
		USBDevice.addInterface(g_ps5[u]);
}
void Ps5Controller::task()
{
	for (uint8_t u = 0; u < g_usbMountCount; u++) {
		if (!g_ps5[u].ready())
			continue;
		if (millis() - g_ps5LastMs[u] < USB_STREAM_MS)
			continue;
		int bond = g_usbToBond[u];
		if (bond < 0)
			continue;
		g_ps5LastMs[u] = millis();
		uint8_t p[63];
		ps5Build(u, (uint8_t)bond, p);
		g_ps5[u].sendReport(0x01, p, sizeof p);
	}
}
