// mode_switch2_pro.cpp -- Nintendo Switch 2 Pro Controller (057E:2069).
//
// The USB descriptors are byte-for-byte captures of a physical controller. The
// native 0x09 motion packing is still undocumented, so this implementation
// deliberately advertises zero 0x09 motion length and carries decoded Steam IMU
// samples only in the documented common 0x05 report.
#include "mode_switch2_pro.h"
#include "bonds.h"
#include "config.h"
#include "gamepad_util.h"
#include "haptics.h"
#include "rf_link.h"
#include "triton.h"
#include "usb_app_drivers.h"
#include "usb_mount.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

extern "C" {
#include "class/hid/hid_device.h"
#include "device/usbd_pvt.h"
}

Switch2ProController g_switch2Pro;

static const uint8_t SWITCH2_PRO_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x05, 0x05, 0xff, 0x09,
	0x01, 0x15, 0x00, 0x26, 0xff, 0x00, 0x95, 0x3f, 0x75, 0x08, 0x81,
	0x02, 0x85, 0x09, 0x09, 0x01, 0x95, 0x02, 0x81, 0x02, 0x05, 0x09,
	0x19, 0x01, 0x29, 0x15, 0x25, 0x01, 0x95, 0x15, 0x75, 0x01, 0x81,
	0x02, 0x95, 0x01, 0x75, 0x03, 0x81, 0x03, 0x05, 0x01, 0x09, 0x01,
	0xa1, 0x00, 0x09, 0x30, 0x09, 0x31, 0x09, 0x33, 0x09, 0x35, 0x26,
	0xff, 0x0f, 0x95, 0x04, 0x75, 0x0c, 0x81, 0x02, 0xc0, 0x05, 0xff,
	0x09, 0x02, 0x26, 0xff, 0x00, 0x95, 0x34, 0x75, 0x08, 0x91, 0x02,
	0x85, 0x02, 0x09, 0x01, 0x95, 0x3f, 0x91, 0x02, 0xc0,
};
static_assert(sizeof SWITCH2_PRO_HID_DESC == 97,
	      "Switch 2 Pro HID descriptor must remain byte-exact");

// Everything after the 9-byte configuration header. Interface and endpoint
// numbers are fixed because this clean personality is the only mounted USB
// function: IF0 HID, IF1 vendor bulk, IF2-4 Nintendo audio shape.
static const uint8_t SWITCH2_PRO_CFG_BODY[] = {
	0x08, 0x0b, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x09, 0x04, 0x00, 0x00,
	0x02, 0x03, 0x00, 0x00, 0x05, 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
	0x61, 0x00, 0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x04, 0x07, 0x05, 0x01,
	0x03, 0x40, 0x00, 0x04, 0x08, 0x0b, 0x01, 0x01, 0xff, 0x00, 0x00, 0x00,
	0x09, 0x04, 0x01, 0x00, 0x02, 0xff, 0x00, 0x00, 0x06, 0x07, 0x05, 0x02,
	0x02, 0x40, 0x00, 0x00, 0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00, 0x08,
	0x0b, 0x02, 0x03, 0x01, 0x01, 0x00, 0x00, 0x09, 0x04, 0x02, 0x00, 0x00,
	0x01, 0x01, 0x00, 0x00, 0x0a, 0x24, 0x01, 0x00, 0x01, 0x47, 0x00, 0x02,
	0x03, 0x04, 0x0c, 0x24, 0x02, 0x01, 0x01, 0x01, 0x00, 0x02, 0x03, 0x00,
	0x00, 0x00, 0x0a, 0x24, 0x06, 0x02, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00,
	0x09, 0x24, 0x03, 0x03, 0x02, 0x03, 0x00, 0x02, 0x00, 0x0c, 0x24, 0x02,
	0x04, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x09, 0x24, 0x06,
	0x05, 0x04, 0x01, 0x03, 0x00, 0x00, 0x09, 0x24, 0x03, 0x06, 0x01, 0x01,
	0x00, 0x05, 0x00, 0x09, 0x04, 0x03, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,
	0x09, 0x04, 0x03, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00, 0x07, 0x24, 0x01,
	0x01, 0x00, 0x01, 0x00, 0x0b, 0x24, 0x02, 0x01, 0x02, 0x02, 0x10, 0x01,
	0x80, 0xbb, 0x00, 0x07, 0x05, 0x03, 0x0d, 0xc0, 0x00, 0x01, 0x07, 0x25,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x09, 0x04, 0x04, 0x00, 0x00, 0x01, 0x02,
	0x00, 0x00, 0x09, 0x04, 0x04, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00, 0x07,
	0x24, 0x01, 0x06, 0x00, 0x01, 0x00, 0x0b, 0x24, 0x02, 0x01, 0x02, 0x02,
	0x10, 0x01, 0x80, 0xbb, 0x00, 0x07, 0x05, 0x83, 0x0d, 0xc0, 0x00, 0x01,
	0x07, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00,
};
static_assert(sizeof SWITCH2_PRO_CFG_BODY == 259,
	      "Switch 2 Pro configuration body must remain byte-exact");

// Report 0x05 is the only input report that carries motion: per SWITCH2_PRO_HID_DESC, 0x09 is
// 2 vendor bytes + 21 button bits + four 12-bit axes and has no IMU fields at all, while 0x05
// is the 63-byte vendor blob whose tail sw2Build05 fills with accel/gyro. Defaulting to 0x09
// made gyro structurally impossible on any host that never sends the 0x03/0x0a report select
// (buttons still worked, which is what made it look like an IMU problem). An explicit host
// selection is still honoured either way.
static volatile uint8_t g_sw2ActiveReport = 0x05;
static volatile uint8_t g_sw2Features = 0;
static volatile uint8_t g_sw2FeatureMask = 0;
static volatile bool g_sw2InputEnabled = false;
static volatile uint8_t g_sw2Counter8 = 0;
static volatile uint32_t g_sw2Counter32 = 0;
static int8_t g_sw2LastRumbleBond = -1;
static uint16_t g_sw2LastRumbleLeft = 0;
static uint16_t g_sw2LastRumbleRight = 0;
// L4, R4, L5, R5, View, Menu, Steam, QAM -> Switch 2 button codes. Mind the naming trap
// documented in triton.h: TB_VIEW is the START-side button (Switch +) and TB_MENU the
// SELECT-side one (Switch -), so View maps to Plus and Menu to Minus -- reading the names
// literally swaps Start and Select on the host.
static const uint8_t def[8] = { 18, 18, 23, 22, 10, 9, 11, 21 };
// cfgExt[0] versions this table. A puck that already persisted the reversed Plus/Minus
// defaults would otherwise keep them forever; bumping the version re-normalises once.
#define SW2_MAP_FMT_INDEX 0u
#define SW2_MAP_FMT_VERSION 2u
static uint8_t g_sw2Map[8];
static bool g_sw2MapLoaded = false;

static void switch2ProLoadMap()
{
	if (g_sw2MapLoaded)
		return;
	g_sw2MapLoaded = true;
	const bool stale = cfgExtRead(SW2_MAP_FMT_INDEX) != SW2_MAP_FMT_VERSION;
	bool normalize = stale;
	for (uint8_t i = 0; i < 8; i++) {
		const uint8_t saved = cfgExtRead((uint8_t)(3u + i));
		if (!stale && saved <= 23u) {
			g_sw2Map[i] = saved;
		} else {
			g_sw2Map[i] = def[i];
			cfgExtWrite((uint8_t)(3u + i), def[i]);
			normalize = true;
		}
	}
	if (normalize) {
		cfgExtWrite(SW2_MAP_FMT_INDEX, SW2_MAP_FMT_VERSION);
		saveCfg();
	}
}

uint8_t switch2ProMapGet(uint8_t index)
{
	switch2ProLoadMap();
	return index < 8u ? g_sw2Map[index] : 0u;
}

void switch2ProMapSet(uint8_t index, uint8_t value)
{
	switch2ProLoadMap();
	if (index >= 8u || value > 23u)
		return;
	g_sw2Map[index] = value;
	cfgExtWrite((uint8_t)(3u + index), value);
}
static volatile bool g_sw2VendorCommandPending = false;
static volatile bool g_sw2VendorInFlight = false;
static volatile uint8_t g_sw2VendorCommandLen = 0;
static volatile uint8_t g_sw2VendorReplyLen = 0;
static uint8_t g_sw2VendorOut[64];
static uint8_t g_sw2VendorReply[64];
static uint8_t g_sw2ControlReply[64];
static const uint8_t SW2_VENDOR_IDENTITY[64] = {
	0x01, 0x00, 'H',  'E',	'W',  '7',  '0',  '0',	'0',  '6',  '1',
	'6',  '9',  '7',  '8',	'0',  0x00, 0x00, 0x7e, 0x05, 0x69, 0x20,
	0x01, 0x06, 0x01, 0x23, 0x23, 0x23, 0xa0, 0xa0, 0xa0, 0xe6, 0xe6,
	0xe6, 0x32, 0x32, 0x32, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
static_assert(sizeof SW2_VENDOR_IDENTITY == 64,
	      "Switch 2 Pro vendor identity must remain 64 bytes");

static const uint8_t SW2_VENDOR_PROTOCOL[16] = {
	0x02, 0x01, 0x04, 0x00, 0x00, 0x00, 0x0c, 0x00,
	0x02, 0x03, 0x3d, 0x17, 0x69, 0xab, 0xa9, 0x3c,
};

static volatile uint8_t g_sw2VendorEpOut = 0;
static volatile uint8_t g_sw2VendorEpIn = 0;
static volatile uint8_t g_sw2Rhport = 0;
static unsigned long g_sw2LastReportMs = 0;
static bool g_sw2DrainRegistered = false;

class Switch2ProUsbInterface : public Adafruit_USBD_Interface {
    public:
	uint16_t getInterfaceDescriptor(uint8_t, uint8_t *buf,
					uint16_t bufsize) override
	{
		if (!buf)
			return sizeof SWITCH2_PRO_CFG_BODY;
		if (bufsize < sizeof SWITCH2_PRO_CFG_BODY)
			return 0;

		uint8_t first = TinyUSBDevice.allocInterface(5);
		uint8_t hidIn = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
		uint8_t hidOut = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		uint8_t vendorOut = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		uint8_t vendorIn = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
		uint8_t audioOut = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		uint8_t audioIn = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
		if (first != 0 || hidIn != 0x81 || hidOut != 0x01 ||
		    vendorOut != 0x02 || vendorIn != 0x82 || audioOut != 0x03 ||
		    audioIn != 0x83)
			return 0;

		uint8_t configStr =
			TinyUSBDevice.addStringDescriptor("Config_0");
		uint8_t hidStr = TinyUSBDevice.addStringDescriptor("If_Hid");
		uint8_t vendorStr = TinyUSBDevice.addStringDescriptor(
			"Switch 2 Pro Controller");
		if (configStr != 4 || hidStr != 5 || vendorStr != 6)
			return 0;

		memcpy(buf, SWITCH2_PRO_CFG_BODY, sizeof SWITCH2_PRO_CFG_BODY);
		return sizeof SWITCH2_PRO_CFG_BODY;
	}
};
static Switch2ProUsbInterface g_sw2Usb;

static inline uint16_t sw2Stick12(int16_t v)
{
	return (uint16_t)(((int32_t)v + 32768) >> 4);
}

static inline void sw2PackStick(uint8_t out[3], int16_t x, int16_t y)
{
	uint16_t sx = sw2Stick12(x), sy = sw2Stick12(y);
	out[0] = (uint8_t)sx;
	out[1] = (uint8_t)((sx >> 8) | (sy << 4));
	out[2] = (uint8_t)(sy >> 4);
}

static void sw2SetCode(uint8_t code, uint8_t b09[3], uint8_t b05[4])
{
	switch (code) {
	case 1: // A
		b09[0] |= 0x02;
		b05[0] |= 0x08;
		break;
	case 2: // B
		b09[0] |= 0x01;
		b05[0] |= 0x04;
		break;
	case 3: // X
		b09[0] |= 0x08;
		b05[0] |= 0x02;
		break;
	case 4: // Y
		b09[0] |= 0x04;
		b05[0] |= 0x01;
		break;
	case 5: // L
		b09[1] |= 0x10;
		b05[2] |= 0x40;
		break;
	case 6: // R
		b09[0] |= 0x10;
		b05[0] |= 0x40;
		break;
	case 7: // left stick
		b09[1] |= 0x80;
		b05[1] |= 0x08;
		break;
	case 8: // right stick
		b09[0] |= 0x80;
		b05[1] |= 0x04;
		break;
	case 9: // minus
		b09[1] |= 0x40;
		b05[1] |= 0x01;
		break;
	case 10: // plus
		b09[0] |= 0x40;
		b05[1] |= 0x02;
		break;
	case 11: // home
		b09[2] |= 0x01;
		b05[1] |= 0x10;
		break;
	case 12: // up
		b09[1] |= 0x08;
		b05[2] |= 0x02;
		break;
	case 13: // down
		b09[1] |= 0x01;
		b05[2] |= 0x01;
		break;
	case 14: // left
		b09[1] |= 0x04;
		b05[2] |= 0x08;
		break;
	case 15: // right
		b09[1] |= 0x02;
		b05[2] |= 0x04;
		break;
	case 18: // capture
		b09[2] |= 0x02;
		b05[1] |= 0x20;
		break;
	case 19: // ZL
		b09[1] |= 0x20;
		b05[2] |= 0x80;
		break;
	case 20: // ZR
		b09[0] |= 0x20;
		b05[0] |= 0x80;
		break;
	case 21: // C
		b09[2] |= 0x10;
		b05[1] |= 0x40;
		break;
	case 22: // GR
		b09[2] |= 0x04;
		b05[3] |= 0x01;
		break;
	case 23: // GL
		b09[2] |= 0x08;
		b05[3] |= 0x02;
		break;
	default:
		break;
	}
}

static void sw2Buttons(uint8_t slot, uint8_t b09[3], uint8_t b05[4])
{
	memset(b09, 0, 3);
	memset(b05, 0, 4);
	uint32_t b = g_in[slot].buttons;
	if ((b & CHORD_BACK4) == CHORD_BACK4)
		b &= ~(uint32_t)(TB_A | TB_B | TB_X | TB_Y | TB_DUP | TB_DDN |
				 TB_DLF | TB_DRT);
	if (b & TB_A)
		sw2SetCode(g_abSwap ? 2 : 1, b09, b05);
	if (b & TB_B)
		sw2SetCode(g_abSwap ? 1 : 2, b09, b05);
	if (b & TB_X)
		sw2SetCode(g_abSwap ? 4 : 3, b09, b05);
	if (b & TB_Y)
		sw2SetCode(g_abSwap ? 3 : 4, b09, b05);
	if (b & TB_LB)
		sw2SetCode(5, b09, b05);
	if (b & TB_RB)
		sw2SetCode(6, b09, b05);
	if (g_in[slot].lt >= SW_TRIG_ON || (b & 0x8000000u))
		sw2SetCode(19, b09, b05);
	if (g_in[slot].rt >= SW_TRIG_ON || (b & 0x800000u))
		sw2SetCode(20, b09, b05);
	if (b & TB_L3)
		sw2SetCode(7, b09, b05);
	if (b & TB_R3)
		sw2SetCode(8, b09, b05);
	if (b & TB_DUP)
		sw2SetCode(12, b09, b05);
	if (b & TB_DDN)
		sw2SetCode(13, b09, b05);
	if (b & TB_DLF)
		sw2SetCode(14, b09, b05);
	if (b & TB_DRT)
		sw2SetCode(15, b09, b05);

	static const uint32_t source[8] = {
		TB_L4, TB_R4, TB_L5, TB_R5, TB_VIEW, TB_MENU, TB_STEAM, TB_QAM
	};
	for (uint8_t i = 0; i < 8; i++)
		if (b & source[i])
			sw2SetCode(switch2ProMapGet(i), b09, b05);
}

static uint8_t sw2PowerInfo(uint8_t slot)
{
	uint8_t pct = slot < NSLOT ? g_battery[slot] : 0;
	uint8_t level = pct ? (uint8_t)(((uint16_t)pct * 9u + 50u) / 100u) : 0;
	if (level > 9)
		level = 9;
	return (uint8_t)(level << 2);
}

static void sw2Build09(uint8_t slot, uint8_t out[63])
{
	memset(out, 0, 63);
	uint8_t b09[3], b05[4];
	sw2Buttons(slot, b09, b05);
	out[0] = g_sw2Counter8++;
	out[1] = sw2PowerInfo(slot);
	memcpy(out + 2, b09, sizeof b09);
	int16_t lx, ly, rx, ry;
	slotSticks(slot, &lx, &ly, &rx, &ry);
	sw2PackStick(out + 5, lx, ly);
	sw2PackStick(out + 8, rx, ry);
	out[0x0b] = (g_sw2Features & (1u << 5)) ? 0x38 : 0x30;
	out[0x0e] = 0; // native 0x09 packed motion remains unknown
}

static inline void sw2Put16(uint8_t *p, int16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)((uint16_t)v >> 8);
}

static void sw2Build05(uint8_t slot, uint8_t out[63])
{
	memset(out, 0, 63);
	uint8_t b09[3], b05[4];
	sw2Buttons(slot, b09, b05);
	uint32_t counter = g_sw2Counter32++;
	memcpy(out, &counter, sizeof counter);
	memcpy(out + 4, b05, sizeof b05);
	int16_t lx, ly, rx, ry;
	slotSticks(slot, &lx, &ly, &rx, &ry);
	sw2PackStick(out + 0x0a, lx, ly);
	sw2PackStick(out + 0x0d, rx, ry);
	out[0x1f] = 0xd8;
	out[0x20] = 0x0e;
	out[0x21] = 0x34;
	out[0x29] = 0x01;

	uint8_t *m = out + 0x2a;
	uint32_t ts = g_in[slot].imuTimestampUs;
	if (!ts)
		ts = micros();
	memcpy(m, &ts, sizeof ts);
	int16_t ax = (int16_t)(g_in[slot].ay / 4);
	int16_t ay = (int16_t)((-(int16_t)g_in[slot].ax) / 4);
	int16_t az = (int16_t)(g_in[slot].az / 4);
	int16_t gx = (int16_t)g_in[slot].gy;
	int16_t gy = (int16_t)(-(int16_t)g_in[slot].gx);
	int16_t gz = (int16_t)g_in[slot].gz;
	if (!g_swGyroLegacy) {
		gx = (int16_t)(((int32_t)gx * 4) / 5);
		gy = (int16_t)(((int32_t)gy * 9) / 10);
		gz = (int16_t)(((int32_t)gz * 9) / 10);
	}
	// m[4..5] temperature stays zero; sensor axes begin at m[6].
	sw2Put16(m + 6, ax);
	sw2Put16(m + 8, ay);
	sw2Put16(m + 10, az);
	sw2Put16(m + 12, gx);
	sw2Put16(m + 14, gy);
	sw2Put16(m + 16, gz);
}

static void sw2AckHeader(uint8_t *out, uint8_t cmd, uint8_t transport,
			 uint8_t sub)
{
	out[0] = cmd;
	out[1] = 0x01;
	out[2] = transport;
	out[3] = sub;
	out[4] = 0x00;
	out[5] = 0xf8;
	out[6] = 0x00;
	out[7] = 0x00;
}

static void sw2DataHeader(uint8_t *out, uint8_t cmd, uint8_t transport,
			  uint8_t sub)
{
	out[0] = cmd;
	out[1] = 0x01;
	out[2] = transport;
	out[3] = sub;
	out[4] = 0x10;
	out[5] = 0x78;
	out[6] = 0x00;
	out[7] = 0x00;
}

static uint8_t sw2QueueAckHeader(uint8_t cmd, uint8_t transport, uint8_t sub,
				 uint8_t *reply)
{
	sw2AckHeader(reply, cmd, transport, sub);
	return 8;
}

static uint8_t sw2QueueDataHeader(uint8_t cmd, uint8_t transport, uint8_t sub,
				  uint8_t *reply)
{
	sw2DataHeader(reply, cmd, transport, sub);
	return 8;
}

static const uint8_t SW2_FACTORY_13040[16] = {
	0x3b, 0xe0, 0xd3, 0x41, 0xc6, 0x60, 0x6a, 0xbc,
	0x4d, 0xd7, 0xa2, 0xbb, 0x71, 0x1e, 0xdd, 0x37,
};

static const uint8_t SW2_FACTORY_13080[49] = {
	0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a,
	0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e, 0xe2, 0x20, 0x0e, 0x9a,
	0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5,
	0x50, 0x2f, 0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff,
	0xb3, 0x67, 0x83, 0x2e, 0x66, 0x5e, 0x3a, 0x06, 0x5f,
};

static const uint8_t SW2_FACTORY_130C0[49] = {
	0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a,
	0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e, 0xe2, 0x20, 0x0e, 0x9a,
	0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5,
	0x50, 0x2f, 0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff,
	0x2c, 0x08, 0x84, 0xd1, 0x65, 0x63, 0x2a, 0x26, 0x62,
};

static const uint8_t SW2_FACTORY_13100[24] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xa6, 0xf2, 0x62, 0xbd, 0xa8, 0x00, 0x08, 0x3d, 0x2f, 0xed, 0x20, 0x41,
};

static void sw2OverlayFlash(uint32_t address, uint8_t *out, uint8_t len,
			    uint32_t knownAddress, const uint8_t *known,
			    uint8_t knownLen)
{
	uint32_t begin = address > knownAddress ? address : knownAddress;
	uint32_t end = address + len;
	uint32_t knownEnd = knownAddress + knownLen;
	if (end > knownEnd)
		end = knownEnd;
	if (begin >= end)
		return;
	memcpy(out + (begin - address), known + (begin - knownAddress),
	       end - begin);
}

static void sw2FillFlashBlock(uint32_t address, uint8_t *block, uint8_t len)
{
	// Uninitialised controller flash is 0xff; overlay captured Pro factory data.
	memset(block, 0xff, len);
	sw2OverlayFlash(address, block, len, 0x00013000, SW2_VENDOR_IDENTITY,
			sizeof SW2_VENDOR_IDENTITY);
	sw2OverlayFlash(address, block, len, 0x00013040, SW2_FACTORY_13040,
			sizeof SW2_FACTORY_13040);
	static const uint8_t factory13060[4] = { 0x4c, 0x09, 0x00, 0x00 };
	sw2OverlayFlash(address, block, len, 0x00013060, factory13060,
			sizeof factory13060);
	sw2OverlayFlash(address, block, len, 0x00013080, SW2_FACTORY_13080,
			sizeof SW2_FACTORY_13080);
	sw2OverlayFlash(address, block, len, 0x000130c0, SW2_FACTORY_130C0,
			sizeof SW2_FACTORY_130C0);
	sw2OverlayFlash(address, block, len, 0x00013100, SW2_FACTORY_13100,
			sizeof SW2_FACTORY_13100);
}

static uint8_t sw2HandleFlashCommand(const uint8_t *cmd, uint8_t n,
				     uint8_t *reply)
{
	uint8_t sub = cmd[3];
	if (n < 16)
		return sw2QueueDataHeader(0x02, cmd[2], sub, reply);

	uint8_t requested;
	if (sub == 0x01)
		requested = 0x40;
	else if (sub == 0x04)
		requested = cmd[8];
	else
		return sw2QueueDataHeader(0x02, cmd[2], sub, reply);

	sw2DataHeader(reply, 0x02, cmd[2], sub);
	uint8_t copied = requested > 80 ? 80 : requested;
	uint32_t address = (uint32_t)cmd[12] | ((uint32_t)cmd[13] << 8) |
			   ((uint32_t)cmd[14] << 16) |
			   ((uint32_t)cmd[15] << 24);
	reply[8] = copied;
	reply[12] = cmd[12];
	reply[13] = cmd[13];
	reply[14] = cmd[14];
	reply[15] = cmd[15];
	sw2FillFlashBlock(address, reply + 16, copied);
	return (uint8_t)(16 + copied);
}

static void sw2FeatureInfo(uint8_t flags, uint8_t out[8])
{
	memset(out, 0, 8);
	if (flags & 0x01)
		out[0] = 0x07;
	if (flags & 0x02)
		out[1] = 0x07;
	if (flags & 0x04)
		out[2] = 0x01;
	if (flags & 0x80)
		out[3] = 0x01;
	if (flags & 0x10)
		out[4] = 0x01;
	if (flags & 0x20)
		out[5] = 0x03;
}

static uint8_t sw2HandleUsbCommand(const uint8_t *cmd, uint8_t n,
				   uint8_t *reply)
{
	uint8_t sub = cmd[3];
	sw2AckHeader(reply, 0x03, cmd[2], sub);
	if (sub == 0x03) {
		g_sw2InputEnabled = n >= 9 && cmd[8] != 0;
		reply[8] = 0x01;
		return 12;
	}
	if (sub == 0x0d) {
		g_sw2InputEnabled = true;
		reply[8] = 0x01;
		return 12;
	}
	if (sub == 0x0f) {
		reply[8] = 0x05;
		return 12;
	}
	if (sub == 0x07 || sub == 0x09) {
		sw2DataHeader(reply, 0x03, cmd[2], sub);
		return 8;
	}
	if (sub == 0x0a && n >= 9 && (cmd[8] == 0x05 || cmd[8] == 0x09))
		g_sw2ActiveReport = cmd[8];
	return 8;
}

static uint8_t g_sw2PairLtk[16];
static bool g_sw2PairLtkValid = false;
static const uint8_t SW2_PAIR_PUBLIC[16] = {
	0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1,
	0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10,
};

struct Sw2EcbBlock {
	uint8_t key[16];
	uint8_t clear[16];
	uint8_t cipher[16];
} __attribute__((aligned(4)));

static bool sw2Aes128Ecb(const uint8_t key[16], const uint8_t clear[16],
			 uint8_t cipher[16])
{
	static Sw2EcbBlock block;
	memcpy(block.key, key, 16);
	memcpy(block.clear, clear, 16);
	NRF_ECB->TASKS_STOPECB = 1;
	NRF_ECB->EVENTS_ENDECB = 0;
	NRF_ECB->EVENTS_ERRORECB = 0;
	NRF_ECB->ECBDATAPTR = (uint32_t)&block;
	NRF_ECB->TASKS_STARTECB = 1;
	while (!NRF_ECB->EVENTS_ENDECB && !NRF_ECB->EVENTS_ERRORECB) {
	}
	if (NRF_ECB->EVENTS_ERRORECB)
		return false;
	memcpy(cipher, block.cipher, 16);
	return true;
}

static void sw2PairResponse(const uint8_t challenge[16], uint8_t out[16])
{
	uint8_t key[16], clear[16], cipher[16];
	for (uint8_t i = 0; i < 16; i++) {
		key[i] = g_sw2PairLtk[15 - i];
		clear[i] = challenge[15 - i];
	}
	if (!g_sw2PairLtkValid || !sw2Aes128Ecb(key, clear, cipher)) {
		memset(out, 0, 16);
		return;
	}
	// Nintendo's confirmation packet uses the AES output in the order returned
	// by the hardware after reversing the key and challenge inputs.
	memcpy(out, cipher, 16);
}

static void sw2ControllerAddress(uint8_t out[6])
{
	uint32_t a = NRF_FICR->DEVICEID[0], b = NRF_FICR->DEVICEID[1];
	out[0] = (uint8_t)a;
	out[1] = (uint8_t)(a >> 8);
	out[2] = (uint8_t)(a >> 16);
	out[3] = (uint8_t)(a >> 24);
	out[4] = (uint8_t)b;
	out[5] = (uint8_t)(b >> 8);
}

static uint8_t sw2HandlePairing(const uint8_t *cmd, uint8_t n, uint8_t *reply)
{
	uint8_t sub = cmd[3];
	sw2DataHeader(reply, 0x15, cmd[2], sub);
	if (sub == 0x01) {
		uint8_t addr[6];
		sw2ControllerAddress(addr);
		reply[8] = 0x01;
		reply[9] = 0x04;
		reply[10] = 0x01;
		memcpy(reply + 11, addr, sizeof addr);
		return 17;
	}
	if (sub == 0x04 && n >= 25) {
		reply[8] = 0x01;
		memcpy(reply + 9, SW2_PAIR_PUBLIC, sizeof SW2_PAIR_PUBLIC);
		for (uint8_t i = 0; i < 16; i++)
			g_sw2PairLtk[i] = cmd[9 + i] ^ SW2_PAIR_PUBLIC[i];
		g_sw2PairLtkValid = true;
		return 25;
	}
	if (sub == 0x02 && n >= 25) {
		reply[8] = 0x01;
		sw2PairResponse(cmd + 9, reply + 9);
		return 25;
	}
	if (sub == 0x03) {
		reply[8] = 0x01;
		return 9;
	}
	reply[8] = 0x01;
	return 9;
}

static uint8_t sw2HandleFeatures(const uint8_t *cmd, uint8_t n, uint8_t *reply)
{
	uint8_t sub = cmd[3], flags = n >= 9 ? cmd[8] : 0;
	sw2DataHeader(reply, 0x0c, cmd[2], sub);
	if (sub == 0x01) {
		sw2FeatureInfo(flags, reply + 12);
		return 20;
	}
	if (sub == 0x02)
		g_sw2FeatureMask = flags;
	else if (sub == 0x03) {
		g_sw2FeatureMask = 0;
		g_sw2Features = 0;
	} else if (sub == 0x04) {
		g_sw2Features |= flags & g_sw2FeatureMask;
	} else if (sub == 0x05) {
		g_sw2Features &= (uint8_t) ~(flags & g_sw2FeatureMask);
	}
	return 12;
}

static void sw2BuildVendorReply(void)
{
	uint8_t cmd[64];
	uint8_t reply[96];
	uint8_t n;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	n = g_sw2VendorCommandLen;
	if (n > sizeof cmd)
		n = sizeof cmd;
	memcpy(cmd, g_sw2VendorOut, n);
	g_sw2VendorCommandPending = false;
	__set_PRIMASK(pm);

	memset(reply, 0, sizeof reply);
	if (n < 8 || cmd[1] != 0x91) {
		g_sw2VendorReplyLen = 0;
		return;
	}
	uint8_t id = cmd[0], sub = cmd[3], seq = cmd[2];
	sw2AckHeader(reply, id, seq, sub);
	uint8_t replyLen = 8;

	switch (id) {
	case 0x02:
		replyLen = sw2HandleFlashCommand(cmd, n, reply);
		break;
	case 0x03:
		replyLen = sw2HandleUsbCommand(cmd, n, reply);
		break;
	case 0x0c:
		replyLen = sw2HandleFeatures(cmd, n, reply);
		break;
	case 0x15:
		replyLen = sw2HandlePairing(cmd, n, reply);
		break;
	case 0x07:
		replyLen = sw2QueueDataHeader(id, seq, sub, reply);
		reply[8] = 0;
		replyLen = 9;
		break;
	case 0x01:
		if (sub == 0x0c) {
			sw2DataHeader(reply, id, seq, sub);
			reply[8] = 0x61;
			reply[9] = 0x12;
			reply[10] = 0x50;
			reply[11] = 0x0d;
			replyLen = 12;
		}
		break;
	case 0x10:
		if (sub == 0x01) {
			static const uint8_t info[12] = {
				0x02, 0x01, 0x04, 0x02, 0x0c, 0x00,
				0x00, 0x00, 0x00, 0x02, 0x03, 0x00,
			};
			sw2DataHeader(reply, id, seq, sub);
			memcpy(reply + 8, info, sizeof info);
			replyLen = 20;
		}
		break;
	case 0x11:
		if (sub == 0x03) {
			static const uint8_t imuCal[] = {
				0x01, 0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c,
				0x3b, 0x79, 0x7d, 0x8b, 0x3a, 0x0a, 0xe8, 0x9c,
				0x42, 0x58, 0xa0, 0x0b, 0x42, 0x0a, 0xe8, 0x9c,
				0x41, 0x58, 0xa0, 0x0b, 0x41,
			};
			sw2DataHeader(reply, id, seq, sub);
			memcpy(reply + 8, imuCal, sizeof imuCal);
			replyLen = (uint8_t)(8 + sizeof imuCal);
		} else if (sub == 0x01) {
			sw2DataHeader(reply, id, seq, sub);
			reply[8] = 0x01;
			replyLen = 12;
		}
		break;
	case 0x16:
		if (sub == 0x01) {
			sw2DataHeader(reply, id, seq, sub);
			memset(reply + 8, 0, 24);
			replyLen = 32;
		}
		break;
	case 0x18:
		if (sub == 0x01) {
			static const uint8_t powerInfo[8] = {
				0x00, 0x00, 0x40, 0xf0, 0x00, 0x00, 0x60, 0x00,
			};
			sw2DataHeader(reply, id, seq, sub);
			memcpy(reply + 8, powerInfo, sizeof powerInfo);
			replyLen = 16;
		} else if (sub == 0x03 && n >= 9) {
			sw2DataHeader(reply, id, seq, sub);
			reply[8] = cmd[8];
			replyLen = 9;
		}
		break;
	case 0x06:
		if (sub == 0x02) {
			g_sw2VendorReplyLen = 0;
			return;
		}
		replyLen = sw2QueueDataHeader(id, seq, sub, reply);
		break;
	case 0x09:
	case 0x0a:
	case 0x0d:
	case 0x17:
		replyLen = sw2QueueDataHeader(id, seq, sub, reply);
		break;
	case 0x0b:
		sw2DataHeader(reply, id, seq, sub);
		if (sub == 0x03) {
			reply[8] = 0xa5;
			reply[9] = 0x0e;
			replyLen = 12;
		} else if (sub == 0x04) {
			reply[8] = 0x34;
			reply[10] = 0x83;
			replyLen = 12;
		} else if (sub == 0x06) {
			reply[8] = 0x11;
			replyLen = 12;
		}
		break;
	default:
		replyLen = sw2QueueDataHeader(id, seq, sub, reply);
		break;
	}

	uint8_t first = replyLen > sizeof g_sw2VendorReply ?
				sizeof g_sw2VendorReply :
				replyLen;
	memcpy(g_sw2VendorReply, reply, first);
	if (replyLen > first)
		memcpy(g_sw2VendorOut, reply + first, replyLen - first);
	g_sw2VendorReplyLen = replyLen;
}

static void sw2Drain(void)
{
	if (g_usbMode != MODE_SW2_PRO || g_usbMountCount == 0)
		return;

	if (g_sw2VendorCommandPending && !g_sw2VendorInFlight) {
		sw2BuildVendorReply();
		uint8_t n = g_sw2VendorReplyLen;
		uint8_t first = n > sizeof g_sw2VendorReply ?
					sizeof g_sw2VendorReply :
					n;
		if (first && g_sw2VendorEpIn &&
		    usbd_edpt_xfer(g_sw2Rhport, g_sw2VendorEpIn,
				   g_sw2VendorReply, first))
			g_sw2VendorInFlight = true;
	}

	if (!g_sw2InputEnabled || !tud_hid_n_ready(0))
		return;
	if ((uint32_t)(millis() - g_sw2LastReportMs) < USB_STREAM_MS)
		return;
	int bond = g_usbToBond[0];
	if (bond < 0 || bond >= NSLOT)
		return;
	uint8_t p[63];
	uint8_t rid = g_sw2ActiveReport;
	if (rid == 0x05)
		sw2Build05((uint8_t)bond, p);
	else {
		rid = 0x09;
		sw2Build09((uint8_t)bond, p);
	}
	if (tud_hid_n_report(0, rid, p, sizeof p))
		g_sw2LastReportMs = millis();
}

bool switch2ProVendorControlXfer(uint8_t rhport, uint8_t stage,
				 const tusb_control_request_t *request)
{
	if (g_usbMode != MODE_SW2_PRO || !request)
		return false;

	bool identity = request->bmRequestType == 0xc0 &&
			request->bRequest == 0x03 && request->wLength >= 64;
	bool protocol = request->bmRequestType == 0xc0 &&
			request->bRequest == 0x02 && request->wLength >= 16;
	bool commit = request->bmRequestType == 0x40 &&
		      request->bRequest == 0x04 && request->wLength == 0;
	if (!identity && !protocol && !commit)
		return false;
	if (stage != CONTROL_STAGE_SETUP)
		return true;

	if (commit)
		return tud_control_status(rhport, request);
	if (identity) {
		memcpy(g_sw2ControlReply, SW2_VENDOR_IDENTITY,
		       sizeof SW2_VENDOR_IDENTITY);
		return tud_control_xfer(rhport, request, g_sw2ControlReply,
					sizeof SW2_VENDOR_IDENTITY);
	}
	memcpy(g_sw2ControlReply, SW2_VENDOR_PROTOCOL,
	       sizeof SW2_VENDOR_PROTOCOL);
	return tud_control_xfer(rhport, request, g_sw2ControlReply,
				sizeof SW2_VENDOR_PROTOCOL);
}

static void sw2DriverInit(void)
{
	g_sw2VendorEpOut = g_sw2VendorEpIn = 0;
}
static bool sw2DriverDeinit(void)
{
	return true;
}
static void sw2DriverReset(uint8_t rhport)
{
	(void)rhport;
	g_sw2VendorEpOut = g_sw2VendorEpIn = 0;
	g_sw2VendorCommandPending = false;
	g_sw2VendorInFlight = false;
	g_sw2InputEnabled = false;
	g_sw2ActiveReport = 0x05;
	g_sw2Features = 0;
	g_sw2FeatureMask = 0;
	g_sw2LastRumbleBond = -1;
	g_sw2LastRumbleLeft = 0;
	g_sw2LastRumbleRight = 0;
}

static uint16_t sw2DriverOpen(uint8_t rhport, tusb_desc_interface_t const *itf,
			      uint16_t maxLen)
{
	if (g_usbMode != MODE_SW2_PRO)
		return 0;
	g_sw2Rhport = rhport;
	if (itf->bInterfaceNumber == 0 &&
	    itf->bInterfaceClass == TUSB_CLASS_HID)
		return hidd_open(rhport, itf, maxLen);

	if (itf->bInterfaceNumber == 1 && itf->bInterfaceClass == 0xff) {
		uint8_t const *p = (uint8_t const *)itf;
		uint8_t const *end = p + maxLen;
		uint16_t used = 0;
		uint8_t opened = 0;
		while (p < end) {
			uint8_t len = p[0], type = p[1];
			if (!len)
				return 0;
			if (p != (uint8_t const *)itf &&
			    (type == TUSB_DESC_INTERFACE ||
			     type == TUSB_DESC_INTERFACE_ASSOCIATION))
				break;
			if (type == TUSB_DESC_ENDPOINT) {
				auto ep = (tusb_desc_endpoint_t const *)p;
				if (!usbd_edpt_open(rhport, ep))
					return 0;
				if (tu_edpt_dir(ep->bEndpointAddress) ==
				    TUSB_DIR_IN)
					g_sw2VendorEpIn = ep->bEndpointAddress;
				else
					g_sw2VendorEpOut = ep->bEndpointAddress;
				opened++;
			}
			used += len;
			p += len;
		}
		if (opened != 2 || !g_sw2VendorEpOut || !g_sw2VendorEpIn)
			return 0;
		if (!usbd_edpt_xfer(rhport, g_sw2VendorEpOut, g_sw2VendorOut,
				    sizeof g_sw2VendorOut))
			return 0;
		return used;
	}

	// Preserve the captured audio-control/streaming descriptor block as one
	// associated three-interface Nintendo function. Audio data itself is not
	// synthesized by OpenPuck.
	if (itf->bInterfaceNumber == 2 &&
	    itf->bInterfaceClass == TUSB_CLASS_AUDIO)
		return maxLen;
	return 0;
}

static bool sw2DriverControl(uint8_t rhport, uint8_t stage,
			     tusb_control_request_t const *req)
{
	if (g_usbMode != MODE_SW2_PRO)
		return false;
	if (req->wIndex == 0)
		return hidd_control_xfer_cb(rhport, stage, req);
	return stage != CONTROL_STAGE_SETUP;
}

static bool sw2DriverXfer(uint8_t rhport, uint8_t ep, xfer_result_t result,
			  uint32_t transferred)
{
	if (ep == g_sw2VendorEpOut) {
		if (result == XFER_RESULT_SUCCESS) {
			g_sw2VendorCommandLen =
				(uint8_t)(transferred > 64 ? 64 : transferred);
			g_sw2VendorCommandPending = true;
		}
		// Intentionally do not re-arm OUT here. The shared response buffer
		// belongs to the matching IN transfer until that transfer completes.
		return true;
	}
	if (ep == g_sw2VendorEpIn) {
		if (result == XFER_RESULT_SUCCESS && g_sw2VendorReplyLen > 64) {
			uint8_t tail = g_sw2VendorReplyLen - 64;
			g_sw2VendorReplyLen = tail;
			return usbd_edpt_xfer(rhport, g_sw2VendorEpIn,
					      g_sw2VendorOut, tail);
		}
		g_sw2VendorReplyLen = 0;
		g_sw2VendorInFlight = false;
		return usbd_edpt_xfer(rhport, g_sw2VendorEpOut, g_sw2VendorOut,
				      sizeof g_sw2VendorOut);
	}
	return hidd_xfer_cb(rhport, ep, result, transferred);
}

static const usbd_class_driver_t g_sw2Driver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "SW2-PRO",
#endif
	.init = sw2DriverInit,
	.deinit = sw2DriverDeinit,
	.reset = sw2DriverReset,
	.open = sw2DriverOpen,
	.control_xfer_cb = sw2DriverControl,
	.xfer_cb = sw2DriverXfer,
	.sof = nullptr,
};

const usbd_class_driver_t *switch2ProClassDriver(void)
{
	return &g_sw2Driver;
}

extern "C" uint8_t const *__real_tud_descriptor_device_cb(void);
extern "C" uint8_t const *__wrap_tud_descriptor_device_cb(void)
{
	uint8_t const *real = __real_tud_descriptor_device_cb();
	if (g_usbMode != MODE_SW2_PRO)
		return real;
	static tusb_desc_device_t d;
	memcpy(&d, real, sizeof d);
	d.bDeviceClass = 0xef;
	d.bDeviceSubClass = 0x02;
	d.bDeviceProtocol = 0x01;
	return (uint8_t const *)&d;
}

extern "C" uint8_t const *__real_tud_hid_descriptor_report_cb(uint8_t itf);
extern "C" uint8_t const *__wrap_tud_hid_descriptor_report_cb(uint8_t itf)
{
	if (g_usbMode == MODE_SW2_PRO && itf == 0)
		return SWITCH2_PRO_HID_DESC;
	return __real_tud_hid_descriptor_report_cb(itf);
}

extern "C" uint16_t __real_tud_hid_get_report_cb(uint8_t itf, uint8_t reportId,
						 hid_report_type_t reportType,
						 uint8_t *buffer,
						 uint16_t reqLen);
extern "C" uint16_t __wrap_tud_hid_get_report_cb(uint8_t itf, uint8_t reportId,
						 hid_report_type_t reportType,
						 uint8_t *buffer,
						 uint16_t reqLen)
{
	if (g_usbMode != MODE_SW2_PRO || itf != 0)
		return __real_tud_hid_get_report_cb(itf, reportId, reportType,
						    buffer, reqLen);
	(void)reportType;
	if (!buffer || !reqLen)
		return 0;
	uint8_t p[63];
	int bond = g_usbMountCount ? g_usbToBond[0] : -1;
	if (bond < 0)
		memset(p, 0, sizeof p);
	else if (reportId == 0x05)
		sw2Build05((uint8_t)bond, p);
	else if (reportId == 0x09)
		sw2Build09((uint8_t)bond, p);
	else
		return 0;
	uint16_t n = reqLen < sizeof p ? reqLen : sizeof p;
	memcpy(buffer, p, n);
	return n;
}

// Switch 2 Pro output report 0x02 carries two 16-byte LRA packets after the
// report ID: left then right. Each packet starts with a 0x50|sequence byte and
// contains three absolute 5-byte HD-rumble frames. Steam Controller report
// 0x80 cannot reproduce Nintendo's frequency-domain waveform, so preserve the
// spatial left/right intent, reduce each motor to its peak encoded
// amplitude, and route it to the corresponding Steam haptic channel.
static uint16_t sw2RumbleFramePeak(const uint8_t frame[5])
{
	uint16_t highAmp = (uint16_t)(((uint16_t)(frame[1] & 0xfc) << 4) |
				      ((uint16_t)(frame[2] & 0x0f) << 12));
	uint16_t lowAmp =
		(uint16_t)((frame[3] & 0xc0) | ((uint16_t)frame[4] << 8));
	return highAmp > lowAmp ? highAmp : lowAmp;
}

static uint16_t sw2RumbleMotorPeak(const uint8_t packet[16])
{
	uint16_t peak = 0;
	for (uint8_t i = 0; i < 3; i++) {
		uint16_t amp = sw2RumbleFramePeak(packet + 1 + 5 * i);
		if (amp > peak)
			peak = amp;
	}
	return peak;
}

static void sw2Rumble(const uint8_t *payload, uint16_t size)
{
	if (!payload || size < 32 || !g_usbMountCount)
		return;
	int bond = g_usbToBond[0];
	if (bond < 0 || bond >= NSLOT)
		return;

	uint16_t left = sw2RumbleMotorPeak(payload);
	uint16_t right = sw2RumbleMotorPeak(payload + 16);
	if (bond == g_sw2LastRumbleBond && left == g_sw2LastRumbleLeft &&
	    right == g_sw2LastRumbleRight)
		return;

	if (hapticSteamRumble(left, right, (uint8_t)bond)) {
		g_sw2LastRumbleBond = (int8_t)bond;
		g_sw2LastRumbleLeft = left;
		g_sw2LastRumbleRight = right;
	}
}

extern "C" void __real_tud_hid_set_report_cb(uint8_t itf, uint8_t reportId,
					     hid_report_type_t reportType,
					     uint8_t const *buffer,
					     uint16_t size);
extern "C" void __wrap_tud_hid_set_report_cb(uint8_t itf, uint8_t reportId,
					     hid_report_type_t reportType,
					     uint8_t const *buffer,
					     uint16_t size)
{
	if (g_usbMode == MODE_SW2_PRO && itf == 0) {
		if (reportType == HID_REPORT_TYPE_OUTPUT) {
			if (reportId == 0x02) {
				sw2Rumble(buffer, size);
			} else if (reportId == 0 && size > 0 &&
				   buffer[0] == 0x02) {
				sw2Rumble(buffer + 1, (uint16_t)(size - 1));
			}
		}
		return;
	}
	__real_tud_hid_set_report_cb(itf, reportId, reportType, buffer, size);
}

void Switch2ProController::begin()
{
}
uint8_t Switch2ProController::maxSlots() const
{
	return 1;
}
void Switch2ProController::usbIdentity()
{
	USBDevice.setID(0x057e, 0x2069);
	USBDevice.setVersion(0x0200);
	USBDevice.setDeviceVersion(0x0201);
	USBDevice.setManufacturerDescriptor("Nintendo");
	USBDevice.setProductDescriptor("Switch 2 Pro Controller");
	USBDevice.setSerialDescriptor("00");
}
void Switch2ProController::beginPool()
{
	if (!g_sw2DrainRegistered) {
		usbTxRegisterDrain(sw2Drain);
		g_sw2DrainRegistered = true;
	}
}
void Switch2ProController::mountSlots(uint8_t k)
{
	if (k)
		USBDevice.addInterface(g_sw2Usb);
}
void Switch2ProController::task()
{
	// Report and vendor transfers are drained by usbTxPump() so endpoint
	// submission shares the same priority-inversion protection as other modes.
}
