#include "mode_ps5.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_mount.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

Ps5Controller g_ps5Ctl;

// Byte-for-byte the USB report descriptor of a real DualSense (CFI-ZCT1W, github.com/nondebug/dualsense).
// Sony's libScePad inspects the parsed report layout, so feature reports we never answer (0x80-0xF5) are
// still declared.
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
static_assert(sizeof PS5_HID_DESC == 273, "PS5 report descriptor size");
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

// USB firmware-information payload from a DualSense, excluding report ID 0x20.
// This emulated identity is independent of OpenPuck's own build version.
static const uint8_t PS5_FIRMWARE_INFO[] = {
	0x4A, 0x75, 0x6C, 0x20, 0x20, 0x34, 0x20, 0x32, 0x30, 0x32, 0x35,
	0x31, 0x30, 0x3A, 0x31, 0x30, 0x3A, 0x33, 0x32, 0x02, 0x00, 0x04,
	0x00, 0x13, 0x04, 0x00, 0x00, 0x2A, 0x00, 0x10, 0x01, 0x50, 0x38,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
	0x06, 0x00, 0x00, 0x2A, 0x00, 0x01, 0x00, 0x0A, 0x00, 0x02, 0x00,
	0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static_assert(sizeof PS5_FIRMWARE_INFO == 63, "PS5 firmware report size");

// Pairing-report bytes after the pad's MAC: a constant 08 25 00, then the paired host's MAC (little-endian).
static const uint8_t PS5_PAIRING_TAIL[9] = { 0x08, 0x25, 0x00, 0x1E, 0x00,
					     0xEE, 0x74, 0xD0, 0xBC };

// GET_FEATURE handler. Per-slot dispatch via per-instance callback. Sizes per drivers/hid/hid-playstation.c:
// 0x05=41, 0x09=20, 0x20=64. TinyUSB writes the report id itself and hands us the buffer PAST it, so we
// fill only the PAYLOAD and return size-1.
//
// A real USB DualSense either answers a feature GET in full or stalls it; it never sends a short reply.
// TinyUSB always prepends the report id and stalls only when the total length is 0, so PS5_STALL wraps its
// uint16_t length (1 + 0xFFFF) to 0.
#define PS5_STALL 0xFFFFu
static void ps5Build(uint8_t usbSlot, uint8_t slot, uint8_t out[63]);

static uint16_t ps5GetCommon(uint8_t slot, uint8_t rid, hid_report_type_t type,
			     uint8_t *buf, uint16_t reqlen)
{
	(void)slot;
	if (!buf || reqlen == 0)
		return 0;
	memset(buf, 0, reqlen);

	// DirectInput and game polling via GET_REPORT(INPUT, 0x01)
	if (type == HID_REPORT_TYPE_INPUT) {
		if ((rid == 0x01 || rid == 0) && reqlen >= 63) {
			int bond = (slot < NSLOT) ? g_usbToBond[slot] : -1;
			if (bond < 0 || !g_slot[bond].used) {
				for (int s = 0; s < NSLOT; s++) {
					if (g_slot[s].used) {
						bond = s;
						break;
					}
				}
			}
			if (bond >= 0)
				ps5Build(slot, (uint8_t)bond, buf);
			return 63;
		}
		return 0;
	}

	if (type != HID_REPORT_TYPE_FEATURE)
		return 0;

	// Payload sizes as declared by PS5_HID_DESC; the replies mirror a real pad's (all-zero unless set below).
	uint16_t len;
	switch (rid) {
	case 0x05: // motion calibration
		len = 40;
		break;
	case 0x09: // pairing info / MAC
		len = 19;
		break;
	case 0x20: // firmware info
	case 0x22: // hardware info
	case 0x81:
	case 0x83:
	case 0xE0:
	case 0xF1:
		len = 63;
		break;
	case 0x85:
		len = 2;
		break;
	case 0xF2:
		len = 15;
		break;
	case 0xF5:
		len = 3;
		break;
	default: // 0x08, 0x0A, 0x21, 0x80, 0x82, 0x84, 0xA0, 0xF0, 0xF4 and undeclared ids
		return PS5_STALL;
	}
	if (reqlen < len)
		return PS5_STALL;

	switch (rid) {
	case 0x05:
		psNeutralCalib(buf);
		break;
	case 0x09:
		// MAC at kernel buf[1..6] = payload[0..5]
		memcpy(buf, g_ps5Mac[slot], 6);
		// A real pad follows its MAC with 08 25 00 and the MAC of the host it is paired with; mirror
		// that rather than an all-zero tail, which a genuine DualSense never reports.
		memcpy(buf + 6, PS5_PAIRING_TAIL, sizeof PS5_PAIRING_TAIL);
		break;
	case 0x20:
		memcpy(buf, PS5_FIRMWARE_INFO, sizeof PS5_FIRMWARE_INFO);
		break;
	case 0x22:
		// Repeats fields of the firmware report around the pad's MAC. The real pad's remaining bytes are
		// of unknown meaning or per-unit, so they stay zero.
		buf[0] = PS5_FIRMWARE_INFO[19];
		memcpy(buf + 2, PS5_FIRMWARE_INFO + 23, 8);
		memcpy(buf + 16, g_ps5Mac[slot], 6);
		memcpy(buf + 22, PS5_FIRMWARE_INFO + 51, 6);
		memcpy(buf + 52, PS5_FIRMWARE_INFO + 47, 4);
		break;
	case 0x83:
		memset(buf, 0xFF, 4);
		break;
	case 0x85:
		buf[1] = 0xFF;
		break;
	case 0xE0:
		buf[0] = 0x04;
		buf[2] = 0x18;
		buf[3] = 0x07;
		buf[11] = 0x06;
		break;
	case 0xF2:
		buf[2] = 0x10;
		break;
	}
	return len;
}
static void ps5SetCommon(uint8_t slot, uint8_t rid, hid_report_type_t type,
			 uint8_t const *b, uint16_t n)
{
	if (type != HID_REPORT_TYPE_OUTPUT || !b || n < 1)
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
	if (id != 0x02 || pn < 4)
		return;
	// `slot` here is the USB slot the report arrived on -> route rumble to the bond slot it's mapped to.
	int bond = (slot < NSLOT) ? g_usbToBond[slot] : -1;
	if (bond < 0 || !g_slot[bond].used) {
		for (int s = 0; s < NSLOT; s++) {
			if (g_slot[s].used) {
				bond = s;
				break;
			}
		}
	}
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
	psPadClickEdge(slot, b & (TB_LPADC | TB_RPADC));
	// A pad mapped to a stick must NOT also report as a touchpad contact -- the host would read the same
	// finger twice (stick deflection AND a cursor drag).
	bool lTouch = g_padStick[0] == PS_OFF &&
		      ((b & TB_LPADT) || (b & TB_LPADC)),
	     rTouch = g_padStick[1] == PS_OFF &&
		      ((b & TB_RPADT) || (b & TB_RPADC));
	memset(out, 0, 63);
	int16_t lx, ly, rx, ry;
	slotSticks(slot, &lx, &ly, &rx, &ry);
	out[0] = swStick(lx, false);
	out[1] = swStick(ly, true);
	out[2] = swStick(rx, false);
	out[3] = swStick(ry, true);
	out[4] = g_in[slot].lt;
	out[5] = g_in[slot].rt;
	static uint8_t seq[NSLOT] = { 0 };
	out[6] = seq[usbSlot]++;
	out[7] = psHatNibble(b) | psFaceNibble(b);
	out[8] = psShouldersByte(b, g_in[slot].lt, g_in[slot].rt);
	out[9] = ((b & TB_STEAM) ? 0x01 : 0) |
		 ((b & TB_TOUCH || b & TB_LPADC || b & TB_RPADC) ? 0x02 : 0) |
		 ((b & TB_MUTE) ? 0x04 : 0);
	static uint32_t pktSeq[NSLOT] = { 0 };
	uint32_t s = ++pktSeq[usbSlot];
	out[11] = (uint8_t)(s & 0xFF);
	out[12] = (uint8_t)((s >> 8) & 0xFF);
	out[13] = (uint8_t)((s >> 16) & 0xFF);
	out[14] = (uint8_t)((s >> 24) & 0xFF);
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
	// SDL treats the normal DualSense 32-bit sensor timestamp as ~1/3 us ticks.
	uint32_t ps5SensorTimestamp = g_in[slot].imuTimestampUs * 3u;
	out[27] = (uint8_t)(ps5SensorTimestamp & 0xFF);
	out[28] = (uint8_t)((ps5SensorTimestamp >> 8) & 0xFF);
	out[29] = (uint8_t)((ps5SensorTimestamp >> 16) & 0xFF);
	out[30] = (uint8_t)((ps5SensorTimestamp >> 24) & 0xFF);
	// Values a real pad sends where we have no source: sensor temperature, two constant bytes, and a second
	// clock that runs alongside the sensor timestamp.
	out[31] = 0x02;
	out[41] = 0x09;
	out[42] = 0x09;
	memcpy(out + 48, out + 27, 4);
	uint16_t tlx, tly, trx, trry;
	steamPadsToTouch(b, PS5_TOUCH_H, g_in[slot].lpx, g_in[slot].lpy,
			 g_in[slot].rpx, g_in[slot].rpy, &tlx, &tly, &trx,
			 &trry);
	touchPackPadsStateful(slot, out + 32, lTouch, rTouch, tlx, tly, trx,
			      trry);
	out[52] = PS5_STATUS_USB;
	PsImuFrame imu = psImuFromSteam(g_in[slot]);
	le16(out + 15, imu.gx);
	le16(out + 17, imu.gy);
	le16(out + 19, imu.gz);
	le16(out + 21, imu.ax);
	le16(out + 23, imu.ay);
	le16(out + 25, imu.az);
	out[53] = 0x08; // USB connected state
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
	USBDevice.setVersion(0x0200);
	// bcdDevice 1.00 like a real DualSense; Windows/Wine report it to games as the HID VersionNumber.
	USBDevice.setDeviceVersion(0x0100);
	USBDevice.setManufacturerDescriptor("Sony Interactive Entertainment");
	USBDevice.setProductDescriptor("DualSense Wireless Controller");
}
#include "mode_ps5_audio.h"

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
// A real DualSense has its audio function on interfaces 0-2 and the gamepad on 3. Linux names the sound card
// after the first audio interface (...Wireless_Controller-00), a name GE-Proton's DualSense
// patches match, and Wine reports the gamepad's interface number to games.
void Ps5Controller::mountSlots(uint8_t k)
{
	USBDevice.addInterface(g_ps5Audio);
	uint8_t count = (k > 0) ? k : 1;
	for (uint8_t u = 0; u < count; u++)
		USBDevice.addInterface(g_ps5[u]);
}

// A real DualSense reports no serial string, so its card is named ...Wireless_Controller-00 with no serial in
// it. The Adafruit core always reports one, so drop it in the clean mode. MODE_PS5 keeps it: the mount count
// in it makes Windows re-read the configuration when a controller connects.
extern "C" uint8_t const *__real_tud_descriptor_device_cb(void);
extern "C" uint8_t const *__wrap_tud_descriptor_device_cb(void)
{
	const uint8_t *desc = __real_tud_descriptor_device_cb();
	if (g_usbMode != MODE_PS5_GAME)
		return desc;
	static tusb_desc_device_t s_noSerial __attribute__((aligned(4)));
	memcpy(&s_noSerial, desc, sizeof s_noSerial);
	s_noSerial.iSerialNumber = 0;
	return (const uint8_t *)&s_noSerial;
}
void Ps5Controller::task()
{
	ps5AudioTask();
	for (uint8_t u = 0; u < g_usbMountCount; u++) {
		if (!g_ps5[u].ready())
			continue;
		if (millis() - g_ps5LastMs[u] < USB_STREAM_MS)
			continue;
		int bond = g_usbToBond[u];
		if (bond < 0 || !g_slot[bond].used) {
			for (int s = 0; s < NSLOT; s++) {
				if (g_slot[s].used) {
					bond = s;
					break;
				}
			}
		}
		if (bond < 0)
			continue;
		g_ps5LastMs[u] = millis();
		uint8_t p[63];
		ps5Build(u, (uint8_t)bond, p);
		usbTxHid(&g_ps5[u], 0x01, p, sizeof p);
	}
}
