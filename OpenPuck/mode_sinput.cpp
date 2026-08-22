#include "mode_sinput.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "identity.h"
#include "rf_link.h" // g_battery / g_batteryState
#include "usb_mount.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

SInputController g_sinputCtl;

// Report descriptor per the SInput reference implementation (HandHeldLegend/SINPUT-LIB-HID, MIT-0): three
// reports on one interface -- 64-byte input 0x01 (gamepad state), 64-byte input 0x02 (command replies),
// 48-byte output 0x03 (haptics + commands). SDL talks to the device through hidapi, so the descriptor only has
// to make the host expose those three sizes; the field usages are cosmetic padding around the wire layout.
static const uint8_t SIN_HID_DESC[] = {
	// clang-format off
	0x05, 0x01,		// Usage Page (Generic Desktop)
	0x09, 0x05,		// Usage (Gamepad)
	0xA1, 0x01,		// Collection (Application)
	0x85, 0x01,		//   Report ID (1)
	0x06, 0x00, 0xFF,	//   Usage Page (Vendor Defined)
	0x09, 0x01,		//   Usage (Vendor 1) -- plug status + charge percent
	0x15, 0x00,		//   Logical Minimum (0)
	0x25, 0xFF,		//   Logical Maximum (255)
	0x75, 0x08,		//   Report Size (8)
	0x95, 0x02,		//   Report Count (2)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x05, 0x09,		//   Usage Page (Button)
	0x19, 0x01,		//   Usage Minimum (Button 1)
	0x29, 0x20,		//   Usage Maximum (Button 32)
	0x15, 0x00,		//   Logical Minimum (0)
	0x25, 0x01,		//   Logical Maximum (1)
	0x75, 0x01,		//   Report Size (1)
	0x95, 0x20,		//   Report Count (32)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x05, 0x01,		//   Usage Page (Generic Desktop)
	0x09, 0x30,		//   Usage (X)   left stick X
	0x09, 0x31,		//   Usage (Y)   left stick Y
	0x09, 0x32,		//   Usage (Z)   right stick X
	0x09, 0x35,		//   Usage (Rz)  right stick Y
	0x09, 0x33,		//   Usage (Rx)  left trigger
	0x09, 0x34,		//   Usage (Ry)  right trigger
	0x16, 0x00, 0x80,	//   Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,	//   Logical Maximum (32767)
	0x75, 0x10,		//   Report Size (16)
	0x95, 0x06,		//   Report Count (6)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x06, 0x00, 0xFF,	//   Usage Page (Vendor Defined)
	0x09, 0x20,		//   Usage (Vendor 0x20) -- IMU timestamp (us)
	0x15, 0x00,		//   Logical Minimum (0)
	0x26, 0xFF, 0xFF,	//   Logical Maximum (65535)
	0x75, 0x20,		//   Report Size (32)
	0x95, 0x01,		//   Report Count (1)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x09, 0x21,		//   Usage (Vendor 0x21) -- accel XYZ + gyro XYZ
	0x16, 0x00, 0x80,	//   Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,	//   Logical Maximum (32767)
	0x75, 0x10,		//   Report Size (16)
	0x95, 0x06,		//   Report Count (6)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x09, 0x22,		//   Usage (Vendor 0x22) -- touchpads + reserved
	0x15, 0x00,		//   Logical Minimum (0)
	0x26, 0xFF, 0x00,	//   Logical Maximum (255)
	0x75, 0x08,		//   Report Size (8)
	0x95, 0x1D,		//   Report Count (29)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x85, 0x02,		//   Report ID (2) -- command replies
	0x09, 0x23,		//   Usage (Vendor 0x23)
	0x15, 0x00,		//   Logical Minimum (0)
	0x26, 0xFF, 0x00,	//   Logical Maximum (255)
	0x75, 0x08,		//   Report Size (8)
	0x95, 0x3F,		//   Report Count (63)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x85, 0x03,		//   Report ID (3) -- host commands
	0x09, 0x24,		//   Usage (Vendor 0x24)
	0x15, 0x00,		//   Logical Minimum (0)
	0x26, 0xFF, 0x00,	//   Logical Maximum (255)
	0x75, 0x08,		//   Report Size (8)
	0x95, 0x2F,		//   Report Count (47)
	0x91, 0x02,		//   Output (Data,Var,Abs)
	0xC0			// End Collection
	// clang-format on
};

#define SIN_RID_INPUT 0x01
#define SIN_RID_CMDIN 0x02
#define SIN_RID_CMDOUT 0x03
#define SIN_REPLEN 63 // payload after the report id (64-byte reports)

#define SIN_CMD_HAPTIC 0x01
#define SIN_CMD_FEATURES 0x02
#define SIN_CMD_PLAYERLED 0x03
#define SIN_CMD_JOYSTICKRGB 0x04

// SDL_GamepadType / SInput face-layout hints reported in the features reply.
#define SIN_GAMEPAD_TYPE_STEAM 12
#define SIN_FACE_STYLE_ABXY 1

// SC2 accelerometer is +/-2g full scale (16384 counts/g -- see mode_switch_pro.cpp). The gyro full scale is
// not reverse-engineered; declare the SInput default so SDL's rad/s conversion is in the right ballpark.
#define SIN_ACCEL_G_RANGE 2
#define SIN_GYRO_DPS_RANGE 2000

// plug_status wire values (SDL reads 1..4): 1 = no battery, 2 = charging, 3 = charged, 4 = on battery.
#define SIN_PLUG_CHARGING 2
#define SIN_PLUG_CHARGED 3
#define SIN_PLUG_ON_BATTERY 4

static Adafruit_USBD_HID g_sin[NSLOT];
static unsigned long g_sinLastMs[NSLOT] = { 0 };

// Set by the host's FEATURES command in the usbd callback, answered from task() in loop context (the reply is
// an INPUT report, and every device->host send in this firmware goes through the usb_tx queue).
static volatile uint8_t g_sinFeatReq[NSLOT] = { 0 };

// Per-slot serial the host shows for the device (SDL formats it as a MAC). Derived from the board's FICR-based
// unit id so it is stable across reboots, with the slot in the last byte so four controllers stay distinct.
static uint8_t g_sinMac[NSLOT][6];
static bool g_sinMacInit = false;
static void initSinMacs()
{
	if (g_sinMacInit)
		return;
	for (uint8_t s = 0; s < NSLOT; s++) {
		// locally-administered, unicast OUI
		g_sinMac[s][0] = 0x02;
		g_sinMac[s][1] = 'O';
		g_sinMac[s][2] = 'P';
		g_sinMac[s][3] = (uint8_t)g_unit[3];
		g_sinMac[s][4] = (uint8_t)g_unit[4];
		g_sinMac[s][5] = s;
	}
	g_sinMacInit = true;
}

static inline int16_t sinInv(int16_t v)
{
	return (int16_t)(v == INT16_MIN ? INT16_MAX : -v);
}

// SInput triggers are full-range signed axes: released = INT16_MIN, fully pulled = INT16_MAX.
static inline int16_t sinTrigger(uint8_t v)
{
	return (int16_t)(-32768 + (int32_t)v * 257);
}

// Static capability + identity block SDL reads once at open (command 0x02). Layout: payload[0] = command id,
// then the reference lib's out[2..] shifted down by the report id TinyUSB writes for us.
static void sinBuildFeatures(uint8_t slot, uint8_t out[SIN_REPLEN])
{
	memset(out, 0, SIN_REPLEN);
	out[0] = SIN_CMD_FEATURES;
	out[1] = 0x01; // protocol version 0x0001
	// flags 1: rumble, accel, gyro, both sticks, both analog triggers (no player LEDs)
	out[3] = 0x01 | 0x04 | 0x08 | 0x10 | 0x20 | 0x40 | 0x80;
	out[4] = 0x01; // flags 2: touchpads supported
	out[5] = SIN_GAMEPAD_TYPE_STEAM;
	out[6] = (uint8_t)(SIN_FACE_STYLE_ABXY << 5);
	// polling interval: our host stream cadence, what SDL uses as the sensor rate
	out[7] = (uint8_t)((USB_STREAM_MS * 1000u) & 0xFF);
	out[8] = (uint8_t)((USB_STREAM_MS * 1000u) >> 8);
	out[9] = (uint8_t)(SIN_ACCEL_G_RANGE & 0xFF);
	out[10] = (uint8_t)(SIN_ACCEL_G_RANGE >> 8);
	out[11] = (uint8_t)(SIN_GYRO_DPS_RANGE & 0xFF);
	out[12] = (uint8_t)(SIN_GYRO_DPS_RANGE >> 8);
	// Which of the 32 button bits actually exist -- SDL derives its mapping from these. ABXY + D-pad; stick
	// clicks, bumpers, digital triggers, upper grips (L4/R4); start/select/guide/share, lower grips (L5/R5),
	// both touchpad clicks. No power or misc buttons.
	out[13] = 0xFF;
	out[14] = 0xFF;
	out[15] = 0xFF;
	out[16] = 0x00;
	out[17] = 2; // two touchpads
	out[18] = 1; // one finger each
	// Key the serial off the BOND slot, not the USB slot: the USB prefix is packed by connection order, so a
	// USB-slot serial would follow plug order instead of staying with the controller across sessions.
	int bond = (slot < NSLOT) ? g_usbToBond[slot] : -1;
	memcpy(out + 19, g_sinMac[bond >= 0 ? (uint8_t)bond : 0], 6);
}

static void sinBuild(uint8_t bond, uint8_t out[SIN_REPLEN])
{
	const PuckInput &in = g_in[bond];
	uint32_t b = in.buttons;
	memset(out, 0, SIN_REPLEN);
	// EChargeState (0x43 body[0]): 2 = charging, 4 = charge complete; anything else reads as discharging.
	uint8_t st = g_batteryState[bond];
	out[0] = (st == 2) ?
			 SIN_PLUG_CHARGING :
			 ((st == 4) ? SIN_PLUG_CHARGED : SIN_PLUG_ON_BATTERY);
	out[1] = g_battery[bond] > 100 ? 100 : g_battery[bond];
	// Face-button bit order follows SDL's SINPUT_BUTTONMASK_* (east, south, north, west) -- the reference
	// lib's struct declares south, east, west, north, and the two disagree. SDL is the consumer, and its
	// order is the one that reads correctly on hardware (the other way round swaps A/B and X/Y).
	out[2] = (uint8_t)(((b & TB_B) ? 0x01 : 0) | // east
			   ((b & TB_A) ? 0x02 : 0) | // south
			   ((b & TB_Y) ? 0x04 : 0) | // north
			   ((b & TB_X) ? 0x08 : 0) | // west
			   ((b & TB_DUP) ? 0x10 : 0) |
			   ((b & TB_DDN) ? 0x20 : 0) |
			   ((b & TB_DLF) ? 0x40 : 0) |
			   ((b & TB_DRT) ? 0x80 : 0));
	out[3] = (uint8_t)(((b & TB_L3) ? 0x01 : 0) | ((b & TB_R3) ? 0x02 : 0) |
			   ((b & TB_LB) ? 0x04 : 0) | ((b & TB_RB) ? 0x08 : 0) |
			   ((b & TB_L2) ? 0x10 : 0) | // digital trigger
			   ((b & TB_R2) ? 0x20 : 0) |
			   ((b & TB_L4) ? 0x40 : 0) | // upper grips
			   ((b & TB_R4) ? 0x80 : 0));
	// TB_VIEW / TB_MENU are named backwards relative to the physical buttons (see triton.h).
	out[4] = (uint8_t)(((b & TB_VIEW) ? 0x01 : 0) | // start
			   ((b & TB_MENU) ? 0x02 : 0) | // select
			   ((b & TB_STEAM) ? 0x04 : 0) | // guide
			   ((b & TB_QAM) ? 0x08 : 0) | // share/capture
			   ((b & TB_L5) ? 0x10 : 0) | // lower grips
			   ((b & TB_R5) ? 0x20 : 0) |
			   ((b & TB_LPADC) ? 0x40 : 0) |
			   ((b & TB_RPADC) ? 0x80 : 0));
	le16(out + 6, in.lx);
	le16(out + 8, sinInv(in.ly)); // SDL axes are down-positive
	le16(out + 10, in.rx);
	le16(out + 12, sinInv(in.ry));
	le16(out + 14, sinTrigger(in.lt));
	le16(out + 16, sinTrigger(in.rt));
	uint32_t us = micros();
	out[18] = (uint8_t)(us & 0xFF);
	out[19] = (uint8_t)((us >> 8) & 0xFF);
	out[20] = (uint8_t)((us >> 16) & 0xFF);
	out[21] = (uint8_t)((us >> 24) & 0xFF);
	// The SInput wire IMU frame is (+x left, +y away from the user, +z up) -- SDL's driver rotates it into its
	// own (+x right, +y up, +z toward the user) convention. That is the SC2's own frame (derived from the
	// hardware-verified Switch Pro mapping in mode_switch_pro.cpp: gz = yaw about the up axis, gy = roll about
	// the forward axis, -gx = pitch about the right axis), so both vectors pass through raw -- and, crucially,
	// with the SAME permutation, which is what hosts that FUSE accel with gyro require.
	le16(out + 22, in.ax);
	le16(out + 24, in.ay);
	le16(out + 26, in.az);
	le16(out + 28, in.gx);
	le16(out + 30, in.gy);
	le16(out + 32, in.gz);
	// Touchpads: full-scale signed coordinates (SDL normalises x/65536 + 0.5), pressure 0 = no contact.
	bool lTouch = (b & TB_LPADT) || (b & TB_LPADC);
	bool rTouch = (b & TB_RPADT) || (b & TB_RPADC);
	if (lTouch) {
		le16(out + 34, in.lpx);
		le16(out + 36, sinInv(in.lpy));
		le16(out + 38, INT16_MAX);
	}
	if (rTouch) {
		le16(out + 40, in.rpx);
		le16(out + 42, sinInv(in.rpy));
		le16(out + 44, INT16_MAX);
	}
}

// Host command channel (output report 0x03), usbd task.
static void sinSetCommon(uint8_t slot, uint8_t rid, hid_report_type_t type,
			 uint8_t const *b, uint16_t n)
{
	if (type != HID_REPORT_TYPE_OUTPUT || n < 1)
		return;
	const uint8_t *p;
	uint16_t pn;
	if (rid == 0) { // OUT endpoint: report id leads the buffer
		if (b[0] != SIN_RID_CMDOUT || n < 2)
			return;
		p = b + 1;
		pn = (uint16_t)(n - 1);
	} else if (rid == SIN_RID_CMDOUT) {
		p = b;
		pn = n;
	} else {
		return;
	}
	if (p[0] == SIN_CMD_FEATURES) {
		if (slot < NSLOT)
			g_sinFeatReq[slot] = 1;
		return;
	}
	if (p[0] != SIN_CMD_HAPTIC || pn < 2)
		return; // player LED / RGB: not advertised
	// `slot` is the USB slot the report arrived on -> route rumble to its mapped bond slot.
	int bond = (slot < NSLOT) ? g_usbToBond[slot] : -1;
	if (bond < 0)
		return;
	uint16_t lo = 0, hi = 0;
	if (p[1] == 0x01 && pn >= 18) {
		// stereo frequency/amplitude pairs: we can only render amplitude, so take the louder band per side
		uint16_t l1 = (uint16_t)(p[4] | (p[5] << 8));
		uint16_t l2 = (uint16_t)(p[8] | (p[9] << 8));
		uint16_t r1 = (uint16_t)(p[12] | (p[13] << 8));
		uint16_t r2 = (uint16_t)(p[16] | (p[17] << 8));
		lo = l1 > l2 ? l1 : l2;
		hi = r1 > r2 ? r1 : r2;
	} else if (p[1] == 0x02 && pn >= 6) {
		lo = (uint16_t)p[2] * 257u; // ERM model: left/right amplitude
		hi = (uint16_t)p[4] * 257u;
	} else {
		return;
	}
	hapticSteamRumble(lo, hi,
			  (uint8_t)bond); // SInput: left = low, right = high
}
// Per-slot trampolines: the Adafruit HID class shares one callback pair across instances and doesn't pass the
// interface index, so close over the slot (same pattern as puck_hid.cpp / mode_ps5.cpp).
#define SINPUTCB(N)                                           \
	static void sinSet##N(uint8_t r, hid_report_type_t t, \
			      uint8_t const *b, uint16_t n)   \
	{                                                     \
		sinSetCommon(N, r, t, b, n);                  \
	}
// clang-format off
SINPUTCB(0)
SINPUTCB(1)
SINPUTCB(2)
SINPUTCB(3)
// clang-format on
typedef void (*sin_setcb_t)(uint8_t, hid_report_type_t, uint8_t const *,
			    uint16_t);
static sin_setcb_t const SIN_SETCB[NSLOT] = { sinSet0, sinSet1, sinSet2,
					      sinSet3 };

// Dynamic-mount mode: begin() is unused (setup() calls beginPool()+usbReenumerate instead).
void SInputController::begin()
{
}
uint8_t SInputController::maxSlots() const
{
	uint8_t cap = (uint8_t)(CFG_TUD_HID - 1); // wake mouse holds HID 0
	return cap < NSLOT ? cap : (uint8_t)NSLOT;
}
void SInputController::usbIdentity()
{
	// The generic SInput device ID SDL's SInput driver matches on (2E8A = Raspberry Pi, the VID Hand Held
	// Legend register their gamepads under; 10C6 = generic SInput/HOJA gamepad).
	USBDevice.setID(0x2E8A, 0x10C6);
	USBDevice.setDeviceVersion(0x0210);
	USBDevice.setManufacturerDescriptor("OpenPuck");
	USBDevice.setProductDescriptor("OpenPuck SInput");
}
void SInputController::beginPool()
{
	initSinMacs();
	uint8_t pool = maxSlots();
	for (uint8_t s = 0; s < pool; s++) {
		g_sin[s].enableOutEndpoint(true);
		g_sin[s].setReportCallback(NULL, SIN_SETCB[s]);
		g_sin[s].setReportDescriptor(SIN_HID_DESC, sizeof SIN_HID_DESC);
		g_sin[s].setPollInterval(1);
		g_sin[s].begin();
	}
}
void SInputController::mountSlots(uint8_t k)
{
	for (uint8_t u = 0; u < k; u++)
		USBDevice.addInterface(g_sin[u]);
}
void SInputController::task()
{
	for (uint8_t u = 0; u < g_usbMountCount; u++) {
		if (!g_sin[u].ready())
			continue;
		uint8_t p[SIN_REPLEN];
		// The features reply gates SDL's whole init (it gives up after ~100 ms), so it jumps the stream gate.
		if (g_sinFeatReq[u]) {
			g_sinFeatReq[u] = 0;
			sinBuildFeatures(u, p);
			usbTxHid(&g_sin[u], SIN_RID_CMDIN, p, sizeof p);
		}
		if (millis() - g_sinLastMs[u] < USB_STREAM_MS)
			continue;
		int bond = g_usbToBond[u];
		if (bond < 0)
			continue;
		g_sinLastMs[u] = millis();
		sinBuild((uint8_t)bond, p);
		usbTxHid(&g_sin[u], SIN_RID_INPUT, p, sizeof p);
	}
}
