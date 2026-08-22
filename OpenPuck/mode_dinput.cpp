#include "mode_dinput.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "usb_mount.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

DInputController g_dinputCtl;

// Two top-level Application collections, one HID interface. DirectInput can only surface 8 axes per device
// (DIJOYSTATE2: X/Y/Z/Rx/Ry/Rz + 2 sliders), and the SC2 has 13 analog inputs -- so the exotic ones live on a
// second collection, which Windows exposes as a second joystick (one HIDClass PDO per top-level collection):
//
//   report 1 "stick" device: X/Y = left stick, Rx/Ry = right stick, Z/Rz = LT/RT, hat = D-pad, buttons 1-26
//   report 2 "motion" device: X/Y = left pad, Rx/Ry = right pad, Z/Rz/Slider = gyro X/Y/Z, buttons 1-4 = pad
//                             click/touch
//
// Both reports are 15 payload bytes. Nothing here needs an OUT endpoint: DirectInput force feedback is the PID
// class (not the vendor rumble reports our other modes decode), so this mode is input-only.
//
// If a host ever refuses to split the collections, device #1 still works and the fallback is to mount the two
// collections as two separate HID interfaces per slot (halves the controller count: 2 interfaces per slot
// against the CFG_TUD_HID budget).
static const uint8_t DI_HID_DESC[] = {
	// clang-format off
	0x05, 0x01,		// Usage Page (Generic Desktop)
	0x09, 0x04,		// Usage (Joystick)
	0xA1, 0x01,		// Collection (Application)
	0x85, 0x01,		//   Report ID (1)
	0x09, 0x30,		//   Usage (X)   left stick X
	0x09, 0x31,		//   Usage (Y)   left stick Y
	0x09, 0x33,		//   Usage (Rx)  right stick X
	0x09, 0x34,		//   Usage (Ry)  right stick Y
	0x16, 0x00, 0x80,	//   Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,	//   Logical Maximum (32767)
	0x75, 0x10,		//   Report Size (16)
	0x95, 0x04,		//   Report Count (4)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x09, 0x32,		//   Usage (Z)   left trigger
	0x09, 0x35,		//   Usage (Rz)  right trigger
	0x15, 0x00,		//   Logical Minimum (0)
	0x26, 0xFF, 0x00,	//   Logical Maximum (255)
	0x75, 0x08,		//   Report Size (8)
	0x95, 0x02,		//   Report Count (2)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x09, 0x39,		//   Usage (Hat switch)
	0x15, 0x00,		//   Logical Minimum (0)
	0x25, 0x07,		//   Logical Maximum (7)
	0x35, 0x00,		//   Physical Minimum (0)
	0x46, 0x3B, 0x01,	//   Physical Maximum (315)
	0x65, 0x14,		//   Unit (English rotation, degrees)
	0x75, 0x04,		//   Report Size (4)
	0x95, 0x01,		//   Report Count (1)
	// Null State: a 4-bit value outside 0..7 (we send 8) reads as "hat centred"
	0x81, 0x42,		//   Input (Data,Var,Abs,Null State)
	0x65, 0x00,		//   Unit (None)
	0x75, 0x04,		//   Report Size (4)
	0x95, 0x01,		//   Report Count (1)
	0x81, 0x03,		//   Input (Const) -- pad the hat byte
	0x05, 0x09,		//   Usage Page (Button)
	0x19, 0x01,		//   Usage Minimum (Button 1)
	0x29, 0x1A,		//   Usage Maximum (Button 26)
	0x15, 0x00,		//   Logical Minimum (0)
	0x25, 0x01,		//   Logical Maximum (1)
	0x75, 0x01,		//   Report Size (1)
	0x95, 0x1A,		//   Report Count (26)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x75, 0x01,		//   Report Size (1)
	0x95, 0x06,		//   Report Count (6)
	0x81, 0x03,		//   Input (Const) -- pad to 4 button bytes
	0xC0,			// End Collection

	0x05, 0x01,		// Usage Page (Generic Desktop)
	0x09, 0x04,		// Usage (Joystick)
	0xA1, 0x01,		// Collection (Application)
	0x85, 0x02,		//   Report ID (2)
	0x09, 0x30,		//   Usage (X)      left pad X
	0x09, 0x31,		//   Usage (Y)      left pad Y
	0x09, 0x33,		//   Usage (Rx)     right pad X
	0x09, 0x34,		//   Usage (Ry)     right pad Y
	0x09, 0x32,		//   Usage (Z)      gyro X
	0x09, 0x35,		//   Usage (Rz)     gyro Y
	0x09, 0x36,		//   Usage (Slider) gyro Z
	0x16, 0x00, 0x80,	//   Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,	//   Logical Maximum (32767)
	0x75, 0x10,		//   Report Size (16)
	0x95, 0x07,		//   Report Count (7)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x05, 0x09,		//   Usage Page (Button)
	0x19, 0x01,		//   Usage Minimum (Button 1)
	0x29, 0x04,		//   Usage Maximum (Button 4)
	0x15, 0x00,		//   Logical Minimum (0)
	0x25, 0x01,		//   Logical Maximum (1)
	0x75, 0x01,		//   Report Size (1)
	0x95, 0x04,		//   Report Count (4)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x75, 0x04,		//   Report Size (4)
	0x95, 0x01,		//   Report Count (1)
	0x81, 0x03,		//   Input (Const) -- pad to a whole byte
	0xC0			// End Collection
	// clang-format on
};

#define DI_RID_STICK 0x01
#define DI_RID_MOTION 0x02
#define DI_REPLEN 15

static Adafruit_USBD_HID g_di[NSLOT];
static unsigned long g_diLastMs[NSLOT] = { 0 };

// Latched trackpad axes. A pad only reports a position while it is touched, so a raw pass-through would snap
// the axis to centre the moment the finger lifts -- useless for the throttle/trim bindings sim users want. The
// axis therefore HOLDS the last touched position; a pad CLICK re-centres it (the deliberate reset gesture).
struct PadAxis {
	int16_t lx, ly, rx, ry;
};
static PadAxis g_padAxis[NSLOT];

// HID axes are down-positive; the SC2 sticks/pads are up-positive. Negating INT16_MIN overflows, so clamp.
static inline int16_t diInv(int16_t v)
{
	return (int16_t)(v == INT16_MIN ? INT16_MAX : -v);
}

// Buttons 1..26, raw SC2 bits -- no back-paddle/QAM remap: DirectInput bindings are done in the sim, and a
// paddle that duplicates a face button would just cost the user two bindable inputs.
static uint32_t diButtons(uint32_t b)
{
	uint32_t o = 0;
	if (b & TB_A)
		o |= 1u << 0;
	if (b & TB_B)
		o |= 1u << 1;
	if (b & TB_X)
		o |= 1u << 2;
	if (b & TB_Y)
		o |= 1u << 3;
	if (b & TB_LB)
		o |= 1u << 4;
	if (b & TB_RB)
		o |= 1u << 5;
	if (b & TB_L2)
		o |= 1u << 6;
	if (b & TB_R2)
		o |= 1u << 7;
	// button 9 = Start-side, button 10 = Select-side (TB_VIEW/TB_MENU are named backwards -- see triton.h)
	if (b & TB_VIEW)
		o |= 1u << 8;
	if (b & TB_MENU)
		o |= 1u << 9;
	if (b & TB_L3)
		o |= 1u << 10;
	if (b & TB_R3)
		o |= 1u << 11;
	if (b & TB_DUP)
		o |= 1u << 12;
	if (b & TB_DDN)
		o |= 1u << 13;
	if (b & TB_DLF)
		o |= 1u << 14;
	if (b & TB_DRT)
		o |= 1u << 15;
	if (b & TB_L4)
		o |= 1u << 16;
	if (b & TB_R4)
		o |= 1u << 17;
	if (b & TB_L5)
		o |= 1u << 18;
	if (b & TB_R5)
		o |= 1u << 19;
	if (b & TB_LPADC)
		o |= 1u << 20;
	if (b & TB_RPADC)
		o |= 1u << 21;
	if (b & TB_LPADT)
		o |= 1u << 22;
	if (b & TB_RPADT)
		o |= 1u << 23;
	if (b & TB_STEAM)
		o |= 1u << 24;
	if (b & TB_QAM)
		o |= 1u << 25;
	return o;
}

static void diBuildStick(uint8_t bond, uint8_t out[DI_REPLEN])
{
	const PuckInput &in = g_in[bond];
	memset(out, 0, DI_REPLEN);
	le16(out + 0, in.lx);
	le16(out + 2, diInv(in.ly));
	le16(out + 4, in.rx);
	le16(out + 6, diInv(in.ry));
	out[8] = in.lt;
	out[9] = in.rt;
	out[10] = psHatNibble(in.buttons);
	uint32_t btn = diButtons(in.buttons);
	out[11] = (uint8_t)(btn & 0xFF);
	out[12] = (uint8_t)((btn >> 8) & 0xFF);
	out[13] = (uint8_t)((btn >> 16) & 0xFF);
	out[14] = (uint8_t)((btn >> 24) & 0xFF);
}

static void diBuildMotion(uint8_t bond, uint8_t out[DI_REPLEN])
{
	const PuckInput &in = g_in[bond];
	PadAxis &p = g_padAxis[bond];
	if (in.buttons & TB_LPADC) {
		p.lx = 0;
		p.ly = 0;
	} else if (in.buttons & TB_LPADT) {
		p.lx = in.lpx;
		p.ly = in.lpy;
	}
	if (in.buttons & TB_RPADC) {
		p.rx = 0;
		p.ry = 0;
	} else if (in.buttons & TB_RPADT) {
		p.rx = in.rpx;
		p.ry = in.rpy;
	}
	memset(out, 0, DI_REPLEN);
	le16(out + 0, p.lx);
	le16(out + 2, diInv(p.ly));
	le16(out + 4, p.rx);
	le16(out + 6, diInv(p.ry));
	le16(out + 8, in.gx);
	le16(out + 10, in.gy);
	le16(out + 12, in.gz);
	out[14] = (uint8_t)(((in.buttons & TB_LPADC) ? 0x01 : 0) |
			    ((in.buttons & TB_RPADC) ? 0x02 : 0) |
			    ((in.buttons & TB_LPADT) ? 0x04 : 0) |
			    ((in.buttons & TB_RPADT) ? 0x08 : 0));
}

// Dynamic-mount mode: begin() is unused (setup() calls beginPool()+usbReenumerate instead).
void DInputController::begin()
{
}
uint8_t DInputController::maxSlots() const
{
	uint8_t cap = (uint8_t)(CFG_TUD_HID - 1); // wake mouse holds HID 0
	return cap < NSLOT ? cap : (uint8_t)NSLOT;
}
void DInputController::usbIdentity()
{
	// Deliberately NOT a cloned identity: nothing about DirectInput keys off VID/PID, and a generic ID keeps
	// Steam Input / the console whitelists from claiming the device out from under the sim.
	USBDevice.setID(0x1209, 0x4F50); // pid.codes community VID, "OP"
	USBDevice.setDeviceVersion(0x0100);
	USBDevice.setManufacturerDescriptor("OpenPuck");
	USBDevice.setProductDescriptor("OpenPuck Joystick");
}
void DInputController::beginPool()
{
	uint8_t pool = maxSlots();
	for (uint8_t s = 0; s < pool; s++) {
		g_di[s].setReportDescriptor(DI_HID_DESC, sizeof DI_HID_DESC);
		g_di[s].setPollInterval(1);
		g_di[s].begin();
	}
}
void DInputController::mountSlots(uint8_t k)
{
	for (uint8_t u = 0; u < k; u++)
		USBDevice.addInterface(g_di[u]);
}
void DInputController::task()
{
	for (uint8_t u = 0; u < g_usbMountCount; u++) {
		if (!g_di[u].ready())
			continue;
		if (millis() - g_diLastMs[u] < USB_STREAM_MS)
			continue;
		int bond = g_usbToBond[u];
		if (bond < 0)
			continue;
		g_diLastMs[u] = millis();
		uint8_t p[DI_REPLEN];
		diBuildStick((uint8_t)bond, p);
		usbTxHid(&g_di[u], DI_RID_STICK, p, sizeof p);
		diBuildMotion((uint8_t)bond, p);
		usbTxHid(&g_di[u], DI_RID_MOTION, p, sizeof p);
	}
}
