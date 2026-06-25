#include "mode_ps3.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_mount.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

Ps3Controller g_ps3Ctl;

// DS3 HID report descriptor. Input report 0x01 = 48 bytes (48 payload after report ID).
// Layout mirrors the real DualShock 3 byte-for-byte so the PS3 console and hid-sony/
// hid-playstation drivers parse it correctly:
//   [0-1]   reserved (constant)
//   [2]     buttons byte 1: Select L3 R3 Start Up Right Down Left
//   [3]     buttons byte 2: L2 R2 L1 R1 Triangle Circle Cross Square
//   [4]     buttons byte 3: PS (bit 0), padding (bits 1-7)
//   [5]     reserved (constant)
//   [6-9]   LX LY RX RY analog sticks (0x80 center)
//   [10-13] reserved
//   [14-25] analog button pressures (Up Right Down Left L2 R2 L1 R1 Tri Cir X Sq)
//   [26-27] reserved
//   [28]    status byte (0x02 = USB connected)
//   [29-39] reserved
//   [40-41] AccX big-endian uint16 (center 512)
//   [42-43] AccY big-endian uint16 (center 512)
//   [44-45] AccZ big-endian uint16 (center 512)
//   [46-47] GyroZ big-endian uint16 (center 512, yaw axis)
static const uint8_t DS3_HID_DESC[] = {
	0x05, 0x01, // Usage Page (Generic Desktop)
	0x09, 0x04, // Usage (Joystick)
	0xA1, 0x01, // Collection (Application)
	0xA1, 0x02, //   Collection (Logical)
	0x85, 0x01, //   Report ID (1)

	// bytes 0-1: 2 constant (reserved) bytes
	0x75, 0x08, //   Report Size (8)
	0x95, 0x02, //   Report Count (2)
	0x15, 0x00, //   Logical Min (0)
	0x26, 0xFF, 0x00, //   Logical Max (255)
	0x81, 0x03, //   Input (Const, Var, Abs)

	// bytes 2-4 bits 0-18: 19 digital buttons
	0x75, 0x01, //   Report Size (1)
	0x95, 0x13, //   Report Count (19)
	0x15, 0x00, //   Logical Min (0)
	0x25, 0x01, //   Logical Max (1)
	0x35, 0x00, //   Physical Min (0)
	0x45, 0x01, //   Physical Max (1)
	0x05, 0x09, //   Usage Page (Button)
	0x19, 0x01, //   Usage Min (Button 1)
	0x29, 0x13, //   Usage Max (Button 19)
	0x81, 0x02, //   Input (Data, Var, Abs)

	// byte 4 bits 3-7: 5 padding bits (complete the byte started by buttons 17-19)
	0x75, 0x01, //   Report Size (1)
	0x95, 0x05, //   Report Count (5)
	0x81, 0x03, //   Input (Const, Var, Abs)

	// byte 5: 1 reserved constant byte
	0x75, 0x08, //   Report Size (8)
	0x95, 0x01, //   Report Count (1)
	0x81, 0x03, //   Input (Const, Var, Abs)

	// bytes 6-9: LX LY RX RY (4 analog stick axes)
	0x75, 0x08, //   Report Size (8)
	0x95, 0x04, //   Report Count (4)
	0x15, 0x00, //   Logical Min (0)
	0x26, 0xFF, 0x00, //   Logical Max (255)
	0x05, 0x01, //   Usage Page (Generic Desktop)
	0x09, 0x01, //   Usage (Pointer)
	0xA1, 0x00, //   Collection (Physical)
	0x09, 0x30, //     Usage (X)
	0x09, 0x31, //     Usage (Y)
	0x09, 0x32, //     Usage (Z)
	0x09, 0x35, //     Usage (Rz)
	0x81, 0x02, //     Input (Data, Var, Abs)
	0xC0, //   End Collection (Physical)

	// bytes 10-47: 38 bytes (reserved, pressures, status, IMU)
	0x75, 0x08, //   Report Size (8)
	0x95, 0x26, //   Report Count (38)
	0x15, 0x00, //   Logical Min (0)
	0x26, 0xFF, 0x00, //   Logical Max (255)
	0x09, 0x01, //   Usage (Pointer)
	0x81, 0x02, //   Input (Data, Var, Abs)

	// output report 0x01: 48 bytes (rumble + LED control from PS3)
	0x75, 0x08, //   Report Size (8)
	0x95, 0x30, //   Report Count (48)
	0x15, 0x00, //   Logical Min (0)
	0x26, 0xFF, 0x00, //   Logical Max (255)
	0x09, 0x01, //   Usage (Pointer)
	0x91, 0x02, //   Output (Data, Var, Abs)

	0xC0, // End Collection (Logical)
	0xC0, // End Collection (Application)
};

// One HID slot -- PS3 console expects one gamepad per USB port.
static Adafruit_USBD_HID g_ds3;
static unsigned long g_ds3LastMs = 0;

// OUI 001BDC is Sony's; 0x4F55 is the same arbitrary device suffix used by the
// DS4 pool in mode_hidgyro.cpp. Last byte 0x70 is fixed (only one slot).
static const uint8_t DS3_MAC_BASE[5] = { 0x00, 0x1B, 0xDC, 0x4F, 0x55 };
static uint8_t g_ds3Mac[6];
static bool g_ds3MacInit = false;
static void initDs3Mac()
{
	if (g_ds3MacInit)
		return;
	memcpy(g_ds3Mac, DS3_MAC_BASE, 5);
	g_ds3Mac[5] = 0x70;
	g_ds3MacInit = true;
}

// Map signed SC2 int16 IMU value to DS3 unsigned big-endian format (center 512).
// >>6 maps the full int16 range (±32768) to ±512 around center, filling the
// DS3's useful 0-1023 window. Increase the shift to narrow the live range
// (less sensitive), decrease it to widen (more sensitive, clips sooner).
// The PS3 OS applies its own calibration on top, so exact scaling is not critical.
static uint16_t imuToDs3(int16_t v)
{
	int32_t out = 512 + ((int32_t)v >> 6);
	if (out < 0)
		out = 0;
	if (out > 1023)
		out = 1023;
	return (uint16_t)out;
}

// GET_FEATURE handler. The PS3 reads 0xF2 (device info/MAC) and 0xF5 (connection
// status) during the controller-enable handshake. TinyUSB strips the report ID and
// passes us the payload buffer; we return the number of payload bytes to send.
static uint16_t ds3GetReport(uint8_t rid, hid_report_type_t type, uint8_t *buf,
			     uint16_t reqlen)
{
	if (type != HID_REPORT_TYPE_FEATURE || !buf || reqlen == 0)
		return 0;
	memset(buf, 0, reqlen);
	switch (rid) {
	// Device info / Bluetooth MAC (17 bytes total = 1 report ID + 16 payload).
	// The PS3 reads this to identify and pair the controller via Bluetooth.
	// payload[0] = 0x00 (status), payload[1..6] = controller BT MAC address.
	case 0xF2:
		if (reqlen < 16)
			return 0;
		memcpy(buf + 1, g_ds3Mac, 6);
		return 16;
	// Connection status (8 bytes total = 1 report ID + 7 payload).
	// payload[0] = 0x10 signals USB-connected state to the PS3.
	case 0xF5:
		if (reqlen < 7)
			return 0;
		buf[0] = 0x10;
		return 7;
	default:
		return 0;
	}
}

// SET_REPORT handler. Receives rumble + LED output report (id 0x01) from the PS3
// and feature 0xF4 (enable handshake). DS3 output layout (bytes after report ID):
//   [1]  right (HFR) motor duration  (0xFF = run indefinitely)
//   [2]  right motor on/off          (>0 means on; binary -- HFR has no speed control)
//   [3]  left  (LFR) motor duration  (0xFF = run indefinitely)
//   [4]  left  motor power           (0x00-0xFF)
static void ds3SetReport(uint8_t rid, hid_report_type_t type, uint8_t const *b,
			 uint16_t n)
{
	// Feature 0xF4 is the PS3's "enable SIXAXIS" command; accept silently.
	if (type == HID_REPORT_TYPE_FEATURE)
		return;
	if (type != HID_REPORT_TYPE_OUTPUT || n < 5)
		return;
	// For output, TinyUSB may deliver the report ID in rid or in b[0].
	const uint8_t *p;
	uint16_t pn;
	if (rid == 0) {
		if (b[0] != 0x01 || n < 6)
			return;
		p = b + 1;
		pn = (uint16_t)(n - 1);
	} else {
		if (rid != 0x01)
			return;
		p = b;
		pn = n;
	}
	if (pn < 5)
		return;
	int bond = (g_usbMountCount > 0) ? g_usbToBond[0] : -1;
	if (bond < 0)
		return;
	// p[4] = left (LFR) motor power, p[2] > 0 means right (HFR) motor on.
	// 257 = 65535/255: scales an 8-bit motor value to the 16-bit haptic range.
	hapticSteamRumble((uint16_t)p[4] * 257u,
			  (uint16_t)(p[2] > 0 ? 255 : 0) * 257u, (uint8_t)bond);
}

static void ds3Build(uint8_t slot, uint8_t out[48])
{
	uint32_t b = psButtonsFromSteam(g_in[slot].buttons);
	bool l2 = (g_in[slot].lt > SW_TRIG_ON) || (b & TB_L2);
	bool r2 = (g_in[slot].rt > SW_TRIG_ON) || (b & TB_R2);
	memset(out, 0, 48);

	// bytes 0-1: reserved (stay zero from memset)

	// byte 2: Select L3 R3 Start Up Right Down Left
	out[2] = ((b & TB_VIEW) ? 0x01 : 0) | ((b & TB_L3) ? 0x02 : 0) |
		 ((b & TB_R3) ? 0x04 : 0) | ((b & TB_MENU) ? 0x08 : 0) |
		 ((b & TB_DUP) ? 0x10 : 0) | ((b & TB_DRT) ? 0x20 : 0) |
		 ((b & TB_DDN) ? 0x40 : 0) | ((b & TB_DLF) ? 0x80 : 0);

	// byte 3: L2 R2 L1 R1 Triangle Circle Cross Square
	// g_abSwap swaps A<->B and X<->Y (Nintendo vs PlayStation face layout).
	if (g_abSwap) {
		out[3] = (l2 ? 0x01 : 0) | (r2 ? 0x02 : 0) |
			 ((b & TB_LB) ? 0x04 : 0) | ((b & TB_RB) ? 0x08 : 0) |
			 ((b & TB_A) ? 0x10 : 0) | // Triangle
			 ((b & TB_B) ? 0x20 : 0) | // Circle
			 ((b & TB_X) ? 0x40 : 0) | // Cross
			 ((b & TB_Y) ? 0x80 : 0); // Square
	} else {
		out[3] = (l2 ? 0x01 : 0) | (r2 ? 0x02 : 0) |
			 ((b & TB_LB) ? 0x04 : 0) | ((b & TB_RB) ? 0x08 : 0) |
			 ((b & TB_Y) ? 0x10 : 0) | // Triangle
			 ((b & TB_B) ? 0x20 : 0) | // Circle
			 ((b & TB_A) ? 0x40 : 0) | // Cross
			 ((b & TB_X) ? 0x80 : 0); // Square
	}

	// byte 4: PS button (bit 0); bits 1-7 are the 5-bit HID padding (stay zero)
	out[4] = (b & TB_STEAM) ? 0x01 : 0;

	// byte 5: reserved (stays zero)

	// bytes 6-9: analog sticks
	out[6] = swStick(g_in[slot].lx, false);
	out[7] = swStick(g_in[slot].ly, true);
	out[8] = swStick(g_in[slot].rx, false);
	out[9] = swStick(g_in[slot].ry, true);

	// bytes 10-13: reserved

	// bytes 14-25: analog button pressures (0xFF when pressed, 0x00 released)
	out[14] = (b & TB_DUP) ? 0xFF : 0;
	out[15] = (b & TB_DRT) ? 0xFF : 0;
	out[16] = (b & TB_DDN) ? 0xFF : 0;
	out[17] = (b & TB_DLF) ? 0xFF : 0;
	out[18] = g_in[slot].lt; // L2 analog
	out[19] = g_in[slot].rt; // R2 analog
	out[20] = (b & TB_LB) ? 0xFF : 0; // L1
	out[21] = (b & TB_RB) ? 0xFF : 0; // R1
	if (g_abSwap) {
		out[22] = (b & TB_A) ? 0xFF : 0; // Triangle
		out[23] = (b & TB_B) ? 0xFF : 0; // Circle
		out[24] = (b & TB_X) ? 0xFF : 0; // Cross
		out[25] = (b & TB_Y) ? 0xFF : 0; // Square
	} else {
		out[22] = (b & TB_Y) ? 0xFF : 0; // Triangle
		out[23] = (b & TB_B) ? 0xFF : 0; // Circle
		out[24] = (b & TB_A) ? 0xFF : 0; // Cross
		out[25] = (b & TB_X) ? 0xFF : 0; // Square
	}

	// bytes 26-27: reserved
	// byte 28: status -- 0x02 = USB connected
	out[28] = 0x02;
	// bytes 29-39: reserved

	// bytes 40-47: IMU. SC2 int16 (center 0) -> DS3 big-endian uint16 (center 512).
	// AccX/Y/Z are the three accelerometer axes. GyroZ is the single yaw axis of
	// the DS3 (the SIXAXIS/DS3 has only one gyro axis, unlike the DualSense/DS4).
	uint16_t accX = imuToDs3(g_in[slot].ax);
	uint16_t accY = imuToDs3(g_in[slot].ay);
	uint16_t accZ = imuToDs3(g_in[slot].az);
	uint16_t gyrZ = imuToDs3(g_in[slot].gz); // yaw axis
	out[40] = (uint8_t)(accX >> 8);
	out[41] = (uint8_t)(accX & 0xFF);
	out[42] = (uint8_t)(accY >> 8);
	out[43] = (uint8_t)(accY & 0xFF);
	out[44] = (uint8_t)(accZ >> 8);
	out[45] = (uint8_t)(accZ & 0xFF);
	out[46] = (uint8_t)(gyrZ >> 8);
	out[47] = (uint8_t)(gyrZ & 0xFF);
}

void Ps3Controller::begin()
{
}

// maxSlots = 1: the PS3 console expects one gamepad per USB port. Even on PC, a single
// DS3 interface is the standard configuration for this personality.
uint8_t Ps3Controller::maxSlots() const
{
	return 1;
}

void Ps3Controller::usbIdentity()
{
	// DualShock 3 (PID 0x0289 = with vibration, vs SIXAXIS 0x0268 without).
	// Using DS3 so the PS3 sends rumble output reports.
	USBDevice.setID(0x054C, 0x0289);
	USBDevice.setDeviceVersion(0x0100);
	USBDevice.setManufacturerDescriptor("Sony Computer Entertainment");
	USBDevice.setProductDescriptor("PLAYSTATION(R)3 Controller");
}

void Ps3Controller::beginPool()
{
	initDs3Mac();
	g_ds3.enableOutEndpoint(true);
	g_ds3.setReportCallback(ds3GetReport, ds3SetReport);
	g_ds3.setReportDescriptor(DS3_HID_DESC, sizeof DS3_HID_DESC);
	g_ds3.setPollInterval(1);
	g_ds3.begin();
}

void Ps3Controller::mountSlots(uint8_t k)
{
	if (k > 0)
		USBDevice.addInterface(g_ds3);
}

void Ps3Controller::task()
{
	if (g_usbMountCount < 1)
		return;
	if (!g_ds3.ready())
		return;
	if (millis() - g_ds3LastMs < USB_STREAM_MS)
		return;
	int bond = g_usbToBond[0];
	if (bond < 0)
		return;
	g_ds3LastMs = millis();
	uint8_t p[48];
	ds3Build((uint8_t)bond, p);
	g_ds3.sendReport(0x01, p, sizeof p);
}
