#include "mode_ps3.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

Ps3Controller g_ps3Ctl;

// ---------------------------------------------------------------------------------------------------------
// Genuine Sony Sixaxis / DualShock 3 (054C:0268) HID report descriptor -- 148 bytes, captured verbatim from
// real hardware (Nefarius "SIXAXIS native HID Report Descriptor"). The PS3 console does NOT parse this to
// drive input -- it recognises the pad by VID/PID and the control-transfer handshake below -- but a faithful
// descriptor (declaring input 0x01, output 0x01, and feature reports 0x01/0x02/0xEE/0xEF) keeps both the
// console and PC HID stacks happy. Do not "tidy" it: this is the real firmware's (famously odd) descriptor.
// ---------------------------------------------------------------------------------------------------------
static const uint8_t DS3_HID_DESC[148] = {
	0x05, 0x01, 0x09, 0x04, 0xA1, 0x01, 0xA1, 0x02, 0x85, 0x01, 0x75, 0x08,
	0x95, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x81, 0x03, 0x75, 0x01, 0x95,
	0x13, 0x15, 0x00, 0x25, 0x01, 0x35, 0x00, 0x45, 0x01, 0x05, 0x09, 0x19,
	0x01, 0x29, 0x13, 0x81, 0x02, 0x75, 0x01, 0x95, 0x0D, 0x06, 0x00, 0xFF,
	0x81, 0x03, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x05, 0x01, 0x09, 0x01, 0xA1,
	0x00, 0x75, 0x08, 0x95, 0x04, 0x35, 0x00, 0x46, 0xFF, 0x00, 0x09, 0x30,
	0x09, 0x31, 0x09, 0x32, 0x09, 0x35, 0x81, 0x02, 0xC0, 0x05, 0x01, 0x75,
	0x08, 0x95, 0x27, 0x09, 0x01, 0x81, 0x02, 0x75, 0x08, 0x95, 0x30, 0x09,
	0x01, 0x91, 0x02, 0x75, 0x08, 0x95, 0x30, 0x09, 0x01, 0xB1, 0x02, 0xC0,
	0xA1, 0x02, 0x85, 0x02, 0x75, 0x08, 0x95, 0x30, 0x09, 0x01, 0xB1, 0x02,
	0xC0, 0xA1, 0x02, 0x85, 0xEE, 0x75, 0x08, 0x95, 0x30, 0x09, 0x01, 0xB1,
	0x02, 0xC0, 0xA1, 0x02, 0x85, 0xEF, 0x75, 0x08, 0x95, 0x30, 0x09, 0x01,
	0xB1, 0x02, 0xC0, 0xC0
};

// ---------------------------------------------------------------------------------------------------------
// Magic GET_REPORT(Feature) responses. Lifted verbatim from GIMX-firmwares EMUPS3 (matlo) -- a LUFA-based DS3
// emulator proven to enumerate on a real PS3. GIMX sends each 64-byte array RAW (Endpoint_Write_Control_-
// Stream_LE), so the genuine wire byte 0 is the array's first byte -- which for a Sixaxis is NOT the report
// id (e.g. GET 0xF5 returns 01 00.., GET 0xEF returns 00 EF..). See emitFeature() for how we reproduce that
// despite TinyUSB force-prepending the report id. The PS3 enable handshake is:
//   GET 0x01 -> GET 0xF2 -> {SET 0xEF / GET 0xEF}x2 -> GET 0xF8 -> SET output 0x01 -> SET 0xF4(enable)
// then input report 0x01 streams. (GIMX does not even handle 0xF4 and streams unconditionally; so do we.)
// ---------------------------------------------------------------------------------------------------------
static const uint8_t report_01[64] = {
	0x00, 0x01, 0x04, 0x00, 0x07, 0x0c, 0x01, 0x02, 0x18, 0x18, 0x18,
	0x18, 0x09, 0x0a, 0x10, 0x11, 0x12, 0x13, 0x00, 0x00, 0x00, 0x00,
	0x04, 0x00, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x04, 0x04,
	0x04, 0x04, 0x00, 0x00, 0x04, 0x00, 0x01, 0x02, 0x07, 0x00, 0x17,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 0xF2 device info; bytes [4..9] = the controller's Bluetooth MAC (overlaid from g_ds3Mac at runtime).
static const uint8_t report_f2[64] = {
	0xf2, 0xff, 0xff, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00,
	0x03, 0x50, 0x81, 0xd8, 0x01, 0x8a, 0x13, 0x00, 0x00, 0x00, 0x00,
	0x04, 0x00, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x04, 0x04,
	0x04, 0x04, 0x00, 0x00, 0x04, 0x00, 0x01, 0x02, 0x07, 0x00, 0x17,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 0xF5 host-pairing info; bytes [2..7] = master (host) BT MAC, overlaid once learned from a SET 0xF5.
static const uint8_t report_f5[64] = {
	0x01, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xff, 0xf7, 0x00,
	0x03, 0x50, 0x81, 0xd8, 0x01, 0x8a, 0x13, 0x00, 0x00, 0x00, 0x00,
	0x04, 0x00, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x04, 0x04,
	0x04, 0x04, 0x00, 0x00, 0x04, 0x00, 0x01, 0x02, 0x07, 0x00, 0x17,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 0xEF / 0xF8 calibration; byte [7] echoes whatever the PS3 last wrote in SET 0xEF byte[6] (the "do you
// remember" check the console performs during the 0xEF dance).
static const uint8_t report_ef[64] = {
	0x00, 0xef, 0x04, 0x00, 0x07, 0x03, 0x01, 0xb0, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x6b, 0x02, 0x68, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const uint8_t report_f8[64] = {
	0x00, 0x01, 0x00, 0x00, 0x07, 0x03, 0x01, 0xb0, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x6b, 0x02, 0x68, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const uint8_t report_f7[64] = {
	0x01, 0x04, 0xc4, 0x02, 0xd6, 0x01, 0xee, 0xff, 0x14, 0x13, 0x01,
	0x02, 0xc4, 0x01, 0xd6, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x03,
	0x00, 0x00, 0x02, 0x00, 0x00, 0x02, 0x62, 0x01, 0x02, 0x01, 0x5e,
	0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// One HID slot -- a PS3 expects exactly one Sixaxis per USB port.
static Adafruit_USBD_HID g_ds3;
static unsigned long g_ds3LastMs = 0;
// A slot counts as the live controller while it has replied within this window (matches usb_mount's CONN_UP_MS).
#define DS3_CONN_MS 1200u
// First bonded slot with a recent RF reply (the controller we present), or -1 if none is connected. Used for
// both input streaming and rumble routing (this mode is static, so the dynamic g_usbToBond map isn't built).
static int ds3ActiveSlot()
{
	unsigned long now = millis();
	for (int s = 0; s < NSLOT; s++)
		if (g_slot[s].used && g_connReplyMs[s] &&
		    (now - g_connReplyMs[s]) < DS3_CONN_MS)
			return s;
	return -1;
}
static uint8_t g_byte6ef =
	0xb0; // last SET-0xEF byte[6]; echoed back in GET 0xEF/0xF8 byte[7]
static uint8_t g_masterBd[6] = {
	0, 0, 0, 0, 0, 0
}; // host MAC learned via SET 0xF5
static bool g_haveMaster = false;

// Controller BT MAC. OUI 00:1B:DC matches the other PS modes here; last byte 0x80 keeps it distinct. Used
// only for BT pairing identity -- irrelevant over USB, but the PS3 reads it via GET 0xF2.
static uint8_t g_ds3Mac[6] = { 0x00, 0x1B, 0xDC, 0x4F, 0x55, 0x80 };

// Reproduce a genuine 64-byte Sixaxis feature report on the wire. TinyUSB's HID GET_REPORT path force-writes
// the requested report id as wire byte 0 and hands us the buffer PAST it (`buf` = ctrl[1]); a real Sixaxis
// (and GIMX) instead puts the array's own first byte there. So we overwrite that prepended id via buf[-1]
// (== ctrl[0], a valid writable byte for this control transfer) and copy bytes [1..] into buf. Returns the
// payload length; the stack adds the 1 byte we placed -> total = min(64, wLength), exactly like GIMX's
// Endpoint_Write_Control_Stream_LE(arr, wLength). reqlen is post-prepend (host wLength - 1); the PS3 always
// reads these with wLength >= 8, so the prepend always happened and buf[-1] is in bounds.
static uint16_t emitFeature(uint8_t *buf, uint16_t reqlen,
			    const uint8_t arr[64])
{
	// buf[-1] is only the prepended id (and writable) when the stack actually prepended -- which it does iff
	// the host's wLength was > 1. reqlen here is already post-decrement, so reqlen < 2 means either no prepend
	// (wLength <= 1) or a 2-byte request; bail in both cases so we never write out of bounds. The PS3 reads
	// these reports with wLength >= 8, so real requests are unaffected.
	if (reqlen < 2)
		return 0;
	buf[-1] = arr[0];
	uint16_t n = 63; // 64-byte report minus byte 0 (placed above)
	if (n > reqlen)
		n = reqlen;
	memcpy(buf, arr + 1, n);
	return n;
}

static uint16_t ds3GetReport(uint8_t rid, hid_report_type_t type, uint8_t *buf,
			     uint16_t reqlen)
{
	if (type != HID_REPORT_TYPE_FEATURE || !buf || reqlen == 0)
		return 0;
	uint8_t r[64];
	switch (rid) {
	case 0x01:
		return emitFeature(buf, reqlen, report_01);
	case 0xf2: // overlay our controller MAC at [4..9]
		memcpy(r, report_f2, 64);
		memcpy(r + 4, g_ds3Mac, 6);
		return emitFeature(buf, reqlen, r);
	case 0xf5: // overlay learned host MAC at [2..7] (else the dummy aa.. from the template)
		memcpy(r, report_f5, 64);
		if (g_haveMaster)
			memcpy(r + 2, g_masterBd, 6);
		return emitFeature(buf, reqlen, r);
	case 0xef: // echo the PS3's remembered byte at [7]
		memcpy(r, report_ef, 64);
		r[7] = g_byte6ef;
		return emitFeature(buf, reqlen, r);
	case 0xf8:
		memcpy(r, report_f8, 64);
		r[7] = g_byte6ef;
		return emitFeature(buf, reqlen, r);
	case 0xf7:
		return emitFeature(buf, reqlen, report_f7);
	default:
		return 0;
	}
}

static void ds3SetReport(uint8_t rid, hid_report_type_t type, uint8_t const *b,
			 uint16_t n)
{
	if (type == HID_REPORT_TYPE_FEATURE) {
		// 0xEF: remember byte[6] so the next GET 0xEF/0xF8 echoes it. 0xF5: learn the host's BT MAC.
		// 0xF4: the PS3's "set operational"/power command -- ACK silently (no gating needed; we stream
		// unconditionally like GIMX). b[1]==0x08 would be a power-off request; ignored here.
		if (rid == 0xef && n >= 7)
			g_byte6ef = b[6];
		else if (rid == 0xf5 && n >= 8) {
			memcpy(g_masterBd, b + 2, 6);
			g_haveMaster = true;
		}
		return;
	}
	if (type != HID_REPORT_TYPE_OUTPUT || n < 1)
		return;
	// DS3 output report 0x01 (rumble + LED). It reaches us two ways: on the OUT interrupt endpoint (rid=0,
	// report id 0x01 included as b[0]) or via control SET_REPORT(Output,0x01) (rid=0x01, id not in the data).
	// Normalise to `p` = the bytes AFTER the report id, where the full report is
	//   [0]=id [1]=0x00 [2]=small(right/HFR) dur [3]=small power(on/off) [4]=large(left/LFR) dur [5]=large power.
	// So in p: p[2]=small power, p[4]=large power. Map large->lowFreq, small->highFreq;
	// 257 = 65535/255 scales 8-bit power to the 16-bit haptic range.
	const uint8_t *p;
	uint16_t pn;
	if (rid == 0) { // OUT endpoint: report id leads the transfer
		if (b[0] != 0x01)
			return;
		p = b + 1;
		pn = (uint16_t)(n - 1);
	} else if (rid ==
		   0x01) { // control SET_REPORT: id may or may not be echoed in the data
		if (b[0] == 0x01) {
			p = b + 1;
			pn = (uint16_t)(n - 1);
		} else {
			p = b;
			pn = n;
		}
	} else {
		return;
	}
	if (pn < 5)
		return;
	int slot = ds3ActiveSlot();
	if (slot < 0)
		return;
	hapticSteamRumble((uint16_t)p[4] * 257u, p[2] ? 0xFFFFu : 0u,
			  (uint8_t)slot);
}

// SC2 IMU int16 (center 0) -> DS3 10-bit unsigned (center 511), little-endian on the wire (low byte first).
// >>6 maps the full int16 swing onto roughly +/-512 around center; the PS3 applies its own calibration so
// exact scale is not critical. Writes 2 bytes at out[0..1].
static void ds3Imu(uint8_t *out, int16_t v)
{
	int32_t e = 511 + ((int32_t)v >> 6);
	if (e < 0)
		e = 0;
	if (e > 1023)
		e = 1023;
	out[0] = (uint8_t)(e & 0xFF);
	out[1] = (uint8_t)((e >> 8) & 0xFF);
}

// Build the 48-byte input-report PAYLOAD (the stack prepends report id 0x01 -> 49-byte Sixaxis report).
// Offsets are the genuine report's rd[] minus one (rd[0] is the prepended id).
static void ds3Build(uint8_t slot, uint8_t out[48])
{
	uint32_t b = psButtonsFromSteam(g_in[slot].buttons);
	bool l2 = (g_in[slot].lt > SW_TRIG_ON) || (b & TB_L2);
	bool r2 = (g_in[slot].rt > SW_TRIG_ON) || (b & TB_R2);
	memset(out, 0, 48);

	// out[0] reserved (rd[1]); if it were 0xFF the kernel/console would drop the report -- keep 0.

	// out[1] (rd[2]): Select(bit0) L3 R3 Start(bit3) Up Right Down Left.
	// SC2 View -> Start, Menu -> Select: matches the physical button positions on the puck/controller (the
	// opposite of the PS5/DS4 Create/Options convention, which read reversed on the DS3 in testing).
	out[1] = ((b & TB_MENU) ? 0x01 : 0) | ((b & TB_L3) ? 0x02 : 0) |
		 ((b & TB_R3) ? 0x04 : 0) | ((b & TB_VIEW) ? 0x08 : 0) |
		 ((b & TB_DUP) ? 0x10 : 0) | ((b & TB_DRT) ? 0x20 : 0) |
		 ((b & TB_DDN) ? 0x40 : 0) | ((b & TB_DLF) ? 0x80 : 0);

	// out[2] (rd[3]): L2 R2 L1 R1 Triangle Circle Cross Square. g_abSwap swaps the A/B + X/Y face pair.
	uint8_t tri, cir, crs, sqr;
	if (g_abSwap) {
		tri = (b & TB_A) ? 0x10 : 0;
		cir = (b & TB_B) ? 0x20 : 0;
		crs = (b & TB_X) ? 0x40 : 0;
		sqr = (b & TB_Y) ? 0x80 : 0;
	} else {
		tri = (b & TB_Y) ? 0x10 : 0;
		cir = (b & TB_B) ? 0x20 : 0;
		crs = (b & TB_A) ? 0x40 : 0;
		sqr = (b & TB_X) ? 0x80 : 0;
	}
	out[2] = (l2 ? 0x01 : 0) | (r2 ? 0x02 : 0) | ((b & TB_LB) ? 0x04 : 0) |
		 ((b & TB_RB) ? 0x08 : 0) | tri | cir | crs | sqr;

	// out[3] (rd[4]): PS button
	out[3] = (b & TB_STEAM) ? 0x01 : 0;

	// out[5..8] (rd[6..9]): LX LY RX RY, center 0x80
	out[5] = swStick(g_in[slot].lx, false);
	out[6] = swStick(g_in[slot].ly, true);
	out[7] = swStick(g_in[slot].rx, false);
	out[8] = swStick(g_in[slot].ry, true);

	// out[13..24] (rd[14..25]): analog pressures, order Up Right Down Left L2 R2 L1 R1 Tri Cir Cross Square
	out[13] = (b & TB_DUP) ? 0xFF : 0;
	out[14] = (b & TB_DRT) ? 0xFF : 0;
	out[15] = (b & TB_DDN) ? 0xFF : 0;
	out[16] = (b & TB_DLF) ? 0xFF : 0;
	out[17] = g_in[slot].lt;
	out[18] = g_in[slot].rt;
	out[19] = (b & TB_LB) ? 0xFF : 0;
	out[20] = (b & TB_RB) ? 0xFF : 0;
	out[21] = tri ? 0xFF : 0;
	out[22] = cir ? 0xFF : 0;
	out[23] = crs ? 0xFF : 0;
	out[24] = sqr ? 0xFF : 0;

	// out[28..29] (rd[29..30]): connection/charge status. Battery "full"; cosmetic on the PS3 over USB
	// (the console knows it's wired from enumeration, not this byte).
	out[28] = 0x00;
	out[29] = 0x05; // battery level: full

	// out[40..47] (rd[41..48]): accel X, accel Z, accel Y, gyro Z -- each 10-bit LE, center 511.
	ds3Imu(out + 40, g_in[slot].ax);
	ds3Imu(out + 42, g_in[slot].az);
	ds3Imu(out + 44, g_in[slot].ay);
	ds3Imu(out + 46, g_in[slot].gz);
}

// Neutral input report: centered sticks + IMU, no buttons. Streamed when no controller is linked so the PS3
// keeps the Sixaxis "alive" (and a games/XMB controller slot) instead of seeing a silent endpoint.
static void ds3Neutral(uint8_t out[48])
{
	memset(out, 0, 48);
	out[5] = out[6] = out[7] = out[8] = 0x80; // sticks centered
	out[29] = 0x05; // battery full
	ds3Imu(out + 40, 0);
	ds3Imu(out + 42, 0);
	ds3Imu(out + 44, 0);
	ds3Imu(out + 46, 0);
}

void Ps3Controller::usbIdentity()
{
	// Genuine Sixaxis / DualShock 3 identity. The PS3 recognises the pad by exactly this VID/PID.
	USBDevice.setID(0x054C, 0x0268);
	USBDevice.setDeviceVersion(0x0100);
	USBDevice.setManufacturerDescriptor("Sony");
	USBDevice.setProductDescriptor("PLAYSTATION(R)3 Controller");
}

// Static mount: present the single DS3 interface permanently (the setup() static path calls begin() then
// attaches; it does NOT call usbIdentity() or build a slot pool, so we set the identity and register the HID
// here). g_ds3.begin() both adds the interface to the descriptor and locks its TinyUSB instance.
void Ps3Controller::begin()
{
	usbIdentity();
	g_ds3.enableOutEndpoint(
		true); // OUT endpoint for rumble/LED output reports
	g_ds3.setReportCallback(ds3GetReport, ds3SetReport);
	g_ds3.setReportDescriptor(DS3_HID_DESC, sizeof DS3_HID_DESC);
	g_ds3.setPollInterval(1); // 1 ms interrupt IN, like the genuine pad
	g_ds3.begin();
}

void Ps3Controller::task()
{
	if (!g_ds3.ready())
		return;
	if (millis() - g_ds3LastMs < USB_STREAM_MS)
		return;
	g_ds3LastMs = millis();
	uint8_t p[48];
	int slot = ds3ActiveSlot();
	if (slot >= 0)
		ds3Build((uint8_t)slot, p);
	else
		ds3Neutral(p);
	usbTxHid(&g_ds3, 0x01, p, sizeof p);
}
