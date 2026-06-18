#include "mode_hidgyro.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

HidGyroController g_hidGyroCtl;

static const uint8_t GYRO_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31,
	0x09, 0x32, 0x09, 0x35, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95,
	0x04, 0x81, 0x02, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46,
	0x3B, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x0E, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
	0x95, 0x0E, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x20, 0x75, 0x06, 0x95,
	0x01, 0x15, 0x00, 0x25, 0x7F, 0x81, 0x02, 0x05, 0x01, 0x09, 0x33, 0x09,
	0x34, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02,
	0x06, 0x00, 0xFF, 0x09, 0x21, 0x95, 0x36, 0x81, 0x02, 0x85, 0x05, 0x09,
	0x22, 0x95, 0x1F, 0x91, 0x02, 0x85, 0x04, 0x09, 0x23, 0x95, 0x24, 0xB1,
	0x02, 0x85, 0x02, 0x09, 0x24, 0x95, 0x24, 0xB1, 0x02, 0xC0
};
#define DS4_TOUCH_H 942
#define DS4_STATUS_USB 0x1B // cable + level 11 (full)
static unsigned long g_gyroLastMs = 0;
static Adafruit_USBD_HID g_hidGyro;

static void hidGyroSet(uint8_t rid, hid_report_type_t type, uint8_t const *b,
		       uint16_t n)
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
	} else {
		id = rid;
		p = b;
		pn = n;
	}
	// DS4 USB effects: report 0x05, magic at byte 0, effects block starts at byte 3.
	if (id != 0x05 || pn < 5)
		return;
	hapticSteamRumble((uint16_t)p[4] * 257u,
			  (uint16_t)p[3] * 257u); // DS4: left=low, right=high
}

static void hidGyroBuild(uint8_t out[63])
{
	uint32_t b = psButtonsFromSteam(g_in.buttons);
	bool lTouch = (b & TB_LPADT) || (b & TB_LPADC),
	     rTouch = (b & TB_RPADT) || (b & TB_RPADC);
	memset(out, 0, 63);
	out[0] = swStick(g_in.lx, false);
	out[1] = swStick(g_in.ly, true);
	out[2] = swStick(g_in.rx, false);
	out[3] = swStick(g_in.ry, true);
	out[4] = psHatNibble(b) | psFaceNibble(b);
	out[5] = psShouldersByte(b);
	static uint8_t ctr = 0;
	out[6] = ((ctr++ & 0x0F) << 4) |
		 ((b & TB_TOUCH || b & TB_LPADC || b & TB_RPADC) ? 0x02 : 0) |
		 ((b & TB_STEAM) ? 0x01 : 0);
	out[7] = g_in.lt;
	out[8] = g_in.rt;
	out[12] = g_in.gx & 0xFF;
	out[13] = g_in.gx >> 8;
	out[14] = g_in.gy & 0xFF;
	out[15] = g_in.gy >> 8;
	out[16] = (-g_in.gz) & 0xFF;
	out[17] = (-g_in.gz) >> 8;
	out[18] = g_in.ax & 0xFF;
	out[19] = g_in.ax >> 8;
	out[20] = g_in.ay & 0xFF;
	out[21] = g_in.ay >> 8;
	out[22] = g_in.az & 0xFF;
	out[23] = g_in.az >> 8;
	out[29] = DS4_STATUS_USB;
	if (lTouch || rTouch) {
		uint16_t lx, ly, rx, ry;
		steamPadsToTouch(b, DS4_TOUCH_H, g_in.lpx, g_in.lpy, g_in.rpx,
				 g_in.rpy, &lx, &ly, &rx, &ry);
		static uint8_t tstamp = 0;
		out[32] = 1;
		out[33] = tstamp++;
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

	// Host re-reads config by VID:PID:serial (per-mode suffix); bump invalidates any cached descriptor
	USBDevice.setDeviceVersion(0x0104);
	USBDevice.setManufacturerDescriptor("Sony Computer Entertainment");
	USBDevice.setProductDescriptor("Wireless Controller");
	g_hidGyro.enableOutEndpoint(true);
	g_hidGyro.setReportCallback(NULL, hidGyroSet);
	g_hidGyro.setReportDescriptor(GYRO_HID_DESC, sizeof GYRO_HID_DESC);

	// 1ms bInterval so the RF rate is the only latency limit (matches Xbox)
	g_hidGyro.setPollInterval(1);
	g_hidGyro.begin();
}
void HidGyroController::task()
{
	if (!g_hidGyro.ready())
		return;
	if (millis() - g_gyroLastMs < USB_STREAM_MS)
		return;
	g_gyroLastMs = millis();
	uint8_t p[63];
	hidGyroBuild(p);
	g_hidGyro.sendReport(0x01, p, sizeof p);
}
