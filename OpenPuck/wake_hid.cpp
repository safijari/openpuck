#include "wake_hid.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>

// Boot MOUSE descriptor -- proven to enumerate and wake Windows. The host enumerates a "HID-compliant mouse"
// child and arms IT as the wake source, so the wake nudge (wakeHidMove, driven from the puck wake path) must
// ride THIS interface -- a report on the gamepad slot lands on an interface the host never allow-listed. (A boot
// keyboard didn't enumerate on Windows and suppressed the wake the mouse class provides.)
static const uint8_t WAKE_HID_DESC[] = { TUD_HID_REPORT_DESC_MOUSE() };
static Adafruit_USBD_HID g_wakeHid;
static bool g_wakeHidPresent = false;

void wakeHidBegin()
{
	// boot mouse = a wake device class honored by Windows + Linux
	g_wakeHid.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
	g_wakeHid.setStringDescriptor("OpenPuck Wake");
	g_wakeHid.setReportDescriptor(WAKE_HID_DESC, sizeof WAKE_HID_DESC);
	g_wakeHid.setPollInterval(10);
	g_wakeHid.begin();
	g_wakeHidPresent = true;
}

bool wakeHidPresent()
{
	return g_wakeHidPresent;
}

bool wakeHidReady()
{
	return g_wakeHidPresent && g_wakeHid.ready();
}

bool wakeHidMove(int8_t dx, int8_t dy)
{
	if (!wakeHidReady())
		return false;
	// boot mouse descriptor has no report ID -> report_id 0; buttons=0 so we move but never click. Queued for
	// the usbd task (usbTxHid) rather than sent inline, like every other report -- loop() issues no tud_* call.
	hid_mouse_report_t m;
	m.buttons = 0;
	m.x = dx;
	m.y = dy;
	m.wheel = 0;
	m.pan = 0;
	usbTxHid(&g_wakeHid, 0, &m, sizeof m);
	return true;
}

void wakeHidAddInterface()
{
	TinyUSBDevice.addInterface(g_wakeHid);
}
