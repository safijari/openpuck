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

// A device-level resume can be accepted
// yet still fail to wake some Windows hosts until a real input report arrives on
// the HID interface the host armed as a wake source. Emit a harmless net-zero
// boot-mouse nudge after resume. Strict-console personalities omit this interface,
// so this state machine stays inert there.
#define NUDGE_JIGGLE_PX 10
#define NUDGE_STEP_MS 15u
#define NUDGE_EXPIRE_MS 5000u
static uint8_t g_wakeNudgeStep = 0; // 0=idle, 1=+X, 2=-X
static unsigned long g_wakeNudgeArmMs = 0;
static unsigned long g_wakeNudgeStepMs = 0;

void wakeHidArmNudge()
{
	if (!g_wakeHidPresent)
		return; // intentional strict-console/debug-CDC exception
	g_wakeNudgeStep = 1;
	g_wakeNudgeArmMs = millis();
	g_wakeNudgeStepMs = 0;
}

void wakeHidTask()
{
	if (!g_wakeNudgeStep)
		return;
	if (!g_wakeHidPresent) {
		g_wakeNudgeStep = 0;
		return;
	}
	unsigned long now = millis();
	if (now - g_wakeNudgeArmMs > NUDGE_EXPIRE_MS) {
		g_wakeNudgeStep = 0;
		return;
	}
	if (USBDevice.suspended() || !wakeHidReady())

		// wait until the bus has actually resumed and the HID can accept input
		return;
	if (g_wakeNudgeStepMs && now - g_wakeNudgeStepMs < NUDGE_STEP_MS)
		return;
	int8_t dx = (g_wakeNudgeStep == 1) ? NUDGE_JIGGLE_PX : -NUDGE_JIGGLE_PX;
	if (!wakeHidMove(dx, 0))
		return;
	g_wakeNudgeStepMs = now;
	if (g_wakeNudgeStep == 1)
		g_wakeNudgeStep = 2;
	else
		g_wakeNudgeStep =

			// +10 then -10: net-zero cursor displacement, no clicks
			0;
}

void wakeHidAddInterface()
{
	TinyUSBDevice.addInterface(g_wakeHid);
}
