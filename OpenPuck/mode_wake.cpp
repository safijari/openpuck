#include "mode_wake.h"
#include "config.h"
#include "rf_link.h" // anySlotLinkUp()
#include "usb_mount.h" // modeSwitchReboot()
#include "usb_tx.h" // usbTxHid()
#include "status_led.h" // ledWakePulse()
#include <Adafruit_TinyUSB.h>
#include <string.h>

WakeController g_wakeCtl;

// Plain boot keyboard, NO report ID. Report ID 0 matters: the EC will very likely put us in boot protocol
// (SET_PROTOCOL 0), whose report format is the fixed 8-byte [modifier][reserved][keycode x6] with no id
// byte. TUD_HID_REPORT_DESC_KEYBOARD() with no id argument emits exactly that, so the report protocol and
// boot protocol payloads are identical and it does not matter which one the host picks.
//
// Note wake_hid.cpp's comment that a boot KEYBOARD failed to enumerate on Windows where the boot mouse
// worked -- that was a keyboard added to the puck composite as an extra wake interface. This is a
// standalone single-interface keyboard talking to a firmware EC, which is the case a real keyboard
// exercises. If it does not enumerate, that is the thing to bisect first.
static const uint8_t WAKE_KBD_DESC[] = { TUD_HID_REPORT_DESC_KEYBOARD() };
static Adafruit_USBD_HID g_kbd;

// Post-fire handoff grace (see mode_wake.h). The arm is PERSISTED (Cfg.wakeHandoff, one-shot consumed by
// loadCfg like bootMode) because this board class's bootloader wipes .noinit RAM on every reset -- a RAM
// flag simply does not survive the return reboot. The flash write costs nothing extra: the return mode
// switch already rewrites cfg for its own one-shot bootMode.
void wakeHandoffMark()
{
	g_wakeHandoffArm =
		1; // persisted by the modeSwitchReboot save that follows
}

bool wakeHandoffActive()
{
	static bool checked = false, active = false;
	static uint32_t t0 = 0;
	if (!checked) {
		checked = true;
		if (g_wakeHandoffBoot) {
			active = true;
			t0 = millis();
		}
	}
	if (active &&
	    (USBDevice.mounted() || millis() - t0 >= WAKE_HANDOFF_GRACE_MS))
		active = false;
	return active;
}

enum {
	W_WAIT_DOWN, // link must go quiet before we arm (see WAKE_ARM_DOWN_MS)
	W_ARMED, // waiting for a controller to connect
	W_PRESS, // send modifier-down
	W_MODLEAD, // modifier alone, then add the key
	W_RELEASE, // hold (re-sending), then send key-up
	W_GAP, // spacing between repeats
	W_SETTLE, // let the key-up land before we tear the device down
	W_DONE // reboot issued (modeSwitchReboot does not return)
};
static uint8_t s_st = W_WAIT_DOWN;
static uint32_t s_t = 0; // start of the current timed phase
static uint32_t s_dl = 0; // press-sequence start, for WAKE_SEQ_DEADLINE_MS
static uint32_t s_resend = 0; // last held-report re-send (WAKE_RESEND_MS)
static uint8_t s_shots = 0;

// One boot-keyboard report. Queued for the usbTx drain like every other report in this firmware -- loop()
// never calls tud_* directly (see usb_tx.h). key==0 with mod==0 is the all-keys-up report.
static void kbdSend(uint8_t mod, uint8_t key)
{
	hid_keyboard_report_t r;
	memset(&r, 0, sizeof r);
	r.modifier = mod;
	r.keycode[0] = key;
	usbTxHid(&g_kbd, 0, &r, sizeof r);
}

void WakeController::usbIdentity()
{
	// Distinct identity from every other personality. Hosts cache the configuration descriptor by
	// VID:PID:bcdDevice (see the bcdDevice rule in ARCHITECTURE.md), so the wake keyboard must not share
	// one with the puck or a host can serve the wrong cached descriptor for whichever enumerates second.
	// The PID is arbitrary and unclaimed -- nothing matches on it; the EC keys off the HID class.
	USBDevice.setID(0x28DE, 0x574B);
	USBDevice.setDeviceVersion(0x0100);
	USBDevice.setManufacturerDescriptor("OpenPuck");
	USBDevice.setProductDescriptor("OpenPuck Wake Keyboard");
}

void WakeController::begin()
{
	usbIdentity();
	g_kbd.setBootProtocol(HID_ITF_PROTOCOL_KEYBOARD);
	g_kbd.setStringDescriptor("OpenPuck Wake Keyboard");
	g_kbd.setReportDescriptor(WAKE_KBD_DESC, sizeof WAKE_KBD_DESC);
	// 10 ms, a real keyboard's cadence; nothing here is latency-sensitive
	g_kbd.setPollInterval(10);
	g_kbd.begin();
}

// Non-blocking throughout: every wait is a millis() comparison, never a delay(). loop() has to keep feeding
// the ~8 s watchdog and keep the RF poll running while we sit here, so nothing in this file may spin.
void WakeController::task()
{
	const uint32_t now = millis();
	const bool up = anySlotLinkUp();

	switch (s_st) {
	case W_WAIT_DOWN:
		// Any link activity restarts the quiet timer. s_t starts at 0, so a boot with no controller
		// already gets the full WAKE_ARM_DOWN_MS of grace before arming.
		if (up) {
			s_t = now;
			break;
		}
		if (now - s_t >= WAKE_ARM_DOWN_MS)
			s_st = W_ARMED;
		break;

	case W_ARMED:
		// The wake gesture. ready() means the host has actually configured our interface -- on a machine
		// in S5 that is the EC's minimal stack having enumerated us. If it never goes true we simply never
		// fire, and the LED never flashes, which is the diagnostic.
		if (up && g_kbd.ready()) {
			s_shots = 0;
			s_dl = now;
			s_st = W_PRESS;
		}
		break;

	case W_PRESS:
		// Deadline: the EC deconfigured us mid-sequence (port reset as POST takes over the bus). Bail to
		// settle instead of stalling here until the booted OS re-enumerates us and gets a stray Alt+P.
		// The queued all-keys-up is harmless whenever it drains.
		if (now - s_dl >= WAKE_SEQ_DEADLINE_MS) {
			kbdSend(0, 0);
			s_t = now;
			s_st = W_SETTLE;
			break;
		}
		if (!g_kbd.ready())
			break;
		// Stage the press like a human types it: Alt down alone first. A
		// minimal EC hotkey parser may track modifier-then-key transitions
		// rather than accepting a combined report out of nowhere.
		kbdSend(WAKE_MOD, 0);
		s_t = now;
		s_st = W_MODLEAD;
		break;

	case W_MODLEAD:
		if (now - s_t < WAKE_MOD_LEAD_MS)
			break;
		kbdSend(WAKE_MOD, WAKE_KEY);
		// wake-debugger convention: flash = a wake was actually sent
		ledWakePulse();
		s_resend = now;
		s_t = now;
		s_st = W_RELEASE;
		break;

	case W_RELEASE:
		if (now - s_t < WAKE_HOLD_MS) {
			// Keep restating the held chord: an EC that samples the
			// current report (rather than edge-detecting) can miss a
			// single transfer.
			if (now - s_resend >= WAKE_RESEND_MS && g_kbd.ready()) {
				kbdSend(WAKE_MOD, WAKE_KEY);
				s_resend = now;
			}
			break;
		}
		kbdSend(0, 0);
		s_t = now;
		s_st = (++s_shots < WAKE_REPEAT) ? W_GAP : W_SETTLE;
		break;

	case W_GAP:
		// Repeats are cheap insurance against the EC missing the first report. Extra Alt+P presses that
		// land during POST are harmless -- Lenovo's POST hotkeys are F1/F12/Enter.
		if (now - s_t >= WAKE_REPEAT_MS)
			s_st = W_PRESS;
		break;

	case W_SETTLE:
		if (now - s_t < WAKE_SETTLE_MS)
			break;
		s_st = W_DONE;
		// The machine is now powering on but will look bus-suspended until POST enumerates the reborn
		// puck; hold the suspend policies off across that window (see mode_wake.h).
		wakeHandoffMark();
		// Clean detach + reboot into the puck. Does not return.
		modeSwitchReboot(WAKE_RETURN_MODE);
		break;

	case W_DONE:
	default:
		break;
	}
}

// Auto-re-arm watcher, called from loop() in every mode (compiled to nothing unless the WAKE_AUTO_REARM
// build option is on -- see mode_wake.h for why it is off by default). Note this deliberately bypasses the
// "no mode switch while suspended" convention the chord + console paths follow: their rule exists so a
// SLEEPING host never resumes to find a different device, and here a long suspend meaning "the host is
// down" is the entire premise.
void wakeAutoRearmTask()
{
#if WAKE_AUTO_REARM
	static uint32_t downAt = 0;
	static bool wasSusp = false;
	// A machine POSTing after our own wake fire looks suspended; re-arming now would swap the puck out
	// from under the boot. Hold off until the handoff grace resolves (host mounts us, or deadline).
	if (wakeHandoffActive())
		return;
	const bool susp = USBDevice.suspended();
	if (susp && !wasSusp)
		downAt = millis();
	wasSusp = susp;
	if (!susp || modeIsWake(g_usbMode))
		return;
	if (millis() - downAt >= WAKE_AUTO_REARM_MS)
		modeSwitchReboot(MODE_WAKE);
#endif
}
