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
// flag simply does not survive the return reboot. saveCfg() here because the return reboot goes through
// modeSwitchReboot(0xFF) ("keep current"), which intentionally skips its own saveMode.
void wakeHandoffMark()
{
	// 0xA5, not 1: this byte is the repurposed legacy rumble-strength slot (0..100), so a plausible
	// legacy value must not read as an arm after a firmware upgrade. 0xA5 is outside that range.
	g_wakeHandoffArm = 0xA5;
	saveCfg();
}

bool wakeHandoffActive()
{
	// Deadline-based (mode_wake.h): a POST-time SET_CONFIGURATION reads mounted() long before the OS
	// is up, so a bare mounted() exit would re-expose the suspend power-off during the BIOS->kernel
	// gap. The early end below instead requires a SUSTAINED mounted+active session -- something POST
	// never produces -- so a shutdown shortly after a wake-boot gets normal power-off behavior back.
	static bool init = false, done = false;
	static uint32_t t0 = 0, upSince = 0;
	if (!g_wakeHandoffBoot)
		return false;
	if (!init) {
		init = true;
		t0 = millis();
	}
	if (done)
		return false;
	if (USBDevice.mounted() && !USBDevice.suspended()) {
		if (!upSince)
			upSince = millis();
		else if (millis() - upSince >= WAKE_HANDOFF_MOUNTED_MS)
			done = true;
	} else {
		upSince = 0;
	}
	if (millis() - t0 >= WAKE_HANDOFF_GRACE_MS)
		done = true;
	return !done;
}

bool wakeHandoffExpired()
{
	return g_wakeHandoffBoot && !wakeHandoffActive();
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
// begin() timestamp, anchor for the rearm-boot fire window. Wrap-safe subtraction against this --
// never a raw millis() comparison -- so a puck parked in W_ARMED on always-on VBUS for 49.7 days
// cannot see the no-edge window reopen at the millis() wrap.
static uint32_t s_bootMs = 0;

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
	// Marked rearm boot (0xC3 -- see WAKE_REARM_FIRE_WINDOW_MS): the host was proven down and
	// nothing was on the air at the reboot, so the link-quiet clock's job -- keeping a mode switch
	// from typing into a live session -- is already done, and any prompt connect is a fresh human
	// press. Arm immediately so that press is answerable the moment it (re)links. Rearm boots that
	// had a link up (or only freshly down) are unmarked and go through W_WAIT_DOWN like a manual
	// entry, so a controller that survived its power-off can never fire without a human gesture.
	s_bootMs = millis();
	if (g_wakeRearmBoot)
		s_st = W_ARMED;
}

// Non-blocking throughout: every wait is a millis() comparison, never a delay(). loop() has to keep feeding
// the ~8 s watchdog and keep the RF poll running while we sit here, so nothing in this file may spin.
void WakeController::task()
{
	const uint32_t now = millis();
	const bool up = anySlotLinkUp();

	// The wake gesture is the connect up-EDGE, latched briefly: an EC that suspends the port between
	// enumeration and the press only reads ready() a few ms after the connect (rf_link's remoteWakeup has
	// to resume the bus first). The latch EXPIRES (WAKE_FIRE_LATCH_MS): a controller that connected
	// against a dead port must not fire hours later when a manually booted OS enumerates the keyboard.
	static bool s_wasUp = false;
	static bool s_pend = false;
	static uint32_t s_pendMs = 0;
	if (up && !s_wasUp) {
		s_pend = true;
		s_pendMs = now;
	}
	if (!up || now - s_pendMs >= WAKE_FIRE_LATCH_MS)
		s_pend = false;
	s_wasUp = up;

	switch (s_st) {
	case W_WAIT_DOWN:
		// Link activity restarts the quiet timer -- and so does a closed connect cooldown: before
		// rfConnectOpen() nothing is on air, so a controller that was linked at the mode switch cannot
		// have shown itself yet, and counting that silence would arm us just in time to type Alt+P
		// into the live session it then relinks into.
		if (up || !rfConnectOpen()) {
			s_t = now;
			break;
		}
		if (now - s_t >= WAKE_ARM_DOWN_MS)
			s_st = W_ARMED;
		break;

	case W_ARMED:
		// ready() means the host has actually configured our interface -- on a machine in S5 that is
		// the EC's minimal stack having enumerated us. If it never goes true we simply never fire, and
		// the LED never flashes, which is the diagnostic.
		//
		// Rearm-boot fire window: on a g_wakeRearmBoot (connect-triggered) boot the gesture already
		// happened -- the connect that caused the rearm -- and the controller relinks within ~1 s
		// of boot, but the EC's enumeration of the reborn keyboard can take longer than
		// WAKE_FIRE_LATCH_MS after that edge. For the first WAKE_REARM_FIRE_WINDOW_MS of the boot a
		// link that is simply UP when ready() lands fires without a fresh edge; past the window the
		// strict edge-latch rules return (a controller connected against a dead port must not fire
		// when a manually booted OS finally enumerates us). Wrap-safe subtraction against s_bootMs
		// on purpose (see its comment).
		if ((s_pend || (g_wakeRearmBoot && up &&
				(uint32_t)(now - s_bootMs) <=
					WAKE_REARM_FIRE_WINDOW_MS)) &&
		    g_kbd.ready()) {
			s_pend = false;
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
		kbdSend(g_wakeMod, 0);
		s_t = now;
		s_st = W_MODLEAD;
		break;

	case W_MODLEAD:
		if (now - s_t < WAKE_MOD_LEAD_MS)
			break;
		kbdSend(g_wakeMod, g_wakeKey);
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
				kbdSend(g_wakeMod, g_wakeKey);
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
		// Clean detach + reboot into the return personality (0xFF = the boot-policy default:
		// the user's persisted mode, or Steam). Does not return.
		modeSwitchReboot(WAKE_RETURN_MODE);
		break;

	case W_DONE:
	default:
		break;
	}
}

// True while a press sequence is in flight (haptics' suspend power-off must not kill the triggering
// controller mid-wake; every other wake-mode state keeps the battery-saving power-off).
bool wakeFireInFlight()
{
	return modeIsWake(g_usbMode) && s_st >= W_PRESS;
}

// True while a connect-triggered rearm boot is still inside its fire window (see mode_wake.h): the
// W_ARMED wait for g_kbd.ready() is part of answering the user's press, and haptics' suspend
// power-off must not shoot the triggering controller down during it.
bool wakeRearmWaitActive()
{
	return modeIsWake(g_usbMode) && g_wakeRearmBoot &&
	       (uint32_t)(millis() - s_bootMs) <= WAKE_REARM_FIRE_WINDOW_MS;
}

// The one definition of "a byte the wake press may type": the defined HID keyboard usage range.
// Shared by loadCfg and the WebUSB field-31 setter so the accept policies can never drift apart.
bool wakeKeyValid(uint8_t k)
{
	return k >= HID_KEY_A && k <= HID_KEY_GUI_RIGHT;
}

// Auto-re-arm watcher, called from loop() in every mode (compiled to nothing unless the WAKE_AUTO_REARM
// build option is on -- see mode_wake.h for why it is off by default). Note this deliberately bypasses the
// "no mode switch while suspended" convention the chord + console paths follow: their rule exists so a
// SLEEPING host never resumes to find a different device, and here a long suspend meaning "the host is
// down" is the entire premise.
#if WAKE_AUTO_REARM
// The one device-visible difference between "asleep" and "off": an OS entering S3/s2idle arms
// DEVICE_REMOTE_WAKEUP on wakeup-enabled devices BEFORE suspending (the existing wake-from-S3 path only
// works because it does), while a shutdown never sets the feature. TinyUSB samples the flag at each
// suspend edge and hands it to this weak callback (nothing else in the core claims it). A host that has
// wakeup disabled for the device sends no SET_FEATURE, so its sleep reads as a shutdown -- that just
// degrades to the old always-re-arm behavior, documented in WAKE_MODE.md.
static volatile bool s_suspWkArmed = false;
extern "C" void tud_suspend_cb(bool remote_wakeup_en)
{
	s_suspWkArmed = remote_wakeup_en;
}

// Marked rearm reboot: carries the persisted one-shot marker (Cfg.wakeHandoff 0xC3, flash for the
// same reason as the fire handoff: this board class's bootloader wipes .noinit RAM) attesting that
// nothing was on the air (see WAKE_REARM_FIRE_WINDOW_MS in mode_wake.h). saveCfg runs inside
// modeSwitchReboot's saveMode, so the marker rides the same flash write. Callers with a link up (or
// only freshly down) call modeSwitchReboot(MODE_WAKE) bare instead -- no marker, normal quiet-clock
// arming. Does not return.
static void wakeRearmReboot()
{
	g_wakeHandoffArm = 0xC3;
	modeSwitchReboot(MODE_WAKE);
}
#endif

void wakeAutoRearmTask()
{
#if WAKE_AUTO_REARM
	static uint32_t suspSince = 0; // last suspend EDGE (flaps included)
	static uint32_t downAt = 0; // episode start (0 = none yet)
	static uint32_t lastResumeMs = 0;
	static bool wasSusp = false;
	static bool epArmed = false; // episode's sleep-vs-shutdown decision
	const bool susp = USBDevice.suspended();
	// Track episodes UNCONDITIONALLY -- only the reboot action is gated below. Skipping the tracking
	// during the handoff grace left a shutdown-inside-the-grace unrecorded, so the first post-grace
	// edge sampled the EC's stale remote-wakeup arming as if it were the OS sleeping.
	if (susp && !wasSusp) {
		suspSince = millis();
		// Fresh episode: bus was up >= WAKE_REARM_FLAP_MS (a real host session, however brief), no
		// episode yet, or the current one aged past WAKE_REARM_EPISODE_MS. Otherwise this edge is
		// a flap continuing the current episode: keep its decision.
		if (downAt == 0 ||
		    (uint32_t)(millis() - lastResumeMs) >= WAKE_REARM_FLAP_MS ||
		    (uint32_t)(millis() - downAt) >= WAKE_REARM_EPISODE_MS) {
			// The arming only counts as the OS's sleep intent if a real host is actually
			// CONFIGURED on the bus: a reset-then-suspend EC takeover clears the config and
			// then SET_FEATUREs remote wakeup itself before suspending -- with no resume edge
			// since the reset, that reads as a fresh episode, and sampling the bare flag there
			// classified every such shutdown as a sleeping host and vetoed the re-arm. An OS
			// entering S3 is always mounted at its suspend edge.
			epArmed = s_suspWkArmed && USBDevice.mounted();
			downAt = millis();
		}
	}
	if (!susp && wasSusp)
		lastResumeMs = millis();
	wasSusp = susp;
	// Connect-edge tracking for the rearm-on-connect fast path. Tracked UNCONDITIONALLY like the
	// episode state above -- updating it only past the gates would leave a stale "down" across the
	// grace and misread the first post-grace pass as a connect edge. An edge only counts as a
	// GESTURE if the link was continuously down >= WAKE_REARM_CONN_DOWN_MS first: a real press
	// follows a long controller-off, while an RF fade of a controller that survived its power-off
	// blips down for well under a second (see mode_wake.h).
	static bool wasUp = false;
	static uint32_t linkDownMs =
		0; // when the link last went down (0 = currently up)
	const bool linkUp = anySlotLinkUp();
	bool connGesture = false;
	if (linkUp && !wasUp)
		connGesture = linkDownMs && (uint32_t)(millis() - linkDownMs) >=
						    WAKE_REARM_CONN_DOWN_MS;
	if (!linkUp && wasUp)
		linkDownMs = millis() ? millis() : 1;
	else if (!linkUp && !linkDownMs)
		linkDownMs = millis() ? millis() : 1; // down since boot
	if (linkUp)
		linkDownMs = 0;
	wasUp = linkUp;
	// Track the bus-lost clock UNCONDITIONALLY (like the episode tracking above): latching it below
	// the grace gate made a failed wake's 45 s dead-bus countdown start only after the 90 s grace,
	// re-arming at ~140 s instead of the documented ~grace-end.
	static uint32_t busLostMs = 0;
	if (USBDevice.mounted())
		busLostMs = 0;
	else if (!busLostMs)
		busLostMs = millis() ? millis() : 1;
	// A machine POSTing after our own wake fire looks suspended (or dead); re-arming now would swap
	// the puck out from under the boot. Track above, act only past the grace -- and give haptics'
	// suspend power-off AND its full retry schedule (ripe through send + RETRIES x RETRY_MS = 12 s)
	// a head start when the grace ends with the bus already down, so the re-arm reboot can never
	// destroy a pending resend.
	if (wakeHandoffActive() || modeIsWake(g_usbMode))
		return;
	static uint32_t graceClearMs = 0;
	if (g_wakeHandoffBoot) {
		if (!graceClearMs)
			graceClearMs = millis();
		if ((uint32_t)(millis() - graceClearMs) < 15000u)
			return;
	}
	if (susp) {
		// Rearm-on-connect (see WAKE_REARM_FIRE_WINDOW_MS in mode_wake.h): with a shutdown-decided
		// episode and the suspension settled past the EC-takeover flap window, a controller connect
		// after a real controller-off IS the user pressing power for a wake -- reboot into
		// MODE_WAKE right now instead of letting the suspend power-off shoot the controller down
		// and making the user wait out the timer. Runs before rfLinkTask/hapticTask in loop(), and
		// the power-off frame only leaves on rfLinkTask's poll replies, so this reboot always
		// preempts a same-pass power-off. A sleep episode (epArmed) never lands here: its connect
		// edge did the S3 remote wakeup in rf_link instead.
		if (!epArmed && connGesture &&
		    (uint32_t)(millis() - suspSince) >= WAKE_REARM_FLAP_MS)
			wakeRearmReboot();
		// Countdown from the LAST suspend edge, not the episode start: a manual power-on's POST
		// flaps continue the shutdown episode, and counting from its start would fire the re-arm
		// in the middle of a boot the user started by hand. Every flap restarts the clock; only
		// WAKE_AUTO_REARM_MS of CONTINUOUS suspension (and a shutdown-decided episode) re-arms.
		// Marked (instant-arm) only when nothing is on the air and hasn't been for
		// WAKE_REARM_CONN_DOWN_MS -- then a prompt connect on the wake boot must be a fresh human
		// press, and a press RACING this timer fires first try instead of connecting into an
		// unarmed quiet wait. With a link up (or only just dropped, i.e. possibly a fade of a
		// controller that survived its power-off), boot UNMARKED: quiet-clock arming, so the
		// survivor can never power the machine back on by itself.
		if (!epArmed &&
		    (uint32_t)(millis() - suspSince) >= WAKE_AUTO_REARM_MS) {
			if (!linkUp && linkDownMs &&
			    (uint32_t)(millis() - linkDownMs) >=
				    WAKE_REARM_CONN_DOWN_MS)
				wakeRearmReboot();
			else
				modeSwitchReboot(MODE_WAKE);
		}
		return;
	}
	// Dead-bus path (see WAKE_DEADBUS_REARM_MS): the bus is neither mounted nor suspended -- a
	// reset-style shutdown, a failed wake's return boot, or a plug into an off host. The clock runs
	// from the moment the bus was lost (tracked above, through the grace), NOT puck uptime: an
	// uptime clock fired mid-shutdown and rebooted the puck out from under the pending power-off.
	// No connect fast path here ON PURPOSE: a dead bus is also what a manually-rebooting PC looks
	// like during POST, and a controller powered on during a normal boot must not swap the puck for
	// the wake keyboard -- only the (deliberately long) timer may act on a dead bus. Marked
	// (instant-arm) under the same nothing-on-the-air condition as the suspend timer above; with a
	// link up the boot stays unmarked and quiet-clock arming keeps a standing link from ever
	// firing without a human gesture (a controller off for 3 s+ arms via the quiet clock ~8 s
	// later anyway, so the marker changes timing, not exposure).
	if (busLostMs &&
	    (uint32_t)(millis() - busLostMs) >= WAKE_DEADBUS_REARM_MS) {
		if (!linkUp && linkDownMs &&
		    (uint32_t)(millis() - linkDownMs) >=
			    WAKE_REARM_CONN_DOWN_MS)
			wakeRearmReboot();
		else
			modeSwitchReboot(MODE_WAKE);
	}
#endif
}
