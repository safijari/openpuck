// mode_wake.h -- MODE_WAKE: a boot-keyboard-only personality that powers on a
// sleeping/off desktop with its firmware hotkey, then hands the port back to the puck.
//
// WHY THIS EXISTS. USB remote-wakeup (wake_hid.cpp + the rf_link connect edge) only works from S3: it
// needs a host controller that is still clocked. From a full shutdown (S5) there is no host stack to
// resume -- the only thing listening on the port is the board's embedded controller, and on Lenovo
// ThinkCentre (BIOS: Power > Smart Power On) it powers the machine on when it sees Alt+P from a USB
// keyboard on the designated always-on port. Dell/others have the same feature under different names and
// different chords; see WAKE_MOD / WAKE_KEY below.
//
// WHY A SEPARATE MODE RATHER THAN ONE MORE INTERFACE ON THE PUCK. Two reasons:
//   1. The EC's S5 USB stack is minimal. A real keyboard is what it was written against, so the safest
//      thing to show it is a single boot-protocol HID keyboard and nothing else -- no wake mouse, no
//      WebUSB, no CDC. setup() drops all of those for this mode (see `bareHid` in OpenPuck.ino).
//   2. Endpoints. The puck composite already uses 6 of the nRF52840's 7 data IN endpoints, so bolting a
//      keyboard onto it would spend the last one. Two-phase costs nothing: each phase is small.
//
// THE FLOW.
//   arm     -- chord / WebUSB panel / CDC console switch to MODE_WAKE (modeSwitchReboot). With the default
//              "always boot Steam" policy this is a ONE-SHOT bootMode, which is exactly right: one wake,
//              then back to normal.
//   settle  -- the mode refuses to arm until the RF link has been DOWN for WAKE_ARM_DOWN_MS. This is the
//              stand-in for "is the host off?": you arm while the controller is still on, and arming only
//              completes once you put the controller down / power the desk off. Without it the still-
//              connected controller would trip the wake the instant we rebooted, firing Alt+P into the
//              running session and bouncing straight back to puck mode.
//   fire    -- first link-up edge after arming sends Alt+P (WAKE_REPEAT times), LED pulses per the
//              status_led wake-debugger convention: flash = we sent it, no flash = we never fired.
//   hand off-- after WAKE_SETTLE_MS, modeSwitchReboot(WAKE_RETURN_MODE). The reboot is a real detach +
//              re-enumerate, so the machine -- which is a couple of seconds into POST by then -- enumerates
//              the puck normally and Steam sees it as it always does.
//
// RE-ARMING. The smart-power-on port keeps VBUS up in S5, so the MCU never cold-boots and nothing re-arms
// this mode after it fires: by default it is one wake per manual arm. The WAKE_AUTO_REARM build option
// (below) makes it repeatable: a persistent USB suspend in any other mode reboots back into MODE_WAKE.
#pragma once
#include "controllers.h"

// Where to land after the hotkey has been sent. 0xFF = modeSwitchReboot's "keep current" sentinel, which
// boots by the normal policy: the persisted mode for persist-last-mode users, Steam otherwise -- so a
// wake never overwrites the user's chosen personality. Set a concrete mode to force one instead.
#define WAKE_RETURN_MODE 0xFF

// The hotkey itself. Lenovo ThinkCentre Smart Power On = Alt+P.
#define WAKE_MOD KEYBOARD_MODIFIER_LEFTALT
#define WAKE_KEY HID_KEY_P

// How long the RF link must be continuously down before an up-edge counts as a wake gesture. Must be
// comfortably longer than anySlotLinkUp()'s 300 ms window so an ordinary RF hiccup can't arm us -- and
// long enough that a controller briefly walking out of range while the host is LIVE can't re-arm a
// keyboard pointed at the session. The quiet clock only runs once the RF side is on air (rfConnectOpen()),
// so with a controller off at the mode switch, arming lands ~WAKE_ARM_DOWN_MS after the cooldown opens.
#define WAKE_ARM_DOWN_MS 5000u

// An up-EDGE seen while the keyboard is not yet ready() stays a valid gesture for this long. Long enough
// for the EC's resume-then-configure lag after our remote wakeup (milliseconds), far shorter than a human
// power-on: a controller that connected against a dead port must NOT fire hours later when a manually
// booted OS finally enumerates the keyboard.
#define WAKE_FIRE_LATCH_MS 2000u

// Press shaping. A combined Alt+P report held 60 ms was read but IGNORED by the
// ThinkCentre M90q Gen 6 EC (it drained both repeats off the endpoint, so the
// reports arrived; a real keyboard on the same port woke the machine). Shaping
// the press like a human types it -- modifier alone first, long hold, restate
// the held chord -- makes the same EC fire. Which ingredient is load-bearing is
// untested (each attempt costs an S5 power cycle); all three are what a real
// keyboard produces anyway.
#define WAKE_MOD_LEAD_MS \
	150u // Alt alone before Alt+P (EC may edge-detect modifier-then-key)
#define WAKE_RESEND_MS 100u // restate the held report (slow-sampling ECs)
#define WAKE_HOLD_MS \
	450u // key-down duration (EC samples HID reports, not edges)
#define WAKE_REPEAT 4 // how many Alt+P presses to send
#define WAKE_REPEAT_MS 600u // gap between them
// Quiet time after the last key-up before we detach and re-enumerate as the puck. The EC latches power-on
// immediately; this only needs to outlast the key-up transfer. POST enumeration is seconds later, so
// there is a lot of slack here in both directions. Kept short: every ms here extends the RF outage the
// triggering controller must ride out before the reborn puck answers it again (M90q: the controller gives
// up somewhere between ~2.5 s and ~5 s of silence).
#define WAKE_SETTLE_MS 600u

// Give up on the press sequence this long after it starts. If the EC deconfigures us mid-way (port reset
// as POST takes the bus over), ready() goes false and the machine would otherwise stall in W_PRESS until
// whatever boots next re-enumerates us -- and then receive a stray Alt+P. On deadline: all-keys-up,
// settle, reboot to the puck.
#define WAKE_SEQ_DEADLINE_MS 10000u

// EXPERIMENTAL, default OFF; build with EXTRA_FLAGS="-DWAKE_AUTO_REARM=1" to enable. While running any
// OTHER personality, a continuous USB suspend longer than WAKE_AUTO_REARM_MS reboots into MODE_WAKE, so
// the wake re-arms itself every time the host goes down (the smart-power-on port keeps VBUS in S5, so the
// MCU never cold-boots -- without this, one wake per manual arm). Sleep is distinguished from shutdown by
// the host's own hand: an OS entering S3 arms DEVICE_REMOTE_WAKEUP before suspending (tud_suspend_cb in
// mode_wake.cpp) and such suspends never re-arm, so wake-on-connect-from-S3 keeps working. Still off by
// default: a host with wakeup disabled for the device (sysfs) makes its sleep look like a shutdown,
// degrading to re-arm-on-sleep -- acceptable on hosts that shut down to S4/S5, surprising elsewhere.
#ifndef WAKE_AUTO_REARM
#define WAKE_AUTO_REARM 0
#endif
#define WAKE_AUTO_REARM_MS 15000u
// A bus resume shorter than this does NOT start a new down-episode: during poweroff the smart-port EC
// takes the bus over with a brief resume (~2.4 s captured on the M90q) and re-suspends WITH remote
// wakeup armed -- its own listening mechanism, not the OS's sleep intent. The sleep-vs-shutdown decision
// is sampled once, at the first suspend edge of the episode, and EC flaps cannot overwrite it.
#define WAKE_REARM_FLAP_MS 5000u
// ...but an episode older than this always re-samples: USB selective-suspend churn can put a sub-5 s
// resume right before a genuine sleep, and inheriting an hours-old "shutdown" decision there would
// re-arm under a sleeping host. The EC takeover flap arrives ~3 s into its episode, far under this cap.
#define WAKE_REARM_EPISODE_MS 60000u
// Dead-bus re-arm: TinyUSB only reports suspend on a bus that got a SETUP packet, so a failed wake's
// return boot (EC cannot enumerate the composite) and a plug into an already-off host read neither
// mounted nor suspended -- ever. Never-mounted this long after boot cannot be a live host (any real
// boot's USB is up in well under this), so treat it as a shutdown and re-arm.
#define WAKE_DEADBUS_REARM_MS 90000u

// Post-fire handoff grace. After the hotkey is sent the machine spends tens of seconds in POST + OS boot
// with the bus suspended (nothing has enumerated the reborn puck yet) -- which is indistinguishable from
// "the host went to sleep". Two standing policies misfire on that state: the suspend-persisted controller
// power-off (haptics.cpp, SUSPEND_OFF_MS) shuts down the controller that just triggered the wake, and
// WAKE_AUTO_REARM would yank the puck back into the wake personality mid-boot. wakeHandoffMark() arms a
// PERSISTED one-shot (Cfg.wakeHandoff -- flash, because this board class's bootloader wipes .noinit RAM
// on reset) right before the return reboot; wakeHandoffActive() then holds both policies off for this
// full window. Deadline-only ON PURPOSE -- USBDevice.mounted() is NOT "the OS is up": BIOS legacy-HID
// support SET_CONFIGURATIONs keyboard-bearing devices during POST, and the BIOS->kernel handoff right
// after is exactly the suspend gap the grace exists to cover. Nobody sleeps a machine within 90 s of
// waking it, so nothing is lost by holding the two policies for the whole window; if the machine never
// boots, haptics' expiry fallback still powers the controller off (battery saved). The handoff boot also
// skips setup()'s mount wait and opens the RF connect cooldown immediately, so the reborn puck answers
// the triggering controller in <0.5 s -- before it gives up searching and powers itself off.
#define WAKE_HANDOFF_GRACE_MS 90000u
void wakeHandoffMark();
bool wakeHandoffActive();
// True from this boot's grace expiry onward (false on non-handoff boots): haptics' never-booted fallback.
bool wakeHandoffExpired();
// True while a press sequence is in flight (first press queued .. return reboot): the only wake-mode
// window where haptics' suspend power-off must hold its fire (it would kill the triggering controller).
bool wakeFireInFlight();

class WakeController : public IController {
    public:
	// STATIC mount, single HID, no slot pool: this mode never forwards controller input to the host --
	// onReport45/onAuxReport stay no-ops so a trackpad graze can't leak a keystroke into the BIOS or the
	// just-woken desktop. All the work happens in task().
	void begin() override;
	void task() override;
	void usbIdentity() override;
};
extern WakeController g_wakeCtl;

// Called from loop() in EVERY mode. No-op unless built with WAKE_AUTO_REARM (see above).
void wakeAutoRearmTask();
