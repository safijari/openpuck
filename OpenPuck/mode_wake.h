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

// Where to land after the hotkey has been sent. MODE_STEAM is the normal puck; change if you live in
// another personality.
#define WAKE_RETURN_MODE MODE_STEAM

// The hotkey itself. Lenovo ThinkCentre Smart Power On = Alt+P.
#define WAKE_MOD KEYBOARD_MODIFIER_LEFTALT
#define WAKE_KEY HID_KEY_P

// How long the RF link must be continuously down before an up-edge counts as a wake gesture. Must be
// comfortably longer than anySlotLinkUp()'s 300 ms window so an ordinary RF hiccup can't arm us.
#define WAKE_ARM_DOWN_MS 3000u

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
// MCU never cold-boots -- without this, one wake per manual arm). Off by default because S3 sleep is also
// a persistent suspend, and a device that re-attaches mid-S3 is never configured: with this on, the
// existing wake-on-connect-from-S3 path stops working. Use it on hosts that shut down to S4/S5.
#ifndef WAKE_AUTO_REARM
#define WAKE_AUTO_REARM 0
#endif
#define WAKE_AUTO_REARM_MS 15000u

// Post-fire handoff grace. After the hotkey is sent the machine spends tens of seconds in POST + OS boot
// with the bus suspended (nothing has enumerated the reborn puck yet) -- which is indistinguishable from
// "the host went to sleep". Two standing policies misfire on that state: the suspend-persisted controller
// power-off (haptics.cpp, SUSPEND_OFF_MS) shuts down the controller that just triggered the wake, and
// WAKE_AUTO_REARM would yank the puck back into the wake personality mid-boot. wakeHandoffMark() arms a
// PERSISTED one-shot (Cfg.wakeHandoff -- flash, because this board class's bootloader wipes .noinit RAM
// on reset) right before the return reboot; wakeHandoffActive() then holds both policies off until a USB
// host actually mounts us, or this deadline passes (machine never booted -> the normal power-saving
// behavior is correct after all). The handoff boot also skips setup()'s mount wait and opens the RF
// connect cooldown immediately, so the reborn puck answers the triggering controller ~3 s sooner --
// before it gives up searching and powers itself off.
#define WAKE_HANDOFF_GRACE_MS 90000u
void wakeHandoffMark();
bool wakeHandoffActive();

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
