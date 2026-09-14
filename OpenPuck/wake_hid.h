// wake_hid.h -- a minimal boot-protocol HID interface whose ONLY job is to make the device a host-recognized
// USB wake source.
//
// Every mode already advertises remote-wakeup in its config descriptor, so the device is *armed* to wake the
// host. But hosts only *honor* a resume signal from an allow-listed input device class -- a HID keyboard/mouse.
// Windows (especially Modern Standby) ignores the wake from a bare gamepad / vendor / composite presentation
// even though it's armed. Exposing a boot MOUSE interface puts the device in that allow-list. It never sends
// reports -- the actual wake is the device-level USBDevice.remoteWakeup() resume signal driven from
// rf_link.cpp; this interface only changes how the host classifies us. (A boot keyboard didn't enumerate on
// Windows.)
//
// Added for every clean mode AND for puck mode on a normal boot: puck mode drops its CDC serial console by
// default (freeing the endpoint this interface needs) so it too can wake Windows. The one-shot debug boot keeps
// CDC and skips this interface (no endpoint room for both) -- see config.h (g_debugCdcThisBoot).
#pragma once

#include <stdint.h>

// register the boot-mouse wake interface (call from setup() for clean modes + normal puck boot). Locks its
// TinyUSB HID instance index -- for dynamic-mount modes call this BEFORE the slot pool so it is HID instance 0.
void wakeHidBegin();

// Re-add the (already-begun) wake-mouse interface to a freshly-cleared config descriptor, for a dynamic
// re-enumeration that doesn't reboot. begin() is once-only; this re-emits the interface descriptor.
void wakeHidAddInterface();

// true once wakeHidBegin() has registered the boot mouse this boot (false on the debug-CDC boot, which omits it)
bool wakeHidPresent();

// boot mouse enumerated and ready to accept a report (false while the bus is suspended/unconfigured)
bool wakeHidReady();

// send one boot-mouse movement report (buttons=0). This is the input that actually wakes the host: it rides the
// interface Windows armed as the wake source, unlike a gamepad-slot report. Returns false if not ready.
bool wakeHidMove(int8_t dx, int8_t dy);

// Arm and service a harmless post-resume net-zero mouse nudge.
// If this boot intentionally omitted the wake HID (strict console / debug exception),
// wakeHidArmNudge() is a no-op; Steam's existing puck-HID fallback remains separate.
void wakeHidArmNudge();
void wakeHidTask();
