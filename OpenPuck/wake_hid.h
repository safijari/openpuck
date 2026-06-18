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

// register the boot-keyboard wake interface (call from setup() for clean modes)
void wakeHidBegin();
