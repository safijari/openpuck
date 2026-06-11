// wake_hid.h -- a minimal boot-protocol HID interface whose ONLY job is to make the device a host-recognized
// USB wake source.
//
// Every mode already advertises remote-wakeup in its config descriptor (the Adafruit core sets
// TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP), so the device is *armed* to wake the host. But hosts only *honor* a
// remote-wakeup resume signal from an allow-listed input device class -- a HID keyboard/mouse. Windows
// (especially under Modern Standby) ignores the wake from a bare gamepad / vendor / composite presentation
// even though it's armed. Exposing a boot MOUSE interface (the same shape Xbox mode already uses to wake
// Windows) puts the device in that allow-list. It never sends reports -- the actual wake is the device-level
// USBDevice.remoteWakeup() resume signal driven from rf_link.cpp; this interface only changes how the host
// classifies us. (A boot keyboard was tried and was worse -- it didn't enumerate on Windows.)
//
// Added for every clean mode AND for puck mode on a normal boot: puck mode drops its CDC serial console by
// default (freeing the endpoint this interface needs) so it too can wake Windows. The exception is the one-shot
// debug boot, where CDC is kept and this interface is skipped (no endpoint room for both) -- see config.h
// (g_debugCdcThisBoot) and ARCHITECTURE.md.
#pragma once

void wakeHidBegin();   // register the boot-keyboard wake interface (call from setup() for clean modes)
