// mode_lizard.h -- configurable lizard (desktop) keyboard+mouse mode.
//
// Not a standalone IController: lizard rides ON the puck interface (puck_hid.cpp calls rfLizard
// when Steam is closed or in MODE_LIZARD), driving mouse(0x40)+keyboard(0x41)+consumer(0x03) on
// the same puck HID slot. The mapping table g_lizardMap (lizard_map.h) drives all outputs; the
// default map mirrors the original hardcoded Valve SC1 behavior. Mouse reuses the Xbox-mode
// velocity+friction+sub-pixel glide model (g_mDiv / g_mFric).
#pragma once
#include <Adafruit_TinyUSB.h>
#include <stdint.h>

// Drives the desktop mouse/keyboard from the binding table. Merges input from ALL bonded controllers
// (g_in[s] for every used slot) onto the one shared mouse/keyboard -- a controller on any RF slot
// contributes, and multiple controllers drive the same cursor/keys together. mdev/kdev may be the same
// object (puck mode sends both reports to hid[0]).
void rfLizard(Adafruit_USBD_HID *mdev, Adafruit_USBD_HID *kdev, uint8_t mrid,
	      uint8_t krid);
