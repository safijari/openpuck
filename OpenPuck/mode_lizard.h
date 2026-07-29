// mode_lizard.h -- configurable lizard (desktop) keyboard+mouse mode.
//
// Not a standalone IController: lizard rides ON the puck interface (puck_hid.cpp calls rfLizard
// when Steam is closed or in MODE_LIZARD), driving mouse(0x40)+keyboard(0x41)+consumer(0x03) on
// the same puck HID slot. The mapping table g_lizardMap (lizard_map.h) drives all outputs; the
// default map mirrors the original hardcoded Valve SC1 behavior. Mouse reuses the Xbox-mode
// velocity+friction+sub-pixel glide model (g_mDiv / g_mFric).
//
// EVERY connected controller drives lizard: rfLizard merges g_in[s] across all used bond slots
// (populated by the per-slot RF decode) onto the ONE shared desktop mouse/keyboard, so any controller
// on any bond slot contributes and multiple controllers drive the same cursor/keys together. Per-slot
// glide/edge state keeps one controller's motion from clobbering another's.
#pragma once
#include <Adafruit_TinyUSB.h>
#include <stdint.h>

// Drives the desktop mouse/keyboard from the binding table. Merges input from ALL bonded controllers
// (g_in[s] for every used slot) onto the one shared mouse/keyboard -- a controller on any RF slot
// contributes, and multiple controllers drive the same cursor/keys together. mdev/kdev may be the same
// object (puck mode sends both reports to hid[0]).
void rfLizard(Adafruit_USBD_HID *mdev, Adafruit_USBD_HID *kdev, uint8_t mrid,
	      uint8_t krid);

// Release any keyboard/mouse/consumer input lizard is currently holding on the host and reset rfLizard's
// internal last-sent + glide state. Call on the lizard->Steam handoff (Steam takes over the gamepad with no
// USB re-enumeration): without it a key/button held at that instant stays latched on the desktop until a
// reconnect or power-cycle. mdev/kdev may be the same object (puck mode uses hid[0] for both).
void rfLizardRelease(Adafruit_USBD_HID *mdev, Adafruit_USBD_HID *kdev,
		     uint8_t mrid, uint8_t krid);
