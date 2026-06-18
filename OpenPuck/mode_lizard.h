// mode_lizard.h -- the controller's native "lizard" desktop behavior (driverless keyboard + mouse).
//
// Not a standalone IController: lizard rides ON the puck interface (puck_hid.cpp calls rfLizard when Steam is
// closed), driving mouse(0x40)+keyboard(0x41) on the same puck HID slot. Canonical Valve SC1 map: right pad ->
// mouse (trackball glide), left pad -> scroll wheel + middle-click, R-trigger/R-pad-click -> left mouse,
// L-trigger -> right mouse; A=Enter B=Esc X=PgUp Y=PgDn, d-pad/left-stick -> arrows, LB=LeftCtrl RB=LeftAlt,
// View=Tab, Menu=Esc. Mouse reuses the Xbox-mode velocity+friction+sub-pixel glide model (g_mDiv / g_mFric).
#pragma once
#include <Adafruit_TinyUSB.h>
#include <stdint.h>

void rfLizard(const uint8_t *r, Adafruit_USBD_HID *mdev,
	      Adafruit_USBD_HID *kdev, uint8_t mrid, uint8_t krid);
