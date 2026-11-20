// mode_ps5_auth.h -- real-PS5 personality (MODE_PS5_AUTH) via a "P5General" auth adapter.
//
// A PS5 only accepts a third-party wired pad that passes Sony's challenge/response, and it rejects input
// reports that aren't signed. Neither can be done in firmware, so this mode borrows a licensed
// P5General-class adapter (e.g. the P5 NEX Plus, LiteEMF 2B81:0101) hung off a MAX3421E USB-host chip on
// SPI -- the nRF52840's own USB controller is device-only and is already facing the console.
//
// Towards the PS5 we present the adapter's own identity and HID descriptor. Its auth feature reports
// (F0 challenge in, F2 status / F1 signature out) are relayed to the adapter, and every input report is sent
// to the adapter as an interrupt-OUT report and comes back with a sequence number and an 8-byte hash
// filled in; that signed copy is what the console receives. Same scheme as GP2040-CE's P5General mode.
#pragma once
#include "controllers.h"

class Ps5AuthController : public IController {
    public:
	// Static mount like PS3: a console wants one fixed gamepad interface from plug-in.
	void begin() override;
	void task() override;
	void usbIdentity() override;
};
extern Ps5AuthController g_ps5AuthCtl;
