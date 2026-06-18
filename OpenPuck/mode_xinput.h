// mode_xinput.h -- Xbox 360 wired (XInput) personality (MODE_XBOX): XInput gamepad + right-pad mouse.
//
// Real Xbox 360 pads are NOT HID: they use a vendor interface (class 0xFF / sub 0x5D / proto 0x01) carrying a
// 20-byte XInput report. A custom TinyUSB class driver serves that interface plus a second boot-mouse interface
// for the right trackpad. Host binds by VID/PID (045E:028E) + the FF/5D/01 interface. The OUT endpoint carries
// rumble, relayed to the controller as a haptic by task().
#pragma once
#include "controllers.h"
#include <stdint.h>

class XboxController : public IController {
    public:
	void begin() override;
	void onReport45(const uint8_t *rep, bool fresh,
			uint8_t bodyTlen) override;

	// legacy XInput rumble -> haptic relay + stuck-rumble watchdog
	void task() override;
};
extern XboxController g_xboxCtl;
