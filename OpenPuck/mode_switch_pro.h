// mode_switch_pro.h -- Nintendo Switch Pro Controller personality (MODE_SW_PRO): 057E:2009, report 0x30 + IMU.
//
// The genuine Pro Controller requires a USB handshake + subcommand exchange (device info, SPI calibration reads,
// set-report-mode) before any host (or Steam) will bind it. jcSet() answers that handshake from the USB ISR by
// enqueuing canonical 0x81/0x21 replies; task() drains them (and, once the host selects report mode 0x30, streams
// the standard input report with gyro). Linux hid-nintendo hardcodes the 0x30 layout, so we follow it exactly.
// macOS drops input reports shorter than the descriptor length, so every reply is padded to the full 63 bytes.
#pragma once
#include "controllers.h"

class SwitchProController : public IController {
    public:
	void begin() override;
	void task() override;
	bool dynamicMount() const override
	{
		return true;
	}
	uint8_t maxSlots() const override;
	void usbIdentity() override;
	void beginPool() override;
	void mountSlots(uint8_t k) override;
};
extern SwitchProController g_switchPro;
