// mode_hidgyro.h -- generic HID gyro personality (MODE_HIDGYRO): DS4 (054C:05C4) USB report layout + motion.
//
// A standard DS4 descriptor (input report 0x01, 64 B) with gyro/accel at bytes 12-23 (GIMX layout). Aimed at
// motion-aware PC games (e.g. Fortnite) that read the DS4 report directly. Streamed at ~250Hz from task();
// reuses the shared PS-layout button packers + Steam-trackpad touch mapping.
#pragma once
#include "controllers.h"

class HidGyroController : public IController {
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
extern HidGyroController g_hidGyroCtl;
