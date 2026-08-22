// mode_sinput.h -- SInput personality (MODE_SINPUT): the open SDL-native gamepad protocol.
//
// SInput (Hand Held Legend, docs.handheldlegend.com/s/sinput) is a HID protocol SDL3 and Steam Input bind
// natively via a dedicated driver, so a device can expose analog sticks, analog triggers, gyro/accel, TWO
// touchpads and battery at once without pretending to be somebody else's controller -- which is exactly the
// SC2's shape. Enumerates as the generic SInput device 2E8A:10C6 (the ID SDL's driver matches on).
#pragma once
#include "controllers.h"

class SInputController : public IController {
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

extern SInputController g_sinputCtl;
