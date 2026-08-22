// mode_dinput.h -- generic DirectInput joystick personality (MODE_DINPUT).
//
// Flight/space sims bind axes through DirectInput, which is limited to 8 axes per device (DIJOYSTATE2), while
// the SC2 offers 13 analog inputs. This mode therefore presents each controller as TWO joysticks (two top-level
// HID collections on one interface -- Windows creates one HIDClass PDO, and thus one DirectInput device, per
// top-level collection) so every axis is bindable at the same time. See mode_dinput.cpp for the axis map.
#pragma once
#include "controllers.h"

class DInputController : public IController {
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

extern DInputController g_dinputCtl;
