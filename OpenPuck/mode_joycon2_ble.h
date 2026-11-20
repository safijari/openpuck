// mode_joycon2_ble.h -- Nintendo Switch 2 Joy-Con 2 BLE + Optical Mouse mode.
#pragma once

#include "controllers.h"
#include "jc2_pad_mouse.h"
#include "src/libjoycon2/joycon2.h"

class JoyCon2BleController : public IController {
    public:
	void begin() override;
	void task() override;
	bool pollRadioCoex() override;
};

extern JoyCon2BleController g_joycon2Ble;

// Map OpenPuck's decoded g_in[slot] and active ET_SWITCH config into a
// controller-agnostic libjoycon2 input struct.
jc2_input_t jc2InputFromSlot(uint8_t slot);

// Force a fresh Dual Joy-Con 2 (L + R) BLE pairing session or pulse L+R for
// Switch 2 "Change Grip/Order" registration.
void jc2TriggerPairing(bool clear_bonds);
void jc2PulseGripLR(uint32_t duration_ms);

// Handle Joy-Con 2 serial console commands (JC2_STATUS, JC2_PAIR).
bool jc2HandleConsoleCmd(const char *line);
