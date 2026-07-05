// drv_xinput_ble.h -- input driver for the Xbox Bluetooth HID gamepad report layout ("xinput mode").
//
// This is the layout the 8BitDo SN30 Pro speaks in its X-input (Start+X) mode: it impersonates an Xbox One S
// controller, VID 045E PID 02E0, input report 0x01, rumble output report 0x03 (confirmed by the Linux
// hid-microsoft driver, which quirks 8BitDo pads via the Microsoft ids). NOTE the plain SN30 Pro only speaks it
// over Bluetooth CLASSIC (BR/EDR), which the nRF52840 radio cannot do -- so the pads this driver actually
// connects to over BLE are the real Xbox controllers that use the same layout over HID-over-GATT:
//   - Xbox Series X|S controller (model 1914) -- BLE always (fw >= 5.15)
//   - Xbox One S controller (model 1708) -- BLE after the firmware 5.x update
//   - Xbox Adaptive Controller
// If 8BitDo ever ships a BLE mode it lands here for free (same report bytes, same Microsoft ids).
#pragma once
#include "input_driver.h"

class XInputBleDriver : public IInputDriver {
    public:
	const char *name() const override
	{
		return "xinput";
	}
	bool match(uint16_t vid, uint16_t pid,
		   const char *devName) const override;
	uint8_t inputRidByIndex(uint8_t idx) const override;
	bool decode(int slot, uint8_t rid, const uint8_t *d, uint16_t len,
		    PuckInput *io) override;
	uint8_t buildRumble(uint16_t lo, uint16_t hi, uint8_t out[16]) override;
};
extern XInputBleDriver g_drvXInputBle;
