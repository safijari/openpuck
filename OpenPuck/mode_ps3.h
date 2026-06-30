// mode_ps3.h -- Sony DualShock 3 / Sixaxis personality (MODE_PS3): 054C:0268.
//
// Unlike the PC-facing PS modes, this one is built to enumerate on a REAL PlayStation 3 console. The PS3 does
// NOT parse the HID report descriptor to drive input the way a PC does -- it recognises the Sixaxis by VID/PID
// and a fixed control-transfer "enable" handshake. So this mode replicates the genuine Sixaxis byte-for-byte:
// the exact 148-byte HID report descriptor, the GET_REPORT(Feature) magic reports the console reads during
// enable (0x01 calibration, 0xEF/0xF2 device+MAC info, 0xF5/0xF7/0xF8 status), and the SET_REPORT(0xF4)
// operational command. It then streams the 49-byte Sixaxis input report (id 0x01): buttons, sticks, analog
// pressures, accelerometer X/Y/Z + gyro Z, and a USB-plugged/charged status block. Rumble + LED arrive on the
// 0x01 output report. Clean single-HID device (no wake mouse / WebUSB) -- the console wants a bare Sixaxis.
#pragma once
#include "controllers.h"

class Ps3Controller : public IController {
    public:
	// STATIC mount (NOT dynamicMount): a real PS3 rejects the empty config a dynamic mode shows at plug-in
	// before a controller has linked, and dislikes the later detach/re-attach. A genuine Sixaxis is always
	// exactly one interface, so we present the single DS3 HID permanently from begin() and just stream neutral
	// reports until a controller connects. begin() also sets the USB identity (the static setup path, unlike
	// usbReenumerate, never calls usbIdentity()).
	void begin() override;
	void task() override;
	void usbIdentity() override;
};
extern Ps3Controller g_ps3Ctl;
