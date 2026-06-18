// mode_switch_hori.h -- Switch HORIPAD / Pokkén Tournament Pro Pad personality (MODE_SW_HORI).
//
// Targets a REAL Switch console: a plain HID gamepad (HORI 0F0D:0092) the console accepts on plug-in with NO
// handshake / SPI calibration / encryption -- unlike the genuine Pro Controller. This is the descriptor every
// DIY Switch controller copies (the LUFA "Splatoon printer" descriptor); also recognized by SDL/Steam on PC.
// 8-byte input report streamed at ~250Hz from task().
#pragma once
#include "controllers.h"

class SwitchHoriController : public IController {
    public:
	void begin() override;
	void task() override;
};
extern SwitchHoriController g_switchHori;
