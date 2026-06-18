// controllers.h -- the emulated-USB-controller abstraction.
//
// Each USB personality (Steam puck, Xbox/XInput, Switch HORIPAD, Switch Pro, PS5 DualSense, DS4 gyro) is an
// IController living in its own mode_*.cpp. Exactly ONE is active per boot, selected by g_usbMode. Methods:
//   begin()       -- called once from setup(): set the USB VID/PID/strings and register this mode's
//                    interface(s). Shared USB lifecycle (detach, clear/keep CDC, serial suffix, WebUSB,
//                    attach) stays in setup(); begin() only adds what's specific to this controller.
//   onReport45()  -- called from rf_link.cpp each time a controller input report 0x45 is decoded. PUSH-style
//                    modes (Xbox, puck/lizard) build + send their host report here. STREAM-style modes ignore
//                    it and emit at a fixed cadence from task(); they read the decoded g_in directly.
//   task()        -- called every loop(): streaming emit, handshake/subcommand draining, mode-specific upkeep.
//   isPuck()      -- true for Steam/Lizard (keep the boot CDC composite; different USB lifecycle in setup()).
//
// To add a new controller: implement IController in a new mode_*.cpp, give it a singleton, and wire it into
// controllerFor() in controllers.cpp.
#pragma once
#include <stdint.h>

class IController {
    public:
	virtual ~IController()
	{
	}
	virtual void begin() = 0;
	virtual void onReport45(const uint8_t *rep, bool fresh,
				uint8_t bodyTlen)
	{
		(void)rep;
		(void)fresh;
		(void)bodyTlen;
	}
	// Other controller->host input reports decoded from the F1 reply (NOT 0x45): power/battery status report
	// 0x43, status event 0x44. The real puck forwards these to Steam verbatim -- that's how Steam reads battery.
	// rid = report id, data/n = body after the id. Default no-op (clean modes don't expose these reports).
	virtual void onAuxReport(uint8_t rid, const uint8_t *data, uint8_t n)
	{
		(void)rid;
		(void)data;
		(void)n;
	}
	virtual void task()
	{
	}

	// a wake gesture fired while suspended; queue any post-resume input the host needs to actually wake
	virtual void wakeEvent()
	{
	}
	virtual bool isPuck() const
	{
		return false;
	}
};

// The singleton for a given mode (nullptr if the mode is unknown). Defined in controllers.cpp.
IController *controllerFor(uint8_t mode);

// The active controller for this boot (set in setup()). rf_link/loop dispatch through it.
extern IController *g_active;
