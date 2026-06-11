// controllers.h -- the emulated-USB-controller abstraction.
//
// Each USB personality (Steam puck, Xbox/XInput, Switch HORIPAD, Switch Pro, PS5 DualSense, DS4 gyro) is an
// IController living in its own mode_*.cpp. Exactly ONE is active per boot, selected by g_usbMode. The split
// of responsibilities:
//   begin()       -- called once from setup(): set the USB device VID/PID/strings and register this mode's
//                    interface(s). The shared USB lifecycle (detach, clear/keep CDC, serial suffix, WebUSB,
//                    attach) stays in setup(); begin() only adds what's specific to this controller.
//   onReport45()  -- called from rf_link.cpp each time a controller input report 0x45 is decoded. PUSH-style
//                    modes (Xbox, puck/lizard) build + send their host report here. STREAM-style modes ignore
//                    it and instead emit at a fixed cadence from task(); they read the decoded g_in directly.
//   task()        -- called every loop(): streaming emit, handshake/subcommand draining, mode-specific upkeep.
//   isPuck()      -- true for Steam/Lizard (keep the boot CDC composite; different USB lifecycle in setup()).
//
// To add a new controller: implement IController in a new mode_*.cpp, give it a singleton, and wire it into
// controllerFor() in controllers.cpp. Nothing else needs to change.
#pragma once
#include <stdint.h>

class IController {
public:
  virtual ~IController() {}
  virtual void begin() = 0;
  virtual void onReport45(const uint8_t* rep, bool fresh, uint8_t bodyTlen) { (void)rep; (void)fresh; (void)bodyTlen; }
  virtual void task() {}
  virtual void wakeEvent() {}   // a wake gesture fired while suspended; queue any post-resume input the host needs to actually wake
  virtual bool isPuck() const { return false; }
};

// The singleton for a given mode (nullptr if the mode is unknown). Defined in controllers.cpp.
IController* controllerFor(uint8_t mode);

// The active controller for this boot (set in setup()). rf_link/loop dispatch through it.
extern IController* g_active;
