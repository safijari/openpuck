// mode_ps5.h -- Sony DualSense personality (MODE_PS5): 054C:0CE6 + gyro + split trackpad.
//
// Streams the DualSense USB input report (id 0x01) at ~250Hz from task(): sticks, triggers, PS-layout buttons,
// gyro/accel, and BOTH Steam trackpads mapped onto the single DualSense touchpad (left pad -> left half, right
// pad -> right half) via the shared touch packers in gamepad_util.
#pragma once
#include "controllers.h"

class Ps5Controller : public IController {
public:
  void begin() override;
  void task() override;
};
extern Ps5Controller g_ps5Ctl;
