// pad_dpad.h -- a trackpad set to "D-pad" (g_padStick[pad] == PS_DPAD) presses D-pad directions when clicked.
//
// Clicking, not touching, so a thumb resting on the pad presses nothing. The click point picks the direction:
// 8-way, with diagonals pressing two directions, and the centre of the pad pressing none. The pad's own click
// is consumed, and modes that would forward the pad as a touchpad already skip pads mapped to anything.
#pragma once
#include <stdint.h>

// rf_link, once per decoded input report, before the gesture hooks and the active controller see it.
// Adds the D-pad bits to g_in[slot] and to the raw report (push-style modes read rep).
void padDpadApply(uint8_t slot, uint8_t *rep);
