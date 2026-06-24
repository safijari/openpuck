// report45.h -- encode g_in into the controller's HID input report 0x45.
//
// The exact inverse of the field offsets in triton.h (s16off/u16off/trigU8/btnsOf/imuFrom45). The
// puck's rf_link.cpp decodes this same byte layout, so the round-trip Deck -> g_in -> 0x45 -> puck ->
// Steam is symmetric. Output is REPORT45_LEN bytes: [0x45][seq][buttons u32][...sticks/pads/IMU].
#pragma once
#include <stdint.h>

// 46 bytes: id + 45 payload bytes (matches the puck descriptor's report-0x45 count 0x2D and the
// IMU tail at rep[34..46] that rf_link decodes when tlen >= 46).
#define REPORT45_LEN 46

// Build a fresh report 0x45 from g_in into out[REPORT45_LEN]. seq is the rolling frame counter
// (rep[1]); the puck treats a changed seq as a fresh report.
void buildReport45(uint8_t *out, uint8_t seq);
