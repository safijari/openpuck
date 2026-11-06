// input_idle.h -- is anyone actively using a controller right now?
//
// A settings save erases and rewrites whole 4 KB flash pages; the CPU stalls while the flash controller works,
// so no polls go out and input freezes for roughly a tenth to a few tenths of a second. Saves that happen on
// their own during play (the startup-channel learner, paddle-profile changes) wait for a quiet moment instead.
#pragma once
#include <stdint.h>
#include "triton.h"

// No input on any connected controller for this long counts as a quiet moment.
#define INPUT_IDLE_QUIET_MS 2000u

// rf_link, once per decoded input report. rawButtons = the report's TB_* bits before any masking.
void inputIdleNote(uint8_t slot, uint32_t rawButtons, const PuckInput &in);
// true when no connected controller has been touched for INPUT_IDLE_QUIET_MS, or none is connected
bool inputIdleQuiet();
