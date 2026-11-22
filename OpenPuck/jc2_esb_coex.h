// jc2_esb_coex.h -- 2.4 GHz Steam Controller ESB gap scheduler running inside
// libjoycon2's bare-metal BLE idle windows.
#pragma once

#include <stdbool.h>
#include <stdint.h>

void jc2EsbCoexOnBleStart(bool was_link_up);
void jc2EsbCoexStep(bool game_ready, void (*on_slot_drop)(int slot));
uint32_t jc2EsbCoexPadSampleSeq(uint8_t slot);
void jc2EsbCoexPrintStatus(void);
