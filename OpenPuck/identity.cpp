#include "identity.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

char g_unit[16];
char g_board[16];
char g_usbSerial[18];

const uint8_t ATTR83[25] = {     // 0x83 attributes (product 0x1304 = puck)
  0x01,0x04,0x13,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x0A,0xF2,0xF9,0xD2,0x68,
  0x04,0x53,0xD0,0x18,0x6A,0x09,0x47,0x00,0x00,0x00 };

void genSerial() {
  uint32_t id = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1];
  snprintf(g_unit,  sizeof g_unit,  "FXB99602%05lX", (unsigned long)(id & 0xFFFFF));
  snprintf(g_board, sizeof g_board, "MXB99602%05lX", (unsigned long)(id & 0xFFFFF));
}
