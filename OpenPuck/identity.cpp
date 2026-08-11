#include "identity.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

char g_unit[16];
char g_board[16];
char g_usbSerial[18];

// clang-format off
const uint8_t ATTR83[25] = { // 0x83 attributes (product 0x1304 = Proteus puck)
	// 1 byte tag, 4 byte little-endian value.
	0x01,  0x04, 0x13, 0x00, 0x00, 		// product_id = 0x1304
	0x02,  0x00, 0x00, 0x00, 0x00, 		// capabilities = 0 (bitfield)
	0x0A,  0xF2, 0xF9, 0xD2, 0x68,      // boot_build_timestamp = 0x68D2F9F2
	0x04,  0x53, 0xD0, 0x18, 0x6A,      // build_timestamp = 0x6A18D053 todo: should this be updated?
	0x09,  0x47, 0x00, 0x00, 0x00       // hw_id = 0x47
};
// clang-format on

void genSerial()
{
	uint32_t id = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1];
	snprintf(g_unit, sizeof g_unit, "FXB99602%05lX",
		 (unsigned long)(id & 0xFFFFF));
	snprintf(g_board, sizeof g_board, "MXB99602%05lX",
		 (unsigned long)(id & 0xFFFFF));
}
