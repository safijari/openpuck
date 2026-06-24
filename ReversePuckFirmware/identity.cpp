#include "identity.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

char g_unit[16];
char g_board[16];

// 0x83 attributes for the CONTROLLER -- the EXACT 25-byte blob read from a real Steam Controller
// (28DE:1302) over USB: `scmd 1302 --up 1 83` -> `01 83 19 <these 25 bytes>`. As [tag][u32-LE] records:
// tag 01 = product 0x1302; tag 02 = capabilities; tag 0A = bootloader build 0x68D2F92E; tag 04 = fw
// build 0x6A18D057; tag 09 = board rev 0x48. These are firmware/model values (shared across units, not
// per-device), so verbatim is correct; the per-unit serial lives in 0xAE instead. Steam validates these
// byte-for-byte -- two wrong bytes (the puck's bootloader build + board rev) were why it kept re-asking.
const uint8_t ATTR83[] = { 0x01, 0x02, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
			   0x00, 0x0A, 0x2E, 0xF9, 0xD2, 0x68, 0x04, 0x57, 0xD0,
			   0x18, 0x6A, 0x09, 0x48, 0x00, 0x00, 0x00 };
const uint16_t ATTR83_LEN = sizeof ATTR83;

void genSerial()
{
	uint32_t id = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1];
	snprintf(g_unit, sizeof g_unit, "FXA99602%05lX",
		 (unsigned long)(id & 0xFFFFF));
	snprintf(g_board, sizeof g_board, "MXA99602%05lX",
		 (unsigned long)(id & 0xFFFFF));
}
