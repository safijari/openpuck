// identity.c — puck identity (see identity.h).
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "puck/identity.h"

#include <stdio.h>
#include <string.h>
#include "pico/unique_id.h"

// 0x83 attributes (product 0x1304, stored as u32 LE at offset 1). Verbatim from
// OpenPuck/identity.cpp — Steam matches the puck by this blob.
const uint8_t ATTR83[25] = {
	0x01, 0x04, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
	0x00, 0x0A, 0xF2, 0xF9, 0xD2, 0x68, 0x04, 0x53, 0xD0,
	0x18, 0x6A, 0x09, 0x47, 0x00, 0x00, 0x00
};

char g_board[16];
char g_unit[16];
char g_usb_serial[18];

void identity_init(void)
{
	// Fold the 64-bit unique flash id into 20 bits, matching the shape of the
	// nRF puck's FICR-derived serials ("FXB.../MXB..." + 5 hex digits).
	pico_unique_board_id_t id;
	pico_get_unique_board_id(&id);
	uint32_t x = 0;
	for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++)
		x = (x * 31u) + id.id[i];
	unsigned tail = (unsigned)(x & 0xFFFFFu);

	snprintf(g_unit, sizeof g_unit, "FXB99602%05X", tail);
	snprintf(g_board, sizeof g_board, "MXB99602%05X", tail);
	snprintf(g_usb_serial, sizeof g_usb_serial, "PPK%08X", (unsigned)x);
}
