// identity.h — puck identity: 0x83 attribute blob and derived serials.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_IDENTITY_H
#define PICOPUCK_IDENTITY_H

#include <stdint.h>

// The 25-byte 0x83 GET_ATTRIBUTES payload (product 0x1304 = Valve puck), copied
// from OpenPuck/identity.cpp. Steam reads this to identify the dongle.
extern const uint8_t ATTR83[25];

// Board / unit serial strings (Valve "MXB..." / "FXB..." shape), derived from
// the RP2040/RP2350 unique flash id at boot. NUL-terminated, <=15 chars.
extern char g_board[16];
extern char g_unit[16];

// USB string-descriptor serial (also derived from the unique id).
extern char g_usb_serial[18];

// Populate the serials. Call once at boot before USB init.
void identity_init(void);

#endif // PICOPUCK_IDENTITY_H
