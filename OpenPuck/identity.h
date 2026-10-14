// identity.h -- device identity derived from the nRF52 FICR DEVICEID.
//
// A UNIQUE serial (never clashes with a real puck) is generated at boot. Steam mode reports g_unit as its
// USB serial (its pairing identity); the other modes append a 1-char suffix (setup() in OpenPuck.ino).
#pragma once
#include <stdint.h>

extern char g_unit[16]; // "FXB99602xxxxx"
extern char g_board[16]; // "MXB99602xxxxx"
extern char g_usbSerial
	[18]; // per-mode USB serial (Steam uses g_unit; others get a suffix)

// 0x83 attributes, returned by the feature command channel.
extern const uint8_t ATTR83_MACHINE[25];
extern const uint8_t ATTR83_PUCK[25];

void genSerial();
