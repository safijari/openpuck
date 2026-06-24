// identity.h -- emulated-controller identity derived from the nRF52 FICR DEVICEID.
//
// Mirrors OpenPuck/identity.h but for the CONTROLLER (IBEX, USB PID 0x1302). A real wired Steam
// Controller enumerates as 28DE:1302 with an "FXA99602…" unit serial (HANDOFF.md). We generate a
// unique FXA/MXA serial so we never clash with a real controller, and expose a 0x1302 product blob.
#pragma once
#include <stdint.h>

// "FXA99602xxxxx" -- the controller's pairing identity (USB serial)
extern char g_unit[16];
extern char g_board[16]; // "MXA99602xxxxx"

// 0x83 attributes, returned by the feature command channel (product code 0x1302). Unsized so a
// captured real-controller blob of any length drops in; ATTR83_LEN carries the true byte count.
extern const uint8_t ATTR83[];
extern const uint16_t ATTR83_LEN;

void genSerial();
