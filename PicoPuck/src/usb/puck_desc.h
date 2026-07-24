// puck_desc.h — the cloned Valve puck HID report descriptor.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_PUCK_DESC_H
#define PICOPUCK_PUCK_DESC_H

#include <stdint.h>
#include <stddef.h>

// Byte-for-byte the descriptor a real Valve puck presents on each of its four
// control interfaces: mouse(0x40) + keyboard(0x41) + vendor(FF00) with input
// reports 0x42/0x43/0x44/0x45/0x79/0x7B, output reports 0x80-0x89, and 63-byte
// FEATURE command reports on report ids 0x01/0x02. Copied verbatim from
// OpenPuck/puck_hid.cpp so Steam's SDL driver sees an identical device.
// Compile-time size, needed by the config-descriptor macros. Verified against
// the actual array with a static assert in puck_desc.c.
#define PUCK_HID_DESC_SIZE 372

extern const uint8_t PUCK_HID_DESC[];
extern const size_t PUCK_HID_DESC_LEN;

#endif // PICOPUCK_PUCK_DESC_H
