// webusb.h — WebUSB config channel for the OpenPuck panel.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_WEBUSB_H
#define PICOPUCK_WEBUSB_H

#include <stdbool.h>

// Protocol version reported in the status frame; bump when the frame layout or
// opcode set changes so the panel can gate features.
#define PP_WEBUSB_VER 1

void webusb_init(void);

// Drain the vendor OUT endpoint, dispatch opcodes, and send any queued frames.
// Call each loop.
void webusb_task(void);

// Set from the vendor class request (0x22) when the panel attaches/detaches.
void webusb_set_connected(bool connected);

#endif // PICOPUCK_WEBUSB_H
