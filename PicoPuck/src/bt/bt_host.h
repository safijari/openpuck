// bt_host.h — BTstack dual-mode (Classic + BLE) controller host.
//
// Owns the CYW43 radio via BTstack, discovers game controllers, and (in later
// increments) routes their input into the puck slots. Runs on the CYW43
// async_context, which is serviced by cyw43_arch_poll() in the main loop.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_BT_HOST_H
#define PICOPUCK_BT_HOST_H

#include <stdbool.h>

// Bring up BTstack on the (already-initialised) CYW43 radio and power on the
// controller. Returns false if BTstack init failed.
bool bt_host_init(void);

// True once the HCI stack has reached the working state.
bool bt_host_ready(void);

// Periodic housekeeping: scan timeout + rumble flush. Call each loop.
void bt_host_task(void);

#endif // PICOPUCK_BT_HOST_H
