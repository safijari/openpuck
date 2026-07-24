// input_driver.h — per-controller decode/rumble interface.
//
// A driver claims a connected controller (by VID/PID/name/transport), decodes
// its HID input reports into a puck_input_t, and builds a rumble output report.
// hids_client (BLE) and the Classic HID host both deliver reports as
// (report_id, data, len), so drivers key off report_id directly.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_INPUT_DRIVER_H
#define PICOPUCK_INPUT_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "puck/triton.h"

typedef struct input_driver {
	const char *name;
	uint8_t kind;  // matches bt_control kind: 1 SC2, 2 BLE gamepad, 3 Classic gamepad

	// Claim this device? `is_ble` distinguishes BLE from Classic transports.
	bool (*match)(uint16_t vid, uint16_t pid, const char *devname, bool is_ble);

	// Decode one input report into `io` (accumulating). Returns true if `io`
	// changed and should be presented.
	bool (*decode)(uint8_t report_id, const uint8_t *d, uint16_t len,
		       puck_input_t *io);

	// Build a rumble output report from 16-bit magnitudes. Returns its length
	// (0 = no rumble); the report id to send it on is `rumble_report_id`.
	uint8_t (*build_rumble)(uint16_t lo, uint16_t hi, uint8_t *out, uint8_t max);
	uint8_t rumble_report_id;
} input_driver_t;

// First driver whose match() accepts the device, or NULL.
const input_driver_t *input_driver_match(uint16_t vid, uint16_t pid,
					 const char *devname, bool is_ble);

#endif // PICOPUCK_INPUT_DRIVER_H
