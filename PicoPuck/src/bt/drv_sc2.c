// drv_sc2.c — Steam Controller 2 identity (BLE, Valve GATT).
//
// The SC2's input is forwarded verbatim by the Valve GATT client (bt_valve.c),
// not decoded here — so this driver exists only to (a) claim the device so the
// host routes it to the Valve path and (b) label it "Steam Controller 2" (kind 1)
// in the panel. A future emulated-mode path could add a real decode().
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "bt/input_driver.h"
#include <string.h>

static bool sc2_match(uint16_t vid, uint16_t pid, const char *devname, bool is_ble)
{
	(void)pid;
	if (!is_ble)
		return false;
	if (vid == 0x28DE)
		return true;
	return devname && strstr(devname, "Steam") != NULL;
}

static bool sc2_decode(uint8_t rid, const uint8_t *d, uint16_t len, puck_input_t *io)
{
	(void)rid;
	(void)d;
	(void)len;
	(void)io;
	return false;  // handled verbatim by bt_valve.c
}

const input_driver_t drv_sc2 = {
	.name = "Steam Controller 2 (BLE)",
	.kind = 1,
	.match = sc2_match,
	.decode = sc2_decode,
	.build_rumble = NULL,
	.rumble_report_id = 0,
};
