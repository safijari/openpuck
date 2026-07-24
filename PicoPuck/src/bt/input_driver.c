// input_driver.c — driver registry (see input_driver.h).
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "bt/input_driver.h"
#include <stddef.h>

extern const input_driver_t drv_sc2;
extern const input_driver_t drv_xinput;
extern const input_driver_t drv_ds4;
// Future Classic pads (same interface): drv_ds5 (needs CRC32 output + adaptive
// triggers) and drv_switch_pro (needs a subcommand init handshake).

// Most specific first: SC2 (by name/VID) before the generic BLE/Classic matchers.
static const input_driver_t *const REGISTRY[] = {
	&drv_sc2,
	&drv_xinput,
	&drv_ds4,
};

const input_driver_t *input_driver_match(uint16_t vid, uint16_t pid,
					 const char *devname, bool is_ble)
{
	for (size_t i = 0; i < sizeof(REGISTRY) / sizeof(REGISTRY[0]); i++) {
		const input_driver_t *d = REGISTRY[i];
		if (d->match && d->match(vid, pid, devname, is_ble))
			return d;
	}
	return NULL;
}
