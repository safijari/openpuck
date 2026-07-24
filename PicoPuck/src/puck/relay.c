// relay.c — host→controller relay dispatch (see relay.h).
//
// The actual routing lives in the BT host (bt_relay), which knows what is bound
// to each slot: a generic pad's rumble output, or an SC2's Valve report
// characteristic. bt_relay is weak so a BT-less build still links.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "puck/relay.h"

__attribute__((weak)) void bt_relay(int slot, uint8_t cmd, const uint8_t *payload,
				    uint16_t len)
{
	(void)slot;
	(void)cmd;
	(void)payload;
	(void)len;
}

void relay_enqueue(int slot, uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	bt_relay(slot, cmd, payload, len);
}
