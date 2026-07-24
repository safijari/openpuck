// slots.c — puck bond slots and live state (see slots.h).
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "puck/slots.h"
#include <string.h>

puck_slot_t g_slot[PP_NSLOT];
uint8_t g_battery[PP_NSLOT];
uint8_t g_battery_state[PP_NSLOT];

// A slot counts as "live" for a short window after its last input report, so a
// brief BT hiccup doesn't bounce the connection state Steam sees. Mirrors the
// 500 ms window OpenPuck uses for 0xB4.
#define SLOT_LIVE_WINDOW_MS 500u

void slots_init(void)
{
	memset(g_slot, 0, sizeof(g_slot));
	memset(g_battery, 0, sizeof(g_battery));
	memset(g_battery_state, 0, sizeof(g_battery_state));
}

bool slot_is_live(int slot, uint32_t now_ms)
{
	if (slot < 0 || slot >= PP_NSLOT)
		return false;
	const puck_slot_t *s = &g_slot[slot];
	return s->connected && s->conn_reply_ms &&
	       (now_ms - s->conn_reply_ms < SLOT_LIVE_WINDOW_MS);
}

void slot_note_input(int slot, uint32_t now_ms)
{
	if (slot < 0 || slot >= PP_NSLOT)
		return;
	g_slot[slot].conn_reply_ms = now_ms ? now_ms : 1;
}
