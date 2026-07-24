// slots.h — the four puck bond slots and their live state.
//
// A slot models one of the puck's four HID interfaces. It carries the bond
// record Steam reads/writes (0xA2/0xA3), a settings shadow for 0x87/0x89
// read-back, and the live-connection bookkeeping the personality uses to drive
// the 0x79 / 0xB4 / 0x43 connection reporting. Bluetooth controllers are bound
// to slots in later phases; a real BT link sets `connected`.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_SLOTS_H
#define PICOPUCK_SLOTS_H

#include <stdint.h>
#include <stdbool.h>
#include "config/picopuck_config.h"

#define PP_BOND_REC_LEN 24
#define PP_SETTINGS_MAX 0x53  // 0x87 setting-id space (id < 0x53 shadowed)

typedef struct {
	bool used;                      // a bond record is present (paired)
	bool connected;                 // a BT controller is live on this slot
	uint8_t rec[PP_BOND_REC_LEN];   // Steam's 0xA2/0xA3 bond view
	uint16_t set_shadow[PP_SETTINGS_MAX];  // 0x87 values, read back by 0x89
	uint32_t conn_reply_ms;         // ms of last input report (0 = none)

	// Staged feature-GET reply (built in a feature SET, read by the GET).
	uint8_t resp[63];
	uint8_t resp_len;
} puck_slot_t;

extern puck_slot_t g_slot[PP_NSLOT];

// Per-slot battery, set by the bound controller (BAS / report). Percent 0-100;
// state: 0 = discharging, 1 = charging (mirrors OpenPuck g_battery/g_batteryState).
extern uint8_t g_battery[PP_NSLOT];
extern uint8_t g_battery_state[PP_NSLOT];

// True while a controller is currently presented on this slot (connected and
// producing input recently). Drives 0xB4 / 0x79.
bool slot_is_live(int slot, uint32_t now_ms);

// Mark that a fresh input report arrived for `slot` (updates conn_reply_ms).
void slot_note_input(int slot, uint32_t now_ms);

void slots_init(void);

#endif // PICOPUCK_SLOTS_H
