// bt_control.h — Bluetooth control surface used by the WebUSB panel.
//
// The panel drives scanning / pairing / forgetting through these calls, and the
// status frame reads per-slot info from bt_slot_info(). Phase 1 links weak
// no-op defaults; Phase 2 (bt_host.c) provides the real implementations.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_BT_CONTROL_H
#define PICOPUCK_BT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// One discovered (not yet paired) device, for the panel's scan list.
typedef struct {
	uint8_t addr[6];
	uint8_t addr_type;  // 0 = Classic, 1 = BLE public, 2 = BLE random
	uint8_t kind;       // 0 unknown, 1 SC2, 2 BLE gamepad, 3 Classic gamepad
	int8_t rssi;
	char name[16];
} bt_scan_entry_t;

// Per-slot live info for the status frame (kind/rssi/addr/name of the bound
// controller). Returns false if the slot has no bound controller.
bool bt_slot_info(int slot, uint8_t *kind, int8_t *rssi, uint8_t addr[6],
		  char name[16]);

void bt_scan_start(uint16_t seconds);
void bt_scan_stop(void);
bool bt_scan_active(void);

// Diagnostics for the panel. flags bits: 0 scan running, 1 HCI working,
// 2 connect in flight, 3 btstack init ok, 4 power-on issued. hci_state = last
// BTSTACK_EVENT_STATE (0 off, 1 initialising, 2 working). hci_events = total
// HCI events seen (0 ⇒ run loop dead).
void bt_scan_diag(uint16_t *adv_seen, uint8_t *flags, uint8_t *hci_state,
		  uint16_t *hci_events);

// Copy up to `max` scan entries into `out`; returns the count written.
uint8_t bt_scan_list(bt_scan_entry_t *out, uint8_t max);

bool bt_pair(const uint8_t addr[6], uint8_t addr_type);
void bt_forget(const uint8_t addr[6], uint8_t addr_type);
void bt_forget_all(void);
void bt_disconnect_slot(int slot);

#endif // PICOPUCK_BT_CONTROL_H
