// bt_control_stub.c — weak no-op BT control (see bt_control.h).
//
// Lets Phase 1 build and the panel connect before the BTstack host exists. Each
// symbol is weak; Phase 2's bt_host.c overrides it with the real implementation.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "bt/bt_control.h"

__attribute__((weak)) bool bt_slot_info(int slot, uint8_t *kind, int8_t *rssi,
					uint8_t addr[6], char name[16])
{
	(void)slot;
	(void)kind;
	(void)rssi;
	(void)addr;
	(void)name;
	return false;
}

__attribute__((weak)) void bt_scan_start(uint16_t seconds) { (void)seconds; }
__attribute__((weak)) void bt_scan_stop(void) {}
__attribute__((weak)) bool bt_scan_active(void) { return false; }

__attribute__((weak)) void bt_scan_diag(uint16_t *adv_seen, uint8_t *flags,
					uint8_t *hci_state, uint16_t *hci_events)
{
	if (adv_seen)
		*adv_seen = 0;
	if (flags)
		*flags = 0;
	if (hci_state)
		*hci_state = 0;
	if (hci_events)
		*hci_events = 0;
}

__attribute__((weak)) uint8_t bt_scan_list(bt_scan_entry_t *out, uint8_t max)
{
	(void)out;
	(void)max;
	return 0;
}

__attribute__((weak)) bool bt_pair(const uint8_t addr[6], uint8_t addr_type)
{
	(void)addr;
	(void)addr_type;
	return false;
}

__attribute__((weak)) void bt_forget(const uint8_t addr[6], uint8_t addr_type)
{
	(void)addr;
	(void)addr_type;
}

__attribute__((weak)) void bt_forget_all(void) {}
__attribute__((weak)) void bt_disconnect_slot(int slot) { (void)slot; }
