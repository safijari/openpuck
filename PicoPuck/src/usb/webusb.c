// webusb.c — WebUSB config channel (see webusb.h).
//
// Host→device opcodes (each a single bulk-OUT packet):
//   0x01                       get status          → 0xA5 frame
//   0x02 [secs]                scan start
//   0x03                       scan stop
//   0x04                       get scan list       → 0xA6 frame
//   0x05 [addr6][type]         pair / connect
//   0x06 [addr6][type]         forget one bond
//   0x07                       forget all bonds
//   0x08 [slot]                disconnect a slot
//   0x0A                       reboot
//   0x0B [0x42 0x4C]           reboot to UF2 bootloader (BOOTSEL)
//   0x0C [0xDE 0xAD]           factory reset (bonds + settings)
// Device→host frames: 0xA5 status, 0xA6 scan list.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "usb/webusb.h"
#include "config/picopuck_config.h"
#include "puck/slots.h"
#include "bt/bt_control.h"

#include <string.h>
#include "tusb.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"

static bool s_connected;

// Slot record: state, kind, battery, rssi, bd_addr[6], name[16] = 26 bytes.
#define SLOT_REC_LEN 26
// Scan record: kind, addr_type, rssi, bd_addr[6], name[16] = 25 bytes.
#define SCAN_REC_LEN 25
#define SCAN_MAX 12

void webusb_init(void)
{
	s_connected = false;
}

void webusb_set_connected(bool connected)
{
	s_connected = connected;
}

// ---- frame senders ---------------------------------------------------------
static void send_frame(const uint8_t *frame, uint16_t len)
{
	// Drop rather than block if the panel isn't draining (avoids wedging loop).
	if (tud_vendor_write_available() < len)
		return;
	tud_vendor_write(frame, len);
	tud_vendor_write_flush();
}

static void send_status(void)
{
	uint8_t f[2 + 17 + PP_NSLOT * SLOT_REC_LEN + 6];
	uint8_t *p = f;
	*p++ = 0xA5;
	uint8_t *plen = p++;  // payload length, filled below
	uint8_t *start = p;

	*p++ = PP_WEBUSB_VER;
	*p++ = PP_BOARD;
	*p++ = 0;  // reserved (git-dirty flag)

	char build[12] = { 0 };
	strncpy(build, PICOPUCK_GIT_COMMIT, sizeof(build));
	memcpy(p, build, 12);
	p += 12;

	*p++ = bt_scan_active() ? 1 : 0;
	*p++ = PP_NSLOT;

	uint32_t now = 0;  // (state doesn't need a timestamp here)
	(void)now;
	for (int s = 0; s < PP_NSLOT; s++) {
		uint8_t kind = 0;
		int8_t rssi = 0;
		uint8_t addr[6] = { 0 };
		char name[16] = { 0 };
		bool bound = bt_slot_info(s, &kind, &rssi, addr, name);
		uint8_t state = g_slot[s].connected ? 2 :
				((bound || g_slot[s].used) ? 1 : 0);

		*p++ = state;
		*p++ = kind;
		*p++ = g_battery[s];
		*p++ = (uint8_t)rssi;
		memcpy(p, addr, 6);
		p += 6;
		memcpy(p, name, 16);
		p += 16;
	}

	// Diagnostics tail: [advSeen 2][flags 1][hciState 1][hciEvents 2].
	uint16_t adv = 0, hev = 0;
	uint8_t flags = 0, hci_state = 0;
	bt_scan_diag(&adv, &flags, &hci_state, &hev);
	*p++ = (uint8_t)(adv & 0xFF);
	*p++ = (uint8_t)(adv >> 8);
	*p++ = flags;
	*p++ = hci_state;
	*p++ = (uint8_t)(hev & 0xFF);
	*p++ = (uint8_t)(hev >> 8);

	*plen = (uint8_t)(p - start);
	send_frame(f, (uint16_t)(p - f));
}

static void send_scan_list(void)
{
	bt_scan_entry_t ents[SCAN_MAX];
	uint8_t n = bt_scan_list(ents, SCAN_MAX);

	uint8_t f[2 + 1 + SCAN_MAX * SCAN_REC_LEN];
	uint8_t *p = f;
	*p++ = 0xA6;
	uint8_t *plen = p++;
	uint8_t *start = p;
	*p++ = n;
	for (uint8_t i = 0; i < n; i++) {
		*p++ = ents[i].kind;
		*p++ = ents[i].addr_type;
		*p++ = (uint8_t)ents[i].rssi;
		memcpy(p, ents[i].addr, 6);
		p += 6;
		memcpy(p, ents[i].name, 16);
		p += 16;
	}
	*plen = (uint8_t)(p - start);
	send_frame(f, (uint16_t)(p - f));
}

// ---- opcode dispatch -------------------------------------------------------
// Expected total length (incl. opcode) for each known opcode, or 0 if unknown.
static uint8_t opcode_len(uint8_t op)
{
	switch (op) {
	case 0x01: return 1;
	case 0x02: return 2;
	case 0x03: return 1;
	case 0x04: return 1;
	case 0x05: return 8;
	case 0x06: return 8;
	case 0x07: return 1;
	case 0x08: return 2;
	case 0x0A: return 1;
	case 0x0B: return 3;
	case 0x0C: return 3;
	default: return 0;
	}
}

static void dispatch(const uint8_t *c, uint8_t n)
{
	switch (c[0]) {
	case 0x01:
		send_status();
		break;
	case 0x02:
		bt_scan_start(c[1]);
		break;
	case 0x03:
		bt_scan_stop();
		break;
	case 0x04:
		send_scan_list();
		break;
	case 0x05:
		bt_pair(&c[1], c[7]);
		break;
	case 0x06:
		bt_forget(&c[1], c[7]);
		break;
	case 0x07:
		bt_forget_all();
		break;
	case 0x08:
		bt_disconnect_slot(c[1]);
		break;
	case 0x0A:
		watchdog_reboot(0, 0, 10);
		break;
	case 0x0B:
		if (c[1] == 0x42 && c[2] == 0x4C)
			reset_usb_boot(0, 0);
		break;
	case 0x0C:
		if (c[1] == 0xDE && c[2] == 0xAD)
			bt_forget_all();  // settings wipe added with the settings module
		break;
	default:
		break;
	}
	(void)n;
}

void webusb_task(void)
{
	static uint8_t rx[64];
	static uint8_t rxlen;

	while (tud_vendor_available()) {
		uint32_t got = tud_vendor_read(rx + rxlen,
					       (uint32_t)(sizeof(rx) - rxlen));
		if (!got)
			break;
		rxlen += (uint8_t)got;
	}

	uint8_t off = 0;
	while (off < rxlen) {
		uint8_t need = opcode_len(rx[off]);
		if (need == 0) {
			// Unknown opcode: resync by dropping one byte.
			off++;
			continue;
		}
		if ((uint8_t)(rxlen - off) < need)
			break;  // wait for the rest of this command
		dispatch(&rx[off], need);
		off += need;
	}

	// Keep any partial trailing command for the next poll.
	if (off > 0 && off < rxlen)
		memmove(rx, rx + off, (size_t)(rxlen - off));
	rxlen -= off;
}
