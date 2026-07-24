// personality.c — the puck HID personality (see personality.h).
//
// Ported from OpenPuck/puck_hid.cpp (SteamPuckController). TinyUSB routes HID
// control transfers to us per instance, where instance == slot. We answer the
// feature command channel locally (identity/bond/settings) and hand actuator /
// config writes to relay_enqueue for whatever controller is bound to the slot.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "puck/personality.h"
#include "puck/slots.h"
#include "puck/identity.h"
#include "puck/relay.h"
#include "puck/triton.h"
#include "usb/usb_tx.h"
#include "config/picopuck_config.h"

#include <string.h>
#include "tusb.h"
#include "pico/time.h"

static uint32_t s_steam_alive_ms;
static bool s_pairing;

static uint8_t s_conn_state[PP_NSLOT];  // last emitted 0x79 state (0 none/1 disc/2 conn)
static uint32_t s_last_7b_ms[PP_NSLOT];
static uint32_t s_last_43_ms[PP_NSLOT];

static inline uint32_t now_ms(void)
{
	return to_ms_since_boot(get_absolute_time());
}

static bool rec_empty(const uint8_t *rec)
{
	for (int i = 0; i < PP_BOND_REC_LEN; i++)
		if (rec[i])
			return false;
	return true;
}

void puck_personality_init(void)
{
	s_steam_alive_ms = 0;
	s_pairing = false;
	memset(s_conn_state, 0, sizeof(s_conn_state));
	memset(s_last_7b_ms, 0, sizeof(s_last_7b_ms));
	memset(s_last_43_ms, 0, sizeof(s_last_43_ms));
}

bool puck_steam_active(void)
{
	// Steam is "driving" if it has written the command channel in the last 5 s.
	return s_steam_alive_ms && (now_ms() - s_steam_alive_ms < 5000u);
}

static uint8_t s_seq[PP_NSLOT];

void puck_present_synth(int slot)
{
	if (slot < 0 || slot >= PP_NSLOT)
		return;
	uint8_t rep[PUCK45_LEN];
	puck_synth45(&g_in[slot], s_seq[slot]++, time_us_32(), rep);
	usb_tx_hid((uint8_t)slot, rep[0], rep + 1, PUCK45_LEN - 1);
	slot_note_input(slot, now_ms());
}

void puck_present_raw(int slot, const uint8_t *rep, uint8_t len)
{
	if (slot < 0 || slot >= PP_NSLOT || len < 1)
		return;
	usb_tx_hid((uint8_t)slot, rep[0], rep + 1, (uint16_t)(len - 1));
	slot_note_input(slot, now_ms());
}

void puck_set_connected(int slot, bool connected)
{
	if (slot < 0 || slot >= PP_NSLOT)
		return;
	g_slot[slot].connected = connected;
	if (!connected)
		g_slot[slot].conn_reply_ms = 0;
}

// ---- feature / output writes (host → puck/controller) ----------------------
static void handle_set(int slot, uint8_t rid, hid_report_type_t type,
		       const uint8_t *b, uint16_t n)
{
	if (slot < 0 || slot >= PP_NSLOT)
		return;
	puck_slot_t *S = &g_slot[slot];

	// OUTPUT reports 0x80-0x89: haptics/actuators. Relay the 0x80-0x86 range to
	// the bound controller; the rest are config that rides the feature path.
	if (type == HID_REPORT_TYPE_OUTPUT) {
		if (rid >= 0x80 && rid <= 0x89)
			s_steam_alive_ms = now_ms();
		if (rid >= 0x80 && rid <= 0x86 && n >= 1)
			relay_enqueue(slot, rid, b, n);
		return;
	}

	if (type != HID_REPORT_TYPE_FEATURE || n < 1)
		return;

	uint8_t cmd = b[0];
	uint8_t len = (n > 1) ? b[1] : 0;
	const uint8_t *pl = b + 2;
	uint16_t pln = (n >= 2) ? (uint16_t)(n - 2) : 0;

	if (cmd >= 0x80 && cmd <= 0x89)
		s_steam_alive_ms = now_ms();

	// report id 1 = commands relayed to the bonded controller. Do NOT relay the
	// identity/bond/settings reads we answer locally (their reply can't come
	// back over the link, and sub-0x87 commands would execute on the controller).
	if (rid == 1 && n >= 2) {
		bool local_answer =
			(cmd == 0x83 || cmd == 0x89 || cmd == 0xAE ||
			 cmd == 0xA2 || cmd == 0xA3 || cmd == 0xAD ||
			 cmd == 0xB4 || cmd == 0xED || cmd == 0xA4);
		if (!local_answer) {
			uint8_t rl = (len <= pln) ? len : (uint8_t)pln;
			relay_enqueue(slot, cmd, pl, rl);
		}
	}

	memset(S->resp, 0, sizeof(S->resp));
	S->resp_len = 0;

	switch (cmd) {
	case 0x83:  // GET_ATTRIBUTES
		S->resp[0] = 0x83;
		S->resp[1] = sizeof(ATTR83);
		memcpy(S->resp + 2, ATTR83, sizeof(ATTR83));
		// rid 1 = the bonded controller's attributes → product 0x1302, so Steam
		// takes the modern quiet config path instead of the legacy 0x81 storm.
		if (rid == 1)
			S->resp[2 + 1] = 0x02;
		S->resp_len = 63;
		break;

	case 0xAE: {  // string attributes
		uint8_t idx = pln > 0 ? pl[0] : 1;
		S->resp[0] = 0xAE;
		S->resp[1] = 0x14;
		S->resp[2] = idx;
		if (rid == 1 && S->used && (idx == 0 || idx == 1 || idx == 4)) {
			// The bonded controller's serial lives in its bond record
			// (rec[8..24]); Steam matches the controller to its bond by it.
			memcpy(S->resp + 3, S->rec + 8, 16);
		} else {
			const char *s = (rid == 1) ? "NA" :
					(idx == 0 || idx == 4) ? g_board :
					(idx == 1) ? g_unit : "NA";
			memcpy(S->resp + 3, s, strlen(s));
		}
		S->resp_len = 63;
		break;
	}

	case 0xB4:  // connection / version state
		s_steam_alive_ms = now_ms();
		S->resp[0] = 0xB4;
		S->resp[1] = 0x01;
		S->resp[2] = slot_is_live(slot, now_ms()) ? 0x02 : 0x01;
		S->resp_len = 63;
		break;

	case 0xAD:  // enter/exit pairing
		s_pairing = (pln > 0 && pl[0] != 0);
		S->resp[0] = 0xAD;
		S->resp_len = 63;
		break;

	case 0xA2:  // write/clear this slot's bond record
		if (len >= PP_BOND_REC_LEN && pln >= PP_BOND_REC_LEN) {
			if (rec_empty(pl)) {
				S->used = false;
				memset(S->rec, 0, PP_BOND_REC_LEN);
			} else {
				memcpy(S->rec, pl, PP_BOND_REC_LEN);
				S->used = true;
			}
		}
		S->resp[0] = 0xA2;
		S->resp_len = 63;
		break;

	case 0xA3:  // read this slot's bond record
		S->resp[0] = 0xA3;
		S->resp[1] = 0x18;
		if (S->used)
			memcpy(S->resp + 2, S->rec, PP_BOND_REC_LEN);
		S->resp_len = 63;
		break;

	case 0x87:  // SET_SETTINGS_VALUES → shadow for 0x89 read-back
		for (uint16_t i = 0; i + 2 < pln; i += 3) {
			uint8_t id = pl[i];
			if (id < PP_SETTINGS_MAX)
				S->set_shadow[id] =
					(uint16_t)(pl[i + 1] | (pl[i + 2] << 8));
		}
		S->resp[0] = 0x87;
		S->resp_len = 63;
		break;

	case 0x89: {  // GET_SETTINGS_VALUES
		uint8_t id = (pln > 0) ? pl[0] : 0;
		uint16_t v = (id < PP_SETTINGS_MAX) ? S->set_shadow[id] : 0;
		S->resp[0] = 0x89;
		S->resp[1] = 0x03;
		S->resp[2] = id;
		S->resp[3] = (uint8_t)(v & 0xFF);
		S->resp[4] = (uint8_t)(v >> 8);
		S->resp_len = 63;
		break;
	}

	default:
		break;
	}
}

// ---- feature reads (host ← puck) -------------------------------------------
static uint16_t handle_get(int slot, uint8_t rid, hid_report_type_t type,
			   uint8_t *buf, uint16_t reqlen)
{
	(void)rid;
	if (type != HID_REPORT_TYPE_FEATURE || slot < 0 || slot >= PP_NSLOT)
		return 0;
	puck_slot_t *S = &g_slot[slot];
	uint16_t n = S->resp_len ? S->resp_len : 63;
	if (n > reqlen)
		n = reqlen;
	memcpy(buf, S->resp, n);
	return n;
}

// ---- TinyUSB HID callbacks (instance == slot) ------------------------------
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
			       hid_report_type_t report_type, uint8_t *buffer,
			       uint16_t reqlen)
{
	return handle_get(instance, report_id, report_type, buffer, reqlen);
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
			   hid_report_type_t report_type, uint8_t const *buffer,
			   uint16_t bufsize)
{
	handle_set(instance, report_id, report_type, buffer, bufsize);
}

// ---- periodic status (0x79 / 0x7B / 0x43) ----------------------------------
void puck_personality_task(void)
{
	if (tud_suspended())
		return;  // don't emit while the host sleeps (could wake it)

	uint32_t t = now_ms();

	for (int s = 0; s < PP_NSLOT; s++) {
		bool live = slot_is_live(s, t);
		uint8_t state = live ? 0x02 : 0x01;

		// 0x79 connection-state edge: emit on change.
		if (state != s_conn_state[s]) {
			s_conn_state[s] = state;
			uint8_t body = state;
			usb_tx_hid((uint8_t)s, 0x79, &body, 1);
		}

		if (!live)
			continue;

		// 0x7B status (RSSI template) every 750 ms.
		if (t - s_last_7b_ms[s] >= 750) {
			uint8_t s7b[12] = { 0xF7, 0x01, 0x89, 0x00, 0x00, 0x00,
					    0x03, 0x00, 0xDD, 0x00, 0x3A, 0x02 };
			usb_tx_hid((uint8_t)s, 0x7B, s7b, sizeof(s7b));
			s_last_7b_ms[s] = t;
		}

		// Synthesized 0x43 battery every 2 s (SDL needs the full 14-byte form).
		if (g_battery[s] && t - s_last_43_ms[s] >= 2000) {
			uint8_t st = g_battery_state[s];
			if (st != 1 && st != 2 && st != 4)
				st = 1;  // discharging
			uint8_t b43[14] = { 0 };
			b43[0] = st;
			b43[1] = g_battery[s];
			usb_tx_hid((uint8_t)s, 0x43, b43, sizeof(b43));
			s_last_43_ms[s] = t;
		}
	}
}
