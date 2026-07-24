// bt_host.c — BTstack dual-mode controller host (see bt_host.h).
//
// Phase 2 scope: BLE central. Scans for HID-over-GATT controllers, connects and
// pairs (Just Works), drives BTstack's hids_client to subscribe to input
// reports, decodes each via the matching input_driver into g_in[slot], and
// presents it to the puck personality. Rumble flows back through bt_relay ->
// the driver's output report. Classic HID (Phase 3) and the SC2 Valve GATT
// client (Phase 4) plug into the same slot/driver model.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "bt/bt_host.h"
#include "bt/bt_control.h"
#include "bt/input_driver.h"
#include "bt/bt_valve.h"
#include "puck/personality.h"
#include "puck/triton.h"
#include "config/picopuck_config.h"

#include <stdio.h>
#include <string.h>
#include "btstack.h"
#include "ble/gatt-service/hids_client.h"
#include "classic/hid_host.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"

#define SCAN_UI_INTERVAL 0x00A0    // 100 ms / 50 ms while the panel is scanning
#define SCAN_UI_WINDOW 0x0050
#define SCAN_IDLE_INTERVAL 0x0140  // 200 ms / 30 ms background reconnect (~15%)
#define SCAN_IDLE_WINDOW 0x0030
#define SCAN_MAX 12
#define RUMBLE_MIN_GAP_MS 30

static btstack_packet_callback_registration_t s_hci_cb;
static btstack_packet_callback_registration_t s_sm_cb;
static bool s_ready;
static uint8_t s_hid_desc_storage[1024];    // BLE hids_client
static uint8_t s_classic_desc_storage[1024]; // Classic hid_host

// Per-slot connection state.
typedef struct {
	bool active;
	hci_con_handle_t handle;
	uint16_t hids_cid;
	bool hid_started;  // hids_client_connect issued
	bool is_valve;     // SC2 native BLE (Valve GATT, not HOGP)
	bool is_classic;   // Bluetooth Classic HID (hids_cid holds the hid_host cid)
	const input_driver_t *drv;
	bool is_ble;
	bd_addr_t addr;
	bd_addr_type_t addr_type;
	char name[16];
	int8_t rssi;
	uint16_t rum_lo, rum_hi;
	bool rum_pending;
	uint32_t rum_last_ms;
} conn_t;
static conn_t g_conn[PP_NSLOT];

// The device we are currently connecting to (before a slot is assigned).
static struct {
	bool busy;
	uint32_t started_ms;
	bd_addr_t addr;
	bd_addr_type_t addr_type;
	char name[16];
} s_connecting;

static uint16_t s_adv_seen;  // diagnostic: raw advertising reports since scan start
static uint16_t s_hci_events;  // diagnostic: total HCI events seen (run-loop alive?)
static uint8_t s_last_state;   // diagnostic: last BTSTACK_EVENT_STATE value
static bool s_bt_init_ok;      // btstack_cyw43_init returned success
static bool s_power_on;        // hci_power_control(HCI_POWER_ON) was issued
#define CONNECT_TIMEOUT_MS 10000u

// Scan state. s_scanning = panel/UI scan (populates the list); a low-duty
// background scan also runs whenever a bonded device is offline, for reconnect.
static bool s_scanning;
static uint32_t s_scan_until_ms;
static bt_scan_entry_t s_scan[SCAN_MAX];
static uint8_t s_scan_n;
static bool s_scan_running;  // gap_start_scan is active
static bool s_scan_ui;       // running with UI (high-duty) params

// Pending pair target (set by bt_pair, connected on the next advert or directly).
static bool s_pair_pending;
static bd_addr_t s_pair_addr;
static bd_addr_type_t s_pair_type;

static uint32_t now_ms(void)
{
	return to_ms_since_boot(get_absolute_time());
}

// ---- slot / connection helpers --------------------------------------------
static int slot_by_handle(hci_con_handle_t h)
{
	for (int i = 0; i < PP_NSLOT; i++)
		if (g_conn[i].active && g_conn[i].handle == h)
			return i;
	return -1;
}
static int slot_by_cid(uint16_t cid)
{
	for (int i = 0; i < PP_NSLOT; i++)
		if (g_conn[i].active && !g_conn[i].is_classic &&
		    g_conn[i].hids_cid == cid)
			return i;
	return -1;
}
static int slot_by_hid_cid(uint16_t cid)
{
	for (int i = 0; i < PP_NSLOT; i++)
		if (g_conn[i].active && g_conn[i].is_classic &&
		    g_conn[i].hids_cid == cid)
			return i;
	return -1;
}
static int slot_alloc(void)
{
	for (int i = 0; i < PP_NSLOT; i++)
		if (!g_conn[i].active)
			return i;
	return -1;
}
static void slot_release(int slot)
{
	if (slot < 0 || slot >= PP_NSLOT)
		return;
	memset(&g_conn[slot], 0, sizeof(g_conn[slot]));
	puck_set_connected(slot, false);
}

static bool any_free_slot(void)
{
	for (int i = 0; i < PP_NSLOT; i++)
		if (!g_conn[i].active)
			return true;
	return false;
}
static bool addr_connected(const bd_addr_t addr)
{
	for (int i = 0; i < PP_NSLOT; i++)
		if (g_conn[i].active && memcmp(g_conn[i].addr, addr, 6) == 0)
			return true;
	return false;
}
static bool addr_bonded_le(const bd_addr_t addr)
{
	int max = le_device_db_max_count();
	for (int i = 0; i < max; i++) {
		int type;
		bd_addr_t a;
		le_device_db_info(i, &type, a, NULL);
		if (type != (int)BD_ADDR_TYPE_UNKNOWN && memcmp(a, addr, 6) == 0)
			return true;
	}
	return false;
}
static bool any_le_bond(void)
{
	int max = le_device_db_max_count();
	for (int i = 0; i < max; i++) {
		int type;
		bd_addr_t a;
		le_device_db_info(i, &type, a, NULL);
		if (type != (int)BD_ADDR_TYPE_UNKNOWN)
			return true;
	}
	return false;
}

// Start/stop the BLE scan to match need: high-duty while the panel scans,
// low-duty while a bonded device is offline (reconnect), off otherwise.
static void scan_ensure(void)
{
	if (s_connecting.busy)
		return;  // don't touch scan state while a connect is in flight
	bool want = s_scanning || (any_free_slot() && any_le_bond());
	bool ui = s_scanning;
	if (want && (!s_scan_running || s_scan_ui != ui)) {
		if (s_scan_running)
			gap_stop_scan();
		if (ui)
			gap_set_scan_params(1, SCAN_UI_INTERVAL, SCAN_UI_WINDOW, 0);
		else
			gap_set_scan_params(1, SCAN_IDLE_INTERVAL, SCAN_IDLE_WINDOW, 0);
		gap_start_scan();
		s_scan_running = true;
		s_scan_ui = ui;
	} else if (!want && s_scan_running) {
		gap_stop_scan();
		s_scan_running = false;
	}
}

// ---- forward decl ----------------------------------------------------------
static void hids_handler(uint8_t type, uint16_t channel, uint8_t *packet,
			 uint16_t size);
static void start_hids(int slot);

// ---- HCI events ------------------------------------------------------------
static void handle_advertising_report(uint8_t *packet)
{
	bd_addr_t addr;
	gap_event_advertising_report_get_address(packet, addr);
	bd_addr_type_t addr_type = gap_event_advertising_report_get_address_type(packet);
	int8_t rssi = gap_event_advertising_report_get_rssi(packet);
	uint8_t adv_len = gap_event_advertising_report_get_data_length(packet);
	const uint8_t *adv_data = gap_event_advertising_report_get_data(packet);

	char name[16] = { 0 };
	uint16_t appearance = 0;
	bool has_hid_uuid = false;

	ad_context_t ctx;
	for (ad_iterator_init(&ctx, adv_len, adv_data); ad_iterator_has_more(&ctx);
	     ad_iterator_next(&ctx)) {
		uint8_t t = ad_iterator_get_data_type(&ctx);
		uint8_t l = ad_iterator_get_data_len(&ctx);
		const uint8_t *d = ad_iterator_get_data(&ctx);
		if (t == BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME ||
		    t == BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME) {
			uint8_t n = (l < sizeof(name) - 1) ? l : sizeof(name) - 1;
			memcpy(name, d, n);
			name[n] = 0;
		} else if (t == BLUETOOTH_DATA_TYPE_APPEARANCE && l >= 2) {
			appearance = d[0] | (d[1] << 8);
		} else if ((t == BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS ||
			    t == BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS)) {
			for (int i = 0; i + 1 < l; i += 2)
				if ((d[i] | (d[i + 1] << 8)) == 0x1812)
					has_hid_uuid = true;
		}
	}

	s_adv_seen++;  // diagnostic: every advertising report, pre-filter

	bool is_gamepad = has_hid_uuid || (appearance >= 0x03C0 && appearance <= 0x03C4);

	// Add / merge the scan list for the panel. A controller splits its data
	// across the advertisement and the scan response (name in one, UUID/
	// appearance in the other), and the SC2 advertises no standard HID UUID at
	// all — so keep anything with a name or a gamepad hint, and merge fields by
	// address across reports rather than requiring one packet to have everything.
	if (s_scanning && (name[0] || is_gamepad)) {
		int found = -1;
		for (int i = 0; i < s_scan_n; i++)
			if (memcmp(s_scan[i].addr, addr, 6) == 0) {
				found = i;
				break;
			}
		if (found < 0 && s_scan_n < SCAN_MAX) {
			found = s_scan_n++;
			memset(&s_scan[found], 0, sizeof(s_scan[found]));
		}
		if (found >= 0) {
			memcpy(s_scan[found].addr, addr, 6);
			s_scan[found].addr_type = addr_type;
			s_scan[found].rssi = rssi;
			if (name[0])  // only overwrite when this report carried a name
				memcpy(s_scan[found].name, name,
				       sizeof(s_scan[found].name));
			s_scan[found].kind =
				strstr(s_scan[found].name, "Steam") ? 1 : 2;
		}
	}

	// Connect if this is our pending pair target, or a bonded device coming back
	// online while a slot is free (auto-reconnect). A driver match needs the
	// name, so for auto-reconnect wait for a report that carries it.
	bool want = (s_pair_pending && memcmp(addr, s_pair_addr, 6) == 0);
	if (!want && !s_connecting.busy && name[0] && any_free_slot() &&
	    addr_bonded_le(addr) && !addr_connected(addr))
		want = true;
	if (want && !s_connecting.busy) {
		s_pair_pending = false;
		s_connecting.busy = true;
		s_connecting.started_ms = now_ms();
		memcpy(s_connecting.addr, addr, 6);
		s_connecting.addr_type = addr_type;
		if (name[0])
			memcpy(s_connecting.name, name, sizeof(s_connecting.name));
		if (s_scan_running) {  // stop scanning while a connect is in flight
			gap_stop_scan();
			s_scan_running = false;
		}
		gap_connect(addr, addr_type);
	}
}

static void handle_inquiry_result(uint8_t *packet)
{
	bd_addr_t addr;
	gap_event_inquiry_result_get_bd_addr(packet, addr);
	uint32_t cod = gap_event_inquiry_result_get_class_of_device(packet);

	// Peripheral major device class (0x05) covers gamepads/keyboards/mice.
	if (((cod >> 8) & 0x1F) != 0x05)
		return;

	char name[16] = { 0 };
	if (gap_event_inquiry_result_get_name_available(packet)) {
		int n = gap_event_inquiry_result_get_name_len(packet);
		if (n > (int)sizeof(name) - 1)
			n = sizeof(name) - 1;
		memcpy(name, gap_event_inquiry_result_get_name(packet), n);
	}
	int8_t rssi = gap_event_inquiry_result_get_rssi_available(packet)
			      ? gap_event_inquiry_result_get_rssi(packet)
			      : 0;

	if (!s_scanning)
		return;
	int found = -1;
	for (int i = 0; i < s_scan_n; i++)
		if (memcmp(s_scan[i].addr, addr, 6) == 0) {
			found = i;
			break;
		}
	if (found < 0 && s_scan_n < SCAN_MAX)
		found = s_scan_n++;
	if (found >= 0) {
		memcpy(s_scan[found].addr, addr, 6);
		s_scan[found].addr_type = 0;  // 0 = Classic
		s_scan[found].kind = 3;       // Classic gamepad
		s_scan[found].rssi = rssi;
		memcpy(s_scan[found].name, name, sizeof(s_scan[found].name));
	}
}

static void handle_le_connection_complete(uint8_t *packet)
{
	uint8_t status = hci_subevent_le_connection_complete_get_status(packet);
	hci_con_handle_t handle =
		hci_subevent_le_connection_complete_get_connection_handle(packet);
	if (status != ERROR_CODE_SUCCESS) {
		s_connecting.busy = false;
		return;
	}
	int slot = slot_alloc();
	if (slot < 0) {
		gap_disconnect(handle);
		s_connecting.busy = false;
		return;
	}
	conn_t *c = &g_conn[slot];
	memset(c, 0, sizeof(*c));
	c->active = true;
	c->handle = handle;
	c->is_ble = true;
	c->is_valve = (strstr(s_connecting.name, "Steam") != NULL);
	memcpy(c->addr, s_connecting.addr, 6);
	c->addr_type = s_connecting.addr_type;
	memcpy(c->name, s_connecting.name, sizeof(c->name));
	s_connecting.busy = false;

	printf("[bt] LE connected slot %d handle 0x%04X name '%s'\n", slot, handle,
	       c->name);
	// Encrypt/bond; hids_client starts once secured (SM_EVENT_PAIRING/REENCRYPTION).
	sm_request_pairing(handle);
}

static void packet_handler(uint8_t type, uint16_t channel, uint8_t *packet,
			   uint16_t size)
{
	(void)channel;
	(void)size;
	if (type != HCI_EVENT_PACKET)
		return;

	s_hci_events++;

	switch (hci_event_packet_get_type(packet)) {
	case BTSTACK_EVENT_STATE:
		s_last_state = btstack_event_state_get_state(packet);
		if (s_last_state == HCI_STATE_WORKING) {
			s_ready = true;
			printf("[bt] host up (dual-mode)\n");
			scan_ensure();  // begin reconnect scan if bonds exist
		}
		break;
	case GAP_EVENT_ADVERTISING_REPORT:
		handle_advertising_report(packet);
		break;
	case GAP_EVENT_INQUIRY_RESULT:
		handle_inquiry_result(packet);
		break;
	case GAP_EVENT_INQUIRY_COMPLETE:
		if (s_scanning)
			gap_inquiry_start(8);  // keep looking while the panel scans
		break;
	case HCI_EVENT_LE_META:
		if (hci_event_le_meta_get_subevent_code(packet) ==
		    HCI_SUBEVENT_LE_CONNECTION_COMPLETE)
			handle_le_connection_complete(packet);
		break;
	case HCI_EVENT_DISCONNECTION_COMPLETE: {
		hci_con_handle_t h =
			hci_event_disconnection_complete_get_connection_handle(packet);
		int slot = slot_by_handle(h);
		if (slot >= 0) {
			printf("[bt] disconnect slot %d\n", slot);
			if (g_conn[slot].is_valve)
				valve_disconnected(h);
			slot_release(slot);
			scan_ensure();  // resume reconnect scan
		}
		break;
	}
	default:
		break;
	}
}

// ---- Security manager events ----------------------------------------------
static void sm_handler(uint8_t type, uint16_t channel, uint8_t *packet,
		       uint16_t size)
{
	(void)channel;
	(void)size;
	if (type != HCI_EVENT_PACKET)
		return;

	switch (hci_event_packet_get_type(packet)) {
	case SM_EVENT_JUST_WORKS_REQUEST:
		sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
		break;
	case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
		sm_numeric_comparison_confirm(
			sm_event_numeric_comparison_request_get_handle(packet));
		break;
	case SM_EVENT_PAIRING_COMPLETE: {
		if (sm_event_pairing_complete_get_status(packet) == ERROR_CODE_SUCCESS) {
			int slot = slot_by_handle(
				sm_event_pairing_complete_get_handle(packet));
			if (slot >= 0)
				start_hids(slot);
		}
		break;
	}
	case SM_EVENT_REENCRYPTION_COMPLETE: {
		int slot = slot_by_handle(
			sm_event_reencryption_complete_get_handle(packet));
		if (slot >= 0)
			start_hids(slot);
		break;
	}
	default:
		break;
	}
}

static void start_hids(int slot)
{
	conn_t *c = &g_conn[slot];
	if (c->hid_started)
		return;
	c->hid_started = true;

	// SC2 speaks a proprietary Valve GATT service, not HOGP.
	if (c->is_valve) {
		c->drv = input_driver_match(0, 0, c->name, true);  // kind 1 (SC2)
		puck_set_connected(slot, true);
		valve_start(slot, c->handle);
		return;
	}

	uint8_t r = hids_client_connect(c->handle, hids_handler,
					HID_PROTOCOL_MODE_REPORT, &c->hids_cid);
	if (r != ERROR_CODE_SUCCESS) {
		printf("[bt] hids_client_connect failed %d\n", r);
		c->hid_started = false;
	}
}

// ---- HID-over-GATT client events -------------------------------------------
static void hids_handler(uint8_t type, uint16_t channel, uint8_t *packet,
			 uint16_t size)
{
	(void)channel;
	(void)size;
	if (type != HCI_EVENT_PACKET ||
	    hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META)
		return;

	switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
	case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED: {
		uint16_t cid = gattservice_subevent_hid_service_connected_get_hids_cid(packet);
		uint8_t status = gattservice_subevent_hid_service_connected_get_status(packet);
		int slot = slot_by_cid(cid);
		if (slot < 0)
			break;
		conn_t *c = &g_conn[slot];
		if (status != ERROR_CODE_SUCCESS) {
			printf("[bt] HID connect failed slot %d status 0x%02X\n", slot,
			       status);
			gap_disconnect(c->handle);
			break;
		}
		c->drv = input_driver_match(0, 0, c->name, c->is_ble);
		if (!c->drv) {
			printf("[bt] no driver for '%s' — dropping\n", c->name);
			gap_disconnect(c->handle);
			break;
		}
		printf("[bt] slot %d driver '%s' ready\n", slot, c->drv->name);
		puck_set_connected(slot, true);
		hids_client_enable_notifications(cid);
		break;
	}
	case GATTSERVICE_SUBEVENT_HID_REPORT: {
		uint16_t cid = gattservice_subevent_hid_report_get_hids_cid(packet);
		int slot = slot_by_cid(cid);
		if (slot < 0 || !g_conn[slot].drv || !g_conn[slot].drv->decode)
			break;
		// hids_client delivers [report_id][body...]; drivers want body-only.
		const uint8_t *rep = gattservice_subevent_hid_report_get_report(packet);
		uint16_t rlen = gattservice_subevent_hid_report_get_report_len(packet);
		if (rlen < 1)
			break;
		if (g_conn[slot].drv->decode(rep[0], rep + 1, (uint16_t)(rlen - 1),
					     &g_in[slot]))
			puck_present_synth(slot);
		break;
	}
	default:
		break;
	}
}

// ---- Classic HID host ------------------------------------------------------
static void classic_handler(uint8_t type, uint16_t channel, uint8_t *packet,
			    uint16_t size)
{
	(void)channel;
	(void)size;
	if (type != HCI_EVENT_PACKET ||
	    hci_event_packet_get_type(packet) != HCI_EVENT_HID_META)
		return;

	switch (hci_event_hid_meta_get_subevent_code(packet)) {
	case HID_SUBEVENT_INCOMING_CONNECTION:
		// A pad (e.g. a DualShock) initiated — accept in report mode.
		hid_host_accept_connection(
			hid_subevent_incoming_connection_get_hid_cid(packet),
			HID_PROTOCOL_MODE_REPORT);
		break;
	case HID_SUBEVENT_CONNECTION_OPENED: {
		uint8_t status = hid_subevent_connection_opened_get_status(packet);
		uint16_t cid = hid_subevent_connection_opened_get_hid_cid(packet);
		if (status != ERROR_CODE_SUCCESS)
			break;
		int slot = slot_alloc();
		if (slot < 0) {
			hid_host_disconnect(cid);
			break;
		}
		conn_t *c = &g_conn[slot];
		memset(c, 0, sizeof(*c));
		c->active = true;
		c->is_classic = true;
		c->hids_cid = cid;
		hid_subevent_connection_opened_get_bd_addr(packet, c->addr);
		memcpy(c->name, s_connecting.name, sizeof(c->name));
		s_connecting.busy = false;
		c->drv = input_driver_match(0, 0, c->name, false);
		if (!c->drv) {
			printf("[bt] no Classic driver for '%s'\n", c->name);
			hid_host_disconnect(cid);
			slot_release(slot);
			break;
		}
		printf("[bt] Classic slot %d driver '%s'\n", slot, c->drv->name);
		puck_set_connected(slot, true);
		break;
	}
	case HID_SUBEVENT_REPORT: {
		int slot = slot_by_hid_cid(hid_subevent_report_get_hid_cid(packet));
		if (slot < 0 || !g_conn[slot].drv || !g_conn[slot].drv->decode)
			break;
		const uint8_t *r = hid_subevent_report_get_report(packet);
		uint16_t rlen = hid_subevent_report_get_report_len(packet);
		if (rlen < 1)
			break;
		// Classic HID interrupt reports carry the report-id as byte 0.
		if (g_conn[slot].drv->decode(r[0], r + 1, (uint16_t)(rlen - 1),
					     &g_in[slot]))
			puck_present_synth(slot);
		break;
	}
	case HID_SUBEVENT_CONNECTION_CLOSED: {
		int slot = slot_by_hid_cid(
			hid_subevent_connection_closed_get_hid_cid(packet));
		if (slot >= 0) {
			printf("[bt] Classic disconnect slot %d\n", slot);
			slot_release(slot);
		}
		break;
	}
	default:
		break;
	}
}

// ---- init ------------------------------------------------------------------
bool bt_host_init(void)
{
	// cyw43_arch_init() (with CYW43_ENABLE_BLUETOOTH=1) has ALREADY run
	// btstack_cyw43_init(): BTstack memory, the async-context run loop, the HCI
	// transport, and the flash-bank bond store (Classic link keys + LE device DB)
	// are all set up. We must NOT call btstack_cyw43_init() again. Here we only
	// bring up the protocol services and power on.
	s_bt_init_ok = true;

	l2cap_init();
	sm_init();
	sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
	sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION |
					   SM_AUTHREQ_BONDING);
	gatt_client_init();
	hids_client_init(s_hid_desc_storage, sizeof(s_hid_desc_storage));

	// Classic HID host (DS4/DS5/etc). Discoverable + connectable so Sony pads
	// can initiate the connection inbound.
	hid_host_init(s_classic_desc_storage, sizeof(s_classic_desc_storage));
	hid_host_register_packet_handler(&classic_handler);
	gap_set_class_of_device(0x000104);  // Computer / Desktop
	gap_set_bondable_mode(1);
	gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
	gap_ssp_set_auto_accept(1);
	gap_ssp_set_authentication_requirement(
		SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_DEDICATED_BONDING);
	gap_discoverable_control(1);
	gap_connectable_control(1);

	gap_set_local_name("PicoPuck");

	s_hci_cb.callback = &packet_handler;
	hci_add_event_handler(&s_hci_cb);
	s_sm_cb.callback = &sm_handler;
	sm_add_event_handler(&s_sm_cb);

	hci_power_control(HCI_POWER_ON);
	s_power_on = true;
	return true;
}

bool bt_host_ready(void)
{
	return s_ready;
}

void bt_host_task(void)
{
	if (s_scanning && s_scan_until_ms && now_ms() > s_scan_until_ms)
		bt_scan_stop();

	// A connect with no completion event would otherwise wedge scanning forever.
	if (s_connecting.busy && now_ms() - s_connecting.started_ms > CONNECT_TIMEOUT_MS) {
		gap_connect_cancel();
		s_connecting.busy = false;
	}

	scan_ensure();     // keep reconnect scan matched to need
	valve_periodic();  // SC2 lizard-off keepalive + IMU enable

	// Flush latched rumble, rate-limited.
	uint32_t t = now_ms();
	for (int i = 0; i < PP_NSLOT; i++) {
		conn_t *c = &g_conn[i];
		if (!c->active || !c->rum_pending || !c->drv || !c->drv->build_rumble)
			continue;
		if (t - c->rum_last_ms < RUMBLE_MIN_GAP_MS)
			continue;
		uint8_t buf[96];
		uint8_t n = c->drv->build_rumble(c->rum_lo, c->rum_hi, buf, sizeof(buf));
		if (n) {
			if (c->is_classic)
				hid_host_send_set_report(c->hids_cid,
							 HID_REPORT_TYPE_OUTPUT,
							 c->drv->rumble_report_id,
							 buf, n);
			else
				hids_client_send_write_report(
					c->hids_cid, c->drv->rumble_report_id,
					HID_REPORT_TYPE_OUTPUT, buf, n);
			c->rum_last_ms = t;
			c->rum_pending = false;
		}
	}
}

// ---- relay (host → controller): rumble for generic pads --------------------
void bt_relay(int slot, uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	if (slot < 0 || slot >= PP_NSLOT || !g_conn[slot].active)
		return;

	// SC2: forward the whole command verbatim to the Valve report characteristic
	// (this is the transparent haptics/settings/power path).
	if (g_conn[slot].is_valve) {
		valve_feature_write(slot, cmd, payload, len);
		return;
	}

	// Generic pad: map Steam's 0x80 rumble to the pad's rumble output.
	// 0x80 layout: [type][..][lo u16 @3][.. ][hi u16 @6].
	if (cmd == 0x80 && len >= 8) {
		g_conn[slot].rum_lo = (uint16_t)(payload[3] | (payload[4] << 8));
		g_conn[slot].rum_hi = (uint16_t)(payload[6] | (payload[7] << 8));
		g_conn[slot].rum_pending = true;
	}
}

// ---- bt_control (panel surface) --------------------------------------------
bool bt_slot_info(int slot, uint8_t *kind, int8_t *rssi, uint8_t addr[6],
		  char name[16])
{
	if (slot < 0 || slot >= PP_NSLOT || !g_conn[slot].active)
		return false;
	conn_t *c = &g_conn[slot];
	*kind = c->drv ? c->drv->kind : 0;
	*rssi = c->rssi;
	memcpy(addr, c->addr, 6);
	memcpy(name, c->name, 16);
	return true;
}

void bt_scan_start(uint16_t seconds)
{
	s_scan_n = 0;
	s_adv_seen = 0;
	s_scanning = true;
	scan_ensure();          // BLE scan at UI duty
	gap_inquiry_start(8);   // Classic (~10 s rounds, restarted on complete)
	s_scan_until_ms = seconds ? (now_ms() + (uint32_t)seconds * 1000) : 0;
}

void bt_scan_stop(void)
{
	if (s_scanning)
		gap_inquiry_stop();
	s_scanning = false;
	s_scan_until_ms = 0;
	scan_ensure();  // drop to background reconnect duty (or off)
}

bool bt_scan_active(void)
{
	return s_scanning;
}

void bt_scan_diag(uint16_t *adv_seen, uint8_t *flags, uint8_t *hci_state,
		  uint16_t *hci_events)
{
	if (adv_seen)
		*adv_seen = s_adv_seen;
	if (flags)
		*flags = (uint8_t)((s_scan_running ? 1 : 0) |
				   (s_ready ? 2 : 0) |
				   (s_connecting.busy ? 4 : 0) |
				   (s_bt_init_ok ? 8 : 0) |
				   (s_power_on ? 16 : 0));
	if (hci_state)
		*hci_state = s_last_state;
	if (hci_events)
		*hci_events = s_hci_events;
}

uint8_t bt_scan_list(bt_scan_entry_t *out, uint8_t max)
{
	uint8_t n = s_scan_n < max ? s_scan_n : max;
	memcpy(out, s_scan, n * sizeof(bt_scan_entry_t));
	return n;
}

bool bt_pair(const uint8_t addr[6], uint8_t addr_type)
{
	// Look up the scanned entry to carry its name/kind into the connection.
	uint8_t kind = 2;
	memset(&s_connecting, 0, sizeof(s_connecting));
	for (int i = 0; i < s_scan_n; i++)
		if (memcmp(s_scan[i].addr, addr, 6) == 0) {
			memcpy(s_connecting.name, s_scan[i].name,
			       sizeof(s_connecting.name));
			kind = s_scan[i].kind;
		}
	memcpy(s_connecting.addr, addr, 6);
	s_connecting.addr_type = addr_type;
	s_connecting.busy = true;
	s_connecting.started_ms = now_ms();

	if (kind == 3) {
		// Classic: connect the HID host directly (slot bound on OPENED).
		uint16_t cid;
		uint8_t r = hid_host_connect((uint8_t *)addr,
					     HID_PROTOCOL_MODE_REPORT, &cid);
		if (r != ERROR_CODE_SUCCESS) {
			s_connecting.busy = false;
			return false;
		}
		return true;
	}

	// BLE: connect directly (slot bound on connection complete).
	memcpy(s_pair_addr, addr, 6);
	s_pair_type = addr_type;
	if (s_scan_running) {
		gap_stop_scan();
		s_scan_running = false;
	}
	uint8_t r = gap_connect((uint8_t *)addr, (bd_addr_type_t)addr_type);
	if (r != ERROR_CODE_SUCCESS) {
		s_connecting.busy = false;
		s_pair_pending = true;  // fall back to connecting on next advert
		return false;
	}
	return true;
}

static void conn_disconnect(conn_t *c)
{
	if (c->is_classic)
		hid_host_disconnect(c->hids_cid);
	else
		gap_disconnect(c->handle);
}

void bt_forget(const uint8_t addr[6], uint8_t addr_type)
{
	for (int i = 0; i < PP_NSLOT; i++)
		if (g_conn[i].active && memcmp(g_conn[i].addr, addr, 6) == 0)
			conn_disconnect(&g_conn[i]);
	// Drop both bond stores (whichever holds it).
	gap_delete_bonding((bd_addr_type_t)addr_type, (uint8_t *)addr);
	gap_drop_link_key_for_bd_addr((uint8_t *)addr);
}

void bt_forget_all(void)
{
	for (int i = 0; i < PP_NSLOT; i++)
		if (g_conn[i].active)
			conn_disconnect(&g_conn[i]);
	// Wipe the LE bond database.
	int max = le_device_db_max_count();
	for (int i = 0; i < max; i++) {
		int addr_type;
		bd_addr_t a;
		le_device_db_info(i, &addr_type, a, NULL);
		if (addr_type != (int)BD_ADDR_TYPE_UNKNOWN)
			le_device_db_remove(i);
	}
	// Classic link keys.
	gap_delete_all_link_keys();
}

void bt_disconnect_slot(int slot)
{
	if (slot >= 0 && slot < PP_NSLOT && g_conn[slot].active)
		conn_disconnect(&g_conn[slot]);
}
