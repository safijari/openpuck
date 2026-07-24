// bt_valve.c — Steam Controller 2 native-BLE client (see bt_valve.h).
//
// Ported from joypad-os btstack_host.c's Valve GATT client. One instance per
// slot; events are demuxed by con_handle.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "bt/bt_valve.h"
#include "puck/personality.h"
#include "puck/triton.h"
#include "config/picopuck_config.h"

#include <stdio.h>
#include <string.h>
#include "pico/time.h"

// 128-bit UUIDs in textual/big-endian order (as BTstack's uuid128 API expects).
static const uint8_t VALVE_SERVICE[16] = { 0x10, 0x0F, 0x6C, 0x32, 0x17, 0x35,
					   0x43, 0x13, 0xB4, 0x02, 0x38, 0x56,
					   0x71, 0x31, 0xE5, 0xF3 };
static const uint8_t VALVE_INPUT_45[16] = { 0x10, 0x0F, 0x6C, 0x7A, 0x17, 0x35,
					    0x43, 0x13, 0xB4, 0x02, 0x38, 0x56,
					    0x71, 0x31, 0xE5, 0xF3 };
static const uint8_t VALVE_INPUT_47[16] = { 0x10, 0x0F, 0x6C, 0x7C, 0x17, 0x35,
					    0x43, 0x13, 0xB4, 0x02, 0x38, 0x56,
					    0x71, 0x31, 0xE5, 0xF3 };
static const uint8_t VALVE_REPORT[16] = { 0x10, 0x0F, 0x6C, 0x34, 0x17, 0x35,
					  0x43, 0x13, 0xB4, 0x02, 0x38, 0x56,
					  0x71, 0x31, 0xE5, 0xF3 };

// The SC2 re-enables lizard (keyboard/mouse) mode on a ~3 s watchdog, so resend
// lizard-off well inside that. {0x87 ID_SET_SETTINGS, 0x03 len, 0x09 id, u16 0}.
#define VALVE_KEEPALIVE_MS 2000u
static uint8_t s_lizard_off[5] = { 0x87, 0x03, 0x09, 0x00, 0x00 };
// IMU on: SEND_RAW_ACCEL(0x08) | SEND_RAW_GYRO(0x10) = 0x18 on setting id 0x30.
static uint8_t s_imu_on[5] = { 0x87, 0x03, 0x30, 0x18, 0x00 };

typedef enum {
	V_IDLE = 0,
	V_DISC_SERVICE,
	V_DISC_CHARS,
	V_ENABLE_CCC,
	V_READY,
} valve_state_t;

typedef struct {
	valve_state_t state;
	int slot;
	hci_con_handle_t handle;
	gatt_client_service_t service;
	gatt_client_characteristic_t input_char;
	uint8_t report_id;              // 0x45 or 0x47
	uint16_t report_value_handle;   // 100F6C34 write target
	gatt_client_notification_t notify;
	uint32_t last_keepalive_ms;
	bool imu_enabled;
} valve_t;

static valve_t s_valve[PP_NSLOT];

static uint32_t now_ms(void)
{
	return to_ms_since_boot(get_absolute_time());
}

static valve_t *by_handle(hci_con_handle_t h)
{
	for (int i = 0; i < PP_NSLOT; i++)
		if (s_valve[i].state != V_IDLE && s_valve[i].handle == h)
			return &s_valve[i];
	return NULL;
}

static void valve_reset(valve_t *v)
{
	if (v->state == V_READY)
		gatt_client_stop_listening_for_characteristic_value_updates(&v->notify);
	memset(v, 0, sizeof(*v));
	v->handle = HCI_CON_HANDLE_INVALID;
}

void valve_disconnected(hci_con_handle_t handle)
{
	valve_t *v = by_handle(handle);
	if (v)
		valve_reset(v);
}

// Input notifications: prepend the report id and forward verbatim to the slot.
static void notify_handler(uint8_t type, uint16_t channel, uint8_t *packet,
			   uint16_t size)
{
	(void)channel;
	(void)size;
	if (type != HCI_EVENT_PACKET ||
	    hci_event_packet_get_type(packet) != GATT_EVENT_NOTIFICATION)
		return;
	hci_con_handle_t h = gatt_event_notification_get_handle(packet);
	valve_t *v = by_handle(h);
	if (!v || v->state != V_READY)
		return;
	if (gatt_event_notification_get_value_handle(packet) !=
	    v->input_char.value_handle)
		return;

	uint16_t len = gatt_event_notification_get_value_length(packet);
	const uint8_t *val = gatt_event_notification_get_value(packet);
	if (len < 18)
		return;

	uint8_t rep[PUCK45_LEN + 8];
	if ((size_t)len + 1 > sizeof(rep))
		len = sizeof(rep) - 1;
	rep[0] = v->report_id;
	memcpy(rep + 1, val, len);
	puck_present_raw(v->slot, rep, (uint8_t)(len + 1));  // transparent forward
}

static void write_cb(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	(void)type;
	(void)channel;
	(void)packet;
	(void)size;  // completion only; ignore
}

static void gatt_handler(uint8_t type, uint16_t channel, uint8_t *packet,
			 uint16_t size)
{
	(void)channel;
	(void)size;
	if (type != HCI_EVENT_PACKET)
		return;
	hci_con_handle_t h = gatt_event_query_complete_get_handle(packet);
	valve_t *v = by_handle(h);
	if (!v)
		return;

	switch (hci_event_packet_get_type(packet)) {
	case GATT_EVENT_SERVICE_QUERY_RESULT:
		gatt_event_service_query_result_get_service(packet, &v->service);
		break;
	case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
		gatt_client_characteristic_t ch;
		gatt_event_characteristic_query_result_get_characteristic(packet, &ch);
		if (memcmp(ch.uuid128, VALVE_INPUT_45, 16) == 0) {
			v->input_char = ch;
			v->report_id = 0x45;
		} else if (memcmp(ch.uuid128, VALVE_INPUT_47, 16) == 0 &&
			   v->report_id == 0) {
			v->input_char = ch;
			v->report_id = 0x47;
		} else if (memcmp(ch.uuid128, VALVE_REPORT, 16) == 0) {
			v->report_value_handle = ch.value_handle;
		}
		break;
	}
	case GATT_EVENT_QUERY_COMPLETE: {
		uint8_t status = gatt_event_query_complete_get_att_status(packet);
		if (status != ATT_ERROR_SUCCESS) {
			printf("[sc2] GATT query failed state=%d status=0x%02X\n",
			       v->state, status);
			valve_reset(v);
			break;
		}
		if (v->state == V_DISC_SERVICE) {
			if (v->service.start_group_handle == 0) {
				printf("[sc2] no Valve service\n");
				valve_reset(v);
				break;
			}
			v->state = V_DISC_CHARS;
			gatt_client_discover_characteristics_for_service(
				gatt_handler, v->handle, &v->service);
		} else if (v->state == V_DISC_CHARS) {
			if (v->report_id == 0 || v->input_char.value_handle == 0) {
				printf("[sc2] no Valve input characteristic\n");
				valve_reset(v);
				break;
			}
			v->state = V_ENABLE_CCC;
			gatt_client_write_client_characteristic_configuration(
				gatt_handler, v->handle, &v->input_char,
				GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
		} else if (v->state == V_ENABLE_CCC) {
			gatt_client_listen_for_characteristic_value_updates(
				&v->notify, notify_handler, v->handle,
				&v->input_char);
			v->state = V_READY;
			v->last_keepalive_ms = now_ms();
			printf("[sc2] ready (report id 0x%02X)\n", v->report_id);
		}
		break;
	}
	default:
		break;
	}
}

void valve_start(int slot, hci_con_handle_t handle)
{
	if (slot < 0 || slot >= PP_NSLOT)
		return;
	valve_t *v = &s_valve[slot];
	memset(v, 0, sizeof(*v));
	v->slot = slot;
	v->handle = handle;
	v->state = V_DISC_SERVICE;
	printf("[sc2] discovering Valve service (slot %d)\n", slot);
	gatt_client_discover_primary_services_by_uuid128(gatt_handler, handle,
							 VALVE_SERVICE);
}

void valve_periodic(void)
{
	for (int i = 0; i < PP_NSLOT; i++) {
		valve_t *v = &s_valve[i];
		if (v->state != V_READY || v->report_value_handle == 0)
			continue;
		if (!gatt_client_is_ready(v->handle))
			continue;  // one GATT op at a time

		// Enable IMU once — but only when Steam isn't configuring it itself
		// (its own 0x87 writes are relayed through; don't fight them).
		if (!v->imu_enabled && !puck_steam_active()) {
			v->imu_enabled = true;
			gatt_client_write_value_of_characteristic(
				write_cb, v->handle, v->report_value_handle,
				sizeof(s_imu_on), s_imu_on);
			continue;
		}
		uint32_t t = now_ms();
		if (t - v->last_keepalive_ms >= VALVE_KEEPALIVE_MS) {
			v->last_keepalive_ms = t;
			gatt_client_write_value_of_characteristic(
				write_cb, v->handle, v->report_value_handle,
				sizeof(s_lizard_off), s_lizard_off);
		}
	}
}

void valve_feature_write(int slot, uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	if (slot < 0 || slot >= PP_NSLOT)
		return;
	valve_t *v = &s_valve[slot];
	if (v->state != V_READY || v->report_value_handle == 0)
		return;
	if (!gatt_client_is_ready(v->handle))
		return;  // busy → drop (Steam re-sends); keeps it simple and non-blocking
	if (len > 60)
		len = 60;
	uint8_t buf[64];
	buf[0] = cmd;
	buf[1] = (uint8_t)len;
	memcpy(buf + 2, payload, len);
	gatt_client_write_value_of_characteristic(write_cb, v->handle,
						  v->report_value_handle,
						  (uint16_t)(len + 2), buf);
}
