/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 *
 * libjoycon2 -- Portable C99 Joy-Con 2 BLE 5.0 peripheral & optical mouse stack.
 */

#ifndef JOYCON2_H
#define JOYCON2_H

#include "jc2_types.h"
#include "jc2_proto.h"
#include "jc2_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Power-of-two callback event ring depth for thread-safe jc2_poll() draining. */
#define JC2_CB_EVT_Q_DEPTH 16u
#define JC2_RUMBLE_MAX_LEN 64u

#define JC2_CB_EVT_CONNECTED 1u
#define JC2_CB_EVT_DISCONNECTED 2u
#define JC2_CB_EVT_PLAYER_LED 3u

typedef struct {
	uint8_t kind;
	uint8_t side;
	uint8_t aux;
	uint8_t peer_addr[6];
} jc2_cb_evt_t;

typedef struct {
	volatile bool pending;
	uint16_t len;
	uint8_t data[JC2_RUMBLE_MAX_LEN];
} jc2_rumble_slot_t;

/*
 * Caller-allocated Joy-Con 2 dual-link device state (zero dynamic allocation).
 */
typedef struct {
	jc2_config_t cfg;
	jc2_proto_state_t proto;
	jc2_input_t input;
	int32_t accum_mouse_dx;
	int32_t accum_mouse_dy;
	bool started;
	bool game_ready;
	bool both_ready_fired;
	bool fresh_sync_pair;
	volatile bool bond_dirty;
	bool adv_use_reconnect[JC2_MAX_LINKS];
	uint32_t adv_switch_ms;
	uint32_t conn_ms[JC2_MAX_LINKS];
	uint32_t ready_ms[JC2_MAX_LINKS];
	uint32_t cccd08_ms[JC2_MAX_LINKS];
	uint32_t last_cmd_ms[JC2_MAX_LINKS];
	uint32_t game_ready_ms;
	uint32_t grip_pulse_start_ms;
	uint32_t grip_pulse_end_ms;
	uint32_t last_poll_ms;
	uint8_t peer_addr[JC2_MAX_LINKS][6];
	uint8_t peer_addr_type[JC2_MAX_LINKS];
	volatile uint8_t cb_evt_head;
	volatile uint8_t cb_evt_tail;
	jc2_cb_evt_t cb_evt_q[JC2_CB_EVT_Q_DEPTH];
	jc2_rumble_slot_t rumble_mb[JC2_MAX_LINKS];
} jc2_device_t;

/* Populate a jc2_config_t with factory device MAC and authentic Joy-Con 2 defaults. */
void jc2_config_default(jc2_config_t *cfg);

/* Initialize a caller-allocated jc2_device_t (loads saved bond via callback if set). */
void jc2_init(jc2_device_t *dev, const jc2_config_t *cfg);

/* Start BLE Link Layer and begin advertising/reconnecting enabled side(s). */
void jc2_start(jc2_device_t *dev, uint32_t now_ms);

/* Disconnect active links, optionally rotate MAC epoch & wipe bonds, and advertise. */
void jc2_start_pairing(jc2_device_t *dev, bool clear_bonds, uint32_t now_ms);

/*
 * Pulse virtual grip registration buttons for Switch 2 "Change Grip/Order"
 * (L+R in JC2_MODE_PAIR_LR, or SL+SR in JC2_MODE_RIGHT_ONLY / JC2_MODE_LEFT_ONLY).
 */
void jc2_pulse_grip_lr(jc2_device_t *dev, uint32_t now_ms,
		       uint32_t duration_ms);

/*
 * Update live L+R input state; mouse_dx / mouse_dy are accumulated losslessly
 * until consumed by the next scheduled Right Joy-Con 2 BLE notification.
 */
void jc2_set_input(jc2_device_t *dev, const jc2_input_t *in);

/* Drive BLE polling, advertising/reconnect FSM, callbacks, and deferred bond saves. */
void jc2_poll(jc2_device_t *dev, uint32_t now_ms);

/* True once all enabled Joy-Con 2 side(s) are actively streaming at steady state. */
bool jc2_is_ready(const jc2_device_t *dev);

/* True if the BLE peripheral stack has been started. */
bool jc2_is_started(const jc2_device_t *dev);

/* Populate a read-only diagnostic snapshot of both links and protocol state. */
void jc2_get_status(const jc2_device_t *dev, jc2_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* JOYCON2_H */
