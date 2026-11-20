/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#ifndef JC2_PROTO_H
#define JC2_PROTO_H

#include "jc2_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Report 0x05 byte offsets (64-byte common input notification). */
#define JC2_OFF05_SEQ 0x00u
#define JC2_OFF05_BTN_R_1 0x04u
#define JC2_OFF05_BTN_R_2 0x05u
#define JC2_OFF05_BTN_L_1 0x05u
#define JC2_OFF05_BTN_L_2 0x06u
#define JC2_OFF05_LEFT_STICK 0x0Au
#define JC2_OFF05_RIGHT_STICK 0x0Du
#define JC2_OFF05_MOUSE_X 0x10u
#define JC2_OFF05_MOUSE_Y 0x12u
#define JC2_OFF05_MOUSE_SURFACE 0x14u
#define JC2_OFF05_MOUSE_STATUS 0x16u
#define JC2_OFF05_MOUSE_LIFT 0x17u
#define JC2_OFF05_BATTERY_MV 0x1Fu
#define JC2_OFF05_CHARGE_STATE 0x21u
#define JC2_OFF05_REPORT_MARKER 0x29u
#define JC2_OFF05_ACCEL_X 0x30u
#define JC2_OFF05_ACCEL_Y 0x32u
#define JC2_OFF05_ACCEL_Z 0x34u
#define JC2_OFF05_GYRO_X 0x36u
#define JC2_OFF05_GYRO_Y 0x38u
#define JC2_OFF05_GYRO_Z 0x3Au
#define JC2_OFF05_TRIGGER_L 0x3Cu
#define JC2_OFF05_TRIGGER_R 0x3Du

#define JC2_SURFACE_LIFT_ON 0x00u
#define JC2_SURFACE_LIFT_AIR 0x0Cu

typedef struct {
	bool bond_updated;
	bool led_updated;
	uint8_t led_mask;
	bool stream_started;
	bool stream_ready;
} jc2_cmd_effects_t;

typedef struct {
	jc2_bond_t bond;
	jc2_identity_t identity;
	uint8_t base_mac[6];
	uint8_t mac[JC2_MAX_LINKS][6];
	uint8_t player_led_mask;
	uint8_t last_cmd_id;
	uint8_t last_sub_id;
	uint32_t cmd_count;
	bool stream_enabled;
	bool side_stream_ready[JC2_MAX_LINKS];
	uint8_t seq05[JC2_MAX_LINKS];
	uint8_t seq08[JC2_MAX_LINKS];
	uint32_t motion_ts[JC2_MAX_LINKS];
	uint32_t motion_last_us[JC2_MAX_LINKS];
	uint32_t motion_rem_us[JC2_MAX_LINKS];
} jc2_proto_state_t;

/* 12-bit analog stick packing and calibration helpers. */
uint16_t jc2_stick_s16_to_u12(int16_t v);
void jc2_pack_stick(uint8_t out3[3], int16_t x, int16_t y);
void jc2_unpack_stick(const uint8_t in3[3], uint16_t *x_out, uint16_t *y_out);
void jc2_pack_cal_stick(int16_t x, int16_t y, int32_t cx, int32_t cy,
			uint8_t out3[3]);

/* Populate authentic Joy-Con 2 (R/L) colors and serial numbers. */
void jc2_identity_default(jc2_identity_t *out);

/* Initialize protocol state with default identity and empty bond. */
void jc2_proto_init(jc2_proto_state_t *st, const uint8_t base_mac[6],
		    const jc2_identity_t *identity);

/* Derive the Right/Left BLE MAC pair from base_mac and bond.mac_epoch. */
void jc2_proto_apply_mac_epoch(jc2_proto_state_t *st);

/* Reset the 800 Hz motion FIFO timestamp baseline for one side. */
void jc2_proto_reset_motion(jc2_proto_state_t *st, uint8_t side);

/* Read up to 72 bytes of synthesized Joy-Con 2 SPI flash for side (0=R, 1=L). */
void jc2_proto_read_flash(const jc2_proto_state_t *st, uint8_t side,
			  uint32_t addr, uint8_t *out, uint8_t read_len);

/*
 * Locate an 8-byte Joy-Con 2 command header inside a raw GATT write payload
 * (supports bare commands as well as 17-byte and 33-byte rumble prefixes).
 */
const uint8_t *jc2_proto_unwrap_cmd(const uint8_t *buf, uint16_t *io_len);

/*
 * Process a host command write and synthesize the response notification.
 * Returns the response length in bytes (0 if payload is not a valid command).
 */
uint16_t jc2_proto_handle_cmd(jc2_proto_state_t *st, uint8_t side,
			      const uint8_t *cmd, uint16_t len,
			      uint8_t resp_out[JC2_CMD_RESP_MAX],
			      jc2_cmd_effects_t *effects);

/* Build a 64-byte Joy-Con 2 common Input Report 0x05 for side (0=R, 1=L). */
void jc2_proto_build_report05(uint8_t side, uint8_t seq, const jc2_input_t *in,
			      uint8_t out64[JC2_REPORT05_SIZE]);

/*
 * Build a 63-byte Joy-Con 2 BLE Input Report 0x08 (Right) or 0x07 (Left).
 * Pass has_now_us=true with monotonic microsecond timestamp for live 800 Hz
 * FIFO delta pacing; pass has_now_us=false for fixed 12-tick (15 ms) framing.
 */
void jc2_proto_build_report0708(jc2_proto_state_t *st, uint8_t side,
				uint8_t seq, const jc2_input_t *in,
				uint32_t now_us, bool has_now_us,
				uint8_t out63[JC2_REPORT0708_SIZE]);

/* Build Switch 2 SoC-filtered 31-byte ADV_IND and SCAN_RSP payloads. */
void jc2_proto_build_adv(const jc2_proto_state_t *st, uint8_t side,
			 bool reconnect, uint8_t adv_out[31],
			 uint8_t scan_rsp_out[20], uint8_t *scan_rsp_len);

#ifdef __cplusplus
}
#endif

#endif /* JC2_PROTO_H */
