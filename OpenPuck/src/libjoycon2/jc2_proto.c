/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#include "jc2_proto.h"
#include "jc2_crypto.h"

#include <string.h>

/* Report 0x05 button bit definitions. */
#define R05_R_Y 0x01u
#define R05_R_B 0x02u
#define R05_R_X 0x04u
#define R05_R_A 0x08u
#define R05_R_SR 0x10u
#define R05_R_SL 0x20u
#define R05_R_R 0x40u
#define R05_R_ZR 0x80u

#define R05_R5_GR 0x01u
#define R05_R5_PLUS 0x02u
#define R05_R5_R3 0x04u
#define R05_R5_HOME 0x10u
#define R05_R5_CHAT 0x40u

#define R05_L5_MINUS 0x01u
#define R05_L5_GL 0x02u
#define R05_L5_L3 0x08u
#define R05_L5_CAPTURE 0x20u

#define R05_L_DOWN 0x01u
#define R05_L_UP 0x02u
#define R05_L_RIGHT 0x04u
#define R05_L_LEFT 0x08u
#define R05_L_SR 0x10u
#define R05_L_SL 0x20u
#define R05_L_L 0x40u
#define R05_L_ZL 0x80u

/* Report 0x08 (Right) and Report 0x07 (Left) button bit definitions. */
#define R08_B0_R3 0x80u
#define R08_B0_PLUS 0x40u
#define R08_B0_ZR 0x20u
#define R08_B0_R 0x10u
#define R08_B0_X 0x08u
#define R08_B0_Y 0x04u
#define R08_B0_A 0x02u
#define R08_B0_B 0x01u
#define R08_B1_SL 0x80u
#define R08_B1_SR 0x40u
#define R08_B1_CHAT 0x10u
#define R08_B1_HOME 0x01u

#define R07_B0_L3 0x80u
#define R07_B0_MINUS 0x40u
#define R07_B0_ZL 0x20u
#define R07_B0_L 0x10u
#define R07_B0_UP 0x08u
#define R07_B0_LEFT 0x04u
#define R07_B0_RIGHT 0x02u
#define R07_B0_DOWN 0x01u
#define R07_B1_SL 0x80u
#define R07_B1_SR 0x40u
#define R07_B1_CAPTURE 0x01u

uint16_t jc2_stick_s16_to_u12(int16_t v)
{
	int32_t a = 2048 + ((int32_t)v >> 4);

	if (a < 0)
		return 0u;
	if (a > 4095)
		return 4095u;
	return (uint16_t)a;
}

void jc2_pack_stick(uint8_t out3[3], int16_t x, int16_t y)
{
	uint16_t x12 = jc2_stick_s16_to_u12(x);
	uint16_t y12 = jc2_stick_s16_to_u12(y);

	out3[0] = (uint8_t)(x12 & 0xFFu);
	out3[1] = (uint8_t)(((x12 >> 8) & 0x0Fu) | ((y12 & 0x0Fu) << 4));
	out3[2] = (uint8_t)((y12 >> 4) & 0xFFu);
}

void jc2_unpack_stick(const uint8_t in3[3], uint16_t *x_out, uint16_t *y_out)
{
	if (x_out)
		*x_out = (uint16_t)(((uint16_t)(in3[1] & 0x0Fu) << 8) | in3[0]);
	if (y_out)
		*y_out = (uint16_t)(((uint16_t)in3[2] << 4) |
				    ((in3[1] >> 4) & 0x0Fu));
}

void jc2_pack_cal_stick(int16_t x, int16_t y, int32_t cx, int32_t cy,
			uint8_t out3[3])
{
	int32_t sx = (int32_t)jc2_stick_s16_to_u12(x);
	int32_t sy = (int32_t)jc2_stick_s16_to_u12(y);
	int32_t ox = cx + ((sx - 2048) * 3) / 4;
	int32_t oy = cy + ((sy - 2048) * 3) / 4;

	if (ox < 256)
		ox = 256;
	else if (ox > 3840)
		ox = 3840;
	if (oy < 256)
		oy = 256;
	else if (oy > 3840)
		oy = 3840;
	out3[0] = (uint8_t)(ox & 0xFF);
	out3[1] = (uint8_t)(((ox >> 8) & 0x0F) | ((oy & 0x0F) << 4));
	out3[2] = (uint8_t)((oy >> 4) & 0xFF);
}

void jc2_identity_default(jc2_identity_t *out)
{
	static const uint8_t def_body[3] = { 0x32, 0x32, 0x32 };
	static const uint8_t def_btn[3] = { 0xAA, 0xAA, 0xAA };
	static const uint8_t def_acc_r[3] = { 0xFF, 0x8C, 0x5F };
	static const uint8_t def_acc_l[3] = { 0x9B, 0xE1, 0xE6 };

	if (!out)
		return;
	memset(out, 0, sizeof(*out));
	memcpy(out->body_rgb[JC2_SIDE_RIGHT], def_body, 3);
	memcpy(out->body_rgb[JC2_SIDE_LEFT], def_body, 3);
	memcpy(out->button_rgb[JC2_SIDE_RIGHT], def_btn, 3);
	memcpy(out->button_rgb[JC2_SIDE_LEFT], def_btn, 3);
	memcpy(out->accent_rgb[JC2_SIDE_RIGHT], def_acc_r, 3);
	memcpy(out->accent_rgb[JC2_SIDE_LEFT], def_acc_l, 3);
	memcpy(out->serial[JC2_SIDE_RIGHT], "HCW00000000001", 14);
	memcpy(out->serial[JC2_SIDE_LEFT], "HBW00000000002", 14);
}

void jc2_proto_init(jc2_proto_state_t *st, const uint8_t base_mac[6],
		    const jc2_identity_t *identity)
{
	if (!st)
		return;
	memset(st, 0, sizeof(*st));
	if (base_mac)
		memcpy(st->base_mac, base_mac, 6);
	if (identity)
		st->identity = *identity;
	else
		jc2_identity_default(&st->identity);
	st->bond.magic = JC2_BOND_MAGIC;
	st->bond.mac_epoch = JC2_DEFAULT_MAC_EPOCH;
	st->player_led_mask = 0x01u;
	st->stream_enabled = true;
	jc2_proto_apply_mac_epoch(st);
}

void jc2_proto_apply_mac_epoch(jc2_proto_state_t *st)
{
	uint8_t mr[6];
	uint8_t ml[6];

	if (!st)
		return;
#if JC2_DEFAULT_MAC_EPOCH > 0u
	if (st->bond.mac_epoch < JC2_DEFAULT_MAC_EPOCH)
		st->bond.mac_epoch = JC2_DEFAULT_MAC_EPOCH;
#endif

	memcpy(mr, st->base_mac, 6);
	/* Nintendo OUI 48:F1:EB (stored LSB-first on the BLE air interface). */
	mr[5] = 0x48u;
	mr[4] = 0xF1u;
	mr[3] = 0xEBu;
	mr[2] ^= (uint8_t)(0x3Cu + st->bond.mac_epoch * 0x13u);
	mr[1] ^= (uint8_t)(st->bond.mac_epoch * 0x29u);
	mr[0] = (uint8_t)((st->base_mac[0] & 0xFEu) | 0x01u);

	memcpy(ml, mr, 6);
	ml[0] = (uint8_t)(mr[0] ^ 0x01u);

	memcpy(st->mac[JC2_SIDE_RIGHT], mr, 6);
	memcpy(st->mac[JC2_SIDE_LEFT], ml, 6);
}

void jc2_proto_reset_motion(jc2_proto_state_t *st, uint8_t side)
{
	uint8_t s;

	if (!st)
		return;
	s = side & 1u;
	st->motion_ts[s] = 0u;
	st->motion_last_us[s] = 0u;
	st->motion_rem_us[s] = 0u;
}

static void read_flash_block(const jc2_proto_state_t *st, uint8_t side,
			     uint32_t block_addr, uint8_t block[64])
{
	uint8_t s = side & 1u;

	memset(block, 0xFF, 64);
	if (block_addr == 0x00013000u) {
		block[0] = 0x01u;
		block[1] = 0x00u;
		memcpy(&block[2], st->identity.serial[s], 16);
		block[18] = 0x7Eu;
		block[19] = 0x05u;
		block[20] = (s == JC2_SIDE_RIGHT) ? 0x66u : 0x67u;
		block[21] = 0x20u;
		block[22] = 0x01u;
		block[23] = 0x08u;
		block[24] = 0x02u;
		memcpy(&block[25], st->identity.body_rgb[s], 3);
		memcpy(&block[28], st->identity.button_rgb[s], 3);
		memcpy(&block[31], st->identity.accent_rgb[s], 3);
		memcpy(&block[34], st->identity.body_rgb[s], 3);
	} else if (block_addr == 0x00013040u) {
		static const uint8_t r_13040[16] = { 0xE2, 0xD4, 0xC3, 0x41,
						     0x9F, 0xBA, 0xF6, 0xBA,
						     0x72, 0xA7, 0x38, 0xBC,
						     0x8C, 0x0F, 0xB7, 0xB8 };
		static const uint8_t l_13040[16] = { 0x70, 0x88, 0xD6, 0x41,
						     0xAD, 0xA9, 0xEE, 0x3B,
						     0x71, 0xD2, 0xE4, 0x3B,
						     0xD9, 0x75, 0x05, 0x3C };

		memcpy(block, (s == JC2_SIDE_RIGHT) ? r_13040 : l_13040, 16);
		block[32] = (s == JC2_SIDE_RIGHT) ? 0x5Bu : 0x52u;
		block[33] = 0x09u;
		block[34] = 0x00u;
		block[35] = 0x00u;
	} else if (block_addr == 0x00013080u) {
		static const uint8_t hdr_13080[40] = {
			0x03, 0xAC, 0xCA, 0xAA, 0x53, 0x35, 0x55, 0x93,
			0x30, 0x09, 0x93, 0x30, 0x09, 0xD2, 0x20, 0x0D,
			0xD2, 0x20, 0x0D, 0xCC, 0xAC, 0xD9, 0xCC, 0xAC,
			0xD9, 0xD4, 0x42, 0x2D, 0xD4, 0x42, 0x2D, 0x8F,
			0xF4, 0x48, 0x8F, 0xF4, 0x48, 0x0F, 0xFF, 0xFF
		};
		static const uint8_t r_cal[9] = { 0xBD, 0x47, 0x7E, 0x93, 0xA4,
						  0x49, 0xB8, 0x34, 0x4C };
		static const uint8_t l_cal[9] = { 0xF2, 0xB7, 0x7C, 0xD1, 0x84,
						  0x4B, 0x98, 0xA4, 0x4A };

		memcpy(block, hdr_13080, sizeof(hdr_13080));
		memcpy(&block[40], (s == JC2_SIDE_RIGHT) ? r_cal : l_cal, 9);
	} else if (block_addr == 0x00013100u) {
		static const uint8_t r_13100[24] = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x59, 0x7C, 0x1D, 0x41,
			0xC4, 0x76, 0xDC, 0xBC, 0xCA, 0x46, 0xCD, 0x3D
		};
		static const uint8_t l_13100[24] = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x9D, 0x18, 0x1D, 0x41,
			0x35, 0x1E, 0xE9, 0x3D, 0xA1, 0xF5, 0x98, 0xBE
		};

		memcpy(block, (s == JC2_SIDE_RIGHT) ? r_13100 : l_13100, 24);
	} else if (block_addr == 0x00013140u) {
		static const uint8_t b_13140[9] = { 0x00, 0xD7, 0xA3,
						    0xBC, 0x41, 0xD7,
						    0xA3, 0xBC, 0x41 };

		memcpy(block, b_13140, sizeof(b_13140));
	} else if (block_addr == 0x00013E00u) {
		static const uint8_t b_13e20[4] = { 0x01, 0x02, 0x02, 0x02 };
		static const uint8_t b_13e30[10] = { 0x03, 0x02, 0x04, 0x01,
						     0x05, 0x02, 0x06, 0x02,
						     0x07, 0x02 };
		const char *lot = (s == JC2_SIDE_RIGHT) ? "00000000A00A0001M" :
							  "00000000A00A0002M";

		memset(block, 0x00, 32);
		memcpy(block, lot, 17);
		memcpy(&block[32], b_13e20, sizeof(b_13e20));
		memcpy(&block[48], b_13e30, sizeof(b_13e30));
	} else if (block_addr == 0x00013EC0u) {
		block[59] = 0x01u;
		block[60] = 0x00u;
		block[61] = (s == JC2_SIDE_RIGHT) ? 0x6Fu : 0x70u;
		block[62] = (s == JC2_SIDE_RIGHT) ? 0x7Du : 0x5Au;
		block[63] = 0xFFu;
	} else if (block_addr == 0x001FA000u) {
		if (st->bond.has_host && st->bond.has_ltk[s]) {
			int i;

			memset(block, 0x00, 64);
			block[0] = 0x02u;
			block[1] = 0xC4u;
			block[2] = 0x11u;
			for (i = 0; i < 6; i++) {
				block[8 + i] = st->bond.host_addr[5 - i];
				block[48 + i] = st->bond.host_addr[5 - i];
			}
			block[53] ^= 0x01u;
			memcpy(&block[26], st->bond.ltk[s], 16);
		}
	} else if (block_addr == 0x001FA040u) {
		if (st->bond.has_host && st->bond.has_ltk[s]) {
			memcpy(&block[2], st->bond.ltk[s], 16);
			block[18] = 0xA5u;
			block[19] = 0xA5u;
			block[20] = 0xFDu;
		}
	} else if (block_addr == 0x001FB000u) {
		static const uint8_t b_1fb000[16] = { 0x02, 0x00, 0x02, 0x00,
						      0x01, 0x00, 0x11, 0x03,
						      0xFF, 0xFF, 0xFF, 0xFF,
						      0xFF, 0xFF, 0x06, 0x01 };

		memcpy(block, b_1fb000, sizeof(b_1fb000));
	} else if (block_addr == 0x001FB040u) {
		static const uint8_t b_1fb040_lo[30] = {
			0xFF, 0xFF, 0x02, 0x00, 0x01, 0x00, 0xC8, 0x00,
			0x78, 0x00, 0x98, 0x0E, 0xDD, 0x0D, 0x4B, 0x0D,
			0xE5, 0x0C, 0xB2, 0x0C, 0x76, 0x0C, 0x32, 0x0C,
			0x18, 0x0C, 0xA9, 0x0B, 0xAA, 0x0A
		};
		static const uint8_t b_1fb040_hi[12] = {
			0x98, 0x0E, 0xA9, 0x0B, 0xFF, 0xFF,
			0x54, 0x09, 0xFF, 0xFF, 0x54, 0x0D
		};

		memcpy(block, b_1fb040_lo, sizeof(b_1fb040_lo));
		memcpy(&block[48], b_1fb040_hi, sizeof(b_1fb040_hi));
		block[60] = 0x54u;
		block[61] = 0x0Cu;
		block[62] = 0xA9u;
		block[63] = 0x0Bu;
	} else if (block_addr == 0x001FD000u) {
		memset(&block[16], 0x00, 4);
	}
}

void jc2_proto_read_flash(const jc2_proto_state_t *st, uint8_t side,
			  uint32_t addr, uint8_t *out, uint8_t read_len)
{
	uint8_t copied = 0u;

	if (!st || !out || read_len == 0u)
		return;
	while (copied < read_len) {
		uint32_t cur = addr + (uint32_t)copied;
		uint32_t base = cur & ~0x3Fu;
		uint32_t off = cur & 0x3Fu;
		uint8_t chunk = (uint8_t)(64u - off);
		uint8_t rem = (uint8_t)(read_len - copied);
		uint8_t blk[64];

		if (chunk > rem)
			chunk = rem;
		read_flash_block(st, side, base, blk);
		memcpy(&out[copied], &blk[off], chunk);
		copied = (uint8_t)(copied + chunk);
	}
}

static inline bool is_valid_cmd_hdr(const uint8_t *hdr, uint16_t rem_len)
{
	return hdr && rem_len >= 8u && hdr[0] >= 0x01u && hdr[0] <= 0x20u &&
	       hdr[1] == 0x91u && hdr[2] <= 0x01u && hdr[6] == 0x00u &&
	       hdr[7] == 0x00u && hdr[5] <= 80u &&
	       rem_len >= (uint16_t)(8u + hdr[5]);
}

const uint8_t *jc2_proto_unwrap_cmd(const uint8_t *buf, uint16_t *io_len)
{
	uint16_t len;

	if (!buf || !io_len)
		return NULL;
	len = *io_len;
	if (is_valid_cmd_hdr(buf, len))
		return buf;
	if (len >= 25u && is_valid_cmd_hdr(buf + 17u, (uint16_t)(len - 17u))) {
		*io_len = (uint16_t)(len - 17u);
		return buf + 17u;
	}
	if (len >= 41u && is_valid_cmd_hdr(buf + 33u, (uint16_t)(len - 33u))) {
		*io_len = (uint16_t)(len - 33u);
		return buf + 33u;
	}
	return NULL;
}

static uint16_t handle_spi_read(const jc2_proto_state_t *st, uint8_t side,
				const uint8_t *cmd, uint16_t len,
				uint8_t resp_out[JC2_CMD_RESP_MAX])
{
	uint8_t sub_id = cmd[3];
	uint8_t read_len;
	uint32_t addr;

	if ((sub_id != 0x04u && sub_id != 0x01u) || len < 16u)
		return 8u;
	read_len = (sub_id == 0x04u) ? cmd[8] : 0x40u;
	if (read_len > 0x48u)
		read_len = 0x48u;
	addr = (uint32_t)cmd[12] | ((uint32_t)cmd[13] << 8) |
	       ((uint32_t)cmd[14] << 16) | ((uint32_t)cmd[15] << 24);
	resp_out[8] = read_len;
	memcpy(&resp_out[12], &cmd[12], 4);
	jc2_proto_read_flash(st, side, addr, &resp_out[16], read_len);
	return (uint16_t)(16u + read_len);
}

static uint16_t handle_pairing_15(jc2_proto_state_t *st, uint8_t side,
				  const uint8_t *cmd, uint16_t len,
				  uint8_t resp_out[JC2_CMD_RESP_MAX],
				  jc2_cmd_effects_t *eff)
{
	uint8_t sub_id = cmd[3];

	if (sub_id == 0x01u) {
		if (len >= 10u && cmd[5] == 0x02u && cmd[9] == 0x00u) {
			resp_out[8] = 0x02u;
			resp_out[9] = 0x01u;
			resp_out[10] = 0x02u;
			return 11u;
		}
		if (len >= 16u && cmd[9] >= 1u) {
			memcpy(st->bond.host_addr, &cmd[10], 6);
			st->bond.has_host = 1u;
		}
		resp_out[8] = 0x01u;
		resp_out[9] = 0x04u;
		resp_out[10] = 0x01u;
		memcpy(&resp_out[11], st->mac[side], 6);
		return 17u;
	}
	if (sub_id == 0x04u && len >= 25u) {
		static const uint8_t ltk_b1[16] = { 0x5C, 0xF6, 0xEE, 0x79,
						    0x2C, 0xDF, 0x05, 0xE1,
						    0xBA, 0x2B, 0x63, 0x25,
						    0xC4, 0x1A, 0x5F, 0x10 };
		int i;

		for (i = 0; i < 16; i++)
			st->bond.ltk[side][i] = cmd[9 + i] ^ ltk_b1[i];
		st->bond.has_ltk[side] = 1u;
		resp_out[8] = 0x01u;
		memcpy(&resp_out[9], ltk_b1, 16);
		return 25u;
	}
	if (sub_id == 0x02u && len >= 25u) {
		uint8_t key_be[16];
		uint8_t pt_be[16];
		int i;

		resp_out[8] = 0x01u;
		for (i = 0; i < 16; i++) {
			key_be[i] = st->bond.ltk[side][15 - i];
			pt_be[i] = cmd[9 + 15 - i];
		}
		jc2_aes128_ecb_be(key_be, pt_be, &resp_out[9]);
		return 25u;
	}
	if (sub_id == 0x03u) {
		resp_out[8] = 0x01u;
		if (eff)
			eff->bond_updated = true;
		return 9u;
	}
	return 8u;
}

static uint16_t handle_feature_0c(jc2_proto_state_t *st, const uint8_t *cmd,
				  uint16_t len,
				  uint8_t resp_out[JC2_CMD_RESP_MAX],
				  jc2_cmd_effects_t *eff)
{
	uint8_t sub_id = cmd[3];

	if (sub_id == 0x01u) {
		uint8_t flags = (len >= 9u) ? cmd[8] : 0x37u;

		memset(&resp_out[8], 0, 12);
		resp_out[12] = (flags & 0x01u) ? 0x07u : 0x00u;
		resp_out[13] = (flags & 0x02u) ? 0x07u : 0x00u;
		resp_out[14] = (flags & 0x04u) ? 0x03u : 0x00u;
		resp_out[15] = (flags & 0x80u) ? 0x03u : 0x00u;
		resp_out[16] = (flags & 0x10u) ? 0x03u : 0x00u;
		resp_out[17] = (flags & 0x20u) ? 0x03u : 0x00u;
		return 20u;
	}
	if (sub_id == 0x04u) {
		st->stream_enabled = true;
		if (eff)
			eff->stream_started = true;
		return 12u;
	}
	if (sub_id == 0x06u) {
		resp_out[12] = (len >= 13u && cmd[12] != 0u) ? cmd[12] : 0x02u;
		resp_out[16] = 0x00u;
		resp_out[17] = (len >= 17u && cmd[16] == 0x8Au) ? 0x76u : 0x50u;
		return 48u;
	}
	return 12u;
}

static uint16_t handle_power_0b(uint8_t side, uint8_t sub_id,
				uint8_t resp_out[JC2_CMD_RESP_MAX])
{
	if (sub_id == 0x03u) {
		resp_out[8] = (side == JC2_SIDE_RIGHT) ? 0x54u : 0x4Cu;
		resp_out[9] = 0x0Du;
	} else if (sub_id == 0x04u) {
		resp_out[10] = 0x03u;
	} else if (sub_id == 0x06u) {
		resp_out[8] = 0x11u;
	}
	return 12u;
}

static uint16_t handle_mouse_13(uint8_t sub_id,
				uint8_t resp_out[JC2_CMD_RESP_MAX])
{
	resp_out[8] = 0x01u;
	if (sub_id == 0x02u) {
		resp_out[12] = 0xC3u;
		resp_out[13] = 0x09u;
		resp_out[14] = 0xC3u;
		resp_out[15] = 0x09u;
		return 16u;
	}
	if (sub_id == 0x03u) {
		resp_out[12] = 0x6Au;
		resp_out[13] = 0x6Au;
		return 16u;
	}
	return 12u;
}

uint16_t jc2_proto_handle_cmd(jc2_proto_state_t *st, uint8_t side,
			      const uint8_t *cmd, uint16_t len,
			      uint8_t resp_out[JC2_CMD_RESP_MAX],
			      jc2_cmd_effects_t *effects)
{
	uint8_t cmd_id;
	uint8_t transport;
	uint8_t sub_id;

	if (effects)
		memset(effects, 0, sizeof(*effects));
	if (!st || !cmd || !resp_out || len < 8u)
		return 0u;
	cmd = jc2_proto_unwrap_cmd(cmd, &len);
	if (!cmd)
		return 0u;

	side &= 1u;
	cmd_id = cmd[0];
	transport = cmd[2];
	sub_id = cmd[3];
	st->last_cmd_id = cmd_id;
	st->last_sub_id = sub_id;
	st->cmd_count++;

	memset(resp_out, 0, JC2_CMD_RESP_MAX);
	resp_out[0] = cmd_id;
	resp_out[1] = 0x01u;
	resp_out[2] = transport;
	resp_out[3] = sub_id;
	resp_out[4] = 0x10u;
	resp_out[5] = 0x78u;
	resp_out[6] = 0x00u;
	resp_out[7] = 0x00u;

	switch (cmd_id) {
	case JC2_CMD_CONN_PARAM:
		if (sub_id == 0x0Cu) {
			st->side_stream_ready[side] = true;
			if (effects)
				effects->stream_ready = true;
			resp_out[8] = 0x61u;
			resp_out[9] = 0x12u;
			resp_out[10] = 0x50u;
			resp_out[11] = 0x0Du;
			return 12u;
		}
		return 8u;

	case JC2_CMD_SPI:
		return handle_spi_read(st, side, cmd, len, resp_out);

	case JC2_CMD_HOST_INFO:
		if (sub_id == 0x07u && len >= 30u) {
			if (!st->bond.has_host) {
				memcpy(st->bond.host_addr, &cmd[8], 6);
				st->bond.has_host = 1u;
			}
			if (!st->bond.has_ltk[side]) {
				int i;

				for (i = 0; i < 16; i++)
					st->bond.ltk[side][i] =
						cmd[14 + 15 - i];
				st->bond.has_ltk[side] = 1u;
			}
			if (effects)
				effects->bond_updated = true;
		}
		return 8u;

	case JC2_CMD_SET_MODE:
		if (sub_id == 0x01u) {
			resp_out[8] = 0x00u;
			return 9u;
		}
		return 8u;

	case JC2_CMD_LED:
		if (sub_id == 0x07u && len >= 9u) {
			st->player_led_mask = cmd[8];
			if (effects) {
				effects->led_updated = true;
				effects->led_mask = cmd[8];
			}
		}
		return 8u;

	case JC2_CMD_POWER:
		return handle_power_0b(side, sub_id, resp_out);

	case JC2_CMD_FEATURE:
		return handle_feature_0c(st, cmd, len, resp_out, effects);

	case JC2_CMD_FW_INFO:
		if (sub_id == 0x01u) {
			uint8_t fw[12] = { 0x02, 0x01, 0x04, 0x00, 0x0C, 0x00,
					   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

			fw[3] = (side == JC2_SIDE_RIGHT) ? 0x01u : 0x00u;
			memcpy(&resp_out[8], fw, sizeof(fw));
			return 20u;
		}
		return 8u;

	case JC2_CMD_IMU:
		if (sub_id == 0x03u) {
			static const uint8_t r1103[29] = {
				0x01, 0x20, 0x03, 0x00, 0x00, 0x0A, 0xE8, 0x1C,
				0x3B, 0x79, 0x7D, 0x8B, 0x3A, 0x0A, 0xE8, 0x9C,
				0x42, 0x58, 0xA0, 0x0B, 0x42, 0x0A, 0xE8, 0x9C,
				0x41, 0x58, 0xA0, 0x0B, 0x41
			};

			memcpy(&resp_out[8], r1103, sizeof(r1103));
			return 37u;
		}
		if (sub_id == 0x01u) {
			resp_out[8] = 0x01u;
			return 12u;
		}
		return 8u;

	case JC2_CMD_MOUSE:
		return handle_mouse_13(sub_id, resp_out);

	case JC2_CMD_PAIR:
		return handle_pairing_15(st, side, cmd, len, resp_out, effects);

	case JC2_CMD_EXT_INFO:
		if (sub_id == 0x01u)
			return 32u;
		return 8u;

	case JC2_CMD_AUDIO:
		if (sub_id == 0x01u) {
			/* Pro Controller 2 headset probe: report unsupported. */
			resp_out[1] = 0x04u;
			return 8u;
		}
		if (sub_id == 0x03u) {
			resp_out[8] = (len >= 9u) ? cmd[8] : 0x07u;
			return 9u;
		}
		return 8u;

	default:
		return 8u;
	}
}

static void write_imu05(uint8_t *rep, int16_t ax, int16_t ay, int16_t az,
			int16_t gx, int16_t gy, int16_t gz)
{
	jc2_write_s16_le(&rep[JC2_OFF05_ACCEL_X], ax);
	jc2_write_s16_le(&rep[JC2_OFF05_ACCEL_Y], ay);
	jc2_write_s16_le(&rep[JC2_OFF05_ACCEL_Z], az);
	jc2_write_s16_le(&rep[JC2_OFF05_GYRO_X], gx);
	jc2_write_s16_le(&rep[JC2_OFF05_GYRO_Y], gy);
	jc2_write_s16_le(&rep[JC2_OFF05_GYRO_Z], gz);
}

void jc2_proto_build_report05(uint8_t side, uint8_t seq, const jc2_input_t *in,
			      uint8_t out64[JC2_REPORT05_SIZE])
{
	uint8_t s = side & 1u;
	uint32_t b;
	uint16_t mv;

	if (!out64)
		return;
	memset(out64, 0, JC2_REPORT05_SIZE);
	if (!in)
		return;

	b = in->buttons;
	mv = in->battery_mv ? in->battery_mv : 4000u;
	out64[JC2_OFF05_SEQ] = seq;
	jc2_write_u16_le(&out64[JC2_OFF05_BATTERY_MV], mv);
	out64[JC2_OFF05_CHARGE_STATE] = in->charge_state;
	out64[JC2_OFF05_REPORT_MARKER] = 0x01u;

	if (s == JC2_SIDE_LEFT) {
		uint8_t b5 = 0u;
		uint8_t b6 = 0u;

		if (b & JC2_BTN_MINUS)
			b5 |= R05_L5_MINUS;
		if (b & JC2_BTN_GL)
			b5 |= R05_L5_GL;
		if (b & JC2_BTN_L3)
			b5 |= R05_L5_L3;
		if (b & JC2_BTN_CAPTURE)
			b5 |= R05_L5_CAPTURE;
		if (b & JC2_BTN_DOWN)
			b6 |= R05_L_DOWN;
		if (b & JC2_BTN_UP)
			b6 |= R05_L_UP;
		if (b & JC2_BTN_RIGHT)
			b6 |= R05_L_RIGHT;
		if (b & JC2_BTN_LEFT)
			b6 |= R05_L_LEFT;
		if (b & JC2_BTN_L_SR)
			b6 |= R05_L_SR;
		if (b & JC2_BTN_L_SL)
			b6 |= R05_L_SL;
		if (b & JC2_BTN_L)
			b6 |= R05_L_L;
		if (b & JC2_BTN_ZL)
			b6 |= R05_L_ZL;

		out64[JC2_OFF05_BTN_L_1] = b5;
		out64[JC2_OFF05_BTN_L_2] = b6;
		jc2_pack_stick(&out64[JC2_OFF05_LEFT_STICK], in->lx, in->ly);
		jc2_pack_stick(&out64[JC2_OFF05_RIGHT_STICK], 0, 0);
		out64[JC2_OFF05_MOUSE_LIFT] = JC2_SURFACE_LIFT_AIR;
		write_imu05(out64, in->accel_x, in->accel_y, in->accel_z,
			    in->gyro_x, in->gyro_y, in->gyro_z);
		out64[JC2_OFF05_TRIGGER_L] = in->left_trigger;
		return;
	}

	{
		uint8_t b4 = 0u;
		uint8_t b5 = 0u;

		if (b & JC2_BTN_Y)
			b4 |= R05_R_Y;
		if (b & JC2_BTN_B)
			b4 |= R05_R_B;
		if (b & JC2_BTN_X)
			b4 |= R05_R_X;
		if (b & JC2_BTN_A)
			b4 |= R05_R_A;
		if (b & JC2_BTN_R_SR)
			b4 |= R05_R_SR;
		if (b & JC2_BTN_R_SL)
			b4 |= R05_R_SL;
		if (b & JC2_BTN_R)
			b4 |= R05_R_R;
		if (b & JC2_BTN_ZR)
			b4 |= R05_R_ZR;

		if (b & JC2_BTN_GR)
			b5 |= R05_R5_GR;
		if (b & JC2_BTN_PLUS)
			b5 |= R05_R5_PLUS;
		if (b & JC2_BTN_R3)
			b5 |= R05_R5_R3;
		if (b & JC2_BTN_HOME)
			b5 |= R05_R5_HOME;
		if (b & JC2_BTN_CHAT)
			b5 |= R05_R5_CHAT;

		out64[JC2_OFF05_BTN_R_1] = b4;
		out64[JC2_OFF05_BTN_R_2] = b5;
		jc2_pack_stick(&out64[JC2_OFF05_LEFT_STICK], 0, 0);
		jc2_pack_stick(&out64[JC2_OFF05_RIGHT_STICK], in->rx, in->ry);

		if (in->mouse_on_surface) {
			jc2_write_s16_le(&out64[JC2_OFF05_MOUSE_X],
					 in->mouse_dx);
			jc2_write_s16_le(&out64[JC2_OFF05_MOUSE_Y],
					 in->mouse_dy);
			out64[JC2_OFF05_MOUSE_SURFACE] = 0x16u;
			out64[JC2_OFF05_MOUSE_STATUS] = 0x02u;
			out64[JC2_OFF05_MOUSE_LIFT] = JC2_SURFACE_LIFT_ON;
			write_imu05(out64, 0, 0, JC2_ACCEL_1G, 0, 0, 0);
		} else {
			out64[JC2_OFF05_MOUSE_SURFACE] = 0x00u;
			out64[JC2_OFF05_MOUSE_STATUS] = 0x00u;
			out64[JC2_OFF05_MOUSE_LIFT] = JC2_SURFACE_LIFT_AIR;
			write_imu05(out64, in->accel_x, in->accel_y,
				    in->accel_z, in->gyro_x, in->gyro_y,
				    in->gyro_z);
		}
		out64[JC2_OFF05_TRIGGER_R] = in->right_trigger;
	}
}

static void pack_buttons_0708(uint8_t side, const jc2_input_t *in,
			      uint8_t out63[JC2_REPORT0708_SIZE])
{
	uint32_t b = in->buttons;

	if (side == JC2_SIDE_RIGHT) {
		uint8_t r0 = 0u;
		uint8_t r1 = 0u;

		if (b & JC2_BTN_R3)
			r0 |= R08_B0_R3;
		if (b & JC2_BTN_PLUS)
			r0 |= R08_B0_PLUS;
		if (b & JC2_BTN_ZR)
			r0 |= R08_B0_ZR;
		if (b & JC2_BTN_R)
			r0 |= R08_B0_R;
		if (b & JC2_BTN_X)
			r0 |= R08_B0_X;
		if (b & JC2_BTN_Y)
			r0 |= R08_B0_Y;
		if (b & JC2_BTN_A)
			r0 |= R08_B0_A;
		if (b & JC2_BTN_B)
			r0 |= R08_B0_B;
		if (b & JC2_BTN_R_SL)
			r1 |= R08_B1_SL;
		if (b & JC2_BTN_R_SR)
			r1 |= R08_B1_SR;
		if (b & JC2_BTN_CHAT)
			r1 |= R08_B1_CHAT;
		if (b & JC2_BTN_HOME)
			r1 |= R08_B1_HOME;
		out63[2] = r0;
		out63[3] = r1;
		out63[4] = 0x07u;
		jc2_pack_cal_stick(in->rx, in->ry, 0x7BD, 0x7E4, &out63[5]);
	} else {
		uint8_t l0 = 0u;
		uint8_t l1 = 0u;

		if (b & JC2_BTN_L3)
			l0 |= R07_B0_L3;
		if (b & JC2_BTN_MINUS)
			l0 |= R07_B0_MINUS;
		if (b & JC2_BTN_ZL)
			l0 |= R07_B0_ZL;
		if (b & JC2_BTN_L)
			l0 |= R07_B0_L;
		if (b & JC2_BTN_UP)
			l0 |= R07_B0_UP;
		if (b & JC2_BTN_LEFT)
			l0 |= R07_B0_LEFT;
		if (b & JC2_BTN_RIGHT)
			l0 |= R07_B0_RIGHT;
		if (b & JC2_BTN_DOWN)
			l0 |= R07_B0_DOWN;
		if (b & JC2_BTN_L_SL)
			l1 |= R07_B1_SL;
		if (b & JC2_BTN_L_SR)
			l1 |= R07_B1_SR;
		if (b & JC2_BTN_CAPTURE)
			l1 |= R07_B1_CAPTURE;
		out63[2] = l0;
		out63[3] = l1;
		out63[4] = 0x08u;
		jc2_pack_cal_stick(in->lx, in->ly, 0x7F2, 0x7CB, &out63[5]);
	}
}

void jc2_proto_build_report0708(jc2_proto_state_t *st, uint8_t side,
				uint8_t seq, const jc2_input_t *in,
				uint32_t now_us, bool has_now_us,
				uint8_t out63[JC2_REPORT0708_SIZE])
{
	static const uint8_t imu_flat_26[26] = {
		0x00, 0x22, 0x61, 0xFE, 0x01, 0x10, 0xD2, 0x1B, 0x01,
		0x55, 0x74, 0x80, 0x68, 0x2B, 0x77, 0x06, 0x70, 0x8F,
		0xA0, 0xF3, 0x60, 0xEF, 0x3F, 0x08, 0xE0, 0x03
	};
	static const uint8_t imu_mouse_26[26] = {
		0x03, 0x14, 0x99, 0xA1, 0x00, 0x82, 0xF3, 0xD2, 0x01,
		0xE5, 0x22, 0x6F, 0x10, 0xB8, 0xBE, 0x0F, 0x8E, 0xC8,
		0x99, 0xFE, 0x9C, 0xF8, 0x56, 0xFF, 0x00, 0x04
	};
	static const uint8_t imu_stationary[2][36] = {
		{ 0x03, 0xCD, 0xFF, 0x8D, 0xFE, 0x3F, 0x20, 0x00, 0xA8,
		  0x6E, 0x77, 0xF7, 0xB8, 0x0C, 0x9F, 0xFF, 0xF3, 0x3F,
		  0xF7, 0x7F, 0xDA, 0x86, 0xF7, 0x5C, 0xE6, 0xD8, 0x1F,
		  0xF0, 0xBF, 0xF8, 0x3F, 0xD6, 0xAE, 0xEF, 0xB7, 0x19 },
		{ 0xC3, 0x3D, 0x02, 0xCA, 0xE6, 0x3F, 0xD1, 0x08, 0xB8,
		  0x6F, 0xE0, 0x0D, 0xC6, 0x8D, 0x34, 0x43, 0xC9, 0xBF,
		  0x1C, 0x83, 0xF7, 0xD0, 0x0F, 0xD5, 0x46, 0x8A, 0x61,
		  0x21, 0x60, 0x68, 0xE1, 0x01, 0x91, 0x21, 0x6C, 0x1A },
	};
	uint8_t s = side & 1u;
	bool mouse_on_surface;
	bool mouse_fifo;
	bool short_fifo;
	uint16_t delta_ticks = 12u;
	uint8_t mlen;
	uint8_t *m;
	uint16_t idx12;

	if (!st || !out63)
		return;
	memset(out63, 0, JC2_REPORT0708_SIZE);
	if (!in)
		return;

	out63[0] = seq;
	out63[1] = 0x1Cu;
	pack_buttons_0708(s, in, out63);

	mouse_on_surface = (s == JC2_SIDE_RIGHT) && in->mouse_on_surface;
	out63[8] = 0x38u;
	if (mouse_on_surface) {
		jc2_write_s16_le(&out63[9], in->mouse_dx);
		jc2_write_s16_le(&out63[11], in->mouse_dy);
	}
	out63[13] = mouse_on_surface ? 0x16u : 0xFFu;

	/*
	 * Real Joy-Con 2 firmware sends mlen = 0x00 on the initial seq == 0
	 * notification when 0x000E CCCD is first enabled, seeding its 800 Hz
	 * (1250 us) motion timestamp baseline for subsequent frames.
	 */
	if (seq == 0u && st->motion_last_us[s] == 0u) {
		st->motion_last_us[s] = (has_now_us && now_us != 0u) ? now_us :
								       1u;
		st->motion_rem_us[s] = 0u;
		st->motion_ts[s] = 0u;
		return;
	}

	if (has_now_us) {
		if (st->motion_last_us[s] != 0u) {
			uint32_t dt_us =
				(uint32_t)(now_us - st->motion_last_us[s]) +
				st->motion_rem_us[s];
			uint32_t ticks = dt_us / 1250u;

			st->motion_rem_us[s] = dt_us % 1250u;
			if (ticks < 1u) {
				ticks = 1u;
			} else if (ticks > 36u) {
				ticks = 36u;
				st->motion_rem_us[s] = 0u;
			}
			delta_ticks = (uint16_t)ticks;
		}
		st->motion_last_us[s] = now_us ? now_us : 1u;
	}

	/*
	 * Keep Right mouse-on-surface pinned to the quiet 0x0C (26 B) rail-down
	 * sample even when a TDMA overlap widens delta_ticks past 6; flipping
	 * to a separate 36 B capture creates a cumulative-gyro discontinuity
	 * and rotation spike that kicks Horizon OS out of mouse mode.
	 */
	mouse_fifo = mouse_on_surface;
	short_fifo = (delta_ticks <= 6u) || mouse_fifo;
	mlen = short_fifo ? 0x1Eu : 0x28u;
	if (s == JC2_SIDE_RIGHT) {
		out63[14] = 0x00u;
		out63[15] = mlen;
		m = &out63[16];
	} else {
		out63[14] = mlen;
		m = &out63[15];
	}

	st->motion_ts[s] = (st->motion_ts[s] + delta_ticks) & 0x0FFFu;
	idx12 = (uint16_t)st->motion_ts[s];
	m[0] = (uint8_t)(idx12 & 0xFFu);
	m[1] = (uint8_t)(((idx12 >> 8) & 0x0Fu) | ((delta_ticks & 0x0Fu) << 4));
	m[2] = (uint8_t)((delta_ticks >> 4) & 0xFFu);
	if (short_fifo) {
		m[3] = 0x0Cu;
		memcpy(&m[4], mouse_fifo ? imu_mouse_26 : imu_flat_26, 26);
	} else {
		m[3] = (delta_ticks <= 10u) ?
			       0x0Du :
			       ((delta_ticks <= 14u) ? 0x0Eu : 0x0Fu);
		memcpy(&m[4], imu_stationary[s], 36);
	}
}

void jc2_proto_build_adv(const jc2_proto_state_t *st, uint8_t side,
			 bool reconnect, uint8_t adv_out[31],
			 uint8_t scan_rsp_out[20], uint8_t *scan_rsp_len)
{
	uint8_t s = side & 1u;
	uint8_t pid_lo = (s == JC2_SIDE_RIGHT) ? 0x66u : 0x67u;
	/*
	 * Exact 31-byte Switch 2 SoC filter ADV_IND: Flags (02 01 06) + 26-byte
	 * Nintendo 0x0553 Manufacturer Data. Device name lives in SCAN_RSP.
	 */
	uint8_t adv31[31] = { 0x02, 0x01, 0x06, 0x1B, 0xFF,   0x53, 0x05, 0x01,
			      0x00, 0x03, 0x7E, 0x05, pid_lo, 0x20, 0x00, 0x01,
			      0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x0F,
			      0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00 };
	const char *nm = (s == JC2_SIDE_RIGHT) ? "Joy-Con 2 (R)" :
						 "Joy-Con 2 (L)";
	uint8_t nlen = (uint8_t)strlen(nm);

	if (st && reconnect && st->bond.has_host && st->bond.has_ltk[s])
		memcpy(&adv31[17], st->bond.host_addr, 6);
	if (adv_out)
		memcpy(adv_out, adv31, sizeof(adv31));
	if (scan_rsp_out) {
		scan_rsp_out[0] = (uint8_t)(nlen + 1u);
		scan_rsp_out[1] = 0x09u;
		memcpy(&scan_rsp_out[2], nm, nlen);
	}
	if (scan_rsp_len)
		*scan_rsp_len = (uint8_t)(nlen + 2u);
}
