/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#include "jc2_att.h"

#include <string.h>

#define ATT_OP_ERROR_RSP 0x01u
#define ATT_OP_MTU_REQ 0x02u
#define ATT_OP_MTU_RSP 0x03u
#define ATT_OP_FIND_INFO_REQ 0x04u
#define ATT_OP_FIND_INFO_RSP 0x05u
#define ATT_OP_FIND_BY_TYPE_REQ 0x06u
#define ATT_OP_FIND_BY_TYPE_RSP 0x07u
#define ATT_OP_READ_BY_TYPE_REQ 0x08u
#define ATT_OP_READ_BY_TYPE_RSP 0x09u
#define ATT_OP_READ_REQ 0x0Au
#define ATT_OP_READ_RSP 0x0Bu
#define ATT_OP_READ_BLOB_REQ 0x0Cu
#define ATT_OP_READ_BLOB_RSP 0x0Du
#define ATT_OP_READ_BY_GRP_REQ 0x10u
#define ATT_OP_READ_BY_GRP_RSP 0x11u
#define ATT_OP_WRITE_REQ 0x12u
#define ATT_OP_WRITE_RSP 0x13u
#define ATT_OP_WRITE_CMD 0x52u

#define ATT_ERR_INVALID_HANDLE 0x01u
#define ATT_ERR_READ_NOT_PERM 0x02u
#define ATT_ERR_WRITE_NOT_PERM 0x03u
#define ATT_ERR_INVALID_PDU 0x04u
#define ATT_ERR_REQ_NOT_SUPP 0x06u
#define ATT_ERR_INVALID_OFFSET 0x07u
#define ATT_ERR_ATTR_NOT_FOUND 0x0Au

#define JC2_GATT_ATTR_COUNT 48u

enum jc2_attr_kind {
	ATTR_PRIM_SVC16 = 0u,
	ATTR_PRIM_SVC128,
	ATTR_CHAR_DECL16,
	ATTR_CHAR_DECL128,
	ATTR_VAL16,
	ATTR_VAL128,
	ATTR_CCCD,
	ATTR_DESC128,
};

typedef struct {
	uint16_t handle;
	uint8_t kind;
	uint8_t props;
	uint16_t aux_u16;
	const uint8_t *uuid128_r;
	const uint8_t *uuid128_l;
} jc2_gatt_attr_t;

static const uint8_t UUID_CACHE_SVC[16] = { 0x80, 0xD2, 0x6B, 0xF9, 0x56, 0x19,
					    0x51, 0x8F, 0x30, 0x4E, 0x64, 0x19,
					    0x5D, 0xAF, 0xC5, 0x00 };
static const uint8_t UUID_CACHE_C1[16] = { 0x81, 0xD2, 0x6B, 0xF9, 0x56, 0x19,
					   0x51, 0x8F, 0x30, 0x4E, 0x64, 0x19,
					   0x5D, 0xAF, 0xC5, 0x00 };
static const uint8_t UUID_CACHE_C2[16] = { 0x82, 0xD2, 0x6B, 0xF9, 0x56, 0x19,
					   0x51, 0x8F, 0x30, 0x4E, 0x64, 0x19,
					   0x5D, 0xAF, 0xC5, 0x00 };
static const uint8_t UUID_CACHE_C3[16] = { 0x83, 0xD2, 0x6B, 0xF9, 0x56, 0x19,
					   0x51, 0x8F, 0x30, 0x4E, 0x64, 0x19,
					   0x5D, 0xAF, 0xC5, 0x00 };

static const uint8_t UUID_SVC_MAIN[16] = { 0xD0, 0x7F, 0xDF, 0x09, 0x8F, 0x11,
					   0x8F, 0x82, 0xAD, 0x49, 0xFE, 0x89,
					   0xBE, 0xE9, 0x7D, 0xAB };
static const uint8_t UUID_INPUT_05[16] = { 0xD2, 0x7F, 0xDF, 0x09, 0x8F, 0x11,
					   0x8F, 0x82, 0xAD, 0x49, 0xFE, 0x89,
					   0xBE, 0xE9, 0x7D, 0xAB };
static const uint8_t UUID_INPUT_08_R[16] = { 0x42, 0xF4, 0x2B, 0x14, 0x67, 0x8B,
					     0x0C, 0xB2, 0xCA, 0x4C, 0xFC, 0x2F,
					     0x1E, 0xE0, 0xA9, 0xD5 };
static const uint8_t UUID_INPUT_07_L[16] = { 0x2A, 0xA3, 0x41, 0xB2, 0x1C, 0xA8,
					     0x16, 0xA7, 0x32, 0x4D, 0x54, 0x73,
					     0xB5, 0xBB, 0x1B, 0xCC };
static const uint8_t UUID_RATE_DESC[16] = { 0xCB, 0x6E, 0x48, 0x80, 0xDF, 0x95,
					    0x57, 0x95, 0xEE, 0x4D, 0x24, 0x5A,
					    0x10, 0x55, 0x9D, 0x67 };
static const uint8_t UUID_VIB_R[16] = { 0x49, 0xC1, 0x00, 0x9E, 0xB0, 0xBB,
					0xA1, 0x84, 0xA7, 0x46, 0x1F, 0xCD,
					0xFB, 0xB0, 0x19, 0xFA };
static const uint8_t UUID_VIB_L[16] = { 0x41, 0x82, 0xF1, 0x14, 0x0C, 0x24,
					0xF4, 0xA8, 0x5D, 0x48, 0x71, 0xA4,
					0xCB, 0x26, 0x93, 0x28 };
static const uint8_t UUID_CMD_WRITE[16] = { 0x05, 0xF0, 0xE5, 0x4F, 0xA5, 0x1E,
					    0x44, 0xAF, 0x6C, 0x4E, 0xB7, 0x8E,
					    0xC9, 0x4A, 0x9D, 0x64 };
static const uint8_t UUID_RUMBLE_R[16] = { 0xFF, 0x27, 0x6B, 0x37, 0x42, 0xA3,
					   0x78, 0x80, 0x61, 0x4A, 0xE7, 0xF1,
					   0xB3, 0x24, 0xA7, 0x65 };
static const uint8_t UUID_RUMBLE_L[16] = { 0xEA, 0xDB, 0xAA, 0x88, 0xCF, 0xC8,
					   0x1E, 0x93, 0xAE, 0x48, 0xED, 0xDC,
					   0x30, 0xA8, 0x49, 0xCE };
static const uint8_t UUID_FW_UPD[16] = { 0x8D, 0x9F, 0xF5, 0x5D, 0x3E, 0xD2,
					 0xF7, 0xA4, 0xF7, 0x4D, 0xAE, 0xFD,
					 0x3D, 0x42, 0x47, 0x41 };
static const uint8_t UUID_CMD_RESP[16] = { 0x6A, 0x83, 0x11, 0xB1, 0x15, 0x53,
					   0x0A, 0xA2, 0x36, 0x4D, 0xD8, 0xD9,
					   0x61, 0xA9, 0x65, 0xC7 };
static const uint8_t UUID_UNK_DESC[16] = { 0x79, 0xF9, 0xA4, 0xED, 0xBB, 0xE3,
					   0xD2, 0x9C, 0x5B, 0x49, 0x58, 0xF3,
					   0x8C, 0xDF, 0x46, 0xB7 };
static const uint8_t UUID_EXT_RESP_R[16] = { 0x0B, 0x69, 0x2B, 0xAF, 0x6F, 0x42,
					     0xF3, 0xA7, 0x0C, 0x41, 0x88, 0x0E,
					     0x8E, 0xA5, 0x0C, 0x64 };
static const uint8_t UUID_EXT_RESP_L[16] = { 0x96, 0xB9, 0x3C, 0x40, 0x52, 0x3D,
					     0x10, 0x90, 0x4B, 0x47, 0xC7, 0xAE,
					     0x0F, 0x81, 0xA3, 0x63 };
static const uint8_t UUID_UNK_22[16] = { 0x80, 0x2A, 0x6D, 0x40, 0x6F, 0xF8,
					 0x15, 0xAB, 0x41, 0x42, 0x1C, 0x84,
					 0xD2, 0x69, 0xBD, 0xD3 };
static const uint8_t UUID_UNK_26[16] = { 0xDE, 0x7F, 0xDF, 0x09, 0x8F, 0x11,
					 0x8F, 0x82, 0xAD, 0x49, 0xFE, 0x89,
					 0xBE, 0xE9, 0x7D, 0xAB };
static const uint8_t UUID_UNK_2A[16] = { 0xDF, 0x7F, 0xDF, 0x09, 0x8F, 0x11,
					 0x8F, 0x82, 0xAD, 0x49, 0xFE, 0x89,
					 0xBE, 0xE9, 0x7D, 0xAB };

/*
 * 0x0001..0x0007 is the Switch 2 proprietary GATT cache service, with a
 * per-side C3 hash at 0x0007 so Right (0x2066) and Left (0x2067) never
 * collide in the console's GATT cache.
 */
static const jc2_gatt_attr_t JC2_GATT_TABLE[JC2_GATT_ATTR_COUNT] = {
	{ 0x0001u, ATTR_PRIM_SVC128, 0x02u, 0x0007u, UUID_CACHE_SVC,
	  UUID_CACHE_SVC },
	{ 0x0002u, ATTR_CHAR_DECL128, 0x02u, 0x0003u, UUID_CACHE_C1,
	  UUID_CACHE_C1 },
	{ 0x0003u, ATTR_VAL128, 0x02u, 0u, UUID_CACHE_C1, UUID_CACHE_C1 },
	{ 0x0004u, ATTR_CHAR_DECL128, 0x08u, 0x0005u, UUID_CACHE_C2,
	  UUID_CACHE_C2 },
	{ 0x0005u, ATTR_VAL128, 0x0Cu, 0u, UUID_CACHE_C2, UUID_CACHE_C2 },
	{ 0x0006u, ATTR_CHAR_DECL128, 0x02u, 0x0007u, UUID_CACHE_C3,
	  UUID_CACHE_C3 },
	{ 0x0007u, ATTR_VAL128, 0x02u, 0u, UUID_CACHE_C3, UUID_CACHE_C3 },

	{ 0x0008u, ATTR_PRIM_SVC128, 0x02u, 0x002Au, UUID_SVC_MAIN,
	  UUID_SVC_MAIN },
	{ 0x0009u, ATTR_CHAR_DECL128, 0x12u, 0x000Au, UUID_INPUT_05,
	  UUID_INPUT_05 },
	{ 0x000Au, ATTR_VAL128, 0x12u, 0u, UUID_INPUT_05, UUID_INPUT_05 },
	{ 0x000Bu, ATTR_CCCD, 0x0Au, 0x2902u, NULL, NULL },
	{ 0x000Cu, ATTR_DESC128, 0x0Au, 0u, UUID_RATE_DESC, UUID_RATE_DESC },

	{ 0x000Du, ATTR_CHAR_DECL128, 0x12u, 0x000Eu, UUID_INPUT_08_R,
	  UUID_INPUT_07_L },
	{ 0x000Eu, ATTR_VAL128, 0x12u, 0u, UUID_INPUT_08_R, UUID_INPUT_07_L },
	{ 0x000Fu, ATTR_CCCD, 0x0Au, 0x2902u, NULL, NULL },
	{ 0x0010u, ATTR_DESC128, 0x0Au, 0u, UUID_RATE_DESC, UUID_RATE_DESC },

	{ 0x0011u, ATTR_CHAR_DECL128, 0x0Cu, 0x0012u, UUID_VIB_R, UUID_VIB_L },
	{ 0x0012u, ATTR_VAL128, 0x0Cu, 0u, UUID_VIB_R, UUID_VIB_L },

	{ 0x0013u, ATTR_CHAR_DECL128, 0x0Cu, 0x0014u, UUID_CMD_WRITE,
	  UUID_CMD_WRITE },
	{ 0x0014u, ATTR_VAL128, 0x0Cu, 0u, UUID_CMD_WRITE, UUID_CMD_WRITE },

	{ 0x0015u, ATTR_CHAR_DECL128, 0x0Cu, 0x0016u, UUID_RUMBLE_R,
	  UUID_RUMBLE_L },
	{ 0x0016u, ATTR_VAL128, 0x0Cu, 0u, UUID_RUMBLE_R, UUID_RUMBLE_L },

	{ 0x0017u, ATTR_CHAR_DECL128, 0x0Cu, 0x0018u, UUID_FW_UPD,
	  UUID_FW_UPD },
	{ 0x0018u, ATTR_VAL128, 0x0Cu, 0u, UUID_FW_UPD, UUID_FW_UPD },

	{ 0x0019u, ATTR_CHAR_DECL128, 0x10u, 0x001Au, UUID_CMD_RESP,
	  UUID_CMD_RESP },
	{ 0x001Au, ATTR_VAL128, 0x10u, 0u, UUID_CMD_RESP, UUID_CMD_RESP },
	{ 0x001Bu, ATTR_CCCD, 0x0Au, 0x2902u, NULL, NULL },
	{ 0x001Cu, ATTR_DESC128, 0x0Au, 0u, UUID_UNK_DESC, UUID_UNK_DESC },

	{ 0x001Du, ATTR_CHAR_DECL128, 0x10u, 0x001Eu, UUID_EXT_RESP_R,
	  UUID_EXT_RESP_L },
	{ 0x001Eu, ATTR_VAL128, 0x10u, 0u, UUID_EXT_RESP_R, UUID_EXT_RESP_L },
	{ 0x001Fu, ATTR_CCCD, 0x0Au, 0x2902u, NULL, NULL },
	{ 0x0020u, ATTR_DESC128, 0x0Au, 0u, UUID_UNK_DESC, UUID_UNK_DESC },

	{ 0x0021u, ATTR_CHAR_DECL128, 0x10u, 0x0022u, UUID_UNK_22,
	  UUID_UNK_22 },
	{ 0x0022u, ATTR_VAL128, 0x10u, 0u, UUID_UNK_22, UUID_UNK_22 },
	{ 0x0023u, ATTR_CCCD, 0x0Au, 0x2902u, NULL, NULL },
	{ 0x0024u, ATTR_DESC128, 0x0Au, 0u, UUID_UNK_DESC, UUID_UNK_DESC },

	{ 0x0025u, ATTR_CHAR_DECL128, 0x12u, 0x0026u, UUID_UNK_26,
	  UUID_UNK_26 },
	{ 0x0026u, ATTR_VAL128, 0x12u, 0u, UUID_UNK_26, UUID_UNK_26 },
	{ 0x0027u, ATTR_CCCD, 0x0Au, 0x2902u, NULL, NULL },
	{ 0x0028u, ATTR_DESC128, 0x0Au, 0u, UUID_RATE_DESC, UUID_RATE_DESC },

	{ 0x0029u, ATTR_CHAR_DECL128, 0x0Cu, 0x002Au, UUID_UNK_2A,
	  UUID_UNK_2A },
	{ 0x002Au, ATTR_VAL128, 0x0Cu, 0u, UUID_UNK_2A, UUID_UNK_2A },

	{ 0x002Bu, ATTR_PRIM_SVC16, 0x02u, 0x1800u, NULL, NULL },
	{ 0x002Cu, ATTR_CHAR_DECL16, 0x02u, 0x2A00u, NULL, NULL },
	{ 0x002Du, ATTR_VAL16, 0x02u, 0x2A00u, NULL, NULL },
	{ 0x002Eu, ATTR_CHAR_DECL16, 0x02u, 0x2A01u, NULL, NULL },
	{ 0x002Fu, ATTR_VAL16, 0x02u, 0x2A01u, NULL, NULL },
	{ 0x0030u, ATTR_PRIM_SVC16, 0x02u, 0x1801u, NULL, NULL },
};

static const uint8_t *attr_uuid128(const jc2_gatt_attr_t *a, uint8_t side)
{
	return (side == JC2_SIDE_RIGHT) ? a->uuid128_r : a->uuid128_l;
}

static uint8_t attr_type_uuid(const jc2_gatt_attr_t *a, uint8_t side,
			      uint8_t out_uuid[16])
{
	switch (a->kind) {
	case ATTR_PRIM_SVC16:
	case ATTR_PRIM_SVC128:
		jc2_write_u16_le(out_uuid, 0x2800u);
		return 2u;
	case ATTR_CHAR_DECL16:
	case ATTR_CHAR_DECL128:
		jc2_write_u16_le(out_uuid, 0x2803u);
		return 2u;
	case ATTR_CCCD:
		jc2_write_u16_le(out_uuid, 0x2902u);
		return 2u;
	case ATTR_VAL16:
		jc2_write_u16_le(out_uuid, a->aux_u16);
		return 2u;
	case ATTR_VAL128:
	case ATTR_DESC128:
		memcpy(out_uuid, attr_uuid128(a, side), 16);
		return 16u;
	default:
		return 0u;
	}
}

static uint16_t svc_end_handle(uint16_t svc_handle)
{
	if (svc_handle == 0x0001u)
		return 0x0007u;
	if (svc_handle == 0x0008u)
		return 0x002Au;
	if (svc_handle == 0x002Bu)
		return 0x002Fu;
	return 0x0030u;
}

static uint8_t read_attr_value(const jc2_ll_conn_t *c, const jc2_gatt_attr_t *a,
			       const jc2_ll_callbacks_t *cbs, uint8_t *out,
			       uint16_t *out_len)
{
	uint8_t side = c->side & 1u;

	switch (a->kind) {
	case ATTR_PRIM_SVC16:
		jc2_write_u16_le(out, a->aux_u16);
		*out_len = 2u;
		return 0u;
	case ATTR_PRIM_SVC128:
		memcpy(out, attr_uuid128(a, side), 16);
		*out_len = 16u;
		return 0u;
	case ATTR_CHAR_DECL16:
		out[0] = a->props;
		jc2_write_u16_le(&out[1], (uint16_t)(a->handle + 1u));
		jc2_write_u16_le(&out[3], a->aux_u16);
		*out_len = 5u;
		return 0u;
	case ATTR_CHAR_DECL128:
		out[0] = a->props;
		jc2_write_u16_le(&out[1], a->aux_u16);
		memcpy(&out[3], attr_uuid128(a, side), 16);
		*out_len = 19u;
		return 0u;
	case ATTR_CCCD: {
		uint16_t v = 0u;

		if (a->handle == JC2_HDL_INPUT05_CCCD)
			v = c->cccd_000a;
		else if (a->handle == JC2_HDL_INPUT0708_CCCD)
			v = c->cccd_000e;
		else if (a->handle == JC2_HDL_CMD_RESP_CCCD)
			v = c->cccd_001a;
		else if (a->handle == JC2_HDL_EXT_RESP_CCCD)
			v = c->cccd_001e;
		else if (a->handle == JC2_HDL_UNK22_CCCD)
			v = c->cccd_0022;
		else if (a->handle == JC2_HDL_UNK26_CCCD)
			v = c->cccd_0026;
		jc2_write_u16_le(out, v);
		*out_len = 2u;
		return 0u;
	}
	case ATTR_DESC128: {
		uint16_t v = 0u;

		if (a->handle == 0x000Cu)
			v = c->rate_000c;
		else if (a->handle == JC2_HDL_INPUT0708_RATE)
			v = c->rate_0010;
		else if (a->handle == 0x0028u)
			v = c->rate_0028;
		jc2_write_u16_le(out, v);
		*out_len = 2u;
		return 0u;
	}
	case ATTR_VAL16:
		if (a->aux_u16 == 0x2A00u) {
			const char *nm = (side == JC2_SIDE_RIGHT) ?
						 "Joy-Con 2 (R)" :
						 "Joy-Con 2 (L)";
			uint16_t nlen = (uint16_t)strlen(nm);

			memcpy(out, nm, nlen);
			*out_len = nlen;
			return 0u;
		}
		out[0] = 0u;
		out[1] = 0u;
		*out_len = 2u;
		return 0u;
	case ATTR_VAL128:
		if ((a->props & 0x02u) == 0u)
			return ATT_ERR_READ_NOT_PERM;
		if (a->handle == 0x0003u) {
			static const uint8_t c1_val[7] = { 0x04, 0x00, 0x05,
							   0x00, 0x01, 0x01,
							   0x00 };

			memcpy(out, c1_val, sizeof(c1_val));
			*out_len = sizeof(c1_val);
			return 0u;
		}
		if (a->handle == 0x0007u) {
			uint8_t c3_hash[8] = { 0x29, 0x31, 0x95, 0x87,
					       0x06, 0x4C, 0x0A, 0x9F };

			c3_hash[7] ^= side;
			memcpy(out, c3_hash, sizeof(c3_hash));
			*out_len = sizeof(c3_hash);
			return 0u;
		}
		if (a->handle == JC2_HDL_INPUT05_VAL ||
		    a->handle == JC2_HDL_INPUT0708_VAL) {
			uint16_t cap = (a->handle == JC2_HDL_INPUT05_VAL) ?
					       JC2_REPORT05_SIZE :
					       JC2_REPORT0708_SIZE;

			memset(out, 0, cap);
			if (cbs && cbs->read_input_value)
				cbs->read_input_value(side, a->handle, out,
						      &cap, cbs->user_ctx);
			*out_len = cap;
			return 0u;
		}
		memset(out, 0, 16);
		*out_len = 16u;
		return 0u;
	default:
		return ATT_ERR_INVALID_HANDLE;
	}
}

static void send_att_error(jc2_ll_conn_t *c, uint8_t req_op, uint16_t hdl,
			   uint8_t err)
{
	uint8_t rsp[5];

	rsp[0] = ATT_OP_ERROR_RSP;
	rsp[1] = req_op;
	jc2_write_u16_le(&rsp[2], hdl);
	rsp[4] = err;
	(void)jc2_ll_enqueue_l2cap_att(c, rsp, sizeof(rsp), false);
}

static void att_handle_mtu_req(jc2_ll_conn_t *c, const uint8_t *att,
			       uint16_t len, jc2_link_diag_t *diag)
{
	uint16_t cmtu;
	uint8_t rsp[3];

	if (len < 3u) {
		send_att_error(c, ATT_OP_MTU_REQ, 0u, ATT_ERR_INVALID_PDU);
		return;
	}
	cmtu = jc2_read_u16_le(&att[1]);
	if (cmtu < 23u)
		cmtu = 23u;
	c->mtu = (cmtu > 512u) ? 512u : cmtu;
	if (diag)
		diag->mtu = c->mtu;
	jc2_ll_log_evt(JC2_EVT_MTU_REQ, c->side, cmtu);
	rsp[0] = ATT_OP_MTU_RSP;
	jc2_write_u16_le(&rsp[1], 512u);
	(void)jc2_ll_enqueue_l2cap_att(c, rsp, sizeof(rsp), false);
}

static void att_handle_read_by_grp(jc2_ll_conn_t *c, const uint8_t *att,
				   uint16_t len, uint16_t max_rsp)
{
	uint16_t start_h;
	uint16_t end_h;
	uint8_t req_ulen;
	uint8_t rsp[247];
	uint16_t rpos = 2u;
	uint8_t elem_len = 0u;
	size_t i;

	if (len != 7u && len != 21u) {
		send_att_error(c, ATT_OP_READ_BY_GRP_REQ, 0u,
			       ATT_ERR_INVALID_PDU);
		return;
	}
	start_h = jc2_read_u16_le(&att[1]);
	end_h = jc2_read_u16_le(&att[3]);
	req_ulen = (uint8_t)(len - 5u);
	if (start_h == 0u || start_h > end_h) {
		send_att_error(c, ATT_OP_READ_BY_GRP_REQ, start_h,
			       ATT_ERR_INVALID_HANDLE);
		return;
	}
	if (req_ulen != 2u || jc2_read_u16_le(&att[5]) != 0x2800u) {
		send_att_error(c, ATT_OP_READ_BY_GRP_REQ, start_h,
			       ATT_ERR_ATTR_NOT_FOUND);
		return;
	}
	for (i = 0u; i < JC2_GATT_ATTR_COUNT; i++) {
		const jc2_gatt_attr_t *a = &JC2_GATT_TABLE[i];
		uint8_t vlen;
		uint8_t elen;

		if (a->handle < start_h || a->handle > end_h)
			continue;
		if (a->kind != ATTR_PRIM_SVC16 && a->kind != ATTR_PRIM_SVC128)
			continue;
		vlen = (a->kind == ATTR_PRIM_SVC16) ? 2u : 16u;
		elen = (uint8_t)(4u + vlen);
		if (elem_len == 0u)
			elem_len = elen;
		else if (elen != elem_len)
			break;
		if (rpos + elen > max_rsp)
			break;
		jc2_write_u16_le(&rsp[rpos], a->handle);
		jc2_write_u16_le(&rsp[rpos + 2u], svc_end_handle(a->handle));
		if (vlen == 2u)
			jc2_write_u16_le(&rsp[rpos + 4u], a->aux_u16);
		else
			memcpy(&rsp[rpos + 4u], attr_uuid128(a, c->side), 16);
		rpos = (uint16_t)(rpos + elen);
	}
	if (elem_len == 0u) {
		send_att_error(c, ATT_OP_READ_BY_GRP_REQ, start_h,
			       ATT_ERR_ATTR_NOT_FOUND);
		return;
	}
	rsp[0] = ATT_OP_READ_BY_GRP_RSP;
	rsp[1] = elem_len;
	(void)jc2_ll_enqueue_l2cap_att(c, rsp, rpos, false);
}

static void att_handle_find_by_type(jc2_ll_conn_t *c, const uint8_t *att,
				    uint16_t len, uint16_t max_rsp,
				    const jc2_ll_callbacks_t *cbs)
{
	uint16_t start_h;
	uint16_t end_h;
	uint16_t u16;
	const uint8_t *target_val;
	uint16_t target_len;
	uint8_t rsp[247];
	uint16_t rpos = 1u;
	size_t i;

	if (len < 7u) {
		send_att_error(c, ATT_OP_FIND_BY_TYPE_REQ, 0u,
			       ATT_ERR_INVALID_PDU);
		return;
	}
	start_h = jc2_read_u16_le(&att[1]);
	end_h = jc2_read_u16_le(&att[3]);
	u16 = jc2_read_u16_le(&att[5]);
	target_val = &att[7];
	target_len = (uint16_t)(len - 7u);
	if (start_h == 0u || start_h > end_h) {
		send_att_error(c, ATT_OP_FIND_BY_TYPE_REQ, start_h,
			       ATT_ERR_INVALID_HANDLE);
		return;
	}
	rsp[0] = ATT_OP_FIND_BY_TYPE_RSP;
	for (i = 0u; i < JC2_GATT_ATTR_COUNT; i++) {
		const jc2_gatt_attr_t *a = &JC2_GATT_TABLE[i];
		uint8_t auuid[16];
		uint8_t aulen;
		uint8_t aval[68];
		uint16_t avlen = 0u;
		uint16_t grp_end;

		if (a->handle < start_h || a->handle > end_h)
			continue;
		aulen = attr_type_uuid(a, c->side, auuid);
		if (aulen != 2u || jc2_read_u16_le(auuid) != u16)
			continue;
		if (read_attr_value(c, a, cbs, aval, &avlen) != 0u)
			continue;
		if (avlen != target_len ||
		    memcmp(aval, target_val, target_len) != 0)
			continue;
		if (rpos + 4u > max_rsp)
			break;
		jc2_write_u16_le(&rsp[rpos], a->handle);
		grp_end = (u16 == 0x2800u) ? svc_end_handle(a->handle) :
					     a->handle;
		jc2_write_u16_le(&rsp[rpos + 2u], grp_end);
		rpos = (uint16_t)(rpos + 4u);
	}
	if (rpos == 1u) {
		send_att_error(c, ATT_OP_FIND_BY_TYPE_REQ, start_h,
			       ATT_ERR_ATTR_NOT_FOUND);
		return;
	}
	(void)jc2_ll_enqueue_l2cap_att(c, rsp, rpos, false);
}

static void att_handle_read_by_type(jc2_ll_conn_t *c, const uint8_t *att,
				    uint16_t len, uint16_t max_rsp,
				    const jc2_ll_callbacks_t *cbs)
{
	uint16_t start_h;
	uint16_t end_h;
	const uint8_t *req_uuid;
	uint8_t req_ulen;
	uint8_t rsp[247];
	uint16_t rpos = 2u;
	uint8_t elem_len = 0u;
	uint8_t first_err = 0u;
	uint16_t err_hdl;
	size_t i;

	if (len != 7u && len != 21u) {
		send_att_error(c, ATT_OP_READ_BY_TYPE_REQ, 0u,
			       ATT_ERR_INVALID_PDU);
		return;
	}
	start_h = jc2_read_u16_le(&att[1]);
	end_h = jc2_read_u16_le(&att[3]);
	req_uuid = &att[5];
	req_ulen = (uint8_t)(len - 5u);
	err_hdl = start_h;
	if (start_h == 0u || start_h > end_h) {
		send_att_error(c, ATT_OP_READ_BY_TYPE_REQ, start_h,
			       ATT_ERR_INVALID_HANDLE);
		return;
	}
	for (i = 0u; i < JC2_GATT_ATTR_COUNT; i++) {
		const jc2_gatt_attr_t *a = &JC2_GATT_TABLE[i];
		uint8_t auuid[16];
		uint8_t aulen;
		uint8_t aval[68];
		uint16_t avlen = 0u;
		uint16_t max_val;
		uint8_t err;
		uint8_t elen;

		if (a->handle < start_h || a->handle > end_h)
			continue;
		aulen = attr_type_uuid(a, c->side, auuid);
		if (aulen != req_ulen || memcmp(auuid, req_uuid, req_ulen) != 0)
			continue;
		err = read_attr_value(c, a, cbs, aval, &avlen);
		if (err != 0u) {
			if (elem_len == 0u && first_err == 0u) {
				first_err = err;
				err_hdl = a->handle;
			}
			break;
		}
		max_val = (max_rsp > 4u) ? (uint16_t)(max_rsp - 4u) : 1u;
		if (avlen > max_val)
			avlen = max_val;
		if (avlen > 253u)
			avlen = 253u;
		elen = (uint8_t)(2u + avlen);
		if (elem_len == 0u)
			elem_len = elen;
		else if (elen != elem_len)
			break;
		if (rpos + elen > max_rsp)
			break;
		jc2_write_u16_le(&rsp[rpos], a->handle);
		memcpy(&rsp[rpos + 2u], aval, avlen);
		rpos = (uint16_t)(rpos + elen);
	}
	if (elem_len == 0u) {
		send_att_error(c, ATT_OP_READ_BY_TYPE_REQ, err_hdl,
			       first_err ? first_err : ATT_ERR_ATTR_NOT_FOUND);
		return;
	}
	rsp[0] = ATT_OP_READ_BY_TYPE_RSP;
	rsp[1] = elem_len;
	(void)jc2_ll_enqueue_l2cap_att(c, rsp, rpos, false);
}

static void att_handle_find_info(jc2_ll_conn_t *c, const uint8_t *att,
				 uint16_t len, uint16_t max_rsp)
{
	uint16_t start_h;
	uint16_t end_h;
	uint8_t rsp[247];
	uint16_t rpos = 2u;
	uint8_t fmt = 0u;
	size_t i;

	if (len < 5u) {
		send_att_error(c, ATT_OP_FIND_INFO_REQ, 0u,
			       ATT_ERR_INVALID_PDU);
		return;
	}
	start_h = jc2_read_u16_le(&att[1]);
	end_h = jc2_read_u16_le(&att[3]);
	if (start_h == 0u || start_h > end_h) {
		send_att_error(c, ATT_OP_FIND_INFO_REQ, start_h,
			       ATT_ERR_INVALID_HANDLE);
		return;
	}
	for (i = 0u; i < JC2_GATT_ATTR_COUNT; i++) {
		const jc2_gatt_attr_t *a = &JC2_GATT_TABLE[i];
		uint8_t auuid[16];
		uint8_t aulen;
		uint8_t afmt;
		uint8_t elen;

		if (a->handle < start_h || a->handle > end_h)
			continue;
		aulen = attr_type_uuid(a, c->side, auuid);
		afmt = (aulen == 2u) ? 0x01u : 0x02u;
		elen = (uint8_t)(2u + aulen);
		if (fmt == 0u)
			fmt = afmt;
		else if (afmt != fmt)
			break;
		if (rpos + elen > max_rsp)
			break;
		jc2_write_u16_le(&rsp[rpos], a->handle);
		memcpy(&rsp[rpos + 2u], auuid, aulen);
		rpos = (uint16_t)(rpos + elen);
	}
	if (fmt == 0u) {
		send_att_error(c, ATT_OP_FIND_INFO_REQ, start_h,
			       ATT_ERR_ATTR_NOT_FOUND);
		return;
	}
	rsp[0] = ATT_OP_FIND_INFO_RSP;
	rsp[1] = fmt;
	(void)jc2_ll_enqueue_l2cap_att(c, rsp, rpos, false);
}

static void att_handle_read(jc2_ll_conn_t *c, const uint8_t *att, uint16_t len,
			    uint16_t max_rsp, const jc2_ll_callbacks_t *cbs)
{
	uint8_t op = att[0];
	bool is_blob = (op == ATT_OP_READ_BLOB_REQ);
	uint16_t hdl;
	uint16_t off;
	const jc2_gatt_attr_t *a;
	uint8_t aval[68];
	uint16_t avlen = 0u;
	uint8_t err;
	uint16_t copy_len;
	uint8_t rsp[247];

	if (len < (is_blob ? 5u : 3u)) {
		send_att_error(c, op, 0u, ATT_ERR_INVALID_PDU);
		return;
	}
	hdl = jc2_read_u16_le(&att[1]);
	off = is_blob ? jc2_read_u16_le(&att[3]) : 0u;
	if (hdl == 0u || hdl > JC2_GATT_ATTR_COUNT) {
		send_att_error(c, op, hdl, ATT_ERR_INVALID_HANDLE);
		return;
	}
	a = &JC2_GATT_TABLE[hdl - 1u];
	err = read_attr_value(c, a, cbs, aval, &avlen);
	if (err != 0u) {
		send_att_error(c, op, hdl, err);
		return;
	}
	if (off > avlen) {
		send_att_error(c, op, hdl, ATT_ERR_INVALID_OFFSET);
		return;
	}
	copy_len = (uint16_t)(avlen - off);
	if (copy_len > (uint16_t)(max_rsp - 1u))
		copy_len = (uint16_t)(max_rsp - 1u);
	rsp[0] = is_blob ? ATT_OP_READ_BLOB_RSP : ATT_OP_READ_RSP;
	if (copy_len > 0u)
		memcpy(&rsp[1], &aval[off], copy_len);
	(void)jc2_ll_enqueue_l2cap_att(c, rsp, (uint16_t)(1u + copy_len),
				       false);
}

static void att_handle_write(jc2_ll_conn_t *c, const uint8_t *att, uint16_t len,
			     jc2_link_diag_t *diag,
			     const jc2_ll_callbacks_t *cbs)
{
	uint8_t op = att[0];
	bool need_rsp = (op == ATT_OP_WRITE_REQ);
	uint16_t hdl;
	const uint8_t *wdata;
	uint16_t wlen;
	const jc2_gatt_attr_t *a;
	bool skip_trace;
	uint16_t v16;

	if (len < 3u) {
		if (need_rsp)
			send_att_error(c, op, 0u, ATT_ERR_INVALID_PDU);
		return;
	}
	hdl = jc2_read_u16_le(&att[1]);
	wdata = &att[3];
	wlen = (uint16_t)(len - 3u);
	if (hdl == 0u || hdl > JC2_GATT_ATTR_COUNT) {
		if (need_rsp)
			send_att_error(c, op, hdl, ATT_ERR_INVALID_HANDLE);
		return;
	}
	a = &JC2_GATT_TABLE[hdl - 1u];
	if ((a->props & 0x0Cu) == 0u) {
		if (need_rsp)
			send_att_error(c, op, hdl, ATT_ERR_WRITE_NOT_PERM);
		return;
	}
	skip_trace = (hdl == JC2_HDL_VIB_VAL || hdl == JC2_HDL_CMD_WRITE_VAL ||
		      hdl == JC2_HDL_RUMBLE_CMD_VAL);
	if (!skip_trace)
		jc2_ll_log_evt(JC2_EVT_WRITE, c->side, hdl);

	v16 = (wlen >= 2u) ? jc2_read_u16_le(wdata) :
			     (wlen == 1u ? wdata[0] : 0u);
	if (a->kind == ATTR_CCCD) {
		if (hdl == JC2_HDL_INPUT05_CCCD)
			c->cccd_000a = v16;
		else if (hdl == JC2_HDL_INPUT0708_CCCD)
			c->cccd_000e = v16;
		else if (hdl == JC2_HDL_CMD_RESP_CCCD)
			c->cccd_001a = v16;
		else if (hdl == JC2_HDL_EXT_RESP_CCCD)
			c->cccd_001e = v16;
		else if (hdl == JC2_HDL_UNK22_CCCD)
			c->cccd_0022 = v16;
		else if (hdl == JC2_HDL_UNK26_CCCD)
			c->cccd_0026 = v16;
	} else if (a->kind == ATTR_DESC128) {
		if (hdl == 0x000Cu)
			c->rate_000c = v16;
		else if (hdl == JC2_HDL_INPUT0708_RATE) {
			c->rate_0010 = v16;
			if (diag)
				diag->rate_desc = v16;
		} else if (hdl == 0x0028u) {
			c->rate_0028 = v16;
		}
	}

	/*
	 * Queue WRITE_RSP before invoking callbacks so the mandatory ATT
	 * acknowledgment precedes any newly triggered notification.
	 */
	if (need_rsp) {
		uint8_t rsp[1] = { ATT_OP_WRITE_RSP };

		(void)jc2_ll_enqueue_l2cap_att(c, rsp, 1u, false);
	}

	if (a->kind == ATTR_CCCD &&
	    (hdl == JC2_HDL_INPUT0708_CCCD || hdl == JC2_HDL_INPUT05_CCCD)) {
		if (cbs && cbs->on_input_cccd)
			cbs->on_input_cccd(c->side, hdl, (v16 & 0x01u) != 0u,
					   cbs->user_ctx);
	} else if (a->kind == ATTR_VAL128) {
		if (diag && (hdl == JC2_HDL_VIB_VAL ||
			     (hdl == JC2_HDL_RUMBLE_CMD_VAL && wlen < 25u))) {
			diag->rx_vib++;
		}
		if (cbs && cbs->on_att_write)
			cbs->on_att_write(c->side, hdl, wdata, wlen,
					  cbs->user_ctx);
	}
}

void jc2_att_handle_pdu(jc2_ll_conn_t *c, const uint8_t *att, uint16_t len,
			jc2_link_diag_t *diag, const jc2_ll_callbacks_t *cbs)
{
	uint8_t op;
	uint16_t max_rsp;

	if (!c || !att || len == 0u)
		return;
	op = att[0];
	max_rsp = c->mtu;
	if (max_rsp > 247u)
		max_rsp = 247u;
	if (max_rsp < 23u)
		max_rsp = 23u;

	switch (op) {
	case ATT_OP_MTU_REQ:
		att_handle_mtu_req(c, att, len, diag);
		return;
	case ATT_OP_READ_BY_GRP_REQ:
		att_handle_read_by_grp(c, att, len, max_rsp);
		return;
	case ATT_OP_FIND_BY_TYPE_REQ:
		att_handle_find_by_type(c, att, len, max_rsp, cbs);
		return;
	case ATT_OP_READ_BY_TYPE_REQ:
		att_handle_read_by_type(c, att, len, max_rsp, cbs);
		return;
	case ATT_OP_FIND_INFO_REQ:
		att_handle_find_info(c, att, len, max_rsp);
		return;
	case ATT_OP_READ_REQ:
	case ATT_OP_READ_BLOB_REQ:
		att_handle_read(c, att, len, max_rsp, cbs);
		return;
	case ATT_OP_WRITE_REQ:
	case ATT_OP_WRITE_CMD:
		att_handle_write(c, att, len, diag, cbs);
		return;
	default:
		if ((op & 0x40u) == 0u)
			send_att_error(c, op, 0u, ATT_ERR_REQ_NOT_SUPP);
		return;
	}
}
