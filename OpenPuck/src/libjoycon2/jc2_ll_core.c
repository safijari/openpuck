/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#include "jc2_ll_core.h"
#include "jc2_att.h"
#include "jc2_crypto.h"
#include "jc2_port.h"

#include <string.h>

#define ATT_OP_HANDLE_VAL_NTF 0x1Bu

#define LL_CTRL_CONNECTION_UPDATE_IND 0x00u
#define LL_CTRL_CHANNEL_MAP_IND 0x01u
#define LL_CTRL_TERMINATE_IND 0x02u
#define LL_CTRL_ENC_REQ 0x03u
#define LL_CTRL_ENC_RSP 0x04u
#define LL_CTRL_START_ENC_REQ 0x05u
#define LL_CTRL_START_ENC_RSP 0x06u
#define LL_CTRL_UNKNOWN_RSP 0x07u
#define LL_CTRL_FEATURE_REQ 0x08u
#define LL_CTRL_FEATURE_RSP 0x09u
#define LL_CTRL_VERSION_IND 0x0Cu
#define LL_CTRL_REJECT_IND 0x0Du
#define LL_CTRL_SLAVE_FEATURE_REQ 0x0Eu
#define LL_CTRL_PING_REQ 0x12u
#define LL_CTRL_PING_RSP 0x13u
#define LL_CTRL_LENGTH_REQ 0x14u
#define LL_CTRL_LENGTH_RSP 0x15u
#define LL_CTRL_PHY_REQ 0x16u
#define LL_CTRL_PHY_RSP 0x17u
#define LL_CTRL_PHY_UPDATE_IND 0x18u

static jc2_evt_log_t s_evt_log[JC2_EVT_LOG_DEPTH];
static volatile uint8_t s_evt_count = 0u;
static uint8_t s_conn_gen_seq = 0u;

void jc2_ll_log_evt(uint8_t evt_id, uint8_t side, uint16_t hdl)
{
	uint32_t pm = jc2_port_irq_lock();
	uint8_t idx = s_evt_count & (JC2_EVT_LOG_DEPTH - 1u);

	s_evt_log[idx].evt_id = evt_id;
	s_evt_log[idx].conn = side;
	s_evt_log[idx].hdl = hdl;
	s_evt_count++;
	jc2_port_irq_unlock(pm);
}

uint8_t jc2_ll_get_evt_count(void)
{
	return s_evt_count;
}

const jc2_evt_log_t *jc2_ll_get_evt_log(uint8_t *out_max_len)
{
	if (out_max_len)
		*out_max_len = JC2_EVT_LOG_DEPTH;
	return s_evt_log;
}

static void rebuild_remap_table(jc2_ll_conn_t *c)
{
	uint8_t n = 0u;
	uint8_t ch;

	for (ch = 0u; ch < 37u; ch++) {
		if (c->chm[ch >> 3] & (uint8_t)(1u << (ch & 7u)))
			c->remap_table[n++] = ch;
	}
	if (n == 0u) {
		c->remap_table[0] = 0u;
		n = 1u;
	}
	c->num_used_ch = n;
}

static uint16_t csa2_permute(uint16_t v)
{
	v = (uint16_t)(((v & 0xAAAAu) >> 1) | ((v & 0x5555u) << 1));
	v = (uint16_t)(((v & 0xCCCCu) >> 2) | ((v & 0x3333u) << 2));
	v = (uint16_t)(((v & 0xF0F0u) >> 4) | ((v & 0x0F0Fu) << 4));
	return v;
}

static uint16_t csa2_mam(uint16_t a, uint16_t b)
{
	return (uint16_t)((17u * (uint32_t)a + b) & 0xFFFFu);
}

uint8_t jc2_ll_next_data_channel(jc2_ll_conn_t *c)
{
	uint8_t unmapped;

	if (c->csa2) {
		uint16_t x = (uint16_t)(c->event_counter ^ c->chan_id);
		uint16_t prn_e;
		uint8_t idx;

		x = csa2_mam(csa2_permute(x), c->chan_id);
		x = csa2_mam(csa2_permute(x), c->chan_id);
		x = csa2_mam(csa2_permute(x), c->chan_id);
		prn_e = (uint16_t)(x ^ c->chan_id);
		unmapped = (uint8_t)(prn_e % 37u);
		if (c->chm[unmapped >> 3] & (uint8_t)(1u << (unmapped & 7u))) {
			c->current_ch = unmapped;
			return unmapped;
		}
		idx = (uint8_t)(((uint32_t)c->num_used_ch * (uint32_t)prn_e) >>
				16);
		c->current_ch = c->remap_table[idx];
		return c->current_ch;
	}

	unmapped = (uint8_t)((c->last_unmapped_ch + c->hop_inc) % 37u);
	c->last_unmapped_ch = unmapped;
	if (c->chm[unmapped >> 3] & (uint8_t)(1u << (unmapped & 7u))) {
		c->current_ch = unmapped;
		return unmapped;
	}
	c->current_ch = c->remap_table[unmapped % c->num_used_ch];
	return c->current_ch;
}

bool jc2_ll_enqueue_ctrl_pdu(jc2_ll_conn_t *c, const uint8_t *data, uint8_t len,
			     bool is_term)
{
	uint32_t prev_bp;
	uint8_t used;
	uint8_t slot;

	if (!c || len > sizeof(c->ll_q[0].data))
		return false;
	prev_bp = jc2_port_bh_lock();
	used = (uint8_t)(c->ll_q_tail - c->ll_q_head);
	if (used >= JC2_LL_Q_DEPTH) {
		jc2_port_bh_unlock(prev_bp);
		return false;
	}
	slot = c->ll_q_tail & (JC2_LL_Q_DEPTH - 1u);
	c->ll_q[slot].len = len;
	c->ll_q[slot].is_term = is_term ? 1u : 0u;
	if (len > 0u && data)
		memcpy(c->ll_q[slot].data, data, len);
	jc2_port_dmb();
	c->ll_q_tail++;
	jc2_port_bh_unlock(prev_bp);
	return true;
}

static bool enqueue_tx_pdu(jc2_ll_conn_t *c, uint8_t llid, const uint8_t *data,
			   uint8_t len, bool is_input_hvn)
{
	uint32_t prev_bp;
	uint8_t used;
	uint8_t slot;

	if (llid == 0x03u)
		return jc2_ll_enqueue_ctrl_pdu(c, data, len, false);
	prev_bp = jc2_port_bh_lock();
	used = (uint8_t)(c->tx_q_tail - c->tx_q_head);
	if (used >= JC2_TX_Q_DEPTH) {
		jc2_port_bh_unlock(prev_bp);
		return false;
	}
	slot = c->tx_q_tail & (JC2_TX_Q_DEPTH - 1u);
	c->tx_q[slot].llid = llid;
	c->tx_q[slot].len = len;
	c->tx_q[slot].is_input_hvn = is_input_hvn ? 1u : 0u;
	if (len > 0u && data)
		memcpy(c->tx_q[slot].data, data, len);
	jc2_port_dmb();
	c->tx_q_tail++;
	jc2_port_bh_unlock(prev_bp);
	return true;
}

bool jc2_ll_enqueue_l2cap_att(jc2_ll_conn_t *c, const uint8_t *att,
			      uint16_t len, bool is_input_hvn)
{
	uint8_t pdu[251];

	if (!c || !att || len + 4u > 251u)
		return false;
	jc2_write_u16_le(&pdu[0], len);
	jc2_write_u16_le(&pdu[2], 0x0004u);
	memcpy(&pdu[4], att, len);
	return enqueue_tx_pdu(c, 0x02u, pdu, (uint8_t)(len + 4u), is_input_hvn);
}

bool jc2_ll_enqueue_att_ntf(jc2_ll_conn_t *c, uint16_t handle,
			    const uint8_t *val, uint16_t len, bool is_input_hvn)
{
	uint8_t att[247];

	if (!c || !c->active || len + 7u > 251u)
		return false;
	att[0] = ATT_OP_HANDLE_VAL_NTF;
	jc2_write_u16_le(&att[1], handle);
	if (len > 0u && val)
		memcpy(&att[3], val, len);
	return jc2_ll_enqueue_l2cap_att(c, att, (uint16_t)(len + 3u),
					is_input_hvn);
}

bool jc2_ll_accept_connect_ind(jc2_ll_conn_t *c, uint8_t side, uint8_t pdu_hdr,
			       const uint8_t *pay34, uint32_t connect_end_us,
			       jc2_link_diag_t *diag)
{
	static const uint8_t dle_req[9] = { LL_CTRL_LENGTH_REQ,
					    0xFB,
					    0x00,
					    0x48,
					    0x08,
					    0xFB,
					    0x00,
					    0x48,
					    0x08 };
	uint8_t win_size;
	uint16_t win_offset;
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
	uint8_t hop;
	uint8_t next_gen;

	if (!c || !pay34)
		return false;
	win_size = pay34[19];
	win_offset = jc2_read_u16_le(&pay34[20]);
	interval = jc2_read_u16_le(&pay34[22]);
	latency = jc2_read_u16_le(&pay34[24]);
	timeout = jc2_read_u16_le(&pay34[26]);
	hop = (uint8_t)(pay34[33] & 0x1Fu);

	/* Accept Switch 2's 5.0 ms (interval = 4) as well as standard >= 6. */
	if (interval < 4u || interval > 3200u || latency > 499u ||
	    timeout < 10u || timeout > 3200u || win_size == 0u ||
	    win_size > 8u || hop < 1u || hop > 16u)
		return false;

	next_gen = (uint8_t)(s_conn_gen_seq + 1u);
	if (next_gen == 0u)
		next_gen = 1u;
	s_conn_gen_seq = next_gen;

	memset(c, 0, sizeof(*c));
	c->conn_gen = next_gen;
	c->active = true;
	c->side = side & 1u;
	memcpy(c->peer_addr, &pay34[0], 6);
	c->peer_addr_type = (uint8_t)((pdu_hdr >> 6) & 1u);
	c->csa2 = ((pdu_hdr >> 5) & 1u) != 0u;
	c->access_addr = (uint32_t)pay34[12] | ((uint32_t)pay34[13] << 8) |
			 ((uint32_t)pay34[14] << 16) |
			 ((uint32_t)pay34[15] << 24);
	c->crc_init = (uint32_t)pay34[16] | ((uint32_t)pay34[17] << 8) |
		      ((uint32_t)pay34[18] << 16);
	c->chan_id =
		(uint16_t)((c->access_addr >> 16) ^ (c->access_addr & 0xFFFFu));

	c->hop_inc = hop;
	c->last_unmapped_ch = 0u;
	memcpy(c->chm, &pay34[28], 5);
	c->chm[4] &= 0x1Fu;
	rebuild_remap_table(c);

	c->phy_rx = 1u;
	c->phy_tx = 1u;
	c->mtu = 23u;
	c->interval_125 = interval;
	c->latency = latency;
	c->sup_timeout_10ms = timeout;
	c->interval_us = (uint32_t)interval * 1250u;
	c->sup_timeout_us = (uint32_t)timeout * 10000u;

	/* Legacy CONNECT_IND uses transmitWindowDelay = 1250 us. */
	c->anchor_us = connect_end_us + 1250u + (uint32_t)win_offset * 1250u;
	c->rx_win_extra_us = (uint32_t)win_size * 1250u;
	c->created_us = connect_end_us;
	c->last_rx_ok_us = connect_end_us;
	c->event_counter = 0u;
	jc2_ll_next_data_channel(c);

	/* Genuine Joy-Con 2 initiates 251-octet / 2120-us DLE on event 0. */
	(void)enqueue_tx_pdu(c, 0x03u, dle_req, sizeof(dle_req), false);
	(void)jc2_ll_stage_next_tx(c, NULL);

	if (diag) {
		uint8_t prev_disc_cnt = diag->disc_cnt;
		uint8_t prev_disc_rsn = diag->last_disc_reason;

		memset(diag, 0, sizeof(*diag));
		diag->disc_cnt = prev_disc_cnt;
		diag->last_disc_reason = prev_disc_rsn;
		diag->conn_interval_125 = interval;
		diag->slave_latency = latency;
		diag->sup_timeout_10ms = timeout;
		diag->mtu = 23u;
		diag->dle_tx_octets = 27u;
		diag->dle_rx_octets = 27u;
		diag->dle_tx_time_us = 328u;
		diag->dle_rx_time_us = 328u;
		diag->phy_tx = 1u;
		diag->phy_rx = 1u;
		diag->tx_win_us = connect_end_us;
	}
	jc2_ll_log_evt(JC2_EVT_CONNECTED, c->side, 0u);
	return true;
}

static bool instant_passed(uint16_t cur, uint16_t instant)
{
	/* Bluetooth Core Spec Vol 6 Part B 5.1: valid instants satisfy 0 <= (instant - cur) <= 32767. */
	return (int16_t)(cur - instant) > 0;
}

bool jc2_ll_apply_pending_instants(jc2_ll_conn_t *c)
{
	if (!c || !c->active)
		return false;

	if ((c->conn_upd_pending &&
	     instant_passed(c->event_counter, c->upd_instant)) ||
	    (c->chm_upd_pending &&
	     instant_passed(c->event_counter, c->chm_instant)) ||
	    (c->phy_upd_pending &&
	     instant_passed(c->event_counter, c->phy_instant))) {
		c->active = false;
		c->conn_upd_pending = false;
		c->chm_upd_pending = false;
		c->phy_upd_pending = false;
		c->terminate_reason = 0x28u;
		jc2_ll_log_evt(JC2_EVT_DISCONNECTED, c->side, 0x28u);
		return false;
	}

	if (c->conn_upd_pending && c->event_counter == c->upd_instant) {
		c->conn_upd_pending = false;
		/*
		 * At Instant, anchor_us was already stepped by the previous
		 * interval_us; apply transmitWindowOffset (delay = 0).
		 */
		c->anchor_us += (uint32_t)c->upd_win_offset * 1250u;
		c->rx_win_extra_us = (uint32_t)c->upd_win_size * 1250u;
		c->interval_125 = c->upd_interval_125;
		c->latency = c->upd_latency;
		c->sup_timeout_10ms = c->upd_timeout_10ms;
		c->interval_us = (uint32_t)c->interval_125 * 1250u;
		c->sup_timeout_us = (uint32_t)c->sup_timeout_10ms * 10000u;
		jc2_ll_log_evt(JC2_EVT_CONN_PARAM_UPD, c->side,
			       c->interval_125);
	}
	if (c->chm_upd_pending && c->event_counter == c->chm_instant) {
		c->chm_upd_pending = false;
		memcpy(c->chm, c->upd_chm, 5);
		c->chm[4] &= 0x1Fu;
		rebuild_remap_table(c);
	}
	if (c->phy_upd_pending && c->event_counter == c->phy_instant) {
		c->phy_upd_pending = false;
		if (c->upd_phy_m2s & 0x02u)
			c->phy_rx = 2u;
		else if (c->upd_phy_m2s & 0x01u)
			c->phy_rx = 1u;
		if (c->upd_phy_s2m & 0x02u)
			c->phy_tx = 2u;
		else if (c->upd_phy_s2m & 0x01u)
			c->phy_tx = 1u;
		jc2_ll_log_evt(JC2_EVT_PHY_UPD, c->side,
			       (uint16_t)((c->phy_tx << 8) | c->phy_rx));
	}
	return true;
}

static void notify_instant_passed(jc2_ll_conn_t *c, jc2_link_diag_t *diag,
				  const jc2_ll_callbacks_t *cbs)
{
	if (diag) {
		diag->disc_cnt++;
		diag->last_disc_reason = 0x28u;
	}
	if (cbs && cbs->on_disconnected)
		cbs->on_disconnected(c->side, 0x28u, cbs->user_ctx);
}

static void handle_ll_conn_upd(jc2_ll_conn_t *c, const uint8_t *pay,
			       uint8_t len, jc2_link_diag_t *diag,
			       const jc2_ll_callbacks_t *cbs)
{
	uint8_t wsize;
	uint16_t woff;
	uint16_t intv;
	uint16_t lat;
	uint16_t sup;
	uint16_t inst;
	uint32_t pm;
	bool ok;

	if (len < 12u)
		return;
	wsize = pay[1];
	woff = jc2_read_u16_le(&pay[2]);
	intv = jc2_read_u16_le(&pay[4]);
	lat = jc2_read_u16_le(&pay[6]);
	sup = jc2_read_u16_le(&pay[8]);
	inst = jc2_read_u16_le(&pay[10]);
	if (intv < 4u || wsize == 0u)
		return;

	pm = jc2_port_irq_lock();
	c->upd_win_size = wsize;
	c->upd_win_offset = woff;
	c->upd_interval_125 = intv;
	c->upd_latency = lat;
	c->upd_timeout_10ms = sup;
	c->upd_instant = inst;
	c->conn_upd_pending = true;
	ok = jc2_ll_apply_pending_instants(c);
	jc2_port_irq_unlock(pm);

	if (!ok) {
		notify_instant_passed(c, diag, cbs);
		return;
	}

	if (diag) {
		diag->conn_interval_125 = intv;
		diag->slave_latency = lat;
		diag->sup_timeout_10ms = sup;
		diag->cp_upd_cnt++;
	}
}

static void handle_ll_enc_req(jc2_ll_conn_t *c, const uint8_t *pay, uint8_t len,
			      const jc2_ll_callbacks_t *cbs)
{
	uint8_t ltk_le[16];
	uint8_t skds[8];
	uint8_t ivs[4];
	uint8_t enc_rsp[13];
	uint8_t start_req[1] = { LL_CTRL_START_ENC_REQ };
	uint32_t pm;

	if (len < 23u)
		return;
	if (!cbs || !cbs->get_ltk ||
	    !cbs->get_ltk(c->side, ltk_le, cbs->user_ctx)) {
		uint8_t rej[2] = { LL_CTRL_REJECT_IND, 0x06u };

		(void)jc2_ll_enqueue_ctrl_pdu(c, rej, sizeof(rej), false);
		c->enc_pause_data = false;
		return;
	}

	jc2_port_random_bytes(skds, sizeof(skds));
	jc2_port_random_bytes(ivs, sizeof(ivs));
	jc2_ble_derive_sk(ltk_le, &pay[11], skds, c->sk_be);
	memcpy(&c->iv[0], &pay[19], 4);
	memcpy(&c->iv[4], ivs, 4);

	pm = jc2_port_irq_lock();
	c->rx_ccm_cnt = 0u;
	c->tx_ccm_cnt = 0u;
	c->enc_tx = false;
	c->enc_pause_data = true;
	c->enc_rx = true;
	jc2_port_irq_unlock(pm);

	enc_rsp[0] = LL_CTRL_ENC_RSP;
	memcpy(&enc_rsp[1], skds, 8);
	memcpy(&enc_rsp[9], ivs, 4);
	(void)jc2_ll_enqueue_ctrl_pdu(c, enc_rsp, sizeof(enc_rsp), false);
	(void)jc2_ll_enqueue_ctrl_pdu(c, start_req, sizeof(start_req), false);
}

static void handle_ll_length(jc2_ll_conn_t *c, uint8_t op, const uint8_t *pay,
			     uint8_t len, jc2_link_diag_t *diag)
{
	if (len >= 9u && diag) {
		uint16_t peer_rx_oct = jc2_read_u16_le(&pay[1]);
		uint16_t peer_rx_us = jc2_read_u16_le(&pay[3]);
		uint16_t peer_tx_oct = jc2_read_u16_le(&pay[5]);
		uint16_t peer_tx_us = jc2_read_u16_le(&pay[7]);

		diag->dle_tx_octets = (peer_rx_oct < 251u) ? peer_rx_oct : 251u;
		diag->dle_rx_octets = (peer_tx_oct < 251u) ? peer_tx_oct : 251u;
		diag->dle_tx_time_us = (peer_rx_us < 2120u) ? peer_rx_us :
							      2120u;
		diag->dle_rx_time_us = (peer_tx_us < 2120u) ? peer_tx_us :
							      2120u;
		jc2_ll_log_evt(JC2_EVT_DLE_UPD, c->side, diag->dle_tx_octets);
	}
	if (op == LL_CTRL_LENGTH_REQ) {
		static const uint8_t dle_rsp[9] = { LL_CTRL_LENGTH_RSP,
						    0xFB,
						    0x00,
						    0x48,
						    0x08,
						    0xFB,
						    0x00,
						    0x48,
						    0x08 };

		(void)enqueue_tx_pdu(c, 0x03u, dle_rsp, sizeof(dle_rsp), false);
	}
}

static void handle_ll_phy_upd(jc2_ll_conn_t *c, const uint8_t *pay, uint8_t len,
			      jc2_link_diag_t *diag,
			      const jc2_ll_callbacks_t *cbs)
{
	uint8_t m2s;
	uint8_t s2m;
	uint16_t inst;
	uint32_t pm;
	bool ok;

	if (len < 5u)
		return;
	m2s = pay[1];
	s2m = pay[2];
	inst = jc2_read_u16_le(&pay[3]);
	if (m2s == 0u && s2m == 0u)
		return;

	pm = jc2_port_irq_lock();
	c->upd_phy_m2s = m2s;
	c->upd_phy_s2m = s2m;
	c->phy_instant = inst;
	c->phy_upd_pending = true;
	ok = jc2_ll_apply_pending_instants(c);
	jc2_port_irq_unlock(pm);

	if (!ok) {
		notify_instant_passed(c, diag, cbs);
		return;
	}

	if (diag) {
		if (s2m & 0x02u)
			diag->phy_tx = 2u;
		else if (s2m & 0x01u)
			diag->phy_tx = 1u;
		if (m2s & 0x02u)
			diag->phy_rx = 2u;
		else if (m2s & 0x01u)
			diag->phy_rx = 1u;
	}
}

static void handle_ll_ctrl(jc2_ll_conn_t *c, const uint8_t *pay, uint8_t len,
			   jc2_link_diag_t *diag, const jc2_ll_callbacks_t *cbs)
{
	uint8_t op;

	if (len == 0u)
		return;
	op = pay[0];

	switch (op) {
	case LL_CTRL_CONNECTION_UPDATE_IND:
		handle_ll_conn_upd(c, pay, len, diag, cbs);
		return;
	case LL_CTRL_CHANNEL_MAP_IND:
		if (len >= 8u) {
			uint32_t pm = jc2_port_irq_lock();
			bool ok;

			memcpy(c->upd_chm, &pay[1], 5);
			c->chm_instant = jc2_read_u16_le(&pay[6]);
			c->chm_upd_pending = true;
			ok = jc2_ll_apply_pending_instants(c);
			jc2_port_irq_unlock(pm);
			if (!ok)
				notify_instant_passed(c, diag, cbs);
		}
		return;
	case LL_CTRL_TERMINATE_IND: {
		uint8_t rsn = (len >= 2u) ? pay[1] : 0x13u;
		uint32_t pm = jc2_port_irq_lock();

		c->active = false;
		jc2_port_irq_unlock(pm);
		if (diag) {
			diag->disc_cnt++;
			diag->last_disc_reason = rsn;
		}
		jc2_ll_log_evt(JC2_EVT_DISCONNECTED, c->side, rsn);
		if (cbs && cbs->on_disconnected)
			cbs->on_disconnected(c->side, rsn, cbs->user_ctx);
		return;
	}
	case LL_CTRL_ENC_REQ:
		handle_ll_enc_req(c, pay, len, cbs);
		return;
	case LL_CTRL_START_ENC_RSP: {
		uint8_t start_rsp[1] = { LL_CTRL_START_ENC_RSP };

		c->tx_ccm_cnt = 0u;
		c->enc_tx = true;
		c->secured = true;
		(void)jc2_ll_enqueue_ctrl_pdu(c, start_rsp, sizeof(start_rsp),
					      false);
		c->enc_pause_data = false;
		jc2_ll_log_evt(JC2_EVT_SEC_UPD, c->side, 0u);
		return;
	}
	case LL_CTRL_FEATURE_REQ:
	case LL_CTRL_SLAVE_FEATURE_REQ: {
		/*
		 * Exact feature mask reported by genuine Joy-Con 2 hardware
		 * (omits Connection Parameters Request bit 1 so Switch 2
		 * always uses direct LL_CONNECTION_UPDATE_IND).
		 */
		static const uint8_t feat_rsp[9] = { LL_CTRL_FEATURE_RSP,
						     0xFD,
						     0xFD,
						     0xFF,
						     0xF7,
						     0x00,
						     0x00,
						     0x00,
						     0x00 };

		(void)enqueue_tx_pdu(c, 0x03u, feat_rsp, sizeof(feat_rsp),
				     false);
		return;
	}
	case LL_CTRL_VERSION_IND:
		if (!c->version_sent) {
			static const uint8_t ver_rsp[6] = { LL_CTRL_VERSION_IND,
							    0x0Cu,
							    0x0Fu,
							    0x00u,
							    0x01u,
							    0x00u };

			c->version_sent = true;
			(void)enqueue_tx_pdu(c, 0x03u, ver_rsp, sizeof(ver_rsp),
					     false);
		}
		return;
	case LL_CTRL_PING_REQ: {
		uint8_t prsp[1] = { LL_CTRL_PING_RSP };

		(void)enqueue_tx_pdu(c, 0x03u, prsp, sizeof(prsp), false);
		return;
	}
	case LL_CTRL_LENGTH_REQ:
	case LL_CTRL_LENGTH_RSP:
		handle_ll_length(c, op, pay, len, diag);
		return;
	case LL_CTRL_PHY_REQ: {
		static const uint8_t phy_rsp[3] = { LL_CTRL_PHY_RSP, 0x02u,
						    0x02u };

		(void)enqueue_tx_pdu(c, 0x03u, phy_rsp, sizeof(phy_rsp), false);
		return;
	}
	case LL_CTRL_PHY_UPDATE_IND:
		handle_ll_phy_upd(c, pay, len, diag, cbs);
		return;
	case LL_CTRL_UNKNOWN_RSP:
	case LL_CTRL_FEATURE_RSP:
	case LL_CTRL_PING_RSP:
	case LL_CTRL_PHY_RSP:
	case LL_CTRL_REJECT_IND:
		return;
	default: {
		uint8_t unk[2] = { LL_CTRL_UNKNOWN_RSP, op };

		(void)enqueue_tx_pdu(c, 0x03u, unk, sizeof(unk), false);
		return;
	}
	}
}

void jc2_ll_process_rx_pdu(jc2_ll_conn_t *c, uint8_t hdr,
			   const uint8_t *payload, uint8_t len,
			   jc2_link_diag_t *diag, const jc2_ll_callbacks_t *cbs)
{
	uint8_t llid;

	if (!c || !c->active || !payload || len == 0u)
		return;
	llid = (uint8_t)(hdr & 0x03u);
	if (llid == 0x03u) {
		handle_ll_ctrl(c, payload, len, diag, cbs);
		return;
	}
	if (llid == 0x02u) {
		uint16_t l2len;
		uint16_t cid;
		uint16_t frag;

		if (len < 4u)
			return;
		l2len = jc2_read_u16_le(&payload[0]);
		cid = jc2_read_u16_le(&payload[2]);
		frag = (uint16_t)(len - 4u);
		if (l2len > sizeof(c->l2cap_rx_buf)) {
			c->l2cap_rx_expected = 0u;
			return;
		}
		if (frag >= l2len) {
			c->l2cap_rx_expected = 0u;
			if (cid == 0x0004u)
				jc2_att_handle_pdu(c, &payload[4], l2len, diag,
						   cbs);
		} else {
			memcpy(c->l2cap_rx_buf, &payload[4], frag);
			c->l2cap_rx_len = frag;
			c->l2cap_rx_expected = l2len;
			c->l2cap_rx_cid = cid;
		}
		return;
	}
	if (llid == 0x01u && c->l2cap_rx_expected > 0u) {
		uint16_t need =
			(uint16_t)(c->l2cap_rx_expected - c->l2cap_rx_len);
		uint16_t copy_n = (len < need) ? len : need;

		memcpy(&c->l2cap_rx_buf[c->l2cap_rx_len], payload, copy_n);
		c->l2cap_rx_len = (uint16_t)(c->l2cap_rx_len + copy_n);
		if (c->l2cap_rx_len >= c->l2cap_rx_expected) {
			uint16_t full_len = c->l2cap_rx_expected;
			uint16_t cid = c->l2cap_rx_cid;

			c->l2cap_rx_expected = 0u;
			if (cid == 0x0004u)
				jc2_att_handle_pdu(c, c->l2cap_rx_buf, full_len,
						   diag, cbs);
		}
	}
}

static bool stage_raw_pdu(jc2_ll_conn_t *c, uint8_t gen, uint8_t llid,
			  const uint8_t *pt, uint8_t pt_len, bool is_input,
			  bool is_term, uint8_t src)
{
	/*
	 * Encrypt into a stack buffer with priority-0 radio IRQs enabled so
	 * AES-CCM never delays T_IFS and cannot clobber a reconnected slot.
	 * Advance tx_ccm_cnt only after committing inside the IRQ lock.
	 */
	uint8_t tmp[258];
	uint8_t hdr0 = (uint8_t)(llid & 0x03u);
	bool was_enc = c->enc_tx;
	bool did_enc = false;
	uint64_t ccm_cnt = 0u;
	uint16_t raw_total;
	uint32_t pm;

	tmp[0] = hdr0;
	if (was_enc && pt_len > 0u) {
		ccm_cnt = c->tx_ccm_cnt;
		did_enc = true;
		tmp[1] = (uint8_t)(pt_len + 4u);
		jc2_ble_ccm_encrypt(c->sk_be, c->iv, ccm_cnt, false, hdr0, pt,
				    pt_len, &tmp[2]);
		raw_total = (uint16_t)(pt_len + 6u);
	} else {
		tmp[1] = pt_len;
		if (pt_len > 0u)
			memcpy(&tmp[2], pt, pt_len);
		raw_total = (uint16_t)(pt_len + 2u);
	}

	pm = jc2_port_irq_lock();
	if (!c->active || c->conn_gen != gen || c->staged_valid ||
	    (!was_enc && c->enc_pause_data && llid != 0x03u)) {
		jc2_port_irq_unlock(pm);
		return false;
	}
	if (did_enc)
		c->tx_ccm_cnt = ccm_cnt + 1u;
	memcpy(c->staged_buf.bytes, tmp, raw_total);
	c->staged_is_input = is_input;
	c->staged_is_term = is_term;
	c->staged_src = src;
	jc2_port_dmb();
	c->staged_valid = true;
	jc2_port_irq_unlock(pm);
	return true;
}

bool jc2_ll_stage_next_tx(jc2_ll_conn_t *c, const jc2_ll_callbacks_t *cbs)
{
	uint8_t gen;

	if (!c || !c->active)
		return false;
	if (c->staged_valid)
		return true;

	gen = c->conn_gen;

	/*
	 * Peek-on-stage, pop-on-promote: RADIO_IRQHandler advances ll_q_head /
	 * tx_q_head only when promoting staged_buf to inflight_buf, preserving
	 * any unencrypted ATT PDU if LL_ENC_REQ arrives mid-flight.
	 */
	if (c->ll_q_head != c->ll_q_tail) {
		uint8_t slot = c->ll_q_head & (JC2_LL_Q_DEPTH - 1u);
		const jc2_ll_ctrl_pdu_t *lp = &c->ll_q[slot];

		return stage_raw_pdu(c, gen, 0x03u, lp->data, lp->len, false,
				     lp->is_term != 0u, JC2_STAGED_LL_Q);
	}

	if (!c->enc_pause_data && c->tx_q_head != c->tx_q_tail) {
		uint8_t slot = c->tx_q_head & (JC2_TX_Q_DEPTH - 1u);
		const jc2_ll_tx_pdu_t *p = &c->tx_q[slot];

		return stage_raw_pdu(c, gen, p->llid, p->data, p->len,
				     p->is_input_hvn != 0u, false,
				     JC2_STAGED_ATT_Q);
	}

	if (!c->enc_pause_data && cbs && cbs->build_live_input) {
		uint16_t hdl = 0u;
		uint8_t pdu[76];
		uint16_t vlen = cbs->build_live_input(c->side, &hdl, &pdu[7],
						      64u, cbs->user_ctx);

		if (vlen > 0u && vlen <= 64u) {
			jc2_write_u16_le(&pdu[0], (uint16_t)(vlen + 3u));
			jc2_write_u16_le(&pdu[2], 0x0004u);
			pdu[4] = ATT_OP_HANDLE_VAL_NTF;
			jc2_write_u16_le(&pdu[5], hdl);
			return stage_raw_pdu(c, gen, 0x02u, pdu,
					     (uint8_t)(vlen + 7u), true, false,
					     JC2_STAGED_LIVE);
		}
	}
	return false;
}

const uint8_t *jc2_ll_arq_step_rx(jc2_ll_conn_t *c, uint8_t rx_hdr,
				  const uint8_t *rx_payload_raw,
				  uint8_t rx_raw_len, bool *out_new_rx,
				  bool *out_term_acked, jc2_link_diag_t *diag)
{
	uint8_t rx_nesn = (uint8_t)((rx_hdr >> 2) & 1u);
	uint8_t rx_sn = (uint8_t)((rx_hdr >> 3) & 1u);
	bool is_new;
	uint8_t *tx_ptr = NULL;
	bool has_more;
	uint8_t md;

	if (out_term_acked)
		*out_term_acked = false;

	if (!c->has_transmitted) {
		c->has_transmitted = true;
	} else if (rx_nesn != c->tx_sn) {
		c->tx_sn ^= 1u;
		c->last_empty_unacked = false;
		if (c->tx_inflight) {
			c->tx_inflight = false;
			if (c->inflight_is_input && diag) {
				diag->hvx_ok++;
				diag->tx_win_cnt++;
			}
			if (c->terminate_pending && c->inflight_is_term) {
				c->active = false;
				if (diag) {
					diag->disc_cnt++;
					diag->last_disc_reason =
						c->terminate_reason;
				}
				jc2_ll_log_evt(JC2_EVT_DISCONNECTED, c->side,
					       c->terminate_reason);
				if (out_term_acked)
					*out_term_acked = true;
			}
		}
	}

	is_new = (rx_sn == c->nesn);
	if (is_new) {
		c->nesn ^= 1u;
		if (!c->enc_rx && (rx_hdr & 0x03u) == 0x03u &&
		    rx_raw_len >= 1u && rx_payload_raw &&
		    rx_payload_raw[0] == LL_CTRL_ENC_REQ) {
			c->enc_pause_data = true;
			/*
			 * Drop only unencrypted data PDUs; staged_src leaves
			 * any peeked tx_q item intact for post-ENC encryption.
			 */
			if (c->staged_valid &&
			    c->staged_src != JC2_STAGED_LL_Q && !c->enc_tx) {
				c->staged_valid = false;
				c->staged_src = JC2_STAGED_NONE;
			}
		}
	}
	if (out_new_rx)
		*out_new_rx = is_new;

	if (c->tx_inflight) {
		tx_ptr = c->inflight_buf.bytes;
	} else if (!c->last_empty_unacked && c->staged_valid) {
		uint16_t raw_total = (uint16_t)(c->staged_buf.bytes[1] + 2u);

		memcpy(c->inflight_buf.bytes, c->staged_buf.bytes, raw_total);
		c->inflight_is_input = c->staged_is_input;
		c->inflight_is_term = c->staged_is_term;
		if (c->staged_src == JC2_STAGED_LL_Q &&
		    c->ll_q_head != c->ll_q_tail)
			c->ll_q_head++;
		else if (c->staged_src == JC2_STAGED_ATT_Q &&
			 c->tx_q_head != c->tx_q_tail)
			c->tx_q_head++;
		c->staged_src = JC2_STAGED_NONE;
		c->staged_valid = false;
		c->tx_inflight = true;
		tx_ptr = c->inflight_buf.bytes;
	}

	if (!tx_ptr) {
		c->empty_buf.bytes[0] = 0x01u;
		c->empty_buf.bytes[1] = 0x00u;
		c->last_empty_unacked = true;
		tx_ptr = c->empty_buf.bytes;
	}

	has_more = c->staged_valid || (c->ll_q_head != c->ll_q_tail) ||
		   (!c->enc_pause_data && c->tx_q_head != c->tx_q_tail);
	md = has_more ? 0x10u : 0x00u;
	tx_ptr[0] = (uint8_t)((tx_ptr[0] & 0xE3u) | ((c->nesn & 1u) << 2) |
			      ((c->tx_sn & 1u) << 3) | md);
	return tx_ptr;
}
