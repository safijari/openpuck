/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#include "jc2_ll_loopback.h"
#include "jc2_crypto.h"
#include "jc2_port.h"

#include <string.h>

#if defined(JC2_HOST_TEST)

static jc2_ll_callbacks_t s_lb_cbs = { 0 };
static jc2_ll_conn_t s_lb_conns[JC2_MAX_LINKS];
static jc2_link_diag_t s_lb_diags[JC2_MAX_LINKS];
static uint8_t s_lb_macs[JC2_MAX_LINKS][6];

static bool s_lb_adv = false;
static uint8_t s_lb_adv_side = 0u;
static uint8_t s_lb_adv_data[31];
static uint8_t s_lb_adv_len = 0u;
static uint8_t s_lb_scan_rsp[31];
static uint8_t s_lb_scan_rsp_len = 0u;

static uint32_t s_lb_now_us = 1000u;
static uint32_t s_lb_gap_free_us = 0xFFFFFFFFu;
static bool s_lb_gap_held = false;
static jc2_gap_stats_t s_lb_gap_stats = { 0 };
static uint32_t s_lb_prng_state = 0xA53C9E47u;

uint32_t jc2_port_irq_lock(void)
{
	return 0u;
}

void jc2_port_irq_unlock(uint32_t key)
{
	(void)key;
}

uint32_t jc2_port_bh_lock(void)
{
	return 0u;
}

void jc2_port_bh_unlock(uint32_t key)
{
	(void)key;
}

void jc2_port_dmb(void)
{
}

void jc2_port_random_bytes(uint8_t *out, size_t len)
{
	size_t i;

	if (!out)
		return;
	for (i = 0u; i < len; i++) {
		s_lb_prng_state = s_lb_prng_state * 1664525u + 1013904223u;
		out[i] = (uint8_t)(s_lb_prng_state >> 24);
	}
}

bool jc2_port_aes128_ecb_be(const uint8_t key_be[16], const uint8_t pt_be[16],
			    uint8_t ct_be[16])
{
	(void)key_be;
	(void)pt_be;
	(void)ct_be;
	/* Always exercise portable software AES-128 in host tests. */
	return false;
}

void jc2_lb_reset(void)
{
	memset(&s_lb_cbs, 0, sizeof(s_lb_cbs));
	memset(s_lb_conns, 0, sizeof(s_lb_conns));
	memset(s_lb_diags, 0, sizeof(s_lb_diags));
	memset(s_lb_macs, 0, sizeof(s_lb_macs));
	s_lb_adv = false;
	s_lb_adv_side = 0u;
	s_lb_adv_len = 0u;
	s_lb_scan_rsp_len = 0u;
	s_lb_now_us = 1000u;
	s_lb_gap_free_us = 0xFFFFFFFFu;
	s_lb_gap_held = false;
	memset(&s_lb_gap_stats, 0, sizeof(s_lb_gap_stats));
	s_lb_prng_state = 0xA53C9E47u;
}

void jc2_lb_set_now_us(uint32_t now_us)
{
	s_lb_now_us = now_us;
}

void jc2_lb_advance_us(uint32_t delta_us)
{
	s_lb_now_us += delta_us;
}

void jc2_lb_set_gap_budget_us(uint32_t free_us)
{
	s_lb_gap_free_us = free_us;
}

bool jc2_lb_get_adv(uint8_t *out_side, uint8_t out_mac[6], uint8_t out_adv[31],
		    uint8_t *out_adv_len, uint8_t out_scan_rsp[31],
		    uint8_t *out_scan_rsp_len)
{
	if (!s_lb_adv)
		return false;
	if (out_side)
		*out_side = s_lb_adv_side;
	if (out_mac)
		memcpy(out_mac, s_lb_macs[s_lb_adv_side & 1u], 6);
	if (out_adv && s_lb_adv_len > 0u)
		memcpy(out_adv, s_lb_adv_data, s_lb_adv_len);
	if (out_adv_len)
		*out_adv_len = s_lb_adv_len;
	if (out_scan_rsp && s_lb_scan_rsp_len > 0u)
		memcpy(out_scan_rsp, s_lb_scan_rsp, s_lb_scan_rsp_len);
	if (out_scan_rsp_len)
		*out_scan_rsp_len = s_lb_scan_rsp_len;
	return true;
}

bool jc2_lb_connect(jc2_lb_central_t *cent, uint8_t side, uint8_t pdu_hdr,
		    const uint8_t pay34[34])
{
	jc2_ll_conn_t *c;

	side &= 1u;
	c = &s_lb_conns[side];
	if (!cent || !pay34 || c->active)
		return false;
	if (!jc2_ll_accept_connect_ind(c, side, pdu_hdr, pay34, s_lb_now_us,
				       &s_lb_diags[side]))
		return false;

	memset(cent, 0, sizeof(*cent));
	cent->side = side;
	s_lb_adv = false;
	if (s_lb_cbs.on_connected)
		s_lb_cbs.on_connected(side, c->peer_addr, c->peer_addr_type,
				      s_lb_cbs.user_ctx);
	return true;
}

bool jc2_lb_exchange(jc2_lb_central_t *cent, uint8_t rx_llid,
		     const uint8_t *rx_plain, uint8_t rx_plain_len,
		     uint8_t *out_tx_llid, uint8_t *out_tx_plain,
		     uint8_t *out_tx_plain_len, uint8_t *out_tx_raw_hdr)
{
	jc2_ll_conn_t *c;
	uint8_t rx_hdr;
	uint8_t rx_air[255];
	uint8_t rx_air_len;
	bool was_enc_rx;
	bool is_new = false;
	bool term_acked = false;
	const uint8_t *tx_ptr;
	uint8_t tx_hdr;
	uint8_t tx_air_len;
	uint8_t tx_sn;

	if (!cent)
		return false;
	c = &s_lb_conns[cent->side & 1u];
	if (!c->active)
		return false;

	if (!c->staged_valid)
		(void)jc2_ll_stage_next_tx(c, &s_lb_cbs);

	rx_hdr = (uint8_t)((rx_llid & 0x03u) | ((cent->nesn & 1u) << 2) |
			   ((cent->sn & 1u) << 3));
	if (cent->enc_tx && rx_plain_len > 0u) {
		if (!rx_plain || rx_plain_len > 251u)
			return false;
		jc2_ble_ccm_encrypt(cent->sk_be, cent->iv8, cent->m2s_ccm_cnt++,
				    true, rx_hdr, rx_plain, rx_plain_len,
				    rx_air);
		rx_air_len = (uint8_t)(rx_plain_len + 4u);
	} else {
		rx_air_len = rx_plain_len;
		if (rx_air_len > 0u && rx_plain)
			memcpy(rx_air, rx_plain, rx_air_len);
	}

	was_enc_rx = c->enc_rx;
	tx_ptr = jc2_ll_arq_step_rx(c, rx_hdr, rx_air, rx_air_len, &is_new,
				    &term_acked, &s_lb_diags[c->side]);
	tx_hdr = tx_ptr[0];
	tx_air_len = tx_ptr[1];
	tx_sn = (uint8_t)((tx_hdr >> 3) & 1u);

	if (is_new)
		cent->sn ^= 1u;
	if (tx_sn == cent->nesn)
		cent->nesn ^= 1u;

	if (out_tx_raw_hdr)
		*out_tx_raw_hdr = tx_hdr;
	if (out_tx_llid)
		*out_tx_llid = (uint8_t)(tx_hdr & 0x03u);

	if (cent->enc_rx && tx_air_len > 0u) {
		uint8_t dec[251];

		if (!jc2_ble_ccm_decrypt(cent->sk_be, cent->iv8,
					 cent->s2m_ccm_cnt++, false, tx_hdr,
					 &tx_ptr[2], tx_air_len, dec))
			return false;
		if (out_tx_plain)
			memcpy(out_tx_plain, dec, (size_t)(tx_air_len - 4u));
		if (out_tx_plain_len)
			*out_tx_plain_len = (uint8_t)(tx_air_len - 4u);
	} else {
		if (out_tx_plain && tx_air_len > 0u)
			memcpy(out_tx_plain, &tx_ptr[2], tx_air_len);
		if (out_tx_plain_len)
			*out_tx_plain_len = tx_air_len;
	}

	if (term_acked && s_lb_cbs.on_disconnected)
		s_lb_cbs.on_disconnected(c->side, c->terminate_reason,
					 s_lb_cbs.user_ctx);

	if (c->active && is_new && rx_air_len > 0u) {
		if (was_enc_rx) {
			uint8_t pt[251];

			if (!jc2_ble_ccm_decrypt(c->sk_be, c->iv,
						 c->rx_ccm_cnt++, true, rx_hdr,
						 rx_air, rx_air_len, pt)) {
				c->active = false;
				s_lb_diags[c->side].disc_cnt++;
				s_lb_diags[c->side].last_disc_reason = 0x3Du;
				jc2_ll_log_evt(JC2_EVT_DISCONNECTED, c->side,
					       0x3Du);
				if (s_lb_cbs.on_disconnected)
					s_lb_cbs.on_disconnected(
						c->side, 0x3Du,
						s_lb_cbs.user_ctx);
				return true;
			}
			jc2_ll_process_rx_pdu(c, rx_hdr, pt,
					      (uint8_t)(rx_air_len - 4u),
					      &s_lb_diags[c->side], &s_lb_cbs);
		} else {
			jc2_ll_process_rx_pdu(c, rx_hdr, rx_air, rx_air_len,
					      &s_lb_diags[c->side], &s_lb_cbs);
		}
	}

	if (c->active) {
		s_lb_now_us += c->interval_us;
		c->anchor_us += c->interval_us;
		c->event_counter++;
		if (!jc2_ll_apply_pending_instants(c)) {
			s_lb_diags[c->side].disc_cnt++;
			s_lb_diags[c->side].last_disc_reason = 0x28u;
			if (s_lb_cbs.on_disconnected)
				s_lb_cbs.on_disconnected(c->side, 0x28u,
							 s_lb_cbs.user_ctx);
			return true;
		}
		jc2_ll_next_data_channel(c);
		if (!c->staged_valid)
			(void)jc2_ll_stage_next_tx(c, &s_lb_cbs);
	}
	return true;
}

jc2_ll_conn_t *jc2_lb_get_conn(uint8_t side)
{
	return &s_lb_conns[side & 1u];
}

void jc2_ll_read_device_mac(uint8_t out_mac[6])
{
	static const uint8_t host_mac[6] = {
		0x11, 0x22, 0x33, 0xEB, 0xF1, 0x48
	};

	if (out_mac)
		memcpy(out_mac, host_mac, 6);
}

void jc2_ll_init(const jc2_ll_callbacks_t *cbs)
{
	jc2_lb_reset();
	if (cbs)
		s_lb_cbs = *cbs;
}

void jc2_ll_set_mac(uint8_t side, const uint8_t mac_le[6])
{
	if (mac_le)
		memcpy(s_lb_macs[side & 1u], mac_le, 6);
}

void jc2_ll_start_adv(uint8_t side, const uint8_t *adv_data, uint8_t adv_len,
		      const uint8_t *scan_rsp, uint8_t scan_rsp_len)
{
	if (adv_len > 31u)
		adv_len = 31u;
	if (scan_rsp_len > 31u)
		scan_rsp_len = 31u;
	s_lb_adv = true;
	s_lb_adv_side = side & 1u;
	s_lb_adv_len = adv_len;
	if (adv_len > 0u && adv_data)
		memcpy(s_lb_adv_data, adv_data, adv_len);
	s_lb_scan_rsp_len = scan_rsp_len;
	if (scan_rsp_len > 0u && scan_rsp)
		memcpy(s_lb_scan_rsp, scan_rsp, scan_rsp_len);
}

void jc2_ll_stop_adv(void)
{
	s_lb_adv = false;
}

bool jc2_ll_is_advertising(void)
{
	return s_lb_adv;
}

uint8_t jc2_ll_adv_side(void)
{
	return s_lb_adv_side;
}

bool jc2_ll_is_connected(uint8_t side)
{
	return s_lb_conns[side & 1u].active;
}

bool jc2_ll_is_secured(uint8_t side)
{
	const jc2_ll_conn_t *c = &s_lb_conns[side & 1u];

	return c->active && c->secured;
}

uint16_t jc2_ll_get_cccd(uint8_t side, uint16_t cccd_handle)
{
	const jc2_ll_conn_t *c = &s_lb_conns[side & 1u];

	if (!c->active)
		return 0u;
	if (cccd_handle == JC2_HDL_INPUT05_CCCD)
		return c->cccd_000a;
	if (cccd_handle == JC2_HDL_INPUT0708_CCCD)
		return c->cccd_000e;
	if (cccd_handle == JC2_HDL_CMD_RESP_CCCD)
		return c->cccd_001a;
	if (cccd_handle == JC2_HDL_EXT_RESP_CCCD)
		return c->cccd_001e;
	if (cccd_handle == JC2_HDL_UNK22_CCCD)
		return c->cccd_0022;
	if (cccd_handle == JC2_HDL_UNK26_CCCD)
		return c->cccd_0026;
	return 0u;
}

uint16_t jc2_ll_get_mtu(uint8_t side)
{
	const jc2_ll_conn_t *c = &s_lb_conns[side & 1u];

	return c->active ? c->mtu : 0u;
}

uint8_t jc2_ll_tx_queue_depth(uint8_t side)
{
	const jc2_ll_conn_t *c = &s_lb_conns[side & 1u];

	if (!c->active)
		return 0u;
	return (uint8_t)((uint8_t)(c->tx_q_tail - c->tx_q_head) +
			 (uint8_t)(c->ll_q_tail - c->ll_q_head));
}

bool jc2_ll_notify(uint8_t side, uint16_t val_handle, const uint8_t *data,
		   uint16_t len, bool is_input_hvn)
{
	jc2_ll_conn_t *c = &s_lb_conns[side & 1u];
	bool ok;

	ok = jc2_ll_enqueue_att_ntf(c, val_handle, data, len, is_input_hvn);
	if (!ok && is_input_hvn && c->active)
		s_lb_diags[side & 1u].hvx_drop++;
	return ok;
}

void jc2_ll_disconnect(uint8_t side, uint8_t reason)
{
	jc2_ll_conn_t *c = &s_lb_conns[side & 1u];

	if (!c->active || c->terminate_pending)
		return;
	{
		uint8_t term[2] = { 0x02u, reason };

		c->terminate_reason = reason;
		c->terminate_pending = true;
		if (!jc2_ll_enqueue_ctrl_pdu(c, term, sizeof(term), true)) {
			c->active = false;
			s_lb_diags[c->side].disc_cnt++;
			s_lb_diags[c->side].last_disc_reason = reason;
			jc2_ll_log_evt(JC2_EVT_DISCONNECTED, c->side, reason);
			if (s_lb_cbs.on_disconnected)
				s_lb_cbs.on_disconnected(c->side, reason,
							 s_lb_cbs.user_ctx);
		}
	}
}

void jc2_ll_poll(void)
{
	uint8_t s;

	for (s = 0u; s < JC2_MAX_LINKS; s++) {
		if (s_lb_conns[s].active && !s_lb_conns[s].staged_valid)
			(void)jc2_ll_stage_next_tx(&s_lb_conns[s], &s_lb_cbs);
	}
}

uint32_t jc2_ll_lock_bottom_half(void)
{
	return jc2_port_bh_lock();
}

void jc2_ll_unlock_bottom_half(uint32_t prev_key)
{
	jc2_port_bh_unlock(prev_key);
}

uint32_t jc2_ll_now_us(void)
{
	return s_lb_now_us;
}

uint32_t jc2_ll_remaining_gap_us(void)
{
	return s_lb_gap_free_us;
}

bool jc2_ll_acquire_gap(uint32_t min_free_us)
{
	if (s_lb_gap_held || s_lb_gap_free_us < min_free_us) {
		s_lb_gap_stats.skips++;
		return false;
	}
	s_lb_gap_held = true;
	s_lb_gap_stats.grants++;
	return true;
}

void jc2_ll_release_gap(void)
{
	s_lb_gap_held = false;
}

const jc2_link_diag_t *jc2_ll_get_diag(uint8_t side)
{
	return &s_lb_diags[side & 1u];
}

void jc2_ll_get_gap_stats(jc2_gap_stats_t *out, bool reset)
{
	if (out)
		*out = s_lb_gap_stats;
	if (reset)
		memset(&s_lb_gap_stats, 0, sizeof(s_lb_gap_stats));
}

#else /* Target firmware build: jc2_ll_loopback is host-test only */

typedef int jc2_ll_loopback_unused_tu_t;

#endif /* defined(JC2_HOST_TEST) */
