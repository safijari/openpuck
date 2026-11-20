/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#include "joycon2.h"
#include "jc2_port.h"

#include <string.h>

#define JC2_ADV_ALT_PAIR_MS 2200u
#define JC2_ADV_ALT_RECONN_MS 1100u
#define JC2_BOND_FLUSH_GAP_US 1500u

static bool is_side_enabled(const jc2_device_t *dev, uint8_t side)
{
	side &= 1u;
	if (dev->cfg.mode == JC2_MODE_RIGHT_ONLY)
		return side == JC2_SIDE_RIGHT;
	if (dev->cfg.mode == JC2_MODE_LEFT_ONLY)
		return side == JC2_SIDE_LEFT;
	return true;
}

static uint8_t primary_adv_side(const jc2_device_t *dev)
{
	if (dev->cfg.mode == JC2_MODE_RIGHT_ONLY)
		return JC2_SIDE_RIGHT;
	return JC2_SIDE_LEFT;
}

static bool all_enabled_have_ltk(const jc2_device_t *dev)
{
	bool r_ok = !is_side_enabled(dev, JC2_SIDE_RIGHT) ||
		    (dev->proto.bond.has_ltk[JC2_SIDE_RIGHT] != 0u);
	bool l_ok = !is_side_enabled(dev, JC2_SIDE_LEFT) ||
		    (dev->proto.bond.has_ltk[JC2_SIDE_LEFT] != 0u);

	return r_ok && l_ok;
}

static uint32_t grip_pulse_mask(const jc2_device_t *dev)
{
	if (dev->cfg.mode == JC2_MODE_RIGHT_ONLY)
		return JC2_BTN_R_SL | JC2_BTN_R_SR;
	if (dev->cfg.mode == JC2_MODE_LEFT_ONLY)
		return JC2_BTN_L_SL | JC2_BTN_L_SR;
	return JC2_BTN_L | JC2_BTN_R;
}

static void push_cb_evt(jc2_device_t *dev, uint8_t kind, uint8_t side,
			uint8_t aux, const uint8_t peer_addr[6])
{
	uint8_t used = (uint8_t)(dev->cb_evt_tail - dev->cb_evt_head);
	jc2_cb_evt_t *ev;

	if (used >= JC2_CB_EVT_Q_DEPTH)
		return;
	ev = &dev->cb_evt_q[dev->cb_evt_tail & (JC2_CB_EVT_Q_DEPTH - 1u)];
	ev->kind = kind;
	ev->side = side & 1u;
	ev->aux = aux;
	if (peer_addr)
		memcpy(ev->peer_addr, peer_addr, 6);
	else
		memset(ev->peer_addr, 0, 6);
	jc2_port_dmb();
	dev->cb_evt_tail++;
}

static void dispatch_rumble(jc2_device_t *dev, uint8_t side,
			    const uint8_t *data, uint16_t len)
{
	uint8_t s = side & 1u;

	if (dev->cfg.cbs.on_rumble_isr)
		dev->cfg.cbs.on_rumble_isr(s, data, len, dev->cfg.user_ctx);
	if (dev->cfg.cbs.on_rumble && len > 0u) {
		uint16_t n = (len <= JC2_RUMBLE_MAX_LEN) ? len :
							   JC2_RUMBLE_MAX_LEN;
		jc2_rumble_slot_t *mb = &dev->rumble_mb[s];

		memcpy(mb->data, data, n);
		mb->len = n;
		jc2_port_dmb();
		mb->pending = true;
	}
}

static void start_adv_for_side(jc2_device_t *dev, uint8_t side, uint32_t now_ms)
{
	uint8_t adv31[31];
	uint8_t srsp[20];
	uint8_t srsp_len = 0u;

	side &= 1u;
	dev->adv_switch_ms = now_ms;
	jc2_proto_build_adv(&dev->proto, side, dev->adv_use_reconnect[side],
			    adv31, srsp, &srsp_len);
	jc2_ll_start_adv(side, adv31, sizeof(adv31), srsp, srsp_len);
}

static bool ll_cb_get_ltk(uint8_t side, uint8_t out_ltk_le[16], void *ctx)
{
	jc2_device_t *dev = (jc2_device_t *)ctx;

	side &= 1u;
	if (!dev || !dev->proto.bond.has_ltk[side])
		return false;
	memcpy(out_ltk_le, dev->proto.bond.ltk[side], 16);
	return true;
}

static void ll_cb_on_connected(uint8_t side, const uint8_t peer_addr[6],
			       uint8_t peer_addr_type, void *ctx)
{
	jc2_device_t *dev = (jc2_device_t *)ctx;

	if (!dev)
		return;
	side &= 1u;
	memcpy(dev->proto.bond.host_addr, peer_addr, 6);
	dev->proto.bond.has_host = 1u;
	if (dev->proto.bond.has_ltk[side])
		dev->adv_use_reconnect[side] = true;
	dev->proto.side_stream_ready[side] = false;
	dev->cccd08_ms[side] = 0u;
	dev->conn_ms[side] = dev->last_poll_ms ? dev->last_poll_ms : 1u;
	dev->ready_ms[side] = 0u;
	memcpy(dev->peer_addr[side], peer_addr, 6);
	dev->peer_addr_type[side] = peer_addr_type;
	push_cb_evt(dev, JC2_CB_EVT_CONNECTED, side, peer_addr_type, peer_addr);
}

static void ll_cb_on_disconnected(uint8_t side, uint8_t reason, void *ctx)
{
	jc2_device_t *dev = (jc2_device_t *)ctx;

	if (!dev)
		return;
	side &= 1u;
	if (dev->proto.bond.has_ltk[side] && dev->proto.bond.has_host)
		dev->adv_use_reconnect[side] = true;
	dev->proto.side_stream_ready[side] = false;
	dev->cccd08_ms[side] = 0u;
	dev->both_ready_fired = false;
	dev->conn_ms[side] = 0u;
	dev->ready_ms[side] = 0u;
	push_cb_evt(dev, JC2_CB_EVT_DISCONNECTED, side, reason, NULL);
}

static void ll_cb_on_input_cccd(uint8_t side, uint16_t handle, bool enabled,
				void *ctx)
{
	jc2_device_t *dev = (jc2_device_t *)ctx;

	if (!dev || !enabled)
		return;
	side &= 1u;
	if (handle == JC2_HDL_INPUT0708_CCCD) {
		uint8_t rep[JC2_REPORT0708_SIZE];
		uint32_t now_us = jc2_ll_now_us();

		dev->proto.seq08[side] = 0u;
		jc2_proto_reset_motion(&dev->proto, side);
		dev->cccd08_ms[side] = dev->last_poll_ms ? dev->last_poll_ms :
							   1u;
		jc2_proto_build_report0708(&dev->proto, side,
					   dev->proto.seq08[side]++,
					   &dev->input, now_us, true, rep);
		(void)jc2_ll_notify(side, JC2_HDL_INPUT0708_VAL, rep,
				    sizeof(rep), true);
		dev->proto.side_stream_ready[side] = true;
	} else if (handle == JC2_HDL_INPUT05_CCCD) {
		dev->proto.side_stream_ready[side] = true;
	}
}

static void ll_cb_on_att_write(uint8_t side, uint16_t handle,
			       const uint8_t *data, uint16_t len, void *ctx)
{
	jc2_device_t *dev = (jc2_device_t *)ctx;
	uint8_t resp[JC2_CMD_RESP_MAX];
	jc2_cmd_effects_t eff;
	uint16_t rlen;

	if (!dev)
		return;
	side &= 1u;

	if (handle == JC2_HDL_VIB_VAL) {
		dispatch_rumble(dev, side, data, len);
		return;
	}

	/*
	 * HD Rumble + Command characteristic (0x0016) carries a 17-byte or
	 * 33-byte rumble frame ahead of an optional 8-byte Joy-Con 2 command.
	 */
	if (handle == JC2_HDL_RUMBLE_CMD_VAL) {
		uint16_t cmd_len = len;
		const uint8_t *cmd_ptr = jc2_proto_unwrap_cmd(data, &cmd_len);

		if (!cmd_ptr) {
			if (len > 0u)
				dispatch_rumble(dev, side, data, len);
			return;
		}
		if (cmd_ptr > data)
			dispatch_rumble(dev, side, data,
					(uint16_t)(cmd_ptr - data));
		data = cmd_ptr;
		len = cmd_len;
	} else if (handle != JC2_HDL_CMD_WRITE_VAL) {
		return;
	}

	rlen = jc2_proto_handle_cmd(&dev->proto, side, data, len, resp, &eff);
	if (rlen == 0u)
		return;

	dev->last_cmd_ms[side] = dev->last_poll_ms ? dev->last_poll_ms : 1u;
	if (eff.bond_updated) {
		dev->adv_use_reconnect[side] = true;
		dev->bond_dirty = true;
	}
	if (eff.led_updated)
		push_cb_evt(dev, JC2_CB_EVT_PLAYER_LED, side, eff.led_mask,
			    NULL);
	if (eff.stream_started && dev->grip_pulse_end_ms <= dev->last_poll_ms) {
		dev->grip_pulse_start_ms = dev->last_poll_ms + 1200u;
		dev->grip_pulse_end_ms = dev->last_poll_ms + 1900u;
	}
	(void)jc2_ll_notify(side, JC2_HDL_CMD_RESP_VAL, resp, rlen, false);
}

static jc2_input_t snapshot_effective_input(const jc2_device_t *dev,
					    bool include_accum)
{
	jc2_input_t in = dev->input;
	uint32_t now = dev->last_poll_ms;

	if (dev->grip_pulse_end_ms > dev->grip_pulse_start_ms &&
	    (int32_t)(now - dev->grip_pulse_start_ms) >= 0 &&
	    (int32_t)(dev->grip_pulse_end_ms - now) > 0) {
		if ((((now - dev->grip_pulse_start_ms) / 400u) & 1u) == 0u)
			in.buttons |= grip_pulse_mask(dev);
	}
	if (include_accum) {
		int32_t dx = dev->accum_mouse_dx;
		int32_t dy = dev->accum_mouse_dy;

		if (dx < -32768)
			dx = -32768;
		else if (dx > 32767)
			dx = 32767;
		if (dy < -32768)
			dy = -32768;
		else if (dy > 32767)
			dy = 32767;
		in.mouse_dx = (int16_t)dx;
		in.mouse_dy = (int16_t)dy;
	}
	return in;
}

static uint16_t ll_cb_build_live_input(uint8_t side, uint16_t *out_handle,
				       uint8_t *out_buf, uint16_t max_len,
				       void *ctx)
{
	jc2_device_t *dev = (jc2_device_t *)ctx;
	jc2_input_t in;

	if (!dev)
		return 0u;
	side &= 1u;
	if (!dev->proto.stream_enabled || !dev->proto.side_stream_ready[side])
		return 0u;

	if ((jc2_ll_get_cccd(side, JC2_HDL_INPUT0708_CCCD) & 0x01u) != 0u &&
	    max_len >= JC2_REPORT0708_SIZE) {
		*out_handle = JC2_HDL_INPUT0708_VAL;
		in = snapshot_effective_input(dev, side == JC2_SIDE_RIGHT);
		jc2_proto_build_report0708(&dev->proto, side,
					   dev->proto.seq08[side]++, &in,
					   jc2_ll_now_us(), true, out_buf);
		if (side == JC2_SIDE_RIGHT) {
			dev->accum_mouse_dx = 0;
			dev->accum_mouse_dy = 0;
		}
		return JC2_REPORT0708_SIZE;
	}

	if ((jc2_ll_get_cccd(side, JC2_HDL_INPUT05_CCCD) & 0x01u) != 0u &&
	    max_len >= JC2_REPORT05_SIZE) {
		*out_handle = JC2_HDL_INPUT05_VAL;
		in = snapshot_effective_input(dev, side == JC2_SIDE_RIGHT);
		jc2_proto_build_report05(side, ++dev->proto.seq05[side], &in,
					 out_buf);
		if (side == JC2_SIDE_RIGHT) {
			dev->accum_mouse_dx = 0;
			dev->accum_mouse_dy = 0;
		}
		return JC2_REPORT05_SIZE;
	}
	return 0u;
}

static void ll_cb_read_input_value(uint8_t side, uint16_t handle,
				   uint8_t *out_buf, uint16_t *inout_len,
				   void *ctx)
{
	jc2_device_t *dev = (jc2_device_t *)ctx;
	jc2_input_t in;

	if (!dev || !out_buf || !inout_len)
		return;
	side &= 1u;
	in = snapshot_effective_input(dev, side == JC2_SIDE_RIGHT);
	if (handle == JC2_HDL_INPUT0708_VAL &&
	    *inout_len >= JC2_REPORT0708_SIZE) {
		jc2_proto_build_report0708(&dev->proto, side,
					   dev->proto.seq08[side], &in,
					   jc2_ll_now_us(), true, out_buf);
		*inout_len = JC2_REPORT0708_SIZE;
	} else if (handle == JC2_HDL_INPUT05_VAL &&
		   *inout_len >= JC2_REPORT05_SIZE) {
		jc2_proto_build_report05(side, dev->proto.seq05[side], &in,
					 out_buf);
		*inout_len = JC2_REPORT05_SIZE;
	}
}

void jc2_config_default(jc2_config_t *cfg)
{
	if (!cfg)
		return;
	memset(cfg, 0, sizeof(*cfg));
	jc2_ll_read_device_mac(cfg->base_mac);
	jc2_identity_default(&cfg->identity);
}

void jc2_init(jc2_device_t *dev, const jc2_config_t *cfg)
{
	jc2_bond_t loaded;
	bool have_bond = false;

	if (!dev)
		return;
	memset(dev, 0, sizeof(*dev));
	if (cfg)
		dev->cfg = *cfg;
	else
		jc2_config_default(&dev->cfg);

	if (dev->cfg.mode != JC2_MODE_PAIR_LR &&
	    dev->cfg.mode != JC2_MODE_RIGHT_ONLY &&
	    dev->cfg.mode != JC2_MODE_LEFT_ONLY)
		dev->cfg.mode = JC2_MODE_PAIR_LR;

	jc2_proto_init(&dev->proto, dev->cfg.base_mac, &dev->cfg.identity);

	if (dev->cfg.cbs.load_bond) {
		memset(&loaded, 0, sizeof(loaded));
		if (dev->cfg.cbs.load_bond(&loaded, dev->cfg.user_ctx) &&
		    loaded.magic == JC2_BOND_MAGIC) {
			dev->proto.bond = loaded;
			have_bond = true;
		}
	}
	if (!have_bond
#if JC2_DEFAULT_MAC_EPOCH > 0u
	    || dev->proto.bond.mac_epoch < JC2_DEFAULT_MAC_EPOCH
#endif
	) {
		dev->proto.bond.magic = JC2_BOND_MAGIC;
		dev->proto.bond.mac_epoch = JC2_DEFAULT_MAC_EPOCH;
		dev->proto.bond.has_ltk[JC2_SIDE_RIGHT] = 0u;
		dev->proto.bond.has_ltk[JC2_SIDE_LEFT] = 0u;
		dev->proto.bond.has_host = 0u;
		dev->bond_dirty = true;
	}

	dev->adv_use_reconnect[JC2_SIDE_RIGHT] =
		(dev->proto.bond.has_host &&
		 dev->proto.bond.has_ltk[JC2_SIDE_RIGHT]);
	dev->adv_use_reconnect[JC2_SIDE_LEFT] =
		(dev->proto.bond.has_host &&
		 dev->proto.bond.has_ltk[JC2_SIDE_LEFT]);
	dev->fresh_sync_pair = !all_enabled_have_ltk(dev);

	jc2_proto_apply_mac_epoch(&dev->proto);
}

void jc2_start(jc2_device_t *dev, uint32_t now_ms)
{
	jc2_ll_callbacks_t ll_cbs;

	if (!dev || dev->started)
		return;
	dev->started = true;
	dev->last_poll_ms = now_ms ? now_ms : 1u;

	ll_cbs.get_ltk = ll_cb_get_ltk;
	ll_cbs.on_connected = ll_cb_on_connected;
	ll_cbs.on_disconnected = ll_cb_on_disconnected;
	ll_cbs.on_att_write = ll_cb_on_att_write;
	ll_cbs.on_input_cccd = ll_cb_on_input_cccd;
	ll_cbs.build_live_input = ll_cb_build_live_input;
	ll_cbs.read_input_value = ll_cb_read_input_value;
	ll_cbs.user_ctx = dev;

	jc2_ll_init(&ll_cbs);
	jc2_ll_set_mac(JC2_SIDE_RIGHT, dev->proto.mac[JC2_SIDE_RIGHT]);
	jc2_ll_set_mac(JC2_SIDE_LEFT, dev->proto.mac[JC2_SIDE_LEFT]);
	start_adv_for_side(dev, primary_adv_side(dev), dev->last_poll_ms);
}

void jc2_start_pairing(jc2_device_t *dev, bool clear_bonds, uint32_t now_ms)
{
	uint32_t bh;

	if (!dev)
		return;
	dev->last_poll_ms = now_ms ? now_ms : 1u;
	dev->game_ready = false;
	if (!dev->started)
		jc2_start(dev, now_ms);

	jc2_ll_stop_adv();
	if (jc2_ll_is_connected(JC2_SIDE_RIGHT))
		jc2_ll_disconnect(JC2_SIDE_RIGHT, 0x13u);
	if (jc2_ll_is_connected(JC2_SIDE_LEFT))
		jc2_ll_disconnect(JC2_SIDE_LEFT, 0x13u);

	bh = jc2_ll_lock_bottom_half();
	dev->conn_ms[0] = 0u;
	dev->conn_ms[1] = 0u;
	dev->ready_ms[0] = 0u;
	dev->ready_ms[1] = 0u;
	dev->proto.side_stream_ready[0] = false;
	dev->proto.side_stream_ready[1] = false;
	dev->both_ready_fired = false;

	if (clear_bonds) {
		dev->fresh_sync_pair = true;
		dev->adv_use_reconnect[0] = false;
		dev->adv_use_reconnect[1] = false;
#if JC2_DEFAULT_MAC_EPOCH > 0u
		if (dev->proto.bond.mac_epoch < JC2_DEFAULT_MAC_EPOCH)
			dev->proto.bond.mac_epoch = JC2_DEFAULT_MAC_EPOCH;
		else
			dev->proto.bond.mac_epoch++;
		if (dev->proto.bond.mac_epoch < JC2_DEFAULT_MAC_EPOCH)
			dev->proto.bond.mac_epoch = JC2_DEFAULT_MAC_EPOCH;
#else
		dev->proto.bond.mac_epoch++;
#endif
		dev->proto.bond.has_ltk[0] = 0u;
		dev->proto.bond.has_ltk[1] = 0u;
		dev->proto.bond.has_host = 0u;
		jc2_proto_apply_mac_epoch(&dev->proto);
		jc2_ll_set_mac(JC2_SIDE_RIGHT, dev->proto.mac[JC2_SIDE_RIGHT]);
		jc2_ll_set_mac(JC2_SIDE_LEFT, dev->proto.mac[JC2_SIDE_LEFT]);
		dev->bond_dirty = true;
	} else {
		dev->fresh_sync_pair = !all_enabled_have_ltk(dev);
		dev->adv_use_reconnect[0] = (dev->proto.bond.has_ltk[0] &&
					     dev->proto.bond.has_host);
		dev->adv_use_reconnect[1] = (dev->proto.bond.has_ltk[1] &&
					     dev->proto.bond.has_host);
	}
	jc2_ll_unlock_bottom_half(bh);
	start_adv_for_side(dev, primary_adv_side(dev), dev->last_poll_ms);
}

void jc2_pulse_grip_lr(jc2_device_t *dev, uint32_t now_ms, uint32_t duration_ms)
{
	if (!dev)
		return;
	dev->grip_pulse_start_ms = now_ms;
	dev->grip_pulse_end_ms = now_ms + (duration_ms ? duration_ms : 2400u);
}

void jc2_set_input(jc2_device_t *dev, const jc2_input_t *in)
{
	uint32_t bh;

	if (!dev || !in)
		return;
	bh = jc2_ll_lock_bottom_half();
	dev->input = *in;
	if (in->mouse_on_surface) {
		dev->accum_mouse_dx += (int32_t)in->mouse_dx;
		dev->accum_mouse_dy += (int32_t)in->mouse_dy;
	} else {
		dev->accum_mouse_dx = 0;
		dev->accum_mouse_dy = 0;
	}
	jc2_ll_unlock_bottom_half(bh);
}

static void update_side_ready_timeout(jc2_device_t *dev, uint8_t side,
				      bool *conn_io, bool in08, uint32_t now)
{
	if (in08) {
		if (dev->ready_ms[side] == 0u)
			dev->ready_ms[side] = now;
		return;
	}
	dev->ready_ms[side] = 0u;
	if (*conn_io && dev->conn_ms[side] != 0u &&
	    (jc2_ll_get_cccd(side, JC2_HDL_INPUT05_CCCD) & 0x01u) == 0u &&
	    (uint32_t)(now - dev->conn_ms[side]) >= 25000u &&
	    (uint32_t)(now - dev->last_cmd_ms[side]) >= 10000u) {
		jc2_ll_disconnect(side, 0x13u);
		*conn_io = false;
	}
}

static void update_adv_fsm(jc2_device_t *dev, uint32_t now)
{
	bool r_en = is_side_enabled(dev, JC2_SIDE_RIGHT);
	bool l_en = is_side_enabled(dev, JC2_SIDE_LEFT);
	bool r_conn = r_en && jc2_ll_is_connected(JC2_SIDE_RIGHT);
	bool l_conn = l_en && jc2_ll_is_connected(JC2_SIDE_LEFT);
	bool r_in08 =
		r_conn && dev->proto.side_stream_ready[JC2_SIDE_RIGHT] &&
		((jc2_ll_get_cccd(JC2_SIDE_RIGHT, JC2_HDL_INPUT0708_CCCD) &
		  0x01u) != 0u);
	bool l_in08 = l_conn && dev->proto.side_stream_ready[JC2_SIDE_LEFT] &&
		      ((jc2_ll_get_cccd(JC2_SIDE_LEFT, JC2_HDL_INPUT0708_CCCD) &
			0x01u) != 0u);
	bool r_ready;
	bool l_ready;
	bool r_settled;
	bool l_settled;
	bool both_bonded;
	uint32_t alt_ms;

	if (r_en)
		update_side_ready_timeout(dev, JC2_SIDE_RIGHT, &r_conn, r_in08,
					  now);
	if (l_en)
		update_side_ready_timeout(dev, JC2_SIDE_LEFT, &l_conn, l_in08,
					  now);

	r_ready = !r_en || r_in08;
	l_ready = !l_en || l_in08;
	if (r_ready && l_ready) {
		if (!dev->both_ready_fired) {
			dev->both_ready_fired = true;
			if (dev->fresh_sync_pair) {
				dev->fresh_sync_pair = false;
				dev->grip_pulse_start_ms = now + 800u;
				dev->grip_pulse_end_ms = now + 5600u;
			}
		}
		r_settled = !r_en ||
			    (dev->ready_ms[0] != 0u &&
			     (uint32_t)(now - dev->ready_ms[0]) >= 500u);
		l_settled = !l_en ||
			    (dev->ready_ms[1] != 0u &&
			     (uint32_t)(now - dev->ready_ms[1]) >= 500u);
		if (!dev->game_ready && r_settled && l_settled) {
			dev->game_ready = true;
			dev->game_ready_ms = now;
		}
	} else {
		dev->both_ready_fired = false;
		dev->game_ready = false;
	}

	if ((!r_en || r_conn) && (!l_en || l_conn)) {
		if (jc2_ll_is_advertising())
			jc2_ll_stop_adv();
		return;
	}

	if (!r_en || !l_en) {
		uint8_t s = r_en ? JC2_SIDE_RIGHT : JC2_SIDE_LEFT;

		if (dev->proto.bond.has_host && dev->proto.bond.has_ltk[s])
			dev->adv_use_reconnect[s] = true;
		if (!jc2_ll_is_advertising() || jc2_ll_adv_side() != s)
			start_adv_for_side(dev, s, now);
		return;
	}

	if (r_conn && !l_conn) {
		uint32_t settle_ms = dev->proto.bond.has_ltk[1] ? 100u : 800u;

		if (r_in08 && dev->ready_ms[0] != 0u &&
		    (uint32_t)(now - dev->ready_ms[0]) >= settle_ms) {
			if (dev->proto.bond.has_ltk[1] &&
			    dev->proto.bond.has_host)
				dev->adv_use_reconnect[1] = true;
			if (!jc2_ll_is_advertising() ||
			    jc2_ll_adv_side() != JC2_SIDE_LEFT)
				start_adv_for_side(dev, JC2_SIDE_LEFT, now);
		} else if (jc2_ll_is_advertising()) {
			jc2_ll_stop_adv();
		}
		return;
	}

	if (!r_conn && l_conn) {
		uint32_t settle_ms = dev->proto.bond.has_ltk[0] ? 100u : 800u;

		if (l_in08 && dev->ready_ms[1] != 0u &&
		    (uint32_t)(now - dev->ready_ms[1]) >= settle_ms) {
			if (dev->proto.bond.has_ltk[0] &&
			    dev->proto.bond.has_host)
				dev->adv_use_reconnect[0] = true;
			if (!jc2_ll_is_advertising() ||
			    jc2_ll_adv_side() != JC2_SIDE_RIGHT)
				start_adv_for_side(dev, JC2_SIDE_RIGHT, now);
		} else if (jc2_ll_is_advertising()) {
			jc2_ll_stop_adv();
		}
		return;
	}

	both_bonded = dev->proto.bond.has_host && dev->proto.bond.has_ltk[0] &&
		      dev->proto.bond.has_ltk[1];
	if (dev->proto.bond.has_host) {
		if (dev->proto.bond.has_ltk[0])
			dev->adv_use_reconnect[0] = true;
		if (dev->proto.bond.has_ltk[1])
			dev->adv_use_reconnect[1] = true;
	}
	alt_ms = both_bonded ? JC2_ADV_ALT_RECONN_MS : JC2_ADV_ALT_PAIR_MS;
	if (!jc2_ll_is_advertising()) {
		start_adv_for_side(dev, JC2_SIDE_LEFT, now);
	} else if ((uint32_t)(now - dev->adv_switch_ms) >= alt_ms) {
		start_adv_for_side(dev, (uint8_t)(jc2_ll_adv_side() ^ 1u), now);
	}
}

static void drain_pending_callbacks(jc2_device_t *dev)
{
	uint8_t s;

	while (dev->cb_evt_head != dev->cb_evt_tail) {
		jc2_cb_evt_t ev = dev->cb_evt_q[dev->cb_evt_head &
						(JC2_CB_EVT_Q_DEPTH - 1u)];

		jc2_port_dmb();
		dev->cb_evt_head++;
		if (ev.kind == JC2_CB_EVT_CONNECTED &&
		    dev->cfg.cbs.on_connected) {
			dev->cfg.cbs.on_connected(ev.side, ev.peer_addr, ev.aux,
						  dev->cfg.user_ctx);
		} else if (ev.kind == JC2_CB_EVT_DISCONNECTED &&
			   dev->cfg.cbs.on_disconnected) {
			dev->cfg.cbs.on_disconnected(ev.side, ev.aux,
						     dev->cfg.user_ctx);
		} else if (ev.kind == JC2_CB_EVT_PLAYER_LED &&
			   dev->cfg.cbs.on_player_led) {
			dev->cfg.cbs.on_player_led(ev.side, ev.aux,
						   dev->cfg.user_ctx);
		}
	}

	if (!dev->cfg.cbs.on_rumble)
		return;
	for (s = 0u; s < JC2_MAX_LINKS; s++) {
		uint8_t snap[JC2_RUMBLE_MAX_LEN];
		uint16_t slen = 0u;
		bool have = false;

		if (dev->rumble_mb[s].pending) {
			uint32_t bh = jc2_ll_lock_bottom_half();

			if (dev->rumble_mb[s].pending) {
				slen = dev->rumble_mb[s].len;
				if (slen > JC2_RUMBLE_MAX_LEN)
					slen = JC2_RUMBLE_MAX_LEN;
				memcpy(snap, dev->rumble_mb[s].data, slen);
				dev->rumble_mb[s].pending = false;
				have = true;
			}
			jc2_ll_unlock_bottom_half(bh);
		}
		if (have)
			dev->cfg.cbs.on_rumble(s, snap, slen,
					       dev->cfg.user_ctx);
	}
}

static void flush_pending_bond(jc2_device_t *dev, uint32_t now)
{
	bool any_conn;
	bool steady_stream;
	uint32_t min_gap;

	if (!dev->bond_dirty || !dev->cfg.cbs.save_bond)
		return;

	any_conn = jc2_ll_is_connected(JC2_SIDE_RIGHT) ||
		   jc2_ll_is_connected(JC2_SIDE_LEFT);
	steady_stream = dev->game_ready && dev->game_ready_ms != 0u &&
			(uint32_t)(now - dev->game_ready_ms) >= 2000u;
	if (any_conn && !steady_stream)
		return;

	/*
	 * While connected at dual 5.0 ms CI, evenly staggered links leave ~1.8
	 * to 2.0 ms idle gaps; 1500 us accommodates append-only bond record
	 * writes (~451 us on nRF52840) without requiring a 3200 us gap.
	 */
	min_gap = any_conn ? JC2_BOND_FLUSH_GAP_US : 0u;
	if (jc2_ll_acquire_gap(min_gap)) {
		jc2_bond_t snap;
		uint32_t bh;
		bool saved;

		bh = jc2_ll_lock_bottom_half();
		snap = dev->proto.bond;
		dev->bond_dirty = false;
		jc2_ll_unlock_bottom_half(bh);

		saved = dev->cfg.cbs.save_bond(&snap, dev->cfg.user_ctx);
		jc2_ll_release_gap();
		if (!saved) {
			bh = jc2_ll_lock_bottom_half();
			dev->bond_dirty = true;
			jc2_ll_unlock_bottom_half(bh);
		}
	}
}

void jc2_poll(jc2_device_t *dev, uint32_t now_ms)
{
	uint32_t now;

	if (!dev || !dev->started)
		return;
	now = now_ms ? now_ms : 1u;
	dev->last_poll_ms = now;
	jc2_ll_poll();
	drain_pending_callbacks(dev);
	update_adv_fsm(dev, now);
	flush_pending_bond(dev, now);
}

bool jc2_is_ready(const jc2_device_t *dev)
{
	return dev ? dev->game_ready : false;
}

bool jc2_is_started(const jc2_device_t *dev)
{
	return dev ? dev->started : false;
}

void jc2_get_status(const jc2_device_t *dev, jc2_status_t *out)
{
	uint8_t s;

	if (!out)
		return;
	memset(out, 0, sizeof(*out));
	if (!dev)
		return;

	out->mode = dev->cfg.mode;
	out->started = dev->started;
	out->advertising = dev->started && jc2_ll_is_advertising();
	out->game_ready = dev->game_ready;
	out->adv_side = jc2_ll_adv_side();
	out->mac_epoch = dev->proto.bond.mac_epoch;
	out->last_cmd_id = dev->proto.last_cmd_id;
	out->last_sub_id = dev->proto.last_sub_id;
	out->cmd_count = dev->proto.cmd_count;

	for (s = 0u; s < JC2_MAX_LINKS; s++) {
		bool conn = dev->started && jc2_ll_is_connected(s);
		bool c08 = conn &&
			   ((jc2_ll_get_cccd(s, JC2_HDL_INPUT0708_CCCD) &
			     0x01u) != 0u);

		out->connected[s] = conn;
		out->secured[s] = dev->started && jc2_ll_is_secured(s);
		out->streaming[s] = conn && dev->proto.side_stream_ready[s] &&
				    c08;
		out->has_ltk[s] = dev->proto.bond.has_ltk[s] != 0u;
		out->adv_reconnect[s] = dev->adv_use_reconnect[s];
		memcpy(out->mac[s], dev->proto.mac[s], 6);
		memcpy(out->peer_addr[s], dev->peer_addr[s], 6);
		out->peer_addr_type[s] = dev->peer_addr_type[s];
		out->mtu[s] = dev->started ? jc2_ll_get_mtu(s) : 0u;
		out->tx_queue_depth[s] =
			dev->started ? jc2_ll_tx_queue_depth(s) : 0u;
		out->link[s] = *jc2_ll_get_diag(s);
	}
}
