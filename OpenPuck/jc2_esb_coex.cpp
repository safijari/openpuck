// jc2_esb_coex.cpp -- 2.4 GHz Steam Controller ESB gap scheduler running
// inside libjoycon2's BLE 5.0 radio idle windows.
#include "jc2_esb_coex.h"
#include "bonds.h"
#include "config.h"
#include "haptics.h"
#include "rf_link.h"
#include "src/libjoycon2/jc2_ll.h"
#include "usb_mount.h"

#include <string.h>

#if !defined(OPK_HOST_TEST)
#include "radio.h"
#include "triton.h"
#include <Arduino.h>

struct Jc2EsbCoex {
	uint8_t cycle;
	uint8_t live_rr;
	uint8_t acq_rr;
	uint8_t poll_pid[NSLOT];
	volatile bool link_up;
	bool was_up[NSLOT];
	unsigned long last_poll_us;
	unsigned long last_sess_e1_ms[NSLOT];
	unsigned long last_disc_e1_ms;
	unsigned long win_ms;
	unsigned long off_ms[NSLOT];
	bool off_fired[NSLOT];
	uint8_t prev_rep_rid[NSLOT];
	uint8_t prev_rep_seq[NSLOT];
	bool has_prev_rep_seq[NSLOT];
	uint32_t pad_sample_seq[NSLOT];
	uint32_t win_polls;
	uint32_t win_f1;
	uint32_t win_new;
	uint32_t win_crc;
	uint32_t win_to;
	uint32_t win_relay;
	uint32_t beacons;
	uint32_t sess_e1;
	uint32_t relays;
	uint16_t poll_hz;
	uint16_t f1_hz;
	uint16_t new_hz;
	uint16_t crc_hz;
	uint16_t to_hz;
};

static Jc2EsbCoex s_esb = {};

void jc2EsbCoexOnBleStart(bool was_link_up)
{
	if (!was_link_up)
		return;
	unsigned long now = millis();
	for (int s = 0; s < NSLOT; s++) {
		if (g_slot[s].used && g_connReplyMs[s] != 0)
			g_connReplyMs[s] = now;
	}
	s_esb.link_up = true;
}

static void tx_esb_beacon(int slot, bool discovery)
{
	if (slot < 0 || slot >= NSLOT || !g_slot[slot].used)
		return;
	const uint8_t *rec = g_slot[slot].rec;
	memset(rftx, 0, 24);
	rftx[0] = 0x12u;
	rftx[1] = (uint8_t)((g_pid++ & 3u) << 1);
	rftx[2] = 0xE1u;
	memcpy(rftx + 3, rec + 0, 4);
	memcpy(rftx + 7, rec + 4, 4);
	rftx[11] = g_sessCh;
	memcpy(rftx + 15, g_sessBase[slot], 4);
	rftx[19] = g_sessPrefix[slot];
	rfConfig(discovery ? 2u : g_sessCh);
	rfSetAddr(discovery ? g_rfBase : g_sessBase[slot],
		  discovery ? g_rfPrefix : g_sessPrefix[slot]);
	NRF_RADIO->PACKETPTR = (uint32_t)rftx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	if (discovery)
		s_esb.beacons++;
	else
		s_esb.sess_e1++;
}

static bool bare_poll_once(int slot, uint32_t max_wait_us,
			   const uint8_t *tx_pay, uint8_t tx_len)
{
	rfConfig(g_sessCh);
	rfSetAddr(g_sessBase[slot], g_sessPrefix[slot]);
	uint8_t s1 = (uint8_t)(((s_esb.poll_pid[slot]++ & 3u) << 1) | 1u);
	if (tx_pay && tx_len >= 1u && tx_len <= 68u) {
		rftx[0] = tx_len;
		rftx[1] = s1;
		memcpy(rftx + 2, tx_pay, tx_len);
	} else {
		memset(rftx, 0, 8);
		rftx[0] = 0x01u;
		rftx[1] = s1;
		rftx[2] = 0xE3u;
	}

	NRF_RADIO->PACKETPTR = (uint32_t)rftx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;

	rfrx[0] = 0;
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
			    RADIO_SHORTS_DISABLED_RSSISTOP_Msk;
	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->EVENTS_END = 0;
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	uint32_t c0 = DWT->CYCCNT;
	NRF_RADIO->TASKS_RXEN = 1;

	// Abort early if no address arrives within 380 us so absent polls stay
	// short, while live 55 B 0x42 / 66 B 0x43 frames get up to max_wait_us.
	bool saw_addr = false;
	uint32_t addr_abort_us = (max_wait_us < 380u) ? max_wait_us : 380u;
	while (!NRF_RADIO->EVENTS_END) {
		uint32_t us = (uint32_t)(DWT->CYCCNT - c0) >> 6;
		if (!saw_addr && NRF_RADIO->EVENTS_ADDRESS)
			saw_addr = true;
		if (!saw_addr && us >= addr_abort_us)
			break;
		if (us >= max_wait_us)
			break;
	}
	bool ok = false;
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		uint8_t crc = (uint8_t)(NRF_RADIO->CRCSTATUS & 1u);
		if (crc && rfrx[0] >= 1u && rfrx[0] <= 96u &&
		    rfrx[2] >= 0xF0u) {
			ok = true;
			uint8_t rs = (uint8_t)(NRF_RADIO->RSSISAMPLE & 0x7Fu);
			if (rs) {
				g_linkRssi[slot] =
					g_linkRssi[slot] ?
						(uint8_t)((g_linkRssi[slot] *
								   7u +
							   rs + 4u) /
							  8u) :
						rs;
			}
			if (rfrx[2] != 0xF2u) {
				unsigned long now_ms = millis();
				if (g_connReplyMs[slot] == 0u ||
				    (uint32_t)(now_ms - g_connReplyMs[slot]) >
					    1500u)
					hapticOnReconnect(slot);
				g_connReplyMs[slot] = now_ms;
			}
		} else {
			s_esb.win_crc++;
		}
	} else {
		s_esb.win_to++;
	}
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	return ok;
}

static void decode_esb_reply(int slot, const uint8_t *buf,
			     void (*on_slot_drop)(int slot))
{
	g_curSlot = slot;
	if (buf[2] < 0xF0u)
		return;
	unsigned long now_ms = millis();
	if (buf[2] == 0xF2u) {
		memset(&g_in[slot], 0, sizeof(g_in[slot]));
		tritonTimestamp47Reset((uint8_t)slot);
		if (on_slot_drop)
			on_slot_drop(slot);
		g_connReplyMs[slot] = 0u;
		s_esb.has_prev_rep_seq[slot] = false;
		bool other_up = false;
		for (int i = 0; i < NSLOT; i++) {
			if (i != slot && g_slot[i].used &&
			    g_connReplyMs[i] != 0u &&
			    (unsigned long)(now_ms - g_connReplyMs[i]) < 300u)
				other_up = true;
		}
		s_esb.link_up = other_up;
		if (!other_up)
			g_connCooldown = now_ms;
		return;
	}
	g_connReplyMs[slot] = now_ms;
	s_esb.link_up = true;
	if (buf[2] != 0xF1u)
		return;
	g_connF1++;

	int idx = 3;
	int end = (int)buf[0] + 2;
	while (idx + 1 < end) {
		uint8_t tlen = buf[idx];
		uint8_t ttype = buf[idx + 1];
		if (tlen == 0)
			break;
		if (ttype == 6 && tlen >= 3 && buf[idx + 2] == 0x43 &&
		    idx + 2 + (int)tlen <= 100) {
			g_batteryState[slot] = buf[idx + 3];
			g_battery[slot] = buf[idx + 4];
		} else if (ttype == 6 &&
			   (((tlen >= 28) &&
			     (buf[idx + 2] == 0x45 || buf[idx + 2] == 0x42)) ||
			    (tlen == 46 &&
			     buf[idx + 2] ==
				     OPK_TRITON_REPORT_STATE_TIMESTAMP)) &&
			   idx + 2 + (int)tlen <= 100) {
			const uint8_t *rep = &buf[idx + 2];
			bool fresh = !s_esb.has_prev_rep_seq[slot] ||
				     rep[0] != s_esb.prev_rep_rid[slot] ||
				     rep[1] != s_esb.prev_rep_seq[slot];
			if (fresh) {
				s_esb.win_new++;
				s_esb.pad_sample_seq[slot]++;
			}
			s_esb.prev_rep_rid[slot] = rep[0];
			s_esb.prev_rep_seq[slot] = rep[1];
			s_esb.has_prev_rep_seq[slot] = true;

			uint32_t bb = btnsOf(rep);
			if ((bb & (TB_STEAM | TB_Y)) == (TB_STEAM | TB_Y)) {
				if (s_esb.off_ms[slot] == 0u)
					s_esb.off_ms[slot] = now_ms;
				else if (!s_esb.off_fired[slot] &&
					 (unsigned long)(now_ms -
							 s_esb.off_ms[slot]) >=
						 2000u) {
					s_esb.off_fired[slot] = true;
					hapticSendShutdown((uint8_t)slot);
				}
				bb &= ~(uint32_t)(TB_STEAM | TB_Y);
			} else {
				s_esb.off_ms[slot] = 0u;
				s_esb.off_fired[slot] = false;
			}
			g_in[slot].buttons = bb;
			g_in[slot].lx = (int16_t)s16off(rep, 8);
			g_in[slot].ly = (int16_t)s16off(rep, 10);
			g_in[slot].rx = (int16_t)s16off(rep, 12);
			g_in[slot].ry = (int16_t)s16off(rep, 14);
			g_in[slot].lt = trigU8(u16off(rep, 4));
			g_in[slot].rt = trigU8(u16off(rep, 6));
			if (rep[0] == OPK_TRITON_REPORT_STATE_TIMESTAMP) {
				g_in[slot].lpx = (int16_t)s16off(rep, 18);
				g_in[slot].lpy = (int16_t)s16off(rep, 20);
				g_in[slot].rpx = (int16_t)s16off(rep, 24);
				g_in[slot].rpy = (int16_t)s16off(rep, 26);
			} else {
				g_in[slot].lpx = (int16_t)s16off(rep, 16);
				g_in[slot].lpy = (int16_t)s16off(rep, 18);
				g_in[slot].rpx = (int16_t)s16off(rep, 22);
				g_in[slot].rpy = (int16_t)s16off(rep, 24);
			}
			if (tlen >= 46 && idx + 2 + 46 <= end) {
				if (rep[0] ==
				    OPK_TRITON_REPORT_STATE_TIMESTAMP) {
					uint16_t tick47 =
						(uint16_t)rep[32] |
						((uint16_t)rep[33] << 8);
					g_in[slot].imuTimestampUs =
						tritonTimestamp47Us(slot,
								    tick47);
				} else {
					tritonTimestamp47Reset(slot);
					g_in[slot].imuTimestampUs =
						(uint32_t)rep[30] |
						((uint32_t)rep[31] << 8) |
						((uint32_t)rep[32] << 16) |
						((uint32_t)rep[33] << 24);
				}
				imuFrom45(rep, &g_in[slot].ax, &g_in[slot].ay,
					  &g_in[slot].az, &g_in[slot].gx,
					  &g_in[slot].gy, &g_in[slot].gz);
			}
			if ((g_in[slot].buttons & CHORD_BACK4) == CHORD_BACK4 &&
			    (g_in[slot].buttons & TB_A)) {
				modeSwitchReboot(MODE_STEAM);
			}
		}
		idx += (int)tlen + 2;
	}
}

static void run_poll_and_record(int slot, uint32_t max_wait_us,
				const uint8_t *tx_pay, uint8_t tx_len,
				void (*on_slot_drop)(int slot))
{
	s_esb.win_polls++;
	if (bare_poll_once(slot, max_wait_us, tx_pay, tx_len)) {
		if (rfrx[2] == 0xF1u)
			s_esb.win_f1++;
		decode_esb_reply(slot, rfrx, on_slot_drop);
	}
}

static int pick_esb_slot(unsigned long now_ms)
{
	if (s_esb.link_up) {
		for (uint8_t n = 0u; n < NSLOT; n++) {
			int k = (int)((s_esb.live_rr + n) % NSLOT);
			if (g_slot[k].used && g_connReplyMs[k] != 0u &&
			    (unsigned long)(now_ms - g_connReplyMs[k]) < 300u) {
				s_esb.live_rr = (uint8_t)((k + 1) % NSLOT);
				return k;
			}
		}
	}
	for (uint8_t n = 0u; n < NSLOT; n++) {
		int k = (int)((s_esb.acq_rr + n) % NSLOT);
		if (g_slot[k].used) {
			s_esb.acq_rr = (uint8_t)((k + 1) % NSLOT);
			return k;
		}
	}
	return -1;
}

static void run_slot_in_gap(void (*on_slot_drop)(int slot))
{
	unsigned long now_ms = millis();
	int slot = pick_esb_slot(now_ms);
	if (slot < 0)
		return;

	uint8_t cyc = s_esb.cycle++;

	if (!s_esb.link_up) {
		bool disc = ((cyc & 1u) == 0u);
		tx_esb_beacon(slot, disc);
		if (!disc)
			s_esb.last_sess_e1_ms[slot] = now_ms;
		if (jc2_ll_remaining_gap_us() >= 1000u)
			run_poll_and_record(slot, 680u, NULL, 0u, on_slot_drop);
	} else {
		rfConnQueueHapticRelay();
		uint8_t relay_buf[8 + RELAY_MAXP];
		uint8_t relay_len = 0u;
		bool have_relay = false;
		if (jc2_ll_remaining_gap_us() >= 1450u &&
		    hapticPopRelayPayload(slot, relay_buf, &relay_len,
					  sizeof(relay_buf))) {
			have_relay = true;
			s_esb.relays++;
			s_esb.win_relay++;
		}
		run_poll_and_record(slot, 820u, have_relay ? relay_buf : NULL,
				    relay_len, on_slot_drop);

		// Keep the 25 ms session E1 alive so SC2 holds ~200 Hz replies.
		if (g_e1keepalive && s_esb.link_up &&
		    (unsigned long)(now_ms - s_esb.last_sess_e1_ms[slot]) >=
			    25u &&
		    jc2_ll_remaining_gap_us() >= 450u) {
			s_esb.last_sess_e1_ms[slot] = now_ms;
			tx_esb_beacon(slot, false);
		} else if ((unsigned long)(now_ms - s_esb.last_disc_e1_ms) >=
				   250u &&
			   jc2_ll_remaining_gap_us() >= 450u) {
			for (uint8_t n = 0u; n < NSLOT; n++) {
				int k = (int)((s_esb.acq_rr + n) % NSLOT);
				bool up = g_slot[k].used &&
					  g_connReplyMs[k] != 0u &&
					  (unsigned long)(now_ms -
							  g_connReplyMs[k]) <
						  300u;
				if (g_slot[k].used && !up) {
					s_esb.acq_rr =
						(uint8_t)((k + 1) % NSLOT);
					s_esb.last_disc_e1_ms = now_ms;
					tx_esb_beacon(k, true);
					break;
				}
			}
		}
	}
}

void jc2EsbCoexStep(bool game_ready, void (*on_slot_drop)(int slot))
{
	unsigned long now_ms = millis();
	if (s_esb.win_ms == 0u)
		s_esb.win_ms = now_ms;
	unsigned long win_dt = now_ms - s_esb.win_ms;
	if (win_dt >= 1000u) {
		s_esb.poll_hz = (uint16_t)((s_esb.win_polls * 1000UL) / win_dt);
		s_esb.f1_hz = (uint16_t)((s_esb.win_f1 * 1000UL) / win_dt);
		s_esb.new_hz = (uint16_t)((s_esb.win_new * 1000UL) / win_dt);
		s_esb.crc_hz = (uint16_t)((s_esb.win_crc * 1000UL) / win_dt);
		s_esb.to_hz = (uint16_t)((s_esb.win_to * 1000UL) / win_dt);
		g_pollsps = s_esb.poll_hz;
		g_f1ps = s_esb.f1_hz;
		g_newps = s_esb.new_hz;
		g_crcps = s_esb.crc_hz;
		g_norxps = s_esb.to_hz;
		g_relayps = (uint16_t)((s_esb.win_relay * 1000UL) / win_dt);
		s_esb.win_polls = 0u;
		s_esb.win_f1 = 0u;
		s_esb.win_new = 0u;
		s_esb.win_crc = 0u;
		s_esb.win_to = 0u;
		s_esb.win_relay = 0u;
		s_esb.win_ms = now_ms;
	}

	for (int s = 0; s < NSLOT; s++) {
		bool up = g_slot[s].used && g_connReplyMs[s] != 0 &&
			  (unsigned long)(now_ms - g_connReplyMs[s]) < 300u;
		if (s_esb.was_up[s] && !up) {
			memset(&g_in[s], 0, sizeof(g_in[s]));
			tritonTimestamp47Reset((uint8_t)s);
			if (on_slot_drop)
				on_slot_drop(s);
			s_esb.has_prev_rep_seq[s] = false;
		}
		s_esb.was_up[s] = up;
	}
	s_esb.link_up = anySlotLinkUp();
	if (bondedSlotCount() == 0)
		return;
	if (g_connCooldown != 0u &&
	    (unsigned long)(now_ms - g_connCooldown) < 2500u)
		return;

	uint32_t now_us = jc2_ll_now_us();
	uint32_t min_interval_us =
		!game_ready ? 8000u : (s_esb.link_up ? 3700u : 4000u);
	if ((uint32_t)(now_us - s_esb.last_poll_us) < min_interval_us)
		return;

	if (jc2_ll_acquire_gap(1350u)) {
		s_esb.last_poll_us = now_us;
		run_slot_in_gap(on_slot_drop);
		jc2_ll_release_gap();
	}
}

uint32_t jc2EsbCoexPadSampleSeq(uint8_t slot)
{
	return (slot < NSLOT) ? s_esb.pad_sample_seq[slot] : 0u;
}

void jc2EsbCoexPrintStatus(void)
{
	int sc_slot = (g_curSlot >= 0 && g_curSlot < NSLOT &&
		       g_slot[g_curSlot].used) ?
			      g_curSlot :
			      -1;
	if (sc_slot < 0) {
		for (int s = 0; s < NSLOT; s++) {
			if (g_slot[s].used) {
				sc_slot = s;
				break;
			}
		}
	}
	int sc_rssi = (sc_slot >= 0 && g_linkRssi[sc_slot]) ?
			      -(int)g_linkRssi[sc_slot] :
			      0;
	uint8_t sc_batt = (sc_slot >= 0) ? g_battery[sc_slot] : 0u;
	uint32_t sc_btn =
		(sc_slot >= 0 && s_esb.link_up) ? g_in[sc_slot].buttons : 0u;
	Serial.printf(
		"# JC2_SC2 link=%u bonded=%u slot=%d ch=%u rssi=%d batt=%u "
		"pollHz=%u f1Hz=%u newHz=%u errHz=%u/%u e1=%lu/%lu rly=%lu "
		"btn=0x%08lX\n",
		s_esb.link_up ? 1u : 0u, (uint8_t)bondedSlotCount(), sc_slot,
		g_sessCh, sc_rssi, sc_batt, s_esb.poll_hz, s_esb.f1_hz,
		s_esb.new_hz, s_esb.crc_hz, s_esb.to_hz,
		(unsigned long)s_esb.beacons, (unsigned long)s_esb.sess_e1,
		(unsigned long)s_esb.relays, (unsigned long)sc_btn);
}

#else

void jc2EsbCoexOnBleStart(bool was_link_up)
{
	(void)was_link_up;
}

void jc2EsbCoexStep(bool game_ready, void (*on_slot_drop)(int slot))
{
	(void)game_ready;
	(void)on_slot_drop;
}

uint32_t jc2EsbCoexPadSampleSeq(uint8_t slot)
{
	(void)slot;
	return 0u;
}

void jc2EsbCoexPrintStatus(void)
{
}

#endif
