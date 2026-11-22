/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#include "jc2_ll_nrf52.h"
#include "jc2_crypto.h"
#include "jc2_port.h"

#include <string.h>

#if defined(NRF52840_XXAA) && !defined(JC2_HOST_TEST)

#if defined(__ZEPHYR__)
#include <nrfx.h>
#else
#include <nrf.h>
#endif

enum jc2_radio_phase {
	RPHASE_IDLE = 0u,
	RPHASE_GAP_BUSY,
	RPHASE_ADV_TX,
	RPHASE_ADV_RX,
	RPHASE_ADV_SCAN_RSP_TX,
	RPHASE_CONN_RX,
	RPHASE_CONN_TX,
};

typedef struct {
	uint8_t side;
	uint8_t gen;
	uint8_t hdr;
	uint8_t len;
	uint8_t encrypted;
	uint64_t ccm_cnt;
	uint8_t data[255];
} jc2_rx_ring_item_t;

typedef struct {
	uint8_t kind; /* 1 = connected, 2 = disconnected */
	uint8_t side;
	uint8_t addr_type;
	uint8_t peer[6];
} jc2_deferred_evt_t;

#define JC2_RX_RING_DEPTH 16u
#define JC2_DEF_EVT_DEPTH 8u

static jc2_ll_callbacks_t s_cbs = { 0 };
static jc2_ll_conn_t s_conns[JC2_MAX_LINKS];
static jc2_link_diag_t s_diags[JC2_MAX_LINKS];
static uint8_t s_macs[JC2_MAX_LINKS][6];

static jc2_rx_ring_item_t s_rx_ring[JC2_RX_RING_DEPTH];
static volatile uint8_t s_rx_head = 0u;
static volatile uint8_t s_rx_tail = 0u;

static jc2_deferred_evt_t s_def_evts[JC2_DEF_EVT_DEPTH];
static volatile uint8_t s_def_head = 0u;
static volatile uint8_t s_def_tail = 0u;

static volatile uint8_t s_radio_phase = RPHASE_IDLE;
static volatile uint8_t s_active_conn_idx = 0u;
static uint8_t s_last_serviced_idx = 0u;

static volatile bool s_adv_enabled = false;
static volatile uint8_t s_adv_side = 0u;
static uint8_t s_adv_ch_idx = 0u;
static uint32_t s_next_adv_us = 0u;
static uint8_t __attribute__((aligned(4))) s_adv_ind_pdu[42];
static uint8_t __attribute__((aligned(4))) s_scan_rsp_pdu[42];
static uint8_t __attribute__((aligned(4))) s_rx_dma_buf[260];

static bool s_ll_hw_started = false;
static uint32_t s_gap_grants = 0u;
static uint32_t s_gap_skips = 0u;
static uint32_t s_last_gap_us = 0u;
static bool s_has_last_gap = false;
static uint32_t s_gap_min_us = 0xFFFFFFFFu;
static uint32_t s_gap_max_us = 0u;
static uint64_t s_gap_sum_us = 0u;
static uint32_t s_gap_samples = 0u;
static uint32_t s_gap_hist[8] = { 0 };
static uint32_t s_rng_fallback = 0xA53C9E47u;

#if JC2_NRF52_USE_RAM_VTOR
/*
 * RAM copy of the vector table with SVC_Handler optionally kept pointing to
 * the Nordic MBR's SVC dispatcher (0x00000000[11]) so core/FS SVCalls work
 * while RADIO_IRQn and TIMER0_IRQn dispatch directly to our bare-metal ISRs.
 * nRF52840 declares 128 vector slots (512 bytes); Cortex-M4 SCB->VTOR masks
 * bits [8:0] and therefore requires 512-byte alignment.
 */
#ifndef JC2_NRF52_ISR_VECTOR_SYM
#define JC2_NRF52_ISR_VECTOR_SYM __isr_vector
#endif
extern uint32_t JC2_NRF52_ISR_VECTOR_SYM[];
static uint32_t s_ram_vectors[128] __attribute__((aligned(512)));
#endif

static inline uint32_t mask_swi(void)
{
	uint32_t prev = __get_BASEPRI();

	__set_BASEPRI_MAX(2u << (8u - __NVIC_PRIO_BITS));
	__DSB();
	__ISB();
	return prev;
}

static inline void unmask_swi(uint32_t prev)
{
	__DMB();
	__set_BASEPRI(prev);
}

uint32_t jc2_port_irq_lock(void)
{
	uint32_t pm = __get_PRIMASK();

	__disable_irq();
	return pm;
}

void jc2_port_irq_unlock(uint32_t key)
{
	if (!key)
		__enable_irq();
}

uint32_t jc2_port_bh_lock(void)
{
	return mask_swi();
}

void jc2_port_bh_unlock(uint32_t key)
{
	unmask_swi(key);
}

void jc2_port_dmb(void)
{
	__DMB();
}

void jc2_port_random_bytes(uint8_t *out, size_t len)
{
	size_t i;

	if (!out || len == 0u)
		return;
	NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Enabled << RNG_CONFIG_DERCEN_Pos;
	NRF_RNG->EVENTS_VALRDY = 0;
	NRF_RNG->TASKS_START = 1;
	for (i = 0u; i < len; i++) {
		uint32_t guard = 4000u;

		while (!NRF_RNG->EVENTS_VALRDY && --guard != 0u) {
		}
		if (NRF_RNG->EVENTS_VALRDY) {
			out[i] = (uint8_t)NRF_RNG->VALUE;
			NRF_RNG->EVENTS_VALRDY = 0;
		} else {
			s_rng_fallback =
				s_rng_fallback * 1664525u + 1013904223u;
			out[i] = (uint8_t)(s_rng_fallback >> 24);
		}
	}
	NRF_RNG->TASKS_STOP = 1;
}

bool jc2_port_aes128_ecb_be(const uint8_t key_be[16], const uint8_t pt_be[16],
			    uint8_t ct_be[16])
{
	struct __attribute__((aligned(4))) {
		uint8_t key[16];
		uint8_t cleartext[16];
		uint8_t ciphertext[16];
	} ecb_blk;
	uint32_t prev_bp;
	uint32_t guard = 2048u;
	bool hw_ok;

	memcpy(ecb_blk.key, key_be, 16);
	memcpy(ecb_blk.cleartext, pt_be, 16);

	prev_bp = mask_swi();
	NRF_ECB->TASKS_STOPECB = 1;
	NRF_ECB->EVENTS_ENDECB = 0;
	NRF_ECB->EVENTS_ERRORECB = 0;
	NRF_ECB->ECBDATAPTR = (uint32_t)&ecb_blk;
	NRF_ECB->TASKS_STARTECB = 1;
	while (!NRF_ECB->EVENTS_ENDECB && !NRF_ECB->EVENTS_ERRORECB &&
	       --guard != 0u) {
	}
	hw_ok = (NRF_ECB->EVENTS_ENDECB != 0u);
	NRF_ECB->EVENTS_ENDECB = 0;
	NRF_ECB->EVENTS_ERRORECB = 0;
	unmask_swi(prev_bp);

	if (hw_ok) {
		memcpy(ct_be, ecb_blk.ciphertext, 16);
		return true;
	}
	return false;
}

static inline uint32_t timer_now_us(void)
{
	NRF_TIMER0->TASKS_CAPTURE[3] = 1;
	return NRF_TIMER0->CC[3];
}

static inline bool time_before_or_eq(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b) <= 0;
}

/* Must be invoked with priority-0 IRQs disabled (or from a priority-0 ISR). */
static void post_deferred_disc(uint8_t side, uint8_t reason)
{
	uint8_t used = (uint8_t)(s_def_tail - s_def_head);
	uint8_t dslot;

	if (used >= JC2_DEF_EVT_DEPTH)
		return;
	dslot = s_def_tail & (JC2_DEF_EVT_DEPTH - 1u);
	s_def_evts[dslot].kind = 2u;
	s_def_evts[dslot].side = side & 1u;
	s_def_evts[dslot].addr_type = reason;
	__DMB();
	s_def_tail++;
	NVIC_SetPendingIRQ(SWI0_EGU0_IRQn);
}

static uint8_t ble_channel_to_freq(uint8_t ch)
{
	if (ch == 37u)
		return 2u;
	if (ch == 38u)
		return 26u;
	if (ch == 39u)
		return 80u;
	if (ch <= 10u)
		return (uint8_t)(4u + ch * 2u);
	return (uint8_t)(6u + ch * 2u);
}

static void program_radio(uint8_t phy, uint32_t aa, uint32_t crc_init,
			  uint8_t ch)
{
	uint32_t plen;

	NRF_RADIO->MODE =
		(phy == 2u) ?
			(RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos) :
			(RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);
	NRF_RADIO->TXPOWER =
		(RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
	/*
	 * Default 140 us ramp-up enables NRF_RADIO's hardware 150 us T_IFS
	 * state machine across both 1M and 2M PHYs without software jitter.
	 */
	NRF_RADIO->MODECNF0 =
		(RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos);
	NRF_RADIO->TIFS = 150u;

	plen = (phy == 2u) ? (1UL << RADIO_PCNF0_PLEN_Pos) : 0UL;
	NRF_RADIO->PCNF0 = plen | (1UL << RADIO_PCNF0_S0LEN_Pos) |
			   (8UL << RADIO_PCNF0_LFLEN_Pos);
	NRF_RADIO->PCNF1 = (1UL << RADIO_PCNF1_WHITEEN_Pos) |
			   (3UL << RADIO_PCNF1_BALEN_Pos) |
			   (255UL << RADIO_PCNF1_MAXLEN_Pos);

	NRF_RADIO->BASE0 = aa << 8;
	NRF_RADIO->PREFIX0 = (aa >> 24) & 0xFFu;
	NRF_RADIO->TXADDRESS = 0u;
	NRF_RADIO->RXADDRESSES = 1u;

	NRF_RADIO->CRCCNF =
		(RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos) |
		(3UL << RADIO_CRCCNF_LEN_Pos);
	NRF_RADIO->CRCPOLY = 0x00065Bu;
	NRF_RADIO->CRCINIT = crc_init & 0xFFFFFFu;

	NRF_RADIO->FREQUENCY = ble_channel_to_freq(ch);
	NRF_RADIO->DATAWHITEIV = ch & 0x3Fu;
}

static void radio_abort_to_idle(void)
{
	NRF_RADIO->INTENCLR = 0xFFFFFFFFu;
	NRF_RADIO->SHORTS = 0u;
	NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
	NRF_TIMER0->EVENTS_COMPARE[1] = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
	if (NRF_RADIO->STATE != 0u) {
		uint32_t g = 1000u;

		NRF_RADIO->TASKS_DISABLE = 1;
		while (!NRF_RADIO->EVENTS_DISABLED && --g != 0u) {
		}
	}
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->EVENTS_ADDRESS = 0;
	s_radio_phase = RPHASE_IDLE;
}

static uint32_t conn_widen_us(const jc2_ll_conn_t *c)
{
	uint32_t w = 35u + (uint32_t)c->missed_evts * 25u;

	if (w > 400u)
		w = 400u;
	return w;
}

static uint32_t conn_rx_start_us(const jc2_ll_conn_t *c)
{
	return c->anchor_us - 140u - conn_widen_us(c);
}

static void advance_missed_conn_events(jc2_ll_conn_t *c, uint32_t now_us)
{
	uint32_t limit_us;

	if (!c->active)
		return;
	limit_us = c->established ? c->sup_timeout_us : (c->interval_us * 6u);
	if ((uint32_t)(now_us - c->last_rx_ok_us) >= limit_us) {
		c->active = false;
		s_diags[c->side].disc_cnt++;
		s_diags[c->side].last_disc_reason = 0x08u;
		jc2_ll_log_evt(JC2_EVT_DISCONNECTED, c->side, 0x08u);
		post_deferred_disc(c->side, 0x08u);
		return;
	}

	while (time_before_or_eq(c->anchor_us + c->rx_win_extra_us + 80u,
				 now_us)) {
		c->anchor_us += c->interval_us;
		c->event_counter++;
		c->missed_evts++;
		if (!jc2_ll_apply_pending_instants(c)) {
			s_diags[c->side].disc_cnt++;
			s_diags[c->side].last_disc_reason = 0x28u;
			post_deferred_disc(c->side, 0x28u);
			return;
		}
		jc2_ll_next_data_channel(c);
	}
}

static void schedule_next_timer0(void);

static void start_conn_event(uint8_t idx)
{
	jc2_ll_conn_t *c = &s_conns[idx];
	uint32_t widen;
	uint32_t addr_ofs;
	uint32_t addr_deadline;
	uint32_t now_us;

	s_active_conn_idx = idx;
	s_last_serviced_idx = idx;
	s_radio_phase = RPHASE_CONN_RX;

	program_radio(c->phy_rx, c->access_addr, c->crc_init, c->current_ch);
	s_rx_dma_buf[0] = 0u;
	s_rx_dma_buf[1] = 0u;
	NRF_RADIO->PACKETPTR = (uint32_t)s_rx_dma_buf;
	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk |
			    RADIO_SHORTS_DISABLED_TXEN_Msk;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFu;
	NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
	NRF_RADIO->TASKS_RXEN = 1;

	widen = conn_widen_us(c);
	addr_ofs = (c->phy_rx == 2u) ? 27u : 45u;
	addr_deadline =
		c->anchor_us + c->rx_win_extra_us + addr_ofs + widen + 55u;
	now_us = timer_now_us();
	if (time_before_or_eq(addr_deadline, now_us + 180u))
		addr_deadline = now_us + 180u;

	NRF_TIMER0->EVENTS_COMPARE[1] = 0;
	NRF_TIMER0->CC[1] = addr_deadline;
	NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
}

static void finish_conn_event_step(jc2_ll_conn_t *c, bool got_crc_ok)
{
	jc2_link_diag_t *d = &s_diags[c->side];
	uint32_t now_us;
	uint32_t dt_us;

	if (got_crc_ok)
		d->evt_win_cnt++;
	else
		c->missed_evts++;

	now_us = timer_now_us();
	dt_us = now_us - d->tx_win_us;
	if (dt_us >= 1000000u) {
		d->tx_hz = (uint16_t)(((uint64_t)d->tx_win_cnt * 1000000ULL) /
				      dt_us);
		d->evt_hz = (uint16_t)(((uint64_t)d->evt_win_cnt * 1000000ULL) /
				       dt_us);
		d->tx_win_cnt = 0u;
		d->evt_win_cnt = 0u;
		d->tx_win_us = now_us;
	}

	c->anchor_us += c->interval_us;
	c->event_counter++;
	if (!jc2_ll_apply_pending_instants(c)) {
		d->disc_cnt++;
		d->last_disc_reason = 0x28u;
		post_deferred_disc(c->side, 0x28u);
		return;
	}
	jc2_ll_next_data_channel(c);
	if (c->active && !c->staged_valid)
		NVIC_SetPendingIRQ(SWI0_EGU0_IRQn);
}

static void start_adv_channel_tx(void)
{
	uint8_t ch = (uint8_t)(37u + s_adv_ch_idx);
	uint32_t now_us;

	s_radio_phase = RPHASE_ADV_TX;
	program_radio(1u, 0x8E89BED6u, 0x555555u, ch);
	NRF_RADIO->PACKETPTR = (uint32_t)s_adv_ind_pdu;
	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk |
			    RADIO_SHORTS_DISABLED_RXEN_Msk;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFu;
	NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
	NRF_RADIO->TASKS_TXEN = 1;

	now_us = timer_now_us();
	NRF_TIMER0->EVENTS_COMPARE[1] = 0;
	NRF_TIMER0->CC[1] = now_us + 1200u;
	NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
}

static void advance_adv_channel(void)
{
	uint32_t now_us = timer_now_us();

	s_adv_ch_idx++;
	if (s_adv_ch_idx < 3u) {
		s_next_adv_us = now_us + 120u;
	} else {
		s_adv_ch_idx = 0u;
		s_next_adv_us = now_us + 18500u;
	}
}

static void schedule_next_timer0(void)
{
	uint32_t now_us;
	bool c0;
	bool c1;
	uint32_t t0;
	uint32_t t1;
	int pick_conn = -1;
	uint32_t best_us = 0u;

	if (s_radio_phase != RPHASE_IDLE)
		return;

	now_us = timer_now_us();
	advance_missed_conn_events(&s_conns[0], now_us);
	advance_missed_conn_events(&s_conns[1], now_us);

	c0 = s_conns[0].active;
	c1 = s_conns[1].active;
	t0 = c0 ? conn_rx_start_us(&s_conns[0]) : 0u;
	t1 = c1 ? conn_rx_start_us(&s_conns[1]) : 0u;

	if (c0 && c1) {
		int32_t diff = (int32_t)(t0 - t1);

		if (diff > -950 && diff < 950) {
			if (s_conns[0].missed_evts != s_conns[1].missed_evts)
				pick_conn = (s_conns[0].missed_evts >
					     s_conns[1].missed_evts) ?
						    0 :
						    1;
			else
				pick_conn = (int)(s_last_serviced_idx ^ 1u);
		} else {
			pick_conn = (diff <= 0) ? 0 : 1;
		}
		best_us = (pick_conn == 0) ? t0 : t1;
	} else if (c0) {
		pick_conn = 0;
		best_us = t0;
	} else if (c1) {
		pick_conn = 1;
		best_us = t1;
	}

	if (pick_conn >= 0 && time_before_or_eq(best_us, now_us + 25u)) {
		start_conn_event((uint8_t)pick_conn);
		return;
	}

	if (s_adv_enabled && !(c0 && c1)) {
		if (time_before_or_eq(s_next_adv_us, now_us + 25u)) {
			/*
			 * Only start a ~1.1 ms ADV transaction when no active
			 * connection event begins within the next 1500 us.
			 */
			if (pick_conn < 0 ||
			    (int32_t)(best_us - now_us) >= 1500) {
				start_adv_channel_tx();
				return;
			}
			s_next_adv_us = best_us + 900u;
		}
		if (pick_conn < 0 || (int32_t)(s_next_adv_us - best_us) < 0) {
			best_us = s_next_adv_us;
			pick_conn = 2;
		}
	}

	if (pick_conn >= 0) {
		if (time_before_or_eq(best_us, now_us + 15u))
			best_us = now_us + 15u;
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		NRF_TIMER0->CC[0] = best_us;
		NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
	} else {
		NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
	}
}

void jc2_nrf52_timer0_isr(void)
{
	if (NRF_TIMER0->EVENTS_COMPARE[1]) {
		NRF_TIMER0->EVENTS_COMPARE[1] = 0;
		NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;

		if (s_radio_phase == RPHASE_CONN_RX ||
		    s_radio_phase == RPHASE_CONN_TX) {
			jc2_ll_conn_t *c;

			if (s_radio_phase == RPHASE_CONN_RX &&
			    NRF_RADIO->EVENTS_ADDRESS) {
				NRF_TIMER0->CC[1] = timer_now_us() + 2400u;
				NRF_TIMER0->INTENSET =
					TIMER_INTENSET_COMPARE1_Msk;
				return;
			}
			c = &s_conns[s_active_conn_idx];
			radio_abort_to_idle();
			if (c->active)
				finish_conn_event_step(c, false);
			schedule_next_timer0();
			return;
		}

		if (s_radio_phase == RPHASE_ADV_TX ||
		    s_radio_phase == RPHASE_ADV_RX ||
		    s_radio_phase == RPHASE_ADV_SCAN_RSP_TX) {
			if (s_radio_phase == RPHASE_ADV_RX &&
			    NRF_RADIO->EVENTS_ADDRESS) {
				NRF_TIMER0->CC[1] = timer_now_us() + 500u;
				NRF_TIMER0->INTENSET =
					TIMER_INTENSET_COMPARE1_Msk;
				return;
			}
			radio_abort_to_idle();
			advance_adv_channel();
			schedule_next_timer0();
			return;
		}
	}

	if (NRF_TIMER0->EVENTS_COMPARE[0]) {
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		schedule_next_timer0();
	}
}

static void handle_radio_conn_rx_end(void)
{
	jc2_ll_conn_t *c = &s_conns[s_active_conn_idx];
	uint32_t entry_st = NRF_RADIO->STATE;
	uint32_t addr_ts;
	uint32_t addr_ofs;
	uint8_t rx_hdr;
	uint8_t rx_len;
	bool was_enc_rx;
	bool is_new = false;
	bool term_acked = false;
	const uint8_t *tx_ptr;
	bool wake_swi;
	uint32_t tx_st;

	NRF_RADIO->EVENTS_END = 0;
	if ((NRF_RADIO->CRCSTATUS & 1u) == 0u ||
	    entry_st == RADIO_STATE_STATE_TxIdle ||
	    entry_st == RADIO_STATE_STATE_Tx ||
	    entry_st == RADIO_STATE_STATE_TxDisable) {
		radio_abort_to_idle();
		if (c->active)
			finish_conn_event_step(c, false);
		schedule_next_timer0();
		return;
	}

	addr_ts = NRF_TIMER0->CC[2];
	addr_ofs = (c->phy_rx == 2u) ? 27u : 45u;
	c->anchor_us = addr_ts - addr_ofs;
	c->rx_win_extra_us = 0u;
	c->established = true;
	c->missed_evts = 0u;
	c->last_rx_ok_us = c->anchor_us;

	rx_hdr = s_rx_dma_buf[0];
	rx_len = s_rx_dma_buf[1];
	was_enc_rx = c->enc_rx;
	tx_ptr = jc2_ll_arq_step_rx(c, rx_hdr, &s_rx_dma_buf[2], rx_len,
				    &is_new, &term_acked, &s_diags[c->side]);
	if (term_acked)
		post_deferred_disc(c->side, c->terminate_reason);

	wake_swi = c->active && !c->staged_valid;
	if (is_new && rx_len > 0u) {
		uint8_t used = (uint8_t)(s_rx_tail - s_rx_head);

		if (used < JC2_RX_RING_DEPTH) {
			jc2_rx_ring_item_t *it =
				&s_rx_ring[s_rx_tail & (JC2_RX_RING_DEPTH - 1u)];

			it->side = c->side;
			it->gen = c->conn_gen;
			it->hdr = rx_hdr;
			it->len = rx_len;
			it->encrypted = was_enc_rx ? 1u : 0u;
			it->ccm_cnt = was_enc_rx ? c->rx_ccm_cnt++ : 0u;
			memcpy(it->data, &s_rx_dma_buf[2], rx_len);
			__DMB();
			s_rx_tail++;
			wake_swi = true;
		} else if (was_enc_rx) {
			c->rx_ccm_cnt++;
		}
	}
	if (wake_swi)
		NVIC_SetPendingIRQ(SWI0_EGU0_IRQn);

	if (c->phy_tx != c->phy_rx) {
		uint32_t plen =
			(c->phy_tx == 2u) ? (1UL << RADIO_PCNF0_PLEN_Pos) : 0UL;

		NRF_RADIO->MODE = (c->phy_tx == 2u) ? (RADIO_MODE_MODE_Ble_2Mbit
						       << RADIO_MODE_MODE_Pos) :
						      (RADIO_MODE_MODE_Ble_1Mbit
						       << RADIO_MODE_MODE_Pos);
		NRF_RADIO->PCNF0 = plen | (1UL << RADIO_PCNF0_S0LEN_Pos) |
				   (8UL << RADIO_PCNF0_LFLEN_Pos);
	}
	NRF_RADIO->PACKETPTR = (uint32_t)tx_ptr;
	NRF_RADIO->EVENTS_READY = 0;
	(void)NRF_RADIO->EVENTS_READY;
	NRF_RADIO->INTENCLR = RADIO_INTENCLR_END_Msk;
	tx_st = NRF_RADIO->STATE;
	if (tx_st == RADIO_STATE_STATE_Tx ||
	    tx_st == RADIO_STATE_STATE_TxDisable) {
		NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk;
		NRF_RADIO->EVENTS_DISABLED = 0;
		(void)NRF_RADIO->EVENTS_DISABLED;
		if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled) {
			radio_abort_to_idle();
			if (c->active)
				finish_conn_event_step(c, true);
			schedule_next_timer0();
			return;
		}
		NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
	} else if (tx_st == RADIO_STATE_STATE_Disabled) {
		radio_abort_to_idle();
		if (c->active)
			finish_conn_event_step(c, true);
		schedule_next_timer0();
		return;
	} else {
		NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk;
	}
	s_radio_phase = RPHASE_CONN_TX;
	NRF_TIMER0->EVENTS_COMPARE[1] = 0;
	NRF_TIMER0->CC[1] = timer_now_us() + 2600u;
	NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
}

static void handle_radio_adv_rx_end(void)
{
	bool crc_ok = (NRF_RADIO->CRCSTATUS & 1u) != 0u;
	uint8_t hdr = s_rx_dma_buf[0];
	uint8_t plen = s_rx_dma_buf[1];
	uint8_t ptype = (uint8_t)(hdr & 0x0Fu);
	uint8_t rx_add = (uint8_t)((hdr >> 7) & 1u);
	const uint8_t *my_mac = s_macs[s_adv_side & 1u];

	NRF_RADIO->EVENTS_END = 0;

	/* SCAN_REQ (0x03): len = 12, AdvA at [8..13] matches our MAC. */
	if (crc_ok && ptype == 0x03u && plen == 12u && rx_add == 0u &&
	    s_scan_rsp_pdu[1] >= 6u &&
	    memcmp(&s_rx_dma_buf[8], my_mac, 6) == 0) {
		NRF_RADIO->PACKETPTR = (uint32_t)s_scan_rsp_pdu;
		NRF_RADIO->EVENTS_READY = 0;
		NRF_RADIO->INTENCLR = RADIO_INTENCLR_END_Msk;
		NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk;
		s_radio_phase = RPHASE_ADV_SCAN_RSP_TX;
		NRF_TIMER0->EVENTS_COMPARE[1] = 0;
		NRF_TIMER0->CC[1] = timer_now_us() + 650u;
		NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
		return;
	}

	radio_abort_to_idle();

	/* CONNECT_IND (0x05): len = 34, AdvA at [8..13] matches our MAC. */
	if (crc_ok && ptype == 0x05u && plen == 34u && rx_add == 0u &&
	    memcmp(&s_rx_dma_buf[8], my_mac, 6) == 0) {
		uint8_t side = s_adv_side & 1u;
		uint32_t addr_ts = NRF_TIMER0->CC[2];
		uint32_t conn_end_us = addr_ts + 307u;

		if (!s_conns[side].active &&
		    jc2_ll_accept_connect_ind(&s_conns[side], side, hdr,
					      &s_rx_dma_buf[2], conn_end_us,
					      &s_diags[side])) {
			uint8_t used = (uint8_t)(s_def_tail - s_def_head);

			s_adv_enabled = false;
			s_adv_ch_idx = 0u;
			if (used < JC2_DEF_EVT_DEPTH) {
				uint8_t dslot = s_def_tail &
						(JC2_DEF_EVT_DEPTH - 1u);

				s_def_evts[dslot].kind = 1u;
				s_def_evts[dslot].side = side;
				s_def_evts[dslot].addr_type =
					s_conns[side].peer_addr_type;
				memcpy(s_def_evts[dslot].peer,
				       s_conns[side].peer_addr, 6);
				__DMB();
				s_def_tail++;
			}
			NVIC_SetPendingIRQ(SWI0_EGU0_IRQn);
			schedule_next_timer0();
			return;
		}
	}

	advance_adv_channel();
	schedule_next_timer0();
}

void jc2_nrf52_radio_isr(void)
{
	uint8_t phase = s_radio_phase;

	if (phase == RPHASE_CONN_RX && NRF_RADIO->EVENTS_END) {
		handle_radio_conn_rx_end();
		return;
	}

	if (phase == RPHASE_CONN_TX) {
		if (NRF_RADIO->EVENTS_READY) {
			uint32_t st;

			NRF_RADIO->EVENTS_READY = 0;
			NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk;
			NRF_RADIO->EVENTS_DISABLED = 0;
			(void)NRF_RADIO->EVENTS_DISABLED;
			/*
			 * Abort cleanly if a late IRQ arrived after TX ended
			 * so RPHASE_CONN_TX never strands in TXIDLE.
			 */
			st = NRF_RADIO->STATE;
			if (st != RADIO_STATE_STATE_Tx &&
			    st != RADIO_STATE_STATE_TxDisable) {
				jc2_ll_conn_t *c = &s_conns[s_active_conn_idx];

				radio_abort_to_idle();
				if (c->active)
					finish_conn_event_step(c, true);
				schedule_next_timer0();
				return;
			}
			NRF_RADIO->INTENCLR = RADIO_INTENCLR_READY_Msk;
			NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
			return;
		}
		if (NRF_RADIO->EVENTS_DISABLED) {
			jc2_ll_conn_t *c = &s_conns[s_active_conn_idx];

			NRF_RADIO->EVENTS_DISABLED = 0;
			NRF_RADIO->INTENCLR = 0xFFFFFFFFu;
			NRF_RADIO->SHORTS = 0u;
			NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
			NRF_TIMER0->EVENTS_COMPARE[1] = 0;
			s_radio_phase = RPHASE_IDLE;
			if (c->active)
				finish_conn_event_step(c, true);
			schedule_next_timer0();
			return;
		}
	}

	if (phase == RPHASE_ADV_TX) {
		if (NRF_RADIO->EVENTS_END) {
			NRF_RADIO->EVENTS_END = 0;
			s_rx_dma_buf[0] = 0u;
			s_rx_dma_buf[1] = 0u;
			NRF_RADIO->PACKETPTR = (uint32_t)s_rx_dma_buf;
			NRF_RADIO->EVENTS_READY = 0;
			NRF_RADIO->INTENCLR = RADIO_INTENCLR_END_Msk;
			NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk;
			return;
		}
		if (NRF_RADIO->EVENTS_READY) {
			NRF_RADIO->EVENTS_READY = 0;
			NRF_RADIO->EVENTS_END = 0;
			NRF_RADIO->EVENTS_ADDRESS = 0;
			NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
					    RADIO_SHORTS_END_DISABLE_Msk |
					    RADIO_SHORTS_DISABLED_TXEN_Msk;
			NRF_RADIO->INTENCLR = RADIO_INTENCLR_READY_Msk;
			NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
			s_radio_phase = RPHASE_ADV_RX;
			NRF_TIMER0->EVENTS_COMPARE[1] = 0;
			NRF_TIMER0->CC[1] = timer_now_us() + 500u;
			NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
			return;
		}
	}

	if (phase == RPHASE_ADV_RX && NRF_RADIO->EVENTS_END) {
		handle_radio_adv_rx_end();
		return;
	}

	if (phase == RPHASE_ADV_SCAN_RSP_TX) {
		if (NRF_RADIO->EVENTS_READY) {
			uint32_t st;

			NRF_RADIO->EVENTS_READY = 0;
			NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk;
			NRF_RADIO->EVENTS_DISABLED = 0;
			(void)NRF_RADIO->EVENTS_DISABLED;
			st = NRF_RADIO->STATE;
			if (st != RADIO_STATE_STATE_Tx &&
			    st != RADIO_STATE_STATE_TxDisable) {
				radio_abort_to_idle();
				advance_adv_channel();
				schedule_next_timer0();
				return;
			}
			NRF_RADIO->INTENCLR = RADIO_INTENCLR_READY_Msk;
			NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
			return;
		}
		if (NRF_RADIO->EVENTS_DISABLED) {
			radio_abort_to_idle();
			advance_adv_channel();
			schedule_next_timer0();
			return;
		}
	}
}

/*
 * Executed exclusively inside priority-2 SWI0_EGU0_IRQHandler so NVIC pending
 * latching prevents lost wakeups while priority-0 radio IRQs stay unmasked.
 */
void jc2_nrf52_swi0_isr(void)
{
	uint8_t s;

	while (s_def_head != s_def_tail) {
		jc2_deferred_evt_t ev =
			s_def_evts[s_def_head & (JC2_DEF_EVT_DEPTH - 1u)];

		__DMB();
		s_def_head++;
		if (ev.kind == 1u && s_cbs.on_connected)
			s_cbs.on_connected(ev.side, ev.peer, ev.addr_type,
					   s_cbs.user_ctx);
		else if (ev.kind == 2u && s_cbs.on_disconnected)
			s_cbs.on_disconnected(ev.side, ev.addr_type,
					      s_cbs.user_ctx);
	}

	while (s_rx_head != s_rx_tail) {
		jc2_rx_ring_item_t it =
			s_rx_ring[s_rx_head & (JC2_RX_RING_DEPTH - 1u)];
		jc2_ll_conn_t *c;

		__DMB();
		s_rx_head++;

		c = &s_conns[it.side & 1u];
		if (!c->active || c->conn_gen != it.gen)
			continue;

		if (it.encrypted) {
			uint8_t pt[251];

			if (!jc2_ble_ccm_decrypt(c->sk_be, c->iv, it.ccm_cnt,
						 true, it.hdr, it.data, it.len,
						 pt)) {
				uint32_t pm = __get_PRIMASK();

				__disable_irq();
				if (c->conn_gen == it.gen)
					c->active = false;
				if (!pm)
					__enable_irq();
				s_diags[c->side].disc_cnt++;
				s_diags[c->side].last_disc_reason = 0x3Du;
				jc2_ll_log_evt(JC2_EVT_DISCONNECTED, c->side,
					       0x3Du);
				if (s_cbs.on_disconnected)
					s_cbs.on_disconnected(c->side, 0x3Du,
							      s_cbs.user_ctx);
				continue;
			}
			jc2_ll_process_rx_pdu(c, it.hdr, pt,
					      (uint8_t)(it.len - 4u),
					      &s_diags[c->side], &s_cbs);
		} else {
			jc2_ll_process_rx_pdu(c, it.hdr, it.data, it.len,
					      &s_diags[c->side], &s_cbs);
		}
	}

	for (s = 0u; s < JC2_MAX_LINKS; s++) {
		jc2_ll_conn_t *c = &s_conns[s];

		if (c->active && !c->staged_valid)
			(void)jc2_ll_stage_next_tx(c, &s_cbs);
	}
}

#if !defined(__ZEPHYR__)
void TIMER0_IRQHandler(void)
{
	jc2_nrf52_timer0_isr();
}

void RADIO_IRQHandler(void)
{
	jc2_nrf52_radio_isr();
}

void SWI0_EGU0_IRQHandler(void)
{
	jc2_nrf52_swi0_isr();
}
#endif

void jc2_ll_read_device_mac(uint8_t out_mac[6])
{
	uint32_t a0;
	uint32_t a1;

	if (!out_mac)
		return;
	a0 = NRF_FICR->DEVICEADDR[0];
	a1 = NRF_FICR->DEVICEADDR[1];
	out_mac[0] = (uint8_t)(a0 & 0xFFu);
	out_mac[1] = (uint8_t)((a0 >> 8) & 0xFFu);
	out_mac[2] = (uint8_t)((a0 >> 16) & 0xFFu);
	out_mac[3] = (uint8_t)((a0 >> 24) & 0xFFu);
	out_mac[4] = (uint8_t)(a1 & 0xFFu);
	out_mac[5] = (uint8_t)((a1 >> 8) & 0xFFu);
}

void jc2_ll_init(const jc2_ll_callbacks_t *cbs)
{
	uint32_t rg = 2000u;

	if (s_ll_hw_started) {
		if (cbs) {
			uint32_t prev_bp = mask_swi();

			s_cbs = *cbs;
			unmask_swi(prev_bp);
		}
		return;
	}
	if (cbs)
		s_cbs = *cbs;
	s_ll_hw_started = true;

#if JC2_NRF52_USE_RAM_VTOR
	memcpy(s_ram_vectors, JC2_NRF52_ISR_VECTOR_SYM, sizeof(s_ram_vectors));
#if JC2_NRF52_USE_MBR
	/*
	 * Read MBR SVC_Handler directly from 0x0000002C (vector [11]) via a
	 * non-zero volatile address so GCC's -fdelete-null-pointer-checks
	 * never treats ((uint32_t *)0)[11] as UB and emits a trap instruction.
	 */
	s_ram_vectors[11] = *(const volatile uint32_t *)0x0000002Cu;
#endif
	__DSB();
	SCB->VTOR = (uint32_t)s_ram_vectors;
	__DSB();
	__ISB();
#endif

	memset(s_conns, 0, sizeof(s_conns));
	memset(s_diags, 0, sizeof(s_diags));
	s_rx_head = 0u;
	s_rx_tail = 0u;
	s_def_head = 0u;
	s_def_tail = 0u;
	s_radio_phase = RPHASE_IDLE;
	s_adv_enabled = false;

	/* Ensure 64 MHz HFXO crystal is running for +/-20 ppm BLE anchors. */
	if ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) == 0u) {
		uint32_t guard = 100000u;

		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
		NRF_CLOCK->TASKS_HFCLKSTART = 1;
		while (!NRF_CLOCK->EVENTS_HFCLKSTARTED && --guard != 0u) {
		}
	}
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	NRF_RADIO->POWER = 1;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFu;
	NRF_RADIO->SHORTS = 0u;
	NRF_RADIO->TASKS_DISABLE = 1;
	while (NRF_RADIO->STATE != 0u && --rg != 0u) {
	}
	NRF_RADIO->EVENTS_DISABLED = 0;

	NRF_TIMER0->TASKS_STOP = 1;
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit
			      << TIMER_BITMODE_BITMODE_Pos;
	/* 16 MHz / 2^4 = 1 MHz (1 tick = 1 us). */
	NRF_TIMER0->PRESCALER = 4u;
	NRF_TIMER0->SHORTS = 0u;
	NRF_TIMER0->INTENCLR = 0xFFFFFFFFu;
	NRF_TIMER0->EVENTS_COMPARE[0] = 0;
	NRF_TIMER0->EVENTS_COMPARE[1] = 0;
	NRF_TIMER0->TASKS_START = 1;

	/* PPI ch 0 latches TIMER0->CC[2] at every RADIO->EVENTS_ADDRESS. */
	NRF_PPI->CHENCLR = (1UL << 0);
	NRF_PPI->CH[0].EEP = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
	NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER0->TASKS_CAPTURE[2];
	NRF_PPI->CHENSET = (1UL << 0);

	NVIC_SetPriority(SWI0_EGU0_IRQn, 2);
	NVIC_ClearPendingIRQ(SWI0_EGU0_IRQn);
	NVIC_EnableIRQ(SWI0_EGU0_IRQn);

	NVIC_SetPriority(RADIO_IRQn, 0);
	NVIC_SetPriority(TIMER0_IRQn, 0);
	NVIC_ClearPendingIRQ(RADIO_IRQn);
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(RADIO_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);
}

void jc2_ll_set_mac(uint8_t side, const uint8_t mac_le[6])
{
	if (mac_le)
		memcpy(s_macs[side & 1u], mac_le, 6);
}

void jc2_ll_start_adv(uint8_t side, const uint8_t *adv_data, uint8_t adv_len,
		      const uint8_t *scan_rsp, uint8_t scan_rsp_len)
{
	uint32_t pm;

	if (adv_len > 31u)
		adv_len = 31u;
	if (scan_rsp_len > 31u)
		scan_rsp_len = 31u;
	side &= 1u;

	pm = __get_PRIMASK();
	__disable_irq();
	if (s_radio_phase == RPHASE_ADV_TX || s_radio_phase == RPHASE_ADV_RX ||
	    s_radio_phase == RPHASE_ADV_SCAN_RSP_TX) {
		radio_abort_to_idle();
	}
	s_adv_side = side;
	s_adv_ch_idx = 0u;
	/*
	 * ADV_IND, Public TxAdd=0, ChSel=1 (CSA #2 supported, matching
	 * c->csa2 handling in jc2_ll_accept_connect_ind() per Core Spec
	 * Vol 6 Part B 4.5.8).
	 */
	s_adv_ind_pdu[0] = 0x20u;
	s_adv_ind_pdu[1] = (uint8_t)(6u + adv_len);
	memcpy(&s_adv_ind_pdu[2], s_macs[side], 6);
	if (adv_len > 0u && adv_data)
		memcpy(&s_adv_ind_pdu[8], adv_data, adv_len);

	/* SCAN_RSP, Public TxAdd=0. */
	s_scan_rsp_pdu[0] = 0x04u;
	s_scan_rsp_pdu[1] = (uint8_t)(6u + scan_rsp_len);
	memcpy(&s_scan_rsp_pdu[2], s_macs[side], 6);
	if (scan_rsp_len > 0u && scan_rsp)
		memcpy(&s_scan_rsp_pdu[8], scan_rsp, scan_rsp_len);

	s_adv_enabled = true;
	s_next_adv_us = timer_now_us() + 80u;
	schedule_next_timer0();
	if (!pm)
		__enable_irq();
}

void jc2_ll_stop_adv(void)
{
	uint32_t pm = __get_PRIMASK();

	__disable_irq();
	s_adv_enabled = false;
	if (s_radio_phase == RPHASE_ADV_TX || s_radio_phase == RPHASE_ADV_RX ||
	    s_radio_phase == RPHASE_ADV_SCAN_RSP_TX) {
		radio_abort_to_idle();
	}
	schedule_next_timer0();
	if (!pm)
		__enable_irq();
}

bool jc2_ll_is_advertising(void)
{
	return s_adv_enabled;
}

uint8_t jc2_ll_adv_side(void)
{
	return s_adv_side;
}

bool jc2_ll_is_connected(uint8_t side)
{
	return s_conns[side & 1u].active;
}

bool jc2_ll_is_secured(uint8_t side)
{
	const jc2_ll_conn_t *c = &s_conns[side & 1u];

	return c->active && c->secured;
}

uint16_t jc2_ll_get_cccd(uint8_t side, uint16_t cccd_handle)
{
	const jc2_ll_conn_t *c = &s_conns[side & 1u];

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
	const jc2_ll_conn_t *c = &s_conns[side & 1u];

	return c->active ? c->mtu : 0u;
}

uint8_t jc2_ll_tx_queue_depth(uint8_t side)
{
	const jc2_ll_conn_t *c = &s_conns[side & 1u];

	if (!c->active)
		return 0u;
	return (uint8_t)((uint8_t)(c->tx_q_tail - c->tx_q_head) +
			 (uint8_t)(c->ll_q_tail - c->ll_q_head));
}

bool jc2_ll_notify(uint8_t side, uint16_t val_handle, const uint8_t *data,
		   uint16_t len, bool is_input_hvn)
{
	uint32_t prev_bp;
	bool ok;

	side &= 1u;
	prev_bp = mask_swi();
	ok = jc2_ll_enqueue_att_ntf(&s_conns[side], val_handle, data, len,
				    is_input_hvn);
	if (ok && s_conns[side].active && !s_conns[side].staged_valid)
		NVIC_SetPendingIRQ(SWI0_EGU0_IRQn);
	if (!ok && is_input_hvn && s_conns[side].active)
		s_diags[side].hvx_drop++;
	unmask_swi(prev_bp);
	return ok;
}

void jc2_ll_disconnect(uint8_t side, uint8_t reason)
{
	uint32_t prev_bp;
	jc2_ll_conn_t *c;

	side &= 1u;
	prev_bp = mask_swi();
	c = &s_conns[side];
	if (c->active && !c->terminate_pending) {
		uint8_t term[2] = { 0x02u, reason };

		c->terminate_reason = reason;
		c->terminate_pending = true;
		if (jc2_ll_enqueue_ctrl_pdu(c, term, sizeof(term), true)) {
			if (!c->staged_valid)
				NVIC_SetPendingIRQ(SWI0_EGU0_IRQn);
		} else {
			uint32_t pm = __get_PRIMASK();

			__disable_irq();
			c->active = false;
			s_diags[side].disc_cnt++;
			s_diags[side].last_disc_reason = reason;
			post_deferred_disc(side, reason);
			if (!pm)
				__enable_irq();
			jc2_ll_log_evt(JC2_EVT_DISCONNECTED, side, reason);
		}
	}
	unmask_swi(prev_bp);
}

void jc2_ll_poll(void)
{
	uint32_t now_us;
	bool need_swi;
	uint8_t s;
	uint32_t pm;

	if (!s_ll_hw_started)
		return;

	now_us = timer_now_us();
	need_swi = (s_def_head != s_def_tail) || (s_rx_head != s_rx_tail);
	for (s = 0u; s < JC2_MAX_LINKS; s++) {
		jc2_ll_conn_t *c = &s_conns[s];

		if (c->active) {
			jc2_link_diag_t *d = &s_diags[s];

			if (!c->staged_valid)
				need_swi = true;
			if ((uint32_t)(now_us - d->tx_win_us) >= 1500000u) {
				d->tx_hz = 0u;
				d->evt_hz = 0u;
			}
		} else {
			s_diags[s].tx_hz = 0u;
			s_diags[s].evt_hz = 0u;
		}
	}
	if (need_swi)
		NVIC_SetPendingIRQ(SWI0_EGU0_IRQn);

	pm = __get_PRIMASK();
	__disable_irq();
	if (s_radio_phase == RPHASE_IDLE)
		schedule_next_timer0();
	if (!pm)
		__enable_irq();
}

uint32_t jc2_ll_lock_bottom_half(void)
{
	if (!s_ll_hw_started)
		return 0u;
	return mask_swi();
}

void jc2_ll_unlock_bottom_half(uint32_t prev_basepri)
{
	if (!s_ll_hw_started)
		return;
	unmask_swi(prev_basepri);
}

uint32_t jc2_ll_now_us(void)
{
	if (s_ll_hw_started)
		return timer_now_us();
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	return DWT->CYCCNT >> 6;
}

uint32_t jc2_ll_remaining_gap_us(void)
{
	uint32_t pm;
	uint32_t now_us;
	uint32_t rem = 0xFFFFFFFFu;
	uint8_t i;

	if (!s_ll_hw_started)
		return 0xFFFFFFFFu;
	pm = __get_PRIMASK();
	__disable_irq();
	now_us = timer_now_us();
	for (i = 0u; i < JC2_MAX_LINKS; i++) {
		int32_t dt;

		if (!s_conns[i].active)
			continue;
		dt = (int32_t)(conn_rx_start_us(&s_conns[i]) - now_us);
		if (dt <= 0) {
			rem = 0u;
			break;
		}
		if ((uint32_t)dt < rem)
			rem = (uint32_t)dt;
	}
	if (!pm)
		__enable_irq();
	return rem;
}

bool jc2_ll_acquire_gap(uint32_t min_free_us)
{
	uint32_t pm;
	uint32_t now_us;
	uint8_t i;

	if (!s_ll_hw_started)
		return true;
	pm = __get_PRIMASK();
	__disable_irq();
	if (s_radio_phase != RPHASE_IDLE) {
		s_gap_skips++;
		if (!pm)
			__enable_irq();
		return false;
	}
	now_us = timer_now_us();
	for (i = 0u; i < JC2_MAX_LINKS; i++) {
		uint32_t rx_s;

		if (!s_conns[i].active)
			continue;
		rx_s = conn_rx_start_us(&s_conns[i]);
		if ((int32_t)(rx_s - now_us) < (int32_t)min_free_us) {
			s_gap_skips++;
			if (!pm)
				__enable_irq();
			return false;
		}
	}
	s_radio_phase = RPHASE_GAP_BUSY;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFu;
	NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk |
			       TIMER_INTENCLR_COMPARE1_Msk;
	s_gap_grants++;
	if (s_has_last_gap) {
		uint32_t gap = now_us - s_last_gap_us;
		uint8_t b = 7u;

		if (gap < s_gap_min_us)
			s_gap_min_us = gap;
		if (gap > s_gap_max_us)
			s_gap_max_us = gap;
		s_gap_sum_us += gap;
		s_gap_samples++;
		if (gap < 2000u)
			b = 0u;
		else if (gap < 4500u)
			b = 1u;
		else if (gap < 7000u)
			b = 2u;
		else if (gap < 10000u)
			b = 3u;
		else if (gap < 15000u)
			b = 4u;
		else if (gap < 25000u)
			b = 5u;
		else if (gap < 50000u)
			b = 6u;
		s_gap_hist[b]++;
	}
	s_last_gap_us = now_us;
	s_has_last_gap = true;
	if (!pm)
		__enable_irq();
	return true;
}

void jc2_ll_release_gap(void)
{
	uint32_t pm;

	if (!s_ll_hw_started)
		return;
	pm = __get_PRIMASK();
	__disable_irq();
	if (s_radio_phase == RPHASE_GAP_BUSY) {
		uint32_t now_us;

		radio_abort_to_idle();
		now_us = timer_now_us();
		if (s_adv_enabled && time_before_or_eq(s_next_adv_us, now_us))
			s_next_adv_us = now_us + 60u;
		schedule_next_timer0();
	}
	if (!pm)
		__enable_irq();
}

const jc2_link_diag_t *jc2_ll_get_diag(uint8_t side)
{
	return &s_diags[side & 1u];
}

void jc2_ll_get_gap_stats(jc2_gap_stats_t *out, bool reset)
{
	uint32_t pm = __get_PRIMASK();

	__disable_irq();
	if (out) {
		out->grants = s_gap_grants;
		out->skips = s_gap_skips;
		out->min_us = s_gap_samples ? s_gap_min_us : 0u;
		out->avg_us = s_gap_samples ?
				      (uint32_t)(s_gap_sum_us / s_gap_samples) :
				      0u;
		out->max_us = s_gap_max_us;
		out->samples = s_gap_samples;
		memcpy(out->hist, s_gap_hist, sizeof(s_gap_hist));
	}
	if (reset) {
		s_gap_grants = 0u;
		s_gap_skips = 0u;
		s_has_last_gap = false;
		s_gap_min_us = 0xFFFFFFFFu;
		s_gap_max_us = 0u;
		s_gap_sum_us = 0u;
		s_gap_samples = 0u;
		memset(s_gap_hist, 0, sizeof(s_gap_hist));
	}
	if (!pm)
		__enable_irq();
}

#else /* Non-nRF52840 / host build: compiled by another jc2_ll_*.c backend */

typedef int jc2_ll_nrf52_unused_tu_t;

#endif /* defined(NRF52840_XXAA) && !defined(JC2_HOST_TEST) */
