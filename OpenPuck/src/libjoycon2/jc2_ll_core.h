/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#ifndef JC2_LL_CORE_H
#define JC2_LL_CORE_H

#include "jc2_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Power-of-two ring depths so uint8_t head/tail wrap cleanly across 256. */
#define JC2_LL_Q_DEPTH 8u
#define JC2_TX_Q_DEPTH 32u
#define JC2_EVT_LOG_DEPTH 32u

#define JC2_STAGED_NONE 0u
#define JC2_STAGED_LL_Q 1u
#define JC2_STAGED_ATT_Q 2u
#define JC2_STAGED_LIVE 3u

typedef struct {
	bool (*get_ltk)(uint8_t side, uint8_t out_ltk_le[16], void *user_ctx);
	void (*on_connected)(uint8_t side, const uint8_t peer_addr[6],
			     uint8_t peer_addr_type, void *user_ctx);
	void (*on_disconnected)(uint8_t side, uint8_t reason, void *user_ctx);
	void (*on_att_write)(uint8_t side, uint16_t handle, const uint8_t *data,
			     uint16_t len, void *user_ctx);
	void (*on_input_cccd)(uint8_t side, uint16_t handle, bool enabled,
			      void *user_ctx);
	uint16_t (*build_live_input)(uint8_t side, uint16_t *out_handle,
				     uint8_t *out_buf, uint16_t max_len,
				     void *user_ctx);
	void (*read_input_value)(uint8_t side, uint16_t handle,
				 uint8_t *out_buf, uint16_t *inout_len,
				 void *user_ctx);
	void *user_ctx;
} jc2_ll_callbacks_t;

typedef struct {
	uint8_t len;
	uint8_t is_term;
	uint8_t data[27];
} jc2_ll_ctrl_pdu_t;

typedef struct {
	uint8_t llid;
	uint8_t len;
	uint8_t is_input_hvn;
	uint8_t data[251];
} jc2_ll_tx_pdu_t;

typedef struct {
	volatile bool active;
	bool established;
	bool has_transmitted;
	bool tx_inflight;
	bool last_empty_unacked;
	volatile bool staged_valid;
	bool staged_is_input;
	bool staged_is_term;
	uint8_t staged_src;
	bool inflight_is_input;
	bool inflight_is_term;
	volatile bool enc_rx;
	volatile bool enc_tx;
	volatile bool enc_pause_data;
	volatile bool secured;
	bool version_sent;
	bool csa2;
	bool terminate_pending;
	uint8_t terminate_reason;
	uint8_t conn_gen;

	uint8_t side;
	uint8_t peer_addr[6];
	uint8_t peer_addr_type;
	uint32_t access_addr;
	uint32_t crc_init;
	uint16_t chan_id;

	uint8_t hop_inc;
	uint8_t last_unmapped_ch;
	uint8_t current_ch;
	uint8_t chm[5];
	uint8_t remap_table[37];
	uint8_t num_used_ch;

	uint8_t nesn;
	uint8_t tx_sn;
	uint8_t phy_rx;
	uint8_t phy_tx;
	uint16_t event_counter;
	uint16_t missed_evts;

	uint16_t interval_125;
	uint16_t latency;
	uint16_t sup_timeout_10ms;
	uint32_t interval_us;
	uint32_t sup_timeout_us;
	uint32_t anchor_us;
	uint32_t rx_win_extra_us;
	uint32_t last_rx_ok_us;
	uint32_t created_us;

	volatile bool conn_upd_pending;
	uint8_t upd_win_size;
	uint16_t upd_win_offset;
	uint16_t upd_interval_125;
	uint16_t upd_latency;
	uint16_t upd_timeout_10ms;
	uint16_t upd_instant;

	volatile bool chm_upd_pending;
	uint8_t upd_chm[5];
	uint16_t chm_instant;

	volatile bool phy_upd_pending;
	uint8_t upd_phy_m2s;
	uint8_t upd_phy_s2m;
	uint16_t phy_instant;

	uint8_t sk_be[16];
	uint8_t iv[8];
	uint64_t rx_ccm_cnt;
	uint64_t tx_ccm_cnt;

	uint16_t mtu;
	uint16_t cccd_000a;
	uint16_t cccd_000e;
	uint16_t cccd_001a;
	uint16_t cccd_001e;
	uint16_t cccd_0022;
	uint16_t cccd_0026;
	uint16_t rate_000c;
	uint16_t rate_0010;
	uint16_t rate_0028;

	uint16_t l2cap_rx_expected;
	uint16_t l2cap_rx_len;
	uint16_t l2cap_rx_cid;
	uint8_t l2cap_rx_buf[520];

	volatile uint8_t ll_q_head;
	volatile uint8_t ll_q_tail;
	jc2_ll_ctrl_pdu_t ll_q[JC2_LL_Q_DEPTH];

	volatile uint8_t tx_q_head;
	volatile uint8_t tx_q_tail;
	jc2_ll_tx_pdu_t tx_q[JC2_TX_Q_DEPTH];

	jc2_dma_buf260_t inflight_buf;
	jc2_dma_buf260_t staged_buf;
	jc2_dma_buf4_t empty_buf;
} jc2_ll_conn_t;

void jc2_ll_log_evt(uint8_t evt_id, uint8_t side, uint16_t hdl);
uint8_t jc2_ll_get_evt_count(void);
const jc2_evt_log_t *jc2_ll_get_evt_log(uint8_t *out_max_len);

bool jc2_ll_enqueue_ctrl_pdu(jc2_ll_conn_t *c, const uint8_t *data, uint8_t len,
			     bool is_term);
bool jc2_ll_enqueue_l2cap_att(jc2_ll_conn_t *c, const uint8_t *att,
			      uint16_t len, bool is_input_hvn);
bool jc2_ll_enqueue_att_ntf(jc2_ll_conn_t *c, uint16_t handle,
			    const uint8_t *val, uint16_t len,
			    bool is_input_hvn);

uint8_t jc2_ll_next_data_channel(jc2_ll_conn_t *c);
/*
 * Apply any connection/channel-map/PHY update whose instant equals
 * c->event_counter. Returns false (and marks c inactive with reason 0x28,
 * Instant Passed) if an instant has already been missed.
 */
bool jc2_ll_apply_pending_instants(jc2_ll_conn_t *c);
bool jc2_ll_accept_connect_ind(jc2_ll_conn_t *c, uint8_t side, uint8_t pdu_hdr,
			       const uint8_t *pay34, uint32_t connect_end_us,
			       jc2_link_diag_t *diag);
void jc2_ll_process_rx_pdu(jc2_ll_conn_t *c, uint8_t hdr,
			   const uint8_t *payload, uint8_t len,
			   jc2_link_diag_t *diag,
			   const jc2_ll_callbacks_t *cbs);
bool jc2_ll_stage_next_tx(jc2_ll_conn_t *c, const jc2_ll_callbacks_t *cbs);
const uint8_t *jc2_ll_arq_step_rx(jc2_ll_conn_t *c, uint8_t rx_hdr,
				  const uint8_t *rx_payload_raw,
				  uint8_t rx_raw_len, bool *out_new_rx,
				  bool *out_term_acked, jc2_link_diag_t *diag);

#ifdef __cplusplus
}
#endif

#endif /* JC2_LL_CORE_H */
