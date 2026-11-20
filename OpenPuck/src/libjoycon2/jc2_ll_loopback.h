/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#ifndef JC2_LL_LOOPBACK_H
#define JC2_LL_LOOPBACK_H

#include "jc2_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Host-side in-memory BLE 5.0 loopback backend API (compiled when
 * JC2_HOST_TEST is defined). Allows CI and unit tests to drive a full
 * jc2_device_t end-to-end as a virtual Switch 2 Central with real Link-Layer
 * ARQ, AES-128-CCM encryption, GATT discovery, and coexistence gap simulation.
 */
typedef struct {
	uint8_t side;
	uint8_t sn;
	uint8_t nesn;
	bool enc_tx;
	bool enc_rx;
	uint8_t sk_be[16];
	uint8_t iv8[8];
	uint64_t m2s_ccm_cnt;
	uint64_t s2m_ccm_cnt;
} jc2_lb_central_t;

void jc2_lb_reset(void);
void jc2_lb_set_now_us(uint32_t now_us);
void jc2_lb_advance_us(uint32_t delta_us);
void jc2_lb_set_gap_budget_us(uint32_t free_us);

bool jc2_lb_get_adv(uint8_t *out_side, uint8_t out_mac[6], uint8_t out_adv[31],
		    uint8_t *out_adv_len, uint8_t out_scan_rsp[31],
		    uint8_t *out_scan_rsp_len);

bool jc2_lb_connect(jc2_lb_central_t *cent, uint8_t side, uint8_t pdu_hdr,
		    const uint8_t pay34[34]);

/*
 * Execute one simulated BLE connection event (Central -> Peripheral RX +
 * Peripheral -> Central TX + bottom-half RX processing + anchor/channel step).
 * Returns false if link is inactive or Central-side CCM MIC verification fails.
 */
bool jc2_lb_exchange(jc2_lb_central_t *cent, uint8_t rx_llid,
		     const uint8_t *rx_plain, uint8_t rx_plain_len,
		     uint8_t *out_tx_llid, uint8_t *out_tx_plain,
		     uint8_t *out_tx_plain_len, uint8_t *out_tx_raw_hdr);

jc2_ll_conn_t *jc2_lb_get_conn(uint8_t side);

#ifdef __cplusplus
}
#endif

#endif /* JC2_LL_LOOPBACK_H */
