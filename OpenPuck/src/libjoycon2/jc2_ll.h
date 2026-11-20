/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#ifndef JC2_LL_H
#define JC2_LL_H

#include "jc2_ll_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Target-neutral BLE 5.0 Link-Layer backend contract.
 * Implemented at link time by the active target driver (e.g. jc2_ll_nrf52.c on
 * nRF52840, jc2_ll_loopback.c under JC2_HOST_TEST, or a custom MCU backend) so
 * priority-0 radio ISRs incur zero virtual-dispatch overhead.
 *
 * Each connection event executes one RX + one TX PDU exchange and transmits
 * L2CAP ATT payloads in a single LL Data PDU, requiring BLE 4.2+ Data Length
 * Extension (DLE, 251-octet PDUs) on the peer central.
 */

/* Read the factory device MAC address (or deterministic fallback in host tests). */
void jc2_ll_read_device_mac(uint8_t out_mac[6]);

/* Hardware Link-Layer & dual-link BLE 5.0 scheduler API. */
void jc2_ll_init(const jc2_ll_callbacks_t *cbs);
void jc2_ll_set_mac(uint8_t side, const uint8_t mac_le[6]);
void jc2_ll_start_adv(uint8_t side, const uint8_t *adv_data, uint8_t adv_len,
		      const uint8_t *scan_rsp, uint8_t scan_rsp_len);
void jc2_ll_stop_adv(void);
bool jc2_ll_is_advertising(void);
uint8_t jc2_ll_adv_side(void);

bool jc2_ll_is_connected(uint8_t side);
bool jc2_ll_is_secured(uint8_t side);
uint16_t jc2_ll_get_cccd(uint8_t side, uint16_t cccd_handle);
uint16_t jc2_ll_get_mtu(uint8_t side);
uint8_t jc2_ll_tx_queue_depth(uint8_t side);
bool jc2_ll_notify(uint8_t side, uint16_t val_handle, const uint8_t *data,
		   uint16_t len, bool is_input_hvn);
void jc2_ll_disconnect(uint8_t side, uint8_t reason);
void jc2_ll_poll(void);

/* Mask/unmask bottom-half IRQ while leaving priority-0 radio IRQs unmasked. */
uint32_t jc2_ll_lock_bottom_half(void);
void jc2_ll_unlock_bottom_half(uint32_t prev_key);

/*
 * Generic radio-coexistence gap API.
 * Allows another 2.4 GHz protocol (or flash NVMC bond persistence) to borrow
 * exclusive radio ownership strictly inside idle windows between scheduled
 * BLE connection events.
 */
uint32_t jc2_ll_now_us(void);
uint32_t jc2_ll_remaining_gap_us(void);
bool jc2_ll_acquire_gap(uint32_t min_free_us);
void jc2_ll_release_gap(void);

const jc2_link_diag_t *jc2_ll_get_diag(uint8_t side);
void jc2_ll_get_gap_stats(jc2_gap_stats_t *out, bool reset);

#ifdef __cplusplus
}
#endif

#endif /* JC2_LL_H */
