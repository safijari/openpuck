/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#ifndef JC2_CRYPTO_H
#define JC2_CRYPTO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* AES-128-ECB on big-endian 16-byte key and plaintext blocks. */
void jc2_aes128_ecb_be(const uint8_t key_be[16], const uint8_t pt_be[16],
		       uint8_t ct_be[16]);

/* Derive big-endian BLE session key SK = AES-ECB(LTK_BE, SKDs_BE || SKDm_BE). */
void jc2_ble_derive_sk(const uint8_t ltk_le[16], const uint8_t skdm[8],
		       const uint8_t skds[8], uint8_t out_sk_be[16]);

/* BLE Link-Layer AES-128-CCM encrypt (Vol 6, Part E, Section 2); appends 4-byte MIC. */
void jc2_ble_ccm_encrypt(const uint8_t sk_be[16], const uint8_t iv8[8],
			 uint64_t pkt_cnt, bool master_to_slave, uint8_t hdr,
			 const uint8_t *pt, uint8_t pt_len,
			 uint8_t *out_ct_mic);

/* BLE Link-Layer AES-128-CCM decrypt + constant-time 4-byte MIC verification. */
bool jc2_ble_ccm_decrypt(const uint8_t sk_be[16], const uint8_t iv8[8],
			 uint64_t pkt_cnt, bool master_to_slave, uint8_t hdr,
			 const uint8_t *ct_mic, uint8_t ct_mic_len,
			 uint8_t *out_pt);

#ifdef __cplusplus
}
#endif

#endif /* JC2_CRYPTO_H */
