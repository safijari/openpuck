/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#include "jc2_crypto.h"

#if !defined(JC2_CRYPTO_SOFT_ONLY)
#include "jc2_port.h"
#endif

#include <string.h>

static const uint8_t JC2_AES_SBOX[256] = {
	0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b,
	0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
	0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26,
	0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
	0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2,
	0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
	0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed,
	0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
	0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f,
	0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
	0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec,
	0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
	0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14,
	0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
	0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
	0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
	0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f,
	0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
	0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11,
	0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
	0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f,
	0xb0, 0x54, 0xbb, 0x16
};

static inline uint8_t aes_xtime(uint8_t x)
{
	return (uint8_t)((x << 1) ^ (((x >> 7) & 1u) * 0x1Bu));
}

static void aes128_soft_ecb_be(const uint8_t key_be[16],
			       const uint8_t pt_be[16], uint8_t ct_be[16])
{
	uint8_t rk[176];
	uint8_t s[16];
	uint8_t rcon = 0x01u;
	uint32_t i;
	uint32_t r;

	memcpy(rk, key_be, 16);
	for (i = 4u; i < 44u; i++) {
		uint8_t t[4];
		uint32_t j;

		memcpy(t, &rk[(i - 1u) * 4u], 4);
		if ((i & 3u) == 0u) {
			uint8_t u = t[0];

			t[0] = JC2_AES_SBOX[t[1]] ^ rcon;
			t[1] = JC2_AES_SBOX[t[2]];
			t[2] = JC2_AES_SBOX[t[3]];
			t[3] = JC2_AES_SBOX[u];
			rcon = aes_xtime(rcon);
		}
		for (j = 0u; j < 4u; j++)
			rk[i * 4u + j] = rk[(i - 4u) * 4u + j] ^ t[j];
	}

	for (i = 0u; i < 16u; i++)
		s[i] = pt_be[i] ^ rk[i];

	for (r = 1u; r <= 10u; r++) {
		uint8_t tmp;

		for (i = 0u; i < 16u; i++)
			s[i] = JC2_AES_SBOX[s[i]];

		tmp = s[1];
		s[1] = s[5];
		s[5] = s[9];
		s[9] = s[13];
		s[13] = tmp;

		tmp = s[2];
		s[2] = s[10];
		s[10] = tmp;
		tmp = s[6];
		s[6] = s[14];
		s[14] = tmp;

		tmp = s[15];
		s[15] = s[11];
		s[11] = s[7];
		s[7] = s[3];
		s[3] = tmp;

		if (r < 10u) {
			uint32_t c;

			for (c = 0u; c < 4u; c++) {
				uint32_t o = c * 4u;
				uint8_t a0 = s[o + 0u];
				uint8_t a1 = s[o + 1u];
				uint8_t a2 = s[o + 2u];
				uint8_t a3 = s[o + 3u];
				uint8_t xr = a0 ^ a1 ^ a2 ^ a3;

				s[o + 0u] ^= xr ^ aes_xtime(a0 ^ a1);
				s[o + 1u] ^= xr ^ aes_xtime(a1 ^ a2);
				s[o + 2u] ^= xr ^ aes_xtime(a2 ^ a3);
				s[o + 3u] ^= xr ^ aes_xtime(a3 ^ a0);
			}
		}

		for (i = 0u; i < 16u; i++)
			s[i] ^= rk[r * 16u + i];
	}
	memcpy(ct_be, s, 16);
}

void jc2_aes128_ecb_be(const uint8_t key_be[16], const uint8_t pt_be[16],
		       uint8_t ct_be[16])
{
#if !defined(JC2_CRYPTO_SOFT_ONLY)
	if (jc2_port_aes128_ecb_be(key_be, pt_be, ct_be))
		return;
#endif
	aes128_soft_ecb_be(key_be, pt_be, ct_be);
}

void jc2_ble_derive_sk(const uint8_t ltk_le[16], const uint8_t skdm[8],
		       const uint8_t skds[8], uint8_t out_sk_be[16])
{
	uint8_t key_be[16];
	uint8_t skd_be[16];
	int i;

	for (i = 0; i < 16; i++)
		key_be[i] = ltk_le[15 - i];
	for (i = 0; i < 8; i++) {
		skd_be[i] = skds[7 - i];
		skd_be[8 + i] = skdm[7 - i];
	}
	jc2_aes128_ecb_be(key_be, skd_be, out_sk_be);
}

static void ble_build_nonce(uint64_t pkt_cnt, bool master_to_slave,
			    const uint8_t iv8[8], uint8_t nonce13[13])
{
	nonce13[0] = (uint8_t)(pkt_cnt & 0xFFu);
	nonce13[1] = (uint8_t)((pkt_cnt >> 8) & 0xFFu);
	nonce13[2] = (uint8_t)((pkt_cnt >> 16) & 0xFFu);
	nonce13[3] = (uint8_t)((pkt_cnt >> 24) & 0xFFu);
	nonce13[4] = (uint8_t)(((pkt_cnt >> 32) & 0x7Fu) |
			       (master_to_slave ? 0x80u : 0x00u));
	memcpy(&nonce13[5], iv8, 8);
}

static void ble_ccm_mac(const uint8_t sk_be[16], const uint8_t nonce13[13],
			uint8_t hdr, const uint8_t *pt, uint8_t pt_len,
			const uint8_t s0[16], uint8_t out_mic4[4])
{
	uint8_t b[16];
	uint8_t x[16];
	uint16_t off;
	int i;

	b[0] = 0x49u;
	memcpy(&b[1], nonce13, 13);
	b[14] = 0u;
	b[15] = pt_len;
	jc2_aes128_ecb_be(sk_be, b, x);

	memset(b, 0, sizeof(b));
	b[0] = 0x00u;
	b[1] = 0x01u;
	/* NESN, SN, and MD bits are excluded from CCM header authentication. */
	b[2] = (uint8_t)(hdr & 0xE3u);
	for (i = 0; i < 16; i++)
		b[i] ^= x[i];
	jc2_aes128_ecb_be(sk_be, b, x);

	for (off = 0u; off < pt_len; off += 16u) {
		uint16_t rem = (uint16_t)(pt_len - off);

		memset(b, 0, sizeof(b));
		if (rem > 16u)
			rem = 16u;
		memcpy(b, &pt[off], rem);
		for (i = 0; i < 16; i++)
			b[i] ^= x[i];
		jc2_aes128_ecb_be(sk_be, b, x);
	}

	for (i = 0; i < 4; i++)
		out_mic4[i] = x[i] ^ s0[i];
}

void jc2_ble_ccm_encrypt(const uint8_t sk_be[16], const uint8_t iv8[8],
			 uint64_t pkt_cnt, bool master_to_slave, uint8_t hdr,
			 const uint8_t *pt, uint8_t pt_len, uint8_t *out_ct_mic)
{
	uint8_t nonce[13];
	uint8_t a[16];
	uint8_t s0[16];
	uint8_t mic[4];
	uint16_t blk_idx = 1u;
	uint16_t off;

	ble_build_nonce(pkt_cnt, master_to_slave, iv8, nonce);

	a[0] = 0x01u;
	memcpy(&a[1], nonce, 13);
	a[14] = 0u;
	a[15] = 0u;
	jc2_aes128_ecb_be(sk_be, a, s0);

	ble_ccm_mac(sk_be, nonce, hdr, pt, pt_len, s0, mic);

	for (off = 0u; off < pt_len; off += 16u, blk_idx++) {
		uint8_t ks[16];
		uint16_t rem = (uint16_t)(pt_len - off);
		uint16_t i;

		a[14] = (uint8_t)(blk_idx >> 8);
		a[15] = (uint8_t)(blk_idx & 0xFFu);
		jc2_aes128_ecb_be(sk_be, a, ks);
		if (rem > 16u)
			rem = 16u;
		for (i = 0u; i < rem; i++)
			out_ct_mic[off + i] = pt[off + i] ^ ks[i];
	}
	memcpy(&out_ct_mic[pt_len], mic, 4);
}

bool jc2_ble_ccm_decrypt(const uint8_t sk_be[16], const uint8_t iv8[8],
			 uint64_t pkt_cnt, bool master_to_slave, uint8_t hdr,
			 const uint8_t *ct_mic, uint8_t ct_mic_len,
			 uint8_t *out_pt)
{
	uint8_t pt_len;
	uint8_t nonce[13];
	uint8_t a[16];
	uint8_t s0[16];
	uint8_t mic[4];
	uint16_t blk_idx = 1u;
	uint16_t off;
	uint8_t diff = 0u;
	int i;

	if (ct_mic_len < 4u)
		return false;
	pt_len = (uint8_t)(ct_mic_len - 4u);
	ble_build_nonce(pkt_cnt, master_to_slave, iv8, nonce);

	a[0] = 0x01u;
	memcpy(&a[1], nonce, 13);
	a[14] = 0u;
	a[15] = 0u;
	jc2_aes128_ecb_be(sk_be, a, s0);

	for (off = 0u; off < pt_len; off += 16u, blk_idx++) {
		uint8_t ks[16];
		uint16_t rem = (uint16_t)(pt_len - off);
		uint16_t j;

		a[14] = (uint8_t)(blk_idx >> 8);
		a[15] = (uint8_t)(blk_idx & 0xFFu);
		jc2_aes128_ecb_be(sk_be, a, ks);
		if (rem > 16u)
			rem = 16u;
		for (j = 0u; j < rem; j++)
			out_pt[off + j] = ct_mic[off + j] ^ ks[j];
	}

	ble_ccm_mac(sk_be, nonce, hdr, out_pt, pt_len, s0, mic);
	for (i = 0; i < 4; i++)
		diff |= (uint8_t)(mic[i] ^ ct_mic[pt_len + i]);
	return diff == 0u;
}
