/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#ifndef JC2_PORT_H
#define JC2_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Platform primitives implemented by the active target backend
 * (e.g. jc2_ll_nrf52.c on nRF52840 or jc2_ll_loopback.c under JC2_HOST_TEST).
 * Separating these from portable core modules ensures an unsupported MCU fails
 * loudly at link time rather than silently compiling no-op locks or weak PRNGs.
 */

/* Mask/unmask all hardware IRQs for short critical sections (~10-20 insns). */
uint32_t jc2_port_irq_lock(void);
void jc2_port_irq_unlock(uint32_t key);

/*
 * Mask/unmask the Link-Layer bottom-half IRQ (e.g. priority-2 SWI0 on nRF52840)
 * while leaving priority-0 radio/timer IRQs completely unmasked.
 */
uint32_t jc2_port_bh_lock(void);
void jc2_port_bh_unlock(uint32_t key);

/* Data memory barrier for lock-free SPSC ring producers and consumers. */
void jc2_port_dmb(void);

/* Fill out[0..len-1] with random bytes (hardware TRNG on target MCUs). */
void jc2_port_random_bytes(uint8_t *out, size_t len);

/*
 * Optional hardware AES-128-ECB accelerator (big-endian key and block).
 * Returns true if ct_be was produced in hardware; returns false to fall back
 * to the portable software AES-128 implementation in jc2_crypto.c.
 */
bool jc2_port_aes128_ecb_be(const uint8_t key_be[16], const uint8_t pt_be[16],
			    uint8_t ct_be[16]);

#ifdef __cplusplus
}
#endif

#endif /* JC2_PORT_H */
