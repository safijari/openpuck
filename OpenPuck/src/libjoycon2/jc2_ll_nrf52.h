/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#ifndef JC2_LL_NRF52_H
#define JC2_LL_NRF52_H

#include "jc2_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * nRF52840 Bare-Metal BLE 5.0 Backend Contract & Resource Ownership
 *
 * Exclusive hardware resources claimed by jc2_ll_nrf52.c after jc2_ll_init():
 * - NRF_RADIO    (shared cooperatively via jc2_ll_acquire_gap / release_gap)
 * - NRF_TIMER0   (1 MHz free-running clock; CC[0..3] scheduler/watchdogs/PPI)
 * - NRF_PPI->CH0 (RADIO->EVENTS_ADDRESS -> TIMER0->TASKS_CAPTURE[2])
 * - NRF_ECB      (hardware AES-128-ECB guarded by BASEPRI = 2)
 * - NRF_RNG      (hardware TRNG for BLE SKDs / IVs)
 * - NVIC IRQs    (RADIO_IRQn @ prio 0, TIMER0_IRQn @ prio 0, SWI0_EGU0 @ prio 2)
 * - SCB->VTOR    (512 B aligned RAM copy when JC2_NRF52_USE_RAM_VTOR == 1)
 *
 * Secondary 2.4 GHz protocols sharing NRF_RADIO inside jc2_ll_acquire_gap()
 * should use NRF_TIMER1..4 and NRF_PPI->CH[1..19] to avoid conflicts.
 *
 * Build-time configuration knobs (override via -D flags before compiling):
 * - JC2_NRF52_USE_RAM_VTOR: 1 to relocate SCB->VTOR to a RAM vector table
 *   (default 1 on bare-metal/Arduino; default 0 when __ZEPHYR__ is defined).
 * - JC2_NRF52_USE_MBR: 1 to preserve Nordic MBR SVC_Handler from 0x0000002C
 *   in the RAM vector table (default 1 when JC2_NRF52_USE_RAM_VTOR == 1).
 * - JC2_NRF52_ISR_VECTOR_SYM: linker symbol name for the flash vector table
 *   copied into RAM (default __isr_vector).
 */
#ifndef JC2_NRF52_USE_RAM_VTOR
#if defined(__ZEPHYR__)
#define JC2_NRF52_USE_RAM_VTOR 0
#else
#define JC2_NRF52_USE_RAM_VTOR 1
#endif
#endif

#ifndef JC2_NRF52_USE_MBR
#define JC2_NRF52_USE_MBR 1
#endif

/*
 * Direct ISR entry points for RTOS / Zephyr integration (e.g. via
 * IRQ_DIRECT_CONNECT(RADIO_IRQn, 0, jc2_nrf52_radio_isr, 0)). On bare-metal
 * and Arduino builds (!__ZEPHYR__), RADIO_IRQHandler / TIMER0_IRQHandler /
 * SWI0_EGU0_IRQHandler forward directly to these functions.
 */
void jc2_nrf52_radio_isr(void);
void jc2_nrf52_timer0_isr(void);
void jc2_nrf52_swi0_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* JC2_LL_NRF52_H */
