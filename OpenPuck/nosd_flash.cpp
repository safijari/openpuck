// nosd_flash.cpp -- SoftDevice-free flash for every OpenPuck board.
//
// OpenPuck never enables the SoftDevice (see nosd.h). But the Adafruit InternalFileSystem flash layer
// (core: libraries/InternalFileSytem/src/flash/flash_nrf5x.c) is hardcoded to the SoftDevice flash SVCs
// -- sd_flash_page_erase / sd_flash_write, plus sd_softdevice_is_enabled to pick sync-vs-async -- and is
// NOT gated by SOFTDEVICE_PRESENT, so it issues those SVCs unconditionally. With SVCALL_AS_NORMAL_FUNCTION
// (nosd.h) those degrade to plain extern calls; this file implements them directly on the NVMC. Because
// the SoftDevice is never running, sd_softdevice_is_enabled reports "disabled", so the caller treats every
// op as synchronous and never waits on a SoftDevice completion callback.
//
// On a classic Feather this replaces the disabled-SoftDevice flash path (equivalent, one fewer moving
// part); on a nosd board (Connect Kit) it is what makes flash writes work at all.

#include "nrf.h"

// ORIGIN(FLASH) from the linker script (nrf52_common.ld PROVIDEs it) -- the application base, which is
// board-specific: 0x1000 on a nosd board, 0x26000 behind an S140 v6, 0x27000 behind S140 v7. Writing below
// it would clobber the MBR and/or a resident SoftDevice, so it is the hard lower bound.
extern uint32_t __flash_arduino_start[];

// nRF52840 bootloader base (UF2 or Nordic Open DFU). Matches InternalFileSystem's own BOOTLOADER_ADDR, and
// the flash_nrf5x_write() guard, so we never erase/write into the bootloader or the pages above it.
#define NOSD_FLASH_BOOTLOADER 0x000F4000u

#define NOSD_NRF_ERROR_FORBIDDEN \
	15u // NRF_ERROR_FORBIDDEN; any non-zero makes the caller treat the op as failed

extern "C" {

static inline void nosd_nvmc_wait(void)
{
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
}

uint32_t sd_softdevice_is_enabled(uint8_t *p_enabled)
{
	if (p_enabled)
		*p_enabled =
			0; // OpenPuck never enables it -> flash ops run synchronously
	return 0; // NRF_SUCCESS
}

uint32_t sd_softdevice_disable(void)
{
	return 0; // NRF_SUCCESS (nothing to disable)
}

uint32_t sd_flash_page_erase(uint32_t page_number)
{
	uint32_t addr = page_number * 4096u; // page index -> byte address
	if (addr < (uint32_t)__flash_arduino_start ||
	    addr >= NOSD_FLASH_BOOTLOADER)
		return NOSD_NRF_ERROR_FORBIDDEN; // never touch MBR / a resident SoftDevice / the bootloader
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
	nosd_nvmc_wait();
	NRF_NVMC->ERASEPAGE = addr;
	nosd_nvmc_wait();
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	nosd_nvmc_wait();
	return 0; // NRF_SUCCESS
}

uint32_t sd_flash_write(uint32_t *p_dst, uint32_t const *p_src,
			uint32_t size /*words*/)
{
	uint32_t a = (uint32_t)p_dst;
	uint32_t end = a + size * 4u;
	if (a < (uint32_t)__flash_arduino_start ||
	    end > NOSD_FLASH_BOOTLOADER || end < a)
		return NOSD_NRF_ERROR_FORBIDDEN;
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
	nosd_nvmc_wait();
	for (uint32_t i = 0; i < size; i++) {
		p_dst[i] = p_src[i];
		nosd_nvmc_wait();
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	nosd_nvmc_wait();
	return 0; // NRF_SUCCESS
}

} // extern "C"
