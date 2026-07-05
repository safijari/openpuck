// Staged firmware update: WebUSB streams the new app image into unused upper app-region flash; a verified
// meta-page commit arms it; the next boot applies it from RAM. See fw_update.h for the protocol and the
// corruption-safety invariant every step here preserves.
#include "fw_update.h"
#include "fault_diag.h"
#include <Arduino.h>
#include <string.h>

// nRF52840 flash map (Adafruit UF2 bootloader layout): MBR 0x0-0x1000, SoftDevice to 0x26000, app region
// 0x26000-0xED000, internal LittleFS 0xED000-0xF4000, bootloader 0xF4000+, its settings page at 0xFF000.
// We manage the TOP of the app region: the meta/commit page at 0xEC000 and the staged image right below it,
// growing downward. Nothing outside [APP_BASE, 0xED000) is ever written except FWUP_BL_SETTINGS (see apply).
#define FWUP_APP_BASE 0x26000UL
#define FWUP_APP_END 0xED000UL
#define FWUP_META 0xEC000UL
#define FWUP_BL_SETTINGS 0xFF000UL
#define FWUP_PAGE 4096UL
// Image cap. Also what makes the apply copy safe from self-overlap: dst ends at most at 0x26000+0x60000 =
// 0x86000, while the staged source starts at (0xEC000-size)&~0xFFF >= 0x8C000 -- disjoint by >=24 KiB.
#define FWUP_MAX_IMG 0x60000UL

// Capability tag, searched for BY THE PANEL inside any .uf2 it is about to flash: an image without this
// exact string predates panel updates, so flashing it silently locks future updates back to UF2-DFU
// drag-and-drop -- the panel warns before letting that happen. Bump the suffix only on a breaking protocol
// change (the panel searches for this exact value). The asm reference below keeps it through --gc-sections.
static const char FWUP_TAG[] = "OPK-FWUP-v1";

#define FWUP_META_MAGIC 0x32465055UL // "UPF2"
struct FwupMeta {
	uint32_t magic, size, crc, staged;
	uint32_t metaCrc; // CRC32 over the four fields above
};
static_assert(sizeof(FwupMeta) == 20, "FwupMeta layout");

// end of the flash the RUNNING image occupies: code/rodata (__etext) plus the .data init image loaded right
// after it (.data is the last flash-loaded section in the core's linker script). Staging must stay above it.
extern "C" {
extern uint32_t __etext[];
extern uint32_t __data_start__[];
extern uint32_t __data_end__[];
}
static uint32_t flashUsedEnd(void)
{
	return (uint32_t)__etext +
	       ((uint32_t)__data_end__ - (uint32_t)__data_start__);
}

// ---- CRC32 (IEEE, reflected, init/xorout 0xFFFFFFFF) -- must match the panel's JS implementation ----------
static uint32_t crc32Step(uint32_t crc, const volatile uint8_t *p, uint32_t n)
{
	while (n--) {
		crc ^= *p++;
		for (int k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1)));
	}
	return crc;
}
// CRC a flash range. Beats the loop-heartbeat between blocks: verifying a full-size image takes ~200 ms of
// CPU, and without beats the fault_diag live-wedge reporter (>300 ms stall) would push an unsolicited status
// blob into the IN pipe right when the panel is waiting for the 0x22 ack.
static uint32_t crc32Flash(uint32_t addr, uint32_t len)
{
	uint32_t crc = 0xFFFFFFFFUL;
	while (len) {
		uint32_t blk = len > 16384 ? 16384 : len;
		crc = crc32Step(crc, (const volatile uint8_t *)addr, blk);
		addr += blk;
		len -= blk;
		faultDiagBeat();
	}
	return ~crc;
}

// ---- NVMC helpers (flash-resident; used for staging + meta only, never on the live app region) ------------
// NVMC ops halt the CPU (page erase ~85 ms) -- callers keep it to at most one erase per loop() pass so the
// 8 s watchdog and the RF link only see brief hiccups.
static void nvmcErasePage(uint32_t page)
{
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
	while (!NRF_NVMC->READY) {
	}
	NRF_NVMC->ERASEPAGE = page;
	while (!NRF_NVMC->READY) {
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (!NRF_NVMC->READY) {
	}
}
static void nvmcWriteWords(uint32_t addr, const uint32_t *w, uint32_t nWords)
{
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
	while (!NRF_NVMC->READY) {
	}
	for (uint32_t i = 0; i < nWords; i++) {
		((volatile uint32_t *)addr)[i] = w[i];
		while (!NRF_NVMC->READY) {
		}
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (!NRF_NVMC->READY) {
	}
}

// staged image must start with a plausible Cortex-M vector table: initial SP word-aligned in RAM (256 KiB on
// the nRF52840), reset vector Thumb within the app region. Last line of defense against arming a non-app blob.
static bool vectorsPlausible(uint32_t imgAddr, uint32_t size)
{
	if (size < 8)
		return false;
	uint32_t sp = ((const volatile uint32_t *)imgAddr)[0];
	uint32_t rv = ((const volatile uint32_t *)imgAddr)[1];
	return sp >= 0x20000000UL && sp <= 0x20040000UL && (sp & 3) == 0 &&
	       (rv & 1) && rv >= FWUP_APP_BASE && rv < FWUP_APP_END;
}

// ---- staging state (transfer is strictly sequential; every ack carries s_off so the panel can resync) -----
static bool s_active = false;
static uint32_t s_size, s_crc, s_base, s_off, s_erased;

uint32_t fwupNextOff(void)
{
	return s_off;
}

uint8_t fwupBegin(uint32_t size, uint32_t crc32)
{
	s_active = false;
	// any BEGIN first disarms a previously staged update -- a restarted transfer must never leave a stale
	// commit behind (the staged bytes it points at are about to be overwritten).
	nvmcErasePage(FWUP_META);
	if (size == 0 || size > FWUP_MAX_IMG || (size & 3))
		return FWUP_ERR_BOUNDS;
	uint32_t base = (FWUP_META - size) & ~(FWUP_PAGE - 1);
	// keep a page of slack above the running image so staging can NEVER touch live code/rodata/.data-init
	if (base < flashUsedEnd() + FWUP_PAGE)
		return FWUP_ERR_BOUNDS;
	s_size = size;
	s_crc = crc32;
	s_base = base;
	s_off = 0;
	s_erased =
		base; // pages erased lazily, one per chunk at most (85 ms CPU stall each)
	s_active = true;
	return FWUP_OK;
}

uint8_t fwupChunk(uint32_t off, const uint8_t *d, uint8_t len)
{
	if (!s_active)
		return FWUP_ERR_STATE;
	if (len == 0 || len > 128 || (len & 3) || off + len > s_size)
		return FWUP_ERR_BOUNDS;
	if (off != s_off) {
		// duplicate of already-written data (panel retried a chunk whose ack got lost): don't rewrite
		// the words (nRF52 flash allows only 2 writes per word between erases) -- just re-ack.
		if (off < s_off && off + len <= s_off)
			return FWUP_OK;
		return FWUP_ERR_OFFSET; // gap/desync: ack carries s_off, panel resends from there
	}
	while (s_erased < s_base + off + len) {
		nvmcErasePage(s_erased);
		s_erased += FWUP_PAGE;
	}
	uint32_t w[32]; // chunk arrives unaligned in the command buffer; NVMC wants word writes
	memcpy(w, d, len);
	nvmcWriteWords(s_base + off, w, len / 4);
	s_off = off + len;
	return FWUP_OK;
}

uint8_t fwupEnd(void)
{
	if (!s_active) {
		// duplicate END (the panel retried because our ack got lost): if the commit it is asking about
		// is exactly the one already on the meta page, re-report success instead of ERR_STATE.
		const volatile FwupMeta *m =
			(const volatile FwupMeta *)FWUP_META;
		if (s_size && m->magic == FWUP_META_MAGIC &&
		    m->size == s_size && m->crc == s_crc)
			return FWUP_OK;
		return FWUP_ERR_STATE;
	}
	if (s_off != s_size)
		return FWUP_ERR_STATE;
	s_active = false;
	// verify what actually landed in flash, not what we were sent
	if (crc32Flash(s_base, s_size) != s_crc)
		return FWUP_ERR_CRC;
	if (!vectorsPlausible(s_base, s_size))
		return FWUP_ERR_VECTOR;
	// COMMIT: this meta page write is the single arming action of the whole protocol
	FwupMeta m = { FWUP_META_MAGIC, s_size, s_crc, s_base, 0 };
	m.metaCrc = ~crc32Step(0xFFFFFFFFUL, (const uint8_t *)&m, 16);
	nvmcErasePage(FWUP_META);
	nvmcWriteWords(FWUP_META, (const uint32_t *)&m, 5);
	return FWUP_OK;
}

void fwupAbort(void)
{
	s_active = false;
	nvmcErasePage(FWUP_META);
}

// ---- boot-time apply ---------------------------------------------------------------------------------------
// Runs entirely from RAM (.data section => copied out of flash by the startup code) with interrupts off: it
// erases and rewrites the very flash the rest of the firmware executes from, so from the first erase to the
// reset it must not touch ANY flash-resident code or data -- only registers, its arguments, and the two
// flash regions it copies between. No calls, no libc, volatile pointers so the compiler can't lift the loops
// into memcpy/memcmp libcalls.
//
// Power-cut safety is in the ordering: all target pages are erased up front (the vector page first), pages
// 1..N-1 are written forward, then the vector page is written BACKWARD so word 0 -- the app-validity marker
// the bootloader checks -- is the final word written anywhere in the app. Interrupt the sequence at any
// point and the board reads as app-less: the UF2 bootloader keeps it, drag-and-drop recovers it.
#define FWUP_RAMFUNC \
	__attribute__((used, noinline, section(".data.opk_ramfunc")))
FWUP_RAMFUNC static void ramApply(uint32_t dst, uint32_t src, uint32_t size,
				  uint32_t metaPage, uint32_t blSettings)
{
	volatile uint32_t *d = (volatile uint32_t *)dst;
	const volatile uint32_t *s = (const volatile uint32_t *)src;
	const uint32_t words = size / 4;
	const uint32_t pg0w = words < FWUP_PAGE / 4 ? words : FWUP_PAGE / 4;

	// 1) erase every target page; dst itself (the vector page) goes first
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
	while (!NRF_NVMC->READY) {
	}
	for (uint32_t a = dst; a < dst + size; a += FWUP_PAGE) {
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		NRF_NVMC->ERASEPAGE = a;
		while (!NRF_NVMC->READY) {
		}
	}
	// 2) write pages 1..N-1 forward, then the vector page backward (word 0 dead-last)
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
	while (!NRF_NVMC->READY) {
	}
	for (uint32_t w = FWUP_PAGE / 4; w < words; w++) {
		if ((w & 1023) == 0)
			NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		d[w] = s[w];
		while (!NRF_NVMC->READY) {
		}
	}
	for (uint32_t w = pg0w; w-- > 0;) {
		d[w] = s[w];
		while (!NRF_NVMC->READY) {
		}
	}
	// 3) verify the copy word-for-word against the staged source
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (!NRF_NVMC->READY) {
	}
	uint32_t bad = 0;
	for (uint32_t w = 0; w < words; w++) {
		if ((w & 4095) == 0)
			NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		if (d[w] != s[w]) {
			bad = 1;
			break;
		}
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
	while (!NRF_NVMC->READY) {
	}
	if (bad) {
		// flash cell failure: make the app unambiguously invalid (bootloader keeps the board) instead
		// of resetting into a half-good image. Meta is erased below so this can't loop.
		NRF_NVMC->ERASEPAGE = dst;
		while (!NRF_NVMC->READY) {
		}
	} else {
		// 4) bootloader settings: leave the exact "valid app, CRC check disabled" state drag-and-drop
		// (MSC) flashing leaves. A previous adafruit-nrfutil serial-DFU flash records a real CRC16 here,
		// and the bootloader re-checks it EVERY boot -- stale, it would fail our new image straight into
		// DFU mode. Layout = SDK11 bootloader_settings_t; word 0 ([bank_0=BANK_VALID_APP][bank_0_crc=0])
		// is written last so an interrupted settings write falls back to the erased-settings path.
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		NRF_NVMC->ERASEPAGE = blSettings;
		while (!NRF_NVMC->READY) {
		}
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
		while (!NRF_NVMC->READY) {
		}
		volatile uint32_t *bs = (volatile uint32_t *)blSettings;
		bs[6] = 0; // sd_image_start
		while (!NRF_NVMC->READY) {
		}
		bs[5] = 0; // app_image_size (bank_0_size is the authoritative one)
		while (!NRF_NVMC->READY) {
		}
		bs[4] = 0; // bl_image_size
		while (!NRF_NVMC->READY) {
		}
		bs[3] = 0; // sd_image_size
		while (!NRF_NVMC->READY) {
		}
		bs[2] = size; // bank_0_size
		while (!NRF_NVMC->READY) {
		}
		bs[1] = 0x000000FEUL; // bank_1 = BANK_ERASED
		while (!NRF_NVMC->READY) {
		}
		bs[0] = 0x00000001UL; // bank_0 = BANK_VALID_APP, bank_0_crc = 0 ("not used")
		while (!NRF_NVMC->READY) {
		}
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
		while (!NRF_NVMC->READY) {
		}
	}
	// 5) disarm: erase the meta page so the (old or new) firmware boots normally from here on
	NRF_NVMC->ERASEPAGE = metaPage;
	while (!NRF_NVMC->READY) {
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (!NRF_NVMC->READY) {
	}
	// 6) reset -- raw AIRCR write; NVIC_SystemReset() is inline but keep even its helpers out of the path
	__asm volatile("dsb 0xF" ::: "memory");
	*(volatile uint32_t *)0xE000ED0CUL = (0x5FAUL << 16) | (1UL << 2);
	for (;;) {
	}
}

void fwupApplyIfArmed(void)
{
	// materialize a reference to the capability tag so the linker's --gc-sections keeps it in the image
	__asm volatile("" ::"r"(FWUP_TAG));
	const volatile FwupMeta *m = (const volatile FwupMeta *)FWUP_META;
	if (m->magic != FWUP_META_MAGIC)
		return; // erased page (0xFFFFFFFF) = nothing armed: the common path, zero boot cost
	// snapshot, then validate EVERYTHING before touching any flash; anything off -> disarm and boot normally
	FwupMeta v = { m->magic, m->size, m->crc, m->staged, m->metaCrc };
	bool ok = v.metaCrc ==
		  (uint32_t)~crc32Step(0xFFFFFFFFUL, (const uint8_t *)&v, 16);
	ok = ok && v.size && v.size <= FWUP_MAX_IMG && (v.size & 3) == 0;
	ok = ok && v.staged >= FWUP_APP_BASE + FWUP_MAX_IMG &&
	     v.staged + v.size <= FWUP_META &&
	     (v.staged & (FWUP_PAGE - 1)) == 0;
	ok = ok && crc32Flash(v.staged, v.size) ==
			   v.crc; // staged bytes still intact?
	ok = ok && vectorsPlausible(v.staged, v.size);
	if (!ok) {
		nvmcErasePage(FWUP_META);
		return;
	}
	// Point of no return. Freeze the system: nothing may run from app flash once the copy starts. The WDT
	// is forced on first (it survives soft reset if the previous boot armed it, but not power-on), so even
	// a fault inside the copier resets into the safe app-less/bootloader state instead of hanging forever.
	__disable_irq();
	if (!NRF_WDT->RUNSTATUS) {
		NRF_WDT->CONFIG =
			(WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
			(WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
		NRF_WDT->CRV = 8UL * 32768UL - 1; // ~8 s, same as the app's own
		NRF_WDT->RREN = WDT_RREN_RR0_Msk;
		NRF_WDT->TASKS_START = 1;
	}
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;
	// call through a volatile pointer: the compiler must materialize the RAM address and may not inline the
	// copier back into flash-resident code
	void (*volatile apply)(uint32_t, uint32_t, uint32_t, uint32_t,
			       uint32_t) = ramApply;
	apply(FWUP_APP_BASE, v.staged, v.size, FWUP_META, FWUP_BL_SETTINGS);
	for (;;) {
	} // unreachable (ramApply resets)
}
