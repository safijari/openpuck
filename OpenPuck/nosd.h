// nosd.h -- compile OpenPuck SoftDevice-free on every board.
//
// This header is FORCE-INCLUDED into every translation unit (see the `-include` in the Makefile's
// USB_EXTRA_FLAGS / RP_USB_FLAGS), ahead of all other code, so it can reshape the SoftDevice surface
// globally regardless of which board (feather / raytac / XIAO / Connect Kit) is being built.
//
// Why: OpenPuck drives the radio bare-metal and NEVER enables the SoftDevice on ANY board. On the classic
// Feather/Pro-Micro path the S140 blob still physically sits in flash (for the boot handoff), and the app
// happens to work only because that disabled-but-present SoftDevice still services the flash SVCs the
// Adafruit InternalFileSystem calls. A board with no SoftDevice at all (the MakerDiary Connect Kit's nosd
// bootloader) has no SVC handler, so those calls fault on the first flash write (e.g. saving a bond during
// pairing). Making the whole codebase SoftDevice-free removes that latent dependency everywhere.
//
// Two moves:
//   1. #undef SOFTDEVICE_PRESENT -- the Adafruit core hardcodes -DSOFTDEVICE_PRESENT; dropping it here
//      gates OUT the SoftDevice-only code paths (the FreeRTOS port's sd_clock_hfclk_* / sd_nvic_* usage,
//      sd_app_evt_wait, ...) so OpenPuck's own bare-metal management (HFXO in the radio, CMSIS critical
//      sections) is used instead. GCC applies -D before -include, so this reliably removes the -D.
//   2. #define SVCALL_AS_NORMAL_FUNCTION -- turns Nordic's inline-SVC sd_* stubs into plain extern
//      declarations, so the few that remain referenced (sd_flash_write / sd_flash_page_erase /
//      sd_softdevice_is_enabled / sd_softdevice_disable) resolve to our direct-NVMC implementations in
//      nosd_flash.cpp instead of trapping into a (possibly absent) SoftDevice.

#undef SOFTDEVICE_PRESENT

#ifndef SVCALL_AS_NORMAL_FUNCTION
#define SVCALL_AS_NORMAL_FUNCTION 1
#endif
