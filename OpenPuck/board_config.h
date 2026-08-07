#pragma once

#if defined(OPK_BOARD_MDBT50Q_CX_40)
// The staged updater rewrites an Adafruit-format bootloader settings page.
#define OPK_HAS_ADAFRUIT_DFU 0
#else
#define OPK_HAS_ADAFRUIT_DFU 1
#endif

// Start of the application region in flash. The Feather/SuperMini builds run under the Adafruit core's
// S140 6.1.1 (app at 0x26000); Seeed's XIAO core ships S140 7.3.0, whose one-page-larger SoftDevice moves
// the app to 0x27000 (see nrf52840_s140_v7.ld). Everything above is unchanged: LittleFS still starts at
// 0xED000, bootloader at 0xF4000, its settings page at 0xFF000.
#if defined(OPK_BOARD_XIAO_NRF52840)
#define OPK_APP_BASE 0x27000UL
#else
#define OPK_APP_BASE 0x26000UL
#endif
