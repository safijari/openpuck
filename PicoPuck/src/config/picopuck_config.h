// picopuck_config.h — compile-time constants shared across PicoPuck.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_CONFIG_H
#define PICOPUCK_CONFIG_H

// ---- puck presentation -----------------------------------------------------
// Number of controller "slots" the puck exposes. The real Valve puck enumerates
// four HID interfaces (one per bond slot); we always present all four so Steam
// sees a normal puck regardless of how many controllers are actually connected.
#define PP_NSLOT 4

// USB identity — identical to a real Valve puck so Steam treats us as one.
#define PP_USB_VID 0x28DE
#define PP_USB_PID 0x1304

// bcdDevice discriminates PicoPuck from the nRF OpenPuck (0x02xx) and the
// ReversePuck dongle (PID 0x1302) for the WebUSB panel and for Windows'
// per-(VID,PID,bcdDevice) descriptor cache (our interface layout differs).
//   0x03x0 = release, 0x03x1 = debug (CDC console) build.
#define PP_BCD_DEVICE 0x0310
#define PP_BCD_DEVICE_CDC 0x0311

// Board type reported in the WebUSB status frame.
#define PP_BOARD_PICO_W 1
#define PP_BOARD_PICO2_W 2

#if defined(PICO_RP2350) || defined(PICO_PLATFORM_RP2350)
#define PP_BOARD PP_BOARD_PICO2_W
#else
#define PP_BOARD PP_BOARD_PICO_W
#endif

// ---- cadences (ms) ---------------------------------------------------------
#define PP_STATUS_LED_MS 500      // heartbeat blink period while idle
#define PP_WATCHDOG_MS 4000       // hardware watchdog timeout

#endif // PICOPUCK_CONFIG_H
