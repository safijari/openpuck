// tusb_config.h — TinyUSB device configuration for PicoPuck.
//
// Device-only. The puck presents four HID interfaces (one per bond slot) plus a
// WebUSB vendor interface for the config panel, and an optional CDC console on
// debug builds. Not compiled until tinyusb_device is linked (Phase 1).
//
// SPDX-License-Identifier: MIT (TinyUSB config template) / AGPL-3.0-or-later
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#include "config/picopuck_config.h"  // PP_NSLOT

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

// The Pico SDK sets CFG_TUSB_OS (OPT_OS_PICO) on the command line; only default
// it if building outside that environment.
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#ifndef CFG_TUD_MEM_SECTION
#define CFG_TUD_MEM_SECTION
#endif
#ifndef CFG_TUD_MEM_ALIGN
#define CFG_TUD_MEM_ALIGN __attribute__((aligned(4)))
#endif

#define CFG_TUD_ENDPOINT0_SIZE 64

// ---- class driver counts ---------------------------------------------------
// Four puck slot interfaces, each a HID instance sharing the cloned puck report
// descriptor (input 0x40-0x45/0x79/0x7B, output 0x80-0x89, feature 0x01/0x02).
#define CFG_TUD_HID PP_NSLOT
#define CFG_TUD_VENDOR 1  // WebUSB config channel

// Optional CDC debug console (debug builds). Kept off by default; the endpoint
// budget (4 HID IN + 2 vendor + 3 CDC) stays within RP2040/RP2350 limits.
#ifdef PICOPUCK_DEBUG_CDC
#define CFG_TUD_CDC 1
#else
#define CFG_TUD_CDC 0
#endif
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0

// ---- buffer sizes ----------------------------------------------------------
#define CFG_TUD_HID_EP_BUFSIZE 64        // 64-byte feature/output reports
#define CFG_TUD_VENDOR_RX_BUFSIZE 64
#define CFG_TUD_VENDOR_TX_BUFSIZE 512    // status/scan frames fit in one FIFO
#ifndef CFG_TUD_CDC_RX_BUFSIZE
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256
#endif

#ifdef __cplusplus
}
#endif

#endif // _TUSB_CONFIG_H_
