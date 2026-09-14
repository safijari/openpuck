#pragma once

#if defined(OPK_BOARD_MDK_USB_DONGLE)
// Makerdiary/GeeekPi MDK USB Dongle uses the Adafruit-format
// settings page and S140 6.1.1 layout expected by OpenPuck WebUSB.
#define OPK_HAS_ADAFRUIT_DFU 1
#elif defined(OPK_BOARD_MDBT50Q_CX_40)
// The staged updater rewrites an Adafruit-format bootloader settings page.
#define OPK_HAS_ADAFRUIT_DFU 0
#else
#define OPK_HAS_ADAFRUIT_DFU 1
#endif
