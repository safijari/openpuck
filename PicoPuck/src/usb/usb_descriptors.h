// usb_descriptors.h — interface / endpoint layout and vendor request codes.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_USB_DESCRIPTORS_H
#define PICOPUCK_USB_DESCRIPTORS_H

#include "config/picopuck_config.h"

// Interface numbers. The four HID slot interfaces come first so slot N == HID
// instance N == interface N (Steam addresses a slot by interface index / wIndex).
// The WebUSB vendor interface follows; an optional CDC console is last.
enum {
	ITF_NUM_HID0 = 0,
	ITF_NUM_HID1,
	ITF_NUM_HID2,
	ITF_NUM_HID3,
	ITF_NUM_VENDOR,
#if CFG_TUD_CDC
	ITF_NUM_CDC,
	ITF_NUM_CDC_DATA,
#endif
	ITF_NUM_TOTAL
};

// Endpoint addresses. HID slots are IN-only (output/feature reports ride EP0
// SET_REPORT, as on the real puck). The vendor interface is a bulk IN/OUT pair.
#define EPNUM_HID0 0x81
#define EPNUM_HID1 0x82
#define EPNUM_HID2 0x83
#define EPNUM_HID3 0x84
#define EPNUM_VENDOR_OUT 0x05
#define EPNUM_VENDOR_IN 0x85
#if CFG_TUD_CDC
#define EPNUM_CDC_NOTIF 0x86
#define EPNUM_CDC_OUT 0x06
#define EPNUM_CDC_IN 0x87
#endif

// Vendor control-transfer request codes referenced by the BOS descriptor.
#define VENDOR_REQUEST_WEBUSB 1
#define VENDOR_REQUEST_MICROSOFT 2

#endif // PICOPUCK_USB_DESCRIPTORS_H
