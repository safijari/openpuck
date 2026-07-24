// usb_tx.h — per-HID-instance report queue, drained from the main loop.
//
// TinyUSB's tud_hid_n_report() can only have one report in flight per instance;
// callers (personality, BLE input path) enqueue here and usb_tx_pump() sends as
// each instance reports ready. Oldest report is dropped when a ring is full, so
// a stalled host never blocks input. Mirrors OpenPuck's usbTxHid.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_USB_TX_H
#define PICOPUCK_USB_TX_H

#include <stdint.h>
#include <stdbool.h>

// Queue a report (report_id + body) for HID instance `inst`. Returns false if
// the report was dropped (ring full → oldest evicted, still returns true; false
// only for invalid args). Safe to call from the main loop.
bool usb_tx_hid(uint8_t inst, uint8_t report_id, const uint8_t *body, uint16_t len);

// Send queued reports for every instance that is ready. Call each loop.
void usb_tx_pump(void);

#endif // PICOPUCK_USB_TX_H
