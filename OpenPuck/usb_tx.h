// usb_tx.h -- marshal device->host HID reports onto the TinyUSB "usbd" task.
//
// THE PROBLEM this fixes (see ARCHITECTURE / the usbd-queue-deadlock notes): TinyUSB's tud_task() runs in a
// dedicated high-priority "usbd" FreeRTOS task, but this firmware historically called hid.sendReport() (i.e.
// tud_hid_*_report) straight from the LOW-priority loop() task. When the nRF52 USBD's single shared EasyDMA is
// momentarily busy, that cross-task send takes TinyUSB's usbd_defer_func() path, which does a BLOCKING
// osal_queue_send on the device event queue. Under a comms burst the queue saturates, the loop task blocks
// there, stops feeding the ~8 s watchdog, and the MCU resets (the controller "disconnects").
//
// THE FIX: loop-context code never calls tud_* directly. It enqueues the report here (non-blocking, ISR/PRIMASK
// safe), and the actual sendReport() happens from tud_sof_cb() -- which TinyUSB dispatches ON THE usbd TASK
// once per USB frame (~1 ms). Every tud_* call is therefore back in the single context TinyUSB expects, so the
// blocking-defer path can never stall the loop. Reports are keyed by their Adafruit_USBD_HID* (NOT the HID
// instance index, which is mode-dependent and fragile -- see the "HID instance order vs dynamic mount" note).
#pragma once
#include <stdint.h>

class Adafruit_USBD_HID;

// Queue one HID report for `hid` to be sent from the usbd task. Safe from any context (loop or usbd). The
// payload is copied, so the caller's buffer can be reused immediately (tud_hid_*_report copies into the
// endpoint buffer too, so this preserves the original fire-and-forget semantics). Never blocks: if that
// destination's ring is full the oldest queued report is dropped (same policy as the haptic relay ring) --
// for a 250 Hz gamepad stream the newest state is what matters. `len` is clamped to the 64 B report ceiling.
void usbTxHid(Adafruit_USBD_HID *hid, uint8_t rid, const void *data,
	      uint16_t len);

// Arm the SOF-driven drain. Call once, after USBDevice.attach(), from setup(). Idempotent.
void usbTxBegin(void);

// Drain pending reports (one ready report per destination per call). usbd-task only -- invoked by tud_sof_cb.
void usbTxDrain(void);

// usbd-task drain hook for senders that don't go through Adafruit_USBD_HID (the XInput custom-class endpoint).
// tud_sof_cb calls this right after usbTxDrain(), on the usbd task. A weak no-op default lives in usb_tx.cpp;
// mode_xinput overrides it to flush its raw IN endpoints, so its gamepad report is also issued off the loop
// task. extern "C" so the override links regardless of translation unit.
extern "C" void usbTxDrainHook(void);
