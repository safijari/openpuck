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

// Kept for call-site stability; no longer does anything (the drain runs from loop() via usbTxPump). Call once.
void usbTxBegin(void);

// Drain all pending sends (HID rings + registered non-HID drains). MUST be called from loop() once per
// iteration -- sends run in loop() context, sequentially with the RF poll, so they never interrupt its
// timing-critical RX windows (which is what off-loop drain contexts did, dropping controllers).
void usbTxPump(void);

// Drain pending HID reports (one ready report per destination per call). Internal -- called by usbTxPump().
void usbTxDrain(void);

// Priority-inversion guard: run a loop-context USB TX window (HID/vendor/CDC writes) at the usbd task's
// priority so TinyUSB's dcd DMA claim+start can't be preempted mid-claim (the issue-72 livelock; see
// usb_tx.cpp). Depth-counted; loop task only, always in a balanced pair. usbTxPump() boosts itself; wrap
// direct Serial/usb_web writes that can run at flood rate (e.g. per-relay logging).
void usbTxBoost(void);
void usbTxUnboost(void);

// Register a callback to run every USB frame ON THE usbd TASK (from tud_sof_cb, right after usbTxDrain()).
// This is how senders that DON'T go through Adafruit_USBD_HID get their transmits off the loop task too:
//   - mode_xinput flushes its raw custom-class IN endpoints;
//   - webusb_config sends the status blob (its usb_web.flush() can otherwise block the loop on a full queue).
// Call once at setup() (before usbTxBegin()). Safe no-op if usb isn't up yet. Capacity is small + fixed.
typedef void (*usbTxDrainFn)(void);
void usbTxRegisterDrain(usbTxDrainFn fn);
