#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

// One outbound ring per active HID destination. Only one USB mode is live at a time, and a mode presents at
// most CFG_TUD_HID HID interfaces plus the wake mouse, so a handful of channels covers every case. Channels are
// allocated lazily on first send to a given Adafruit_USBD_HID* and never freed (the object set is fixed for the
// session). The producer side runs under PRIMASK (loop AND the RF-decode path enqueue); the consumer is the
// single usbd task (tud_sof_cb), so head/tail need no further locking beyond the enqueue critical section.
#define TX_CHAN_MAX 8
#define TX_RING_N \
	4 // depth per destination; 250 Hz produced, ~1 kHz SOF drain -> stays shallow
#define TX_DATA_MAX \
	64 // CFG_TUD_HID_EP_BUFSIZE -- the largest report any mode sends (63 B + id)

struct TxItem {
	uint8_t rid;
	uint8_t len;
	uint8_t data[TX_DATA_MAX];
};
struct TxChan {
	Adafruit_USBD_HID *hid; // nullptr = free slot
	TxItem ring[TX_RING_N];
	volatile uint8_t head,
		tail; // head=next write, tail=next read; empty when equal
};
static TxChan g_chan[TX_CHAN_MAX];

static inline uint8_t txNext(uint8_t i)
{
	return (uint8_t)((i + 1) % TX_RING_N);
}

void usbTxHid(Adafruit_USBD_HID *hid, uint8_t rid, const void *data,
	      uint16_t len)
{
	if (!hid)
		return;
	if (len > TX_DATA_MAX)
		len = TX_DATA_MAX;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	// find this destination's channel, or claim a free one
	TxChan *c = nullptr;
	for (int i = 0; i < TX_CHAN_MAX; i++) {
		if (g_chan[i].hid == hid) {
			c = &g_chan[i];
			break;
		}
	}
	if (!c) {
		for (int i = 0; i < TX_CHAN_MAX; i++) {
			if (!g_chan[i].hid) {
				c = &g_chan[i];
				c->hid = hid;
				c->head = c->tail = 0;
				break;
			}
		}
	}
	if (c) {
		uint8_t h = c->head, nx = txNext(h);
		// full -> drop the oldest (advance tail), never the newest
		if (nx == c->tail)
			c->tail = txNext(c->tail);
		c->ring[h].rid = rid;
		c->ring[h].len = (uint8_t)len;
		if (len)
			memcpy(c->ring[h].data, data, len);
		c->head = nx;
	}
	__set_PRIMASK(pm);
}

// usbd task only (tud_sof_cb). Send at most one ready report per destination per SOF: after a send the
// endpoint stays busy until the host polls it (~1 ms), and SOF fires every ~1 ms, so this naturally paces each
// stream to the host's poll rate without ever blocking. A destination whose endpoint is busy is simply left
// for the next frame; it does not hold up the others.
void usbTxDrain(void)
{
	for (int i = 0; i < TX_CHAN_MAX; i++) {
		TxChan *c = &g_chan[i];
		if (!c->hid)
			continue;
		uint8_t t = c->tail;
		if (t == c->head)
			continue; // empty
		if (!c->hid->ready())
			continue; // endpoint busy / not mounted -> retry next SOF
		TxItem &it = c->ring[t];
		// sendReport copies into the endpoint buffer, so releasing the slot right after is safe.
		if (c->hid->sendReport(it.rid, it.data, it.len))
			c->tail = txNext(t);
	}
}

// Weak no-op default; mode_xinput overrides it to flush its raw custom-class IN endpoints from the usbd task.
extern "C" __attribute__((weak)) void usbTxDrainHook(void)
{
}

// TinyUSB dispatches tud_sof_cb from tud_task() on the usbd task once per USB (micro)frame. Overriding the weak
// default routes that into our drain. Keep it minimal -- it runs every ~1 ms.
extern "C" void tud_sof_cb(uint32_t frame_count)
{
	(void)frame_count;
	usbTxDrain();
	usbTxDrainHook();
}

void usbTxBegin(void)
{
	// SOF callbacks are off by default (they are high frequency); turn them on so usbTxDrain() runs.
	tud_sof_cb_enable(true);
}
