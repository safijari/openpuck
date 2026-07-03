#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

// One outbound ring per active HID destination. Only one USB mode is live at a time, and a mode presents at
// most CFG_TUD_HID HID interfaces plus the wake mouse, so a handful of channels covers every case. Channels are
// allocated lazily on first send to a given Adafruit_USBD_HID* and never freed (the object set is fixed for the
// session). The producer side runs under PRIMASK (loop enqueue); the consumer (usbTxDrain) runs from the
// usbtx task AND tud_hid_report_complete_cb, serialized by a guard flag inside usbTxDrain.
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

// Send at most one ready report per destination per call. After a send the endpoint stays busy until the host
// polls it, so the next report for that destination goes out from tud_hid_report_complete_cb (the instant the
// host reads it) -- that chaining is what paces each stream to the host's poll rate. The usbtx task calls this
// too, to kick a stream that's idle (no completion pending) and to keep non-busy destinations moving. A busy
// destination is simply skipped; it does not hold up the others.
void usbTxDrain(void)
{
	// Re-entrancy guard (defensive): usbTxPump only calls this from loop(), but sendReport below can yield,
	// so guard against a nested loop()-context call ever racing a channel's tail. The loser just skips.
	static volatile unsigned char draining;
	if (__atomic_test_and_set(&draining, __ATOMIC_ACQUIRE))
		return;
	for (int i = 0; i < TX_CHAN_MAX; i++) {
		TxChan *c = &g_chan[i];
		if (!c->hid)
			continue;
		uint8_t t = c->tail;
		if (t == c->head)
			continue; // empty
		if (!c->hid->ready())
			continue; // endpoint busy / not mounted -> retry on completion/next tick
		TxItem &it = c->ring[t];
		// sendReport copies into the endpoint buffer, so releasing the slot right after is safe.
		if (c->hid->sendReport(it.rid, it.data, it.len))
			c->tail = txNext(t);
	}
	__atomic_clear(&draining, __ATOMIC_RELEASE);
}

// Per-slot drain callbacks for non-HID senders (XInput endpoint, WebUSB blob). Registered at setup(), invoked
// from usbTxPump() (loop context). Fixed, tiny capacity; registration is setup-only (single-threaded).
#define TX_DRAIN_MAX 4
static usbTxDrainFn g_drainFns[TX_DRAIN_MAX];
static uint8_t g_nDrain;

void usbTxRegisterDrain(usbTxDrainFn fn)
{
	if (fn && g_nDrain < TX_DRAIN_MAX)
		g_drainFns[g_nDrain++] = fn;
}

// ---- priority-inversion guard (BLACK BOX capture 3, 2026-07-03) --------------------------------------------
// TinyUSB's nrf5x dcd claims the chip's ONE EasyDMA slot (dma_running test_and_set) and only a few
// instructions LATER writes TASKS_STARTEPIN -- and loop-context sends reach that window at TASK_PRIO_LOW.
// Preempted right there during a Steam OUT flood (0x82 haptic storm), the flag is held by a task that never
// runs again: the TASK_PRIO_HIGH usbd task livelocks re-deferring xact_out_dma around the never-started DMA
// (caught live: loopPC=start_dma:139, irqPC=xact_out_dma re-defer) -> USB dead -> watchdog. The fix needs no
// library patch: run loop's USB-TX windows at the usbd task's own priority. Equal priority = no preemption on
// interrupt exit = the claim+start pair completes atomically w.r.t. the usbd task. Depth-counted so nested
// boosted sections (pump -> drain callbacks) stay balanced; only ever used from the loop task.
static UBaseType_t g_boostSaved;
static uint8_t g_boostDepth;
void usbTxBoost(void)
{
	if (!g_boostDepth++) {
		g_boostSaved = uxTaskPriorityGet(NULL);
		vTaskPrioritySet(NULL, TASK_PRIO_HIGH);
	}
}
void usbTxUnboost(void)
{
	if (g_boostDepth && !--g_boostDepth)
		vTaskPrioritySet(NULL, g_boostSaved);
}

// Drain everything from loop() -- NOT from a SOF callback, a dedicated task, or report_complete. All three of
// those were tried and all fight the RF poll, which is a microsecond-precise busy-wait running in loop(): SOF
// got silently disabled by bus resets; the high-priority usbd task (report_complete) preempted the RF RX
// windows -> missed replies + usbd self-blocking on a full event queue -> controllers dropped; a separate
// low-priority task got starved under USB load -> latency. Draining in loop() is what stability-fix did and is
// the only context that does NOT jitter the RF timing: the send and the poll run sequentially in one task, so
// no send ever interrupts an RX window. The cost -- loop() can block if sendReport hits a full usbd event
// queue -- is handled by a deep CFG_TUD_TASK_QUEUE_SZ (so it never fills) rather than by moving sends off loop.
// Call once per loop() iteration.
void usbTxPump(void)
{
	// Whole pump (HID sends + vendor/WebUSB drains) runs boosted -- every dcd claim window in here is
	// then atomic against the usbd task (see usbTxBoost above).
	usbTxBoost();
	usbTxDrain();
	for (uint8_t i = 0; i < g_nDrain; i++)
		g_drainFns[i]();
	usbTxUnboost();
}
void usbTxBegin(void)
{
	// Nothing to arm: usbTxPump() is driven directly from loop(). Kept for call-site stability.
}
