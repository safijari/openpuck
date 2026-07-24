// usb_tx.c — per-HID-instance report queue (see usb_tx.h).
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "usb/usb_tx.h"
#include "config/picopuck_config.h"
#include "tusb.h"

#include <string.h>

#define TX_RING_DEPTH 8
#define TX_MAX_BODY 63  // largest puck report body (0x45 is 45 bytes)

typedef struct {
	uint8_t report_id;
	uint8_t len;
	uint8_t body[TX_MAX_BODY];
} tx_msg_t;

typedef struct {
	tx_msg_t msg[TX_RING_DEPTH];
	uint8_t head;  // next slot to write
	uint8_t tail;  // next slot to read
	uint8_t count;
} tx_ring_t;

static tx_ring_t s_ring[PP_NSLOT];

bool usb_tx_hid(uint8_t inst, uint8_t report_id, const uint8_t *body, uint16_t len)
{
	if (inst >= PP_NSLOT || len > TX_MAX_BODY)
		return false;

	tx_ring_t *r = &s_ring[inst];
	if (r->count == TX_RING_DEPTH) {
		// Drop oldest so a stalled host can't wedge fresh input.
		r->tail = (uint8_t)((r->tail + 1) % TX_RING_DEPTH);
		r->count--;
	}

	tx_msg_t *m = &r->msg[r->head];
	m->report_id = report_id;
	m->len = (uint8_t)len;
	if (len)
		memcpy(m->body, body, len);
	r->head = (uint8_t)((r->head + 1) % TX_RING_DEPTH);
	r->count++;
	return true;
}

void usb_tx_pump(void)
{
	if (!tud_mounted())
		return;

	for (uint8_t inst = 0; inst < PP_NSLOT; inst++) {
		tx_ring_t *r = &s_ring[inst];
		if (r->count == 0 || !tud_hid_n_ready(inst))
			continue;

		tx_msg_t *m = &r->msg[r->tail];
		if (tud_hid_n_report(inst, m->report_id, m->body, m->len)) {
			r->tail = (uint8_t)((r->tail + 1) % TX_RING_DEPTH);
			r->count--;
		}
		// If the send failed, leave it queued and retry next pump.
	}
}
