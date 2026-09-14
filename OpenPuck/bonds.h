// bonds.h -- the puck's bond slots + live link state.
//
// The real puck holds four controller bonds and exposes one HID control interface per slot (interface N owns
// slot N). We mirror that: g_slot[N] holds the bond record [8 uuid][16 serial] plus a staging buffer for that
// interface's pending feature-report reply (filled by puck_hid.cpp's command channel). Bonds persist to
// flash (bonds.bin). g_connReplyMs (per-slot, rf_link sets the active one) is the live RF link state, shared
// here so the USB feature handler can report per-slot connection status without depending on the RF layer.
#pragma once
#include <stdint.h>

#define NSLOT 4

// per-slot state (record = [8 uuid][16 serial]) + per-interface response staging
struct Slot {
	uint8_t rec[24];
	bool used;
	uint8_t resp[63];
	uint16_t resp_len;

	// Controller feature queries are serialized per slot. Failure state stays separate
	// from resp so a reject or timeout never destroys the last valid response. These
	// fields are runtime-only and are not persisted.
	volatile uint8_t

		// 0 = none; otherwise the query currently on-air
		pendingQueryCmd;
	volatile uint8_t

		// host GET stalls until a new query is queued
		pendingQueryFailed;
	volatile uint8_t

		// 0xAE selector distinguishes concurrent same-command replies
		pendingQuerySelector;
	volatile uint8_t pendingQuerySelectorValid;
	volatile uint32_t pendingQueryDeadlineMs;
	// Once Type-4 completes, keep that exact controller
	// response immutable until the host consumes it with a matching RID1 GET.
	volatile uint8_t queryResponseReady;
	volatile uint8_t queryResponseCmd;
	volatile uint8_t queryResponseSelector;
	volatile uint8_t queryResponseSelectorValid;
	// The newest response-bearing RID1 SET owns
	// queryHostGeneration; queued/on-air/ready stages carry that generation end-to-end.
	volatile uint32_t queryHostGeneration;
	volatile uint32_t pendingQueryGeneration;
	volatile uint32_t queryResponseGeneration;
	// A successfully queued host generation remains unresolved
	// until its matching ready response is consumed. This persists across the
	// relay-FIFO dequeue -> pendingQueryCmd RF-arm transition.
	volatile uint32_t queryConsumedGeneration;
	volatile uint32_t

		// actual 0x9F TX time; Type-2 ownership only
		shutdownStatusOwnerMs;
};
extern Slot g_slot[NSLOT];

// Per-slot link state. g_connReplyMs[i] is the millis() of the last F-type reply on slot i (the
// "controller is alive" timestamp). g_linkRssi (rf_link.h) is also per-slot. Steam reads each slot's
// B4/0x79/0x7B against its own entry here, so the four interfaces present four independent controllers
// (each marked connected on its own).
extern unsigned long g_connReplyMs[NSLOT];
extern volatile bool g_dirty; // bonds changed -> flush to flash from loop()
extern bool g_pairing;

bool recEmpty(const uint8_t *r);
void loadBonds();
void saveBonds();
int bondedSlotCount();
