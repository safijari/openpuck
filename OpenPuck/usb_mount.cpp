#include "usb_mount.h"
#include "bonds.h"
#include "input_driver.h" // slotIsBle() -- BLE-driven slots mount like RF ones
#include "fault_diag.h" // faultDiagTrace() -- flight recorder
#include <Arduino.h>

uint8_t g_usbMountCount = 0;
int8_t g_usbToBond[NSLOT];
int8_t g_bondToUsb[NSLOT];

static bool g_dynOn = false;
static uint8_t g_maxSlots = NSLOT;

// A controller counts as "connected" (mount-worthy) while it has replied recently. More lenient than the
// 300ms link-up check so a brief RF gap doesn't unmount it; the debounce below adds further hysteresis.
#define CONN_UP_MS 1200u
// The connected set must hold steady this long before we re-enumerate -- absorbs RF blips and the staggered
// way multiple controllers connect at boot (one re-enumeration once the set settles, not one per controller).
#define MOUNT_DEBOUNCE_MS 2000u
// Lazy removal: a disconnected slot's interface is only dropped (and the device re-enumerated down) once NO
// controller has sent input for this long -- i.e. during genuine downtime. Re-enumeration tears down/rebuilds
// USB from the loop task and is risky to do mid-use, so we never shrink while anything is being played; we
// just let dead slots linger as idle interfaces and clean them up when the rig goes quiet. Additions are NOT
// gated by this -- a newly-connected controller grows the set right away.
#define IDLE_CLEANUP_MS 60000u

static uint8_t connectedMask()
{
	unsigned long now = millis();
	uint8_t m = 0;
	// a slot is mount-worthy if something drives it: an RF bond (g_slot.used) or a live BLE claim --
	// both stamp g_connReplyMs, so the same freshness window covers either source
	for (int s = 0; s < NSLOT; s++)
		if ((g_slot[s].used || slotIsBle(s)) && g_connReplyMs[s] != 0 &&
		    (now - g_connReplyMs[s]) < CONN_UP_MS)
			m |= (uint8_t)(1u << s);
	return m;
}

static uint8_t mountedMask()
{
	uint8_t m = 0;
	for (uint8_t u = 0; u < g_usbMountCount; u++)
		if (g_usbToBond[u] >= 0)
			m |= (uint8_t)(1u << g_usbToBond[u]);
	return m;
}

// Compute the order-preserving, budget-capped mount list for `mask` into out[], returning the count. Ordering
// follows WHEN each controller was mounted: controllers already mounted keep their current USB-slot order (a
// departure compacts the rest down but never reorders survivors), and newly-connected controllers are appended
// at the end (ascending bond order among simultaneous joiners). Does NOT mutate the live map.
static uint8_t computeOrder(uint8_t mask, int8_t out[NSLOT])
{
	uint8_t n = 0;
	// 1. keep currently-mounted controllers that are still connected, in their existing order
	for (uint8_t u = 0; u < g_usbMountCount && n < g_maxSlots; u++) {
		int8_t b = g_usbToBond[u];
		if (b >= 0 && (mask & (1u << b)))
			out[n++] = b;
	}
	// 2. append newly-connected controllers (not already mounted)
	for (int s = 0; s < NSLOT && n < g_maxSlots; s++) {
		if (!(mask & (1u << s)))
			continue;
		bool have = false;
		for (uint8_t i = 0; i < n; i++)
			if (out[i] == (int8_t)s) {
				have = true;
				break;
			}
		if (!have)
			out[n++] = (int8_t)s;
	}
	return n;
}
static uint8_t maskOfList(const int8_t *list, uint8_t n)
{
	uint8_t m = 0;
	for (uint8_t i = 0; i < n; i++)
		if (list[i] >= 0)
			m |= (uint8_t)(1u << list[i]);
	return m;
}
static void commitOrder(const int8_t *list, uint8_t n)
{
	for (int i = 0; i < NSLOT; i++) {
		g_usbToBond[i] = -1;
		g_bondToUsb[i] = -1;
	}
	for (uint8_t u = 0; u < n; u++) {
		g_usbToBond[u] = list[u];
		g_bondToUsb[list[u]] = (int8_t)u;
	}
	g_usbMountCount = n;
}

void usbMountRebuildMap()
{
	int8_t tmp[NSLOT];
	uint8_t n = computeOrder(connectedMask(), tmp);
	commitOrder(tmp, n);
}

void usbMountEnable(bool on, uint8_t maxSlots)
{
	g_dynOn = on;
	g_maxSlots = maxSlots ? (maxSlots <= NSLOT ? maxSlots : NSLOT) : 1;
	for (int i = 0; i < NSLOT; i++)
		g_usbToBond[i] = g_bondToUsb[i] = -1;
	g_usbMountCount = 0;
}

void usbMountTask()
{
	if (!g_dynOn)
		return;
	static uint8_t lastMask = 0xFF; // force first compare
	static unsigned long stableSince = 0;
	unsigned long now = millis();
	// ADD IMMEDIATELY, REMOVE LAZILY. Re-enumeration (usbReenumerate) tears down + rebuilds the whole USB
	// config from the loop task; doing that while the high-priority usbd task is mid-tud_task races its
	// internal state -> HardFault/reboot. So we only ever SHRINK the interface set during genuine downtime:
	//   - active (some controller sent input within IDLE_CLEANUP_MS): target = connected UNION already-mounted
	//     -> the set only grows. A controller that drops (e.g. an errant ReversePuck disconnect) keeps its
	//     now-idle interface and reconnecting reuses it with NO re-enumeration. task() gates input per slot,
	//     so an idle interface just streams nothing.
	//   - idle (no controller input for a full minute): target = exactly the connected set -> any lingering
	//     dead slot is dropped and we re-enumerate down, safely, because nobody is playing.
	// New controllers grow the set right away (after the usual debounce) regardless of idle state.
	unsigned long lastAny = 0;
	for (int s = 0; s < NSLOT; s++)
		if ((g_slot[s].used || slotIsBle(s)) &&
		    g_connReplyMs[s] > lastAny)
			lastAny = g_connReplyMs[s];
	bool idle = (lastAny == 0) || (now - lastAny) >= IDLE_CLEANUP_MS;
	uint8_t want = idle ? connectedMask() :
			      (uint8_t)(connectedMask() | mountedMask());
	if (want != lastMask) { // set is moving -> restart the stability timer
		lastMask = want;
		stableSince = now;
		return;
	}
	// `want` has been steady since stableSince. Re-enumerate once it has held long enough AND the resulting
	// (order-preserving, capped) mount SET differs from what is currently mounted. With the union above,
	// while active that can only happen when a NEW controller needs an interface (grow); a shrink only ever
	// comes from the idle branch dropping a long-dead slot.
	if ((now - stableSince) < MOUNT_DEBOUNCE_MS)
		return;
	int8_t tmp[NSLOT];
	uint8_t n = computeOrder(want, tmp);
	if (maskOfList(tmp, n) == mountedMask())
		return; // same set already mounted (order is preserved across joins/leaves)
	commitOrder(tmp, n);
	faultDiagTrace(FR_MOUNT, g_usbMountCount);
	usbReenumerate(g_usbMountCount);
}
