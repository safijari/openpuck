#include "usb_mount.h"
#include "bonds.h"
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

static uint8_t connectedMask()
{
	unsigned long now = millis();
	uint8_t m = 0;
	for (int s = 0; s < NSLOT; s++)
		if (g_slot[s].used && g_connReplyMs[s] != 0 &&
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
	uint8_t want = connectedMask();
	if (want != lastMask) { // set is moving -> restart the stability timer
		lastMask = want;
		stableSince = now;
		return;
	}
	// `want` has been steady since stableSince. Re-enumerate once it has held long enough AND the resulting
	// (order-preserving, capped) mount SET differs from what is currently mounted.
	if ((now - stableSince) < MOUNT_DEBOUNCE_MS)
		return;
	int8_t tmp[NSLOT];
	uint8_t n = computeOrder(want, tmp);
	if (maskOfList(tmp, n) == mountedMask())
		return; // same set already mounted (order is preserved across joins/leaves)
	commitOrder(tmp, n);
	usbReenumerate(g_usbMountCount);
}
