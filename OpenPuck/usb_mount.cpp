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

// Rebuild the map from a bitmask of bond slots, in ascending bond-slot order (deterministic). Capped at the
// mode's HID budget so we never register more interfaces than CFG_TUD_HID allows.
static void buildMapFromMask(uint8_t mask)
{
	for (int i = 0; i < NSLOT; i++) {
		g_usbToBond[i] = -1;
		g_bondToUsb[i] = -1;
	}
	uint8_t u = 0;
	for (int s = 0; s < NSLOT && u < g_maxSlots; s++) {
		if (mask & (1u << s)) {
			g_usbToBond[u] = (int8_t)s;
			g_bondToUsb[s] = (int8_t)u;
			u++;
		}
	}
	g_usbMountCount = u;
}

void usbMountRebuildMap()
{
	buildMapFromMask(connectedMask());
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
	// `want` has been steady since stableSince. Re-enumerate once it has held long enough AND differs from
	// what is mounted (compare capped: a mask bit beyond g_maxSlots can't be mounted, so mask off the excess).
	if ((now - stableSince) < MOUNT_DEBOUNCE_MS)
		return;
	// cap `want` to the first g_maxSlots set bits so the comparison matches what we can actually mount
	uint8_t capped = 0, cnt = 0;
	for (int s = 0; s < NSLOT && cnt < g_maxSlots; s++)
		if (want & (1u << s)) {
			capped |= (uint8_t)(1u << s);
			cnt++;
		}
	if (capped == mountedMask())
		return; // already mounted exactly this set
	buildMapFromMask(want);
	usbReenumerate(g_usbMountCount);
}
