#include "input_driver.h"
#include "drv_xinput_ble.h"

// Which source drives each bond slot. SRC_RF at boot (matches the legacy behavior for every RF-bonded slot);
// ble_host flips a slot to SRC_BLE for the lifetime of a BLE connection.
volatile uint8_t g_slotSrc[NSLOT] = { SRC_RF, SRC_RF, SRC_RF, SRC_RF };

// The driver registry, first match wins. Order specific -> generic if a catch-all is ever added.
static IInputDriver *const REGISTRY[] = {
	&g_drvXInputBle,
};

IInputDriver *inputDriverFor(uint16_t vid, uint16_t pid, const char *devName)
{
	for (unsigned i = 0; i < sizeof REGISTRY / sizeof REGISTRY[0]; i++)
		if (REGISTRY[i]->match(vid, pid, devName ? devName : ""))
			return REGISTRY[i];
	return nullptr;
}
