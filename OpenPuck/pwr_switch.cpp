#include "pwr_switch.h"
#include "bonds.h" // NSLOT, g_slot, g_connReplyMs
#include "triton.h" // TB_STEAM, g_in
#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // USBDevice.mounted()

#define PWR_SWITCH_RELEASED ((PWR_SWITCH_ACTIVE) == HIGH ? LOW : HIGH)
#define PULSE_MS 300u // motherboard power-header pulse width
#define HOST_OFF_DEBOUNCE_MS \
	1500u // USB must read "not mounted" this long before we trust it
#define RETRIGGER_COOLDOWN_MS 5000u // minimum spacing between pulses

static bool s_pulseActive = false;
static unsigned long s_pulseStartMs = 0;
static unsigned long s_hostOffSinceMs = 0;
static unsigned long s_lastFireMs = 0;
static bool s_everFired = false;

// per-slot: with round-robin polling a shared static would get clobbered by other slots' reports
// between press and release (same reasoning as rf_link.cpp's own Steam-short-press wake detector)
static bool steamWasDown[NSLOT] = {};
static unsigned long steamDownMs[NSLOT] = {};

void pwrSwitchInit()
{
	pinMode(PWR_SWITCH_PIN, OUTPUT);
	digitalWrite(PWR_SWITCH_PIN, PWR_SWITCH_RELEASED);
}

static void firePulse()
{
	digitalWrite(PWR_SWITCH_PIN, PWR_SWITCH_ACTIVE);
	s_pulseActive = true;
	s_pulseStartMs = millis();
	s_lastFireMs = s_pulseStartMs;
	s_everFired = true;
}

void pwrSwitchTask()
{
	// non-blocking release of an in-flight pulse
	if (s_pulseActive && millis() - s_pulseStartMs >= PULSE_MS) {
		digitalWrite(PWR_SWITCH_PIN, PWR_SWITCH_RELEASED);
		s_pulseActive = false;
	}

	// Host-off debounce: USBDevice.mounted() must read false CONTINUOUSLY for HOST_OFF_DEBOUNCE_MS.
	// Resets the instant it reads true, so a genuine boot/enumeration can never look like "off".
	if (USBDevice.mounted()) {
		s_hostOffSinceMs = 0;
	} else if (s_hostOffSinceMs == 0) {
		s_hostOffSinceMs = millis();
	}
	bool hostOff = s_hostOffSinceMs != 0 &&
		       millis() - s_hostOffSinceMs >= HOST_OFF_DEBOUNCE_MS;

	for (int s = 0; s < NSLOT; s++) {
		if (!g_slot[s].used)
			continue;

		// A dropped link forces g_in[s] to zero (rf_link.cpp's own down-edge handling), which
		// would otherwise look like a same-frame button release here -- only trust g_in while
		// this slot is actually link-up (same 300ms threshold as anySlotLinkUp()).
		if (millis() - g_connReplyMs[s] >= 300u) {
			steamWasDown[s] = false;
			continue;
		}

		bool steamNow = (g_in[s].buttons & TB_STEAM) != 0;
		if (steamNow && !steamWasDown[s])
			steamDownMs[s] = millis();
		if (!steamNow && steamWasDown[s] &&
		    millis() - steamDownMs[s] < 1000u && hostOff &&
		    !s_pulseActive &&
		    (!s_everFired ||
		     millis() - s_lastFireMs >= RETRIGGER_COOLDOWN_MS)) {
			firePulse();
		}
		steamWasDown[s] = steamNow;
	}
}
