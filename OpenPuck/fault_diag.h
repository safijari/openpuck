// fault_diag.h -- "why did we reboot?" diagnostic for issue #72 (intermittent disconnects).
//
// Field reports conflate THREE different events that all look identical from the host (USB re-enumerates, the
// WebUSB panel shows "RF Link down"):
//   1. a genuine RF link drop -- NO reboot, the link self-recovers in seconds (handled in rf_link, not here);
//   2. a loop() hang -- the ~8s hardware watchdog fires and resets the MCU (red LED);
//   3. an MCU fault -- a HardFault triggers an immediate reset (the Adafruit core's default handler resets).
// Without recording which one happened, (2) and (3) are indistinguishable in the field, so the thread keeps
// guessing. This module classifies the cause of each boot and surfaces it (boot banner + WebUSB panel).
//
// How: the nRF52 RESETREAS register already distinguishes watchdog (DOG) / software (SREQ) / pin (RESETPIN) /
// lockup resets; the Adafruit core latches it at boot (readResetReason()). A SREQ alone can't tell "we rebooted
// on purpose" (mode-switch chord, config reboot, DFU) from "a HardFault reset us" -- so we stamp the GPREGRET2
// retention register (retained across soft/watchdog/pin reset, cleared only on power-on; GPREGRET itself is
// reserved by the bootloader for the DFU magic): faultDiagArmIntentionalReset() before every deliberate
// NVIC_SystemReset, and the HardFault handler stamps a distinct fault marker.
#pragma once
#include <stdint.h>

// Boot-cause classification (kept in sync with REASON_STR[] in fault_diag.cpp).
enum {
	RR_UNKNOWN = 0,
	RR_POWERON, // cold power-on / brownout
	RR_PIN, // reset pin / cable replug
	RR_WATCHDOG, // WDT fired -> loop() stopped feeding it (a hang)
	RR_LOCKUP, // CPU lockup
	RR_HARDFAULT, // HardFault -> our handler reset us
	RR_REBOOT, // intentional NVIC_SystemReset (mode switch / config / DFU)
	RR_SOFT, // software reset, unattributed
	RR_WAKE, // wake from System OFF
	RR_COUNT,
};

// Classify + log the cause of THIS boot. Call once, early in setup() (after Serial.begin so the line lands).
void faultDiagBoot();
// Stamp "the reset about to happen is intentional" -- call immediately before a deliberate NVIC_SystemReset so
// the next boot classifies it as RR_REBOOT, not RR_HARDFAULT.
void faultDiagArmIntentionalReset();
// Last-boot classification (RR_*) + the raw RESETREAS, for the WebUSB panel / console.
uint8_t faultDiagReason();
uint32_t faultDiagResetReas();
const char *faultDiagReasonStr();
