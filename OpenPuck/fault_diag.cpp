#include "fault_diag.h"
#include <Arduino.h> // readResetReason(), NRF_POWER, NVIC_SystemReset, POWER_RESETREAS_*_Msk

// GPREGRET2 markers. GPREGRET (id 0) is reserved by the Adafruit bootloader for the DFU magic; GPREGRET2 is
// free for the application and is retained across soft/watchdog/pin reset, cleared only on power-on/brownout.
// Arbitrary non-zero tags -- any value distinct from each other and from the power-on default (0) works.
#define G2_INTENT 0xB1u
#define G2_FAULT 0xFAu

static uint8_t g_reason = RR_UNKNOWN;
static uint32_t g_resetReas = 0;

static const char *const REASON_STR[RR_COUNT] = {
	"unknown",   "power-on", "pin/replug", "WATCHDOG (hang)", "CPU lockup",
	"HARDFAULT", "reboot",	 "soft reset", "wake-from-off",
};

// HardFault override. The Adafruit core's default handler (cores/.../debug.cpp) already does NVIC_SystemReset();
// we replace it to first stamp the fault marker so the NEXT boot classifies this SREQ as RR_HARDFAULT rather
// than an intentional reboot. The core's strong symbol is only linked to satisfy the vector table, so our own
// strong definition takes its place. Keep this MINIMAL: we are in fault context on a possibly-corrupt stack --
// no Serial, no allocations, just stamp and reset.
extern "C" void HardFault_Handler(void)
{
	NRF_POWER->GPREGRET2 = G2_FAULT;
	NVIC_SystemReset();
	while (1) {
	}
}

void faultDiagArmIntentionalReset()
{
	NRF_POWER->GPREGRET2 = G2_INTENT;
}

void faultDiagBoot()
{
	uint32_t rr =
		readResetReason(); // latched + cleared by the core's init()
	uint8_t g2 = (uint8_t)NRF_POWER->GPREGRET2;
	NRF_POWER->GPREGRET2 = 0; // consume the marker for this boot cycle
	g_resetReas = rr;

	uint8_t reason;
	// Precedence: a physical pin reset / watchdog / lockup is unambiguous from RESETREAS; only a SREQ needs the
	// GPREGRET2 marker to split intentional reboot vs HardFault.
	if (rr & POWER_RESETREAS_RESETPIN_Msk)
		reason = RR_PIN;
	else if (rr & POWER_RESETREAS_DOG_Msk)
		reason = RR_WATCHDOG;
	else if (rr & POWER_RESETREAS_LOCKUP_Msk)
		reason = RR_LOCKUP;
	else if (rr & POWER_RESETREAS_SREQ_Msk)
		reason = (g2 == G2_FAULT)  ? RR_HARDFAULT :
			 (g2 == G2_INTENT) ? RR_REBOOT :
					     RR_SOFT;
	else if (rr & POWER_RESETREAS_OFF_Msk)
		reason = RR_WAKE;
	else if (rr == 0)
		reason = RR_POWERON; // all bits clear == power-on / brownout
	else
		reason = RR_UNKNOWN;
	g_reason = reason;

	Serial.printf(
		"# reset cause: %s (RESETREAS=0x%08lX gpregret2=0x%02X)\n",
		REASON_STR[reason], (unsigned long)rr, g2);
}

uint8_t faultDiagReason()
{
	return g_reason;
}
uint32_t faultDiagResetReas()
{
	return g_resetReas;
}
const char *faultDiagReasonStr()
{
	return REASON_STR[g_reason < RR_COUNT ? g_reason : 0];
}
