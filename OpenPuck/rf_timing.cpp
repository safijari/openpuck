#include "rf_timing.h"

#include <nrf.h>

#define RF_TIMING_PPI_READY 13u
#define RF_TIMING_PPI_ADDRESS 14u
#define RF_TIMING_PPI_PAYLOAD 15u
#define RF_TIMING_PPI_END 16u
#define RF_TIMING_PPI_MASK                                             \
	((1u << RF_TIMING_PPI_READY) | (1u << RF_TIMING_PPI_ADDRESS) | \
	 (1u << RF_TIMING_PPI_PAYLOAD) | (1u << RF_TIMING_PPI_END))
#define RF_TIMING_SENTINEL 0xFFFFFFFFu
#define RF_TIMING_TICKS_PER_US 16u

struct RfTimingState {
	bool armed;
	uint32_t baseTicks;
	uint32_t decisionTicks;
	uint32_t addressAtDecisionTicks;
};

static RfTimingState g_rfTiming = {};
static bool g_rfTimingReady = false;
static bool g_rfTimingConflict = false;
static bool g_featureResponseTiming = false;

void rfTimingBeginFeatureResponse()
{
	g_featureResponseTiming = true;
}

void rfTimingEndFeatureResponse()
{
	g_featureResponseTiming = false;
}

static void rfTimingInitHardware()
{
	if (g_rfTimingReady || g_rfTimingConflict)
		return;

	// These channels are reserved for RF timing. If another subsystem owns any
	// of them, fall back to the ordinary RX window instead of stealing hardware.
	if ((NRF_PPI->CHEN & RF_TIMING_PPI_MASK) || NRF_TIMER3->SHORTS ||
	    NRF_TIMER3->INTENSET) {
		g_rfTimingConflict = true;
		return;
	}
	const uint8_t channels[] = { RF_TIMING_PPI_READY, RF_TIMING_PPI_ADDRESS,
				     RF_TIMING_PPI_PAYLOAD, RF_TIMING_PPI_END };
	for (uint8_t channel : channels) {
		if (NRF_PPI->CH[channel].EEP || NRF_PPI->CH[channel].TEP) {
			g_rfTimingConflict = true;
			return;
		}
	}

	NRF_TIMER3->TASKS_STOP = 1;
	NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit
			      << TIMER_BITMODE_BITMODE_Pos;
	NRF_TIMER3->PRESCALER = 0; // 16 MHz, 62.5 ns per tick.
	NRF_TIMER3->TASKS_CLEAR = 1;
	NRF_TIMER3->TASKS_START = 1;

	NRF_PPI->CH[RF_TIMING_PPI_READY].EEP =
		(uint32_t)&NRF_RADIO->EVENTS_READY;
	NRF_PPI->CH[RF_TIMING_PPI_READY].TEP =
		(uint32_t)&NRF_TIMER3->TASKS_CAPTURE[0];
	NRF_PPI->CH[RF_TIMING_PPI_ADDRESS].EEP =
		(uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
	NRF_PPI->CH[RF_TIMING_PPI_ADDRESS].TEP =
		(uint32_t)&NRF_TIMER3->TASKS_CAPTURE[1];
	NRF_PPI->CH[RF_TIMING_PPI_PAYLOAD].EEP =
		(uint32_t)&NRF_RADIO->EVENTS_PAYLOAD;
	NRF_PPI->CH[RF_TIMING_PPI_PAYLOAD].TEP =
		(uint32_t)&NRF_TIMER3->TASKS_CAPTURE[2];
	NRF_PPI->CH[RF_TIMING_PPI_END].EEP = (uint32_t)&NRF_RADIO->EVENTS_END;
	NRF_PPI->CH[RF_TIMING_PPI_END].TEP =
		(uint32_t)&NRF_TIMER3->TASKS_CAPTURE[3];
	NRF_PPI->CHENCLR = RF_TIMING_PPI_MASK;
	g_rfTimingReady = true;
}

bool rfTimingArm()
{
	if (!g_featureResponseTiming)
		return false;
	rfTimingInitHardware();
	if (!g_rfTimingReady || g_rfTiming.armed)
		return false;

	NRF_PPI->CHENCLR = RF_TIMING_PPI_MASK;
	for (uint8_t capture = 0; capture < 4; capture++)
		NRF_TIMER3->CC[capture] = RF_TIMING_SENTINEL;
	NRF_TIMER3->TASKS_CAPTURE[4] = 1;
	g_rfTiming.baseTicks = NRF_TIMER3->CC[4];
	g_rfTiming.decisionTicks = g_rfTiming.baseTicks;
	g_rfTiming.addressAtDecisionTicks = RF_TIMING_SENTINEL;
	g_rfTiming.armed = true;
	NRF_PPI->CHENSET = RF_TIMING_PPI_MASK;
	return true;
}

void rfTimingCaptureDecision()
{
	if (!g_rfTiming.armed)
		return;
	NRF_TIMER3->TASKS_CAPTURE[4] = 1;
	g_rfTiming.decisionTicks = NRF_TIMER3->CC[4];
	const uint32_t address = NRF_TIMER3->CC[1];
	if (address != RF_TIMING_SENTINEL &&
	    (int32_t)(g_rfTiming.decisionTicks - address) >= 0)
		g_rfTiming.addressAtDecisionTicks = address;
	else
		g_rfTiming.addressAtDecisionTicks = RF_TIMING_SENTINEL;
}

bool rfTimingBeginCompletionGuard(uint16_t maxAfterAddressUs,
				  uint32_t *deadlineTicks)
{
	if (!g_rfTiming.armed || !deadlineTicks || !maxAfterAddressUs ||
	    g_rfTiming.addressAtDecisionTicks == RF_TIMING_SENTINEL)
		return false;

	const uint32_t maxTicks =
		(uint32_t)maxAfterAddressUs * RF_TIMING_TICKS_PER_US;
	const uint32_t elapsed =
		g_rfTiming.decisionTicks - g_rfTiming.addressAtDecisionTicks;
	if (elapsed >= maxTicks)
		return false;
	*deadlineTicks = g_rfTiming.addressAtDecisionTicks + maxTicks;
	return true;
}

bool rfTimingCompletionDeadlineReached(uint32_t deadlineTicks)
{
	if (!g_rfTiming.armed)
		return true;
	NRF_TIMER3->TASKS_CAPTURE[4] = 1;
	return (int32_t)(NRF_TIMER3->CC[4] - deadlineTicks) >= 0;
}

bool rfTimingWaitForAcquisition(uint16_t afterReadyUs,
				uint16_t packetAfterAddressUs,
				uint16_t readyFailsafeFromBaseUs)
{
	if (!g_rfTiming.armed || !afterReadyUs || !packetAfterAddressUs ||
	    !readyFailsafeFromBaseUs ||
	    g_rfTiming.addressAtDecisionTicks != RF_TIMING_SENTINEL)
		return false;

	const uint32_t readyWindowTicks =
		(uint32_t)afterReadyUs * RF_TIMING_TICKS_PER_US;
	const uint32_t packetWindowTicks =
		(uint32_t)packetAfterAddressUs * RF_TIMING_TICKS_PER_US;
	const uint32_t readyFailsafeTicks =
		g_rfTiming.baseTicks +
		(uint32_t)readyFailsafeFromBaseUs * RF_TIMING_TICKS_PER_US;

	uint32_t ready = NRF_TIMER3->CC[0];
	if (ready != RF_TIMING_SENTINEL &&
	    (int32_t)(g_rfTiming.decisionTicks - (ready + readyWindowTicks)) >=
		    0)
		return false;

	while (!NRF_RADIO->EVENTS_END) {
		const uint32_t address = NRF_TIMER3->CC[1];
		if (address != RF_TIMING_SENTINEL) {
			const uint32_t packetDeadline =
				address + packetWindowTicks;
			while (!NRF_RADIO->EVENTS_END) {
				NRF_TIMER3->TASKS_CAPTURE[4] = 1;
				if ((int32_t)(NRF_TIMER3->CC[4] -
					      packetDeadline) >= 0)
					break;
			}
			break;
		}

		ready = NRF_TIMER3->CC[0];
		NRF_TIMER3->TASKS_CAPTURE[4] = 1;
		const uint32_t now = NRF_TIMER3->CC[4];
		if (ready != RF_TIMING_SENTINEL) {
			if ((int32_t)(now - (ready + readyWindowTicks)) >= 0)
				break;
		} else if ((int32_t)(now - readyFailsafeTicks) >= 0) {
			break;
		}
	}
	return true;
}

void rfTimingFinish()
{
	if (!g_rfTiming.armed)
		return;
	NRF_PPI->CHENCLR = RF_TIMING_PPI_MASK;
	g_rfTiming.armed = false;
}
