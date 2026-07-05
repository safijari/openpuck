#include "rf_timeslot.h"
#include <Arduino.h>
#include <nrf_soc.h>
#include <nrf_sdm.h>

volatile bool g_sdEnabled = false;
volatile uint16_t g_tsGrants = 0, g_tsExtends = 0, g_tsStarved = 0;

// Slot geometry. 15ms base slots with 15ms chained extensions: long enough that the ~4.5ms worst-case ESB op
// fits many times per slot, short enough that the first grant after BLE activity is quick. The margin is how
// long before the true slot end rfRadioOwned() starts refusing new ops -- it must exceed the pessimistic gate
// the callers pass (they gate on the WHOLE op, so only the last-op-start needs to clear the margin).
#define TS_LEN_US 15000u
#define TS_EXT_US 15000u
#define TS_MARGIN_US 500u
// EARLIEST-request timeout: how far in the future the SD may schedule us before giving up (then rfTsTick
// re-arms). Generous -- heavy scanning can legitimately push us out several tens of ms.
#define TS_TIMEOUT_US 100000u

static volatile bool s_sessionOpen = false;
// micros() deadline until which the radio is ours (margin already subtracted). 0 = no slot. Written by the
// priority-0 signal handler, read from loop context -- single aligned 32-bit write, no lock needed.
static volatile uint32_t s_ownedUntil = 0;
// current slot length so far (base + granted extensions), for the TIMER0 end-guard compare
static uint32_t s_slotLen = 0;
static volatile uint32_t s_lastGrantUs =
	0; // last slot-start micros() (starvation watchdog)
static unsigned long s_lastKickMs = 0;

static nrf_radio_request_t s_req;
static nrf_radio_signal_callback_return_param_t s_ret;

static void tsBuildRequest()
{
	s_req.request_type = NRF_RADIO_REQ_TYPE_EARLIEST;
	s_req.params.earliest.hfclk = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
	s_req.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
	s_req.params.earliest.length_us = TS_LEN_US;
	s_req.params.earliest.timeout_us = TS_TIMEOUT_US;
}

// Timeslot signal handler. PRIORITY 0 -- preempts everything, including the SoftDevice's own lower-priority
// handlers. Keep it register-poke + volatile-stamp only: no SD calls except via the returned action, no RTOS,
// no logging. micros() is safe (DWT cycle counter read).
static nrf_radio_signal_callback_return_param_t *tsSignal(uint8_t sig)
{
	s_ret.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
	switch (sig) {
	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
		// The SD hands us the radio with ITS interrupt config -- clear it, we poll (no RADIO signals).
		NRF_RADIO->INTENCLR = 0xFFFFFFFF;
		s_slotLen = TS_LEN_US;
		s_lastGrantUs = micros();
		s_ownedUntil = s_lastGrantUs + TS_LEN_US - TS_MARGIN_US;
		g_tsGrants++;
		// TIMER0 runs from 0 (1 MHz) inside the slot. CC0 = the end guard: hand the slot back BEFORE the
		// scheduled end (overstaying is an SD assert). Extensions push it out.
		NRF_TIMER0->CC[0] = s_slotLen - (TS_MARGIN_US / 2);
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
		// the SD forwards the TIMER0 IRQ to this handler as SIGNAL_TYPE_TIMER0 -- it must be
		// NVIC-enabled (canonical Nordic timeslot pattern)
		NVIC_EnableIRQ(TIMER0_IRQn);
		// chain an extension right away; keeps the radio ours while BLE is idle
		s_ret.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
		s_ret.params.extend.length_us = TS_EXT_US;
		break;
	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
		s_slotLen += TS_EXT_US;
		s_ownedUntil += TS_EXT_US;
		NRF_TIMER0->CC[0] = s_slotLen - (TS_MARGIN_US / 2);
		g_tsExtends++;
		// keep chaining -- the SD fails the extend when a BLE event needs the radio
		s_ret.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
		s_ret.params.extend.length_us = TS_EXT_US;
		break;
	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
		// BLE needs the radio at the current end: ride out the remainder, the CC0 guard hands it back.
		break;
	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
		// End guard fired: stop claiming ownership and hand back, asking for the next slot in one action.
		NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		s_ownedUntil = 0;
		s_ret.callback_action =
			NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
		s_ret.params.request.p_next = &s_req;
		break;
	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
	default:
		// RADIO signals only fire if something set INTENSET (we never do; START clears the SD's).
		break;
	}
	return &s_ret;
}

bool rfTsBegin()
{
	if (s_sessionOpen)
		return true;
	tsBuildRequest();
	if (sd_radio_session_open(tsSignal) != NRF_SUCCESS)
		return false;
	if (sd_radio_request(&s_req) != NRF_SUCCESS) {
		sd_radio_session_close();
		return false;
	}
	s_sessionOpen = true;
	s_lastGrantUs = micros();
	return true;
}

void rfTsEnd()
{
	if (!s_sessionOpen)
		return;
	s_ownedUntil = 0;
	s_sessionOpen = false;
	sd_radio_session_close(); // async; caller is about to disable the SD anyway
}

bool rfRadioOwned(uint32_t usNeeded)
{
	if (!g_sdEnabled)
		return true; // bare-metal mode: the radio is unconditionally ours
	uint32_t d = s_ownedUntil;
	if (!d)
		return false;
	return (int32_t)(d - micros()) > (int32_t)usNeeded;
}

void rfTsTick()
{
	if (!g_sdEnabled || !s_sessionOpen)
		return;
	// Starvation watchdog. A request that ends BLOCKED/CANCELED reports via an SOC event the Adafruit core's
	// SOC task swallows, so a dead session would never self-revive. If no slot has STARTED for a while and we
	// aren't inside one, fire a fresh request. Errors (one already pending) are expected and ignored.
	if (s_ownedUntil)
		return;
	uint32_t now = micros();
	if ((uint32_t)(now - s_lastGrantUs) < 20000u)
		return;
	unsigned long ms = millis();
	if (ms - s_lastKickMs < 10)
		return; // don't SVC-spam while starved
	s_lastKickMs = ms;
	g_tsStarved++;
	tsBuildRequest();
	(void)sd_radio_request(&s_req);
}
