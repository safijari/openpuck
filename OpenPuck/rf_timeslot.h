// rf_timeslot.h -- SoftDevice radio-timeslot coexistence for the bare-metal ESB link.
//
// With BLE off (the default) the S140 SoftDevice is never enabled and the ESB code owns NRF_RADIO outright --
// nothing here does anything and rfRadioOwned() is always true. With BLE on (ble_host.cpp) the SoftDevice owns
// the radio, and touching NRF_RADIO outside a granted timeslot corrupts the BLE stack (SD assert -> reboot).
// This module keeps a radio-timeslot SESSION open and continuously re-requests/extends timeslots, so the ESB
// poll loop keeps the radio most of the time and the SoftDevice claws it back only for BLE events (connection
// events, scan windows).
//
// Ownership model: the timeslot signal handler (priority-0 interrupt) stamps an "owned until" deadline at each
// slot start/extension, minus a safety margin. Loop-context radio code calls rfRadioOwned(worstCaseUs) at the
// START of each self-contained TX/RX sequence (rfConnTx, rfHostFrameOnce, the rf_diag ops, the self-heal); if
// the remaining window can't hold the whole op it simply skips this loop pass and retries when the next slot is
// granted. Ops are never interrupted mid-sequence -- the gate + margin guarantee the radio is untouched by the
// time the SoftDevice takes it back.
//
// Slot lifecycle: request EARLIEST 15ms slots; at slot start immediately chain EXTEND requests (each granted
// extension pushes the deadline out; the SD fails the extension when a BLE event needs the radio, which is what
// bounds us). A TIMER0 compare fires just before the true end and returns REQUEST_AND_END, so the next slot is
// queued with no app-side gap. Requests that get BLOCKED/CANCELED by BLE activity would normally be re-armed
// from the SoftDevice SOC event -- but the Adafruit core's SOC task swallows non-flash events, so rfTsTick()
// (loop) watchdogs "session open but no slot granted lately" and re-requests instead.
#pragma once
#include <stdint.h>

// Open the timeslot session + queue the first request. Call once, AFTER the SoftDevice is enabled
// (Bluefruit.begin()). Returns false if the SoftDevice refused the session.
bool rfTsBegin();

// Close the session (firmware-update path: the SoftDevice is about to be disabled).
void rfTsEnd();

// True if a radio op of `usNeeded` microseconds may start NOW. Always true while the SoftDevice is disabled
// (bare-metal mode). Cheap enough for per-op gating (one volatile read + micros()).
bool rfRadioOwned(uint32_t usNeeded);

// Loop-context watchdog: re-request a timeslot if the session is open but no slot has been granted lately
// (request BLOCKED/CANCELED by BLE activity -- the grant path can't re-arm itself). Called from bleHostTask().
void rfTsTick();

// Set by ble_host when the SoftDevice comes up / goes down. rfRadioOwned() short-circuits on this.
extern volatile bool g_sdEnabled;

// diagnostics (CDC "BLE" command / panel): slots granted + extensions + blocked re-requests since boot
extern volatile uint16_t g_tsGrants, g_tsExtends, g_tsStarved;
