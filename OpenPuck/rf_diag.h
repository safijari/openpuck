// rf_diag.h -- RF reverse-engineering & calibration scaffolding (NOT used in normal operation).
//
// Tooling that solved the puck protocol, preserved here: promiscuous raw capture, CRC-validating config sweeps,
// bit-exact frame replay, address/CRC listen, the scan-then-respond dongle experiment, the dongle beacon, and
// the live-session sniffer. Each is driven by a flag toggled from the CDC console (serial_console.cpp) and
// serviced by rfDiagTask() from loop().
#pragma once
#include <stdint.h>

// ---- mode flags (toggled by the CDC console; mutually managed there) ----
extern bool g_rfListen;
extern bool g_rfBeacon;
extern uint8_t g_plen, g_s1incl; // dongle-beacon payload length / S1INCL toggle
extern bool g_rfRaw;
extern bool g_rfSweep;
extern bool g_rfCap, g_rfCapOne, g_rfReplay;
extern uint8_t g_replayLen;
extern bool g_rfAuto;
extern uint8_t g_capV, g_cfgIdx;
extern bool g_rfRespond;
extern bool g_sniff;
extern uint8_t g_sniffPh, g_sniffPark;

// ---- start hooks (called from the console on command) ----
void rfListenStart(); // RX + log on the discovery addr/channel
void rfRawStart(uint8_t ch); // promiscuous preamble-match capture (no CRC)
void rfCapStart(
	uint8_t ch); // CRC-validating framing sweep (cycles config combos)
void rfReplayOnce(); // bit-exact re-transmit of a captured real-puck frame
void rfBeaconOnce(); // dongle beacon experiment ([01][seq][..][E2@5])
void applyCfg(uint8_t i); // apply autosweep candidate radio config i
void rfRespondStart(); // scan-then-respond (RX ch2 adv, reply host frame)
// session sniffer (learn session params, then capture the live exchange)
void rfSniffStart();

// Per-loop: service whichever diag mode is active.
void rfDiagTask();
