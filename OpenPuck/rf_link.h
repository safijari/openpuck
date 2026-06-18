// rf_link.h -- the operational puck<->controller RF protocol (the "dongle" role).
//
// Built on the radio HW layer (radio.cpp), this is what makes the device act as a puck once a controller is
// bonded: it transmits the host frame the bonded controller reconnects on, runs the connected-mode poll loop
// (E7 awake announce -> E3 + GET-report-0x45 each cycle), and decodes the controller's 0xF1 input reply. The
// decode fills the shared g_in (triton.h) and dispatches to the active IController; it also handles the
// mode-switch chord, USB remote-wakeup on a Steam short-press, adaptive channel hopping (QoS), and the
// host-frame keepalive beacons. RE/calibration scaffolding lives separately in rf_diag.cpp.
//
// RECIPE (IBEX disasm @0x18d80): the controller streams 0xF1 ONLY when (a) connstate==3 [host frame E1
// establishes it], (b) we mark host-awake via E7 [00][00], and (c) each E3 poll carries a GET-report-0x45
// sub-TLV. F1 reply = type 0xF1 then TLV type4(analog/buttons)+type6(per-module gyro/accel/sticks).
#pragma once
#include <stdint.h>

// ---- operational toggles + tunables (set from the CDC console / WebUSB; read here + by webusb status) ----
// auto-start host beacon on boot (resumes puck role after a USB replug)
extern bool g_rfHost;
extern bool g_connOn; // auto-start connected-mode poll on boot
extern uint8_t g_connType; // start packet type (E7 handshake, then E3)

// E7 payload B-byte: 0=current(slow/awake), 1=protocol-version-1
extern uint8_t g_e7b;
extern uint8_t g_connLen;
extern uint8_t g_getParam; // GET report 0x45 param byte

// E3-poll PID/S1 mode (0=fixed07, 1=cyclePID+noack1, 2=cyclePID+noack0)
extern uint8_t g_e3mode;

// full multi-line debug dump vs compact "I45 <hex>" stream
extern bool g_connVerbose;
extern uint32_t g_rxWin; // poll RX-window (us): caps poll rate (~1e6/rxWin)

// set on 0xF2 disconnect; pauses beacon+poll so a powering-off controller can sleep
extern unsigned long g_connCooldown;

// connected-mode state (reset by the 'k' console toggle)
extern uint8_t g_connSt, g_connStep;
extern uint16_t g_connPoll;
extern uint32_t g_connF1; // count of 0xF1 input reports seen

// last protocol version the controller reported in an F3 reply (0xFF=none)
extern uint8_t g_connF3v;

// QoS adaptive channel hopping
extern uint8_t g_qos; // 0=off (static g_sessCh), 1=auto-hop on degradation
extern uint8_t g_hopIdx;
extern volatile uint16_t g_qosBad;
extern unsigned long g_qosCheckMs, g_qosLastHopMs;

// per-second rate readouts for the WebUSB status blob
extern uint16_t g_f1ps; // last completed second's F1 rate
// genuine new-report rate (report 0x45 seq byte changes)
extern uint16_t g_newps;

// last second's poll TX count (GET+relay) -- vs F1 tells starvation from reply-loss
extern uint16_t g_pollsps;

// MEASURED avg us between GET-poll fires (compare to the intended g_pollUs=4000)
extern uint16_t g_pollPeriodUs;

// Smoothed controller->puck signal strength, sampled by the radio (RSSISAMPLE) on each CRC-good controller
// reply during the poll. Stored as the dBm MAGNITUDE (35 = -35dBm); 0 = no sample yet. puck_hid reports it
// to Steam in status report 0x7B byte 8 (signed dBm), after subtracting RSSI_DBM_OFFSET.
extern volatile uint8_t g_linkRssi;
// dB to subtract from our raw RSSI magnitude before reporting, to match the real puck's close-range -35dBm
// (compensates the Pro Micro antenna vs Valve's front-end). Tune against one known-distance reading.
#define RSSI_DBM_OFFSET 20
// Battery percent from the controller's periodic report 0x43 (body[1]); 0 = none yet. Surfaced in the WebUSB
// panel; the raw 0x43 report is also forwarded to Steam verbatim (puck_hid onAuxReport) so Steam reads it itself.
extern volatile uint8_t g_battery;

// TX one connected packet [LEN][S1][payload] on channel ch, then RX the reply into rfrx; decodes 0xF1.
// rxWinUs overrides the reply-wait window (0 = use g_rxWin). Pass a tiny value for NO-ACK relays that expect
// no reply, so they don't burn a full ~1.2ms window of dead air per haptic.
uint8_t rfConnTx(uint8_t ch, uint8_t s1, const uint8_t *payload, uint8_t plen,
		 uint16_t rxWinUs = 0);
// Mid-session channel hop: advertise newCh on the current channel a few times, then move the poll to newCh.
void rfHopTo(uint8_t newCh);
// Per-loop: host-frame beacons + connected-mode poll + remote-wakeup + QoS hop + per-second stats.
void rfLinkTask();
