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
extern bool     g_rfHost;       // auto-start host beacon on boot (resumes puck role after a USB replug)
extern bool     g_connOn;       // auto-start connected-mode poll on boot
extern uint8_t  g_connType;     // start packet type (E7 handshake, then E3)
extern uint8_t  g_e7b;          // E7 payload B-byte: 0=current(slow/awake), 1=test protocol-version-1
extern uint8_t  g_connLen;
extern uint8_t  g_getParam;     // GET report 0x45 param byte
extern uint8_t  g_e3mode;       // E3-poll PID/S1 mode (0=fixed07, 1=cyclePID+noack1, 2=cyclePID+noack0)
extern bool     g_connVerbose;  // full multi-line debug dump vs compact "I45 <hex>" stream
extern uint32_t g_rxWin;        // poll RX-window (us): caps poll rate (~1e6/rxWin)
extern unsigned long g_connCooldown; // set on 0xF2 disconnect; pauses beacon+poll so a powering-off controller can sleep

// connected-mode state (reset by the 'k' console toggle)
extern uint8_t  g_connSt, g_connStep;
extern uint16_t g_connPoll;
extern uint32_t g_connF1;       // count of 0xF1 input reports seen
extern uint8_t  g_connF3v;      // last protocol version the controller reported in an F3 reply (0xFF=none)

// QoS adaptive channel hopping
extern uint8_t  g_qos;          // 0=off (static g_sessCh), 1=auto-hop on degradation
extern uint8_t  g_hopIdx;
extern volatile uint16_t g_qosBad;
extern unsigned long g_qosCheckMs, g_qosLastHopMs;

// per-second rate readouts for the WebUSB status blob
extern uint16_t g_f1ps;         // last completed second's F1 rate
extern uint16_t g_newps;        // genuine new-report rate (report 0x45 seq byte changes)

// TX one connected packet [LEN][S1][payload] on channel ch, then RX the reply into rfrx; decodes 0xF1.
uint8_t rfConnTx(uint8_t ch, uint8_t s1, const uint8_t* payload, uint8_t plen);
// Mid-session channel hop: advertise newCh on the current channel a few times, then move the poll to newCh.
void rfHopTo(uint8_t newCh);
// Per-loop: host-frame beacons + connected-mode poll + remote-wakeup + QoS hop + per-second stats.
void rfLinkTask();
