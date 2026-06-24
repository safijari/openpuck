// deck_input.h -- the USB-CDC link to the Steam Deck forwarder app.
//
// The Deck app (deck/) streams the Deck's grabbed controls as INPUT frames; we decode them into g_in
// (report45.cpp then encodes the RF report 0x45). We stream STATUS frames back so the app's UI can show
// which bonded pucks are live. One framed binary protocol both ways; the reader resynchronizes on a
// 2-byte sync word so a serial monitor's stray text is harmless. The wire format is mirrored exactly in
// deck/frame.py.
//
//   frame = [0xA5][0x5A][type][len][payload(len)][sum8(type,len,payload...)]
//
//   Deck -> nRF:  0x01 INPUT (38B fields), 0x02 CONTROL ([sub][..]: sub 0x01=set-forwarding[0/1],
//                 sub 0x02=clear-bond[slot]), 0x03 REQUEST_STATUS (no payload)
//   nRF -> Deck:  0x10 STATUS, 0x11 HAPTIC (relayed Steam rumble), 0x1F TEXT(debug)
#pragma once
#include <stdint.h>

#define DECK_SYNC0 0xA5
#define DECK_SYNC1 0x5A

#define DECK_T_INPUT 0x01
#define DECK_T_CONTROL 0x02
#define DECK_T_REQ_STATUS 0x03
#define DECK_T_STATUS 0x10
#define DECK_T_HAPTIC 0x11
#define DECK_T_TEXT 0x1F

// Pump the CDC RX state machine: parse any complete frames, fill g_in, update forwarding state.
void deckInputPoll();
// Emit a STATUS frame (bond list + link state) to the Deck. Rate-limited internally; call each loop.
void deckStatusTask();
// Flush the newest staged haptic (from deckForwardHaptic) to the Deck. Rate-limited; call each loop.
void deckHapticTask();
// Send a debug text line as a 0x1F frame (skipped by the app unless it shows debug).
void deckText(const char *s);

// Forward a relayed Steam haptic/rumble to the Deck as a HAPTIC frame (the app plays it as Deck rumble).
// rid = the relayed report id (0x80 SDL Triton rumble, 0x82/0x8F haptic pulse); d/n = its raw payload.
void deckForwardHaptic(uint8_t rid, const uint8_t *d, uint8_t n);

// Is the app currently forwarding (user tapped a puck)? When false, ctrl_link sends a neutral report.
bool deckForwarding();
// millis() of the last INPUT frame; ctrl_link sends neutral if input goes stale while forwarding.
unsigned long deckLastInputMs();
