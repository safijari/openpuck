// ctrl_link.h -- the inverse of OpenPuck's rf_link.cpp: the CONTROLLER side of the puck link.
//
// rf_link.cpp (puck) advertises an E1 host frame, then POLLS the controller (E7 awake, E3 GET 0x45) and
// decodes the controller's 0xF1 reply. Here we run the mirror image: we RX the puck's E1 (adopt its
// advertised session base/prefix/channel when it matches a stored bond), then ANSWER its polls -- F1
// carrying report 0x45 built from the Deck's input (report45.cpp), F3 to the E7 awake/version probe.
//
// No RF pairing handshake: pairing is done over USB (Steam writes both sides), reconnect is purely
// "hear an E1 with my UUIDs -> adopt -> answer polls" -- the same model rf_link relies on.
#pragma once
#include <stdint.h>

// connected bond index (-1 = searching) and the last time we heard a poll on the session address.
extern int g_linkSlot;
extern unsigned long g_linkAliveMs;

void ctrlLinkTask();
// connected AND recently polled (drives the Deck UI "available" indicator + neutral-report decision).
bool ctrlLinkUp();
