// esb_backend.h -- nrf_esb-based radio backend (the "adopt a real ESB library" path).
//
// Built only when OPK_RADIO_ESB=1. Wraps the vendored Nordic nrf_esb (nrf_esb.c) so the puck runs as an ESB
// PTX poll master: it polls the controller (which is an ESB PRX) and the controller's reply rides back in the
// ACK payload. This replaces the raw RADIO half-duplex poller in radio.cpp/rf_link.cpp; the win is hardware
// auto-ack + auto-retransmit + an IRQ/PPI/timer pipeline independent of loop(), so a lost poll is recovered
// in microseconds instead of surfacing as the reply gap that triggers the connect-buzz loop.
//
// STATUS: compiles and configures nrf_esb for the puck's exact link (BLE 2Mbit, CRC16 0x11021/0xFFFF, DPL,
// 5-byte address). It is NOT yet wired into rf_link's hot path and is unvalidated on hardware -- see
// docs/ESB_MIGRATION.md for the remaining steps (rf_link wiring, IRQ/PPI/TIMER ownership, on-air A/B).
#pragma once
#include "config.h"
#if defined(OPK_RADIO_ESB) && OPK_RADIO_ESB
#include <stdint.h>

// Configure nrf_esb as PTX with the puck's link parameters and enable the radio. Returns true on success.
bool esbBackendInit();

// Set the active on-air address. base4/prefix are the STORED bytes (as in the bond / g_rfBase); nrf_esb does
// the per-byte bit-reversal to the wire itself -- the same transform rfSetAddr() does by hand -- so pass the
// stored form, not the reversed form.
void esbBackendSetAddr(const uint8_t base4[4], uint8_t prefix);
void esbBackendSetChannel(uint8_t ch);

// Poll: send one ack-requesting PTX packet (payload[0] = opcode, e.g. 0xE3) and capture the controller's ACK
// payload (its F1/F3 reply) into out (up to outCap). Returns the reply length, 0 if none. Bounded-blocking so
// it can drop in for rfConnTx. The PID is managed by nrf_esb (auto-incremented per packet).
uint8_t esbBackendPoll(const uint8_t *payload, uint8_t plen, uint8_t *out,
		       uint8_t outCap);

// Send one no-ack packet (the E1 discovery/host frame); no reply is expected.
void esbBackendSendNoAck(const uint8_t *payload, uint8_t plen);
#endif // OPK_RADIO_ESB
