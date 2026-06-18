// radio.h -- bare-metal nRF52 RADIO (ESB-style) hardware layer.
//
// Drives the RADIO peripheral directly with NO SoftDevice (BLE is never started), coexisting with TinyUSB.
// Owns the register-level config (PHY/CRC/whitening/address, all live-tunable from the CDC console),
// rfConfig()/rfSetAddr() to program the radio, and the rfrx/rftx DMA buffers. The puck protocol on top lives
// in rf_link.cpp; the RE/calibration tooling in rf_diag.cpp.
//
// Register-confirmed config (decoded from real-puck capture): PHY=Ble_2Mbit, ENDIAN=Big (MSB-first), whitening
// off, addr "ibex", CRC16 poly 0x11021 init 0xFFFF (address included). On-air address = bitrev8 of stored bytes.
#pragma once
#include <stdint.h>
#include <Arduino.h>

// Bounded radio-disable wait: NEVER spin forever. A wedged RADIO would hang the main loop (USB stops being
// serviced -> device "dies" until replug). On timeout we bail; the next rfConfig re-inits the radio.
#define RWAIT_DISABLED()                                   \
	do {                                               \
		uint32_t _w = micros();                    \
		while (!NRF_RADIO->EVENTS_DISABLED &&      \
		       (uint32_t)(micros() - _w) < 3000) { \
		}                                          \
	} while (0)

// Discovery/reconnect address (IBEX FUN_00019000, firmware-derived, definitive):
//   base = "ibex" (69 62 65 78), prefix = 0x10, channel = 2.
extern const uint8_t PAIR_BASE[4]; // "ibex"
extern uint8_t g_rfPrefix; // discovery prefix (rodata byte @0x56f98)
extern uint8_t g_rfCh; // current TX/RX channel (hopped during beacon/poll)
extern uint8_t g_rfBase[4]; // "ibex"
// connected-session channel: a CLEAN data channel off the congested adv ch2
extern uint8_t g_sessCh;

// Per-device SESSION address (base+prefix). Discovery stays on the SHARED "ibex" address (g_rfBase) so any
// controller can find us; the host frame advertises THIS unique address and the controller adopts it for the
// session. Two OpenPucks therefore never share an on-air session address -> no crosstalk, no cross-wake.
// Derived from the FICR DEVICEID (stable per chip, unique across chips).
extern uint8_t g_sessBase[4];
extern uint8_t g_sessPrefix;
void rfGenSessionAddr();

// RADIO DMA buffers (>= MAXLEN+2; the controller's 0x43-augmented F1 is ~66B)
extern uint8_t rfrx[100], rftx[100];
extern uint32_t g_rfRxCount;

// ---- tunable radio parameters (CDC console M/0/2/i/w/I/P/N/T/A) ----
extern uint32_t g_crcinit;
extern uint8_t g_whiteiv;
extern uint8_t g_mode; // RADIO_MODE_MODE_* (real puck = Ble_2Mbit)
extern uint32_t g_pcnf0; // S0LEN0, LFLEN8, S1LEN3 (ESB DPL) -- CRC-VALIDATED
extern uint8_t g_statlen; // static-mode payload length
extern uint32_t g_pcnf1; // ENDIAN=Big, BALEN4, MAXLEN64 -- CRC-VALIDATED
extern uint32_t g_crcpoly;
extern uint16_t g_crccnf; // CRC16, address included
extern uint8_t g_pid;
extern uint8_t g_balen; // ESB 5-byte addr
// false = bitrev8 prefix (ESB addr_conv); true = raw prefix byte
extern bool g_prefixRaw;

uint8_t rfBitrev8(uint8_t x);
// Access address (IBEX FUN_00037530): BASE0 = bitrev8 each base byte packed big-endian; PREFIX0 = raw or
// bitrev8 prefix per g_prefixRaw. "ibex" 69 62 65 78 -> BASE0=0x9646A61E, PREFIX0=0x10.
void rfSetAddr(const uint8_t b4[4], uint8_t prefix);
// Program MODE/FREQUENCY/PCNF/CRC/whitening from the tunables above onto channel ch (radio left disabled).
void rfConfig(uint8_t ch);
