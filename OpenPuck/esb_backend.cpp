#include "esb_backend.h"
#if defined(OPK_RADIO_ESB) && OPK_RADIO_ESB
#include "nrf_esb.h"
#include <Arduino.h>
#include <string.h>

// nrf_esb is event-driven; bridge it to the synchronous poll model rf_link expects. The handler latches TX
// completion and the most recent RX (ACK) payload; esbBackendPoll() writes a packet then spins (bounded) for
// the completion edge. Bounded waits only -- a wedged radio must never hang the USB loop.
static volatile bool s_txDone;
static volatile bool s_rxHave;
static nrf_esb_payload_t s_rx;

static void esbEvt(nrf_esb_evt_t const *evt)
{
	switch (evt->evt_id) {
	case NRF_ESB_EVENT_TX_SUCCESS:
		s_txDone = true;
		break;
	case NRF_ESB_EVENT_TX_FAILED:
		// auto-retransmits are exhausted; drop the packet so the FIFO can't wedge the next poll.
		nrf_esb_flush_tx();
		s_txDone = true;
		break;
	case NRF_ESB_EVENT_RX_RECEIVED:
		while (nrf_esb_read_rx_payload(&s_rx) == NRF_SUCCESS)
			s_rxHave = true;
		break;
	}
}

bool esbBackendInit()
{
	// Field-by-field (not the SDK's NRF_ESB_DEFAULT_CONFIG designated-initializer macro, which g++ rejects
	// as out-of-order before C++20). Parameters mirror the puck link decoded in radio.cpp/PROTOCOL.md.
	nrf_esb_config_t cfg;
	memset(&cfg, 0, sizeof cfg);
	cfg.protocol = NRF_ESB_PROTOCOL_ESB_DPL;
	cfg.mode = NRF_ESB_MODE_PTX;
	cfg.event_handler = esbEvt;
	// RADIO MODE = Ble_2Mbit: the puck uses the BLE 2 Mbit PHY (16-bit preamble), not Nrf_2Mbit.
	cfg.bitrate = NRF_ESB_BITRATE_2MBPS_BLE;
	// Two-byte CRC over address+payload; nrf_esb's 16-bit CRC is poly 0x11021 init 0xFFFF, matching the puck.
	cfg.crc = NRF_ESB_CRC_16BIT;
	cfg.tx_output_power = NRF_ESB_TX_POWER_0DBM;
	// Hardware auto-retransmit -- the reliability layer the raw backend never had. Tunable on hardware.
	cfg.retransmit_count = 3;
	cfg.retransmit_delay = 600;
	cfg.tx_mode = NRF_ESB_TXMODE_AUTO;
	cfg.radio_irq_priority = 3;
	cfg.event_irq_priority = 6;
	cfg.payload_length = 64;
	// Per-packet ack control: the poll requests an ack (to pull the reply), the E1 beacon sets noack.
	cfg.selective_auto_ack = true;
	return nrf_esb_init(&cfg) == NRF_SUCCESS;
}

void esbBackendSetAddr(const uint8_t base4[4], uint8_t prefix)
{
	uint8_t base[4] = { base4[0], base4[1], base4[2], base4[3] };
	uint8_t pfx[1] = { prefix };
	nrf_esb_set_base_address_0(base);
	nrf_esb_set_prefixes(pfx,
			     1); // pipe 0 only -- one controller per address
}

void esbBackendSetChannel(uint8_t ch)
{
	nrf_esb_set_rf_channel(ch);
}

uint8_t esbBackendPoll(const uint8_t *payload, uint8_t plen, uint8_t *out,
		       uint8_t outCap)
{
	nrf_esb_payload_t tx;
	memset(&tx, 0, sizeof tx);
	tx.pipe = 0;
	// request the ack so the controller's reply rides back in the ACK payload
	tx.noack = false;
	if (plen > sizeof tx.data)
		plen = sizeof tx.data;
	memcpy(tx.data, payload, plen);
	tx.length = plen;

	s_txDone = false;
	s_rxHave = false;
	if (nrf_esb_write_payload(&tx) != NRF_SUCCESS)
		return 0;
	uint32_t t0 = micros();
	while (!s_txDone && (uint32_t)(micros() - t0) < 3000) {
	}
	if (!s_rxHave)
		return 0;
	uint8_t n = (s_rx.length < outCap) ? s_rx.length : outCap;
	memcpy(out, s_rx.data, n);
	return n;
}

void esbBackendSendNoAck(const uint8_t *payload, uint8_t plen)
{
	nrf_esb_payload_t tx;
	memset(&tx, 0, sizeof tx);
	tx.pipe = 0;
	tx.noack = true;
	if (plen > sizeof tx.data)
		plen = sizeof tx.data;
	memcpy(tx.data, payload, plen);
	tx.length = plen;

	s_txDone = false;
	nrf_esb_write_payload(&tx);
	uint32_t t0 = micros();
	while (!s_txDone && (uint32_t)(micros() - t0) < 2000) {
	}
}
#endif // OPK_RADIO_ESB
