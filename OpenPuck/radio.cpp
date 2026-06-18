#include "radio.h"

// Pairing rendezvous bytes from IBEX rodata. The 0x91A2A793 key preceding "ibex" is a DIFFERENT key,
// NOT the discovery address.
const uint8_t PAIR_BASE[4] = { 0x69, 0x62, 0x65, 0x78 }; // "ibex"
uint8_t g_rfPrefix = 0x10;
uint8_t g_rfCh = 2;
uint8_t g_rfBase[4] = { 0x69, 0x62, 0x65, 0x78 }; // "ibex"
// ch18: clean channel in the real puck's active hop set {18,2,80}; bad channels collide with trackpad
// data bursts -> reply-rate crash. Tunable 'C'.
uint8_t g_sessCh = 18;

// Safe default; rfGenSessionAddr() overwrites with the per-device value at boot.
uint8_t g_sessBase[4] = { 0x69, 0x62, 0x65, 0x78 };
uint8_t g_sessPrefix = 0x10;

void rfGenSessionAddr()
{
	// Mix the 64-bit FICR DEVICEID into a 4-byte base + prefix: deterministic per chip, unique across chips
	// -> two OpenPucks never collide on the connected session. Avoid degenerate address bytes (0x00/0xFF) for
	// clean preamble correlation, and never reproduce the shared discovery base "ibex" (else isolation is lost).
	uint32_t h = NRF_FICR->DEVICEID[0] * 0x9E3779B1u ^
		     NRF_FICR->DEVICEID[1];
	uint32_t h2 = NRF_FICR->DEVICEID[1] * 0x85EBCA6Bu ^
		      NRF_FICR->DEVICEID[0];
	for (int i = 0; i < 4; i++) {
		uint8_t b = (uint8_t)(h >> (i * 8));
		if (b == 0x00 || b == 0xFF)
			b ^= 0x5A;
		g_sessBase[i] = b;
	}
	uint8_t p = (uint8_t)(h2 >> 16);
	if (p == 0x00 || p == 0xFF)
		p = 0x5C;
	g_sessPrefix = p;
	if (g_sessBase[0] == g_rfBase[0] && g_sessBase[1] == g_rfBase[1] &&
	    g_sessBase[2] == g_rfBase[2] && g_sessBase[3] == g_rfBase[3])
		g_sessBase[0] ^= 0x80; // collided with discovery base "ibex"
}

uint8_t rfrx[100], rftx[100];
uint32_t g_rfRxCount = 0;

// Puck-link config decoded from real-puck capture: puck TX = Ble_2Mbit, ENDIAN=Big (MSB-first), NO whitening,
// addr "ibex", CRC16 0x11021/0xFFFF; static-length frame.
uint32_t g_crcinit = 0xFFFF;
uint8_t g_whiteiv = 37;
uint8_t g_mode = RADIO_MODE_MODE_Ble_2Mbit;

// S0LEN0, LFLEN8, S1LEN3 (ESB DPL) - CRC-VALIDATED
uint32_t g_pcnf0 = 0x00030008UL;
uint8_t g_statlen = 0x20;

// ENDIAN=Big, BALEN4, MAXLEN=96. MAXLEN must be >=66 or the controller's 0x43-battery-augmented F1 (66B)
// truncates -> CRC-fail -> battery never arrives.
uint32_t g_pcnf1 = 0x01040060;
uint32_t g_crcpoly = 0x11021UL;
uint16_t g_crccnf = 0x2; // CRC16, address included
uint8_t g_pid = 0;
uint8_t g_balen = 4; // ESB 5-byte addr
bool g_prefixRaw = false;

uint8_t rfBitrev8(uint8_t x)
{
	uint8_t y = 0;
	for (int i = 0; i < 8; i++) {
		y = (y << 1) | (x & 1);
		x >>= 1;
	}
	return y;
}

void rfSetAddr(const uint8_t b4[4], uint8_t prefix)
{
	uint8_t b[4];
	for (int i = 0; i < 4; i++)
		b[i] = rfBitrev8(b4[i]);
	NRF_RADIO->BASE0 = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
			   ((uint32_t)b[2] << 8) | b[3];
	NRF_RADIO->PREFIX0 = g_prefixRaw ? prefix : rfBitrev8(prefix);
	NRF_RADIO->TXADDRESS = 0;
	NRF_RADIO->RXADDRESSES = 1u << 0;
}

void rfConfig(uint8_t ch)
{
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->MODE = ((uint32_t)g_mode << RADIO_MODE_MODE_Pos);
	NRF_RADIO->FREQUENCY = ch;
	NRF_RADIO->TXPOWER =
		(RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
#if defined(RADIO_MODECNF0_RU_Fast)
	NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
#endif
	NRF_RADIO->PCNF0 = g_pcnf0;

	// g_pcnf1=0 -> build static-length config from g_statlen/g_balen (ENDIAN=Big, WHITEEN=0)
	NRF_RADIO->PCNF1 =
		g_pcnf1 ? g_pcnf1 :
			  ((1u << RADIO_PCNF1_ENDIAN_Pos) |
			   ((uint32_t)g_balen << RADIO_PCNF1_BALEN_Pos) |
			   ((uint32_t)g_statlen << RADIO_PCNF1_STATLEN_Pos) |
			   ((uint32_t)g_statlen << RADIO_PCNF1_MAXLEN_Pos));
	NRF_RADIO->CRCCNF = g_crccnf; // CRC16, address included
	NRF_RADIO->CRCPOLY = g_crcpoly;
	NRF_RADIO->CRCINIT = g_crcinit;
	NRF_RADIO->DATAWHITEIV = g_whiteiv;
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
}
