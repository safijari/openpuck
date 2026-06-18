#include "rf_diag.h"
#include "radio.h"
#include "bonds.h"
#include <Arduino.h>
#include <string.h>

bool g_rfListen = false;
bool g_rfBeacon = false;
uint8_t g_plen = 0x18, g_s1incl = 0;
bool g_rfRaw = false;
bool g_rfSweep = false;
bool g_rfCap = false, g_rfCapOne = false, g_rfReplay = false;
uint8_t g_replayLen = 0;
bool g_rfAuto = false;
uint8_t g_capV = 0, g_cfgIdx = 0;
bool g_rfRespond = false;
bool g_sniff = false;
uint8_t g_sniffPh = 0, g_sniffPark = 0;

static uint8_t g_seq = 0;
static unsigned long g_lastBeacon = 0, g_lastHop = 0, g_lastCfg = 0;
static uint8_t g_replay[48];
static uint32_t g_capPass = 0;
static uint32_t g_advCount = 0;
static uint8_t g_hostTx[48];
static int g_hostLen = 0;
static uint32_t g_sniffN = 0;
static uint8_t g_sbase[4] = { 0, 0, 0, 0 }, g_sprefix = 0;
static uint8_t g_schan[3] = { 78, 2, 80 };
static uint8_t g_schi = 0;

// ===================== promiscuous raw capture (calibration only) =====================
// Preamble-match on 0x55, fixed-length grab, no CRC. Catches any 2Mbit packet on a channel so we can read the
// controller's reconnect addr/prefix/framing.
#define RAWCAP 48
void rfRawStart(uint8_t ch)
{
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos);
#if defined(RADIO_MODECNF0_RU_Fast)
	NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
#endif
	NRF_RADIO->FREQUENCY = ch;
	NRF_RADIO->TXPOWER =
		(RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
	NRF_RADIO->CRCCNF = 0;
	NRF_RADIO->PCNF0 = 0;
	NRF_RADIO->PCNF1 =
		(RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
		(RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) |
		(2u << RADIO_PCNF1_BALEN_Pos) |
		(RAWCAP << RADIO_PCNF1_STATLEN_Pos) |
		(RAWCAP << RADIO_PCNF1_MAXLEN_Pos);
	NRF_RADIO->BASE0 = 0x55555555;
	NRF_RADIO->PREFIX0 = 0x00000055;
	NRF_RADIO->TXADDRESS = 0;
	NRF_RADIO->RXADDRESSES = 1;
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
}

// ===================== CRC-validating framing sweep =====================
// ESB DPL (S0L0, LFLEN6or8, S1LEN3), BALEN4 (5-byte addr), CRC8/16 addr-included.
// Sweep PCNF0{LFLEN6,8} x CRC{8,16} x BALEN{4,3} x ENDIAN{Big,Lit}.
static const uint32_t CAPP0B[] = {
	0x00030006, 0x00030008
}; // S1LEN3 + LFLEN6 / LFLEN8 (dynamic length)
static const uint32_t CAPCRCB[][3] = { { 0x1, 0x107, 0xFF },
				       { 0x2, 0x11021, 0xFFFF } };
void rfCapStart(uint8_t ch)
{
	uint8_t ci = g_capV & 1;
	uint8_t balen = ((g_capV >> 1) & 1) ? 3 : 4; // BALEN 4 first
	uint8_t endbig = ((g_capV >> 2) & 1) ^ 1; // ENDIAN Big first
	uint32_t p0 = CAPP0B[(g_capV >> 3) & 1];
	const uint32_t *cr = CAPCRCB[ci];
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos);
#if defined(RADIO_MODECNF0_RU_Fast)
	NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
#endif
	NRF_RADIO->FREQUENCY = ch;
	NRF_RADIO->PCNF0 = p0;
	NRF_RADIO->PCNF1 = ((uint32_t)endbig << RADIO_PCNF1_ENDIAN_Pos) |
			   ((uint32_t)balen << RADIO_PCNF1_BALEN_Pos) |
			   (60u << RADIO_PCNF1_MAXLEN_Pos);
	NRF_RADIO->CRCCNF = cr[0];
	NRF_RADIO->CRCPOLY = cr[1];
	NRF_RADIO->CRCINIT = cr[2];
	uint8_t b[4];
	for (int i = 0; i < 4; i++)
		b[i] = rfBitrev8(g_rfBase[i]);
	NRF_RADIO->BASE0 = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
			   ((uint32_t)b[2] << 8) | b[3];
	NRF_RADIO->PREFIX0 = rfBitrev8(g_rfPrefix);
	NRF_RADIO->TXADDRESS = 0;
	NRF_RADIO->RXADDRESSES = 1;
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	memset(rfrx, 0, 8);
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
}
// REPLAY: capture one real-puck frame (raw, ENDIAN=Little) then re-transmit it VERBATIM to impersonate the puck
// bit-for-bit -- sidesteps unknown framing/CRC. Controller must be bonded to the REAL puck (replayed frame
// carries real-puck uuids + its valid CRC).
static void rfCapPoll()
{
	if (!g_rfCap)
		return;
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		g_capPass++;
		if (g_rfCapOne) {
			memcpy(g_replay, rfrx, sizeof g_replay);
			g_replayLen = 32;
			g_rfCapOne = false;
			g_rfCap = false;
			Serial.printf("# captured replay frame (%uB): ",
				      g_replayLen);
			for (int i = 0; i < g_replayLen; i++)
				Serial.printf("%02X", g_replay[i]);
			Serial.println();
			return;
		}
		if (NRF_RADIO->CRCSTATUS & 1) {
			uint8_t ci = g_capV & 1;
			uint8_t balen = ((g_capV >> 1) & 1) ? 3 : 4;
			uint8_t eb = ((g_capV >> 2) & 1) ^ 1;
			uint32_t p0 = CAPP0B[(g_capV >> 3) & 1];
			Serial.printf(
				"@@@ CRCOK PCNF0=%lX BALEN=%u ENDIAN=%s POLY=%lX: ",
				(unsigned long)p0, balen, eb ? "Big" : "Lit",
				(unsigned long)CAPCRCB[ci][1]);
			for (int i = 0; i < 28; i++)
				Serial.printf("%02X", rfrx[i]);
			Serial.println();
		}
	}
}
void rfReplayOnce()
{
	if (!g_replayLen)
		return;
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos);
#if defined(RADIO_MODECNF0_RU_Fast)
	NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
#endif
	NRF_RADIO->FREQUENCY = 2;
	NRF_RADIO->PCNF0 = 0; // static, ENDIAN=Little -> reproduce on-air bits
	NRF_RADIO->PCNF1 = (3u << RADIO_PCNF1_BALEN_Pos) |
			   ((uint32_t)g_replayLen << RADIO_PCNF1_STATLEN_Pos) |
			   ((uint32_t)g_replayLen << RADIO_PCNF1_MAXLEN_Pos);
	NRF_RADIO->CRCCNF = 0; // CRC already inside the captured bytes
	uint8_t b[4];
	for (int i = 0; i < 4; i++)
		b[i] = rfBitrev8(g_rfBase[i]);
	NRF_RADIO->BASE0 = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
			   ((uint32_t)b[2] << 8) | b[3];
	NRF_RADIO->PREFIX0 = rfBitrev8(g_rfPrefix);
	NRF_RADIO->TXADDRESS = 0;
	NRF_RADIO->PACKETPTR = (uint32_t)g_replay;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
}

// ===================== address/CRC listen =====================
void rfListenStart()
{
	rfConfig(g_rfCh);
	rfSetAddr(g_rfBase, g_rfPrefix);
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	g_rfListen = true;
	Serial.printf("# RF listen ch%u base %02X%02X%02X%02X prefix %02X\n",
		      g_rfCh, g_rfBase[0], g_rfBase[1], g_rfBase[2],
		      g_rfBase[3], g_rfPrefix);
}
static void rfPoll()
{
	if (g_rfRaw) {
		if (NRF_RADIO->EVENTS_END) {
			NRF_RADIO->EVENTS_END = 0;
			// filter noise: require non-preamble/idle byte structure
			int nz = 0;
			for (int i = 0; i < RAWCAP; i++) {
				uint8_t b = rfrx[i];
				if (b && b != 0x55 && b != 0xAA && b != 0xFF)
					nz++;
			}
			if (nz >= 6) {
				g_rfRxCount++;
				Serial.printf("RAW#%lu ch%u: ",
					      (unsigned long)g_rfRxCount,
					      g_rfCh);
				for (int i = 0; i < RAWCAP; i++)
					Serial.printf("%02X", rfrx[i]);
				Serial.println();
			}
		}
		return;
	}
	if (!g_rfListen)
		return;
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		bool crcok = NRF_RADIO->CRCSTATUS & 1;
		uint8_t len = rfrx[0];
		if (len && len < 64) {
			g_rfRxCount++;
			Serial.printf("RF#%lu ch%u crc%d len%u: ",
				      (unsigned long)g_rfRxCount, g_rfCh, crcok,
				      len);
			for (uint8_t i = 0; i <= len && i < 40; i++)
				Serial.printf("%02X ", rfrx[i]);
			Serial.println();
		}
	}
}

// ===================== autosweep candidate radio configs =====================
// Cycle configs while beaconing so ONE controller search window covers the space. S0LEN=1 / CRC16 0x11021 /
// addr "ibex" held fixed; sweep the residual unknowns (PHY, whitening, BALEN, prefix).
struct RfCfg {
	uint8_t mode;
	uint32_t pcnf0, pcnf1;
	uint8_t whiteiv;
	uint16_t crccnf;
	bool prefRaw;
	const char *tag;
};
static const RfCfg SWEEP[] = {
	{ RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x02040040, 37, 0x2, false,
	  "1M BALEN4 whiv37 pfxRev" },
	{ RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x00040040, 0, 0x2, false,
	  "1M BALEN4 whOFF pfxRev" },
	{ RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x02040040, 37, 0x2, true,
	  "1M BALEN4 whiv37 pfxRAW" },
	{ RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x00040040, 0, 0x2, true,
	  "1M BALEN4 whOFF pfxRAW" },
	{ RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x02030040, 37, 0x2, false,
	  "1M BALEN3 whiv37 pfxRev" },
	{ RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x00030040, 0, 0x2, true,
	  "1M BALEN3 whOFF pfxRAW" },
	{ RADIO_MODE_MODE_Ble_2Mbit, 0x00100108, 0x02040040, 37, 0x2, false,
	  "2M BALEN4 whiv37 pfxRev" },
	{ RADIO_MODE_MODE_Ble_2Mbit, 0x00100108, 0x00040040, 0, 0x2, true,
	  "2M BALEN4 whOFF pfxRAW" },
};
void applyCfg(uint8_t i)
{
	const RfCfg &c = SWEEP[i];
	g_mode = c.mode;
	g_pcnf0 = c.pcnf0;
	g_pcnf1 = c.pcnf1;
	g_whiteiv = c.whiteiv;
	g_crccnf = c.crccnf;
	g_prefixRaw = c.prefRaw;
	Serial.printf("# cfg[%u] %s\n", i, c.tag);
}
#define SWEEP_N (sizeof SWEEP / sizeof SWEEP[0])

// ===================== dongle beacon experiment =====================
// Dongle is PTX: TX a beacon [01][seq][..][E2@5] then RX the controller's response (0xF0...).
void rfBeaconOnce()
{
	uint8_t pl[48];
	memset(pl, 0, sizeof pl);
	pl[0] = 0x01;
	pl[1] = g_seq++;
	pl[5] = 0xE2; // dongle-beacon marker
	rfConfig(g_rfCh);
	rfSetAddr(g_rfBase, g_rfPrefix);
	if (g_s1incl)
		NRF_RADIO->PCNF0 |= (1u << RADIO_PCNF0_S1INCL_Pos);
	// RAM: [LENGTH][ (S1 if S1INCL) ][payload]
	rftx[0] = 0;
	rftx[1] = g_plen;
	memcpy(rftx + 2, pl, g_plen);
	NRF_RADIO->PACKETPTR = (uint32_t)rftx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	// RX window for the controller's response
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	rfrx[0] = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	uint32_t t0 = micros();
	while (!NRF_RADIO->EVENTS_END && (micros() - t0) < 500) {
	}
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		bool crcok = NRF_RADIO->CRCSTATUS & 1;
		uint8_t len = rfrx[0];
		if (len && len < 64) {
			g_rfRxCount++;
			Serial.printf("RESP#%lu crc%d len%u: ",
				      (unsigned long)g_rfRxCount, crcok, len);
			for (uint8_t i = 0; i <= len && i < 40; i++)
				Serial.printf("%02X ", rfrx[i]);
			Serial.println();
		}
	}
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
}

// ===================== scan-then-respond (dongle role) =====================
// Listen on ch2 for the controller's advertisement; the instant one arrives, TX the host frame (its ack).
static void buildHostTx()
{
	int slot = -1;
	for (int i = 0; i < NSLOT; i++)
		if (g_slot[i].used) {
			slot = i;
			break;
		}
	memset(g_hostTx, 0, sizeof g_hostTx);
	g_hostLen = 0;
	if (slot < 0)
		return;
	// S0LEN=1 RAM layout: [0]=S0=0x12, [1]=LENGTH, raw RAM offsets validated by controller.
	memset(g_hostTx, 0, sizeof g_hostTx);
	g_hostTx[0] = 0x12;
	g_hostTx[1] = 0x20;
	g_hostTx[3] = 1;
	g_hostTx[5] = 0xE1;
	memcpy(g_hostTx + 6, g_slot[slot].rec, 4);
	memcpy(g_hostTx + 10, g_slot[slot].rec + 4, 4);
	g_hostTx[0xe] = 2;
	memcpy(g_hostTx + 0x12, g_rfBase, 4);
	g_hostTx[0x16] = g_rfPrefix;
	g_hostLen = 0x22;
}
void rfRespondStart()
{
	buildHostTx();
	rfConfig(2);
	rfSetAddr(g_rfBase, g_rfPrefix); // RX ch2 / 91A2A793
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	g_rfRespond = true;
	Serial.printf(
		"# RESPOND mode: RX ch2/91A2A793, reply host frame (len%d)\n",
		g_hostLen);
}
static void rfRespondPoll()
{
	if (!g_rfRespond)
		return;
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		uint8_t len = rfrx[0];
		if (len && len < 64) {
			g_advCount++;
			// reply: TX the host frame immediately (ack)
			NRF_RADIO->TASKS_DISABLE = 1;
			RWAIT_DISABLED();
			NRF_RADIO->EVENTS_DISABLED = 0;
			NRF_RADIO->PACKETPTR = (uint32_t)g_hostTx;
			NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
					    RADIO_SHORTS_END_DISABLE_Msk;
			NRF_RADIO->EVENTS_DISABLED = 0;
			NRF_RADIO->TASKS_TXEN = 1;
			RWAIT_DISABLED();
			NRF_RADIO->EVENTS_DISABLED = 0;
			Serial.printf("ADV#%lu len%u: ",
				      (unsigned long)g_advCount, len);
			for (uint8_t i = 0; i <= len && i < 32; i++)
				Serial.printf("%02X", rfrx[i]);
			Serial.println();
			NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
			NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
					    RADIO_SHORTS_END_START_Msk;
			NRF_RADIO->EVENTS_END = 0;
			NRF_RADIO->TASKS_RXEN = 1;
		}
	}
}

// ===================== live-session sniffer =====================
// phase 0 RX the real puck's host frame on "ibex"/ch2 to learn the random session base/prefix/channel; phase 1
// RX that session address on the primary channel to capture the live connected exchange (E3 poll + F1 reply).
void rfSniffStart()
{
	// PARK (no hop): primary carries the traffic
	uint8_t ch =
		(g_sniffPh == 0) ? 2 : (g_sniffPark ? g_sniffPark : g_schan[0]);
	const uint8_t *base = (g_sniffPh == 0) ? g_rfBase : g_sbase;
	uint8_t pfx = (g_sniffPh == 0) ? g_rfPrefix : g_sprefix;

	// Ble_2Mbit, PCNF0=0x30008, ENDIAN big, BALEN4, CRC16 0x11021
	rfConfig(ch);
	uint8_t b[4];
	for (int i = 0; i < 4; i++)
		b[i] = rfBitrev8(base[i]);
	NRF_RADIO->BASE0 = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
			   ((uint32_t)b[2] << 8) | b[3];
	NRF_RADIO->PREFIX0 = rfBitrev8(pfx);
	NRF_RADIO->TXADDRESS = 0;
	NRF_RADIO->RXADDRESSES = 1;
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	memset(rfrx, 0, 4);
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
}
static void rfSniffPoll()
{
	if (!g_sniff)
		return;
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		bool crcok = NRF_RADIO->CRCSTATUS & 1;
		uint8_t len = rfrx[0];
		if (crcok && len && len < 40) {
			g_sniffN++;
			// host frame -> learn session params + DUMP FULL (app-data may ride here)
			if (g_sniffPh == 0 && rfrx[2] == 0xE1) {
				memcpy(g_sbase, rfrx + 15, 4);
				g_sprefix = rfrx[19];
				g_schan[0] = rfrx[11];
				Serial.printf(
					"# HOSTFRAME ch=%u sbase=%02X%02X%02X%02X prefix=%02X len%u FULL: ",
					rfrx[11], g_sbase[0], g_sbase[1],
					g_sbase[2], g_sbase[3], g_sprefix, len);
				{
					uint8_t dn = (len + 2 < 64) ? len + 2 :
								      64;
					for (uint8_t i = 0; i < dn; i++)
						Serial.printf("%02X", rfrx[i]);
				}
				Serial.println();
				g_sniffPh = 1;
				g_schi = 0;
				rfSniffStart();
			} else if (g_sniffPh == 1) {
				// 0xF1=input, 0x12/0xE1/0xE4=puck host-frame w/ app-data
				uint8_t ty = rfrx[2];
				const char *tag = (ty == 0xF1) ? "<<<INPUT" :
						  (ty == 0x12 || ty == 0xE1 ||
						   ty == 0xE4) ?
								 ">>>HOSTFR" :
								 "  pkt";
				Serial.printf("%s SNIFF#%lu ch%u len%u: ", tag,
					      (unsigned long)g_sniffN,
					      g_schan[g_schi], len);
				// FULL frame (app-data TLVs sit near the end)
				uint8_t dn = (len + 2 < 64) ? len + 2 : 64;
				for (uint8_t i = 0; i < dn; i++)
					Serial.printf("%02X", rfrx[i]);
				Serial.println();
			}
		}
	}
}

void rfDiagTask()
{
	rfPoll();
	rfRespondPoll();
	rfCapPoll();
	if (g_rfCap && millis() - g_lastHop >=
			       90) { // sweep config combos on ch2, CRC-validate
		g_lastHop = millis();
		g_capV = (uint8_t)(g_capV + 1);
		rfCapStart(g_rfCh);
	}
	if (g_rfBeacon && millis() - g_lastBeacon >= 5) {
		g_lastBeacon = millis();
		rfBeaconOnce();
	}
	if (g_rfSweep && millis() - g_lastHop >= 60) {
		g_lastHop = millis();
		g_rfCh += 2;
		if (g_rfCh > 80)
			g_rfCh = 2;
		rfRawStart(g_rfCh);
	}
	if (g_rfAuto &&
	    millis() - g_lastCfg >=
		    500) { // advance to next candidate config every 500ms
		g_lastCfg = millis();
		g_cfgIdx = (g_cfgIdx + 1) % SWEEP_N;
		applyCfg(g_cfgIdx);
	}
	if (g_rfReplay) {
		rfReplayOnce();
	} // bit-perfect replay of a captured real-puck frame on ch2
	rfSniffPoll();
	if (g_rfRespond &&
	    millis() - g_lastHop >=
		    25) { // scan the 3 BLE adv channels w/ matched whitening
		g_lastHop = millis();
		static const uint8_t advf[3] = { 2, 26, 80 },
				     advw[3] = { 37, 38, 39 };
		static uint8_t ai = 0;
		ai = (ai + 1) % 3;
		g_rfCh = advf[ai];
		g_whiteiv = advw[ai];
		rfConfig(g_rfCh);
		rfSetAddr(g_rfBase, g_rfPrefix);
		NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
		NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
				    RADIO_SHORTS_END_START_Msk;
		NRF_RADIO->EVENTS_END = 0;
		NRF_RADIO->TASKS_RXEN = 1;
	}
}
