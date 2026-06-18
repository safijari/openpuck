#include "rf_link.h"
#include "radio.h"
#include "bonds.h"
#include "config.h"
#include "triton.h"
#include "haptics.h"
#include "controllers.h"
#include "status_led.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

bool g_rfHost = true;
bool g_connOn = true;
uint8_t g_connType = 0xE7; // start with protocol-version handshake, then 0xE3
// 0=current(slow/awake), 1=protocol-version-1. 'V<n>' to toggle.
uint8_t g_e7b = 0;
uint8_t g_connLen = 0x08;
// GET report 0x45 param byte. 'q' cmd.
uint8_t g_getParam = 0x00;
// cycling the ESB PID drains the controller's report queue (~400 new/s vs ~60 with a fixed PID). 'e<n>' selects.
uint8_t g_e3mode = 1;
bool g_connVerbose = false;
// poll RX-window (us); shorter=more polls/s but may miss DELAYED replies. Tunable 'r'.
uint32_t g_rxWin = 1200;
unsigned long g_connCooldown = 0;

uint8_t g_connSt = 0; // 0=announce awake, 1=poll loop
uint8_t g_connStep = 0; // repeat counter within a state
uint16_t g_connPoll = 0; // poll counter (re-assert awake every 32nd)
uint32_t g_connF1 = 0;
uint8_t g_connF3v = 0xFF;

uint8_t g_qos = 0;
// clean, spread channels (from the puck's RSSI/PER scan)
static const uint8_t g_hopCand[] = { 18, 46, 76, 22, 68 };
uint8_t g_hopIdx = 0;
volatile uint16_t g_qosBad = 0;
unsigned long g_qosCheckMs = 0, g_qosLastHopMs = 0;

uint16_t g_f1ps = 0;
uint16_t g_newps = 0;
// polls/s (GET+relay TXs) last second -- distinguishes loop-starvation from reply-loss
uint16_t g_pollsps = 0;
// measured avg us between GET-poll fires (vs intended g_pollUs)
uint16_t g_pollPeriodUs = 0;
static uint32_t g_pollDtSum = 0;
static uint16_t g_pollDtCnt = 0;
// smoothed |dBm| of the controller's replies (0 = none yet)
volatile uint8_t g_linkRssi = 0;
// battery % from the controller's report 0x43 (body[1]); 0 = none yet
volatile uint8_t g_battery = 0;

// ---- internal counters / timers ----
static uint8_t g_e3pid = 0;
static uint32_t g_stPoll = 0, g_stF1 = 0, g_stF3 = 0;
static unsigned long g_stMs = 0;
static uint8_t g_lastSeq = 0;
static uint32_t g_stNew = 0;
static uint32_t g_stCrc = 0, g_stNoRx = 0;
static uint32_t g_chF1[3] = { 0, 0, 0 };
static uint32_t g_lastPollUs = 0;
static uint32_t g_connRx = 0;
static unsigned long g_lastSessBeacon = 0, g_lastDisc = 0;
static unsigned long g_lastStream = 0;

// HOST FRAME the bonded controller waits for (IBEX FUN_00019000 verify: b[0]=0x12, b[5]=0xE1, b[6..10]=
// proteus_uuid, b[10..14]=ibex_uuid). Built like PROTEUS FUN_00027e9a. Sent on the shared rendezvous addr;
// the controller filters by the uuids in the payload, then connects.
// Transmit one host frame. `discovery`=true sends it on the SHARED rendezvous address ("ibex"/ch2) where a
// searching controller looks; =false sends it on this puck's unique SESSION address (the keepalive once the
// controller has adopted the session). EITHER way the payload advertises the session base/prefix/channel, so
// the controller always learns the unique address to connect on.
static void rfHostFrameOnce(int slot, bool discovery)
{
	if (slot < 0 || slot >= NSLOT || !g_slot[slot].used)
		return;
	// [proteus_uuid 4][ibex_uuid 4][serial 16]
	uint8_t *rec = g_slot[slot].rec;
	// CRC-VALIDATED frame (decoded from real puck): ESB-DPL RAM = [LENGTH][S1=PID][payload(18)]. payload:
	// [0]=0xE1, [1..5]=proteus_uuid LE, [5..9]=ibex_uuid LE, [9]=session channel, [10..13]=0, [13..17]=session
	// base, [17]=session prefix. Radio auto-appends CRC16 0x11021.
	memset(rftx, 0, sizeof rftx);
	// LENGTH = 18 (controller's buf[0]==0x12 check validates this)
	rftx[0] = 0x12;
	// S1 = PID<<1 | noack0  (matches real puck 00/02/04/06)
	rftx[1] = (uint8_t)((g_pid++ & 3) << 1);
	rftx[2] = 0xE1; // payload[0] marker
	// payload[1..5] proteus_uuid (LE, as bonded)
	memcpy(rftx + 3, rec + 0, 4);
	memcpy(rftx + 7, rec + 4, 4); // payload[5..9] ibex_uuid

	// payload[9] session channel: controller runs the session on this clean
	// channel (adopts buf[0xe]); discovery beacon still TXes on ch2
	rftx[11] = g_sessCh;
	// payload[13..17] session base  (the per-device UNIQUE address)
	memcpy(rftx + 15, g_sessBase, 4);
	rftx[19] = g_sessPrefix; // payload[17] session prefix
	// TX address: discovery uses the shared "ibex" rendezvous; the session keepalive uses our unique address
	// (where the controller now listens). The advertised session params (above) are identical either way.
	const uint8_t *txBase = discovery ? g_rfBase : g_sessBase;
	uint8_t txPfx = discovery ? g_rfPrefix : g_sessPrefix;
	rfConfig(g_rfCh);
	rfSetAddr(txBase, txPfx);
	NRF_RADIO->PACKETPTR = (uint32_t)rftx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	rfrx[0] = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	// Discovery/pairing beacons listen for the controller's response (matters for RE/pairing); the connected
	// session keepalive expects NO response (the controller answers E3 polls, not the beacon), so don't burn
	// 800us of dead air per frame -- that was the bulk of the idle poll-rate deficit (40 beacons/s x slots).
	uint16_t bwin = discovery ? 800u : 150u;
	uint32_t t0 = micros();
	while (!NRF_RADIO->EVENTS_END && (micros() - t0) < bwin) {
	}
	if (NRF_RADIO->EVENTS_END) {
		// any reception = controller answered our frame
		NRF_RADIO->EVENTS_END = 0;
		g_rfRxCount++;
		bool crcok = NRF_RADIO->CRCSTATUS & 1;
		uint8_t len = rfrx[0];
		// non-blocking: don't stall the loop on CDC backpressure
		if (Serial.availableForWrite() > 90) {
			Serial.printf(
				"*** RESP#%lu ch%u crc%d rxmatch%lu len%u: ",
				(unsigned long)g_rfRxCount, g_rfCh, crcok,
				(unsigned long)NRF_RADIO->RXMATCH, len);
			for (uint8_t i = 0; i < (len < 40 ? len + 2 : 40); i++)
				Serial.printf("%02X ", rfrx[i]);
			Serial.println();
		}
	}
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
}

void rfHopTo(uint8_t newCh)
{
	if (g_connSlot < 0 || newCh == g_sessCh)
		return;
	uint8_t cur = g_sessCh, savedRfCh = g_rfCh;
	g_sessCh = newCh;
	// host frame now advertises newCh but is TXed on cur (session addr)
	g_rfCh = cur;
	for (int k = 0; k < 6; k++) {
		rfHostFrameOnce(g_connSlot, false);
		delayMicroseconds(700);
	}
	g_rfCh = savedRfCh; // poll + session beacon now run on g_sessCh=newCh
}

uint8_t rfConnTx(uint8_t ch, uint8_t s1, const uint8_t *payload, uint8_t plen,
		 uint16_t rxWinUs)
{
	// relays pass a tiny window (no reply expected); polls use g_rxWin
	uint16_t win = rxWinUs ? rxWinUs : g_rxWin;
	memset(rftx, 0, sizeof rftx);
	rftx[0] = plen; // LENGTH = payload byte count
	rftx[1] = s1; // S1 (type-specific)
	memcpy(rftx + 2, payload, plen); // payload[0]=type byte, then data/TLVs
	rfConfig(ch);
	// connected poll runs on this puck's UNIQUE session addr
	rfSetAddr(g_sessBase, g_sessPrefix);
	NRF_RADIO->PACKETPTR = (uint32_t)rftx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	rfrx[0] = 0;
	// RXEN->READY->START; catch the reply. ADDRESS->RSSISTART samples the reply's signal strength (read from
	// RSSISAMPLE below, surfaced to Steam via report 0x7B); DISABLED->RSSISTOP closes the measurement.
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
			    RADIO_SHORTS_DISABLED_RSSISTOP_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	g_stPoll++;
	uint32_t t0 = micros();
	while (!NRF_RADIO->EVENTS_END && (micros() - t0) < win) {
	} // RX window (tunable 'r'; or relay override)
	uint8_t rxlen = 0;
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		bool crcok = NRF_RADIO->CRCSTATUS & 1;
		rxlen = rfrx[0];
		// reply arrived but CRC failed -> RF quality (channel/interference)
		if (!crcok) {
			g_stCrc++;
			g_qosBad++;
		}
		// F1 input ~46B; 0x43-augmented ~66B -> allow up to MAXLEN(96)
		if (crcok && rxlen && rxlen <= 96) {
			// reply type byte (proven offset from captures)
			uint8_t rtype = rfrx[2];
			// Only OUR controller's replies (F-type: 0xF1 input / 0xF2 disconnect / 0xF3 status) mark the link
			// alive. Every OpenPuck shares the same RF address "ibex" + CRC config, and a puck transmits host-frame
			// beacons (0xE1) + polls (0xE2/E3/E7) -- all E-type. Without this gate, puck A receives a SECOND puck's
			// 0xE1 beacon (e.g. one just plugged into another computer), bumps g_connReplyMs, and the "new RF
			// connection" wake in rfLinkTask() fires -> the second puck spuriously wakes this sleeping host.
			if (rtype >= 0xF0) {
				// A reply after a long gap (or the first ever) = a (re)connect. Arm the haptic block + re-init HERE,
				// directly off the reply stream -- reliable even when hapticTask's 300ms link-up edge doesn't fire
				// (e.g. a power-cycled controller that reconnects without us cleanly seeing the link drop).
				if (g_connReplyMs == 0 ||
				    (uint32_t)(millis() - g_connReplyMs) >
					    1500u)
					hapticOnReconnect();
				g_connRx++;
				// link alive -> loop() suppresses the redundant E1 beacon
				g_connReplyMs = millis();
				// |dBm| of this reply (started by the ADDRESS short)
				uint8_t rs =
					(uint8_t)(NRF_RADIO->RSSISAMPLE & 0x7F);
				// EWMA, ~8-sample horizon
				if (rs)
					g_linkRssi =
						g_linkRssi ?
							(uint8_t)((g_linkRssi *
									   7u +
								   rs + 4u) /
								  8u) :
							rs;
			}
			if (rtype == 0xF1)
				g_stF1++;
			// controller disconnecting/powering off -> back off 2.5s
			if (rtype == 0xF2)
				g_connCooldown = millis();
			// F3 = controller status/version reply (reply to E7 handshake, byte[6]=version)
			if (rtype == 0xF3) {
				g_stF3++;
				g_connF3v = rfrx[6];
				if (g_connVerbose &&
				    Serial.availableForWrite() > 40) {
					Serial.print("  F3 ");
					for (uint8_t i = 0;
					     i <
					     (rxlen + 2 < 32 ? rxlen + 2 : 32);
					     i++)
						Serial.printf("%02X", rfrx[i]);
					Serial.println();
				}
			}
			bool isF1 = (rtype == 0xF1);
			if (isF1) {
				g_connF1++;
				// walk ALL type6 TLVs (= HID report 0x45); taking only [0] halves the rate. idx is INT,
				// not uint8_t: tlen 0xFE would make idx+=tlen+2 wrap mod-256 -> infinite loop -> USB hang.
				int idx = 3, end = rxlen + 2;
				const uint8_t *lastRep = nullptr;
				uint8_t lastTlen = 0;
				while (idx + 1 < end) {
					uint8_t tlen = rfrx[idx],
						ttype = rfrx[idx + 1];
					if (tlen == 0)
						break;
					// Only a FULL 0x45 report that fits entirely in rfrx: a short or late/garbled TLV must not let the
					// decode read past the RF buffer (corrupts rftx/RAM -> eventual crash).
					if (ttype == 6 && tlen >= 28 &&
					    (size_t)(idx + 2) + tlen <=
						    sizeof(rfrx) &&
					    rfrx[idx + 2] == 0x45) {
						// report 0x45: [0x45][seq][buttons u32]...
						const uint8_t *rep =
							&rfrx[idx + 2];
						bool fresh =
							(rep[1] != g_lastSeq);
						// genuine new report vs stale poll-repeat
						if (fresh) {
							g_stNew++;
							g_lastSeq = rep[1];
						}
						uint32_t bb = btnsOf(rep);
						// USB remote wakeup on Steam button short press (down + up within 1 s). A long press likely means
						// the user is powering off the controller, so we ignore it.
						{
							static bool steamWasDown =
								false;
							static unsigned long
								steamDownMs = 0;
							if (fresh) {
								bool steamNow =
									(bb &
									 TB_STEAM) !=
									0;
								// rising edge: record press time
								if (steamNow &&
								    !steamWasDown)
									steamDownMs =
										millis();
								// falling edge within 1 s -> short press -> wake
								if (!steamNow &&
								    steamWasDown &&
								    millis() - steamDownMs <
									    1000u &&
								    USBDevice
									    .suspended()) {
									USBDevice
										.remoteWakeup();
									ledWakePulse();
									// post-resume nudge (host needs real input to actually wake)
									if (g_active)
										g_active->wakeEvent();
								}
								steamWasDown =
									steamNow;
							}
						}
						// Decode the report into the shared g_in (one source, read by every IController).
						g_in.buttons = bb;
						// Global power-off chord: Steam + Y held 2 s -> shut the controller down (any mode). Detect on the raw
						// `bb` (pre-mask), time-based (poll rate varies), fires once per hold, re-arms only after release. While
						// held, mask Steam+Y out of g_in.buttons so the press doesn't leak to the host -- in EVERY mode except
						// regular Steam (mode 0 forwards the raw 0x45 to Steam, which owns the Steam button). Runs before
						// onReport45 below so push modes (Xbox) see the mask too; stream modes read g_in in task() (also masked).
						{
							static unsigned long
								offHoldMs = 0;
							static bool offFired =
								false;
							if ((bb & (TB_STEAM |
								   TB_Y)) ==
							    (TB_STEAM | TB_Y)) {
								if (offHoldMs ==
								    0)
									offHoldMs =
										millis();
								else if (
									!offFired &&
									(unsigned long)(millis() -
											offHoldMs) >=
										2000u) {
									offFired =
										true;
									hapticSendShutdown();
								}
								if (g_usbMode !=
								    MODE_STEAM) {
									// stream modes read g_in
									g_in.buttons &=
										~(uint32_t)(TB_STEAM |
											    TB_Y);
									// push modes read btnsOf(rep): TB_Y in rep[2],
									((uint8_t *)
										 rep)
										[2] &=
										~(uint8_t)
											TB_Y;
									// TB_STEAM in rep[4]
									((uint8_t *)
										 rep)
										[4] &=
										~(uint8_t)(TB_STEAM >>
											   16);
								}
							} else {
								offHoldMs = 0;
								offFired =
									false;
							}
						}
						g_in.lx =
							(int16_t)s16off(rep, 8);
						g_in.ly = (int16_t)s16off(rep,
									  10);
						g_in.rx = (int16_t)s16off(rep,
									  12);
						g_in.ry = (int16_t)s16off(rep,
									  14);
						g_in.lt =
							trigU8(u16off(rep, 4));
						// for the Switch digital-trigger threshold
						g_in.rt =
							trigU8(u16off(rep, 6));
						g_in.lpx = (int16_t)s16off(rep,
									   16);
						g_in.lpy = (int16_t)s16off(rep,
									   18);
						g_in.rpx = (int16_t)s16off(rep,
									   22);
						g_in.rpy = (int16_t)s16off(rep,
									   24);
						// IMU lives at report bytes 0x22..0x2D (rep[34..45]). Decode it ONLY when a FULL 46-byte report was
						// actually received -- bounded by `end` (the received length), NOT sizeof(rfrx). The outer gate is
						// tlen>=28 (enough for buttons/sticks/pads, which end at rep[27]), so a short 0x45 (button-only, or
						// one whose IMU tail was lost) still passes it; without this guard imuFrom45 would read STALE bytes
						// past the received data and clobber g_in's gyro/accel. On a short frame, hold the last good IMU.
						if (tlen >= 46 &&
						    (size_t)(idx + 2) + 46 <=
							    (size_t)end)
							imuFrom45(rep, &g_in.ax,
								  &g_in.ay,
								  &g_in.az,
								  &g_in.gx,
								  &g_in.gy,
								  &g_in.gz);
						// Mode-switch chord (all 4 back + face): don't leak the face press to the host. g_in.buttons stays
						// intact so the chord detector still fires; per-mode builders mask the same bits while back-4 held.
						if ((bb & CHORD_BACK4) ==
						    CHORD_BACK4)
							((uint8_t *)rep)[2] &= ~(
								uint8_t)(TB_A |
									 TB_B |
									 TB_X |
									 TB_Y);
						// Hand the report to the active controller. STREAM modes ignore it (they emit from task() reading
						// g_in); PUSH modes (Xbox, puck/lizard) build + send their host report here.
						if (g_active)
							g_active->onReport45(
								rep, fresh,
								tlen);
						lastRep = rep;
						lastTlen = tlen;
					} else if (ttype == 6 &&
						   (size_t)(idx + 2) + tlen <=
							   sizeof(rfrx) &&
						   tlen >= 2 &&
						   (rfrx[idx + 2] == 0x43 ||
						    rfrx[idx + 2] == 0x44)) {
						// Controller STATUS reports (0x43 = periodic power/battery, ~every 2s; 0x44 = status event). The real
						// puck forwards these verbatim (onAuxReport) -- that's how Steam reads battery; also snapshot the
						// battery % for the WebUSB panel.
						// [rid][body...]
						const uint8_t *rep =
							&rfrx[idx + 2];
						// 0x43 body[1] (~0x5e=94) reads as battery % (sniff-derived)
						if (rep[0] == 0x43 && tlen >= 3)
							g_battery = rep[2];
						if (g_active)
							g_active->onAuxReport(
								rep[0], rep + 1,
								(uint8_t)(tlen -
									  1));
					}
					idx += tlen + 2;
				}
				// mode-switch chord (back4 + face): A=always Steam; B/X/Y=configurable (g_chordBtn[]). Debounced.
				{
					static uint8_t chWant = 0xFF, chCnt = 0;
					uint8_t want = 0xFF;
					if ((g_in.buttons & CHORD_BACK4) ==
					    CHORD_BACK4) {
						if (g_in.buttons & TB_A)
							want = MODE_STEAM;
						else if (g_in.buttons & TB_B)
							want = g_chordBtn[0];
						else if (g_in.buttons & TB_X)
							want = g_chordBtn[1];
						else if (g_in.buttons & TB_Y)
							want = g_chordBtn[2];
					}
					if (want != 0xFF && want == chWant) {
						if (++chCnt >= 12 &&
						    want != g_usbMode &&
						    modeValid(want) &&
						    !USBDevice.suspended()) {
							saveMode(want);
							delay(40);
							NVIC_SystemReset();
						}
					} else {
						chWant = want;
						chCnt = (want != 0xFF) ? 1 : 0;
					}
				}
				// compact stream for rf_controller_ui.py -- NON-BLOCKING: skip if CDC TX is backed up (a blocking
				// Serial.print stalls the RF+USB loop -> jaggy input). One line/frame using the last record.
				if (lastRep && !g_connVerbose &&
				    Serial.availableForWrite() > 110 &&
				    millis() - g_lastStream >= 4) {
					g_lastStream = millis();
					Serial.print("I45 ");
					for (uint8_t i = 0; i < lastTlen; i++)
						Serial.printf("%02X",
							      lastRep[i]);
					Serial.println();
				}
			}
			if (g_connVerbose) {
				Serial.printf(
					"%s CRX#%lu txtype%02X ch%u len%u: ",
					isF1 ? "<<<F1" :
					       (rtype == 0xF3 ? "  F3" :
								"  rx"),
					(unsigned long)g_connRx, payload[0], ch,
					rxlen);
				for (uint8_t i = 0;
				     i < (rxlen + 2 <= 66 ? rxlen + 2 : 66);
				     i++)
					Serial.printf("%02X", rfrx[i]);
				Serial.println();
			}
		} else
			rxlen = 0;
		// RX window expired with no packet at all
	} else {
		g_stNoRx++;
		g_qosBad++;
	}
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	return rxlen;
}

// Drive the connected-mode sequence one step per call. Camps on g_sessCh (the host-frame channel the
// controller connected on); the host-frame beacon runs in parallel as keepalive.
static void rfConnStep()
{
	g_connSlot = -1;
	for (int s = 0; s < NSLOT; s++)
		if (g_slot[s].used) {
			g_connSlot = s;
			break;
		}
	// need a bonded slot (session established by host frame)
	if (g_connSlot < 0)
		return;
	uint8_t ch = g_sessCh;
	// announce HOST AWAKE: E7 00 00, a few times
	if (g_connSt == 0) {
		uint8_t p[3] = { 0xE7, 0x00, g_e7b };
		rfConnTx(ch, 0x01, p, 3);
		if (++g_connStep >= 4) {
			g_connSt = 1;
			g_connStep = 0;
			Serial.println(
				"# CONN: awake announced -> polling GET report 0x45");
		}
		// poll loop: E3 + GET report 0x45 every poll; re-assert awake periodically
	} else {
		// CONTROLLED CADENCE: poll ~every g_pollUs
		if ((uint32_t)(micros() - g_lastPollUs) < g_pollUs)
			return;
		// FIXED-RATE schedule: advance the deadline by exactly one interval (NOT reset-to-now, which would add
		// each cycle's ~1ms of poll work + loop jitter to the next interval -- the period drifts to ~5000us
		// instead of the 4000us target). The resync guard prevents a catch-up burst after a real stall.
		{
			uint32_t now = micros();
			static uint32_t lastFire = 0;
			// MEASURED actual period
			if (lastFire) {
				g_pollDtSum += (uint32_t)(now - lastFire);
				g_pollDtCnt++;
			}
			lastFire = now;
			g_lastPollUs += g_pollUs;
			// fell >1 interval behind -> resync
			if ((uint32_t)(now - g_lastPollUs) >= g_pollUs)
				g_lastPollUs = now;
		}
		if ((g_connPoll & 0x1F) == 0) {
			uint8_t pa[3] = { 0xE7, 0x00, g_e7b };
			rfConnTx(ch, 0x01, pa, 3);
		} // re-assert awake/version
		rfConnQueueHapticRelay();
		// Relay (if any) gets its OWN cycled PID, then the GET poll gets the NEXT one -- so they're always distinct
		// and the controller never dedups the GET as a retransmit of the relay (that was dropping ~half the replies
		// during haptics). Advancing g_e3pid even when no relay is pending just skips a PID value (harmless).
		{
			uint8_t rs1 = (uint8_t)((((g_e3pid++) & 3) << 1) | 1);
			rfConnFlushRelay(ch, rs1);
		}
		{
			// E3 + TLV [len=02][subtype=01 GET][id=0x45][param]
			uint8_t p[5] = { 0xE3, 0x02, 0x01, 0x45, g_getParam };
			// cycle PID (S1 1,3,5,7), NO_ACK=1
			// cycle PID (S1 0,2,4,6), NO_ACK=0
			// fixed (matches captured puck poll)
			uint8_t s1 =
				(g_e3mode == 1) ?
					(uint8_t)((((g_e3pid++) & 3) << 1) |
						  1) :
				(g_e3mode == 2) ?
					(uint8_t)(((g_e3pid++) & 3) << 1) :
					0x07;
			uint8_t rx = rfConnTx(ch, s1, p, 5);
			if (rx)
				g_chF1[0]++;
		}
		g_connPoll++;
	}
}

void rfLinkTask()
{
	// Host-frame beacon: sent continuously, INCLUDING while connected. The controller uses the periodic E1 (the
	// real puck's per-hop-cycle announce) to stay synced and keep answering polls at full rate; suppressing it
	// drops the reply rate from ~210/s to ~38/s. Paused only during the post-disconnect cooldown so a controller
	// that's powering off isn't immediately re-woken/reconnected.
	if (g_rfHost && millis() - g_connCooldown > 2500) {
		bool connNow =
			(g_connSlot >= 0 && millis() - g_connReplyMs < 300);
		// session keepalive on the clean channel: every loop while connecting (fast), every 25ms once connected
		// (every-loop beaconing also hammers the session ch and steals reply slots from the poll)
		if (millis() - g_lastSessBeacon >= (connNow ? 25u : 0u)) {
			g_lastSessBeacon = millis();
			g_rfCh = g_sessCh;
			for (int s = 0; s < NSLOT; s++)
				rfHostFrameOnce(s, false);
		}
		// discovery beacon on ch2 (where a searching controller looks): every loop when down, occasionally when up
		if (millis() - g_lastDisc >= (connNow ? 200u : 0u)) {
			g_lastDisc = millis();
			g_rfCh = 2;
			for (int s = 0; s < NSLOT; s++)
				rfHostFrameOnce(s, true);
		}
	}
	if (g_connOn && millis() - g_connCooldown > 2500) {
		rfConnStep();
	} // connected-mode: poll controller, read input
	{
		// remote wakeup on new RF controller connection
		static bool wasRfConn = false;
		bool nowRfConn =
			(g_connSlot >= 0 && millis() - g_connReplyMs < 300);
		if (nowRfConn && !wasRfConn && USBDevice.suspended()) {
			USBDevice.remoteWakeup();
			ledWakePulse();
			// post-resume nudge (host needs real input to actually wake)
			if (g_active)
				g_active->wakeEvent();
		}
		wasRfConn = nowRfConn;
	}
	// QoS: if the current channel is degrading (crcfail+noRx), hop to the next clean candidate (conservative).
	if (g_qos && g_connSlot >= 0 && millis() - g_qosCheckMs >= 600) {
		uint16_t bad = g_qosBad;
		g_qosBad = 0;
		g_qosCheckMs = millis();
		if (bad > 20 && millis() - g_qosLastHopMs > 2000) {
			for (int k = 0; k < (int)sizeof g_hopCand; k++) {
				g_hopIdx = (g_hopIdx + 1) % (sizeof g_hopCand);
				if (g_hopCand[g_hopIdx] != g_sessCh)
					break;
			}
			if (Serial.availableForWrite() > 60)
				Serial.printf(
					"# QoS: ch%u bad=%u -> hop ch%u\n",
					g_sessCh, bad, g_hopCand[g_hopIdx]);
			rfHopTo(g_hopCand[g_hopIdx]);
			g_qosLastHopMs = millis();
		}
	}
	if (g_connOn && millis() - g_stMs >= 1000) {
		g_f1ps = g_stF1;
		g_newps = g_stNew;
		g_pollsps = (uint16_t)g_stPoll;
		g_pollPeriodUs =
			g_pollDtCnt ? (uint16_t)(g_pollDtSum / g_pollDtCnt) : 0;
		g_pollDtSum = 0;
		g_pollDtCnt = 0;
		if (Serial.availableForWrite() > 70)
			Serial.printf(
				"# stat polls=%lu/s F1=%lu/s new=%lu/s F3=%lu/s(v%d) e7b=%u crcfail=%lu noRx=%lu slot=%d\n",
				(unsigned long)g_stPoll, (unsigned long)g_stF1,
				(unsigned long)g_stNew, (unsigned long)g_stF3,
				(int8_t)g_connF3v, g_e7b,
				(unsigned long)g_stCrc, (unsigned long)g_stNoRx,
				g_connSlot);
		g_stPoll = 0;
		g_stF1 = 0;
		g_stNew = 0;
		g_stF3 = 0;
		g_stCrc = 0;
		g_stNoRx = 0;
		g_chF1[0] = g_chF1[1] = g_chF1[2] = 0;
		g_stMs = millis();
	}
}
