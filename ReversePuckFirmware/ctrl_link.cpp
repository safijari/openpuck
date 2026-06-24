#include "ctrl_link.h"
#include "radio.h"
#include "ctrl_bonds.h"
#include "ctrl_usb.h"
#include "report45.h"
#include "deck_input.h"
#include "triton.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

int g_linkSlot = -1;
unsigned long g_linkAliveMs = 0;

// Link is considered up while we've heard a poll within this window (the puck polls ~250 Hz).
#define LINK_ALIVE_MS 1000u
// RX dwell per loop. One transaction (catch a poll, reply) per loop -> ~200 Hz, plenty for input, and
// short enough that USB/CDC servicing isn't starved. Tunable on hardware.
#define RX_WIN_US 4500u
// Time the puck's RX window stays open for our reply; keep the turnaround well under this.
static uint8_t g_seq = 0;
// Last time we caught a frame on the SESSION address (NOT the ch2 discovery beacon). This is the
// truthful "we're actually exchanging with the puck" signal that drives ctrlLinkUp() / the UI
// "available" tile -- hearing the discovery beacon alone does NOT mean the session works.
static unsigned long g_sessRxMs = 0;

// SESSION-CHANNEL HUNT. The E1 host frame advertises a "primary" channel, but that channel mostly
// carries the puck's E1/E2 keepalive -- the ACTIVE session (E3 input poll + our F1 reply) rides a small
// channel set, NOT necessarily the advertised primary (proven by the real-puck capture sniff1.json:
// E1/E2 on ch68, the E3/F1 session on ch48; and by puck_sniffer.ino's session-hunt). So once we adopt a
// session we DWELL across {primary, then these candidates} on the session address until a poll (E2/E3)
// appears, then PARK on that channel. If the parked channel goes dry (QoS hop / puck moved), un-park and
// hunt again. Mirrors puck_sniffer.ino HUNT_CH.
static const uint8_t HUNT_CH[] = { 2, 80, 18, 46, 76, 22, 68, 52, 55, 56, 57 };
static uint8_t g_huntIdx = 0;
static uint8_t g_parkCh = 0; // 0 = hunting; non-zero = parked on this channel
static unsigned long g_huntStepMs = 0;
#define HUNT_DWELL_MS 40u // per-channel dwell while hunting
#define RECONNECT_MS \
	4000u // dry this long -> abandon the session, re-catch E1 on ch2

// connected-mode frame-type diagnostics (reset + printed each second in the status line)
static uint16_t g_cntE2, g_cntE3, g_cntE7, g_cntE4, g_cntOut, g_cntOther;
// E4 hops we actually ADOPTED this second (followed the puck's commanded channel)
static uint16_t g_cntE4follow;
// of the E3 polls: how many were bare/0x45 INPUT polls vs feature/identity GETs (0x83/0xAE/0x87). Tells
// us whether the puck is actually polling for input or stuck looping enumeration GETs.
static uint16_t g_cntE3in, g_cntE3get;
// last non-0x45 report id the puck asked for via an E3 GET sub-TLV (0 = none)
static uint8_t g_getReq = 0;
// distinct channels seen in E4 channel-map commands this second (128-bit) -> reveals the hop pattern:
// a small fixed set is followable; a large/changing set is the unsolved fast-AFH sequence.
static uint8_t g_e4set[16];
// 256-bit: frame types we've already dumped a sample of
static uint8_t g_seenDump[32];
// 256-bit: GET report ids we've dumped a poll+response sample for
static uint8_t g_seenReq[32];

// ---- build replies into rftx (ESB DPL RAM layout: [LENGTH][S1][payload...]) ----
static void buildF1()
{
	// payload = [0xF1][tlen][ttype=6][report45(46)]  -> length = 3 + REPORT45_LEN
	rftx[0] = (uint8_t)(3 + REPORT45_LEN); // LENGTH (payload byte count)
	rftx[1] = (uint8_t)((((g_pid++) & 3) << 1) | 1); // S1: PID<<1 | NO_ACK
	rftx[2] = 0xF1;
	rftx[3] = REPORT45_LEN; // TLV len
	rftx[4] = 6; // TLV type 6 (HID report)
	// neutral report (idle controller) when not forwarding or input has gone stale
	bool fresh = deckForwarding() && (millis() - deckLastInputMs() < 200u);
	if (fresh) {
		buildReport45(rftx + 5, g_seq++);
	} else {
		memset(rftx + 5, 0, REPORT45_LEN);
		rftx[5] = 0x45;
		rftx[6] = g_seq++;
	}
}

// Build an F1 carrying a feature/command RESPONSE. The CORRECT on-wire form (sniff1.json idx 2932/2946)
// is a TWO-TLV F1: a type-4 feature TLV FOLLOWED BY the type-6 report-0x45 input TLV. BUT that 78-byte
// frame destabilizes OUR link because the real puck puts our session into an aggressive E4 hop storm
// (the captured real-controller session was calm single-channel, no E4) and the long frame overruns the
// tight per-hop window -> e3in collapses 245->60. Until the hop storm is calmed (the real blocker), use
// the SHORTER single type-6 TLV (keeps the link stable; Steam won't accept it for enumeration, but
// nothing enumerates on an unstable link anyway). Flip TWO_TLV_FEATURE to 1 once the link is calm.
#define TWO_TLV_FEATURE 1
static void buildFeatureF1(uint8_t reqid, const uint8_t *param, uint8_t plen)
{
	rftx[1] = (uint8_t)((((g_pid++) & 3) << 1) | 1);
	rftx[2] = 0xF1;
	uint8_t v[64];
	uint8_t vlen = ctrlFeatureResp(reqid, param, plen, v);
	if (vlen > 60)
		vlen = 60;
#if TWO_TLV_FEATURE
	rftx[3] = vlen; // TLV1 len
	rftx[4] = 0x04; // TLV1 type (feature/bulk)
	memcpy(rftx + 5, v, vlen);
	uint8_t *t2 =
		rftx + 5 + vlen; // TLV2: report-0x45 input (the link heartbeat)
	t2[0] = REPORT45_LEN;
	t2[1] = 6;
	bool fresh = deckForwarding() && (millis() - deckLastInputMs() < 200u);
	if (fresh)
		buildReport45(t2 + 2, g_seq++);
	else {
		memset(t2 + 2, 0, REPORT45_LEN);
		t2[2] = 0x45;
		t2[3] = g_seq++;
	}
	rftx[0] = (uint8_t)(1 + 2 + vlen + 2 + REPORT45_LEN);
#else
	rftx[3] = vlen; // TLV len
	rftx[4] = 6; // type 6 (single-TLV, stable baseline)
	memcpy(rftx + 5, v, vlen);
	rftx[0] = (uint8_t)(3 + vlen);
#endif
}

static void buildF3()
{
	// Reply to the puck's E7 awake/version probe with a STATUS report. The real puck's RX parser
	// (proteus FUN_0000ba38, F3 branch) reads the connected-state byte at payload[1] (== rftx[3]) and
	// only raises its internal "controller connected" signal if that byte is NONZERO -- so leaving it 0
	// makes the puck bond us but never ACTIVATE us (no enumeration). Set payload[1]=1 (connected). Also
	// set payload[4] (rftx[6]=1) which is where OpenPuck reads its F3 version, so this works both ways.
	rftx[0] = 5;
	rftx[1] = (uint8_t)((((g_pid++) & 3) << 1) | 1);
	rftx[2] = 0xF3;
	// payload[1] = connected state (real puck reads param_2[6]) -- MUST be nonzero
	rftx[3] = 1;
	rftx[4] = 0;
	rftx[5] = 0;
	rftx[6] = 1; // payload[4] = version (OpenPuck reads rfrx[6])
}

// Transmit whatever is staged in rftx on the currently-configured channel/address, then return to RX
// config. Mirrors rf_link's TX path; radio is left disabled at exit.
static void txStaged()
{
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->PACKETPTR = (uint32_t)rftx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
}

// CAMPED continuous RX. The radio stays armed in RX across ctrlLinkTask calls (END_START auto-re-arm) so
// it keeps listening during the loop's CDC servicing instead of being torn down each call. This raises
// the puck-poll catch rate: the real puck AFH-hops AWAY from a channel where our replies look unreliable,
// and the gap-misses (radio off between calls) were making the link look bad. We only (re)configure the
// radio when the address/channel changes or after a reply; otherwise we just poll EVENTS_END.
static bool g_rxArmed = false;
static uint8_t g_rxArmedCh = 0;
static const uint8_t *g_rxArmedBase = nullptr;
static uint8_t g_rxArmedPrefix = 0;

static void armRx(const uint8_t *base, uint8_t prefix, uint8_t ch)
{
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	rfConfig(ch);
	rfSetAddr(base, prefix);
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	rfrx[0] = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	g_rxArmed = true;
	g_rxArmedCh = ch;
	g_rxArmedBase = base;
	g_rxArmedPrefix = prefix;
}

// One RX->maybe-reply transaction on (base,prefix,ch). Returns the received type byte (rfrx[2]) or 0.
// `reply` selects whether we answer polls (true once connected) or just listen (discovery). The radio is
// kept CAMPED between calls (see armRx); only re-armed on an address/channel change or after a packet.
static uint8_t rxOnce(const uint8_t *base, uint8_t prefix, uint8_t ch,
		      uint16_t winUs, bool reply)
{
	if (reply) {
		// CONNECTED: camp the radio (END_START) so it keeps listening between calls. The dispatch reads
		// rfrx + replies INSIDE this function (before re-arm), and the connected caller never reads rfrx
		// afterward, so clobbering rfrx on re-arm is safe.
		if (!g_rxArmed || g_rxArmedCh != ch || g_rxArmedBase != base ||
		    g_rxArmedPrefix != prefix)
			armRx(base, prefix, ch);
	} else {
		// DISCOVERY: fresh, NON-camped RX. discoveryScan reads rfrx AFTER we return, so it must not be
		// clobbered by an END_START auto-re-arm -- plain READY_START (radio idles after the packet).
		g_rxArmed = false;
		rfConfig(ch);
		rfSetAddr(base, prefix);
		NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
		rfrx[0] = 0;
		NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
		NRF_RADIO->EVENTS_END = 0;
		NRF_RADIO->TASKS_RXEN = 1;
	}
	uint32_t t0 = micros();
	while (!NRF_RADIO->EVENTS_END && (uint32_t)(micros() - t0) < winUs) {
	}
	uint8_t type = 0;
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		g_rfRxCount++;
		bool crcok = NRF_RADIO->CRCSTATUS & 1;
		uint8_t rxlen = rfrx[0];
		if (crcok && rxlen && rxlen <= 96) {
			type = rfrx[2];
			if (reply) {
				// DIAGNOSTIC: tally what the puck polls us with, and dump a sample of any frame we
				// DON'T answer with input -- this is how we find identity/feature queries (the likely
				// reason Steam can't enumerate the controller). Surfaced in the 1Hz status line.
				if (type == 0xE2)
					g_cntE2++;
				else if (type == 0xE3) {
					g_cntE3++;
					// E3 may carry a GET sub-TLV [len][01=GET][reportid]; split input vs feature GET
					if (rxlen >= 4 && rfrx[4] == 0x01 &&
					    rfrx[5] != 0x45) {
						g_getReq = rfrx[5];
						g_cntE3get++;
					} else
						g_cntE3in++;
				} else if (type == 0xE7)
					g_cntE7++;
				else if (type == 0xE4)
					g_cntE4++;
				else if (type >= 0x80 && type <= 0x89)
					g_cntOut++;
				else if (type != 0xE1)
					g_cntOther++;
				// dump first sample of any non-input-poll type (E1/E4/E7/output/other)
				if (type != 0xE2 && type != 0xE3 &&
				    !(g_seenDump[type >> 3] &
				      (1 << (type & 7)))) {
					g_seenDump[type >> 3] |=
						(1 << (type & 7));
					if (Serial.availableForWrite() > 70) {
						Serial.printf(
							"# RFtype %02X len%u:",
							type, rxlen);
						uint8_t m = rxlen + 2 < 26 ?
								    rxlen + 2 :
								    26;
						for (uint8_t i = 0; i < m; i++)
							Serial.printf("%02X",
								      rfrx[i]);
						Serial.println();
					}
				}

				// E1 = host-frame keepalive (no reply expected); follow channel hops.
				if (type == 0xE1) {
					uint8_t newCh = rfrx[11];
					if (newCh && newCh != g_sessCh)
						g_sessCh = newCh;
				} else if (type == 0xE4) {
					// channel-map update [E4][ch][02][50]. The real puck floods this ~80/s on ONE
					// channel right after we report connected (F3) and will NOT advance to E3 input
					// polls until it is ACKED -- so reply F3 on the channel we received it (turnaround,
					// like E7), THEN adopt <ch> for subsequent polls (keeping the session base/prefix;
					// the per-timeslot base/prefix hop generator FUN_000491e0 is NOT required to
					// interop -- IBEX_REIMPL_SPEC §5). Earlier we replied nothing here -> the puck
					// retransmitted E4 forever -> 4s timeout -> reconnect w/ incremented counter (flap).
					uint8_t nc = rfrx[3];
					if (rxlen >= 2 && nc >= 1 && nc <= 80) {
						g_e4set[nc >> 3] |=
							(1 << (nc & 7));
						g_cntE4follow++;
						if (deckForwarding()) {
							// ACK the channel map (F3) on the RX channel
							buildF3();
							txStaged();
							if (!(g_seenDump[0xE4 >>
									 3] &
							      (1 << (0xE4 &
								     7))) &&
							    Serial.availableForWrite() >
								    90) {
								g_seenDump[0xE4 >>
									   3] |=
									(1
									 << (0xE4 &
									     7));
								Serial.printf(
									"# E4 ack ch=%u poll[%02X%02X%02X%02X%02X%02X] f3[%02X%02X%02X%02X%02X%02X%02X]\n",
									nc,
									rfrx[0],
									rfrx[1],
									rfrx[2],
									rfrx[3],
									rfrx[4],
									rfrx[5],
									rftx[0],
									rftx[1],
									rftx[2],
									rftx[3],
									rftx[4],
									rftx[5],
									rftx[6]);
							}
						}
						if (nc != g_sessCh) {
							g_sessCh = nc;
							g_parkCh = nc;
							g_huntStepMs = millis();
						}
						g_linkAliveMs = g_sessRxMs =
							millis();
					}
				} else if (!deckForwarding()) {
					// NOT forwarding -> stay SILENT (no F3/F1). We still hear the puck's polls
					// (so the UI shows the puck "available"), but we never answer, so the puck
					// never marks us connected and the HOST shows no controller. The controller
					// only appears once the user taps to forward (deckForwarding() -> true).
				} else if (type == 0xE7) {
					buildF3();
					txStaged();
				} else if (type >= 0xE0 && type <= 0xEF) {
					// E3 GET: if it requests a NON-0x45 report (Steam enumeration -> 0x83
					// attrs / 0xAE serial / 0x87 settings), answer THAT report; a bare poll
					// or a 0x45 request gets the input report.
					if (type == 0xE3 && rxlen >= 4 &&
					    rfrx[4] == 0x05) {
						// Relayed Steam OUTPUT (SET): E3 [tlvlen][05][rid][data]. The puck
						// forwards Steam's rumble/haptic/LED writes this way (legacy type-05
						// form). Hand rumble (0x80) / haptic pulses (0x82/0x8F) to the Deck so
						// it buzzes; relays are NO_ACK so we DON'T reply.
						uint8_t rl =
							(rfrx[3] >= 1) ?
								(uint8_t)(rfrx[3] -
									  1) :
								0;
						deckForwardHaptic(rfrx[5],
								  &rfrx[6], rl);
					} else if (type == 0xE3 && rxlen >= 4 &&
						   rfrx[4] == 0x01 &&
						   (rfrx[5] == 0x83 ||
						    rfrx[5] == 0xAE)) {
						// only 0x83/0xAE get a feature TLV (matches the real controller,
						// sniff1.json); 0x87 / bare / 0x45 -> plain input below.
						uint8_t plen =
							(rxlen >= 5) ?
								(uint8_t)(rxlen -
									  4) :
								0;
						buildFeatureF1(rfrx[5],
							       &rfrx[6], plen);
						// dump the GET poll + our response once per reqid (verify framing)
						uint8_t rq = rfrx[5];
						if (!(g_seenReq[rq >> 3] &
						      (1 << (rq & 7))) &&
						    Serial.availableForWrite() >
							    100) {
							g_seenReq[rq >> 3] |=
								(1 << (rq & 7));
							Serial.printf(
								"# GET %02X poll[",
								rq);
							uint8_t m =
								rxlen + 2 < 20 ?
									rxlen + 2 :
									20;
							for (uint8_t i = 0;
							     i < m; i++)
								Serial.printf(
									"%02X",
									rfrx[i]);
							Serial.print("] resp[");
							uint8_t r =
								rftx[0] + 2 < 30 ?
									rftx[0] +
										2 :
									30;
							for (uint8_t i = 0;
							     i < r; i++)
								Serial.printf(
									"%02X",
									rftx[i]);
							Serial.println("]");
						}
					} else {
						buildF1();
					}
					txStaged();
				}
			}
			// connected: re-camp on the channel the session is NOW on. CRITICAL for hop-following:
			// an E4 we just processed set g_parkCh to the puck's NEW channel -- re-arm there
			// IMMEDIATELY (not on `ch`, the channel we received the E4 on), else we sit a full loop
			// iteration behind and fall off the puck's hop sequence (e3in collapses 144->0 in ~1s).
			// discovery: freeze rfrx.
			if (reply)
				armRx(base, prefix, g_parkCh ? g_parkCh : ch);
			else {
				NRF_RADIO->TASKS_DISABLE = 1;
				RWAIT_DISABLED();
				NRF_RADIO->EVENTS_DISABLED = 0;
			}
			return type;
		}
		// CRC fail. connected: END_START already re-armed -> stay camped. discovery: stop.
		if (!reply) {
			NRF_RADIO->TASKS_DISABLE = 1;
			RWAIT_DISABLED();
			NRF_RADIO->EVENTS_DISABLED = 0;
		}
		return type;
	}
	// timeout, no packet. connected: stay camped (END_START keeps RX live). discovery: stop.
	if (!reply) {
		NRF_RADIO->TASKS_DISABLE = 1;
		RWAIT_DISABLED();
		NRF_RADIO->EVENTS_DISABLED = 0;
	}
	return type;
}

// Listen on the shared "ibex"/ch2 rendezvous for a discovery E1 that matches a stored bond. On a match,
// adopt the advertised session base/prefix/channel and switch to connected.
static void discoveryScan()
{
	uint8_t t = rxOnce(g_rfBase, g_rfPrefix, 2, RX_WIN_US, false);
	if (t != 0xE1)
		return;
	const uint8_t *proteus =
		rfrx + 3; // payload [proteus_uuid 4][ibex_uuid 4]
	const uint8_t *ibex = rfrx + 7;
	int slot = ctrlBondMatch(proteus, ibex);
	if (slot < 0) {
		// We hear the puck's beacon but its UUIDs don't match any stored bond -- the controller
		// side of the pairing didn't land, or we're paired to a different puck. Very useful signal.
		static unsigned long lastMiss = 0;
		if (Serial.availableForWrite() > 40 &&
		    millis() - lastMiss > 1000) {
			lastMiss = millis();
			Serial.printf(
				"# E1 heard, no bond match: puuid=%02X%02X%02X%02X iuuid=%02X%02X%02X%02X\n",
				proteus[0], proteus[1], proteus[2], proteus[3],
				ibex[0], ibex[1], ibex[2], ibex[3]);
		}
		return;
	}
	// adopt the advertised session address (rfHostFrameOnce layout: ch@[11], base@[15..19], prefix@[19])
	uint8_t advCh = rfrx[11];
	memcpy(g_sessBase, rfrx + 15, 4);
	g_sessPrefix = rfrx[19];
	if (advCh)
		g_sessCh = advCh;
	g_linkSlot = slot;
	g_linkAliveMs = millis();
	g_parkCh = 0; // start hunting the session channel set from scratch
	g_huntIdx = 0;
	g_huntStepMs = millis();
	deckText("link: adopted session");
	if (Serial.availableForWrite() > 40)
		Serial.printf(
			"# LINK adopted slot=%d base=%02X%02X%02X%02X/%02X ch=%u\n",
			slot, g_sessBase[0], g_sessBase[1], g_sessBase[2],
			g_sessBase[3], g_sessPrefix, g_sessCh);
}

bool ctrlLinkUp()
{
	// "up" = we caught a real SESSION-address frame recently (a poll/keepalive on the adopted
	// base+channel), not merely the ch2 discovery beacon -- so the UI tile only goes tappable when
	// the puck is genuinely polling us.
	return g_linkSlot >= 0 && (millis() - g_sessRxMs) < LINK_ALIVE_MS;
}

void ctrlLinkTask()
{
	// nothing to do until at least one puck is bonded
	bool anyBond = false;
	for (int i = 0; i < NBOND; i++)
		if (g_bond[i].used)
			anyBond = true;
	if (!anyBond) {
		static unsigned long lastNb = 0;
		if (Serial.availableForWrite() > 40 &&
		    millis() - lastNb > 2000) {
			lastNb = millis();
			Serial.println(
				"# no bonds stored -- pair first (controller never got 0xEE)");
		}
		g_linkSlot = -1;
		return;
	}

	// 1 Hz status line for a serial monitor: searching vs connected, channel, RX count.
	{
		static unsigned long stMs = 0;
		static uint32_t lastRx = 0;
		if (Serial.availableForWrite() > 50 &&
		    millis() - stMs >= 1000) {
			stMs = millis();
			Serial.printf(
				"# link slot=%d %s ch=%u park=%u rx=%lu/s\n",
				g_linkSlot,
				ctrlLinkUp() ? "CONNECTED" : "searching",
				g_sessCh, g_parkCh,
				(unsigned long)(g_rfRxCount - lastRx));
			// poll-type histogram. e3in = INPUT polls (we send 0x45); e3get = identity/feature GETs.
			// If e3in stays ~0 while e3get is high, the puck is stuck enumerating and never asks for
			// input -> the controller can't show as active no matter what we send.
			Serial.printf(
				"# polls e3in=%u e3get=%u e2=%u e7=%u e4=%u(follow=%u) out=%u oth=%u getReq=%02X\n",
				g_cntE3in, g_cntE3get, g_cntE2, g_cntE7,
				g_cntE4, g_cntE4follow, g_cntOut, g_cntOther,
				g_getReq);
			// if the puck flooded E4 this second, list the distinct channels it commanded
			if (g_cntE4) {
				char cb[160];
				int n = 0;
				for (int c = 0;
				     c <= 100 && n < (int)sizeof(cb) - 5; c++)
					if (g_e4set[c >> 3] & (1 << (c & 7)))
						n += snprintf(cb + n,
							      sizeof(cb) - n,
							      "%d ", c);
				cb[n] = 0;
				Serial.printf("# E4 channels: %s\n", cb);
			}
			memset(g_e4set, 0, sizeof g_e4set);
			g_cntE2 = g_cntE3 = g_cntE7 = g_cntE4 = g_cntOut =
				g_cntOther = g_cntE3in = g_cntE3get =
					g_cntE4follow = 0;
			lastRx = g_rfRxCount;
			// re-arm the one-shot frame/GET dumps every ~8s so they stay capturable in the app log
			static uint8_t reArm = 0;
			if (++reArm >= 8) {
				reArm = 0;
				memset(g_seenReq, 0, sizeof g_seenReq);
				memset(g_seenDump, 0, sizeof g_seenDump);
			}
		}
	}

	// No session adopted yet -> hunt the E1 host frame on the shared ch2/"ibex" rendezvous.
	if (g_linkSlot < 0) {
		discoveryScan();
		return;
	}

	// Adopted a session, but the puck has gone fully dry for a long time -> abandon it and re-catch the
	// E1 (its primary channel may have changed). Only after RECONNECT_MS so a hunt sweep has a chance.
	if ((millis() - g_linkAliveMs) >= RECONNECT_MS) {
		g_linkSlot = -1;
		g_parkCh = 0;
		g_huntIdx = 0;
		deckText("link: lost, rescanning");
		if (Serial.availableForWrite() > 40)
			Serial.println("# LINK lost, rescanning ch2");
		return;
	}

	// Pick the channel to listen on: parked channel if we have one, else sweep {primary, HUNT_CH...}.
	uint8_t tryCh = g_parkCh ? g_parkCh :
				   (g_huntIdx == 0 ?
					    g_sessCh :
					    HUNT_CH[(g_huntIdx - 1) %
						    (uint8_t)sizeof HUNT_CH]);

	uint8_t t = rxOnce(g_sessBase, g_sessPrefix, tryCh, RX_WIN_US, true);

	if (t == 0xE1) {
		// keepalive beacon on the session base: refreshes the advertised primary channel (rxOnce already
		// copied it into g_sessCh) + proves the puck is present, but it is NOT a poll -> keep hunting.
		g_linkAliveMs = millis();
	} else if (t == 0xE4) {
		// E4 already adopted ITS commanded channel inside rxOnce (g_parkCh/g_sessCh = nc). Do NOT
		// re-park on the receive channel here -- that would undo the hop-follow. Aliveness already set.
	} else if (t >= 0xE0 && t <= 0xEF) {
		// a real poll (E2/E3/E7) -> the session lives on THIS channel; PARK and answer it from here.
		bool wasUp = (millis() - g_sessRxMs) < LINK_ALIVE_MS;
		g_parkCh = tryCh;
		g_sessCh = tryCh;
		g_linkAliveMs = g_sessRxMs = millis();
		if (!wasUp) {
			deckText("link: session live");
			if (Serial.availableForWrite() > 40)
				Serial.printf(
					"# SESSION live: type=%02X ch=%u base=%02X%02X%02X%02X/%02X\n",
					t, tryCh, g_sessBase[0], g_sessBase[1],
					g_sessBase[2], g_sessBase[3],
					g_sessPrefix);
		}
	} else {
		// nothing this dwell. parked channel gone dry -> resume hunting (QoS hop / puck moved)
		if (g_parkCh && (millis() - g_sessRxMs) >= LINK_ALIVE_MS)
			g_parkCh = 0;
		if (!g_parkCh && (millis() - g_huntStepMs) >= HUNT_DWELL_MS) {
			g_huntStepMs = millis();
			g_huntIdx++;
		}
	}
}
