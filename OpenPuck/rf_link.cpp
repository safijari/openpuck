#include "rf_link.h"
#include "radio.h"
#include "bonds.h"
#include "config.h"
#include "triton.h"
#include "haptics.h"
#include "puck_hid.h" // g_cmdCapture (suppress I45 during feature-command capture)
#include "controllers.h"
#include "status_led.h"
#include "fault_diag.h"
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

// ---- real-puck alignment (sniff1.json: a bonded controller RECONNECTING) ----
// The live air capture of a real puck<->controller reconnect shows the controller streams 0xF1 input in
// response to a BARE 0xE3 poll (1-byte payload, just the opcode) with NO 0xE7 awake-announce and NO
// GET-report-0x45 sub-TLV -- 1857 of the puck's 2003 polls were bare E3, and the very first session frame
// (a bare E3) was answered by F1 immediately. That contradicts the earlier RE "recipe" (rf_link.h) which
// assumed E7 + GET-0x45 were required. So these now default to the real-puck behavior; flip them at runtime
// ('d'/'n' console cmds) to fall back to the legacy GET/E7 path for an A/B comparison on hardware.
bool g_pollGet =
	false; // false = bare E3 poll (real puck); true = append GET-report-0x45 TLV (legacy)
bool g_e7announce =
	false; // false = no E7 awake-announce (real puck); true = announce host-awake (legacy)
// Session-channel E1 host-frame keepalive. The real puck sends NO E1 on its session channel (the bonded
// controller already knows the per-bond address and just resumes). OpenPuck still needs E1 because it runs
// the SHARED "ibex" address, not the (un-reversed) per-bond address -- E1 is how the controller learns this
// puck's session base/prefix/channel. So this defaults ON; turn it off ('m') to test the real-puck "no
// session E1" model once per-bond addressing exists. Discovery on ch2 is separate and always runs.
bool g_e1keepalive = true;

bool g_connVerbose = false;
// poll RX-window (us): the poll BUSY-WAITS up to this long for the controller's reply, so it is the dominant
// per-poll loop cost and directly sets the poll rate -- one slot needs (g_rxWin + overhead) < g_pollUs(4000)
// to sustain 250 Hz. 1200 is the proven 250 Hz value; it was briefly raised to 2000 (issue-72, delayed-reply
// tolerance) which dropped the rate to ~220. FIXED + not configurable (like g_pollUs): there is no good
// reason to raise it in the field, and doing so silently halves the poll rate. Any persisted/old value is
// ignored.
const uint32_t g_rxWin = 1200;
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
// last second's CRC-fail and no-reply-in-window counts (wedge diagnosis)
uint16_t g_crcps = 0, g_norxps = 0;
uint16_t g_rfStallRecover = 0;

// RF-stall self-heal thresholds. A genuine worst-case reply gap during normal play is ~1.5s (the
// hapticOnReconnect re-init window); past RF_STALL_MS with us still actively polling, the whole link is wedged,
// not merely blipping. RF_RECOVER_MS rate-limits the recovery so it can't thrash while a stalled link re-syncs.
#define RF_STALL_MS 2500u
#define RF_RECOVER_MS 2000u
// A genuinely WEDGED radio recovers within a power-cycle or two; a controller that is simply OFF / out of
// range never comes back no matter how many times we power-cycle. So after RF_STALL_GIVEUP consecutive
// recoveries that restored NO link, treat it as "controller absent" and stop hammering: back the power-cycle
// off to RF_STALL_BACKOFF_MS (a slow safety-net kick). Discovery beacons keep running throughout, so a
// returning controller still reconnects normally without the power-cycle -- and the aggressive 2s cadence was
// both wasteful and able to disrupt a controller mid-reconnect. The counter resets the moment any slot replies.
#define RF_STALL_GIVEUP 3u
#define RF_STALL_BACKOFF_MS 30000u
// measured avg us between GET-poll fires (vs intended g_pollUs)
uint16_t g_pollPeriodUs = 0;
static uint32_t g_pollDtSum = 0;
static uint16_t g_pollDtCnt = 0;
// smoothed |dBm| of the controller's replies, per slot (0 = none yet)
volatile uint8_t g_linkRssi[NSLOT] = { 0 };
// battery % from the controller's report 0x43 (body[1]); 0 = none yet. Per-slot -- the active controller's
// battery is the most recently seen one (other slots' values stay in their own array slots).
volatile uint8_t g_battery[NSLOT] = { 0 };
// charge state from report 0x43 body[0] (EChargeState: 1=discharging 2=charging 4=charging-done; 0=unknown).
volatile uint8_t g_batteryState[NSLOT] = { 0 };

// Slot the poll loop is currently driving. Set by rfConnStep before each E7/relay/E3, consumed by the
// decode (g_in[g_curSlot]), the haptic flush (per-slot session address), and the per-second stat dump.
int g_curSlot = -1;

// ---- internal counters / timers ----
// ESB PID is 2 bits. The controller dequeues a FRESH report only when the poll's PID differs from the
// last one it saw on that pipe; a repeated PID reads as a retransmit and returns the SAME (stale) report.
// PER SLOT, because a single shared counter advances by (2 * nWarm) per cycle -- with 2 controllers that's
// +4 per cycle = 0 mod 4, so each slot's GET PID is constant => the controller never dequeues => ~60 new/s
// instead of ~400. Each slot's counter increments once per poll-of-that-slot so it cycles 0,1,2,3 cleanly.
static uint8_t g_pollPid[NSLOT] = {};
static uint8_t g_relayPid[NSLOT] = {};
// All link statistics are PER SLOT: each controller's polls/replies/errors are counted (and reported --
// serial stat line, WebUSB blob v13) against that controller only. The old scalar counters merged every
// slot into one number, so the panel couldn't tell "controller B is drowning" from "everything is slow".
// The legacy aggregate snapshots (g_pollsps & co) are now sums over slots, kept for the serial line and the
// blob's pre-v13 fields.
static uint32_t g_stPoll[NSLOT] = {}, g_stF1[NSLOT] = {};
static uint32_t g_stF3 = 0;
// g_stPoll counts true poll CYCLES (one E3 GET per warm slot per cycle). g_stRelay counts relay frames TX'd
// (host/haptic output reports). They were conflated before (every rfConnTx bumped one counter), which made
// "Polls/s" read ~540 (250 polls + ~290 relays) and hid that each relay steals a reply window from its poll.
static uint32_t g_stRelay[NSLOT] = {};
uint16_t g_relayps = 0;
// per-slot per-second snapshots (WebUSB blob v13 / per-controller panel stats)
uint16_t g_slotPollsps[NSLOT] = {}, g_slotF1ps[NSLOT] = {},
	 g_slotNewps[NSLOT] = {};
uint8_t g_slotCrcps[NSLOT] = {}, g_slotNoRxps[NSLOT] = {},
	g_slotRelayps[NSLOT] = {};
static unsigned long g_stMs = 0;
// Per-slot dedupe seq + per-slot new-report counter (the real puck sends 0x45 per controller; merging all
// slots into a single sequence makes one controller "swallow" the other's frame).
static uint8_t g_lastSeq[NSLOT] = { 0 };
static uint32_t g_stNew[NSLOT] = {};
static uint32_t g_stCrc[NSLOT] = {}, g_stNoRx[NSLOT] = {};
static uint32_t g_chF1[3] = { 0, 0, 0 };
// Cycle gate: fires once per g_pollUs; each fire polls every warm slot so all run at ~250 Hz.
static uint32_t g_lastPollUs = 0;
static uint32_t g_connRx = 0;
static unsigned long g_lastSessBeacon = 0, g_lastDisc = 0;
static unsigned long g_lastStream = 0;

// HOST FRAME the bonded controller waits for (IBEX FUN_00019000 verify: b[0]=0x12, b[5]=0xE1, b[6..10]=
// proteus_uuid, b[10..14]=ibex_uuid). Built like PROTEUS FUN_00027e9a. Sent on the shared rendezvous addr;
// the controller filters by the uuids in the payload, then connects.
// Transmit one host frame. `discovery`=true sends it on the SHARED rendezvous address ("ibex"/ch2) where a
// searching controller looks; =false sends it on this slot's unique SESSION address (the keepalive once the
// controller has adopted the session). EITHER way the payload advertises the session base/prefix/channel,
// so the controller always learns the unique address to connect on.

// Any-slot link helper: true if ANY bonded slot is currently hearing F-type replies (within 300 ms).
// Used for the "we're connected to at least one controller" decisions (beacon pacing, wake detect).
bool anySlotLinkUp()
{
	for (int s = 0; s < NSLOT; s++)
		if (g_slot[s].used && millis() - g_connReplyMs[s] < 300)
			return true;
	return false;
}

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
	// payload[13..17] session base  (the per-bond UNIQUE address; each controller adopts its own)
	memcpy(rftx + 15, g_sessBase[slot], 4);
	rftx[19] = g_sessPrefix[slot]; // payload[17] session prefix
	// TX address: discovery uses the shared "ibex" rendezvous; the session keepalive uses this slot's
	// unique address (where THIS controller now listens). The advertised session params (above) are
	// identical either way -- so the discovery frame can also double as a re-advertisement if needed.
	const uint8_t *txBase = discovery ? g_rfBase : g_sessBase[slot];
	uint8_t txPfx = discovery ? g_rfPrefix : g_sessPrefix[slot];
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
		// non-blocking: don't stall the loop on CDC backpressure (whole line ~165B; CDC write() has no timeout)
		if (Serial.availableForWrite() > 180) {
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
	// QoS hop is shared across all slots -- we run all connected sessions on the same channel for simplicity.
	// The per-slot session ADDRESS is what isolates the controllers from each other; the channel is global.
	if (g_curSlot < 0 || newCh == g_sessCh)
		return;
	uint8_t cur = g_sessCh, savedRfCh = g_rfCh;
	g_sessCh = newCh;
	// host frame now advertises newCh but is TXed on cur (per-slot session addr)
	g_rfCh = cur;
	for (int s = 0; s < NSLOT; s++)
		for (int k = 0; k < 6; k++) {
			rfHostFrameOnce(s, false);
			delayMicroseconds(700);
		}
	g_rfCh = savedRfCh; // poll + session beacon now run on g_sessCh=newCh
}

// TX one connected packet [LEN][S1][payload] on channel ch, then RX the reply into rfrx; decodes 0xF1.
// rxWinUs overrides the reply-wait window (0 = use g_rxWin). Pass a tiny value for NO-ACK relays that expect
// no reply, so they don't burn a full ~1.2ms window of dead air per haptic. Per-slot: the connected poll runs
// on this slot's UNIQUE session address (one address per bonded controller); replies are demuxed by which
// address the controller answers on (each controller only hears its own).
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
	// connected poll runs on this slot's UNIQUE session addr (per-bond). g_curSlot is set by rfConnStep
	// before each call; fall back to slot 0 if not (e.g. rf_diag paths).
	int slot = (g_curSlot >= 0 && g_curSlot < NSLOT) ? g_curSlot : 0;
	rfSetAddr(g_sessBase[slot], g_sessPrefix[slot]);
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
			g_stCrc[slot]++;
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
				int s = (g_curSlot >= 0 && g_curSlot < NSLOT) ?
						g_curSlot :
						0;
				// A reply after a long gap (or the first ever) = a (re)connect. Arm the haptic block + re-init HERE,
				// directly off the reply stream -- reliable even when hapticTask's 300ms link-up edge doesn't fire
				// (e.g. a power-cycled controller that reconnects without us cleanly seeing the link drop).
				if (g_connReplyMs[s] == 0 ||
				    (uint32_t)(millis() - g_connReplyMs[s]) >
					    1500u) {
					// Lifecycle log (CDC debug boot): distinguish a first-ever connect from a reconnect and
					// print the silent gap -- lets a long session of connect/disconnect cycles be diffed to
					// see whether each cycle re-establishes (churn / boot-haptic click) vs stays linked.
					if (Serial.availableForWrite() > 130) {
						uint32_t gap =
							g_connReplyMs[s] ?
								(uint32_t)(millis() -
									   g_connReplyMs
										   [s]) :
								0;
						Serial.printf(
							"# LC t=%lu slot%d %s gap=%lums rtype=%02X cd=%lums\n",
							(unsigned long)millis(),
							s,
							g_connReplyMs[s] ?
								"RECONNECT" :
								"CONNECT",
							(unsigned long)gap,
							rtype,
							(unsigned long)(millis() -
									g_connCooldown));
					}
					hapticOnReconnect(s);
					faultDiagTrace(FR_RFUP, s);
				}
				g_connRx++;
				// link alive -> loop() suppresses the redundant E1 beacon
				g_connReplyMs[s] = millis();
				// |dBm| of this reply (started by the ADDRESS short)
				uint8_t rs =
					(uint8_t)(NRF_RADIO->RSSISAMPLE & 0x7F);
				// EWMA, ~8-sample horizon, per-slot
				if (rs)
					g_linkRssi[s] =
						g_linkRssi[s] ?
							(uint8_t)((g_linkRssi[s] *
									   7u +
								   rs + 4u) /
								  8u) :
							rs;
			}
			if (rtype == 0xF1)
				g_stF1[slot]++;
			// controller disconnecting/powering off -> back off 2.5s so we don't immediately re-wake it.
			// BUT only when no OTHER slot is still live: g_connCooldown is global and gates ALL polling +
			// beacons, so backing off because ONE controller powered off would drop every other connected
			// controller for 2.5s (a real multi-controller disconnect). The powering-off slot goes silent
			// and ages out via SLOT_COLD on its own.
			if (rtype == 0xF2) {
				int others = 0;
				for (int i = 0; i < NSLOT; i++)
					if (i != g_curSlot && g_slot[i].used &&
					    millis() - g_connReplyMs[i] < 300)
						others++;
				faultDiagTrace(FR_RFDN,
					       (uint16_t)((g_curSlot << 8) |
							  (others & 0xFF)));
				if (others == 0)
					g_connCooldown = millis();
				// Lifecycle log: the controller sent F2 (disconnect/power-off). Note whether it armed the
				// 2.5s cooldown (which pauses ALL beacon+poll -> the controller can lose the session and
				// reboot on the next reconnect = a boot-haptic click). Prime suspect for the connect buzz.
				if (Serial.availableForWrite() > 130)
					Serial.printf(
						"# LC t=%lu slot%d F2 DISCONNECT others=%d%s\n",
						(unsigned long)millis(),
						g_curSlot, others,
						others == 0 ?
							" -> cooldown 2.5s (beacon+poll paused)" :
							"");
			}
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
			// Every rfConnTx caller (rfConnStep) sets g_curSlot to a valid 0..NSLOT-1 before a poll, so it
			// IS in range here today. But this block does ~50 unguarded g_curSlot array writes (g_in[],
			// g_lastSeq[], and several static per-slot arrays) -- if any future/edge caller ever left
			// g_curSlot at -1, those become out-of-bounds writes that corrupt RAM (the stack-smash / LOCKUP
			// class we're chasing). Guard once at the top so the decode is robust by construction.
			if (isF1 && (g_curSlot < 0 || g_curSlot >= NSLOT))
				isF1 = false;
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
					// Only a FULL input report that fits entirely in rfrx: a short or late/garbled TLV must not let
					// the decode read past the RF buffer (corrupts rftx/RAM -> eventual crash).
					// Main input report id: 0x45 (legacy, 45B body) OR 0x42 (SC2 beta update ~2026-07, 53B body).
					// VERIFIED from live captures of both: the 0x42 body [0..45] is byte-for-byte the SAME layout as
					// 0x45 (buttons/triggers/sticks/pads/IMU at identical offsets) -- it just adds 8 trailing bytes
					// ([46..47]=0x7FFF const, [48..53]=0) and sets two extra always-on status bits (28/29) that no
					// mode reads. So both decode through this ONE path unchanged; rep[0] carries the id downstream
					// (Steam forwards it verbatim under the right id/length in onReport45).
					if (ttype == 6 && tlen >= 28 &&
					    (size_t)(idx + 2) + tlen <=
						    sizeof(rfrx) &&
					    (rfrx[idx + 2] == 0x45 ||
					     rfrx[idx + 2] == 0x42)) {
						// report 0x45/0x42: [id][seq][buttons u32]...
						const uint8_t *rep =
							&rfrx[idx + 2];
						bool fresh =
							(rep[1] !=
							 g_lastSeq[g_curSlot]);
						// genuine new report vs stale poll-repeat
						if (fresh) {
							g_stNew[g_curSlot]++;
							g_lastSeq[g_curSlot] =
								rep[1];
						}
						uint32_t bb = btnsOf(rep);
						// USB remote wakeup on Steam button short press (down + up within 1 s). A long press likely means
						// the user is powering off the controller, so we ignore it.
						{
							// per-slot: with round-robin polling, a shared static gets
							// reset by other slots' reports between press and release.
							static bool steamWasDown
								[NSLOT] = {};
							static unsigned long steamDownMs
								[NSLOT] = {};
							if (fresh) {
								bool steamNow =
									(bb &
									 TB_STEAM) !=
									0;
								if (steamNow &&
								    !steamWasDown
									    [g_curSlot])
									steamDownMs[g_curSlot] =
										millis();
								if (!steamNow &&
								    steamWasDown
									    [g_curSlot] &&
								    millis() - steamDownMs[g_curSlot] <
									    1000u &&
								    USBDevice
									    .suspended()) {
									USBDevice
										.remoteWakeup();
									ledWakePulse();
									if (g_active)
										g_active->wakeEvent();
								}
								steamWasDown[g_curSlot] =
									steamNow;
							}
						}
						// Decode the report into the shared g_in (one source, read by every IController).
						g_in[g_curSlot].buttons = bb;
						// Global power-off chord: Steam + Y held 2 s -> shut the controller down (any mode). Detect on the raw
						// `bb` (pre-mask), time-based (poll rate varies), fires once per hold, re-arms only after release. While
						// held, mask Steam+Y out of g_in[g_curSlot].buttons so the press doesn't leak to the host -- in EVERY mode except
						// regular Steam (mode 0 forwards the raw 0x45 to Steam, which owns the Steam button). Runs before
						// onReport45 below so push modes (Xbox) see the mask too; stream modes read g_in in task() (also masked).
						{
							// Per-slot: each controller's Steam+Y hold is independent, so the debounce timer must
							// be per-slot. With multiple controllers, a shared timer gets reset every time ANOTHER
							// slot's poll (without the hold) runs -- and the round-robin poll cycles through every
							// used slot, so the timer never reaches 2s.
							static unsigned long
								offHoldMs[NSLOT] = {
									0, 0, 0,
									0
								};
							static bool offFired
								[NSLOT] = {
									false,
									false,
									false,
									false
								};
							if ((bb & (TB_STEAM |
								   TB_Y)) ==
							    (TB_STEAM | TB_Y)) {
								if (offHoldMs[g_curSlot] ==
								    0)
									offHoldMs[g_curSlot] =
										millis();
								else if (
									!offFired[g_curSlot] &&
									(unsigned long)(millis() -
											offHoldMs[g_curSlot]) >=
										2000u) {
									offFired[g_curSlot] =
										true;
									hapticSendShutdown();
								}
								if (g_usbMode !=
								    MODE_STEAM) {
									// stream modes read g_in
									g_in[g_curSlot]
										.buttons &=
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
								offHoldMs[g_curSlot] =
									0;
								offFired[g_curSlot] =
									false;
							}
						}
						g_in[g_curSlot].lx =
							(int16_t)s16off(rep, 8);
						g_in[g_curSlot].ly =
							(int16_t)s16off(rep,
									10);
						g_in[g_curSlot].rx =
							(int16_t)s16off(rep,
									12);
						g_in[g_curSlot].ry =
							(int16_t)s16off(rep,
									14);
						g_in[g_curSlot].lt =
							trigU8(u16off(rep, 4));
						// for the Switch digital-trigger threshold
						g_in[g_curSlot].rt =
							trigU8(u16off(rep, 6));
						g_in[g_curSlot].lpx =
							(int16_t)s16off(rep,
									16);
						g_in[g_curSlot].lpy =
							(int16_t)s16off(rep,
									18);
						g_in[g_curSlot].rpx =
							(int16_t)s16off(rep,
									22);
						g_in[g_curSlot].rpy =
							(int16_t)s16off(rep,
									24);
						// IMU lives at report bytes 0x22..0x2D (rep[34..45]). Decode it ONLY when a FULL 46-byte report was
						// actually received -- bounded by `end` (the received length), NOT sizeof(rfrx). The outer gate is
						// tlen>=28 (enough for buttons/sticks/pads, which end at rep[27]), so a short 0x45 (button-only, or
						// one whose IMU tail was lost) still passes it; without this guard imuFrom45 would read STALE bytes
						// past the received data and clobber g_in's gyro/accel. On a short frame, hold the last good IMU.
						if (tlen >= 46 &&
						    (size_t)(idx + 2) + 46 <=
							    (size_t)end)
							imuFrom45(
								rep,
								&g_in[g_curSlot]
									 .ax,
								&g_in[g_curSlot]
									 .ay,
								&g_in[g_curSlot]
									 .az,
								&g_in[g_curSlot]
									 .gx,
								&g_in[g_curSlot]
									 .gy,
								&g_in[g_curSlot]
									 .gz);
						// Mode-switch chord (all 4 back + face): don't leak the face press to the host. g_in[g_curSlot].buttons stays
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
								g_curSlot, rep,
								fresh, tlen);
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
						// 0x43 body[0] = ucChargeState (EChargeState), body[1] = ucBatteryLevel % (sniff-derived).
						// rep[0]=rid, rep[1]=body[0], rep[2]=body[1]. Snapshot both for the WebUSB panel and the
						// synthesized full-length 0x43 the puck pushes for SDL (see puck_hid.cpp task()).
						if (rep[0] == 0x43 &&
						    tlen >= 3 &&
						    g_curSlot >= 0 &&
						    g_curSlot < NSLOT) {
							g_batteryState[g_curSlot] =
								rep[1];
							g_battery[g_curSlot] =
								rep[2];
						}
						if (g_active)
							g_active->onAuxReport(
								g_curSlot,
								rep[0], rep + 1,
								(uint8_t)(tlen -
									  1));
					} else if (ttype == 6 &&
						   (size_t)(idx + 2) + tlen <=
							   sizeof(rfrx) &&
						   tlen >= 1) {
						// DIAGNOSTIC (beta-update RE): a type-6 HID-report TLV whose report id we
						// DON'T decode (not 0x45 input, not 0x43/0x44 status). If the new controller
						// firmware moved input off 0x45, its record shows up here. Log rid + full body,
						// rate-limited + non-blocking so it can't stall the loop. Remove once pinned.
						static unsigned long lastUnk =
							0;
						if (Serial.availableForWrite() >
							    150 &&
						    millis() - lastUnk >= 200) {
							lastUnk = millis();
							Serial.printf(
								"UNK rid=%02X tlen=%u: ",
								rfrx[idx + 2],
								tlen);
							for (uint8_t i = 0;
							     i < tlen; i++)
								Serial.printf(
									"%02X",
									rfrx[idx +
									     2 +
									     i]);
							Serial.println();
						}
					}
					idx += tlen + 2;
				}
				// mode-switch chord (back4 + face): A=always Steam; B/X/Y=configurable (g_chordBtn[]). Debounced.
				{
					// Per-slot debounce: the chord input is per-slot (g_in[g_curSlot]), so the debounce counter
					// must be too. The shared-static form worked with 1 controller because slot 0 polled
					// back-to-back; with N>1, the round-robin poll cycles through every used slot, and the OTHER
					// slots' non-chord reports reset the counter on every iteration. The counter could never
					// reach 12 with multiple controllers, making the chord effectively dead.
					static uint8_t chWant[NSLOT] = {
						0xFF, 0xFF, 0xFF, 0xFF
					};
					static uint8_t chCnt[NSLOT] = { 0, 0, 0,
									0 };
					uint8_t want = 0xFF;
					if ((g_in[g_curSlot].buttons &
					     CHORD_BACK4) == CHORD_BACK4) {
						if (g_in[g_curSlot].buttons &
						    TB_A)
							want = MODE_STEAM;
						else if (g_in[g_curSlot].buttons &
							 TB_B)
							want = g_chordBtn[0];
						else if (g_in[g_curSlot].buttons &
							 TB_X)
							want = g_chordBtn[1];
						else if (g_in[g_curSlot].buttons &
							 TB_Y)
							want = g_chordBtn[2];
					}
					if (want != 0xFF &&
					    want == chWant[g_curSlot]) {
						if (++chCnt[g_curSlot] >= 12 &&
						    want != g_usbMode &&
						    modeValid(want) &&
						    !USBDevice.suspended()) {
							saveMode(want);
							delay(40);
							faultDiagArmIntentionalReset();
							NVIC_SystemReset();
						}
					} else {
						chWant[g_curSlot] = want;
						chCnt[g_curSlot] =
							(want != 0xFF) ? 1 : 0;
					}
				}
				// compact stream for rf_controller_ui.py -- NON-BLOCKING: skip if CDC TX is backed up (a blocking
				// Serial.print stalls the RF+USB loop -> jaggy input). One line/frame using the last record.
				if (lastRep && !g_connVerbose &&
				    !g_cmdCapture &&
				    Serial.availableForWrite() > 150 &&
				    millis() - g_lastStream >= 4) {
					g_lastStream = millis();
					Serial.print("I45 ");
					for (uint8_t i = 0; i < lastTlen; i++)
						Serial.printf("%02X",
							      lastRep[i]);
					Serial.println();
				}
			}
			if (g_connVerbose && Serial.availableForWrite() > 180) {
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
		g_stNoRx[slot]++;
		g_qosBad++;
	}
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	return rxlen;
}

// Throttle: bonded slots that haven't replied in SLOT_COLD_MS are polled at most every SLOT_COLD_RETRY_MS
// instead of every cycle. This keeps the online controllers at full 250 Hz while barely touching offline ones.
#define SLOT_COLD_MS 5000u
#define SLOT_COLD_RETRY_MS 2000u
// Quiet tier: a slot that WAS replying but has been silent past SLOT_QUIET_MS (controller powering off, out
// of range) backs off to SLOT_QUIET_RETRY_MS retries instead of full-rate polling until SLOT_COLD_MS demotes
// it. Without this, a power-off left the slot at 250 Hz for 5 s with EVERY poll burning the whole g_rxWin
// (~2 ms) waiting for a reply that never comes -- with two controllers powered off together that is ~100%
// radio duty, the churn window the power-off watchdog hang (issue #72 repro) lives in. Recovery stays snappy:
// the first reply re-warms the slot to full rate, so a controller returning from a fade waits <= one retry.
#define SLOT_QUIET_MS 300u
#define SLOT_QUIET_RETRY_MS 50u
static unsigned long g_slotLastAttemptMs[NSLOT] = {};

// Drive the connected-mode sequence. The cycle gate fires once per g_pollUs (250 Hz); each fire
// polls EVERY warm slot back-to-back so all bonded controllers run at full rate regardless of count.
// "Cold" means the slot WAS connected but has been silent for > SLOT_COLD_MS; slots that have never
// replied stay warm so new controllers can connect at any time after boot. Cold slots retry every
// SLOT_COLD_RETRY_MS.
static void rfConnStep()
{
	int firstUsed = -1;
	for (int k = 0; k < NSLOT; k++)
		if (g_slot[k].used) {
			firstUsed = k;
			break;
		}
	if (firstUsed < 0) {
		g_curSlot = -1;
		return;
	}

	uint8_t ch = g_sessCh;
	// announce HOST AWAKE: E7 00 00, a few times. The real puck does NOT do this (the controller streams F1 to a
	// bare E3 with no E7) -- skipped unless g_e7announce ('n') re-enables the legacy handshake.
	if (g_connSt == 0) {
		g_curSlot = firstUsed;
		if (g_e7announce) {
			uint8_t p[3] = { 0xE7, 0x00, g_e7b };
			rfConnTx(ch, 0x01, p, 3);
			if (++g_connStep >= 4) {
				g_connSt = 1;
				g_connStep = 0;
				Serial.println(
					"# CONN: awake announced -> polling");
			}
		} else {
			// real-puck path: straight to the bare-E3 poll loop, no E7
			g_connSt = 1;
			g_connStep = 0;
		}
		return;
	}

	// Cycle gate: fires once per g_pollUs (250 Hz). On each fire we poll EVERY warm slot
	// back-to-back so all bonded controllers run at full rate -- the real puck services all
	// controllers per cycle. Polling one-slot-per-call instead tied the per-slot rate to the
	// loop frequency AND split it across slots, collapsing 2 controllers to ~90 Hz.
	uint32_t nowUs = micros();
	if ((uint32_t)(nowUs - g_lastPollUs) < (uint32_t)g_pollUs)
		return;
	{
		// Cycle period stat: time between gate fires (= each slot's poll period, intended g_pollUs).
		static uint32_t lastCycleUs = 0;
		if (lastCycleUs) {
			g_pollDtSum += (uint32_t)(nowUs - lastCycleUs);
			g_pollDtCnt++;
		}
		lastCycleUs = nowUs;
	}
	g_lastPollUs += g_pollUs;
	if ((uint32_t)(nowUs - g_lastPollUs) >= (uint32_t)g_pollUs)
		g_lastPollUs = nowUs; // catch-up reset when a cycle overran

	unsigned long nowMs = millis();

	// Inline poll helper; emits E7 re-assert (every 32 polls, bounded reply window so a
	// missed F3 doesn't burn the whole slot budget), queues haptics, flushes relay, sends E3 GET.
	auto doPoll = [&](int k) {
		g_slotLastAttemptMs[k] = nowMs;
		g_curSlot = k;
		// re-assert awake/version every 32 polls (legacy; real puck never sends E7 -- gated by g_e7announce)
		if (g_e7announce && (g_connPoll & 0x1F) == 0) {
			uint8_t pa[3] = { 0xE7, 0x00, g_e7b };
			rfConnTx(ch, 0x01, pa, 3, 600);
		}
		rfConnQueueHapticRelay();
		{
			uint8_t rs1 =
				(uint8_t)((((g_relayPid[k]++) & 3) << 1) | 1);
			if (rfConnFlushRelay(ch, rs1))
				g_stRelay[k]++;
		}
		// per-slot PID cycle so each bonded controller's polls stay distinct
		uint8_t pidv = g_pollPid[k]++;
		uint8_t s1 = (g_e3mode == 1) ?
				     (uint8_t)(((pidv & 3) << 1) | 1) :
			     (g_e3mode == 2) ? (uint8_t)((pidv & 3) << 1) :
					       0x07;
		uint8_t rx;
		if (g_pollGet) {
			// legacy: E3 + TLV [len=02][subtype=01 GET][id=0x45][param]
			uint8_t p[5] = { 0xE3, 0x02, 0x01, 0x45, g_getParam };
			rx = rfConnTx(ch, s1, p, 5);
		} else {
			// real puck: BARE E3 (just the opcode) -- the controller streams F1 to any E3 ack
			uint8_t p[1] = { 0xE3 };
			rx = rfConnTx(ch, s1, p, 1);
		}
		if (rx)
			g_chF1[0]++;
		g_stPoll[k]++; // one true poll cycle for this slot
		g_connPoll++;
	};

	// Poll a slot every cycle (full rate) only when its controller is actually here. Throttle the rest to
	// every SLOT_COLD_RETRY_MS:
	//   - "cold": WAS connected but silent > SLOT_COLD_MS (controller powered off / out of range);
	//   - "phantom": NEVER replied AND another controller is already connected. This is the cloned-puck case
	//     (issue: bonds copied from a backup include controllers that aren't present). Polling a phantom slot
	//     every cycle was doubling/tripling Polls/s, flooding noRx, and stealing reply windows from the live
	//     controller. A never-replied slot with nothing else connected still polls full-rate, so the FIRST
	//     controller connects instantly; a phantom only backs off once a real link exists to protect.
	bool linkUp = anySlotLinkUp();
	for (int k = 0; k < NSLOT; k++) {
		if (!g_slot[k].used)
			continue;
		bool everReplied = g_connReplyMs[k] != 0;
		unsigned long silentMs =
			everReplied ? (nowMs - g_connReplyMs[k]) : 0;
		bool cold = everReplied && silentMs > SLOT_COLD_MS;
		bool quiet = everReplied && !cold && silentMs > SLOT_QUIET_MS;
		bool phantom = !everReplied && linkUp;
		unsigned long retry = (cold || phantom) ? SLOT_COLD_RETRY_MS :
				      quiet		? SLOT_QUIET_RETRY_MS :
							  0;
		if (retry && nowMs - g_slotLastAttemptMs[k] < retry)
			continue;
		doPoll(k);
	}
}

void rfLinkTask()
{
	// Host-frame beacon: sent continuously, INCLUDING while connected. The controller uses the periodic E1 (the
	// real puck's per-hop-cycle announce) to stay synced and keep answering polls at full rate; suppressing it
	// drops the reply rate from ~210/s to ~38/s. Paused only during the post-disconnect cooldown so a controller
	// that's powering off isn't immediately re-woken/reconnected.
	if (g_rfHost && millis() - g_connCooldown > 2500) {
		bool connNow = anySlotLinkUp();
		// session keepalive on the clean channel: every loop while connecting (fast), every 25ms once connected
		// (every-loop beaconing also hammers the session ch and steals reply slots from the poll). The real puck
		// sends NO E1 on its session channel; gated by g_e1keepalive ('m') so this can be A/B'd on hardware --
		// but it stays ON by default because OpenPuck's shared-address model relies on E1 to advertise the session.
		if (g_e1keepalive &&
		    millis() - g_lastSessBeacon >= (connNow ? 25u : 0u)) {
			g_lastSessBeacon = millis();
			g_rfCh = g_sessCh;
			for (int s = 0; s < NSLOT; s++)
				rfHostFrameOnce(s, false);
		}
		// discovery beacon on ch2 (where a searching controller looks): every loop when nothing is
		// connected (fastest cold-boot/late-joiner connect), every 200ms once ANY controller is linked.
		// Matches main: while a slot is connected, ch2 discovery shares g_sessCh's air budget, so beaconing
		// faster than 200ms (the old allUp-gated 40ms path) just steals reply windows from the connected
		// controller -> reply-rate sag -> >1500ms gaps -> spurious hapticOnReconnect re-init. A late joiner
		// still enumerates within ~1s at 200ms.
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

	// ---- RF stall self-heal -------------------------------------------------------------------------------
	// Field wedge: the controllers stay powered and show a SOLID (connected) light, the puck keeps issuing E3
	// polls + E1 beacons (radio TX is provably alive -- a power-cycled controller still re-adopts the session
	// and goes solid against this puck), yet NOT ONE slot decodes a reply for seconds. The link reads down, no
	// input reaches the host, and it never recovers on its own -- only a puck REPLUG fixes it. rfConfig() already
	// re-disables (TASKS_DISABLE) the radio before every poll, so whatever this is survives a TASKS_DISABLE; the
	// only thing that clears it is a full peripheral reset -- exactly what the replug does. So when the WHOLE
	// link has been silent past RF_STALL_MS while we're still actively polling a slot that WAS connected, do the
	// replug's job for the radio: power-cycle NRF_RADIO (POWER=0/1 clears every RADIO register; the next
	// rfConnTx/rfHostFrameOnce re-applies all config via rfConfig()) and drop the connected-mode latches so
	// polling/beaconing resume and the handshake re-runs from a clean slate. Gated on EVERY used+previously-
	// connected slot being stalled, so one controller walking off (others still live) never trips it; the
	// cooldown gate keeps it from firing while a controller is intentionally powering off; rate-limited so it
	// cannot thrash. g_rfStallRecover is surfaced to the panel so the wedge -- and its recovery -- is observable.
	if (g_connOn && g_curSlot >= 0 && millis() - g_connCooldown > 2500) {
		static unsigned long lastRecoverMs = 0;
		static uint8_t consecStall = 0;
		unsigned long nowMs2 = millis();
		bool anyEverConnected = false, allStalled = true;
		for (int s = 0; s < NSLOT; s++) {
			if (!(g_slot[s].used && g_connReplyMs[s] != 0))
				continue;
			anyEverConnected = true;
			if ((uint32_t)(nowMs2 - g_connReplyMs[s]) < RF_STALL_MS)
				allStalled = false;
		}
		// Any slot alive -> a recovery worked (or we were never stalled): reset the give-up counter.
		if (!allStalled)
			consecStall = 0;
		// Back off once we've power-cycled RF_STALL_GIVEUP times with nothing coming back (controller absent,
		// not a wedged radio): slow the retry to RF_STALL_BACKOFF_MS instead of hammering every RF_RECOVER_MS.
		uint32_t interval = (consecStall < RF_STALL_GIVEUP) ?
					    RF_RECOVER_MS :
					    RF_STALL_BACKOFF_MS;
		if (anyEverConnected && allStalled &&
		    (uint32_t)(nowMs2 - lastRecoverMs) > interval) {
			lastRecoverMs = nowMs2;
			g_rfStallRecover++;
			faultDiagTrace(FR_HEAL, g_rfStallRecover);
			if (consecStall < 255)
				consecStall++;
			NRF_RADIO->POWER = 0;
			NRF_RADIO->POWER = 1;
			g_connCooldown = 0;
			g_connSt = 0;
			g_connStep = 0;
			if (Serial.availableForWrite() > 110)
				Serial.printf(
					"# RF STALL (#%u consec=%u%s) -> radio power-cycle + reconnect\n",
					g_rfStallRecover, consecStall,
					consecStall >= RF_STALL_GIVEUP ?
						" BACKOFF" :
						"");
		}
	}

	{
		// remote wakeup on new RF controller connection (any slot)
		static bool wasRfConn = false;
		bool nowRfConn = anySlotLinkUp();
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
	if (g_qos && g_curSlot >= 0 && millis() - g_qosCheckMs >= 600) {
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
		// Per-slot snapshots first (blob v13 / per-controller panel stats), then the legacy aggregates as
		// their sums (serial stat line + pre-v13 blob fields).
		uint32_t tPoll = 0, tF1 = 0, tNew = 0, tCrc = 0, tNoRx = 0,
			 tRelay = 0;
		for (int s = 0; s < NSLOT; s++) {
			g_slotPollsps[s] = (uint16_t)g_stPoll[s];
			g_slotF1ps[s] = (uint16_t)g_stF1[s];
			g_slotNewps[s] = (uint16_t)g_stNew[s];
			g_slotCrcps[s] =
				(uint8_t)(g_stCrc[s] > 255 ? 255 : g_stCrc[s]);
			g_slotNoRxps[s] = (uint8_t)(g_stNoRx[s] > 255 ?
							    255 :
							    g_stNoRx[s]);
			g_slotRelayps[s] = (uint8_t)(g_stRelay[s] > 255 ?
							     255 :
							     g_stRelay[s]);
			tPoll += g_stPoll[s];
			tF1 += g_stF1[s];
			tNew += g_stNew[s];
			tCrc += g_stCrc[s];
			tNoRx += g_stNoRx[s];
			tRelay += g_stRelay[s];
			g_stPoll[s] = g_stF1[s] = g_stNew[s] = 0;
			g_stCrc[s] = g_stNoRx[s] = g_stRelay[s] = 0;
		}
		g_f1ps = (uint16_t)tF1;
		g_newps = (uint16_t)tNew;
		g_pollsps = (uint16_t)tPoll;
		g_relayps = (uint16_t)tRelay;
		g_crcps = (uint16_t)tCrc;
		g_norxps = (uint16_t)tNoRx;
		g_pollPeriodUs =
			g_pollDtCnt ? (uint16_t)(g_pollDtSum / g_pollDtCnt) : 0;
		g_pollDtSum = 0;
		g_pollDtCnt = 0;
		// Require room for the WHOLE line (~85B): CDC write() has NO timeout -- it spins yield() until the host
		// drains, so a guard smaller than the line lets write() start, fill the FIFO mid-line, then spin loop()
		// forever if the serial host stalls -> watchdog hang. (This exact line, guarded at >70 for an ~85B line,
		// was the confirmed diagnostic-induced hang: the capture ended truncated mid-"# stat".)
		if (Serial.availableForWrite() > 130)
			Serial.printf(
				"# stat polls=%lu/s F1=%lu/s new=%lu/s F3=%lu/s(v%d) e7b=%u crcfail=%lu noRx=%lu slot=%d\n",
				(unsigned long)tPoll, (unsigned long)tF1,
				(unsigned long)tNew, (unsigned long)g_stF3,
				(int8_t)g_connF3v, g_e7b, (unsigned long)tCrc,
				(unsigned long)tNoRx, g_curSlot);
		g_stF3 = 0;
		g_chF1[0] = g_chF1[1] = g_chF1[2] = 0;
		g_stMs = millis();
	}
}
