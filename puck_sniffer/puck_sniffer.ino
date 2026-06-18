// puck_sniffer.ino -- PASSIVE 2.4GHz sniffer for the Valve SC2 puck <-> controller link.
//
// Flash this onto a SECOND nRF52840 board (NOT the one emulating a puck). It never transmits, so it cannot
// disturb the real puck<->controller pairing -- it only listens. It reuses OpenPuck's CRC-validated radio
// config (radio.cpp): PHY Ble_2Mbit, ENDIAN=Big, no whitening, CRC16 0x11021/0xFFFF, 5-byte ESB address.
//
// HOW IT FINDS THE SESSION
//   The puck and controller talk on a per-puck SESSION address/channel. The puck advertises those in its E1
//   host-frame beacon, which it sends on the SHARED discovery rendezvous ("ibex" / prefix 0x10 / channel 2).
//   So the sniffer: (1) ACQUIRE -- listen on ibex/ch2, catch an E1, read session base/prefix/channel from it;
//   (2) CAPTURE -- retune to that session address/channel and receive every frame both ways. E1 keepalives on
//   the session address re-advertise the channel, so a QoS channel-hop is followed; a stall re-acquires.
//
// Frame in RAM after RX (PCNF0 S0=0/LF=8/S1=3): rfrx[0]=LENGTH(payload bytes), rfrx[1]=S1, rfrx[2..]=payload.
//   payload[0] (rfrx[2]) is the opcode: 0xE? = puck->controller, 0xF? = controller->puck.
//
// STREAM (over WebUSB bulk IN), little-endian:
//   packet:  C0 DE [N] [t_us:4] [ch] [flags] [rssi] [match] [N raw bytes]   flags bit0=CRC ok; match=RXMATCH pipe
//   status:  C1 DE [state][curCh][b0 b1 b2 b3][prefix][cap][rxLo rxHi][advCh][ab0 ab1 ab2 ab3][apfx][lastMatch]
//            (state 0=acquire 1=capture; adv* = session params last parsed out of an E1 host frame; 19 bytes)
// COMMANDS (WebUSB bulk OUT):
//   01 start  | 02 stop  | 03 re-acquire (back to ibex/ch2)  | 04 <ch> pin channel (keep current address)
//   05 <b0 b1 b2 b3 pfx ch> pin a full session: force this exact base/prefix/channel (use after reading a real
//      E1 by eye -- the surest way to camp on the session when auto-acquire's fixed E1 offsets are wrong)
//
// Build (Adafruit nRF52 core, same flags as OpenPuck):
//   arduino-cli compile -b adafruit:nrf52:feather52840 \
//     --build-property "build.extra_flags=-DNRF52840_XXAA {build.flags.usb}" puck_sniffer
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include "radio.h"

Adafruit_USBD_WebUSB usb_web;

// ---- capture ring (single producer + consumer, both loop context -> no locking) ----
struct Cap {
	uint32_t t;
	uint8_t ch, flags, rssi, match, n;
	uint8_t b[96];
};
#define RINGN 512
static Cap g_ring[RINGN];
static volatile uint16_t g_head = 0, g_tail = 0;
static inline uint16_t rnext(uint16_t i)
{
	return (uint16_t)((i + 1) % RINGN);
}

static void rxArmInit();
static void tuneDiscovery();
static void tuneSession();

static bool g_streaming = false; // app sent START -> emit packet frames
static uint8_t g_state = 0; // 0=ACQUIRE (ibex/ch2), 1=CAPTURE (session)
static uint8_t g_curCh = 2;
static uint8_t g_sBase[4] = { 0x69, 0x62, 0x65, 0x78 };
static uint8_t g_sPrefix = 0x10;
static uint8_t g_forceCh =
	0; // !=0 -> user pinned a channel (skip auto-acquire retune)
static bool g_pinSession =
	false; // user pinned a FULL session (cmd 05): never auto-retune addr/ch

// cmd 06: stay on ibex/ch2 and stream EVERY E1 beacon, never auto-lock
// (so you can see how many pucks/slots are advertising + their bases)
static bool g_survey = false;
// cmd 07: lock ONLY the E1 whose ibex_uuid (E1 payload bytes, = rfrx[7..10], LE) matches -- the puck beacons one
// E1 per bonded slot and we must pick the CONNECTED controller's. Read its uuid from pairtui (e.g. 0xEF7171B4).
static bool g_haveTarget = false;
static uint8_t g_targetIbex[4] = { 0, 0, 0, 0 };
static unsigned long g_lastRx = 0; // for stall -> sweep/re-acquire
static uint8_t g_lastMatch =
	0; // RXMATCH pipe of the most recent frame (diagnostic)

// What the last E1 host frame ADVERTISED (parsed by the assumed offsets). Reported in status EVEN IF we don't
// lock, so the panel can show whether those offsets are sane (channel in range, address non-degenerate) -- the
// fastest way to tell if auto-acquire is mis-reading the real puck's E1.
static uint8_t g_advCh = 0, g_advBase[4] = { 0, 0, 0, 0 }, g_advPfx = 0;

// CAPTURE-mode stall sweep: instead of jumping straight back to discovery (which abandons the session and
// strands us on ch2 seeing only E1/E2), step through clean candidate channels KEEPING the learned address --
// this follows a QoS channel hop. Only after a couple of fully-dry sweeps do we fall back to discovery.
static const uint8_t SWEEP_CH[] = { 18, 46, 76, 22, 68, 2, 80, 52, 55, 56, 57 };
static uint8_t g_sweepIdx = 0, g_sweepDry = 0;

// Gazell session-hunt: the puck's 5Hz E1/E2 keepalive trickles in on the primary channel even when the
// controller isn't being actively polled, so "no packets" never fires the sweep. The ACTIVE session (E3 poll +
// 0xF* controller reply) rides a ~3-channel set {primary,2,80} derived from the per-session address. So HUNT on
// "no 0xF* reply" instead: dwell briefly on each of {primary,2,80,+clean candidates} until the controller's
// replies appear, then PARK there. g_lastFrx = last controller (0xF*) frame; g_primaryCh = E1-advertised channel.
static const uint8_t HUNT_CH[] = { 2, 80, 18, 46, 76, 22, 68, 52 };
static uint8_t g_primaryCh = 78, g_huntIdx = 0;
static unsigned long g_lastFrx = 0, g_huntMs = 0;
#define HUNT_DWELL_MS 350u
#define HUNT_NOFRX_MS 1000u

static void tuneDiscovery()
{
	g_state = 0;
	g_curCh = 2;
	rfConfig(2);
	rfSetAddr(PAIR_BASE, 0x10);
	rxArmInit();
}
// Session RX listens across all 8 pipes on the learned base (rfSetAddrMulti) so a same-base/other-prefix
// controller reply is still caught; RXMATCH tells us which pipe each frame used.
static void tuneSession()
{
	g_state = 1;
	rfConfig(g_curCh);
	rfSetAddrMulti(g_sBase, g_sPrefix);
	rxArmInit();
}

// Continuous (gapless) RX: READY->START and END->START keep the radio receiving; ADDRESS->RSSISTART samples
// signal strength for each packet. We poll EVENTS_END and copy rfrx before the next packet can overwrite it.
static void rxArmInit()
{
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_START_Msk |
			    RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
			    RADIO_SHORTS_DISABLED_RSSISTOP_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->TASKS_RXEN = 1;
}

static void ringPush(uint32_t t, uint8_t ch, bool crcok, uint8_t rssi,
		     uint8_t match, const uint8_t *b, uint8_t n)
{
	uint16_t h = g_head, nx = rnext(h);
	if (nx == g_tail)
		g_tail = rnext(g_tail); // full -> drop oldest
	Cap &c = g_ring[h];
	c.t = t;
	c.ch = ch;
	c.flags = crcok ? 1 : 0;
	c.rssi = rssi;
	c.match = match;
	c.n = n;
	memcpy(c.b, b, n);
	g_head = nx;
}

// Drain the radio: copy every completed RX frame into the ring. Called very often from loop() so the tight
// poll->reply exchange is caught. RSSISAMPLE/CRCSTATUS belong to the frame whose END we just saw.
static void rxPump()
{
	for (int guard = 0; guard < 8 && NRF_RADIO->EVENTS_END; guard++) {
		NRF_RADIO->EVENTS_END = 0;
		bool crcok = NRF_RADIO->CRCSTATUS & 1;
		uint8_t rssi = (uint8_t)(NRF_RADIO->RSSISAMPLE & 0x7F);

		// which logical address (pipe) the frame hit
		uint8_t match = (uint8_t)(NRF_RADIO->RXMATCH & 0x7);
		g_lastMatch = match;
		uint8_t len = rfrx[0];
		uint16_t n = (uint16_t)(2 + len);
		if (n > 96)
			n = 96; // LENGTH + S1 + payload, clamped
		ringPush(micros(), g_curCh, crcok, rssi, match, rfrx,
			 (uint8_t)n);
		g_lastRx = millis();
		if (g_state == 1)
			// any frame on the session resets the dry-sweep count
			g_sweepDry = 0;
		if (crcok && len >= 1 && (rfrx[2] & 0xF0) == 0xF0)
			// controller reply = ACTIVE session here -> stop hunting, park
			g_lastFrx = millis();
		// Parse the session params advertised in an E1 host-frame (payload[0]=0xE1) by the ASSUMED offsets, and
		// always record them for the status report -- so the panel shows what auto-acquire would lock onto even
		// when we choose not to (lets us eyeball whether the real puck's offsets match ours).
		if (crcok && len >= 18 && rfrx[2] == 0xE1) {
			g_advCh = rfrx[11];
			g_advBase[0] = rfrx[15];
			g_advBase[1] = rfrx[16];
			g_advBase[2] = rfrx[17];
			g_advBase[3] = rfrx[18];
			g_advPfx = rfrx[19];
			if (g_pinSession || g_survey)
				// pinned / survey -> never auto-retune (just record adv)
				continue;
			// Target a specific bonded slot by its ibex_uuid (E1 bytes rfrx[7..10]): the puck beacons one E1 per slot,
			// so skip any E1 that isn't the CONNECTED controller's -- otherwise we lock an idle slot's keepalive.
			if (g_haveTarget &&
			    memcmp(rfrx + 7, g_targetIbex, 4) != 0)
				continue;
			// ACQUIRE: lock onto the advertised session. CAPTURE: a different advertised channel = QoS hop -> follow.
			if (g_advCh >= 1 && g_advCh <= 80) {
				if (g_state == 0) {
					memcpy(g_sBase, g_advBase, 4);
					g_sPrefix = g_advPfx;
					g_primaryCh = g_advCh;
					g_curCh = g_forceCh ? g_forceCh :
							      g_advCh;
					g_huntIdx = 0;
					g_lastFrx = millis();
					tuneSession();
					return;
				} else if (!g_forceCh && g_advCh != g_curCh) {
					g_curCh = g_advCh;
					tuneSession();
					return;
				}
			}
		}
	}
}

// ---- WebUSB stream out ----
static void emitPacket(const Cap &c)
{
	uint8_t f[11 + 96];
	f[0] = 0xC0;
	f[1] = 0xDE;
	f[2] = c.n;
	f[3] = (uint8_t)c.t;
	f[4] = (uint8_t)(c.t >> 8);
	f[5] = (uint8_t)(c.t >> 16);
	f[6] = (uint8_t)(c.t >> 24);
	f[7] = c.ch;
	f[8] = c.flags;
	f[9] = c.rssi;
	f[10] = c.match;
	memcpy(f + 11, c.b, c.n);
	usb_web.write(f, (uint32_t)(11 + c.n));
}
static void emitStatus()
{
	static unsigned long last = 0;
	if (millis() - last < 250)
		return;
	last = millis();
	static uint16_t rx = 0;
	rx++;
	uint8_t f[19] = { 0xC1,		0xDE,
			  g_state,	g_curCh,
			  g_sBase[0],	g_sBase[1],
			  g_sBase[2],	g_sBase[3],
			  g_sPrefix,	(uint8_t)(g_streaming ? 1 : 0),
			  (uint8_t)rx,	(uint8_t)(rx >> 8),
			  g_advCh,	g_advBase[0],
			  g_advBase[1], g_advBase[2],
			  g_advBase[3], g_advPfx,
			  g_lastMatch };
	usb_web.write(f, sizeof f);
	usb_web.flush();
}
static void drainToHost()
{
	if (!usb_web.connected())
		return;

	// bound per-call so RX stays responsive; the ring covers bursts
	uint16_t budget = 64;
	while (budget-- && g_tail != g_head) {
		if (g_streaming)
			emitPacket(g_ring[g_tail]);
		g_tail = rnext(g_tail);
	}
	usb_web.flush();
	emitStatus();
}
static void handleCmd()
{
	while (usb_web.available()) {
		int c = usb_web.read();
		if (c < 0)
			break;
		if (c == 0x01) {
			g_streaming = true;
			g_head = g_tail = 0;
		} // START (flush stale ring)
		else if (c == 0x02) {
			g_streaming = false;
		} // STOP
		else if (c == 0x03) {
			g_forceCh = 0;
			g_pinSession = false;
			g_survey = false;
			tuneDiscovery();
		} // re-acquire from ibex/ch2
		else if (c == 0x06) {
			g_survey = !g_survey;
			g_forceCh = 0;
			g_pinSession = false;
			tuneDiscovery();
		} // survey: list all E1 beacons on ch2, no lock
		// target ibex_uuid: lock only the matching slot's E1
		else if (c == 0x07) {
			int u0 = usb_web.read(), u1 = usb_web.read(),
			    u2 = usb_web.read(), u3 = usb_web.read();
			if (u0 >= 0 && u1 >= 0 && u2 >= 0 && u3 >= 0) {
				g_targetIbex[0] = u0;
				g_targetIbex[1] = u1;
				g_targetIbex[2] = u2;
				g_targetIbex[3] = u3;
				g_haveTarget = true;
				g_survey = false;
				g_forceCh = 0;
				g_pinSession = false;
				tuneDiscovery();
			}
		} else if (c == 0x04) {
			int ch = usb_web.read();
			if (ch >= 1 && ch <= 80) {
				g_forceCh = (uint8_t)ch;
				g_curCh = (uint8_t)ch;
				tuneSession();
			}
		} // pin channel, keep addr
		else if (c == 0x05) { // pin a FULL session: b0 b1 b2 b3 pfx ch
			int b0 = usb_web.read(), b1 = usb_web.read(),
			    b2 = usb_web.read(), b3 = usb_web.read(),
			    pf = usb_web.read(), ch = usb_web.read();
			if (b0 >= 0 && b1 >= 0 && b2 >= 0 && b3 >= 0 &&
			    pf >= 0 && ch >= 1 && ch <= 80) {
				g_sBase[0] = b0;
				g_sBase[1] = b1;
				g_sBase[2] = b2;
				g_sBase[3] = b3;
				g_sPrefix = (uint8_t)pf;
				g_curCh = (uint8_t)ch;
				g_forceCh = (uint8_t)ch;
				g_pinSession = true;
				tuneSession();
			}
		}
	}
}

void setup()
{
	// Standard Adafruit WebUSB setup (keep the default CDC composite; just add the WebUSB vendor interface). No
	// landing page -> no Chrome "open <url>" notification. No detach/clearConfiguration: that's OpenPuck-specific
	// endpoint juggling and can leave the WebUSB stream un-enabled.

	// 'SN' -- distinct PID so the host doesn't confuse it with a puck
	USBDevice.setID(0x28DE, 0x534E);
	USBDevice.setManufacturerDescriptor("Valve Software");
	USBDevice.setProductDescriptor("OpenPuck Sniffer");
	usb_web.begin();
	Serial.begin(115200);
	for (int i = 0; i < 200 && !USBDevice.mounted(); i++)
		delay(10);

	// harmless (we never TX); leaves the radio tunables at their validated defaults
	rfGenSessionAddr();
	tuneDiscovery();
	g_lastRx = millis();
}

void loop()
{
	rxPump();
	handleCmd();

	// pump again around the (fast, buffered) USB work to catch tight poll->reply pairs
	rxPump();
	drainToHost();
	// Session HUNT (Gazell). A pinned session (cmd 04/05) is never disturbed. Otherwise: while we're NOT seeing the
	// controller's 0xF* replies (the ACTIVE session), the keepalive trickles in on the primary but the real E3/F1
	// exchange is on another channel of the per-session set -> hunt {primary,2,80,+candidates}, dwelling briefly on
	// each, KEEPING the learned address. The instant a 0xF* arrives (g_lastFrx) we stop and park there. After two
	// full dry passes (no replies anywhere -> session address likely changed on a reconnect) fall back to discovery
	// to re-read an E1. Hunting only runs when there are no replies, so an active capture is never disturbed.
	if (g_state == 1 && !g_forceCh && !g_pinSession &&
	    millis() - g_lastFrx > HUNT_NOFRX_MS &&
	    millis() - g_huntMs > HUNT_DWELL_MS) {
		g_huntMs = millis();
		uint8_t ch =
			(g_huntIdx == 0) ?
				g_primaryCh :
				HUNT_CH[(g_huntIdx - 1) % (sizeof HUNT_CH)];
		g_huntIdx++;
		if (g_huntIdx > 1 + 2 * (uint8_t)sizeof HUNT_CH) {
			g_huntIdx = 0;
			tuneDiscovery();
			g_lastFrx = millis();
			g_huntMs = millis();
			return;
		}
		g_curCh = ch;
		rfConfig(g_curCh);
		rfSetAddrMulti(g_sBase, g_sPrefix);
		rxArmInit();
	}
}
