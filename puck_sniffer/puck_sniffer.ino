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
//   status:  C1 DE [state][curCh][b0 b1 b2 b3][prefix][cap][rxLo rxHi][advCh][ab0 ab1 ab2 ab3][apfx][lastMatch][dropLo dropHi][bond]
//            (state 0=acquire 1=capture; adv* = session params last parsed out of an E1 host frame; drop* =
//             frames lost to a full ring -- 0 = lossless; bond bit7=camped on a learned bond, bits4-6=#bonds
//             stored, bits0-3=#channels learned for the camped bond; 22 bytes)
// COMMANDS (WebUSB bulk OUT):
//   01 start | 02 stop | 03 re-acquire (back to ibex/ch2, un-camp) | 04 <ch> pin channel (keep current address)
//   05 <b0 b1 b2 b3 pfx ch> pin a full session: force this exact base/prefix/channel (manual override)
//   06 survey | 07 <u0 u1 u2 u3> lock ibex_uuid (auto-camps if its bond is already learned) | 08 forget all bonds
//
// AUTO-CAMP (no manual pinning): the bonded address is stable across reconnects (RE ground truth), so once we've
// seen a controller's E1 once we persist its session (base/prefix/channel) to flash keyed by ibex_uuid and learn
// which channels carry its replies. On a later boot / uuid-lock we camp straight back on it and sweep its learned
// channels, catching the controller's next clean reconnect with no ch2 catch and no manual pin.
//
// Build (Adafruit nRF52 core, same flags as OpenPuck):
//   arduino-cli compile -b adafruit:nrf52:feather52840 \
//     --build-property "build.extra_flags=-DNRF52840_XXAA {build.flags.usb}" puck_sniffer
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "radio.h"
using namespace Adafruit_LittleFS_Namespace;

Adafruit_USBD_WebUSB usb_web;

// Forward-declared so Arduino's auto-generated prototypes (inserted before the first function below) can name it;
// the full definition follows under "capture ring".
struct Cap;

// ---- learned-bond store (the "no manual pinning" mechanism) -------------------------------------------------
// RE ground truth (~/nrf-sniffer/protocol): a controller's bonded on-air address is DERIVED AT PAIRING and REUSED
// on every reconnect (reconnect = silent ESB resumption, no fresh address, no guaranteed ch2 re-beacon), and the
// session hops a small STABLE channel map. So once we have observed a bond's address + a working channel ONCE, we
// can camp straight back on it for all future reconnects -- no ch2 catch, no manual `pin session`. We persist what
// we learn to flash keyed by the bond's ibex_uuid, so it even survives unplugging the sniffer.
#define BONDFILE "/sniffbonds.bin"
#define BOND_MAGIC 0x53424E32u // "SBN2"
#define NBONDS 4
#define BOND_NCH \
	6 // learned channels per bond (the map is ~4; a couple of spares)
struct Bond {
	uint8_t uuid[4]; // ibex_uuid key; all-zero = empty slot
	uint8_t base[4]; // session ESB base
	uint8_t prefix; // session ESB prefix
	uint8_t primaryCh; // channel last advertised in the bond's E1
	uint8_t chans[BOND_NCH]; // channels we've actually seen carry 0xF* replies
	uint8_t nchans;
};
static Bond g_bonds[NBONDS];
static int g_curBond = -1; // index of the bond we're camped on (-1 = none)
static bool g_bondDirty =
	false; // a learn changed the table -> flush to flash in loop()
static unsigned long g_lastBondSave = 0;

static bool uuidSet(const uint8_t u[4])
{
	return (u[0] | u[1] | u[2] | u[3]) != 0;
}
static int bondFind(const uint8_t uuid[4])
{
	for (int i = 0; i < NBONDS; i++)
		if (uuidSet(g_bonds[i].uuid) &&
		    memcmp(g_bonds[i].uuid, uuid, 4) == 0)
			return i;
	return -1;
}
static int bondFirst()
{
	for (int i = 0; i < NBONDS; i++)
		if (uuidSet(g_bonds[i].uuid))
			return i;
	return -1;
}
// Find or create the slot for a uuid (evicts slot 0 if the small table is full -- fine for a debug tool).
static int bondAlloc(const uint8_t uuid[4])
{
	int i = bondFind(uuid);
	if (i >= 0)
		return i;
	for (int k = 0; k < NBONDS; k++)
		if (!uuidSet(g_bonds[k].uuid)) {
			memset(&g_bonds[k], 0, sizeof(Bond));
			memcpy(g_bonds[k].uuid, uuid, 4);
			return k;
		}
	memset(&g_bonds[0], 0, sizeof(Bond));
	memcpy(g_bonds[0].uuid, uuid, 4);
	return 0;
}
// Record that channel `ch` carried a real (CRC-ok) controller reply for this bond. One known-good channel is
// already enough to re-catch a reconnect (the controller hops the whole map, so it WILL transmit on it); learning
// the few that appear just makes the re-acquire sweep tighter.
static void bondLearnChan(int idx, uint8_t ch)
{
	if (idx < 0 || ch < 1 || ch > 80)
		return;
	Bond &b = g_bonds[idx];
	for (int i = 0; i < b.nchans; i++)
		if (b.chans[i] == ch)
			return;
	if (b.nchans < BOND_NCH) {
		b.chans[b.nchans++] = ch;
		g_bondDirty = true;
	}
}
static void bondsLoad()
{
	memset(g_bonds, 0, sizeof g_bonds);
	File f(InternalFS);
	if (f.open(BONDFILE, FILE_O_READ)) {
		uint32_t magic = 0;
		if (f.read((uint8_t *)&magic, 4) == 4 && magic == BOND_MAGIC)
			f.read((uint8_t *)g_bonds, sizeof g_bonds);
		f.close();
	}
}
// Flash write blocks for tens of ms -> ONLY ever call from loop() (never the RADIO ISR). The ring absorbs the
// frames captured during the write.
static void bondsSave()
{
	InternalFS.remove(BONDFILE);
	File f(InternalFS);
	if (f.open(BONDFILE, FILE_O_WRITE)) {
		uint32_t magic = BOND_MAGIC;
		f.write((uint8_t *)&magic, 4);
		f.write((uint8_t *)g_bonds, sizeof g_bonds);
		f.close();
	}
	g_bondDirty = false;
	g_lastBondSave = millis();
}

// ---- capture ring (single producer + consumer, both loop context -> no locking) ----
struct Cap {
	uint32_t t;
	uint8_t ch, flags, rssi, match, n;
	uint8_t b[96];
};
#define RINGN \
	1024 // ~2 s of headroom at 500 fps so a burst rides out a brief host hiccup without dropping
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
// g_state/g_curCh are written by loop() (retune) and READ by the RADIO END ISR (which stamps each captured frame
// with the current channel/state) -> volatile so the ISR never sees a cached value.
static volatile uint8_t g_state =
	0; // 0=ACQUIRE (ibex/ch2), 1=CAPTURE (session)
static volatile uint8_t g_curCh = 2;
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
static volatile unsigned long g_lastRx = 0; // for stall -> sweep/re-acquire
static volatile uint8_t g_lastMatch =
	0; // RXMATCH pipe of the most recent frame (diagnostic)
// Frames lost to a full ring (producer = RADIO ISR outran the loop's USB drain). Incremented in the ISR, surfaced
// in the status frame so capture completeness is MEASURED rather than silently assumed. Lossless = this stays 0.
static volatile uint32_t g_drops = 0;

// What the last E1 host frame ADVERTISED (parsed by the assumed offsets). Reported in status EVEN IF we don't
// lock, so the panel can show whether those offsets are sane (channel in range, address non-degenerate) -- the
// fastest way to tell if auto-acquire is mis-reading the real puck's E1.
static uint8_t g_advCh = 0, g_advBase[4] = { 0, 0, 0, 0 }, g_advPfx = 0;

// CAPTURE-mode stall sweep: instead of jumping straight back to discovery (which abandons the session and
// strands us on ch2 seeing only E1/E2), step through clean candidate channels KEEPING the learned address --
// this follows a QoS channel hop. Only after a couple of fully-dry sweeps do we fall back to discovery.
static const uint8_t SWEEP_CH[] = { 18, 46, 76, 22, 68, 2, 80, 52, 55, 56, 57 };
static uint8_t g_sweepIdx = 0;
static volatile uint8_t g_sweepDry =
	0; // reset in the RADIO ISR on any session frame

// Gazell session-hunt: the puck's 5Hz E1/E2 keepalive trickles in on the primary channel even when the
// controller isn't being actively polled, so "no packets" never fires the sweep. The ACTIVE session (E3 poll +
// 0xF* controller reply) rides a ~3-channel set {primary,2,80} derived from the per-session address. So HUNT on
// "no 0xF* reply" instead: dwell briefly on each of {primary,2,80,+clean candidates} until the controller's
// replies appear, then PARK there. g_lastFrx = last controller (0xF*) frame; g_primaryCh = E1-advertised channel.
static const uint8_t HUNT_CH[] = { 2, 80, 18, 46, 76, 22, 68, 52 };
static uint8_t g_primaryCh = 78, g_huntIdx = 0;
// g_lastFrx is set in the RADIO ISR (on a 0xF* reply) and read by the hunt logic in loop() -> volatile.
static volatile unsigned long g_lastFrx = 0;
static unsigned long g_huntMs = 0;
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
// Camp directly on a learned bond (no ch2 discovery): adopt its stored address and start listening on its
// best-known channel. Because the bonded address is stable, this is all that's needed to catch the controller's
// next clean reconnect -- the hunt sweep below walks the bond's learned channels until its 0xF* replies appear.
static void campBond(int idx)
{
	if (idx < 0)
		return;
	Bond &b = g_bonds[idx];
	g_curBond = idx;
	memcpy(g_sBase, b.base, 4);
	g_sPrefix = b.prefix;
	if (b.primaryCh >= 1 && b.primaryCh <= 80)
		g_primaryCh = b.primaryCh;
	// prefer a channel we've actually seen replies on, else the advertised primary
	g_curCh = (b.nchans && b.chans[0] >= 1 && b.chans[0] <= 80) ?
			  b.chans[0] :
			  g_primaryCh;
	g_huntIdx = 0;
	g_lastFrx = millis();
	tuneSession();
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
	// Capture the completed frame from the RADIO END interrupt (see RADIO_IRQHandler). The NVIC enable/priority is
	// set once in setup(); here we (re)arm the END interrupt source after every retune.
	NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
	NRF_RADIO->TASKS_RXEN = 1;
}

// Producer = the RADIO END ISR (owns g_head); consumer = loop() (owns g_tail). Lock-free single-producer/
// single-consumer ring: on full we drop the NEWEST frame and COUNT it in g_drops -- the ISR must never touch
// g_tail. (The old code dropped the oldest by advancing g_tail from the producer, which raced the consumer and
// also hid the loss.) g_drops is surfaced in the status frame so a dropped frame is measured, not assumed away.
static void ringPush(uint32_t t, uint8_t ch, bool crcok, uint8_t rssi,
		     uint8_t match, const uint8_t *b, uint8_t n)
{
	uint16_t h = g_head, nx = rnext(h);
	if (nx == g_tail) {
		g_drops++; // ring full -> drop newest, count it
		return;
	}
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

// RADIO END interrupt -- the ONE latency-critical step, and the reason capture is now lossless.
//
// The radio re-arms into the SAME rfrx buffer (END_START short), so the NEXT packet's payload starts overwriting
// rfrx only ~60us after this END (ramp-up + preamble + address). Snapshotting the just-completed frame HERE -- in
// a high-priority ISR that PREEMPTS the USB writes/flushes running in loop() -- guarantees the copy lands well
// inside that 60us window. The previous design polled EVENTS_END from loop(), where a blocking USB flush could
// delay the copy past 60us and let the next frame overwrite rfrx (a tight poll->reply pair, <100us apart, was the
// danger) -- a silent loss. RSSISAMPLE/CRCSTATUS/RXMATCH belong to the frame whose END we just saw.
//
// The ISR ONLY captures. Interpretation (E1 parse / acquire-lock / QoS hop) is left to processFrame() in loop(),
// so this handler stays a few microseconds long and can't itself become the stall it's meant to prevent.
extern "C" void RADIO_IRQHandler(void)
{
	while (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		bool crcok = NRF_RADIO->CRCSTATUS & 1;
		uint8_t rssi = (uint8_t)(NRF_RADIO->RSSISAMPLE & 0x7F);
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
	}
}

// Interpret ONE captured frame (loop context, fed from the ring). Parses the session params advertised in an E1
// host-frame (payload[0]=0xE1) by the ASSUMED offsets and always records them for the status report -- so the
// panel shows what auto-acquire would lock onto even when we choose not to (lets us eyeball whether the real
// puck's offsets match ours). Returns true if it retuned the radio. This is the exact logic that used to run
// inline in rxPump; it just no longer shares a thread with the latency-sensitive copy.
static bool processFrame(const Cap &c)
{
	bool crcok = c.flags & 1;
	uint8_t len = c.n >= 1 ? c.b[0] : 0;
	uint8_t op = c.n >= 3 ? c.b[2] : 0;
	// LEARN: a CRC-ok controller reply (0xF*) on the session we're camped on proves THIS channel carries the
	// bond's traffic -> remember it, so the next reconnect can re-acquire straight onto it (no ch2 catch).
	if (crcok && g_state == 1 && g_curBond >= 0 && (op & 0xF0) == 0xF0)
		bondLearnChan(g_curBond, c.ch);
	if (!(crcok && len >= 18 && op == 0xE1))
		return false;
	g_advCh = c.b[11];
	g_advBase[0] = c.b[15];
	g_advBase[1] = c.b[16];
	g_advBase[2] = c.b[17];
	g_advBase[3] = c.b[18];
	g_advPfx = c.b[19];
	if (g_pinSession || g_survey)
		// pinned / survey -> never auto-retune (just record adv)
		return false;
	// Target a specific bonded slot by its ibex_uuid (E1 bytes [7..10]): the puck beacons one E1 per slot, so skip
	// any E1 that isn't the CONNECTED controller's -- otherwise we lock an idle slot's keepalive.
	if (g_haveTarget && memcmp(c.b + 7, g_targetIbex, 4) != 0)
		return false;
	// ACQUIRE: lock onto the advertised session. CAPTURE: a different advertised channel = QoS hop -> follow.
	if (g_advCh >= 1 && g_advCh <= 80) {
		// LEARN the bonded address from this E1 (keyed by ibex_uuid) and persist it -- this is what lets future
		// reconnects be caught with no manual pinning.
		int bi = bondAlloc(c.b + 7);
		Bond &b = g_bonds[bi];
		if (memcmp(b.base, g_advBase, 4) != 0 || b.prefix != g_advPfx ||
		    b.primaryCh != g_advCh) {
			memcpy(b.base, g_advBase, 4);
			b.prefix = g_advPfx;
			b.primaryCh = g_advCh;
			g_bondDirty = true;
		}
		if (g_state == 0) {
			g_curBond = bi;
			memcpy(g_sBase, g_advBase, 4);
			g_sPrefix = g_advPfx;
			g_primaryCh = g_advCh;
			g_curCh = g_forceCh ? g_forceCh : g_advCh;
			g_huntIdx = 0;
			g_lastFrx = millis();
			tuneSession();
			return true;
		} else if (!g_forceCh && g_advCh != g_curCh) {
			g_curCh = g_advCh;
			tuneSession();
			return true;
		}
	}
	return false;
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
	uint32_t drops = g_drops; // snapshot the volatile once
	// bond byte: bit7 = camped on a learned bond; bits0-3 = #channels learned for it; bits4-6 = #bonds stored
	int nbonds = 0;
	for (int i = 0; i < NBONDS; i++)
		if (uuidSet(g_bonds[i].uuid))
			nbonds++;
	uint8_t curN = (g_curBond >= 0) ? g_bonds[g_curBond].nchans : 0;
	uint8_t bondInfo = (uint8_t)((g_curBond >= 0 ? 0x80 : 0) |
				     ((nbonds & 0x7) << 4) | (curN & 0xF));
	uint8_t f[22] = { 0xC1,
			  0xDE,
			  g_state,
			  g_curCh,
			  g_sBase[0],
			  g_sBase[1],
			  g_sBase[2],
			  g_sBase[3],
			  g_sPrefix,
			  (uint8_t)(g_streaming ? 1 : 0),
			  (uint8_t)rx,
			  (uint8_t)(rx >> 8),
			  g_advCh,
			  g_advBase[0],
			  g_advBase[1],
			  g_advBase[2],
			  g_advBase[3],
			  g_advPfx,
			  g_lastMatch,
			  (uint8_t)drops,
			  (uint8_t)(drops >> 8),
			  bondInfo };
	usb_web.write(f, sizeof f);
	usb_web.flush();
}
// Consume the ring (loop context): interpret each captured frame (E1 parse / acquire-lock / QoS hop) and, when
// streaming, emit it to the host. processFrame() runs even when NOT streaming, because acquisition has to happen
// before Start Cap. Capture itself is the RADIO ISR's job now -- this side just drains, so a slow USB flush here
// can no longer cost us a frame (worst case the ring backs up and g_drops ticks, which the panel shows).
static void consumeRing()
{
	bool connected = usb_web.connected();
	bool emit = connected && g_streaming;
	// bound per-call so the loop stays responsive; the 512-deep ring absorbs anything beyond this
	uint16_t budget = 128;
	while (budget-- && g_tail != g_head) {
		Cap c = g_ring
			[g_tail]; // copy out before releasing the slot to the producer
		g_tail = rnext(g_tail);
		bool retuned = processFrame(
			c); // may retune the radio (acquire / channel hop)
		if (emit)
			emitPacket(c);
		// On a retune, stop draining this round: the remaining queued frames were captured on the OLD channel, and
		// a flooded ch2 often queues several E1s -- re-applying each would thrash the radio. (The old rxPump did the
		// same via an immediate `return`.) The rest drain next loop.
		if (retuned)
			break;
	}
	if (connected) {
		usb_web.flush();
		emitStatus();
	}
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
			g_curBond = -1; // un-camp; relearn from a fresh E1
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
				// If we've already learned this controller's bond, camp straight on it (no ch2 catch needed -- the
				// bonded address is stable, so its next reconnect lands right here). Otherwise discover it on ch2,
				// and processFrame() will persist the bond the first time its E1 is seen.
				int bi = bondFind(g_targetIbex);
				if (bi >= 0)
					campBond(bi);
				else
					tuneDiscovery();
			}
		} else if (c == 0x08) {
			// forget all learned bonds (start clean)
			memset(g_bonds, 0, sizeof g_bonds);
			g_curBond = -1;
			bondsSave();
			tuneDiscovery();
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

	// Capture is interrupt-driven: the RADIO END ISR snapshots each frame the instant it completes, preempting the
	// USB work in loop() so the copy always beats the ~60us buffer-overwrite window. Give it a high priority and
	// enable it once here; rxArmInit() arms the END source (INTENSET) after every (re)tune. (No SoftDevice runs, so
	// RADIO_IRQn is ours.)
	NVIC_SetPriority(RADIO_IRQn, 2);
	NVIC_ClearPendingIRQ(RADIO_IRQn);
	NVIC_EnableIRQ(RADIO_IRQn);

	// harmless (we never TX); leaves the radio tunables at their validated defaults
	rfGenSessionAddr();

	// Load any bonds learned on a previous run. If we already know a controller's bonded session, camp straight on
	// it -- so a clean reconnect is captured the moment the controller powers back on, with NO ch2 catch and NO
	// manual pin. (With multiple bonds we wait for the host to `lock ibex` the one you want; a single bond is
	// unambiguous, so we adopt it immediately.) Otherwise fall back to ch2 discovery to learn one.
	InternalFS.begin();
	bondsLoad();
	int count = 0, b0 = bondFirst();
	for (int i = 0; i < NBONDS; i++)
		if (uuidSet(g_bonds[i].uuid))
			count++;
	if (count == 1)
		campBond(
			b0); // sole known bond -> adopt immediately (unambiguous)
	else
		tuneDiscovery(); // 0 bonds, or several -> discover / wait for a uuid lock
	g_lastRx = millis();
}

void loop()
{
	// Capture now happens in RADIO_IRQHandler (interrupt-driven, gapless), so loop() no longer races to poll the
	// radio. It just services host commands and drains the capture ring to USB.
	handleCmd();
	consumeRing();
	// Persist newly-learned bond data, debounced (flash write blocks tens of ms; the ring covers it).
	if (g_bondDirty && millis() - g_lastBondSave > 3000)
		bondsSave();
	// Session HUNT (Gazell). A pinned session (cmd 04/05) is never disturbed. Otherwise, while we're NOT seeing the
	// controller's 0xF* replies, sweep the session's channels, KEEPING the (stable, bonded) address, dwelling briefly
	// on each. The instant a 0xF* arrives (g_lastFrx) we stop and park. The sweep walks the bond's LEARNED channels
	// first (fast re-acquire onto a known-good one), then the advertised primary, then a broad candidate list to
	// discover the rest of the map. When a bond is known the address is stable, so we never abandon to ch2 -- we just
	// keep sweeping until the controller's reconnect appears; only with NO bond at all do we fall back to discovery
	// to re-read an E1. Hunting only runs when there are no replies, so an active capture is never disturbed.
	if (g_state == 1 && !g_forceCh && !g_pinSession &&
	    millis() - g_lastFrx > HUNT_NOFRX_MS &&
	    millis() - g_huntMs > HUNT_DWELL_MS) {
		g_huntMs = millis();
		uint8_t learned = (g_curBond >= 0) ? g_bonds[g_curBond].nchans :
						     0;
		uint8_t total = learned + 1 + (uint8_t)sizeof HUNT_CH;
		uint8_t slot = g_huntIdx;
		uint8_t ch;
		if (slot < learned)
			ch = g_bonds[g_curBond].chans[slot];
		else if (slot == learned)
			ch = g_primaryCh;
		else
			ch = HUNT_CH[(slot - learned - 1) % (sizeof HUNT_CH)];
		g_huntIdx++;
		if (g_huntIdx >= total) {
			g_huntIdx = 0;
			if (g_curBond < 0) {
				// nothing learned -> re-read an E1 on the ch2 rendezvous
				tuneDiscovery();
				g_lastFrx = millis();
				g_huntMs = millis();
				return;
			}
			// bonded address is stable -> keep sweeping its set, never abandon to ch2
		}
		g_curCh = ch;
		rfConfig(g_curCh);
		rfSetAddrMulti(g_sBase, g_sPrefix);
		rxArmInit();
	}
}
