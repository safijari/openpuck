#include "ble_host.h"
#include "config.h" // g_bleEn
#include "bonds.h" // NSLOT, g_slot, g_connReplyMs
#include "triton.h" // g_in, puckSynth45
#include "input_driver.h"
#include "controllers.h" // g_active
#include "rf_link.h" // g_battery / g_batteryState
#include "rf_timeslot.h"
#include "status_led.h" // ledWakePulse
#include <bluefruit.h>
#include <utility/bonding.h> // bond_load_keys / bond_remove_key (the stack's LTK store)
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <string.h>
using namespace Adafruit_LittleFS_Namespace;

// The clone boards this firmware targets (Pro Micro / SuperMini nRF52840) have no 32.768 kHz crystal, so the
// SoftDevice must run its low-frequency clock on the internal RC oscillator or Bluefruit.begin() fails at
// enable (the BLE_ST_FAILED the panel shows). The clock source is baked in at compile time from the variant's
// USE_LFXO/USE_LFRC define; `make build` selects variant_lfrc/ (RC) via build.variant.path. If that override
// is ever dropped, the stock feather52840 variant (USE_LFXO) would compile fine but silently break BLE at
// runtime -- so fail the build LOUDLY here instead. (Building for a genuine crystal board? Set
// LF_VARIANT_PATH= empty in the Makefile AND delete this guard.)
#if defined(USE_LFXO)
#error "BLE build selected the LFXO (crystal) clock, but the target clone boards have no 32kHz crystal -> Bluefruit.begin() fails. Build with `make build` (selects variant_lfrc/ = RC clock via build.variant.path)."
#endif
#if !defined(USE_LFRC)
#error "No LF clock source defined -- the variant.h was not seen. Build with `make build`."
#endif

// ---- sizing ----
#define BLE_LINKS \
	2 // concurrent BLE controllers (SoftDevice central link count)
#define BLE_NREP \
	6 // HID report characteristics discovered per device (in+out+feature)
#define BLE_NBOND 4 // persisted BLE controller records (panel list)
#define BLE_NSCAN 6 // pairing-UI scan table
#define RAWQ 4 // per-link raw input-report ring (BLE task -> loop)
#define RAW_MAXP 24 // largest raw HID input report we care about (xinput = 16)

#define BLEBOND_FILE "/blebonds.bin"
#define BLEBOND_MAGIC 0xB2

// Boot-attempt latch (anti-bootloop). bleHostBegin() writes this flag to flash BEFORE it enables the
// SoftDevice, and clears it only once BLE has run stably for BLE_STABLE_MS. If a boot enables BLE and then
// crashes (HardFault) or hangs (watchdog) during SoftDevice/central bringup OR in the first few seconds of
// running, the flag is left set -- so the NEXT boot finds it set, DISABLES BLE (persisted) and skips the
// crashing init instead of re-running it. That converts a repeating BLE-bringup crash from a bootloop into a
// single failed boot that comes up with BLE off; the user re-enables it from the panel once the cause is fixed.
#define BLEARM_FILE "/blearm.bin"
// How long BLE must run without a reset before we consider the bringup proven and clear the latch. Comfortably
// longer than the SoftDevice-enable + first-scan/first-connect window, and short enough that a normal session
// clears it quickly. The 3 s USB-mount wait in setup() runs BEFORE bleHostBegin, so this is measured from when
// BLE actually came up, not from boot.
#define BLE_STABLE_MS 6000u

// Scan/reconnect duty cycles (ms units for setIntervalMS). The scanner only runs at all while a bonded pad is
// offline (or the panel scan is armed) -- with everything connected it is stopped and the ESB link keeps the
// whole air. Idle reconnect-watch stays light (5%) because each scan WINDOW is a solid radio blackout for the
// SC2 poll; the UI scan burns half the air but only while the panel asks for it (auto-stops after 60 s).
#define SCAN_IDLE_INT 600
#define SCAN_IDLE_WIN 30
#define SCAN_UI_INT 160
#define SCAN_UI_WIN 80
#define UI_SCAN_MS 60000u
#define CONNECT_TIMEOUT_MS 10000u

struct BleBondRec {
	uint8_t used, addrType;
	uint8_t addr[6]; // identity address (post-bonding)
	uint16_t vid, pid;
	char name[13];
	uint8_t rsvd[3];
};
struct ScanEnt {
	uint8_t used, addrType;
	uint8_t addr[6];
	int8_t rssi;
	uint8_t isHid;
	char name[13];
	unsigned long ms;
};
struct RawEnt {
	uint8_t rid, len;
	uint8_t d[RAW_MAXP];
};

struct BleLink {
	volatile bool inUse; // allocated to a live connection
	volatile bool up; // secured + subscribed + slot claimed (input flowing)
	volatile uint16_t conn;
	volatile int8_t slot; // claimed bond slot, -1
	IInputDriver *drv;
	uint16_t vid, pid;
	char name[13];
	// GATT client objects (registered once at begin; (re)discovered per connection)
	BLEClientService dis, bat, hid;
	BLEClientCharacteristic pnp, batc;
	BLEClientCharacteristic rep[BLE_NREP];
	int8_t repIn[BLE_NREP]; // subscribe order of rep[i] (-1 = not an input report)
	int8_t outIdx; // rep[] index of the output (rumble) report, -1 = none
	// raw input ring: producer = BLE task (notify cb), consumer = loop (bleHostTask); PRIMASK both sides
	RawEnt q[RAWQ];
	volatile uint8_t qh, qt;
	uint8_t seq; // synth45 sequence byte
	// rumble latch: producers = usbd/loop (hapticSteamRumble), consumer = loop flush
	volatile uint16_t rumLo, rumHi;
	volatile bool rumPend;
	unsigned long rumMs;
	volatile uint8_t battery;

	BleLink()
		: inUse(false)
		, up(false)
		, conn(BLE_CONN_HANDLE_INVALID)
		, slot(-1)
		, drv(nullptr)
		, vid(0)
		, pid(0)
		, dis(BLEUuid((uint16_t)0x180A))
		, bat(BLEUuid((uint16_t)0x180F))
		, hid(BLEUuid((uint16_t)0x1812))
		, pnp(BLEUuid((uint16_t)0x2A50))
		, batc(BLEUuid((uint16_t)0x2A19))
		, rep{ BLEUuid((uint16_t)0x2A4D), BLEUuid((uint16_t)0x2A4D),
		       BLEUuid((uint16_t)0x2A4D), BLEUuid((uint16_t)0x2A4D),
		       BLEUuid((uint16_t)0x2A4D), BLEUuid((uint16_t)0x2A4D) }
		, outIdx(-1)
		, qh(0)
		, qt(0)
		, seq(0)
		, rumLo(0)
		, rumHi(0)
		, rumPend(false)
		, rumMs(0)
		, battery(0)
	{
		name[0] = 0;
		for (int i = 0; i < BLE_NREP; i++)
			repIn[i] = -1;
	}
};

static BleLink s_link[BLE_LINKS];
static BleBondRec s_bond[BLE_NBOND];
// RAM cache of the stack's bond keys (IRK for RPA resolution in the scan callback -- file I/O per adv report
// would stall the callback task). Refreshed at begin + whenever a bond is added/removed.
static bond_keys_t s_keys[BLE_NBOND];
static bool s_keysOk[BLE_NBOND];
static ScanEnt s_scan[BLE_NSCAN];

static volatile uint8_t s_state = BLE_ST_OFF;
static volatile bool s_uiScan = false;
static unsigned long s_uiScanUntil = 0;
static volatile bool s_scanOn = false;
static volatile bool s_connecting = false;
static unsigned long s_connectingMs = 0;
// anti-bootloop latch bookkeeping: when we armed the flash flag this session + when BLE came up (for the
// stable-clear timer). s_bootLatched stays true until the stable-clear fires (once).
static bool s_bootLatched = false;
static unsigned long s_bootUpMs = 0;
// Reconnect backoff: a bonded pad whose setup keeps failing (e.g. no free slot -- all four RF-bonded) must not
// connect/disconnect in a tight loop; skip auto-reconnects to it for a while after a failure.
#define RECONNECT_BACKOFF_MS 30000u
static volatile int8_t s_failBond = -1;
static unsigned long s_failBondMs = 0;

// ---- persistence ----
static void bleBondsLoad()
{
	memset(s_bond, 0, sizeof s_bond);
	File f(InternalFS);
	if (f.open(BLEBOND_FILE, FILE_O_READ)) {
		uint8_t m = 0;
		if (f.read(&m, 1) == 1 && m == BLEBOND_MAGIC)
			f.read((uint8_t *)s_bond, sizeof s_bond);
		else
			memset(s_bond, 0, sizeof s_bond);
		f.close();
	}
}
static void bleBondsSave()
{
	InternalFS.remove(BLEBOND_FILE);
	File f(InternalFS);
	if (f.open(BLEBOND_FILE, FILE_O_WRITE)) {
		uint8_t m = BLEBOND_MAGIC;
		f.write(&m, 1);
		f.write((uint8_t *)s_bond, sizeof s_bond);
		f.close();
	}
}
static void keysCacheLoad()
{
	for (int i = 0; i < BLE_NBOND; i++) {
		s_keysOk[i] = false;
		if (!s_bond[i].used)
			continue;
		ble_gap_addr_t a;
		memset(&a, 0, sizeof a);
		a.addr_type = s_bond[i].addrType;
		memcpy(a.addr, s_bond[i].addr, 6);
		s_keysOk[i] =
			bond_load_keys(BLE_GAP_ROLE_CENTRAL, &a, &s_keys[i]);
	}
}

// ---- anti-bootloop boot-attempt latch (see BLEARM_FILE) ----
static bool bleBootPending()
{
	File f(InternalFS);
	if (!f.open(BLEARM_FILE, FILE_O_READ))
		return false;
	uint8_t v = 0;
	f.read(&v, 1);
	f.close();
	return v == 1;
}
static void
bleBootArm() // called before the SoftDevice is enabled -> NVMC flash path
{
	InternalFS.remove(BLEARM_FILE);
	File f(InternalFS);
	if (f.open(BLEARM_FILE, FILE_O_WRITE)) {
		uint8_t v = 1;
		f.write(&v, 1);
		f.close();
	}
}
static void
bleBootDisarm() // may run with the SoftDevice enabled -> LittleFS routes via sd_flash_*
{
	InternalFS.remove(BLEARM_FILE);
}

// ---- small lookups ----
static BleLink *linkByConn(uint16_t ch)
{
	for (int i = 0; i < BLE_LINKS; i++)
		if (s_link[i].inUse && s_link[i].conn == ch)
			return &s_link[i];
	return nullptr;
}
static BleLink *linkByChr(BLEClientCharacteristic *chr, int *repIdx)
{
	for (int i = 0; i < BLE_LINKS; i++) {
		if (!s_link[i].inUse)
			continue;
		if (chr == &s_link[i].batc) {
			*repIdx = -1;
			return &s_link[i];
		}
		for (int r = 0; r < BLE_NREP; r++)
			if (chr == &s_link[i].rep[r]) {
				*repIdx = r;
				return &s_link[i];
			}
	}
	return nullptr;
}
static int bondFind(uint8_t addrType, const uint8_t addr[6])
{
	for (int i = 0; i < BLE_NBOND; i++)
		if (s_bond[i].used && s_bond[i].addrType == addrType &&
		    memcmp(s_bond[i].addr, addr, 6) == 0)
			return i;
	return -1;
}
static bool bondConnected(int b)
{
	for (int i = 0; i < BLE_LINKS; i++)
		if (s_link[i].inUse && s_link[i].up) {
			BLEConnection *c = Bluefruit.Connection(s_link[i].conn);
			if (!c)
				continue;
			ble_gap_addr_t a = c->getPeerAddr();
			if (a.addr_type == s_bond[b].addrType &&
			    memcmp(a.addr, s_bond[b].addr, 6) == 0)
				return true;
		}
	return false;
}
static bool linkAvail()
{
	for (int i = 0; i < BLE_LINKS; i++)
		if (!s_link[i].inUse)
			return true;
	return false;
}

// Claim a free bond slot for a BLE controller, HIGHEST index first: Steam's RF pairing fills slots from
// interface 0 up, so top-down claiming keeps the two from colliding until all four are taken.
static int slotClaim()
{
	for (int s = NSLOT - 1; s >= 0; s--)
		if (!g_slot[s].used && !slotIsBle(s))
			return s;
	return -1;
}
static void slotRelease(int s)
{
	if (s < 0 || s >= NSLOT)
		return;
	g_slotSrc[s] = SRC_RF;
	g_connReplyMs[s] = 0;
	memset((void *)&g_in[s], 0, sizeof g_in[s]);
}

// ---- scanning ----
static void scanApplyParams()
{
	Bluefruit.Scanner.setIntervalMS(s_uiScan ? SCAN_UI_INT : SCAN_IDLE_INT,
					s_uiScan ? SCAN_UI_WIN : SCAN_IDLE_WIN);
}
static bool scanWanted()
{
	if (s_state != BLE_ST_ON || s_connecting)
		return false;
	if (s_uiScan)
		return true;
	for (int i = 0; i < BLE_NBOND; i++)
		if (s_bond[i].used && !bondConnected(i) && linkAvail())
			return true;
	return false;
}
static void scanEnsure()
{
	bool want = scanWanted();
	if (want && !s_scanOn) {
		scanApplyParams();
		if (Bluefruit.Scanner.start(0))
			s_scanOn = true;
	} else if (!want && s_scanOn) {
		Bluefruit.Scanner.stop();
		s_scanOn = false;
	}
}

// Match an adv report against the persisted bonds: exact identity-address match, or an RPA resolved with the
// cached IRK (Xbox pads advertise resolvable private addresses after bonding).
static int bondMatchReport(const ble_gap_evt_adv_report_t *rpt)
{
	int direct = bondFind(rpt->peer_addr.addr_type, rpt->peer_addr.addr);
	if (direct >= 0)
		return direct;
	if (rpt->peer_addr.addr_type ==
	    BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE) {
		for (int i = 0; i < BLE_NBOND; i++)
			if (s_bond[i].used && s_keysOk[i] &&
			    Bluefruit.Security.resolveAddress(
				    &rpt->peer_addr,
				    &s_keys[i].peer_id.id_info))
				return i;
	}
	return -1;
}

static void scanAddEntry(ble_gap_evt_adv_report_t *rpt)
{
	char nm[13] = { 0 };
	if (!Bluefruit.Scanner.parseReportByType(
		    rpt, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, (uint8_t *)nm,
		    sizeof nm - 1))
		Bluefruit.Scanner.parseReportByType(
			rpt, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, (uint8_t *)nm,
			sizeof nm - 1);
	bool hid = Bluefruit.Scanner.checkReportForUuid(
		rpt, BLEUuid((uint16_t)0x1812));
	uint8_t ap[2];
	bool gamepad = false;
	if (Bluefruit.Scanner.parseReportByType(rpt, BLE_GAP_AD_TYPE_APPEARANCE,
						ap, 2) == 2) {
		uint16_t a = (uint16_t)(ap[0] | (ap[1] << 8));
		gamepad = (a >> 6) == 0x00F; // HID category (0x03C0..0x03FF)
	}
	if (!hid && !gamepad)
		return; // not a HID device -> keep the list gamepad-only
	// dedupe by address; else take a free row; else evict the stalest
	int idx = -1, oldest = 0;
	for (int i = 0; i < BLE_NSCAN; i++) {
		if (s_scan[i].used &&
		    s_scan[i].addrType == rpt->peer_addr.addr_type &&
		    memcmp(s_scan[i].addr, rpt->peer_addr.addr, 6) == 0) {
			idx = i;
			break;
		}
		if (!s_scan[i].used) {
			if (idx < 0 || s_scan[idx].used)
				idx = i;
		} else if (s_scan[i].ms < s_scan[oldest].ms)
			oldest = i;
	}
	if (idx < 0)
		idx = oldest;
	ScanEnt *e = &s_scan[idx];
	bool fresh = !e->used;
	e->used = 1;
	e->addrType = rpt->peer_addr.addr_type;
	memcpy(e->addr, rpt->peer_addr.addr, 6);
	e->rssi = rpt->rssi;
	e->isHid = 1;
	e->ms = millis();
	if (nm[0] || fresh) {
		memset(e->name, 0, sizeof e->name);
		strncpy(e->name, nm, sizeof e->name - 1);
	}
}

// Bluefruit callback-task context (deferred). MUST resume the scanner unless a connect was initiated.
static void scanCb(ble_gap_evt_adv_report_t *rpt)
{
	if (s_state == BLE_ST_ON && !s_connecting) {
		int b = bondMatchReport(rpt);
		if (b >= 0 && b == s_failBond &&
		    millis() - s_failBondMs < RECONNECT_BACKOFF_MS)
			b = -1; // recent setup failure: let the backoff cool down
		if (b >= 0 && !bondConnected(b) && linkAvail()) {
			// bonded controller woke up: reconnect (connect implicitly stops the scanner)
			s_connecting = true;
			s_connectingMs = millis();
			s_scanOn = false;
			Bluefruit.Central.connect(rpt);
			return;
		}
		if (s_uiScan)
			scanAddEntry(rpt);
	}
	Bluefruit.Scanner.resume();
}

// ---- input path ----
// BLE task context (notify deferral bypassed for latency): copy the raw report into the link ring. Loop-side
// pop uses the same PRIMASK guard -- entries are ~28 B, so the IRQ-off windows stay in the low microseconds.
static void inputNotifyCb(BLEClientCharacteristic *chr, uint8_t *data,
			  uint16_t len)
{
	int ri = -1;
	BleLink *L = linkByChr(chr, &ri);
	if (!L || ri < 0 || !L->up || L->repIn[ri] < 0 || !L->drv)
		return;
	uint8_t rid = L->drv->inputRidByIndex((uint8_t)L->repIn[ri]);
	if (len > RAW_MAXP)
		len = RAW_MAXP;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t h = L->qh, next = (uint8_t)((h + 1) % RAWQ);
	if (next == L->qt) // full: evict oldest (newest state wins)
		L->qt = (uint8_t)((L->qt + 1) % RAWQ);
	L->q[h].rid = rid;
	L->q[h].len = (uint8_t)len;
	memcpy(L->q[h].d, data, len);
	L->qh = next;
	__set_PRIMASK(pm);
}
static bool rawPop(BleLink *L, RawEnt *out)
{
	bool ok = false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (L->qt != L->qh) {
		*out = L->q[L->qt];
		L->qt = (uint8_t)((L->qt + 1) % RAWQ);
		ok = true;
	}
	__set_PRIMASK(pm);
	return ok;
}
static void batNotifyCb(BLEClientCharacteristic *chr, uint8_t *data,
			uint16_t len)
{
	int ri;
	BleLink *L = linkByChr(chr, &ri);
	if (L && len >= 1)
		L->battery = data[0];
}

// ---- connect / disconnect (Bluefruit callback task; blocking waits are fine here) ----
static void linkDrop(BleLink *L, uint16_t ch)
{
	slotRelease(L->slot);
	L->slot = -1;
	L->up = false;
	L->inUse = false;
	L->drv = nullptr;
	L->qh = L->qt = 0;
	L->rumPend = false;
	if (ch != BLE_CONN_HANDLE_INVALID) {
		BLEConnection *c = Bluefruit.Connection(ch);
		if (c && c->connected())
			c->disconnect();
	}
}

// Stamp the reconnect backoff for the bond behind a failing connection (no-op for never-bonded peers).
static void failMark(uint16_t ch)
{
	BLEConnection *c = Bluefruit.Connection(ch);
	if (!c)
		return;
	ble_gap_addr_t a = c->getPeerAddr();
	int b = bondFind(a.addr_type, a.addr);
	if (b >= 0) {
		s_failBond = (int8_t)b;
		s_failBondMs = millis();
	}
}

static void connectCb(uint16_t ch)
{
	s_connecting = false;
	BLEConnection *conn = Bluefruit.Connection(ch);
	BleLink *L = nullptr;
	for (int i = 0; i < BLE_LINKS; i++)
		if (!s_link[i].inUse) {
			L = &s_link[i];
			break;
		}
	if (!L || !conn) {
		if (conn)
			conn->disconnect();
		return;
	}
	L->inUse = true;
	L->up = false;
	L->conn = ch;
	L->slot = -1;
	L->outIdx = -1;
	L->qh = L->qt = 0;
	L->battery = 0;
	L->name[0] = 0;

	// 1) Security first: HID pads gate report traffic behind encryption. A bonded peer re-encrypts
	//    automatically (BLECentral does it on connect); give that 2 s, then fall back to fresh pairing.
	unsigned long t0 = millis();
	while (!conn->secured() && conn->connected() && millis() - t0 < 2000)
		delay(25);
	if (!conn->secured() && conn->connected())
		conn->requestPairing();
	while (!conn->secured() && conn->connected() && millis() - t0 < 12000)
		delay(50);
	if (!conn->secured() || !conn->connected()) {
		linkDrop(L, ch);
		return;
	}

	// 2) identity: GAP name + DIS PnP ID (VID/PID) -> driver match
	conn->getPeerName(L->name, sizeof L->name - 1);
	uint16_t vid = 0, pid = 0;
	if (L->dis.discover(ch) && L->pnp.discover()) {
		uint8_t b[7];
		if (L->pnp.read(b, sizeof b) >= 5) {
			vid = (uint16_t)(b[1] | (b[2] << 8));
			pid = (uint16_t)(b[3] | (b[4] << 8));
		}
	}
	L->drv = inputDriverFor(vid, pid, L->name);
	if (!L->drv) {
		// No driver claims it: drop the connection AND the just-created stack bond so it doesn't
		// auto-reconnect forever. Leaves the scan-table entry for the panel to show.
		ble_gap_addr_t a = conn->getPeerAddr();
		bond_remove_key(BLE_GAP_ROLE_CENTRAL, &a);
		linkDrop(L, ch);
		return;
	}
	L->vid = vid;
	L->pid = pid;

	// 3) battery service (optional)
	if (L->bat.discover(ch) && L->batc.discover()) {
		L->battery = L->batc.read8();
		L->batc.setNotifyCallback(batNotifyCb, false);
		L->batc.enableNotify();
	}

	// 4) HID service: discover up to BLE_NREP report characteristics (positional, handle order), subscribe
	//    every one that has a CCCD (= input report); the first without one is the output (rumble) report.
	if (!L->hid.discover(ch)) {
		failMark(ch);
		linkDrop(L, ch);
		return;
	}
	BLEClientCharacteristic *arr[BLE_NREP];
	for (int i = 0; i < BLE_NREP; i++) {
		L->rep[i].setNotifyCallback(inputNotifyCb, false);
		L->repIn[i] = -1;
		arr[i] = &L->rep[i];
	}
	Bluefruit.Discovery.discoverCharacteristic(ch, arr, BLE_NREP);
	uint8_t inN = 0;
	for (int i = 0; i < BLE_NREP; i++) {
		if (!L->rep[i].discovered())
			continue;
		if (L->rep[i].enableNotify())
			L->repIn[i] = (int8_t)inN++;
		else if (L->outIdx < 0)
			L->outIdx = (int8_t)i;
	}
	if (inN == 0) {
		failMark(ch);
		linkDrop(L, ch);
		return;
	}

	// 5) claim a bond slot + persist the record (identity address is stable once bonded)
	int slot = slotClaim();
	if (slot < 0) {
		failMark(
			ch); // e.g. all four slots RF-bonded -- back off, don't reconnect-loop
		linkDrop(L, ch);
		return;
	}
	memset((void *)&g_in[slot], 0, sizeof g_in[slot]);
	g_slotSrc[slot] = SRC_BLE;
	L->slot = (int8_t)slot;

	ble_gap_addr_t id = conn->getPeerAddr();
	int b = bondFind(id.addr_type, id.addr);
	if (b < 0)
		for (int i = 0; i < BLE_NBOND; i++)
			if (!s_bond[i].used) {
				b = i;
				break;
			}
	if (b >= 0) {
		s_bond[b].used = 1;
		s_bond[b].addrType = id.addr_type;
		memcpy(s_bond[b].addr, id.addr, 6);
		s_bond[b].vid = vid;
		s_bond[b].pid = pid;
		memset(s_bond[b].name, 0, sizeof s_bond[b].name);
		strncpy(s_bond[b].name, L->name, sizeof s_bond[b].name - 1);
		bleBondsSave();
		keysCacheLoad();
	}

	// input latency: ask for a 15 ms connection interval (units of 1.25 ms)
	conn->requestConnectionParameter(12);
	if (b >= 0 && s_failBond == (int8_t)b)
		s_failBond = -1; // made it through -- clear any earlier backoff
	L->up = true;
}

static void disconnectCb(uint16_t ch, uint8_t reason)
{
	(void)reason;
	s_connecting = false;
	BleLink *L = linkByConn(ch);
	if (L)
		linkDrop(L, BLE_CONN_HANDLE_INVALID); // already disconnected
}

// ---- public API ----
uint8_t bleState()
{
	return s_state;
}

void bleHostBegin()
{
	if (s_state == BLE_ST_ON)
		return;
	if (!g_bleEn) {
		// BLE off: clear any stale boot-latch (e.g. BLE was disabled mid-bringup, before the stable-clear)
		// so a later re-enable isn't falsely tripped into the "disable" branch below.
		if (bleBootPending())
			bleBootDisarm();
		return;
	}
	// Anti-bootloop: if the previous boot armed BLE but never reached the stable-clear, its SoftDevice/central
	// bringup (or first seconds of running) faulted or hung. Do NOT re-run that init -- disable BLE (persisted)
	// and come up clean. The user re-enables from the panel once the cause is addressed.
	if (bleBootPending()) {
		bleBootDisarm();
		g_bleEn = 0;
		saveCfg();
		s_state = BLE_ST_FAILED;
		return;
	}
	// Arm the latch in flash BEFORE touching the SoftDevice. Written via NVMC here (SD still off); cleared
	// later (bleHostTask, after BLE_STABLE_MS) once bringup is proven -- by which point a fault/hang would
	// already have reset us with the flag still set.
	bleBootArm();
	s_bootLatched = true;
	// micros() is only µs-resolution when the DWT cycle counter is on (else it degrades to the ~1ms RTOS
	// tick -- fatal for the timeslot deadline math in rf_timeslot). The core never enables it itself (only
	// a debugger does), so force it here.
	dwt_enable();
	Bluefruit.autoConnLed(
		false); // the board LED is the wake debugger (status_led), keep it
	if (!Bluefruit.begin(0, BLE_LINKS)) {
		// clean failure (not a crash) -- e.g. SoftDevice absent; disarm so we don't disable BLE next boot
		bleBootDisarm();
		s_bootLatched = false;
		s_state = BLE_ST_FAILED;
		return;
	}
	g_sdEnabled = true;
	Bluefruit.Central.setConnectCallback(connectCb);
	Bluefruit.Central.setDisconnectCallback(disconnectCb);
	Bluefruit.Central.setConnInterval(12, 24); // 15..30 ms
	for (int i = 0; i < BLE_LINKS; i++) {
		BleLink &L = s_link[i];
		L.dis.begin();
		L.pnp.begin(&L.dis);
		L.bat.begin();
		L.batc.begin(&L.bat);
		L.hid.begin();
		for (int r = 0; r < BLE_NREP; r++)
			L.rep[r].begin(&L.hid);
	}
	Bluefruit.Scanner.setRxCallback(scanCb);
	Bluefruit.Scanner.restartOnDisconnect(
		false); // scanEnsure() owns the scanner
	Bluefruit.Scanner.useActiveScan(
		true); // scan responses carry the names the UI shows
	bleBondsLoad();
	keysCacheLoad();
	memset(s_scan, 0, sizeof s_scan);
	s_state = BLE_ST_ON;
	s_bootUpMs =
		millis(); // start the stable-clear timer for the anti-bootloop latch
	rfTsBegin(); // ESB keeps the radio via timeslots from here on
	scanEnsure();
}

void bleShutdownForUpdate()
{
	if (!g_sdEnabled)
		return;
	rfTsEnd();
	sd_softdevice_disable();
	g_sdEnabled = false; // radio gates open again (bare-metal legal)
	s_state = BLE_ST_OFF_UPDATE;
	for (int s = 0; s < NSLOT; s++)
		if (slotIsBle(s))
			slotRelease(s);
}

bool bleSetRumble(uint8_t slot, uint16_t lo, uint16_t hi)
{
	for (int i = 0; i < BLE_LINKS; i++) {
		BleLink &L = s_link[i];
		if (L.inUse && L.up && L.slot == (int8_t)slot) {
			L.rumLo = lo;
			L.rumHi = hi;
			L.rumPend = true;
			return true;
		}
	}
	return false;
}

void bleScanUi(bool on)
{
	if (s_state != BLE_ST_ON)
		return;
	s_uiScan = on;
	s_uiScanUntil = on ? millis() + UI_SCAN_MS : 0;
	if (on)
		memset(s_scan, 0, sizeof s_scan);
	// re-apply duty cycle: stop -> scanEnsure() restarts with the new params
	if (s_scanOn) {
		Bluefruit.Scanner.stop();
		s_scanOn = false;
	}
	scanEnsure();
}

bool blePairTo(uint8_t addrType, const uint8_t addr[6])
{
	if (s_state != BLE_ST_ON || s_connecting || !linkAvail())
		return false;
	if (s_scanOn) {
		Bluefruit.Scanner.stop();
		s_scanOn = false;
	}
	ble_gap_addr_t a;
	memset(&a, 0, sizeof a);
	a.addr_type = addrType;
	memcpy(a.addr, addr, 6);
	s_connecting = true;
	s_connectingMs = millis();
	return Bluefruit.Central.connect(&a);
}

bool bleForget(uint8_t addrType, const uint8_t addr[6])
{
	int b = bondFind(addrType, addr);
	// drop a live link to it first
	for (int i = 0; i < BLE_LINKS; i++) {
		BleLink &L = s_link[i];
		if (!L.inUse || !L.up)
			continue;
		BLEConnection *c = Bluefruit.Connection(L.conn);
		if (!c)
			continue;
		ble_gap_addr_t pa = c->getPeerAddr();
		if (pa.addr_type == addrType && memcmp(pa.addr, addr, 6) == 0)
			c->disconnect(); // disconnectCb releases the slot/link
	}
	ble_gap_addr_t a;
	memset(&a, 0, sizeof a);
	a.addr_type = addrType;
	memcpy(a.addr, addr, 6);
	if (g_sdEnabled)
		bond_remove_key(BLE_GAP_ROLE_CENTRAL, &a);
	if (b < 0)
		return false;
	memset(&s_bond[b], 0, sizeof s_bond[b]);
	bleBondsSave();
	keysCacheLoad();
	return true;
}

void bleHostTask()
{
	rfTsTick();
	if (s_state != BLE_ST_ON)
		return;
	unsigned long now = millis();

	// Anti-bootloop: bringup has now survived BLE_STABLE_MS of running (SoftDevice enable, first scan and any
	// early connect are all well behind us), so clear the boot-attempt latch -- future boots trust BLE again.
	// A crash before this point leaves the flag set, so the next boot disables BLE instead of looping.
	if (s_bootLatched && (now - s_bootUpMs) >= BLE_STABLE_MS) {
		bleBootDisarm();
		s_bootLatched = false;
	}

	// A BLE controller only PRESENTS in the emulated modes; in the puck modes it stays parked (see header).
	const bool present = g_active && !g_active->isPuck();

	for (int i = 0; i < BLE_LINKS; i++) {
		BleLink &L = s_link[i];
		if (!L.inUse || !L.up || L.slot < 0)
			continue;
		int slot = L.slot;
		// Steam wrote an RF bond into our claimed slot (pairing a new SC2 while a BLE pad is live):
		// the RF poll owns `used` slots, so evacuate -- drop the link; auto-reconnect re-claims a free one.
		if (g_slot[slot].used) {
			BLEConnection *c = Bluefruit.Connection(L.conn);
			slotRelease(slot);
			L.slot = -1;
			L.up = false;
			if (c && c->connected())
				c->disconnect();
			continue;
		}
		bool got = false;
		RawEnt e;
		while (rawPop(&L, &e))
			if (L.drv->decode(slot, e.rid, e.d, e.len, &g_in[slot]))
				got = true;
		if (got && present) {
			g_connReplyMs[slot] = now;
			g_battery[slot] = L.battery;
			g_batteryState[slot] = L.battery ? 1 : 0;
			// wake gesture parity with the RF path: guide press while the bus sleeps wakes the host
			if (USBDevice.suspended() &&
			    (g_in[slot].buttons & TB_STEAM)) {
				USBDevice.remoteWakeup();
				ledWakePulse();
				if (g_active)
					g_active->wakeEvent();
			} else {
				uint8_t r45[PUCK45_LEN];
				puckSynth45(&g_in[slot], ++L.seq, r45);
				g_active->onReport45(slot, r45, true,
						     PUCK45_LEN);
			}
		}
		// rumble latch -> HID output report characteristic, rate-bounded (write-no-response)
		if (L.rumPend && L.outIdx >= 0 && now - L.rumMs >= 30) {
			uint8_t p[16];
			L.rumPend = false;
			L.rumMs = now;
			uint8_t n = L.drv->buildRumble(L.rumLo, L.rumHi, p);
			if (n)
				L.rep[L.outIdx].write(p, n);
		}
	}

	// housekeeping at ~2 Hz: UI-scan timeout, stuck-connect cancel, scanner state
	static unsigned long hkMs = 0;
	if (now - hkMs < 500)
		return;
	hkMs = now;
	if (s_uiScan && (long)(now - s_uiScanUntil) >= 0)
		s_uiScan = false;
	if (s_connecting && now - s_connectingMs > CONNECT_TIMEOUT_MS) {
		sd_ble_gap_connect_cancel();
		s_connecting = false;
	}
	scanEnsure();
}

// ---- panel status frame ----
// [0xAC][paylen][ver=1][state][uiScan][nScan]
//   4 x bond: [flags(b0 used, b1 connected)][slot(0xFF)][addrType][addr6][vid lo,hi][pid lo,hi][batt][name12]
//   nScan x:  [addrType][addr6][rssi s8][isHid][name12]
uint8_t bleStatusFrame(uint8_t *out, uint8_t maxLen)
{
	uint8_t n = 2;
	out[0] = 0xAC;
	out[n++] = 1; // ver
	out[n++] = s_state;
	out[n++] = s_uiScan ? 1 : 0;
	uint8_t nScanAt = n++; // patched below
	for (int b = 0; b < BLE_NBOND; b++) {
		if ((uint8_t)(n + 26) > maxLen)
			return 0;
		uint8_t flags = s_bond[b].used ? 1 : 0;
		uint8_t slot = 0xFF, batt = 0;
		if (s_bond[b].used)
			for (int i = 0; i < BLE_LINKS; i++) {
				BleLink &L = s_link[i];
				if (!L.inUse || !L.up)
					continue;
				BLEConnection *c = Bluefruit.Connection(L.conn);
				if (!c)
					continue;
				ble_gap_addr_t a = c->getPeerAddr();
				if (a.addr_type == s_bond[b].addrType &&
				    memcmp(a.addr, s_bond[b].addr, 6) == 0) {
					flags |= 2;
					slot = (uint8_t)L.slot;
					batt = L.battery;
					break;
				}
			}
		out[n++] = flags;
		out[n++] = slot;
		out[n++] = s_bond[b].addrType;
		memcpy(&out[n], s_bond[b].addr, 6);
		n += 6;
		out[n++] = (uint8_t)(s_bond[b].vid & 0xFF);
		out[n++] = (uint8_t)(s_bond[b].vid >> 8);
		out[n++] = (uint8_t)(s_bond[b].pid & 0xFF);
		out[n++] = (uint8_t)(s_bond[b].pid >> 8);
		out[n++] = batt;
		memcpy(&out[n], s_bond[b].name, 12);
		n += 12;
	}
	uint8_t nScan = 0;
	for (int i = 0; i < BLE_NSCAN; i++) {
		if (!s_scan[i].used)
			continue;
		if ((uint8_t)(n + 21) > maxLen)
			break;
		out[n++] = s_scan[i].addrType;
		memcpy(&out[n], s_scan[i].addr, 6);
		n += 6;
		out[n++] = (uint8_t)s_scan[i].rssi;
		out[n++] = s_scan[i].isHid;
		memcpy(&out[n], s_scan[i].name, 12);
		n += 12;
		nScan++;
	}
	out[nScanAt] = nScan;
	out[1] = (uint8_t)(n - 2);
	return n;
}
