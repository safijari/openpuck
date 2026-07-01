#include "webusb_config.h"
#include "config.h"
#include "bonds.h"
#include "rf_link.h"
#include "haptics.h"
#include "puck_hid.h"
#include "triton.h" // g_in raw IMU (diagnostic readout)
#include "build_info.h"
#include "fault_diag.h"
#include "usb_tx.h"
#include <Arduino.h>
#include <string.h>

Adafruit_USBD_WebUSB usb_web;

// Panel-blob send is deferred to the usbd task: webusbPoll() (loop) just sets this, and webusbSofDrain()
// (registered with usb_tx, runs from tud_sof_cb on the usbd task) does the actual write+flush. usb_web.flush()
// goes through the same blocking osd_queue_send path as HID sends, so doing it from loop() could stall the
// loop and trip the watchdog under load -- with the panel open during heavy use that was a live risk.
static volatile bool g_blobRequest = false;
// Bond export ("clone" backup): a 0x09 panel command asks for a one-shot dump of all four bond slots so the
// browser can save them to a file and restore them onto a second puck. Like the status blob, the actual send
// is deferred to the usbd task (webusbSofDrain) so usb_web.write()/flush() never block loop().
static volatile bool g_bondExportRequest = false;

// blob payload = [ver=10][mode][mDiv][mFric][qamMap(active)][abSwap(active)][back0..3(active)][connSlot(0xFF=none)][linkUp]
//                [f1ps_lo][f1ps_hi][pollU100][newps_lo][newps_hi][e7b][relayOp][relaySub][fwdNewOnly]
//                [qos][persistMode][chordBtn B][chordBtn X][chordBtn Y][pollsps_lo][pollsps_hi]
//                [loopPeriod_lo][loopPeriod_hi][loopWorstIdx][loopWorstUs_lo][loopWorstUs_hi]
//                [pollPeriod_lo][pollPeriod_hi][logEnabled][battery%][rssi|dBm|]
//                [gitDirty][gitHash 12B ASCII, NUL-padded][rumbleScale][swPro120][swGyroScale10][raw accel ax ay az 3x s16 LE]
//                [bondedCount][slot0_up][slot0_batt][slot0_rssi]...[slot3_up][slot3_batt][slot3_rssi]
//                [v10: per-type cfg, 4x8B: ET_XBOX/SWITCH/DS4/DS5 each {back0..3, qam, abSwap, padHaptics, ledBright}]
//                [v11: resetReason(RR_* code)][resetReas raw u32 LE][rxWin/10 us][hapticBlockOn][hapticBlock s]
// p[6]/p[7]/p[8..11] mirror the ACTIVE type (legacy display). v11 extends to 113 bytes (115 total incl header);
// browser reads with transferIn(128) to span the two USB-FS packets.
//                [diag: crc/s][noRx/s][rfStallRecover count]
//                [v12: relayps(lo,hi)][clkLfSrc][clkHfSrc][usPerMs(lo,hi)][hangStage][curStage][stallMs/40][ringFault(lo,hi)][hangPC u32][hangLR u32][usbdStackFree(lo,hi)][loopStackFree(lo,hi)]
#define WB_PAYLEN 139
// The blob send is drop-on-full (never blocks loop), so the vendor TX FIFO MUST be able to hold a whole blob
// -- otherwise tud_vendor_write_available() never reaches the frame size and EVERY frame is dropped (blank
// panel / stale mappings). The Makefile sets -DCFG_TUD_VENDOR_TX_BUFSIZE=256; guard it here so a build without
// that flag (or a future blob that outgrows the FIFO) fails loudly instead of shipping a dead panel.
static_assert(CFG_TUD_VENDOR_TX_BUFSIZE >= 2 + WB_PAYLEN,
	      "CFG_TUD_VENDOR_TX_BUFSIZE too small to hold a WebUSB blob in one "
	      "write -- build via `make build` (sets 256), or raise the flag.");
static void webusbSendBlob()
{
	if (!usb_web.connected())
		return;
	// "connected" = the most recent poll cycle's slot had a fresh F-reply (matches what the firmware
	// presents to Steam on 0x79). The blob is sent on the panel's poll, so the slot + up flag change
	// every ~250ms in normal use.
	int cs = (g_curSlot >= 0 && g_curSlot < NSLOT) ? g_curSlot : 0;
	bool up = (g_curSlot >= 0 && (millis() - g_connReplyMs[cs]) < 300);
	uint8_t p[2 + WB_PAYLEN];
	p[0] = 0xA5;
	p[1] = WB_PAYLEN;

	// protocol version (12 = +relay rate + clock fingerprint; 11 = +reset cause; 10 = +ledBright per type; 9 = +per-type cfg; 8 = +per-slot link status; 7 = +raw accel; 6 = +swPro120/gyroScale)
	p[2] = 12;
	p[3] = g_usbMode;
	p[4] = (uint8_t)g_mDiv;
	p[5] = (uint8_t)g_mFric;
	p[6] = g_qamMap;
	p[7] = g_abSwap;
	p[8] = g_back[0];
	p[9] = g_back[1];
	p[10] = g_back[2];
	p[11] = g_back[3];
	p[12] = (g_curSlot >= 0) ? (uint8_t)g_curSlot : 0xFF;
	p[13] = up ? 1 : 0;
	p[14] = (uint8_t)g_f1ps;
	p[15] = (uint8_t)(g_f1ps >> 8);
	p[16] = (uint8_t)(g_pollUs / 100);
	p[17] = (uint8_t)g_newps;
	p[18] = (uint8_t)(g_newps >> 8);
	p[19] = g_e7b;
	p[20] = g_relayOp;
	p[21] = g_relaySub;
	p[22] = g_fwdNewOnly;
	p[23] = g_qos;
	p[24] = g_persistMode ? 1 : 0;
	p[25] = g_chordBtn[0];
	p[26] = g_chordBtn[1];
	p[27] = g_chordBtn[2];
	p[28] = (uint8_t)g_pollsps;

	// poll TX rate (vs delivered/new -> starvation vs reply-loss)
	p[29] = (uint8_t)(g_pollsps >> 8);
	p[30] = (uint8_t)g_loopPeriodUs;
	p[31] = (uint8_t)(g_loopPeriodUs >> 8); // avg loop iteration time
	p[32] = g_loopWorst;
	p[33] = (uint8_t)g_loopWorstUs;
	p[34] = (uint8_t)(g_loopWorstUs >> 8);
	p[35] = (uint8_t)g_pollPeriodUs;
	p[36] = (uint8_t)(g_pollPeriodUs >>
			  8); // measured poll period (intended 4000)
	p[37] = OPK_LOG; // logging build? panel shows/hides its log UI
	p[38] = g_battery
		[g_curSlot >= 0 && g_curSlot < NSLOT ?
			 g_curSlot :
			 0]; // controller battery % (report 0x43); 0=unknown
	p[39] = g_linkRssi[g_curSlot >= 0 && g_curSlot < NSLOT ?
				   g_curSlot :
				   0]; // RAW signal strength |dBm| (0=no sample)
	// git commit this firmware was built from + dirty flag; injected at build time
	// (build_info.h / gen_version.sh); "unknown" if the version header wasn't generated.
	p[40] = OPK_GIT_DIRTY ? 1 : 0;
	memset(&p[41], 0, 12); // 12B ASCII git hash, NUL-padded
	{
		const char *h = OPK_GIT_HASH;
		for (uint8_t i = 0; i < 12 && h[i]; i++)
			p[41 + i] = (uint8_t)h[i];
	}
	p[53] = g_rumbleScale; // rumble strength % (protocol v5)

	// Switch Pro report rate 0=66/1=120/2=full (protocol v6)
	p[54] = g_swProRate;

	// Switch Pro gyro sensitivity x10 (protocol v6)
	p[55] = g_swGyroScale10;
	{
		int16_t a[3] = { g_in[0].ax, g_in[0].ay, g_in[0].az };
		memcpy(&p[56], a, 6);
	} // raw accelerometer for scale diagnostics (protocol v7)

	// per-slot link status for all bond slots (protocol v8)
	p[62] = (uint8_t)bondedSlotCount();
	{
		unsigned long nowMs = millis();
		for (int s = 0; s < NSLOT; s++) {
			bool sup = g_slot[s].used && g_connReplyMs[s] != 0 &&
				   (nowMs - g_connReplyMs[s]) < 300u;
			p[63 + s * 3] = sup ? 1 : 0;
			p[64 + s * 3] = g_battery[s];
			p[65 + s * 3] = g_linkRssi[s];
		}
	}
	// per-emulated-type button config (protocol v10): 4 types x 8 bytes from p[75]
	for (int et = 0; et < ET_COUNT; et++) {
		uint8_t *q = &p[75 + et * 8];
		q[0] = g_type[et].back[0];
		q[1] = g_type[et].back[1];
		q[2] = g_type[et].back[2];
		q[3] = g_type[et].back[3];
		q[4] = g_type[et].qamMap;
		q[5] = g_type[et].abSwap;
		q[6] = g_type[et].padHaptics;
		q[7] = g_type[et].ledBright;
	}
	// last-boot reset cause (protocol v11): why we (re)booted -- watchdog hang vs HardFault vs intentional
	// reboot vs power-on (issue #72). p[107] = RR_* code; p[108..111] = raw RESETREAS for the curious.
	p[107] = faultDiagReason();
	uint32_t rr = faultDiagResetReas();
	p[108] = (uint8_t)rr;
	p[109] = (uint8_t)(rr >> 8);
	p[110] = (uint8_t)(rr >> 16);
	p[111] = (uint8_t)(rr >> 24);
	// Connection tunables (protocol v11): poll RX window in 10us units, post-connect haptic block enable,
	// and block duration in seconds.
	p[112] = (uint8_t)(g_rxWin / 10);
	p[113] = g_hapticBlockOn;
	p[114] = (uint8_t)(g_hapticBlockMs / 1000);
	// RF wedge diagnostics (always populated, even when the link reads down): CRC-fail/s and no-reply/s,
	// capped at 255. With Polls/s this distinguishes poll-loop-stopped vs RX-dead vs corrupt-replies.
	p[115] = (uint8_t)(g_crcps > 255 ? 255 : g_crcps);
	p[116] = (uint8_t)(g_norxps > 255 ? 255 : g_norxps);
	p[117] = (uint8_t)(g_rfStallRecover > 255 ? 255 : g_rfStallRecover);
	// v12: relay-frame TX rate (separated from true poll cycles -- Polls/s now reads ~250, Relay/s exposes the
	// host output-report flood that steals reply windows), and the clock fingerprint for clone-board triage.
	p[118] = (uint8_t)g_relayps;
	p[119] = (uint8_t)(g_relayps >> 8);
	p[120] = clockLfSrc();
	p[121] = clockHfSrc();
	uint16_t upm = clockUsPerMs();
	p[122] = (uint8_t)upm;
	p[123] = (uint8_t)(upm >> 8);
	// last hang's loop stage (0xFF = last boot wasn't a watchdog/lockup hang, or GPREGRET2 didn't survive)
	p[124] = faultDiagHangStage();
	// LIVE hang localization: current loop stage + how long loop() has been stuck (40ms units, capped). When
	// loop() wedges these keep updating because the blob is sent from the SOF interrupt, not loop().
	p[125] = faultDiagCurStage();
	uint32_t sms = faultDiagStallMs();
	p[126] = (uint8_t)(sms / 40 > 255 ? 255 : sms / 40);
	// relay-ring fault count: non-zero = we caught+recovered a desynced/corrupt ring that would otherwise be an
	// invisible IRQ-off watchdog hang. A climbing count points at the haptic relay path / memory corruption.
	p[127] = (uint8_t)g_ringFault;
	p[128] = (uint8_t)(g_ringFault >> 8);
	// PC/LR of the stuck code captured by the WDT pre-reset ISR on the last watchdog hang (0 = none captured =
	// the hang hard-masked interrupts). Map with addr2line on the .elf to name the function.
	uint32_t hpc = faultDiagHangPC(), hlr = faultDiagHangLR();
	p[129] = (uint8_t)hpc;
	p[130] = (uint8_t)(hpc >> 8);
	p[131] = (uint8_t)(hpc >> 16);
	p[132] = (uint8_t)(hpc >> 24);
	p[133] = (uint8_t)hlr;
	p[134] = (uint8_t)(hlr >> 8);
	p[135] = (uint8_t)(hlr >> 16);
	p[136] = (uint8_t)(hlr >> 24);
	// least-ever free stack (words) on the usbd task (800B total) and loop task -- usbd trending to 0 confirms
	// the overflow. words, not bytes.
	uint16_t usbdFree = faultDiagUsbdStackFree(), loopFree = faultDiagLoopStackFree();
	p[137] = (uint8_t)usbdFree;
	p[138] = (uint8_t)(usbdFree >> 8);
	p[139] = (uint8_t)loopFree;
	p[140] = (uint8_t)(loopFree >> 8);
	// CRITICAL: usb_web.write() SPINS (`while (remain && _connected) yield();`) until the IN FIFO drains or the
	// panel disconnects. If the panel holds the WebUSB interface open but stops reading its IN endpoint -- a
	// backgrounded tab, or the host briefly not servicing transferIn under load -- the FIFO never empties and
	// that spin hangs loop() until the ~8s watchdog resets us (RESETREAS=watchdog/hang). This is the ONE
	// cross-task tud_ call the marshalling refactor left able to block loop() (HID goes through the drop-oldest
	// ring; this didn't). So DROP the blob whenever the FIFO can't take it whole -- a status panel missing an
	// occasional frame is invisible, and loop() can never stall here again. (Matches the closing-the-panel
	// "fix": that just flips _connected=false to break the same spin.)
	if (tud_vendor_write_available() >= sizeof p) {
		usb_web.write(p, sizeof p);
		usb_web.flush();
	}
}

// Bond export frame (panel "Export config to file"). One self-describing frame carrying all four bond slots so
// the browser can clone a puck onto another without re-pairing:
//   [0xA7][len][ver=1][usedMask][slot0 rec 24]...[slot3 rec 24]   (len = 2 + NSLOT*24 = 98)
// usedMask bit s = slot s bonded. The session RF address is NOT exported: each puck re-derives it from the bond
// UUID + its own FICR DEVICEID at boot, and the bonded controller relearns it from the E1 host beacon on every
// reconnect (rf_link.cpp) -- so the 24-byte record is the whole portable identity. 98+2 < transferIn(128) and
// < CFG_TUD_VENDOR_TX_BUFSIZE (256), so it sends whole in one drop-on-full write like the status blob.
#define WB_BOND_PAYLEN (2 + NSLOT * 24)
static void webusbSendBondExport()
{
	if (!usb_web.connected())
		return;
	uint8_t p[2 + WB_BOND_PAYLEN];
	p[0] = 0xA7;
	p[1] = WB_BOND_PAYLEN;
	p[2] = 1; // format version
	uint8_t mask = 0;
	for (int s = 0; s < NSLOT; s++) {
		if (g_slot[s].used)
			mask |= (uint8_t)(1u << s);
		memcpy(&p[4 + s * 24], g_slot[s].rec, 24);
	}
	p[3] = mask;
	// Same anti-hang rule as the status blob: drop the frame if the FIFO can't take it whole, never spin.
	if (tud_vendor_write_available() >= sizeof p) {
		usb_web.write(p, sizeof p);
		usb_web.flush();
	}
}

// Runs on the usbd task (registered via usbTxRegisterDrain -> tud_sof_cb). Sends the blob if loop() asked for
// one. Keeps every usb_web write/flush off the loop task so it can't block on the device event queue.
static void webusbSofDrain(void)
{
	// If loop() has stopped beating, it's wedged -- keep pushing the blob (which carries the live stuck stage)
	// so the panel shows WHERE it hung during the ~8s before the watchdog fires. This runs on the SOF IRQ, so
	// it works even though loop() is dead. drop-on-full in webusbSendBlob keeps it from ever blocking.
	if (faultDiagStallMs() > 300)
		g_blobRequest = true;
	if (g_blobRequest) {
		g_blobRequest = false;
		webusbSendBlob();
	}
	if (g_bondExportRequest) {
		g_bondExportRequest = false;
		webusbSendBondExport();
	}
}
// Register the SOF drain. Call once from setup() (harmless even if WebUSB isn't enumerated -- webusbSendBlob
// no-ops while disconnected, and g_blobRequest is only ever set by panel traffic, which needs a connection).
void webusbInit(void)
{
	usbTxRegisterDrain(webusbSofDrain);
}
#if OPK_LOG
// Stream the capture ring (haptics / relayed host commands) to the panel as 0xA6 frames. Frame formats:
//   entry: [0xA6][L][T=1][age u32 LE][slot][rid][n][bytes n]   (L = 8 + n)
//   end:   [0xA6][1][T=0]
static void webusbCapFrame(uint32_t ms, uint8_t slot, uint8_t rid, uint8_t nb,
			   const uint8_t *b)
{
	if (nb > 16)
		nb = 16;
	uint8_t f[2 + 9 + 16];
	uint8_t L = (uint8_t)(8 + nb);
	f[0] = 0xA6;
	f[1] = L;
	f[2] = 1;
	f[3] = (uint8_t)ms;
	f[4] = (uint8_t)(ms >> 8);
	f[5] = (uint8_t)(ms >> 16);
	f[6] = (uint8_t)(ms >> 24); // absolute log time
	f[7] = slot;
	f[8] = rid;
	f[9] = nb;
	memcpy(f + 10, b, nb);
	usb_web.write(f, (uint16_t)(2 + L));
	usb_web.flush();
}
static void webusbCapEnd()
{
	uint8_t e[3] = { 0xA6, 1, 0 };
	usb_web.write(e, 3);
	usb_web.flush();
}
// 0x06: drain entries since the last drain (or since the 0x05 start/rewind). The panel polls this on a
// timer and accumulates, so a dump-from-boot of the whole ring streams over many polls without blocking the
// loop. A per-call budget bounds how long one poll spends here.
static void webusbDrainCapture()
{
	if (!usb_web.connected())
		return;
	uint32_t ms = 0;
	uint8_t slot = 0, rid = 0, nb = 0, b[16];
	uint16_t budget = 128;
	// Same anti-hang rule as the blob: webusbCapFrame's usb_web.write() spins until the FIFO drains, so stop
	// pulling the moment the FIFO can't hold a max-size frame (2+9+16=27 B) -- the panel resumes the drain on
	// its next 0x06 poll. Never let a backed-up panel hang loop() here.
	while (budget-- && tud_vendor_write_available() >= 27 &&
	       hapLogPull(&ms, &slot, &rid, &nb, b))
		webusbCapFrame(ms, slot, rid, nb, b);
	if (tud_vendor_write_available() >= 3)
		webusbCapEnd();
}
#endif // OPK_LOG
void webusbPoll()
{
	// 40 B holds the largest command: 0x0D writes one bond slot = [0x0D][slot][used][24 rec] = 27 B (still
	// inside one 64-B USB-FS OUT packet, so no command ever spans packets). Every other command is <= 4 B.
	static uint8_t buf[40];
	static uint8_t n = 0;
	while (usb_web.available()) {
		int c = usb_web.read();
		if (c < 0)
			break;
		if (n < sizeof buf)
			buf[n++] = (uint8_t)c;
		// process complete commands from the front of buf
		for (;;) {
			if (n == 0)
				break;
			uint8_t op = buf[0];
			if (op < 0x01 || op > 0x0F) { // resync: drop one byte
				memmove(buf, buf + 1, --n);
				continue;
			}
			// Command length (fixed per opcode). 0x0D = write-one-bond-slot (27 B); 0x05/0x0E/0x0F carry
			// one value byte; 0x02 a field+value; 0x0A a 3-byte magic.
			uint8_t need = (op == 0x0D)		      ? 27 :
				       (op == 0x02)		      ? 3 :
				       (op == 0x03 || op == 0x05 ||
					op == 0x0E || op == 0x0F)     ? 2 :
				       (op == 0x0A)		      ? 4 :
								        1;
			if (n < need)
				break; // wait for more bytes
			if (op == 0x01) {
				g_blobRequest =
					true; // sent from the usbd task (webusbSofDrain)
			}
			// 0x09: export all bond slots (clone backup). One 0xA7 frame, sent from the usbd task.
			else if (op == 0x09) {
				g_bondExportRequest = true;
			}
			// 0x0F: stability test on/off. Puck->controller haptics do NOT reset the controller's own
			// user-input idle auto-off (we already poll it every 4ms without keeping it awake), so instead
			// signal host-awake: enable the E7 announce (0xE7 00 00 = host-awake vs 00 01 = suspended, per
			// the RE) so the controller is told the host is active and (hopefully) doesn't power-save. The
			// 10s buzz stays -- it also exercises the haptic-relay path that correlates with the hang.
			else if (op == 0x0F) {
				g_stabTest = (buf[1] != 0);
				g_e7b = 0; // byte2=0 => host-awake
				g_e7announce = g_stabTest;
			}
#if OPK_LOG

			// rewind drain: buf[1]=1 from boot (whole ring), 0 from now
			else if (op == 0x05) {
				hapLogResetDrain(buf[1] != 0);
			} else if (op == 0x06) {
				webusbDrainCapture();
			} // drain entries since the rewind (panel polls this)
#endif

			// clear a stuck/latched haptic buzz on the controller
			else if (op == 0x07) {
				hapticReinit();
			}

			// trigger controller power-off (same path Steam 0x9F / host-suspend use)
			else if (op == 0x08) {
				hapticSendShutdown();
			}

			// factory wipe: erase cfg.bin + bonds.bin, reboot to clean defaults.
			// Guarded by a 3-byte magic ("ERS") so a stray/corrupt byte can never trigger it.
			// Irreversible -- the controller must be re-paired afterwards.
			else if (op == 0x0A) {
				if (buf[1] == 0x45 && buf[2] == 0x52 &&
				    buf[3] == 0x53) {
					usb_web.flush();
					factoryErase();
					delay(40);
					faultDiagArmIntentionalReset();
					NVIC_SystemReset();
				}

				// reboot into serial DFU (adafruit-nrfutil)
			} else if (op == 0x0B) {
				usb_web.flush();
				delay(40);
				faultDiagArmIntentionalReset();
				enterSerialDfu();

				// reboot into UF2 bootloader (USB mass storage)
			} else if (op == 0x0C) {
				usb_web.flush();
				delay(40);
				faultDiagArmIntentionalReset();
				enterUf2Dfu();

				// 0x0D: write ONE bond slot into RAM -- the "clone onto this puck" side of Export/Import.
				// [0x0D][slot][used][24 rec]. used=0 (or an empty record) clears the slot. The panel sends
				// one of these per slot, then a 0x0E to persist + reboot. Kept to a single small command
				// (<=27 B) so it never spans a USB packet, and acked with a status blob so the panel's
				// read-after-write keeps the OUT pipe flowing (same shape as the proven 0x02 setters).
			} else if (op == 0x0D) {
				uint8_t slot = buf[1], used = buf[2];
				if (slot < NSLOT) {
					if (used && !recEmpty(buf + 3)) {
						memcpy(g_slot[slot].rec, buf + 3,
						       24);
						g_slot[slot].used = true;
					} else {
						memset(g_slot[slot].rec, 0, 24);
						g_slot[slot].used = false;
					}
				}
				g_blobRequest = true; // ack

				// 0x0E: commit an import -- persist the bond slots written by 0x0D, optionally apply a USB
				// mode ([0x0E][mode]; 0xFF = leave unchanged), then reboot so the RF session addresses
				// regenerate from the new bond UUIDs.
			} else if (op == 0x0E) {
				uint8_t mode = buf[1];
				saveBonds();
				if (modeValid(mode))
					saveMode(mode);
				usb_web.flush();
				delay(40);
				faultDiagArmIntentionalReset();
				NVIC_SystemReset();

			} else if (op == 0x02) {
				uint8_t f = buf[1], v = buf[2];

				// every settable field persists (poll rate is no longer settable)
				bool persist = true;
				// per-type cfg writes (protocol v10): field = 40 + et*8 + k, k: 0..3 back, 4 qam, 5 abSwap,
				// 6 padHaptics, 7 ledBright. Edits g_type[et]; refresh the live mirrors if it's the active type.
				if (f >= 40 && f < 40 + ET_COUNT * 8) {
					uint8_t et = (uint8_t)((f - 40) / 8),
						k = (uint8_t)((f - 40) % 8);
					if (et < ET_COUNT) {
						if (k < 4)
							g_type[et].back[k] = v;
						else if (k == 4)
							g_type[et].qamMap = v;
						else if (k == 5)
							g_type[et].abSwap =
								v ? 1 : 0;
						else if (k == 6)
							g_type[et].padHaptics =
								v ? 1 : 0;
						else if (k == 7)
							g_type[et].ledBright =
								v > 100 ? 100 :
									  v;
						if (et == g_etype)
							applyActiveType();
					}
					saveCfg();
					g_blobRequest = true;
					memmove(buf, buf + need, n - need);
					n -= need;
					continue;
				}
				switch (f) {
				case 1:
					g_mDiv = v < 4 ? 4 : v;
					break;
				case 2:
					g_mFric = v > 99 ? 99 : v;
					break;
				// case 3 (padSmooth) removed -- Steam-mode pad coords are forwarded raw; Steam does its own smoothing.
				// Legacy single-value fields (4 abSwap, 5-8 back, 21 qam) edit the ACTIVE emulated type.
				case 4:
					if (g_etype < ET_COUNT) {
						g_type[g_etype].abSwap = v ? 1 :
									     0;
						applyActiveType();
					}
					break;
				case 5:
				case 6:
				case 7:
				case 8:
					if (g_etype < ET_COUNT) {
						g_type[g_etype].back[f - 5] = v;
						applyActiveType();
					}
					break;
				// case 9 (pollU100) removed -- poll rate is fixed at POLL_US_DEFAULT and no longer configurable.

				// E7 protocol-version B-byte (experimental v1 fast)
				case 10:
					g_e7b = v ? 1 : 0;
					break;

				// haptic-relay opcode
				case 11:
					g_relayOp = v;
					break;

				// haptic-relay sub-type
				case 12:
					g_relaySub = v;
					break;
				case 13:
					g_testHaptic = v ? v : 40;
					break; // inject v test haptics (0->40)

				// Steam: forward only fresh reports (dedupe)
				case 14:
					g_fwdNewOnly = v ? 1 : 0;
					break;
				case 15:
					g_qos = v ? 1 : 0;
					g_hopIdx = 0;
					g_qosBad = 0;
					g_qosCheckMs = millis();
					break; // QoS adaptive channel hopping

				// persist last mode across reboots (else always boot Steam)
				case 16:
					g_persistMode = v ? true : false;
					break;

				// back4+B/X/Y mode assignments
				case 17:
				case 18:
				case 19:
					if (modeValid(v))
						g_chordBtn[f - 17] = v;
					break;

				// reboot once WITH the CDC serial console (puck mode), then auto-revert
				case 20:
					armDebugCdcNextBoot();
					usb_web.flush();
					delay(40);
					faultDiagArmIntentionalReset();
					NVIC_SystemReset();
					break;

				// QAM physical button remap code (0=default/unmapped) -- active emulated type
				case 21:
					if (g_etype < ET_COUNT) {
						g_type[g_etype].qamMap = v;
						applyActiveType();
					}
					break;

				// rumble strength % (0=off, 100=1x, 200=double)
				case 22:
					g_rumbleScale = v;
					break;

				// Switch Pro report rate (0=66Hz,1=120Hz,2=full)
				case 23:
					g_swProRate = (v <= 2) ? v : 2;
					swProSaveCfg();
					persist = false;
					break;
				case 24:
					g_swGyroScale10 =
						(v >= 5 && v <= 30) ? v : 10;
					swProSaveCfg();
					persist = false;
					break; // Switch Pro gyro scale x10

				// (field 25, poll RX window, removed -- g_rxWin is now FIXED/not configurable)
				// (fields 27/28, post-connect haptic block, removed -- permanently disabled)
				}
				if (persist)
					saveCfg();
				g_blobRequest = true;
			} else if (op == 0x03) {
				uint8_t m = buf[1];
				if (modeValid(m) && !USBDevice.suspended()) {
					// best-effort status to the panel before the reboot; the SOF drain
					// sends it during the delay(40) below (no blocking flush on loop()).
					g_blobRequest = true;
					saveMode(m);
					delay(40);
					faultDiagArmIntentionalReset();
					NVIC_SystemReset();
				}
			}
			memmove(buf, buf + need, n - need);
			n -= need;
		}
	}
}
