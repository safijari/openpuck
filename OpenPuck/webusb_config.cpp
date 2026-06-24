#include "webusb_config.h"
#include "config.h"
#include "bonds.h"
#include "rf_link.h"
#include "haptics.h"
#include "puck_hid.h"
#include "triton.h" // g_in raw IMU (diagnostic readout)
#include "build_info.h"
#include <Arduino.h>
#include <string.h>

Adafruit_USBD_WebUSB usb_web;

// blob payload = [ver=9][mode][mDiv][mFric][qamMap(active)][abSwap(active)][back0..3(active)][connSlot(0xFF=none)][linkUp]
//                [f1ps_lo][f1ps_hi][pollU100][newps_lo][newps_hi][e7b][relayOp][relaySub][fwdNewOnly]
//                [qos][persistMode][chordBtn B][chordBtn X][chordBtn Y][pollsps_lo][pollsps_hi]
//                [loopPeriod_lo][loopPeriod_hi][loopWorstIdx][loopWorstUs_lo][loopWorstUs_hi]
//                [pollPeriod_lo][pollPeriod_hi][logEnabled][battery%][rssi|dBm|]
//                [gitDirty][gitHash 12B ASCII, NUL-padded][rumbleScale][swPro120][swGyroScale10][raw accel ax ay az 3x s16 LE]
//                [bondedCount][slot0_up][slot0_batt][slot0_rssi]...[slot3_up][slot3_batt][slot3_rssi]
//                [v9: per-type cfg, 4x7B: ET_XBOX/SWITCH/DS4/DS5 each {back0..3, qam, abSwap, padHaptics}]
// p[6]/p[7]/p[8..11] mirror the ACTIVE type (legacy display). v9 extends to 101 bytes (103 total incl header);
// browser reads with transferIn(128) to span the two USB-FS packets.
#define WB_PAYLEN 101
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

	// protocol version (9 = +per-type cfg; 8 = +per-slot link status; 7 = +raw accel; 6 = +swPro120/gyroScale)
	p[2] = 9;
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
	// per-emulated-type button config (protocol v9): 4 types x 7 bytes from p[75]
	for (int et = 0; et < ET_COUNT; et++) {
		uint8_t *q = &p[75 + et * 7];
		q[0] = g_type[et].back[0];
		q[1] = g_type[et].back[1];
		q[2] = g_type[et].back[2];
		q[3] = g_type[et].back[3];
		q[4] = g_type[et].qamMap;
		q[5] = g_type[et].abSwap;
		q[6] = g_type[et].padHaptics;
	}
	usb_web.write(p, sizeof p);
	usb_web.flush();
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
	while (budget-- && hapLogPull(&ms, &slot, &rid, &nb, b))
		webusbCapFrame(ms, slot, rid, nb, b);
	webusbCapEnd();
}
#endif // OPK_LOG
void webusbPoll()
{
	static uint8_t buf[16];
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

			// 0x05 carries a value byte; 0x0A carries a 3-byte magic
			uint8_t need = (op == 0x02)		  ? 3 :
				       (op == 0x03 || op == 0x05) ? 2 :
				       (op == 0x0A)		  ? 4 :
								    1;
			if (op < 0x01 || op > 0x0C) { // resync: drop one byte
				memmove(buf, buf + 1, --n);
				continue;
			}
			if (n < need)
				break; // wait for more bytes
			if (op == 0x01) {
				webusbSendBlob();
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
					NVIC_SystemReset();
				}

				// reboot into serial DFU (adafruit-nrfutil)
			} else if (op == 0x0B) {
				usb_web.flush();
				delay(40);
				enterSerialDfu();

				// reboot into UF2 bootloader (USB mass storage)
			} else if (op == 0x0C) {
				usb_web.flush();
				delay(40);
				enterUf2Dfu();

			} else if (op == 0x02) {
				uint8_t f = buf[1], v = buf[2];

				// every settable field persists (poll rate is no longer settable)
				bool persist = true;
				// per-type cfg writes (protocol v9): field = 40 + et*8 + k, k: 0..3 back, 4 qam, 5 abSwap,
				// 6 padHaptics. Edits g_type[et]; refresh the live mirrors if it's the active type.
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
						if (et == g_etype)
							applyActiveType();
					}
					saveCfg();
					webusbSendBlob();
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
				}
				if (persist)
					saveCfg();
				webusbSendBlob();
			} else if (op == 0x03) {
				uint8_t m = buf[1];
				if (modeValid(m) && !USBDevice.suspended()) {
					webusbSendBlob();
					usb_web.flush();
					saveMode(m);
					delay(40);
					NVIC_SystemReset();
				}
			}
			memmove(buf, buf + need, n - need);
			n -= need;
		}
	}
}
