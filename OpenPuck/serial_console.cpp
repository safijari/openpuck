#include "serial_console.h"
#include "radio.h"
#include "rf_diag.h"
#include "rf_link.h"
#include "haptics.h"
#include "bonds.h"
#include "config.h"
#include "puck_hid.h" // g_cmdCapture (feature-command capture toggle)
#include "fault_diag.h"
#include "usb_mount.h" // modeSwitchReboot()
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// CDC commands: l=listen, s=stop, cN=channel, p<hex>=prefix, a<8hex>=base addr, b=bonds, etc.
// Command order matters: else-if chain, FIRST matching letter wins. 'C' and 'H' appear twice; the second
// occurrence is unreachable.
void serialConsolePoll()
{
	static char line[24];
	static uint8_t li = 0;
	while (Serial.available()) {
		char c = Serial.read();
		if (c == '\n' || c == '\r') {
			line[li] = 0;
			// FULL factory wipe -- requires the exact word "ERASE-ALL" so it can't be fat-fingered. Reformats the
			// internal FS (cfg.bin + bonds.bin gone), reboots into clean defaults; controller must be re-paired.
			if (!strcmp(line, "ERASE-ALL")) {
				Serial.println(
					"# ERASING ALL persistent storage (config + bonds)...");
				factoryErase();
				Serial.println(
					"# done -- rebooting into clean defaults; re-pair the controller");
				delay(40);
				faultDiagArmIntentionalReset();
				NVIC_SystemReset();
			} else if (!strcmp(line, "AMP")) {
				// A/B: land Steam's amp/haptic 0x87 config (0x18/0x2E/0x34/0x35) so haptics play as ticks
				g_landAmp = !g_landAmp;
				Serial.printf(
					"# land amp/haptic 0x87 config %s\n",
					g_landAmp ? "ON" : "off");
			} else if (!strcmp(line, "CD")) {
				// A/B: content dedup on the Steam input forward (drop reports whose body minus the
				// free-running counter is unchanged) -- caps delivered rate at the controller's real
				// distinct-report rate instead of the poll rate.
				g_fwdContentDedup = !g_fwdContentDedup;
				Serial.printf("# content dedup %s\n",
					      g_fwdContentDedup ? "ON" : "off");
			} else if (!strcmp(line, "SS")) {
				// on-demand smoothness snapshot: print the last-second cached rates in ONE line, NOW,
				// instead of waiting for the guarded 1 Hz "# stat" (which the I45/FC flood starves during
				// a drag). new/s = the FRESH-delivered rate (what Steam receives); noRx = RF packet loss.
				Serial.printf(
					"# SS pollrate=%luHz polls=%u/s F1=%u/s new=%u/s crc=%u noRx=%u slot%d\n",
					(unsigned long)(1000000UL / g_pollUs),
					g_pollsps, g_f1ps, g_newps, g_crcps,
					g_norxps, g_curSlot);
			} else if (!strncmp(line, "PR", 2) &&
				   (line[2] >= '0' && line[2] <= '9')) {
				// live poll-rate sweep: "PR<hz>" sets the RF poll cadence (session-only, not persisted).
				// Clamped 100..1000 Hz. Watch the delivered report rate / feel to find the sweet spot.
				long hz = atol(line + 2);
				if (hz < 100)
					hz = 100;
				else if (hz > 1000)
					hz = 1000;
				g_pollUs = (uint32_t)(1000000L / hz);
				Serial.printf(
					"# poll rate -> %ld Hz (%lu us)\n", hz,
					(unsigned long)g_pollUs);
			} else if (!strcmp(line, "HR")) {
				// A/B: disable the puck->controller haptic relay (Steam 0x80-0x86 rumble/pad feedback)
				// to isolate whether relaying Steam's trackpad texture haptics degrades drag smoothness.
				g_hapticRelay = !g_hapticRelay;
				Serial.printf(
					"# haptic relay (Steam 0x80-0x86) %s\n",
					g_hapticRelay ? "ON" : "off");
			} else if (!strcmp(line, "S81")) {
				// A/B: drop Steam's relayed 0x81 CLEAR_DIGITAL_MAPPINGS (the connect amp-clicker)
				g_drop81 = !g_drop81;
				Serial.printf(
					"# drop relayed 0x81 (Steam mode) %s\n",
					g_drop81 ? "ON" : "off");
			} else if (!strcmp(line, "FC")) {
				// feature-command capture: log Steam's USB SET/GET commands to serial + suppress I45
				g_cmdCapture = !g_cmdCapture;
				Serial.printf(
					"# feature-cmd capture %s (I45 %s)\n",
					g_cmdCapture ? "ON" : "off",
					g_cmdCapture ? "suppressed" : "on");
			} else if (!strcmp(line, "L87")) {
				// EXPERIMENT: land all relayed 0x87 config verbatim (real-puck relay) vs the discard-whitelist.
				// Exact-match string (single letters are all taken; 'l' alone = rfListenStart).
				g_landAll87 = !g_landAll87;
				saveCfg();
				Serial.printf(
					"# land-all-0x87 (verbatim 0x87 relay) %s\n",
					g_landAll87 ? "ON" : "off");
			} else if (!strcmp(line, "FR")) {
				// re-dump the flight recorder trail captured before the last watchdog hang (also printed
				// automatically at boot, but CDC may not be attached yet then -- this reprints on demand).
				faultDiagDumpFlight();
			} else if (!strcmp(line, "VIT")) {
				// toggle the live per-second CDC vitals line (loop rate / usbd stack / heap / relay trend)
				g_vitals = !g_vitals;
				Serial.printf("# live vitals line %s\n",
					      g_vitals ? "ON" : "off");
			} else if (line[0] == 'l')
				rfListenStart();
			else if (line[0] == 'B') {
				g_rfBeacon = !g_rfBeacon;
				g_rfListen = false;
				Serial.printf("# beacon %s ch%u\n",
					      g_rfBeacon ? "ON" : "off",
					      g_rfCh);
			} else if (line[0] == 'L') {
				g_plen = strtol(line + 1, 0, 16);
				Serial.printf("# plen=%02X\n", g_plen);
			} else if (line[0] == '1') {
				g_s1incl = !g_s1incl;
				Serial.printf("# s1incl=%u\n", g_s1incl);
			} else if (line[0] == 'R') {
				g_rfCh = atoi(line + 1);
				g_rfRaw = true;
				g_rfListen = false;
				g_rfBeacon = false;
				rfRawStart(g_rfCh);
				Serial.printf("# raw cap ch%u\n", g_rfCh);
			} else if (line[0] == 'Z') {
				g_rfRaw = true;
				g_rfSweep = true;
				g_rfListen = false;
				g_rfBeacon = false;
				g_rfCh = 2;
				rfRawStart(g_rfCh);
				Serial.println("# raw SWEEP");
			} else if (line[0] == 'H') {
				g_rfHost = !g_rfHost;
				g_rfBeacon = false;
				g_rfListen = false;
				g_rfRaw = false;
				g_rfSweep = false;
				g_rfCh = 2;
				Serial.printf(
					"# HOSTFRAME beacon %s base %02X%02X%02X%02X (slot0 uuids %02X%02X%02X%02X/%02X%02X%02X%02X)\n",
					g_rfHost ? "ON" : "off", g_rfBase[0],
					g_rfBase[1], g_rfBase[2], g_rfBase[3],
					g_slot[0].rec[0], g_slot[0].rec[1],
					g_slot[0].rec[2], g_slot[0].rec[3],
					g_slot[0].rec[4], g_slot[0].rec[5],
					g_slot[0].rec[6], g_slot[0].rec[7]);
			} else if (line[0] == 'C') {
				g_rfBeacon = g_rfListen = g_rfRaw = g_rfSweep =
					g_rfHost = false;
				rfRespondStart();
			} else if (line[0] == 's') {
				g_rfListen = false;
				g_rfBeacon = false;
				g_rfRaw = false;
				g_rfSweep = false;
				g_rfHost = false;
				g_rfRespond = false;
				NRF_RADIO->TASKS_DISABLE = 1;
				Serial.println("# RF off");
			} else if (line[0] == 'u') {
				// id9=0 hold (MODE_STEAM only) A/B toggle (persisted; see haptics.h LIZKEEP_MS)
				g_lizKeep = !g_lizKeep;
				saveCfg();
				Serial.printf(
					"# id9=0 hold (Steam mode, /%ums) %s\n",
					(unsigned)LIZKEEP_MS,
					g_lizKeep ? "ON" : "off");
			}

			// switch USB mode: 0=steam 1=xbox 2=hori 3=lizard 4=swpro 5=ps5 6=hidgyro 7=ps5-game/clean 8=ds4-game/clean 9=ps3(dualshock3)
			else if (line[0] == 'x') {
				uint8_t m = strtoul(line + 1, 0, 10);
				if (modeValid(m)) {
					if (USBDevice.suspended()) {
						Serial.println(
							"# mode change blocked: host suspended");
					} else {
						Serial.printf(
							"# switch mode %u (reboot)\n",
							m);
						delay(20);
						// clean detach + reboot (releases held input on the outgoing device)
						modeSwitchReboot(m);
					}
				}
			} else if (line[0] == 'c') {
				g_rfCh = atoi(line + 1);
				Serial.printf("# ch=%u\n", g_rfCh);
				if (g_rfListen)
					rfListenStart();
			} else if (line[0] == 'p') {
				g_rfPrefix = strtol(line + 1, 0, 16);
				Serial.printf("# prefix=%02X\n", g_rfPrefix);
				if (g_rfListen)
					rfListenStart();
			} else if (line[0] == 'i') {
				g_crcinit = strtoul(line + 1, 0, 16);
				Serial.printf("# crcinit=%06lX\n",
					      (unsigned long)g_crcinit);
			} else if (line[0] == 'w') {
				g_whiteiv = atoi(line + 1);
				Serial.printf("# whiteiv=%u\n", g_whiteiv);
			} else if (line[0] == '0') {
				g_pcnf0 = strtoul(line + 1, 0, 16);
				Serial.printf("# pcnf0=%08lX\n",
					      (unsigned long)g_pcnf0);
			} else if (line[0] == '2') {
				g_pcnf1 = strtoul(line + 1, 0, 16);
				Serial.printf("# pcnf1=%08lX\n",
					      (unsigned long)g_pcnf1);
			} else if (line[0] == 'M') {
				g_mode = strtoul(line + 1, 0, 16);
				Serial.printf(
					"# mode=%u (3=Ble1M,4=Ble2M,0/2=Nrf)\n",
					g_mode);
			} else if (line[0] == 'I') {
				g_crcinit = strtoul(line + 1, 0, 16);
				Serial.printf("# crcinit=%08lX\n",
					      (unsigned long)g_crcinit);
			} else if (line[0] == 'P') {
				g_crcpoly = strtoul(line + 1, 0, 16);
				Serial.printf("# crcpoly=%lX\n",
					      (unsigned long)g_crcpoly);
			} else if (line[0] == 'N') {
				g_crccnf = strtoul(line + 1, 0, 16);
				Serial.printf("# crccnf=%X\n", g_crccnf);
			} else if (line[0] == 'Y') {
				g_rfCap = true;
				g_rfCapOne = true;
				g_rfReplay = g_rfHost = g_rfAuto = false;
				g_rfCh = 2;
				rfCapStart(2);
				Serial.println(
					"# capture ONE frame for replay...");
			} else if (line[0] == 'y') {
				g_rfReplay = !g_rfReplay;
				g_rfCap = g_rfHost = g_rfAuto = false;
				Serial.printf("# REPLAY %s (%uB)\n",
					      g_rfReplay ? "ON" : "off",
					      g_replayLen);
			} else if (line[0] == 'T') {
				g_statlen = strtoul(line + 1, 0, 16);
				g_pcnf1 = 0;
				Serial.printf("# statlen=%02X\n", g_statlen);
			} else if (line[0] == 'k') {
				g_connOn = !g_connOn;
				if (g_connOn) {
					g_connSt = 0;
					g_connStep = 0;
					g_connPoll = 0;
					g_connF1 = 0;
				}
				Serial.printf(
					"# CONN mode %s: %s%s poll on ch%u (param=%02X). F1 seen=%lu\n",
					g_connOn ? "ON" : "off",
					g_e7announce ? "E7-awake[00 00] -> " :
						       "",
					g_pollGet ? "E3+GET-report-0x45" :
						    "bare E3",
					g_rfCh, g_getParam,
					(unsigned long)g_connF1);
			} else if (line[0] == 'q') {
				g_getParam = g_getParam ? 0x00 : 0x2D;
				Serial.printf("# GET-0x45 param=%02X\n",
					      g_getParam);
			} else if (line[0] == 'v') {
				g_connVerbose = !g_connVerbose;
				Serial.printf(
					"# conn verbose %s (F1 seen=%lu)\n",
					g_connVerbose ? "ON" : "off",
					(unsigned long)g_connF1);
			} else if (line[0] == 'C') {
				g_sessCh = strtoul(line + 1, 0, 10);
				Serial.printf(
					"# session channel=%u (re-pair/reconnect to apply)\n",
					g_sessCh);
			} else if (line[0] == 'E') {
				g_mDiv = strtoul(line + 1, 0, 10);
				if (g_mDiv < 4)
					g_mDiv = 4;
				saveCfg();
				Serial.printf(
					"# xbox-mouse sensitivity divisor=%d (lower=faster)\n",
					g_mDiv);
			} else if (line[0] == 'F') {
				g_mFric = strtoul(line + 1, 0, 10);
				if (g_mFric > 99)
					g_mFric = 99;
				saveCfg();
				Serial.printf(
					"# xbox-mouse friction=%d%% (higher=more glide/momentum)\n",
					g_mFric);
			} else if (line[0] == 'W') {
				// A/B swap edits the ACTIVE emulated type (Xbox/Switch/DS4/DS5); puck modes have none.
				if (g_etype < ET_COUNT) {
					g_type[g_etype].abSwap =
						!g_type[g_etype].abSwap;
					applyActiveType();
					saveCfg();
					Serial.printf(
						"# A/B + X/Y swap %s (Nintendo layout) [type %u]\n",
						g_abSwap ? "ON" : "off",
						g_etype);
				} else
					Serial.println(
						"# A/B swap N/A in this mode (Steam/Lizard are not emulated types)");
			} else if (line[0] == 'K') {
				int i = line[1] - '0';
				uint8_t code = strtoul(line + 2, 0, 10);
				// back paddles edit the ACTIVE emulated type. code 18 = Capture/Screenshot (Switch only).
				if (i >= 0 && i < 4 && g_etype < ET_COUNT) {
					g_type[g_etype].back[i] = code;
					applyActiveType();
					saveCfg();
					Serial.printf(
						"# back[%d] (%s) -> code %u [type %u]  [0=none 1=A 2=B 3=X 4=Y 5=LB 6=RB 7=L3 8=R3 9=Back 10=QAM 11=Guide 12=Dup 13=Ddown 14=Dleft 15=Dright 16=TouchClick 17=Mute 18=Capture(Switch) 19=LT/L2/ZL 20=RT/R2/ZR]\n",
						i,
						(const char *[]){ "L4", "R4",
								  "L5",
								  "R5" }[i],
						code, g_etype);
				} else if (g_etype >= ET_COUNT)
					Serial.println(
						"# back map N/A in this mode (Steam/Lizard are not emulated types)");
				else
					Serial.println(
						"# usage: K<0-3> <code>  (0=L4 1=R4 2=L5 3=R5)");
			} else if (line[0] == 'Q') {
				uint8_t code = strtoul(line + 1, 0, 10);
				if (g_etype < ET_COUNT) {
					g_type[g_etype].qamMap = code;
					applyActiveType();
					saveCfg();
					Serial.printf(
						"# QAM -> code %u [type %u]  [0=default 1=A 2=B 3=X 4=Y 5=LB 6=RB 7=L3 8=R3 9=Back 10=QAM 11=Guide 12=Dup 13=Ddown 14=Dleft 15=Dright 16=TouchClick 17=Mute 18=Capture(Switch) 19=LT/L2/ZL 20=RT/R2/ZR]\n",
						code, g_etype);
				} else
					Serial.println(
						"# QAM map N/A in this mode (Steam/Lizard are not emulated types)");
			} else if (line[0] == 'J') {
				char *sp = 0;
				uint8_t id = strtoul(line + 1, &sp, 0);

				// inject SET-SETTINGS to controller: report 0x87 [id][val u16 LE]
				uint16_t val = sp ? strtoul(sp, 0, 0) : 0;
				uint8_t pl[3] = { id, (uint8_t)(val & 0xFF),
						  (uint8_t)(val >> 8) };
				// console-injected writes go to all connected controllers (no specific slot in scope)
				relayEnqueue(0x87, pl, 3, 0xFF);
				Serial.printf(
					"# queued SET-SETTINGS id=0x%02X val=%u (relay 0x87) — watch new=/s\n",
					id, val);
			} else if (line[0] == 'G') {
				g_sniff = !g_sniff;
				g_sniffPh = 0;
				g_rfHost = g_connOn = g_rfCap = g_rfAuto =
					g_rfReplay = false;
				if (g_sniff)
					rfSniffStart();
				Serial.printf(
					"# SESSION SNIFF %s (phase0: catch host frame on ibex/ch2; phase1 PARK ch=%s)\n",
					g_sniff ? "ON" : "off",
					g_sniffPark ? "override" : "primary");
			} else if (line[0] == 'g') {
				g_sniffPark = strtoul(line + 1, 0, 10);
				if (g_sniffPh == 1)
					rfSniffStart();
				Serial.printf(
					"# sniff park channel=%u (0=learned primary)\n",
					g_sniffPark);
			} else if (line[0] == 'V') {
				g_e7b = strtoul(line + 1, 0, 10);
				Serial.printf(
					"# E7 protocol-version B-byte=%u (0=awake/slow, 1=try v1-fast; watch new=/s; revert if input stops)\n",
					g_e7b);
			} else if (line[0] == 'O') {
				g_relayOp = strtoul(line + 1, 0, 16);
				Serial.printf("# relay opcode=%02X\n",
					      g_relayOp);
			} else if (line[0] == 'o') {
				g_relaySub = strtoul(line + 1, 0, 16);
				Serial.printf("# relay sub-type=%02X\n",
					      g_relaySub);
			} else if (line[0] == 'h') {
				uint8_t nc = strtoul(line + 1, 0, 10);
				Serial.printf(
					"# HOP %u->%u (advertise on current ch, then poll new). Watch F1=/s\n",
					g_sessCh, nc);
				rfHopTo(nc);
			} else if (line[0] == 'z') {
				g_qos = !g_qos;
				g_hopIdx = 0;
				g_qosBad = 0;
				g_qosCheckMs = millis();
				Serial.printf(
					"# QoS adaptive channel hopping %s (candidates 18,46,76,22,68)\n",
					g_qos ? "ON" : "off");
			} else if (line[0] == 'U') {
				// Per-slot debug dump: which controllers are connected, their RSSI, haptic-block
				// state, and session address. Quick at-a-glance view for multi-controller testing.
				Serial.printf("# session ch%u, curSlot=%d\n",
					      g_sessCh, g_curSlot);
				for (int s = 0; s < NSLOT; s++) {
					bool live = g_slot[s].used &&
						    g_connReplyMs[s] != 0;
					unsigned long age =
						live ? (unsigned long)(millis() -
								       g_connReplyMs
									       [s]) :
						       0;
					int rssi =
						g_linkRssi[s] ?
							(int)g_linkRssi[s] -
								RSSI_DBM_OFFSET :
							0;
					unsigned long now_ms = millis();
					unsigned long blk =
						(g_hapticBlockUntil[s] &&
						 (int32_t)(g_hapticBlockUntil[s] -
							   now_ms) > 0) ?
							(g_hapticBlockUntil[s] -
							 now_ms) :
							0;
					Serial.printf(
						"# slot %d: %s addr %02X%02X%02X%02X/%02X rssi=%ddBm conn=%lums ago batt=%u%% haptic-block=%lums uuid %02X%02X%02X%02X %02X%02X%02X%02X\n",
						s,
						g_slot[s].used ?
							(live ? "LIVE" :
								"BONDED") :
							"empty",
						g_sessBase[s][0],
						g_sessBase[s][1],
						g_sessBase[s][2],
						g_sessBase[s][3],
						g_sessPrefix[s], rssi, age,
						g_battery[s], blk,
						g_slot[s].rec[0],
						g_slot[s].rec[1],
						g_slot[s].rec[2],
						g_slot[s].rec[3],
						g_slot[s].rec[4],
						g_slot[s].rec[5],
						g_slot[s].rec[6],
						g_slot[s].rec[7]);
				}
			} else if (line[0] == 'r') {
				// poll RX-window is FIXED (not configurable) -- report only.
				Serial.printf(
					"# poll RX-window=%lu us (FIXED; poll rate caps ~%lu/s)\n",
					(unsigned long)g_rxWin,
					(unsigned long)(1000000 / g_rxWin));
			} else if (line[0] == 'e') {
				g_e3mode = strtoul(line + 1, 0, 10);
				Serial.printf(
					"# E3 poll PID mode=%u (0=fixed07, 1=cyclePID+noack1, 2=cyclePID+noack0) - watch new=/s\n",
					g_e3mode);
			} else if (line[0] == 'd') {
				g_pollGet = !g_pollGet;
				Serial.printf(
					"# poll = %s (real puck sends BARE E3) - watch F1=/s new=/s\n",
					g_pollGet ?
						"E3 + GET-report-0x45 TLV (legacy)" :
						"bare E3");
			} else if (line[0] == 'n') {
				g_e7announce = !g_e7announce;
				Serial.printf(
					"# E7 awake-announce %s (real puck never sends E7) - watch F1=/s\n",
					g_e7announce ? "ON (legacy)" : "off");
			} else if (line[0] == 'm') {
				g_e1keepalive = !g_e1keepalive;
				Serial.printf(
					"# session-channel E1 keepalive %s (real puck sends none; needed for shared addr) - watch F1=/s\n",
					g_e1keepalive ? "ON" : "off");
			} else if (line[0] == 't') {
				// inject n test haptics (output 0x82 [01 01 F7]) over the relay
				uint8_t n = line[1] ? strtoul(line + 1, 0, 10) :
						      40;
				g_testHaptic = n;
				Serial.printf(
					"# test-haptic burst x%u queued (relay 0x82 01 01 F7 via op=%02X sub=%02X)\n",
					n, g_relayOp, g_relaySub);
			}

			// dump the captured OUTPUT-report history (oldest->newest)
			else if (line[0] == 'H') {
				hapticDumpLog();
			} else if (line[0] == 'j') {
				g_connType = strtoul(line + 1, 0, 16);
				Serial.printf("# connType=%02X\n", g_connType);
			} else if (line[0] == 'Q') {
				g_connLen = strtoul(line + 1, 0, 16);
				Serial.printf("# connLen=%02X\n", g_connLen);
			} else if (line[0] == 'A') {
				g_balen = strtoul(line + 1, 0, 16);
				g_pcnf1 = 0;
				Serial.printf("# balen=%u\n", g_balen);
			} else if (line[0] == 'a') {
				uint32_t v = strtoul(line + 1, 0, 16);
				g_rfBase[0] = v >> 24;
				g_rfBase[1] = v >> 16;
				g_rfBase[2] = v >> 8;
				g_rfBase[3] = v;
				Serial.printf("# base=%08lX\n",
					      (unsigned long)v);
				if (g_rfListen)
					rfListenStart();
			} else if (line[0] == 'b') {
				for (int i = 0; i < NSLOT; i++)
					if (g_slot[i].used) {
						Serial.printf("slot%d ", i);
						for (int j = 0; j < 24; j++)
							Serial.printf(
								"%02X",
								g_slot[i].rec[j]);
						Serial.println();
					}
			} else if (line[0] == 'D') {
				armDebugCdcNextBoot();
				Serial.println(
					"# debug-CDC armed: reset to KEEP the serial console one more boot (auto-reverts after; the wake mouse is dropped that boot)");
			} else if (line[0] == 'X') {
				g_rfCap = !g_rfCap;
				g_rfHost = g_rfAuto = g_rfBeacon = g_rfListen =
					g_rfRaw = g_rfSweep = g_rfRespond =
						false;
				g_rfCh = 2;
				g_capV = 0;
				if (g_rfCap)
					rfCapStart(g_rfCh);
				Serial.printf(
					"# CAPTURE(listen) %s ch%u on \"ibex\", cycling 1M/2M x BALEN3/4 (use cN to set ch)\n",
					g_rfCap ? "ON" : "off", g_rfCh);
			} else if (line[0] == 'S') {
				g_rfAuto = !g_rfAuto;
				g_rfHost = g_rfAuto;
				g_rfBeacon = g_rfListen = g_rfRaw = g_rfSweep =
					false;
				g_rfCh = 2;
				g_cfgIdx = 0;
				if (g_rfAuto)
					applyCfg(0);
				Serial.printf("# AUTOSWEEP %s\n",
					      g_rfAuto ? "ON" : "off");
			}
			li = 0;
		} else if (li < sizeof line - 1)
			line[li++] = c;
	}
}
