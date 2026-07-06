// OpenPuck.ino -- Steam Controller 2 ("Triton") puck reimplementation for an nRF52840.
//
// This firmware impersonates the Valve puck over USB, maintains puck-style bond slots, speaks the
// reverse-engineered RF protocol to the controller, and re-enumerates into Steam, Xbox, Switch, PS5, or DS4
// personalities. Build with `make build` (bakes in the required CFG_TUD_HID=4 + CFG_TUD_TASK_QUEUE_SZ=64).
//
// This file is just the entry point: setup() builds the USB presentation for the persisted mode and arms the
// hardware watchdog; loop() pumps each subsystem. Everything substantive lives in the modules below -- see
// ARCHITECTURE.md for the full map. The short version:
//   config / identity / bonds  -- persisted settings, device serial, the four controller bond slots
//   radio / rf_link / rf_diag   -- bare-metal nRF52 RADIO, the operational puck protocol, and RE tooling
//   triton / gamepad_util       -- the decoded controller input (g_in) + shared host-report build helpers
//   controllers + mode_*        -- one IController per USB personality (puck/lizard, xbox, switch, ps5, ds4)
//   haptics / webusb_config / serial_console -- haptic relay, the browser config channel, the CDC debug CLI
//
// Protocol reference: docs/PROTOCOL.md.  Bond record = [8 uuid][16 serial].
#include <Adafruit_TinyUSB.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

#include "config.h"

// OPK_GIT_HASH: tags the one-time factory-reset build
#include "build_info.h"
#include "identity.h"
#include "bonds.h"
#include "radio.h"
#include "controllers.h"
#include "haptics.h"
#include "rf_link.h"
#include "puck_hid.h" // puckCmdLogDrain()
#include "rf_diag.h"
#include "webusb_config.h"
#include "lizard_map.h"
#include "serial_console.h"
#include "wake_hid.h"
#include "status_led.h"
#include "usb_mount.h"
#include "identity.h"
#include "fault_diag.h"
#include "fw_update.h"
#include "usb_tx.h"
#include <stdio.h>

#if CFG_TUD_HID < 4
#error "CFG_TUD_HID must be >= 4 (up to 4 HID interfaces per mode). Build with `make build` (bakes it in), or pass -DCFG_TUD_HID=4 in build.extra_flags."
#endif

// The TinyUSB device-stack event queue (_usbd_q) defaults to 16 entries. tud_task() runs in the dedicated
// high-priority "usbd" FreeRTOS task, but this firmware emits HID reports from the LOW-priority loop() task
// (every mode: 0x45 forwards, per-slot 0x79/0x7B/0x43 status, streamed gamepad reports). A cross-task
// sendReport whose EasyDMA is momentarily busy takes TinyUSB's usbd_defer_func() path, which does a BLOCKING
// osal_queue_send (xQueueSendToBack WAIT_FOREVER) onto _usbd_q. Under a comms burst (haptics/trackpad floods,
// the connect-time re-init storm, 4 slots at once) the 16-deep queue saturates; the loop task then blocks
// posting to it and stops feeding the ~8 s watchdog -> the MCU resets and the controller "disconnects". A
// deeper queue absorbs realistic bursts so the loop never blocks. Must be set on the build command (it is a
// TinyUSB-internal #ifndef default, so a sketch header cannot reach usbd.c) -- enforce it here so a build that
// omits the flag fails loudly instead of shipping the deadlock.
#if !defined(CFG_TUD_TASK_QUEUE_SZ) || CFG_TUD_TASK_QUEUE_SZ < 32
#error "CFG_TUD_TASK_QUEUE_SZ must be >= 32: the default 16-deep usbd event queue deadlocks the loop task under comms load -> watchdog reset. Build with `make build` (bakes in 64), or pass -DCFG_TUD_TASK_QUEUE_SZ=64 in build.extra_flags."
#endif

// puck composite (4 HID + WebUSB) exceeds the default 256 B config buffer
static uint8_t g_usbCfgDesc[512];

// Per-mode USB serial suffix (modes 1..9: X=xbox N=hori L=lizard P=swpro S=ps5 G=hidgyro Q=ps5game D=ds4game 3=ps3).
static const char MODE_SUFFIX[] = {
	'X', 'N', 'L', 'P', 'S', 'G', 'Q', 'D', '3'
};
// Fixed-interface flags captured at boot so usbReenumerate (dynamic mount, no reboot) replays them.
static bool s_dynWantWebusb = false, s_dynWantWakeMouse = false;

// Dynamic re-enumeration (NO MCU reboot): tear down + rebuild the config descriptor presenting `k` connected-
// controller slots, replaying the fixed interfaces in locked HID-instance order (wake mouse = instance 0,
// before the slot pool; WebUSB is vendor-class so order-free), then re-attach. RF/firmware state survives.
// g_usbToBond must already be built for k connected controllers. Called at boot and by usb_mount's watcher.
void usbReenumerate(uint8_t k)
{
	USBDevice.detach();
	delay(20);
	USBDevice.clearConfiguration();
	USBDevice.setConfigurationBuffer(g_usbCfgDesc, sizeof g_usbCfgDesc);
	g_active->usbIdentity(); // clearConfiguration reset VID/PID/strings -- restore them
	// serial carries the mounted count so the host invalidates its cached config descriptor on a change
	snprintf(
		g_usbSerial, sizeof g_usbSerial, "%s%c%u", g_unit,
		MODE_SUFFIX[(g_usbMode >= 1 && g_usbMode <= 9) ? g_usbMode - 1 :
								 0],
		(unsigned)k);
	USBDevice.setSerialDescriptor(g_usbSerial);
	if (s_dynWantWakeMouse)
		wakeHidAddInterface(); // HID instance 0
	g_active->mountSlots(
		k); // mode's fixed HIDs (if any) + k slot interfaces
	if (s_dynWantWebusb)
		USBDevice.addInterface(usb_web);
	USBDevice.setConfigurationAttribute(0x80 | 0x20);
	USBDevice.attach();
}

// Persist a new USB personality and reboot into it. The clean DETACH before the reset is the point: a bare
// NVIC_SystemReset drops the D+ pullup only for the ~ms the reset takes -- too briefly for some hosts to
// register a real unplug -- so when switching between DIFFERENT personalities (e.g. Steam <-> Xbox, which are
// distinct VID/PID + class drivers), the OUTGOING device can linger as a ghost until a physical replug, and
// any input held at the instant of the switch stays stuck on it. Detaching first makes the reboot behave like
// that replug: the host tears the old device down (releasing all its input) before the new one enumerates.
// Used by every user-facing mode change (back-4 chord, WebUSB panel, CDC console).
void modeSwitchReboot(uint8_t mode)
{
	if (modeValid(mode))
		saveMode(
			mode); // 0xFF / invalid => keep the current mode (bond-import reboot)
	if (USBDevice.mounted())
		USBDevice.detach();
	delay(60); // let the host see the disconnect before the pullup returns on reset
	faultDiagArmIntentionalReset();
	NVIC_SystemReset();
}

void setup()
{
	// full-board wipe (debug-only "erase everything"): if the panel armed one, this never returns -- it erases
	// the app + config/bond + bootloader-settings flash from RAM and resets into the app-less UF2 bootloader.
	// Checked before the staged-update apply (both use the meta page; only one can be armed at a time).
	fwupWipeIfArmed();
	// staged firmware update (WebUSB "flash on reboot"): if the panel committed one, this never returns --
	// it copies staged->app from RAM and resets into the new firmware. MUST run before anything else touches
	// hardware; the board looks dead for the ~5 s the copy takes.
	fwupApplyIfArmed();
	genSerial();
	ledInit();

	// seed defaults so unbonded slots don't share the discovery address
	for (int s = 0; s < NSLOT; s++)
		rfGenSessionAddr(s);
	InternalFS.begin();
#if OPK_FACTORY_RESET
	// Recovery build (-DOPK_FACTORY_RESET=1): wipe ALL persistent storage ONCE on the first boot after flashing,
	// then persist normally (tracked by a git-hash tag so it does NOT wipe every boot). See config.h / config.cpp.
	factoryResetOnce(OPK_GIT_HASH);
#endif
	loadCfg();
	loadBonds();
	// Lizard (desktop) keyboard/mouse binding table. Custom remapping applies ONLY in pure
	// MODE_LIZARD: there we install the user's saved/editable map. In every other mode the
	// seamless lizard (Steam mode with Steam closed) uses the built-in DEFAULT map, so edits
	// made for pure lizard never change Steam-mode desktop behavior. (Without either call
	// g_lizardMap.count stays 0 and lizard produces no keyboard/mouse output at all.)
	if (g_usbMode == MODE_LIZARD)
		loadLizardMap();
	else
		defaultLizardMap();
	// regenerate per-slot session addresses from each bond UUID (deterministic, stable across reboots)
	for (int s = 0; s < NSLOT; s++)
		if (g_slot[s].used)
			rfGenSessionAddr(s);

	// decide USB presentation BEFORE registering interfaces
	g_xbox = !modeIsPuck(g_usbMode);
	g_active = controllerFor(g_usbMode);

	// ---- USB descriptor rebuild ----
	// Puck mode drops the CDC serial console (clearConfiguration) to free a USB endpoint for the wake-mouse
	// interface that wakes a sleeping Windows host. The one-shot debug arm (g_debugCdcThisBoot, set via WebUSB
	// panel / CDC 'D', auto-reverting after one boot) instead KEEPS the boot CDC composite and skips the wake
	// mouse (no endpoint room for both). Clean modes always rebuild bare.
	const bool puckMode = g_active->isPuck();
	const bool keepCdc = puckMode && g_debugCdcThisBoot;
	// The "game" PlayStation modes enumerate as a CLEAN single HID gamepad so games that classify off the raw HID
	// device (Fortnite/UE GameInput, Windows.Gaming.Input) recognise them as PlayStation. A real Sony pad is just
	// the gamepad HID (+ a USB-audio interface); our extra wake-mouse HID and WebUSB vendor interface make those
	// classifiers refuse the PS glyph path even with the correct VID/PID (SDL/Steam tolerate the extras). So
	// clean-PS modes skip BOTH the wake mouse and WebUSB -- no config panel / host-wake; chord back to Steam
	// (back-paddle 4 + A) to reach the panel. Normal MODE_PS5 / MODE_HIDGYRO keep wake + panel.
	const bool psClean = modeIsCleanPS(g_usbMode);
	const bool dynamic = g_active->dynamicMount();

	if (dynamic) {
		// Dynamic mount: present only ACTIVELY-CONNECTED controllers; usbReenumerate re-attaches (no reboot)
		// as the set changes. Emulated modes are never puck; clean-PS drops the wake mouse + WebUSB.
		s_dynWantWakeMouse = !psClean;
		s_dynWantWebusb = !psClean;
		USBDevice.detach();
		delay(30);
		USBDevice.clearConfiguration();
		USBDevice.setConfigurationBuffer(g_usbCfgDesc,
						 sizeof g_usbCfgDesc);
		// Lock TinyUSB HID instance indices ONCE, canonical order: wake mouse (HID 0) then the mode's pool.
		if (s_dynWantWakeMouse)
			wakeHidBegin();
		g_active->beginPool();
		if (s_dynWantWebusb)
			usb_web.begin();
		usbMountEnable(true, g_active->maxSlots());
		usbMountRebuildMap(); // initial connected set (usually empty at cold boot)
		usbReenumerate(
			g_usbMountCount); // build the live descriptor + attach
	} else {
		USBDevice.detach();
		delay(30);
		if (keepCdc) {
			USBDevice.setConfigurationBuffer(g_usbCfgDesc,
							 sizeof g_usbCfgDesc);
		} else {
			USBDevice.clearConfiguration();

			// headroom over the default 256 B cap
			USBDevice.setConfigurationBuffer(g_usbCfgDesc,
							 sizeof g_usbCfgDesc);
		}

		// Distinct USB serial PER MODE (must be set AFTER clearConfiguration, which nulls it). Hosts cache USB
		// identity by VID:PID:serial; reusing one serial under a changing VID:PID can make a host refuse the new
		// identity. Steam keeps the exact unit serial (its pairing identity); the others get a 1-char suffix.
		if (puckMode) {
			USBDevice.setSerialDescriptor(g_unit);
		} else {
			// Bond count in serial so Windows invalidates its cached config descriptor if the count changes.
			snprintf(g_usbSerial, sizeof g_usbSerial, "%s%c%d",
				 g_unit, MODE_SUFFIX[g_usbMode - 1],
				 bondedSlotCount() > 0 ? bondedSlotCount() : 1);
			USBDevice.setSerialDescriptor(g_usbSerial);
		}

		// SDL3's Proteus/Triton HIDAPI driver only binds slot HIDs on USB interfaces 2..5. Register WebUSB (IF 0)
		// and the wake mouse (IF 1) before the four puck slots so hid[0..3] land on IF 2..5 like the real puck.
		if (puckMode)
			usb_web.begin();
		if (puckMode && !keepCdc)
			wakeHidBegin();

		g_active->begin();

		// Boot-mouse wake interface for clean (non-puck) modes, and for puck on the one-shot debug boot (CDC on,
		// no endpoint room for wake mouse on a normal puck boot -- wake is registered above instead). Skipped for PS
		// modes so the device stays a single clean HID gamepad (see psClean above).
		if (!puckMode && !keepCdc && !psClean)
			wakeHidBegin();

		// WebUSB config panel -- every mode EXCEPT the PlayStation modes. Puck: registered above (IF 0) before wake +
		// slots; other clean modes after controller. PS modes omit it to present a genuine single-HID PS controller.
		if (!puckMode && !psClean)
			usb_web.begin();
		// bmAttributes: required(0x80) | remote_wakeup(0x20). Remote Wakeup lets us signal wake-from-sleep.
		// The PS3/Sixaxis (the only clean-PS mode on this static path) advertises 0x80 only -- match it so a
		// strict console doesn't see a capability the genuine pad never claims.
		USBDevice.setConfigurationAttribute(psClean ? 0x80 :
							      (0x80 | 0x20));

		// re-attach with the final descriptor (host re-reads it fresh -> deterministic enumeration)
		USBDevice.attach();
	}
	Serial.begin(115200);
	for (int i = 0; i < 300 && !USBDevice.mounted(); i++)
		delay(10); // wait up to 3s for USB mount, but NEVER hang
	if (USBDevice.suspended()) {
		USBDevice.remoteWakeup();
		ledWakePulse();
	} // wake host if bus was sleeping when we (re-)attached
	// Route all device->host HID sends through the usbd task (SOF drain) so loop() never calls tud_* directly
	// -> the cross-task blocking-defer that deadlocked the loop under comms load can no longer happen.
	usbTxBegin();
	webusbInit(); // also drain the WebUSB status blob from the usbd task (its flush() can block loop() too)
	hapticInit();
	static const char *MODE_NAME[] = {
		"STEAM(puck)",	       "XBOX(xinput+mouse)",
		"SWITCH(horipad)",     "LIZARD(puck kb/mouse)",
		"SWITCH(pro+gyro)",    "PS5(dualsense)",
		"HIDGYRO(ds4+motion)", "PS5(dualsense,game/clean)",
		"DS4(ds4,game/clean)", "PS3(dualshock3/sixaxis)"
	};
	Serial.printf("# copycat up: unit=%s board=%s, mode=%s\n", g_unit,
		      g_board,
		      MODE_NAME[g_usbMode <= MODE_MAX ? g_usbMode : 0]);
	// Classify why we (re)booted: distinguishes a watchdog hang from a HardFault from an intentional reboot
	// (issue #72 -- those are conflated in the field). Surfaced on the WebUSB panel too.
	faultDiagBoot();
	// Clock fingerprint: which crystals/oscillators this (possibly clone) board actually came up on. Surfaced
	// on the panel so flaky clones can be told apart from the known-good board.
	clockDiagBoot();
	if (puckMode)
		Serial.printf(
			"# puck USB: %s\n",
			keepCdc ?
				"DEBUG boot (CDC console on, wake mouse off; reverts next boot)" :
				"normal (CDC off, wake mouse on)");
	Serial.printf(
		"# session ch%u (discovery on ibex/ch2); per-slot session addrs:\n",
		g_sessCh);
	for (int s = 0; s < NSLOT; s++) {
		if (g_slot[s].used) {
			Serial.printf(
				"#   slot %d: %02X%02X%02X%02X/%02X (uuid %02X%02X%02X%02X %02X%02X%02X%02X)\n",
				s, g_sessBase[s][0], g_sessBase[s][1],
				g_sessBase[s][2], g_sessBase[s][3],
				g_sessPrefix[s], g_slot[s].rec[0],
				g_slot[s].rec[1], g_slot[s].rec[2],
				g_slot[s].rec[3], g_slot[s].rec[4],
				g_slot[s].rec[5], g_slot[s].rec[6],
				g_slot[s].rec[7]);
		}
	}
	// Hardware watchdog: if loop() ever stops feeding it (wedged radio busy-wait, HardFault spin, blocked CDC
	// write) the WDT resets the nRF52 after ~8s, re-enumerating USB + re-initialising RF, so a hang no longer
	// needs a physical replug. RUN keeps it counting in sleep; PAUSE freezes it under a debugger.
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
			  (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = 8UL * 32768UL - 1; // timeout in 32.768 kHz ticks (~8 s)
	NRF_WDT->RREN = WDT_RREN_RR0_Msk; // arm reload register 0
	NRF_WDT->TASKS_START = 1;
	// Capture the stuck PC on the WDT's pre-reset interrupt (software stand-in for SWD on the clone hangs).
	faultDiagArmHangCapture();
	// Flash black box: at ~6s of loop stall (2s before the WDT reset) dump both tasks' stacked PCs + vitals
	// to a reserved flash page -- the post-mortem channel that survives even the boards that wipe .noinit.
	faultDiagBlackBoxArm();
}

// loop-timing diagnostics: poll rate is capped by loop ITERATION time (pacing wants 4000us but the poll only
// fires as often as the loop comes around). Per-section us accumulates and each second the avg loop period +
// slowest section are published for the WebUSB panel.

// avg loop iteration time last second (1e6/iterations)
uint16_t g_loopPeriodUs = 0;

// index of the slowest section: 0=webusb 1=ctrl.task 2=serial 3=rfdiag 4=rflink 5=haptic 6=led
uint8_t g_loopWorst = 0;
uint16_t g_loopWorstUs = 0; // that section's avg us per iteration
void loop()
{
	// feed the watchdog; if we ever stop, the ~8s WDT auto-resets us
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;
	faultDiagBeat(); // loop heartbeat -- the SOF blob reports ms-since-beat so a wedge is visible live
	faultDiagStackTick(); // per-task stack headroom (self-gated ~1Hz) -- usbd-overflow hypothesis check
	faultDiagFlightTick(); // flight recorder: refresh vitals + heartbeat crumb + live CDC vitals line
	hapticStabTask(); // stability-test keepalive buzz (no-op unless armed via WebUSB)
	// cross-check HFCLK(micros) vs LFCLK(millis) once a second -- cheap, both builds (clone clock diagnostic)
	clockDiagTick();
	if (g_dirty) {
		g_dirty = false;
		// A bond flash write erases+programs a page with interrupts effectively stalled for ms -- a known
		// watchdog-hang class on slow-flash clones, so leave a crumb bracketing it in the recorder.
		faultDiagTrace(FR_SAVE, 0);
		saveBonds();
	}
#if OPK_LOG
	// Diagnostic build: time each loop section so the panel can show the slowest one + the loop period.
	static uint32_t acc[7] = { 0 };
	static uint32_t loops = 0;
	static unsigned long secMs = 0;
	uint32_t t;
	t = micros();
	faultDiagSetStage(0);
	webusbPoll();
	acc[0] += (uint32_t)(micros() - t);
	t = micros();
	faultDiagSetStage(1);
	if (g_active)
		g_active->task();
	acc[1] += (uint32_t)(micros() - t);
	t = micros();
	faultDiagSetStage(2);
	serialConsolePoll();
	acc[2] += (uint32_t)(micros() - t);
	t = micros();
	faultDiagSetStage(3);
	rfDiagTask();
	acc[3] += (uint32_t)(micros() - t);
	t = micros();
	faultDiagSetStage(4);
	rfLinkTask();
	acc[4] += (uint32_t)(micros() - t);
	t = micros();
	faultDiagSetStage(5);
	hapticTask();
	acc[5] += (uint32_t)(micros() - t);
	t = micros();
	faultDiagSetStage(6);
	ledTask();
	acc[6] += (uint32_t)(micros() - t);
	faultDiagSetStage(7);
	usbMountTask(); // dynamic mount/unmount of connected controllers (no-op unless enabled)
	faultDiagSetStage(8);
	usbTxPump(); // drain queued device->host reports HERE, in loop -- never off-loop (jitters the RF poll)
	puckCmdLogDrain(); // print captured USB feature commands (diagnostic; no-op unless g_cmdCapture)
	loops++;
	if (millis() - secMs >= 1000) {
		g_loopPeriodUs = loops ? (uint16_t)(1000000UL / loops) : 0;
		uint8_t wi = 0;
		uint32_t wv = 0;
		for (int i = 0; i < 7; i++) {
			if (acc[i] > wv) {
				wv = acc[i];
				wi = i;
			}
			acc[i] = 0;
		}
		g_loopWorst = wi;
		g_loopWorstUs = loops ? (uint16_t)(wv / loops) : 0;
		loops = 0;
		secMs = millis();
	}
#else
	// faultDiagSetStage breadcrumb (GPREGRET2) before each stage: if loop() wedges, the watchdog reset's next
	// boot reports which stage was stuck. Cheap (one register write each).
	faultDiagSetStage(0);
	webusbPoll();
	faultDiagSetStage(1);
	if (g_active)
		g_active->task();
	faultDiagSetStage(2);
	serialConsolePoll();
	faultDiagSetStage(3);
	rfDiagTask();
	faultDiagSetStage(4);
	rfLinkTask();
	faultDiagSetStage(5);
	hapticTask();
	faultDiagSetStage(6);
	ledTask();
	faultDiagSetStage(7);
	usbMountTask(); // dynamic mount/unmount of connected controllers (no-op unless enabled)
	faultDiagSetStage(8);
	usbTxPump(); // drain queued device->host reports HERE, in loop -- never off-loop (jitters the RF poll)
	puckCmdLogDrain(); // print captured USB feature commands (diagnostic; no-op unless g_cmdCapture)
#endif
}
