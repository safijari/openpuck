// OpenPuck.ino -- Steam Controller 2 ("Triton") puck reimplementation for an nRF52840.
//
// This firmware impersonates the Valve puck over USB, maintains puck-style bond slots, speaks the
// reverse-engineered RF protocol to the controller, and re-enumerates into Steam, Xbox, Switch, PS5, or DS4
// personalities. Build with -DCFG_TUD_HID=4 (the Adafruit nRF port defaults to 2).
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
#include "rf_diag.h"
#include "webusb_config.h"
#include "serial_console.h"
#include "wake_hid.h"
#include "status_led.h"

#if CFG_TUD_HID < 4
#error "build with -DCFG_TUD_HID=4 (extra_flags): up to 4 HID interfaces per mode"
#endif

// puck composite (4 HID + WebUSB) exceeds the default 256 B config buffer
static uint8_t g_usbCfgDesc[512];

void setup()
{
	genSerial();
	ledInit();

	// per-device unique RF session address (advertised in the host frame; isolates pucks)
	rfGenSessionAddr();
	InternalFS.begin();
#if OPK_FACTORY_RESET
	// Recovery build (-DOPK_FACTORY_RESET=1): wipe ALL persistent storage ONCE on the first boot after flashing,
	// then persist normally (tracked by a git-hash tag so it does NOT wipe every boot). See config.h / config.cpp.
	factoryResetOnce(OPK_GIT_HASH);
#endif
	loadCfg();

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
	static const char MODE_SUFFIX[] = {
		'X', 'N', 'L', 'P', 'S', 'G', 'Q', 'D'
	}; // modes 1..8 (Q=PS5 game, D=DS4 game)
	if (puckMode) {
		USBDevice.setSerialDescriptor(g_unit);
	} else {
		snprintf(g_usbSerial, sizeof g_usbSerial, "%s%c", g_unit,
			 MODE_SUFFIX[g_usbMode - 1]);
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
	USBDevice.setConfigurationAttribute(0x80 | 0x20);

	// re-attach with the final descriptor (host re-reads it fresh -> deterministic enumeration)
	USBDevice.attach();
	Serial.begin(115200);
	for (int i = 0; i < 300 && !USBDevice.mounted(); i++)
		delay(10); // wait up to 3s for USB mount, but NEVER hang
	if (USBDevice.suspended()) {
		USBDevice.remoteWakeup();
		ledWakePulse();
	} // wake host if bus was sleeping when we (re-)attached
	loadBonds();
	hapticInit();
	static const char *MODE_NAME[] = {
		"STEAM(puck)",	       "XBOX(xinput+mouse)",
		"SWITCH(horipad)",     "LIZARD(puck kb/mouse)",
		"SWITCH(pro+gyro)",    "PS5(dualsense)",
		"HIDGYRO(ds4+motion)", "PS5(dualsense,game/clean)",
		"DS4(ds4,game/clean)"
	};
	Serial.printf("# copycat up: unit=%s board=%s, mode=%s\n", g_unit,
		      g_board,
		      MODE_NAME[g_usbMode <= MODE_MAX ? g_usbMode : 0]);
	if (puckMode)
		Serial.printf(
			"# puck USB: %s\n",
			keepCdc ?
				"DEBUG boot (CDC console on, wake mouse off; reverts next boot)" :
				"normal (CDC off, wake mouse on)");
	Serial.printf(
		"# session addr %02X%02X%02X%02X/%02X ch%u (discovery on ibex/ch2)\n",
		g_sessBase[0], g_sessBase[1], g_sessBase[2], g_sessBase[3],
		g_sessPrefix, g_sessCh);
	// Hardware watchdog: if loop() ever stops feeding it (wedged radio busy-wait, HardFault spin, blocked CDC
	// write) the WDT resets the nRF52 after ~8s, re-enumerating USB + re-initialising RF, so a hang no longer
	// needs a physical replug. RUN keeps it counting in sleep; PAUSE freezes it under a debugger.
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
			  (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = 8UL * 32768UL - 1; // timeout in 32.768 kHz ticks (~8 s)
	NRF_WDT->RREN = WDT_RREN_RR0_Msk; // arm reload register 0
	NRF_WDT->TASKS_START = 1;
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
	if (g_dirty) {
		g_dirty = false;
		saveBonds();
	}
#if OPK_LOG
	// Diagnostic build: time each loop section so the panel can show the slowest one + the loop period.
	static uint32_t acc[7] = { 0 };
	static uint32_t loops = 0;
	static unsigned long secMs = 0;
	uint32_t t;
	t = micros();
	webusbPoll();
	acc[0] += (uint32_t)(micros() - t);
	t = micros();
	if (g_active)
		g_active->task();
	acc[1] += (uint32_t)(micros() - t);
	t = micros();
	serialConsolePoll();
	acc[2] += (uint32_t)(micros() - t);
	t = micros();
	rfDiagTask();
	acc[3] += (uint32_t)(micros() - t);
	t = micros();
	rfLinkTask();
	acc[4] += (uint32_t)(micros() - t);
	t = micros();
	hapticTask();
	acc[5] += (uint32_t)(micros() - t);
	t = micros();
	ledTask();
	acc[6] += (uint32_t)(micros() - t);
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
	webusbPoll();
	if (g_active)
		g_active->task();
	serialConsolePoll();
	rfDiagTask();
	rfLinkTask();
	hapticTask();
	ledTask();
#endif
}
