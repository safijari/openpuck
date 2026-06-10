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

static uint8_t g_usbCfgDesc[512];   // puck composite (4 HID + WebUSB) exceeds the default 256 B config buffer

void setup() {
  genSerial();
  ledInit();
  rfGenSessionAddr();   // per-device unique RF session address (advertised in the host frame; isolates pucks)
  InternalFS.begin();
  loadCfg(); g_xbox = !modeIsPuck(g_usbMode);   // load persisted config + decide USB presentation BEFORE registering interfaces
  g_active = controllerFor(g_usbMode);

  // ---- USB descriptor rebuild ----
  // Puck mode NORMALLY drops the CDC serial console (clearConfiguration) to free a USB endpoint for the
  // wake-mouse interface that lets it wake a sleeping Windows host. The one-shot debug arm (g_debugCdcThisBoot,
  // set via the WebUSB panel / CDC 'D', auto-reverting after one boot) instead KEEPS the boot CDC composite
  // for that boot and skips the wake mouse (no endpoint room for both). Clean modes always rebuild bare.
  const bool puckMode = g_active->isPuck();
  const bool keepCdc  = puckMode && g_debugCdcThisBoot;
  USBDevice.detach(); delay(30);
  if (keepCdc) {
    USBDevice.setConfigurationBuffer(g_usbCfgDesc, sizeof g_usbCfgDesc);   // keep boot CDC composite (debug boot)
  } else {
    USBDevice.clearConfiguration();
    USBDevice.setConfigurationBuffer(g_usbCfgDesc, sizeof g_usbCfgDesc);   // headroom over the default 256 B cap
  }

  // Distinct USB serial PER MODE (must be set AFTER clearConfiguration, which nulls it). Hosts cache USB
  // identity by VID:PID:serial; reusing one serial under a changing VID:PID can make a host refuse the new
  // identity. Steam keeps the exact unit serial (its pairing identity); the others get a 1-char suffix.
  static const char MODE_SUFFIX[] = {'X','N','L','P','S','G'};   // modes 1..6
  if (puckMode) { USBDevice.setSerialDescriptor(g_unit); }
  else { snprintf(g_usbSerial, sizeof g_usbSerial, "%s%c", g_unit, MODE_SUFFIX[g_usbMode-1]); USBDevice.setSerialDescriptor(g_usbSerial); }

  g_active->begin();   // register this mode's USB interface(s) + set VID/PID/strings

  // Boot-mouse wake interface so the host honors USBDevice.remoteWakeup() in this mode (see wake_hid.h).
  // Added for every clean mode, and for puck mode on a normal (CDC-dropped) boot. Skipped only on the one-shot
  // debug boot, where CDC takes the endpoint instead.
  if (!keepCdc) wakeHidBegin();

  // WebUSB config panel -- every mode. Puck: historical CDC+HID+vendor stack (Steam + Chrome can share it).
  // Clean modes: bare gamepad + WebUSB (begin() sets bcdUSB 0x0210 + BOS).
  usb_web.begin();
  // Enable USB Remote Wakeup (bit 5) so the host lets us signal wake-from-sleep. Bit 7 is always required.
  USBDevice.setConfigurationAttribute(0x80 | 0x20);  // bmAttributes: required(0x80) | remote_wakeup(0x20)
  USBDevice.attach();   // re-connect with the final descriptor (host re-reads it fresh -> deterministic enumeration)
  Serial.begin(115200);
  for (int i=0; i<300 && !USBDevice.mounted(); i++) delay(10);   // wait up to 3s for USB mount, but NEVER hang
  if (USBDevice.suspended()){ USBDevice.remoteWakeup(); ledWakePulse(); }   // wake host if bus was sleeping when we (re-)attached
  loadBonds();
  hapticInit();   // clear relay/active flags + arm the reconnect block & initial stop burst
  static const char* MODE_NAME[]={"STEAM(puck)","XBOX(xinput+mouse)","SWITCH(horipad)","LIZARD(puck kb/mouse)","SWITCH(pro+gyro)","PS5(dualsense)","HIDGYRO(ds4+motion)"};
  Serial.printf("# copycat up: unit=%s board=%s, mode=%s\n", g_unit, g_board, MODE_NAME[g_usbMode<=MODE_MAX?g_usbMode:0]);
  if (puckMode) Serial.printf("# puck USB: %s\n", keepCdc ? "DEBUG boot (CDC console on, wake mouse off; reverts next boot)" : "normal (CDC off, wake mouse on)");
  Serial.printf("# session addr %02X%02X%02X%02X/%02X ch%u (discovery on ibex/ch2)\n",
                g_sessBase[0],g_sessBase[1],g_sessBase[2],g_sessBase[3],g_sessPrefix,g_sessCh);
  // Hardware watchdog: if loop() ever stops feeding it (a wedged radio busy-wait, a HardFault spin, a blocked
  // CDC write) the WDT resets the nRF52 after ~8s -- re-enumerating USB and re-initialising RF on its own, so a
  // hang no longer needs a physical replug. RUN keeps it counting in sleep; PAUSE freezes it under a debugger.
  NRF_WDT->CONFIG  = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV     = 8UL * 32768UL - 1;     // timeout in 32.768 kHz ticks (~8 s)
  NRF_WDT->RREN    = WDT_RREN_RR0_Msk;       // arm reload register 0
  NRF_WDT->TASKS_START = 1;
}

void loop() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;   // feed the watchdog each loop; if we ever stop, the ~8s WDT auto-resets us
  if (g_dirty) { g_dirty = false; saveBonds(); }
  webusbPoll();
  if (g_active) g_active->task();   // streamed-mode emit / handshake drains / mode upkeep (puck: USB conn presentation; xbox: rumble relay)
  serialConsolePoll();
  rfDiagTask();                     // service whichever RF RE/calibration mode is active (no-op in normal use)
  rfLinkTask();                     // host-frame beacons + connected-mode poll (decodes input -> g_in -> active controller) + QoS + stats
  hapticTask();                     // reconnect-block edge handling + steam 0x82 quiet timeout
  ledTask();                        // times out the 500ms wake-sent LED flash
}
