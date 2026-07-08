// ReversePuckFirmware.ino -- emulated Steam Controller 2 ("Triton"/IBEX) for an nRF52840 Pro Micro.
//
// The mirror image of OpenPuck: instead of receiving a real controller over RF and presenting it to a
// host, this firmware RECEIVES input from a Steam Deck over USB-CDC and TRANSMITS it over the puck RF
// link, so a puck (real Valve or OpenPuck) presents a "Steam Controller 2026" to whatever host the puck
// is plugged into. See ARCHITECTURE.md (OpenPuck) for the puck side; the protocol is symmetric.
//
//   identity / ctrl_bonds   -- FXA controller serial, the (up to 2) bonds to pucks (esb/bond[_2])
//   radio / ctrl_link       -- shared bare-metal RADIO; the inverse RF role (answer the puck's polls)
//   triton / report45       -- the shared input model g_in + the report-0x45 ENCODER
//   deck_input              -- the USB-CDC frame link to the deck/ forwarder app
//   ctrl_usb                -- the USB 28DE:1302 face + steamless pairing command channel
//   webusb_ctrl / fw_update -- the browser-panel vendor channel: list/remove paired pucks, DFU, flash new fw
//
// Build (the vendor TX FIFO must hold the 0xAC bond list whole -- see webusb_ctrl.cpp):
//   arduino-cli compile -b adafruit:nrf52:feather52840 \
//     --build-property "build.extra_flags=-DNRF52840_XXAA {build.flags.usb} -DCFG_TUD_VENDOR_TX_BUFSIZE=256" \
//     ReversePuckFirmware
// or: make reversepuck   (top-level Makefile target, bakes the flag in)
#include <Adafruit_TinyUSB.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

#include "identity.h"
#include "radio.h"
#include "ctrl_bonds.h"
#include "ctrl_usb.h"
#include "ctrl_link.h"
#include "deck_input.h"
#include "webusb_ctrl.h"
#include "fw_update.h"

// CDC (Deck link) + one HID control interface fit well within the default config buffer; use a modest
// fixed buffer so enumeration is deterministic.
static uint8_t g_usbCfgDesc[256];

void setup()
{
	// FIRST, before anything touches flash or USB: apply a panel-staged firmware update / full-board wipe if
	// one was committed on the previous boot. A pending wipe is honored before an apply. Both run from RAM and
	// never return when armed (they reset); the common path (nothing armed) is a couple of flash reads.
	fwupWipeIfArmed();
	fwupApplyIfArmed();

	genSerial();
	// safe default; the real session addr is adopted from the puck's E1
	rfGenSessionAddr();
	InternalFS.begin();
	loadCtrlBonds();

	// ---- USB: keep CDC (the Deck link) + add the Valve control HID; enumerate as 28DE:1302 ----
	USBDevice.detach();
	delay(30);
	USBDevice.setConfigurationBuffer(g_usbCfgDesc, sizeof g_usbCfgDesc);
	// FXA… controller pairing identity
	USBDevice.setSerialDescriptor(g_unit);
	ctrlUsbBegin();
	webusbCtrlBegin(); // vendor interface for the browser panel (manage pucks / DFU / flash)
	USBDevice.setConfigurationAttribute(
		0x80 | 0x20); // bus-powered | remote-wakeup
	USBDevice.attach();

	Serial.begin(115200);
	for (int i = 0; i < 300 && !USBDevice.mounted(); i++)
		delay(10); // wait up to 3s for mount, but NEVER hang

	int nb = 0;
	for (int i = 0; i < NBOND; i++)
		if (g_bond[i].used)
			nb++;
	Serial.printf(
		"# ReversePuck controller up: unit=%s bonds=%d (28DE:1302)\n",
		g_unit, nb);

	// Hardware watchdog (~8s): a wedged RX busy-wait or blocked CDC write self-recovers without a replug.
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
			  (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = 8UL * 32768UL - 1;
	NRF_WDT->RREN = WDT_RREN_RR0_Msk;
	NRF_WDT->TASKS_START = 1;
}

void loop()
{
	NRF_WDT->RR[0] = WDT_RR_RR_Reload; // feed the watchdog

	// flush dirty bonds / honor a pending reboot-to-wireless
	ctrlUsbPoll();
	// browser panel over WebUSB: list/remove paired pucks, DFU, stream a firmware update
	webusbCtrlPoll();
	// drain CDC: parse Deck input frames into g_in + forwarding state
	deckInputPoll();
	// RF: hunt/adopt the puck's E1, answer one poll with report 0x45
	ctrlLinkTask();
	// report bond + link state back to the Deck UI
	deckStatusTask();
	// flush the newest relayed haptic to the Deck (rate-limited, off the RF path)
	deckHapticTask();
}
