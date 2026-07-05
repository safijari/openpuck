// ble_host.h -- BLE central: discover, pair, bond and stream input from BLE HID controllers.
//
// The transport half of the BLE-controller feature (the byte-meaning half is input_driver.h). Runs the S140
// SoftDevice in the CENTRAL role next to the bare-metal ESB link (rf_timeslot.h arbitrates the radio):
//   - scanner: passive-ish background scan that (a) spots bonded controllers advertising and auto-reconnects,
//     and (b) while the panel's "scan" is armed, collects nearby HID gamepads for the pairing UI.
//   - connect flow (runs on the Bluefruit callback task): secure (Just Works pairing / bonded re-encryption;
//     HID pads refuse report traffic before encryption), read the DIS PnP ID for VID/PID, match an
//     IInputDriver, subscribe every HID input-report characteristic, claim a free bond slot.
//   - input path: notify callbacks (BLE task) push raw reports into a per-link ring; bleHostTask() (loop task)
//     drains it, lets the driver decode into g_in[slot], stamps g_connReplyMs[slot], and dispatches a
//     synthesized report 0x45 through g_active->onReport45() -- exactly the shape the RF path produces, so
//     every USB personality works unchanged. g_in stays single-writer-from-loop.
//   - rumble: hapticSteamRumble() forks BLE slots here (bleSetRumble latch); the latch is flushed to the
//     device's HID output-report characteristic from loop at a bounded rate.
//   - persistence: pairing LTKs live in the BLE stack's own bond store (/adafruit/bond_cntr); our metadata
//     (identity address, name, VID/PID) lives in /blebonds.bin so the panel can list and forget devices.
//
// Presentation rule: BLE controllers surface only in the EMULATED modes (XInput/Switch/PS5/DS4 -- everything
// that reads g_in or the synthesized 0x45). In the Steam/Lizard puck modes they stay parked (connected, not
// presented): Steam's per-slot feature channel expects a real SC2 bond record behind each slot.
#pragma once
#include <stdint.h>

// lifecycle state (panel field)
#define BLE_ST_OFF \
	0 // disabled (persisted flag off, or never started this boot)
#define BLE_ST_ON 1 // SoftDevice up, central running
#define BLE_ST_FAILED \
	2 // Bluefruit/SoftDevice refused to start (clone clock issue?)
#define BLE_ST_OFF_UPDATE 3 // shut down mid-run for a staged firmware update

// Start the SoftDevice + central role. No-op unless g_bleEn (config.h) -- callable both at setup() and live
// from the panel's enable command.
void bleHostBegin();
// Per-loop pump: raw-report drain -> g_in -> dispatch, rumble flush, reconnect/scan upkeep, rfTsTick().
void bleHostTask();
// Latch rumble for a BLE-driven slot (called from hapticSteamRumble, any task). False if slot isn't BLE-live.
bool bleSetRumble(uint8_t slot, uint16_t lo, uint16_t hi);
// Staged-firmware-update path: close the timeslot session and DISABLE the SoftDevice so fw_update's direct
// NVMC writes are legal again (raw NVMC under an enabled SD corrupts BLE timing / asserts). BLE drops until
// the post-update reboot. Safe to call when BLE never started.
void bleShutdownForUpdate();

uint8_t bleState(); // BLE_ST_*
void bleScanUi(bool on); // panel scan start/stop (auto-stops after 60 s)
// Panel "pair": connect + bond to a scanned address. Returns false if BLE is off.
bool blePairTo(uint8_t addrType, const uint8_t addr[6]);
// Panel "forget": disconnect if live, drop the stack bond keys + our record.
bool bleForget(uint8_t addrType, const uint8_t addr[6]);

// Build the panel status frame [0xAC][len][ver][state][uiScan][nScan][4x bond rec][nScan x scan rec] into out
// (called from the usbd-task WebUSB drain; unsynchronized reads -- cosmetic). Returns total bytes or 0.
uint8_t bleStatusFrame(uint8_t *out, uint8_t maxLen);
