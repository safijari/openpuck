// Firmware update over WebUSB ("staged UF2"): the panel extracts the raw app image from a .uf2 and streams
// it over the existing WebUSB vendor channel (ops 0x20..0x24, see webusb_config.h) into UNUSED flash high in
// the app region. A one-page metadata commit -- written only after the staged bytes CRC32-verify in flash --
// arms the update; the next reboot applies it before anything else runs, then boots the new firmware.
//
// Corruption safety (the invariant every step preserves): at any instant the device has EITHER a valid old
// app, a valid new app, OR reads as app-less so the resident UF2 bootloader keeps the board (drag-and-drop
// recovery) -- never a half-written image that crash-loops:
//   - The transfer stages into flash the running image doesn't use; until fwupEnd() verifies the full CRC32
//     and commits the meta page, a disconnect/abort/power-cut leaves ZERO trace (no arm, app untouched).
//   - The boot-time applier re-verifies the staged CRC32 and vector table BEFORE erasing anything.
//   - The apply itself runs from RAM (flash code can't survive overwriting itself) with a page ordering that
//     makes power cuts safe: the app's vector-table page is erased FIRST and rewritten LAST (its first word
//     dead-last), so an interrupted apply leaves 0xFFFFFFFF at the app entry -- the Adafruit bootloader's
//     validity check then keeps the board in the UF2BOOT mass-storage bootloader instead of jumping to junk.
//   - The watchdog is force-started before the copy, so even a crash mid-apply resets into that same
//     safe app-less state rather than hanging forever.
//   - The applier is idempotent: a reset after the copy but before the meta clear just re-applies the same
//     verified image.
// The bootloader, MBR/SoftDevice, and the LittleFS config/bond region are never written (one exception: the
// bootloader SETTINGS page is rewritten to the same "valid app, no CRC" state drag-and-drop flashing leaves,
// because a stale CRC recorded by a previous adafruit-nrfutil flash would otherwise fail the new image at
// the bootloader's boot check).
#pragma once
#include <stdbool.h>
#include <stdint.h>

// ACK status codes, echoed to the panel in the 0xAB frame ([0xAB][5][status][nextOff u32 LE]).
enum {
	FWUP_OK = 0,
	FWUP_ERR_STATE =
		1, // command out of sequence (no BEGIN, or END before all bytes)
	FWUP_ERR_BOUNDS =
		2, // image too big / would collide with the running app or config region
	FWUP_ERR_OFFSET =
		3, // chunk offset mismatch (ack carries the expected offset -- resync and resend)
	FWUP_ERR_CRC = 4, // staged bytes don't match the announced CRC32
	FWUP_ERR_VECTOR =
		5, // staged image doesn't start with a plausible Cortex-M vector table
};

uint8_t fwupBegin(
	uint32_t size,
	uint32_t crc32); // 0x20: announce image; disarms any previous staged update
uint8_t fwupChunk(uint32_t off, const uint8_t *d,
		  uint8_t len); // 0x21: sequential data (len%4==0, <=128)
uint8_t
fwupEnd(void); // 0x22: verify staged CRC32 + vectors, commit the meta page (ARMS the update)
void fwupAbort(
	void); // 0x24: disarm + drop any transfer state (staged bytes become inert garbage)
uint32_t fwupNextOff(
	void); // next expected chunk offset (returned in every ack for panel resync)

// Apply a committed staged update. Call FIRST in setup(): if a valid meta page is present this never returns
// (copies staged->app from RAM and resets). Invalid/stale meta is erased and ignored.
void fwupApplyIfArmed(void);

// Full-board wipe (debug-only "erase everything", 0x25). Unlike a factory reset -- which reformats LittleFS to
// clean defaults but keeps the firmware -- this ERASES the entire app region, the LittleFS config/bond region,
// and the bootloader settings page, leaving the board app-less. On every subsequent boot the Adafruit UF2
// bootloader finds no valid app and stays in UF2 mass-storage (drag-and-drop) mode until firmware is flashed
// again, so no trace of OpenPuck remains. The MBR/SoftDevice/bootloader are untouched (the board still mounts
// as the UF2 drive). fwupArmFullWipe() stamps a marker + the caller reboots; fwupWipeIfArmed() does the erase
// from RAM at the next boot (call BEFORE fwupApplyIfArmed) with the same power-cut-safe ordering as the applier.
void fwupArmFullWipe(void);
void fwupWipeIfArmed(void);
