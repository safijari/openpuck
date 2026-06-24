// ctrl_bonds.h -- the emulated controller's bonds to pucks.
//
// A real Steam Controller bonds to up to two pucks under the Zephyr settings keys `esb/bond` (slot 0)
// and `esb/bond_2` (slot 1). We support MORE: an array of slots, each holding the 24-byte bond record
//   rec = [proteus_uuid 4 LE][ibex_uuid 4 LE][puck_serial 16]
// -- the SAME 24-byte shape Steam writes to the puck via 0xA2, except the trailing serial is the
// PUCK's (the peer), not the controller's. The 8-byte uuid key is identical on both sides and is what
// matches the puck's E1 host frame at reconnect. Pairing a new puck find-or-allocates a free slot by
// uuid (so an arbitrary number can be bonded); the Deck GUI can clear a slot. Persists to LittleFS
// `/ctrlbond.bin`.
#pragma once
#include <stdint.h>

#define NBOND 8

struct CtrlBond {
	uint8_t rec[24]; // [proteus_uuid 4][ibex_uuid 4][puck_serial 16]
	bool used;
	// feature-report reply staging (filled in ISR, read on GET)
	uint8_t resp[63];
	uint16_t resp_len;
};
extern CtrlBond g_bond[NBOND];
extern volatile bool g_bondDirty; // bonds changed -> flush to flash from loop()

bool ctrlRecEmpty(const uint8_t *r);
void loadCtrlBonds();
void saveCtrlBonds();
// Find a bond slot matching a puck E1 host frame's [proteus_uuid 4][ibex_uuid 4]; -1 if none.
int ctrlBondMatch(const uint8_t *proteus_uuid, const uint8_t *ibex_uuid);
// Slot for a 24-byte record: reuse the slot whose uuid (rec[0..8]) matches, else first free; -1 if full.
int ctrlBondFindOrAlloc(const uint8_t *rec24);
// Clear (un-bond) a slot and mark the store dirty. Out-of-range slots are ignored.
void ctrlBondClear(int slot);
