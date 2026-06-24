#include "ctrl_bonds.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <string.h>
using namespace Adafruit_LittleFS_Namespace;

CtrlBond g_bond[NBOND];
volatile bool g_bondDirty = false;

#define CTRLBOND_FILE "/ctrlbond.bin"

bool ctrlRecEmpty(const uint8_t *r)
{
	for (int i = 0; i < 24; i++)
		if (r[i])
			return false;
	return true;
}

void saveCtrlBonds()
{
	InternalFS.remove(CTRLBOND_FILE);
	File f(InternalFS);
	if (f.open(CTRLBOND_FILE, FILE_O_WRITE)) {
		for (int i = 0; i < NBOND; i++) {
			uint8_t u = g_bond[i].used ? 1 : 0;
			f.write(&u, 1);
			f.write(g_bond[i].rec, 24);
		}
		f.close();
	}
}

void loadCtrlBonds()
{
	File f(InternalFS);
	if (f.open(CTRLBOND_FILE, FILE_O_READ))
		for (int i = 0; i < NBOND; i++) {
			uint8_t u = 0;
			if (f.read(&u, 1) == 1) {
				g_bond[i].used = u;
				f.read(g_bond[i].rec, 24);
			}
		}
	f.close();
}

int ctrlBondMatch(const uint8_t *proteus_uuid, const uint8_t *ibex_uuid)
{
	for (int i = 0; i < NBOND; i++) {
		if (!g_bond[i].used)
			continue;
		if (memcmp(g_bond[i].rec + 0, proteus_uuid, 4) == 0 &&
		    memcmp(g_bond[i].rec + 4, ibex_uuid, 4) == 0)
			return i;
	}
	return -1;
}

int ctrlBondFindOrAlloc(const uint8_t *rec24)
{
	// re-writing a known puck (same 8-byte uuid key) updates its slot in place
	for (int i = 0; i < NBOND; i++)
		if (g_bond[i].used && memcmp(g_bond[i].rec, rec24, 8) == 0)
			return i;
	// otherwise take the first free slot
	for (int i = 0; i < NBOND; i++)
		if (!g_bond[i].used)
			return i;
	return -1; // bond table full
}

void ctrlBondClear(int slot)
{
	if (slot < 0 || slot >= NBOND)
		return;
	g_bond[slot].used = false;
	memset(g_bond[slot].rec, 0, 24);
	g_bondDirty = true;
}
