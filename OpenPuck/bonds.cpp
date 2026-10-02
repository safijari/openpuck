#include "bonds.h"
#include "storage.h"
#include <string.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

Slot g_slot[NSLOT];
unsigned long g_connReplyMs[NSLOT] = { 0 };
volatile bool g_dirty = false;
bool g_pairing = false;

#define BOND_FILE "/bonds.bin"

bool recEmpty(const uint8_t *r)
{
	for (int i = 0; i < 24; i++)
		if (r[i])
			return false;
	return true;
}

void saveBonds()
{
	uint8_t data[NSLOT * 25];
	for (int i = 0; i < NSLOT; i++) {
		data[i * 25] = g_slot[i].used ? 1 : 0;
		memcpy(data + i * 25 + 1, g_slot[i].rec, 24);
	}
	storageWriteFile(BOND_FILE, "/bonds.tmp", data, sizeof data);
}

void loadBonds()
{
	if (g_storageState == 0)
		return;
	File f(InternalFS);
	if (f.open(BOND_FILE, FILE_O_READ))
		for (int i = 0; i < NSLOT; i++) {
			uint8_t u = 0;
			if (f.read(&u, 1) == 1) {
				g_slot[i].used = u;
				f.read(g_slot[i].rec, 24);
			}
		}
	f.close();
}

int bondedSlotCount()
{
	int n = 0;
	for (int i = 0; i < NSLOT; i++)
		if (g_slot[i].used)
			n++;
	return n;
}
