#include "storage.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <string.h>
using namespace Adafruit_LittleFS_Namespace;

uint8_t g_storageState;

bool storageBegin()
{
	// InternalFileSystem::begin erases on any mount failure. Only blank
	// flash may be initialized automatically; failed mounts retain data.
	if (InternalFS.Adafruit_LittleFS::begin()) {
		g_storageState = 1;
		return true;
	}
	const volatile uint32_t *flash =
		reinterpret_cast<const volatile uint32_t *>(0xED000);
	for (size_t i = 0; i < (7 * 4096) / sizeof(uint32_t); i++) {
		if (flash[i] != 0xFFFFFFFF) {
			g_storageState = 0;
			return false;
		}
	}
	bool mounted = InternalFS.begin();
	g_storageState = mounted ? 2 : 0;
	return mounted;
}

bool storageWriteFile(const char *path, const char *temporary,
		      const uint8_t *data, size_t length)
{
	if (g_storageState == 0)
		return false;
	InternalFS.remove(temporary);
	File file(InternalFS);
	bool complete = file.open(temporary, FILE_O_WRITE);
	if (complete)
		complete = file.write(data, length) == length;
	file.close();
	if (complete) {
		complete = file.open(temporary, FILE_O_READ);
		if (complete)
			complete = file.size() == length;
		uint8_t check[32];
		size_t offset = 0;
		while (complete && offset < length) {
			size_t n = length - offset;
			if (n > sizeof check)
				n = sizeof check;
			complete = file.read(check, n) == (int)n &&
				   memcmp(check, data + offset, n) == 0;
			offset += n;
		}
		file.close();
	}
	// LittleFS rename atomically replaces the old file after verification.
	if (complete)
		complete = InternalFS.rename(temporary, path);
	if (!complete) {
		g_storageState = 3;
		return false;
	}
	g_storageState = 1;
	return true;
}
