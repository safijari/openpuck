#pragma once
#include <stddef.h>
#include <stdint.h>

// 0 = unavailable, 1 = mounted, 2 = initialized blank flash, 3 = save failed.
extern uint8_t g_storageState;
bool storageBegin();
bool storageWriteFile(const char *path, const char *temporary,
		      const uint8_t *data, size_t length);
