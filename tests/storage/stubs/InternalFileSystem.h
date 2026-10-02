#pragma once
#include "Adafruit_LittleFS.h"
class InternalFileSystem : public Adafruit_LittleFS {
    public:
	bool begin()
	{
		++formats;
		return mountOk = true;
	}
	bool remove(const char *p)
	{
		files.erase(p);
		return true;
	}
	bool rename(const char *a, const char *b)
	{
		if (failRename)
			return false;
		files[b] = files[a];
		files.erase(a);
		return true;
	}
};
extern InternalFileSystem InternalFS;
extern uint32_t testFlash[7168];
