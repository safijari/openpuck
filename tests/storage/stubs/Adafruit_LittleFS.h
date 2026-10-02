#pragma once
#include <stdint.h>
#include <stddef.h>
#include <map>
#include <string>
#include <vector>
#include <cstring>
extern bool mountOk, failWrite, failRead, failRename;
extern int formats;
extern std::map<std::string, std::vector<uint8_t> > files;
class Adafruit_LittleFS {
    public:
	bool begin()
	{
		return mountOk;
	}
};
#define FILE_O_WRITE 1
#define FILE_O_READ 2
namespace Adafruit_LittleFS_Namespace
{
class File {
	std::string name;
	size_t pos = 0;

    public:
	template <class T> File(T &)
	{
	}
	bool open(const char *p, int mode)
	{
		name = p;
		pos = 0;
		if (mode == FILE_O_WRITE)
			files[name] = {};
		return files.count(name);
	}
	size_t write(const uint8_t *p, size_t n)
	{
		size_t used = failWrite ? n / 2 : n;
		files[name].assign(p, p + used);
		return used;
	}
	int read(uint8_t *p, size_t n)
	{
		auto &d = files[name];
		if (pos + n > d.size())
			return 0;
		memcpy(p, d.data() + pos, n);
		pos += n;
		if (failRead)
			p[0] ^= 1;
		return n;
	}
	size_t size()
	{
		return files[name].size();
	}
	void close()
	{
	}
};
}
