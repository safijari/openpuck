#include "lizard_map.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <string.h>
#include <class/hid/hid.h>
using namespace Adafruit_LittleFS_Namespace;

LizardMap g_lizardMap;

#define LZ_FILE    "/lizard_map.bin"
#define LZ_MAGIC   0xB1u
#define LZ_VERSION 1u

// KB modifier bits (from TinyUSB hid.h)
#define KM_LCTRL  0x01u
#define KM_LSHIFT 0x02u
#define KM_LALT   0x04u
#define KM_LGUI   0x08u

// Helper: append a binding to g_lizardMap
static void addBind(uint8_t type, const uint8_t *od7, uint32_t trig,
		    uint32_t hold)
{
	if (g_lizardMap.count >= LZ_MAX_BINDINGS)
		return;
	LizardBinding &b = g_lizardMap.bindings[g_lizardMap.count++];
	b.outType = type;
	for (int i = 0; i < 7; i++)
		b.outData[i] = od7[i];
	b.trigMask = trig;
	b.holdMask = hold;
}

static inline void addAxis(uint8_t src, uint8_t gyroAct)
{
	uint8_t d[7] = { src, gyroAct, 0, 0, 0, 0, 0 };
	addBind(LZ_OUT_MOUSE_AXIS, d, 0, 0);
}
static inline void addScroll(uint8_t src)
{
	uint8_t d[7] = { src, 0, 0, 0, 0, 0, 0 };
	addBind(LZ_OUT_SCROLL, d, 0, 0);
}
static inline void addMouseBtn(uint8_t btn, uint32_t trig, uint32_t hold)
{
	uint8_t d[7] = { btn, 0, 0, 0, 0, 0, 0 };
	addBind(LZ_OUT_MOUSE_BTN, d, trig, hold);
}
static inline void addKey(uint8_t mod, uint8_t k0, uint32_t trig,
			  uint32_t hold)
{
	uint8_t d[7] = { mod, k0, 0, 0, 0, 0, 0 };
	addBind(LZ_OUT_KBD_CHORD, d, trig, hold);
}
static inline void addKey3(uint8_t mod, uint8_t k0, uint8_t k1, uint8_t k2,
			   uint32_t trig, uint32_t hold)
{
	uint8_t d[7] = { mod, k0, k1, k2, 0, 0, 0 };
	addBind(LZ_OUT_KBD_CHORD, d, trig, hold);
}
static inline void addConsumer(uint8_t bits, uint32_t trig, uint32_t hold)
{
	uint8_t d[7] = { bits, 0, 0, 0, 0, 0, 0 };
	addBind(LZ_OUT_CONSUMER, d, trig, hold);
}

void defaultLizardMap()
{
	g_lizardMap.count = 0;

	// Analog sources (always active; no trigger mask)
	addAxis(LZ_MSRC_RPAD, LZ_GYRO_ALWAYS); // right pad → mouse
	addScroll(LZ_MSRC_LPAD); // left pad → scroll

	// Mouse buttons
	// right pad click → left mouse
	addMouseBtn(1, 0x400000u /*TB_RPADC*/, 0);
	// right trigger (digital click) → left mouse
	addMouseBtn(1, 0x800000u /*TB_R2*/, 0);
	// left trigger → right mouse
	addMouseBtn(2, 0x8000000u /*TB_L2*/, 0);
	// left pad click → middle mouse
	addMouseBtn(4, 0x4000000u /*TB_LPADC*/, 0);

	// Hold-modifier shortcuts (Steam-button combos: place BEFORE generic X/L4 so they consume first)
	// Steam+L4 → Ctrl+Alt+Delete
	addKey3(KM_LCTRL | KM_LALT, HID_KEY_DELETE, 0, 0, 0x20000u /*TB_L4*/,
		0x10000u /*TB_STEAM*/);
	// Steam+X → Win+Ctrl+O (on-screen keyboard)
	addKey3(KM_LGUI | KM_LCTRL, HID_KEY_O, 0, 0, 0x4u /*TB_X*/,
		0x10000u /*TB_STEAM*/);
	// Steam+L5 → volume down (consumer bit1=0x02)
	addConsumer(0x02, 0x40000u /*TB_L5*/, 0x10000u /*TB_STEAM*/);
	// Steam+R5 → volume up (consumer bit0=0x01)
	addConsumer(0x01, 0x100u /*TB_R5*/, 0x10000u /*TB_STEAM*/);

	// Keyboard keys
	addKey(0, HID_KEY_ENTER, 0x1u /*TB_A*/, 0);
	addKey(0, HID_KEY_ESCAPE, 0x2u /*TB_B*/, 0);
	addKey(0, HID_KEY_PAGE_UP, 0x4u /*TB_X*/, 0);
	addKey(0, HID_KEY_PAGE_DOWN, 0x8u /*TB_Y*/, 0);
	addKey(0, HID_KEY_TAB, 0x40u /*TB_VIEW*/, 0);
	addKey(0, HID_KEY_ESCAPE, 0x4000u /*TB_MENU*/, 0);

	// D-pad → arrow keys
	addKey(0, HID_KEY_ARROW_UP, 0x2000u /*TB_DUP*/, 0);
	addKey(0, HID_KEY_ARROW_DOWN, 0x400u /*TB_DDN*/, 0);
	addKey(0, HID_KEY_ARROW_LEFT, 0x1000u /*TB_DLF*/, 0);
	addKey(0, HID_KEY_ARROW_RIGHT, 0x800u /*TB_DRT*/, 0);

	// Left stick → arrow keys (virtual deflection bits)
	addKey(0, HID_KEY_ARROW_UP, LZ_BTN_LSTICK_UP, 0);
	addKey(0, HID_KEY_ARROW_DOWN, LZ_BTN_LSTICK_DN, 0);
	addKey(0, HID_KEY_ARROW_LEFT, LZ_BTN_LSTICK_LF, 0);
	addKey(0, HID_KEY_ARROW_RIGHT, LZ_BTN_LSTICK_RT, 0);

	// Shoulder buttons → modifier keys only (no keycode)
	addKey(KM_LCTRL, 0, 0x80000u /*TB_LB*/, 0);
	addKey(KM_LALT, 0, 0x200u /*TB_RB*/, 0);
}

void saveLizardMap()
{
	InternalFS.remove(LZ_FILE);
	File f(InternalFS);
	if (!f.open(LZ_FILE, FILE_O_WRITE))
		return;
	uint8_t hdr[3] = { LZ_MAGIC, LZ_VERSION, g_lizardMap.count };
	f.write(hdr, 3);
	f.write((uint8_t *)g_lizardMap.bindings,
		g_lizardMap.count * sizeof(LizardBinding));
	f.close();
}

void loadLizardMap()
{
	File f(InternalFS);
	if (f.open(LZ_FILE, FILE_O_READ)) {
		uint8_t hdr[3] = { 0, 0, 0 };
		if (f.read(hdr, 3) == 3 && hdr[0] == LZ_MAGIC &&
		    hdr[1] == LZ_VERSION && hdr[2] <= LZ_MAX_BINDINGS) {
			uint8_t cnt = hdr[2];
			g_lizardMap.count = 0;
			int got = f.read((uint8_t *)g_lizardMap.bindings,
					 cnt * sizeof(LizardBinding));
			if (got == (int)(cnt * sizeof(LizardBinding)))
				g_lizardMap.count = cnt;
		}
		f.close();
	}
	// If nothing loaded, install + persist defaults
	if (g_lizardMap.count == 0) {
		defaultLizardMap();
		saveLizardMap();
	}
}
