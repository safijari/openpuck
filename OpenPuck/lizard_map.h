// lizard_map.h -- configurable input→keyboard/mouse/consumer binding table for lizard (desktop)
// mode.
//
// g_lizardMap replaces the hardcoded button→key map in mode_lizard.cpp with a runtime-configurable
// list of LizardBinding entries. Each binding maps a trigger condition (button bitmask + optional
// hold-modifier bitmask) to a single output action. rfLizard() walks the table every input frame.
//
// Designed for forward compatibility: the output union can gain a GAMEPAD variant to remap emulated-
// controller buttons without changing the trigger model, storage format, or WebUSB protocol.
#pragma once
#include <stdint.h>

// ---- output type (LizardBinding::outType) ----
// 0 = disabled; skip this entry.
#define LZ_OUT_NONE 0u
// Keyboard chord: outData[0]=modifier bits, outData[1..6]=HID keycodes (up to 6, zero=unused).
#define LZ_OUT_KBD_CHORD 1u
// Mouse button: outData[0]=button bitmask (bit0=left, bit1=right, bit2=middle).
#define LZ_OUT_MOUSE_BTN 2u
// Mouse axis: outData[0]=source (LZ_MSRC_*), outData[1]=gyro activation (LZ_GYRO_*).
#define LZ_OUT_MOUSE_AXIS 3u
// Scroll wheel from an analog source: outData[0]=source (LZ_MSRC_*).
#define LZ_OUT_SCROLL 4u
// Consumer control (edge-triggered, MODE_LIZARD only): outData[0]=report bitmask (bit0=vol+, bit1=vol-).
#define LZ_OUT_CONSUMER 5u

// ---- analog source (outData[0] for MOUSE_AXIS and SCROLL) ----
#define LZ_MSRC_RPAD 0u // right trackpad (MOUSE_AXIS; touch-gated by TB_RPADT)
#define LZ_MSRC_LSTICK 1u // left analog stick (MOUSE_AXIS)
#define LZ_MSRC_GYRO \
	2u // gyroscope (MOUSE_AXIS; activation-gated, see LZ_GYRO_*)
#define LZ_MSRC_LPAD 0u // left trackpad (SCROLL; touch-gated by TB_LPADT)

// ---- gyro activation condition (outData[1] when src=LZ_MSRC_GYRO) ----
#define LZ_GYRO_ALWAYS 0u // gyro always drives mouse
#define LZ_GYRO_RPAD 1u // only when right trackpad is touched
#define LZ_GYRO_LSTICK 2u // only when left stick is deflected past threshold
// only when all bits in LizardBinding::holdMask are held
#define LZ_GYRO_BTN 3u

// ---- virtual stick-deflection trigger bits (usable in trigMask / holdMask) ----
// These reuse bits 28..31 of the button word, which the SC2 *does* use in its 0x45 report (bit 28 =
// right grip touch, bit 29 = left grip touch; PROTOCOL.md §8). To avoid grip-touch firing these,
// lizardButtons() masks the controller's top 4 bits off before OR-ing in the deflection flags below,
// so in MODE_LIZARD these bits mean ONLY stick deflection (grips are not a bindable lizard input).
#define LZ_BTN_LSTICK_RT 0x10000000u // lx >  12000
#define LZ_BTN_LSTICK_LF 0x20000000u // lx < -12000
#define LZ_BTN_LSTICK_DN 0x40000000u // ly < -12000
#define LZ_BTN_LSTICK_UP 0x80000000u // ly >  12000

// ---- binding (16 bytes, naturally 4-byte aligned) ----
// trigMask: (buttons & trigMask) != 0 activates the binding.
// holdMask: ALL bits must also be set at the same time (AND-guard).
// MOUSE_AXIS and SCROLL bindings ignore trigMask/holdMask; the analog source drives them.
struct LizardBinding {
	uint8_t outType; // LZ_OUT_*
	uint8_t outData[7]; // type-specific payload (see constants above)
	uint32_t trigMask; // physical/virtual button bits: any-of (OR)
	uint32_t holdMask; // physical/virtual button bits: all-of (AND guard)
};

#define LZ_MAX_BINDINGS 32u

struct LizardMap {
	uint8_t count;
	LizardBinding bindings[LZ_MAX_BINDINGS];
};

extern LizardMap g_lizardMap;

// fill g_lizardMap with defaults (does NOT save to flash)
void defaultLizardMap();
void loadLizardMap();
void saveLizardMap();
