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
// Low 32 bits preserve the complete native Triton button word. Virtual
// analog-direction triggers live above it and cannot alias physical buttons.
#define LZ_BTN_LSTICK_RT (1ULL << 32) // lx >  12000
#define LZ_BTN_LSTICK_LF (1ULL << 33) // lx < -12000
#define LZ_BTN_LSTICK_DN (1ULL << 34) // ly < -12000
#define LZ_BTN_LSTICK_UP (1ULL << 35) // ly >  12000
#define LZ_BTN_RSTICK_RT (1ULL << 36) // rx >  12000
#define LZ_BTN_RSTICK_LF (1ULL << 37) // rx < -12000
#define LZ_BTN_RSTICK_DN (1ULL << 38) // ry < -12000
#define LZ_BTN_RSTICK_UP (1ULL << 39) // ry >  12000
// ---- binding v2 (24 bytes) ----
// trigMask: any matching bit activates the binding.
// holdMask: all matching bits must also be held.
// MOUSE_AXIS and SCROLL bindings ignore trigMask/holdMask; the analog source drives them.
struct LizardBinding {
	uint8_t outType; // LZ_OUT_*
	uint8_t outData[7]; // type-specific payload (see constants above)
	uint64_t trigMask; // physical/virtual button bits: any-of (OR)
	uint64_t holdMask; // physical/virtual button bits: all-of (AND guard)
};
static_assert(sizeof(LizardBinding) == 24, "LizardBinding v2 layout changed");
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
