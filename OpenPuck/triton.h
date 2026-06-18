// triton.h -- the controller's native input as it comes off the RF link, and the decoders for it.
//
// "Triton" is the Steam Controller 2 controller. Its HID input report 0x45 is what the puck relays; rf_link.cpp
// decodes one into the shared g_in struct each fresh frame, and every USB personality reads g_in to build its
// own host report. report 0x45 layout:
//   [0]=0x45 [1]=seq [2..5]=buttons u32; analog offsets below are from the buttons low byte (rep[2]).
#pragma once
#include <stdint.h>

// ---- Triton button masks (the 32-bit field at rep[2..5]) ----
#define TB_A 0x1u
#define TB_B 0x2u
#define TB_X 0x4u
#define TB_Y 0x8u
#define TB_QAM 0x10u
#define TB_R3 0x20u
#define TB_VIEW 0x40u
#define TB_R4 0x80u
#define TB_R5 0x100u
#define TB_RB 0x200u
#define TB_DDN 0x400u
#define TB_DRT 0x800u
#define TB_DLF 0x1000u
#define TB_DUP 0x2000u
#define TB_MENU 0x4000u
#define TB_L3 0x8000u
#define TB_STEAM 0x10000u
#define TB_L4 0x20000u
#define TB_L5 0x40000u
#define TB_LB 0x80000u
#define TB_RPADT 0x200000u
#define TB_RPADC 0x400000u

// virtual: PS Touch Click (back-paddle/QAM target, not a real Triton button)
#define TB_TOUCH 0x100000u

// virtual: PS5 Mute button (back-paddle/QAM target, not a real Triton button)
#define TB_MUTE 0x1000000u
#define TB_LPADT 0x2000000u
#define TB_LPADC 0x4000000u

// all four back paddles held -> mode-switch chord guard
#define CHORD_BACK4 (TB_R4 | TB_L4 | TB_R5 | TB_L5)

// analog-trigger fraction (of 0xFF) at which digital ZL/ZR (Switch) etc. trip
#define SW_TRIG_ON 40

// ---- report 0x45 field decoders (offsets relative to rep[2], the buttons low byte) ----
static inline int s16off(const uint8_t *r, int off)
{
	int v = r[2 + off] | (r[2 + off + 1] << 8);
	return (v & 0x8000) ? v - 0x10000 : v;
}
static inline int u16off(const uint8_t *r, int off)
{
	return r[2 + off] | (r[2 + off + 1] << 8);
}
// Controller trigger u16 tops out near half-scale (~0x8000) at a full pull, so a straight >>8 reads only ~0x80
// (host sees a half-pressed trigger). Scale x2 (>>7) and saturate so a full pull maps to 0xFF.
static inline uint8_t trigU8(int u16v)
{
	int v = u16v >> 7;
	return (uint8_t)(v > 255 ? 255 : v);
}
static inline uint32_t btnsOf(const uint8_t *r)
{
	return (uint32_t)r[2] | ((uint32_t)r[3] << 8) | ((uint32_t)r[4] << 16) |
	       ((uint32_t)r[5] << 24);
}
// report 0x45 IMU offsets (PROTOCOL.md §8): accel @0x22, gyro @0x28 from report start.
void imuFrom45(const uint8_t *r, int16_t *ax, int16_t *ay, int16_t *az,
	       int16_t *gx, int16_t *gy, int16_t *gz);

// ---- shared decoded input (filled by rf_link.cpp once per fresh report 0x45, read by every mode) ----
struct PuckInput {
	// raw Triton buttons (TB_*); per-mode builders apply their own chord masking
	uint32_t buttons;
	int16_t lx, ly, rx, ry; // sticks (int16, center 0)
	uint8_t lt, rt; // triggers scaled 0..255 (trigU8)
	int16_t lpx, lpy, rpx, rpy; // left / right trackpad coords (int16)
	int16_t ax, ay, az; // accelerometer
	int16_t gx, gy, gz; // gyroscope
};
extern PuckInput g_in;
