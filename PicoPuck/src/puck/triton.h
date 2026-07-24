// triton.h — the Steam Controller 2's native input model, shared by every path.
//
// Bluetooth drivers decode a controller into one PuckInput per slot; the puck
// personality serialises it back into an SC2 report 0x45 (puck_synth45) for the
// host, or — for a real SC2 — forwards the on-air 0x45 verbatim. Ported from
// OpenPuck/triton.h (button masks and field layout are the SC2's).
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_TRITON_H
#define PICOPUCK_TRITON_H

#include <stdint.h>
#include "config/picopuck_config.h"

// Triton button masks (the 32-bit field at report 0x45 bytes [2..5]).
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
#define TB_LPADT 0x2000000u
#define TB_LPADC 0x4000000u
#define TB_R2 0x800000u
#define TB_L2 0x8000000u

// Decoded controller input, one per slot.
typedef struct {
	uint32_t buttons;             // TB_* bits
	int16_t lx, ly, rx, ry;       // sticks, center 0
	uint8_t lt, rt;               // triggers 0..255
	int16_t lpx, lpy, rpx, rpy;   // trackpads (SC2 only; 0 for generic pads)
	int16_t ax, ay, az;           // accelerometer (0 if none)
	int16_t gx, gy, gz;           // gyroscope (0 if none)
} puck_input_t;

extern puck_input_t g_in[PP_NSLOT];

// Length of a full SC2 report 0x45 (report id + 45 body bytes).
#define PUCK45_LEN 46

// Serialise g_in-style input into a report 0x45 buffer (out[0]=0x45). Trackpads
// and IMU are written from the struct (zero for generic pads → neutral). The µs
// timestamp Steam uses for gyro dt is stamped from `usec`. Returns PUCK45_LEN.
uint8_t puck_synth45(const puck_input_t *in, uint8_t seq, uint32_t usec,
		     uint8_t out[PUCK45_LEN]);

// 8-way HID hat (0 idle, 1..8 clockwise from N) → TB_ dpad bits.
uint32_t triton_hat_bits(uint8_t hat);

#endif // PICOPUCK_TRITON_H
