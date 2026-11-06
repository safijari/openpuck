#include "pad_dpad.h"
#include "config.h"
#include "triton.h"
#include <stdlib.h>

// Centre zone with no direction: about a quarter of the pad's radius.
#define DPAD_DEAD 9000
// tan(22.5 deg) as 53/128: an axis counts once it's within 67.5 deg of the press direction, giving eight
// equal 45 deg sectors (diagonals press both directions).
#define DPAD_TAN_NUM 53
#define DPAD_TAN_SHIFT 7

static uint32_t dirBits(int16_t x, int16_t y)
{
	const int32_t ax = abs((int32_t)x), ay = abs((int32_t)y);
	if (ax < DPAD_DEAD && ay < DPAD_DEAD)
		return 0;
	uint32_t b = 0;
	// pads share the sticks' convention: +y is up
	if (ay > ((ax * DPAD_TAN_NUM) >> DPAD_TAN_SHIFT))
		b |= y > 0 ? TB_DUP : TB_DDN;
	if (ax > ((ay * DPAD_TAN_NUM) >> DPAD_TAN_SHIFT))
		b |= x > 0 ? TB_DRT : TB_DLF;
	return b;
}

void padDpadApply(uint8_t slot, uint8_t *rep)
{
	if (slot >= NSLOT || g_etype >= ET_COUNT)
		return;
	PuckInput &in = g_in[slot];
	// All four paddles + a D-pad direction is a mode-switch chord: a pad click must never complete one.
	if ((in.buttons & CHORD_BACK4) == CHORD_BACK4)
		return;
	static const uint32_t CLICK[2] = { TB_LPADC, TB_RPADC };
	const int16_t px[2] = { in.lpx, in.rpx }, py[2] = { in.lpy, in.rpy };
	uint32_t add = 0, consume = 0;
	for (int pad = 0; pad < 2; pad++) {
		if (g_padStick[pad] != PS_DPAD || !(in.buttons & CLICK[pad]))
			continue;
		consume |= CLICK[pad];
		add |= dirBits(px[pad], py[pad]);
	}
	if (!consume)
		return;
	in.buttons = (in.buttons & ~consume) | add;
	uint32_t raw = (uint32_t)rep[2] | (uint32_t)rep[3] << 8 |
		       (uint32_t)rep[4] << 16 | (uint32_t)rep[5] << 24;
	raw = (raw & ~consume) | add;
	for (int i = 0; i < 4; i++)
		rep[2 + i] = (uint8_t)(raw >> (8 * i));
}
