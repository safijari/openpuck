#include "triton.h"

PuckInput g_in[NSLOT];

void imuFrom45(const uint8_t *r, int16_t *ax, int16_t *ay, int16_t *az,
	       int16_t *gx, int16_t *gy, int16_t *gz)
{
	*ax = s16off(r, 32);
	*ay = s16off(r, 34);
	*az = s16off(r, 36);
	*gx = s16off(r, 38);
	*gy = s16off(r, 40);
	*gz = s16off(r, 42);
}

static inline void put16(uint8_t *r, int off, int v)
{
	r[2 + off] = (uint8_t)(v & 0xFF);
	r[2 + off + 1] = (uint8_t)((v >> 8) & 0xFF);
}

// Inverse of the rf_link 0x45 decode: re-serialize a PuckInput as a Triton input report so non-RF input
// sources (BLE controllers) can feed the SAME g_active->onReport45() dispatch the RF path uses -- the push
// modes (XInput's rfXboxGamepad, lizard) parse raw report bytes, not g_in. Field offsets mirror the decode
// exactly (rf_link.cpp / imuFrom45): buttons u32 @rep[2], triggers u16 @+4/+6 (trigU8 is >>7, so <<7 here),
// sticks @+8..+15, trackpads @+16/+18 and +22/+24, IMU @rep[34..45]. Bytes the decode never reads stay zero.
// Returns the body length (PUCK45_LEN, sized so the tlen>=46 IMU gate in the decode path semantics holds).
uint8_t puckSynth45(const PuckInput *in, uint8_t seq, uint8_t out[PUCK45_LEN])
{
	for (int i = 0; i < PUCK45_LEN; i++)
		out[i] = 0;
	out[0] = 0x45;
	out[1] = seq;
	out[2] = (uint8_t)(in->buttons & 0xFF);
	out[3] = (uint8_t)((in->buttons >> 8) & 0xFF);
	out[4] = (uint8_t)((in->buttons >> 16) & 0xFF);
	out[5] = (uint8_t)((in->buttons >> 24) & 0xFF);
	put16(out, 4, (int)in->lt << 7);
	put16(out, 6, (int)in->rt << 7);
	put16(out, 8, in->lx);
	put16(out, 10, in->ly);
	put16(out, 12, in->rx);
	put16(out, 14, in->ry);
	put16(out, 16, in->lpx);
	put16(out, 18, in->lpy);
	put16(out, 22, in->rpx);
	put16(out, 24, in->rpy);
	put16(out, 32, in->ax);
	put16(out, 34, in->ay);
	put16(out, 36, in->az);
	put16(out, 38, in->gx);
	put16(out, 40, in->gy);
	put16(out, 42, in->gz);
	return PUCK45_LEN;
}
