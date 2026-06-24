#include "report45.h"
#include "triton.h"
#include <Arduino.h>
#include <string.h>

// Write a signed 16-bit value little-endian at rep[2+off] (the same indexing triton.h's s16off reads).
static inline void put_s16(uint8_t *rep, int off, int16_t v)
{
	rep[2 + off] = (uint8_t)(v & 0xFF);
	rep[2 + off + 1] = (uint8_t)((v >> 8) & 0xFF);
}
// Write an unsigned 16-bit value little-endian at rep[2+off] (matches u16off).
static inline void put_u16(uint8_t *rep, int off, uint16_t v)
{
	rep[2 + off] = (uint8_t)(v & 0xFF);
	rep[2 + off + 1] = (uint8_t)((v >> 8) & 0xFF);
}
// Write an unsigned 32-bit value little-endian at rep[2+off].
static inline void put_u32(uint8_t *rep, int off, uint32_t v)
{
	rep[2 + off + 0] = (uint8_t)(v & 0xFF);
	rep[2 + off + 1] = (uint8_t)((v >> 8) & 0xFF);
	rep[2 + off + 2] = (uint8_t)((v >> 16) & 0xFF);
	rep[2 + off + 3] = (uint8_t)((v >> 24) & 0xFF);
}

void buildReport45(uint8_t *out, uint8_t seq)
{
	memset(out, 0, REPORT45_LEN);
	out[0] = 0x45;
	out[1] = seq;

	// buttons u32 at rep[2..5] (inverse of btnsOf)
	out[2] = (uint8_t)(g_in.buttons & 0xFF);
	out[3] = (uint8_t)((g_in.buttons >> 8) & 0xFF);
	out[4] = (uint8_t)((g_in.buttons >> 16) & 0xFF);
	out[5] = (uint8_t)((g_in.buttons >> 24) & 0xFF);

	// triggers: the controller's analog trigger is a u16 that trigU8 maps via >>7 (saturating). Inverse:
	// u16 = lt << 7 (so a full 0xFF pull -> 0x7F80 -> trigU8 -> 0xFF). offset 4 = LT, offset 6 = RT.
	put_u16(out, 4, (uint16_t)(g_in.lt << 7));
	put_u16(out, 6, (uint16_t)(g_in.rt << 7));

	// sticks (int16, center 0): lx@8 ly@10 rx@12 ry@14
	put_s16(out, 8, g_in.lx);
	put_s16(out, 10, g_in.ly);
	put_s16(out, 12, g_in.rx);
	put_s16(out, 14, g_in.ry);

	// trackpads: LPad x@16 y@18 pressure@20, RPad x@22 y@24 pressure@26 (per PROTOCOL.md report 0x45)
	put_s16(out, 16, g_in.lpx);
	put_s16(out, 18, g_in.lpy);
	put_u16(out, 20, g_in.lpp);
	put_s16(out, 22, g_in.rpx);
	put_s16(out, 24, g_in.rpy);
	put_u16(out, 26, g_in.rpp);

	// IMU timestamp: report[30..33] = free-running MICROSECOND counter (u32 LE), per IBEX_REIMPL_SPEC
	// report-0x45 layout. CRITICAL for gyro: Steam integrates angular velocity as gyro * dt where dt is
	// the delta of THIS timestamp between reports -- leaving it 0 makes dt=0 so the gyro never moves the
	// view on the host, no matter how correct the gyro values are. off 28 -> rep[30].
	put_u32(out, 28, micros());

	// IMU: accel @0x22 (offset 32), gyro @0x28 (offset 38) -- inverse of imuFrom45
	put_s16(out, 32, g_in.ax);
	put_s16(out, 34, g_in.ay);
	put_s16(out, 36, g_in.az);
	put_s16(out, 38, g_in.gx);
	put_s16(out, 40, g_in.gy);
	put_s16(out, 42, g_in.gz);
}
