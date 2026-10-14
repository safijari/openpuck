#include "triton.h"

PuckInput g_in[NSLOT];

// Triton report 0x47 carries a 16-bit IMU clock in 32-us units.  Unwrap it
// per RF bond slot so translated modes receive the same monotonic microsecond
// source clock shared with 0x42/0x45 reports.
static uint16_t g_triton47LastTick[NSLOT] = {};
static uint32_t g_triton47TimestampUs[NSLOT] = {};
static uint8_t g_triton47TimestampValid[NSLOT] = {};

uint32_t tritonTimestamp47Us(uint8_t slot, uint16_t timestamp32us)
{
	if (slot >= NSLOT)
		return (uint32_t)timestamp32us * 32u;
	if (!g_triton47TimestampValid[slot]) {
		g_triton47TimestampValid[slot] = 1;
		g_triton47LastTick[slot] = timestamp32us;
		g_triton47TimestampUs[slot] = (uint32_t)timestamp32us * 32u;
		return g_triton47TimestampUs[slot];
	}
	uint16_t delta = (uint16_t)(timestamp32us - g_triton47LastTick[slot]);
	g_triton47LastTick[slot] = timestamp32us;
	g_triton47TimestampUs[slot] += (uint32_t)delta * 32u;
	return g_triton47TimestampUs[slot];
}

void tritonTimestamp47Reset(uint8_t slot)
{
	if (slot >= NSLOT)
		return;
	g_triton47LastTick[slot] = 0;
	g_triton47TimestampUs[slot] = 0;
	g_triton47TimestampValid[slot] = 0;
}

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
