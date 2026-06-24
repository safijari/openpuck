#include "triton.h"

// Copied from OpenPuck/triton.cpp. g_in is the shared input struct; imuFrom45 documents the IMU byte
// offsets that report45.cpp inverts when encoding.

PuckInput g_in;

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
