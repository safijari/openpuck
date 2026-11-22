// jc2_pad_mouse.cpp -- Steam Controller capacitive trackpad to Joy-Con 2
// optical mouse filter.
//
// Steam Controller pads span [-32768..+32767] across ~32 mm (~2048 counts/mm)
// with up-positive Y, whereas Switch 2 Report 0x07/0x08 mouse deltas render at
// ~0.50 px/count (1080p) with down-positive Y. Filter capacitive jitter with a
// velocity-adaptive low-pass + radial hysteresis anchor, suppress touch-down
// bloom and lift-off peel, scale by g_mDiv, support g_mFric flick coast, and
// hold surface state briefly so multi-swipes and clicks keep the cursor live.
#include "jc2_pad_mouse.h"

#include <math.h>
#include <string.h>

#define JC2_PAD_SETTLE_MS 25u
#define JC2_PAD_SETTLE_BREAK_RAW 2800.0f
#define JC2_PAD_DEADBAND_RAW 260.0f
#define JC2_PAD_CLICK_DEADBAND_RAW 1100.0f
#define JC2_PAD_CLICK_DAMP_MS 55u
#define JC2_PAD_MAX_STEP_RAW 7500.0f
#define JC2_PAD_FLICK_MIN_RAW_PER_MS 260.0f
#define JC2_PAD_LINGER_MS 8000u

void jc2PadMouseReset(Jc2PadMouseState *st)
{
	if (st)
		memset(st, 0, sizeof(*st));
}

static bool apply_settle(Jc2PadMouseState *st, float rx, float ry,
			 uint32_t now_ms)
{
	if ((uint32_t)(now_ms - st->touch_start_ms) >= JC2_PAD_SETTLE_MS)
		return false;

	if (hypotf(rx - st->start_x, ry - st->start_y) >=
	    JC2_PAD_SETTLE_BREAK_RAW) {
		st->touch_start_ms = now_ms - JC2_PAD_SETTLE_MS;
		return false;
	}
	st->filt_x = rx;
	st->filt_y = ry;
	st->anchor_x = rx;
	st->anchor_y = ry;
	return true;
}

static void apply_adaptive_lpf(Jc2PadMouseState *st, float rx, float ry)
{
	float jx = rx - st->filt_x;
	float jy = ry - st->filt_y;
	float jump = hypotf(jx, jy);

	if (jump > JC2_PAD_MAX_STEP_RAW) {
		float k = JC2_PAD_MAX_STEP_RAW / jump;
		jx *= k;
		jy *= k;
		jump = JC2_PAD_MAX_STEP_RAW;
	}

	float alpha = 0.10f;
	if (jump >= 1200.0f) {
		alpha = 0.82f;
	} else if (jump > 120.0f) {
		float t = (jump - 120.0f) / 1080.0f;
		alpha = 0.10f + 0.72f * t;
	}
	st->filt_x += alpha * jx;
	st->filt_y += alpha * jy;
}

static void apply_deadband(Jc2PadMouseState *st, bool click_down,
			   uint32_t now_ms, uint32_t dt_ms, float div,
			   float *out_x, float *out_y)
{
	bool click_damp = click_down ||
			  (int32_t)(st->click_damp_until_ms - now_ms) > 0;
	float db = click_damp ? JC2_PAD_CLICK_DEADBAND_RAW :
				JC2_PAD_DEADBAND_RAW;
	float ax = st->filt_x - st->anchor_x;
	float ay = st->filt_y - st->anchor_y;
	float r = hypotf(ax, ay);
	float step_x = 0.0f;
	float step_y = 0.0f;

	if (r > db) {
		float s = (r - db) / r;
		step_x = ax * s;
		step_y = ay * s;
		st->anchor_x += step_x;
		st->anchor_y += step_y;
	}

	float inst_vx = step_x / (float)dt_ms;
	float inst_vy = step_y / (float)dt_ms;
	st->vel_x = 0.55f * st->vel_x + 0.45f * inst_vx;
	st->vel_y = 0.55f * st->vel_y + 0.45f * inst_vy;
	st->vel_hist_x[2] = st->vel_hist_x[1];
	st->vel_hist_y[2] = st->vel_hist_y[1];
	st->vel_hist_x[1] = st->vel_hist_x[0];
	st->vel_hist_y[1] = st->vel_hist_y[0];
	st->vel_hist_x[0] = st->vel_x;
	st->vel_hist_y[0] = st->vel_y;
	if (st->vel_hist_len < 3u)
		st->vel_hist_len++;

	*out_x = step_x / div;
	*out_y = -step_y / div;
}

static void step_coast(Jc2PadMouseState *st, uint32_t now_ms, int m_fric,
		       float *out_x, float *out_y)
{
	uint32_t dt_ms = now_ms - st->last_tick_ms;

	if (dt_ms > 20u)
		dt_ms = 20u;
	if (dt_ms == 0u)
		return;

	st->last_tick_ms = now_ms;
	float k = (m_fric > 0) ? powf((float)m_fric * 0.01f, 0.2f) : 0.0f;
	for (uint32_t i = 0u; i < dt_ms; i++) {
		*out_x += st->coast_vx;
		*out_y += st->coast_vy;
		st->coast_vx *= k;
		st->coast_vy *= k;
	}
	st->last_touch_ms = now_ms;
	if (hypotf(st->coast_vx, st->coast_vy) < 0.08f) {
		st->coast_vx = 0.0f;
		st->coast_vy = 0.0f;
	}
}

bool jc2PadMouseStep(Jc2PadMouseState *st, bool touch, uint8_t pad_id,
		     int16_t raw_x, int16_t raw_y, uint32_t sample_seq,
		     bool click_down, bool cancel_linger, uint32_t now_ms,
		     int m_div, int m_fric, int16_t *out_dx, int16_t *out_dy)
{
	if (out_dx)
		*out_dx = 0;
	if (out_dy)
		*out_dy = 0;
	if (!st)
		return false;

	if (m_div < 4)
		m_div = 64;
	if (m_fric < 0)
		m_fric = 0;
	else if (m_fric > 99)
		m_fric = 99;

	// g_mDiv = 64 -> divisor 24.0 (~1365 px per full 32 mm pad swipe).
	float div = (float)m_div * 0.375f;
	float out_x = 0.0f;
	float out_y = 0.0f;

	if (click_down && !st->prev_click)
		st->click_damp_until_ms = now_ms + JC2_PAD_CLICK_DAMP_MS;
	st->prev_click = click_down;

	if (touch && st->touching && pad_id != st->active_pad)
		st->touching = false;

	if (touch) {
		float rx = (float)raw_x;
		float ry = (float)raw_y;
		st->coast_vx = 0.0f;
		st->coast_vy = 0.0f;
		st->surface_linger = true;
		st->last_touch_ms = now_ms;

		if (!st->touching) {
			st->touching = true;
			st->active_pad = pad_id;
			st->touch_start_ms = now_ms;
			st->last_sample_ms = now_ms;
			st->last_sample_seq = sample_seq;
			st->start_x = rx;
			st->start_y = ry;
			st->filt_x = rx;
			st->filt_y = ry;
			st->anchor_x = rx;
			st->anchor_y = ry;
			st->vel_x = 0.0f;
			st->vel_y = 0.0f;
			st->vel_hist_len = 0u;
			st->rem_x = 0.0f;
			st->rem_y = 0.0f;
		} else if (sample_seq != st->last_sample_seq) {
			uint32_t dt_ms = now_ms - st->last_sample_ms;
			if (dt_ms < 1u)
				dt_ms = 1u;
			else if (dt_ms > 25u)
				dt_ms = 25u;
			st->last_sample_ms = now_ms;
			st->last_sample_seq = sample_seq;

			if (!apply_settle(st, rx, ry, now_ms)) {
				apply_adaptive_lpf(st, rx, ry);
				apply_deadband(st, click_down, now_ms, dt_ms,
					       div, &out_x, &out_y);
			}
		}
		st->last_tick_ms = now_ms;
	} else {
		if (st->touching) {
			st->touching = false;
			st->last_touch_ms = now_ms;
			st->last_tick_ms = now_ms;
			// Sample velocity ~10 ms before lift-off so peel shift
			// on the final frame cannot kick the cursor off target.
			if (m_fric > 0 && st->vel_hist_len >= 3u) {
				float pvx = st->vel_hist_x[2];
				float pvy = st->vel_hist_y[2];
				if (hypotf(pvx, pvy) >=
				    JC2_PAD_FLICK_MIN_RAW_PER_MS) {
					st->coast_vx = pvx / div;
					st->coast_vy = -pvy / div;
				}
			}
			st->vel_x = 0.0f;
			st->vel_y = 0.0f;
			st->vel_hist_len = 0u;
		}

		if (cancel_linger) {
			st->surface_linger = false;
			st->coast_vx = 0.0f;
			st->coast_vy = 0.0f;
			st->rem_x = 0.0f;
			st->rem_y = 0.0f;
		} else if (st->coast_vx != 0.0f || st->coast_vy != 0.0f) {
			step_coast(st, now_ms, m_fric, &out_x, &out_y);
		} else if (st->surface_linger &&
			   (uint32_t)(now_ms - st->last_touch_ms) >=
				   JC2_PAD_LINGER_MS) {
			st->surface_linger = false;
			st->rem_x = 0.0f;
			st->rem_y = 0.0f;
		}
	}

	st->rem_x += out_x;
	st->rem_y += out_y;
	int32_t ix = (int32_t)st->rem_x;
	int32_t iy = (int32_t)st->rem_y;
	st->rem_x -= (float)ix;
	st->rem_y -= (float)iy;
	if (ix < -32767)
		ix = -32767;
	else if (ix > 32767)
		ix = 32767;
	if (iy < -32767)
		iy = -32767;
	else if (iy > 32767)
		iy = 32767;
	if (out_dx)
		*out_dx = (int16_t)ix;
	if (out_dy)
		*out_dy = (int16_t)iy;

	return touch || st->surface_linger || st->coast_vx != 0.0f ||
	       st->coast_vy != 0.0f;
}
