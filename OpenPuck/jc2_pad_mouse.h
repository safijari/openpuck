// jc2_pad_mouse.h -- Steam Controller capacitive trackpad to Joy-Con 2 optical
// mouse filter (settle, adaptive LPF, radial hysteresis, coast, and linger).
#pragma once

#include <stdbool.h>
#include <stdint.h>

struct Jc2PadMouseState {
	bool touching;
	bool surface_linger;
	bool prev_click;
	uint8_t active_pad;
	uint32_t last_sample_seq;
	uint32_t touch_start_ms;
	uint32_t last_sample_ms;
	uint32_t last_touch_ms;
	uint32_t last_tick_ms;
	uint32_t click_damp_until_ms;
	float start_x;
	float start_y;
	float filt_x;
	float filt_y;
	float anchor_x;
	float anchor_y;
	float vel_x;
	float vel_y;
	float vel_hist_x[3];
	float vel_hist_y[3];
	uint8_t vel_hist_len;
	float coast_vx;
	float coast_vy;
	float rem_x;
	float rem_y;
};

void jc2PadMouseReset(Jc2PadMouseState *st);
bool jc2PadMouseStep(Jc2PadMouseState *st, bool touch, uint8_t pad_id,
		     int16_t raw_x, int16_t raw_y, uint32_t sample_seq,
		     bool click_down, bool cancel_linger, uint32_t now_ms,
		     int m_div, int m_fric, int16_t *out_dx, int16_t *out_dy);
