// Host-side regression tests for OpenPuck's Switch 2 Joy-Con 2 BLE mode and
// vendored libjoycon2 integration (compiled with -DOPK_HOST_TEST=1).
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bonds.h"
#include "config.h"
#include "gamepad_util.h"
#include "jc2_pad_mouse.h"
#include "mode_joycon2_ble.h"
#include "rf_link.h"
#include "src/libjoycon2/jc2_crypto.h"
#include "src/libjoycon2/jc2_proto.h"
#include "triton.h"

// Minimal OpenPuck global stubs for host-only execution.
PuckInput g_in[NSLOT] = {};
volatile uint8_t g_battery[NSLOT] = {};
volatile uint8_t g_batteryState[NSLOT] = {};
uint8_t g_abSwap = 0;
uint8_t g_back[4] = {};
uint8_t g_qamMap = 0;
uint8_t g_padStick[2] = { PS_OFF, PS_MOUSE };
uint8_t g_swGyroLegacy = 0;
uint8_t g_usbMode = MODE_SW_JC2;
int g_curSlot = 0;
int g_mDiv = 64;
int g_mFric = 0;

void slotSticks(uint8_t slot, int16_t *lx, int16_t *ly, int16_t *rx,
		int16_t *ry)
{
	if (slot >= NSLOT)
		slot = 0u;
	*lx = g_in[slot].lx;
	*ly = g_in[slot].ly;
	*rx = g_in[slot].rx;
	*ry = g_in[slot].ry;
	for (int p = 0; p < 2; p++) {
		uint8_t mode = g_padStick[p];
		if (mode != PS_LEFT && mode != PS_RIGHT)
			continue;
		bool touch = (g_in[slot].buttons &
			      (p == 0 ? TB_LPADT : TB_RPADT)) != 0u;
		if (!touch)
			continue;
		int16_t px = p == 0 ? g_in[slot].lpx : g_in[slot].rpx;
		int16_t py = p == 0 ? g_in[slot].lpy : g_in[slot].rpy;
		if (mode == PS_LEFT) {
			if (abs((int)px) > abs((int)*lx))
				*lx = px;
			if (abs((int)py) > abs((int)*ly))
				*ly = py;
		} else {
			if (abs((int)px) > abs((int)*rx))
				*rx = px;
			if (abs((int)py) > abs((int)*ry))
				*ry = py;
		}
	}
}

static void reset_globals(void)
{
	memset(g_in, 0, sizeof(g_in));
	for (int i = 0; i < NSLOT; i++) {
		g_battery[i] = 0;
		g_batteryState[i] = 0;
	}
	g_abSwap = 0;
	memset(g_back, 0, sizeof(g_back));
	g_qamMap = 0;
	g_padStick[0] = PS_OFF;
	g_padStick[1] = PS_MOUSE;
	g_swGyroLegacy = 0;
}

static void test_mode_constants(void)
{
	assert(MODE_SW_JC2 == 14);
	assert(MODE_MAX >= MODE_SW_JC2);
	assert(etypeForMode(MODE_SW_JC2) == ET_SWITCH);
	assert(PS_MOUSE == 3);
	assert(PS_MAX == 3);
}

static void test_input_mapping_buttons_and_chords(void)
{
	reset_globals();

	// Standard Xbox-label face buttons when g_abSwap == 0.
	g_in[0].buttons = TB_A | TB_X | TB_LB | TB_VIEW | TB_MENU | TB_STEAM |
			  TB_QAM;
	jc2_input_t out = jc2InputFromSlot(0);
	assert((out.buttons & JC2_BTN_A) != 0u);
	assert((out.buttons & JC2_BTN_X) != 0u);
	assert((out.buttons & JC2_BTN_L) != 0u);
	assert((out.buttons & JC2_BTN_PLUS) != 0u);
	assert((out.buttons & JC2_BTN_MINUS) != 0u);
	assert((out.buttons & JC2_BTN_HOME) != 0u);
	assert((out.buttons & JC2_BTN_CHAT) != 0u);

	// Nintendo A/B & X/Y swap + QAM remapped to Capture (code 18).
	g_abSwap = 1;
	g_qamMap = 18;
	out = jc2InputFromSlot(0);
	assert((out.buttons & JC2_BTN_B) != 0u);
	assert((out.buttons & JC2_BTN_A) == 0u);
	assert((out.buttons & JC2_BTN_Y) != 0u);
	assert((out.buttons & JC2_BTN_X) == 0u);
	assert((out.buttons & JC2_BTN_CAPTURE) != 0u);
	assert((out.buttons & JC2_BTN_CHAT) == 0u);

	// Back-4 mode-switch chord masks face + D-pad so chords don't leak.
	reset_globals();
	g_in[0].buttons = CHORD_BACK4 | TB_A | TB_DUP;
	out = jc2InputFromSlot(0);
	assert((out.buttons & (JC2_BTN_A | JC2_BTN_UP)) == 0u);

	// Analog trigger threshold trips ZL/ZR identically to Switch Pro mode.
	reset_globals();
	g_in[0].lt = SW_TRIG_ON;
	g_in[0].rt = SW_TRIG_ON - 1;
	out = jc2InputFromSlot(0);
	assert((out.buttons & JC2_BTN_ZL) != 0u);
	assert((out.buttons & JC2_BTN_ZR) == 0u);
	assert(out.left_trigger == SW_TRIG_ON);
	assert(out.right_trigger == SW_TRIG_ON - 1);

	// Pad clicks: right pad click -> R; left pad click -> L3 (or R in PS_MOUSE).
	reset_globals();
	g_in[0].buttons = TB_RPADC | TB_LPADC;
	g_padStick[0] = PS_OFF;
	out = jc2InputFromSlot(0);
	assert((out.buttons & JC2_BTN_R) != 0u);
	assert((out.buttons & JC2_BTN_L3) != 0u);

	g_in[0].buttons = TB_LPADC;
	g_padStick[0] = PS_MOUSE;
	out = jc2InputFromSlot(0);
	assert((out.buttons & JC2_BTN_R) != 0u);
	assert((out.buttons & JC2_BTN_L3) == 0u);
}

static void test_input_mapping_imu_sticks_and_battery(void)
{
	reset_globals();
	g_in[0].lx = 1200;
	g_in[0].ly = -3400;
	g_in[0].rx = 500;
	g_in[0].ry = -600;
	g_in[0].rpx = 22000;
	g_in[0].rpy = -18000;
	g_in[0].buttons = TB_RPADT;

	// PS_MOUSE leaves physical right stick intact (does not overwrite with pad).
	g_padStick[1] = PS_MOUSE;
	jc2_input_t out = jc2InputFromSlot(0);
	assert(out.rx == 500 && out.ry == -600);

	// PS_RIGHT blends touched right pad into right stick.
	g_padStick[1] = PS_RIGHT;
	out = jc2InputFromSlot(0);
	assert(out.rx == 22000 && out.ry == -18000);

	// SC2 IMU rotation (+ay, -ax, +az)/4 and trimmed gyro (0.8, 0.9, 0.9).
	reset_globals();
	g_in[0].ax = 4000;
	g_in[0].ay = -8000;
	g_in[0].az = 16384;
	g_in[0].gx = 1000;
	g_in[0].gy = 2000;
	g_in[0].gz = -1000;
	g_battery[0] = 80;
	g_batteryState[0] = 2;

	out = jc2InputFromSlot(0);
	assert(out.accel_x == -2000);
	assert(out.accel_y == -1000);
	assert(out.accel_z == 4096);
	assert(out.gyro_x == 1600);
	assert(out.gyro_y == -900);
	assert(out.gyro_z == -900);
	assert(out.battery_mv == 4020);
	assert(out.charge_state == 1u);

	g_swGyroLegacy = 1;
	out = jc2InputFromSlot(0);
	assert(out.gyro_x == 2000);
	assert(out.gyro_y == -1000);
	assert(out.gyro_z == -1000);
}

static void test_pad_mouse_settle_deadband_and_scale(void)
{
	Jc2PadMouseState st;
	jc2PadMouseReset(&st);
	int16_t dx = 0;
	int16_t dy = 0;

	// Touch-down at t=100; small capacitive bloom within 25 ms is suppressed.
	assert(jc2PadMouseStep(&st, true, 1, 0, 0, 1, false, false, 100, 64, 0,
			       &dx, &dy));
	assert(dx == 0 && dy == 0);
	assert(jc2PadMouseStep(&st, true, 1, 600, -500, 2, false, false, 115,
			       64, 0, &dx, &dy));
	assert(dx == 0 && dy == 0);

	// 1.5 s stationary thumb jitter (+-180 raw) produces strictly zero drift.
	uint32_t seq = 3;
	for (uint32_t t = 130; t <= 1630; t += 5) {
		int16_t jx = (int16_t)(600 + ((t & 8u) ? 140 : -140));
		int16_t jy = (int16_t)(-500 + ((t & 4u) ? -120 : 120));
		assert(jc2PadMouseStep(&st, true, 1, jx, jy, seq++, false,
				       false, t, 64, 0, &dx, &dy));
		assert(dx == 0 && dy == 0);
	}

	// Rightward (+X) and upward (+Y on pad) swipe moves +dx, -dy (down-pos).
	int32_t sum_x = 0;
	int32_t sum_y = 0;
	int16_t px = 600;
	int16_t py = -500;
	for (uint32_t t = 1635; t <= 1735; t += 5) {
		px = (int16_t)(px + 400);
		py = (int16_t)(py + 400);
		assert(jc2PadMouseStep(&st, true, 1, px, py, seq++, false,
				       false, t, 64, 0, &dx, &dy));
		sum_x += dx;
		sum_y += dy;
	}
	assert(sum_x > 200);
	assert(sum_y < -200);

	// Lower g_mDiv produces proportionally larger deltas for the same swipe.
	Jc2PadMouseState st_fast;
	jc2PadMouseReset(&st_fast);
	int32_t fast_x = 0;
	assert(jc2PadMouseStep(&st_fast, true, 1, 0, 0, 1, false, false, 100,
			       32, 0, &dx, &dy));
	assert(jc2PadMouseStep(&st_fast, true, 1, 0, 0, 2, false, false, 130,
			       32, 0, &dx, &dy));
	for (uint32_t i = 0; i < 20; i++) {
		assert(jc2PadMouseStep(
			&st_fast, true, 1, (int16_t)((i + 1) * 400), 0, 3 + i,
			false, false, 135 + i * 5, 32, 0, &dx, &dy));
		fast_x += dx;
	}
	assert(fast_x > sum_x + 150);
}

static void test_pad_mouse_click_damp_linger_and_coast(void)
{
	Jc2PadMouseState st;
	jc2PadMouseReset(&st);
	int16_t dx = 0;
	int16_t dy = 0;

	// Settle past 25 ms, then press click with 700-raw thumb squish: damped.
	assert(jc2PadMouseStep(&st, true, 1, 0, 0, 1, false, false, 100, 64, 0,
			       &dx, &dy));
	assert(jc2PadMouseStep(&st, true, 1, 0, 0, 2, false, false, 135, 64, 0,
			       &dx, &dy));
	assert(jc2PadMouseStep(&st, true, 1, 700, 0, 3, true, false, 140, 64, 0,
			       &dx, &dy));
	assert(dx == 0 && dy == 0);

	// Lift-off keeps surface linger alive for < 8000 ms, then drops at 8000 ms.
	assert(jc2PadMouseStep(&st, false, 1, 0, 0, 3, false, false, 200, 64, 0,
			       &dx, &dy));
	assert(jc2PadMouseStep(&st, false, 1, 0, 0, 3, false, false, 8199, 64,
			       0, &dx, &dy));
	assert(!jc2PadMouseStep(&st, false, 1, 0, 0, 3, false, false, 8200, 64,
				0, &dx, &dy));

	// Deliberate stick cancel terminates linger immediately.
	assert(jc2PadMouseStep(&st, true, 1, 0, 0, 4, false, false, 9000, 64, 0,
			       &dx, &dy));
	assert(!jc2PadMouseStep(&st, false, 1, 0, 0, 4, false, true, 9010, 64,
				0, &dx, &dy));

	// Fast flick with m_fric = 85 coasts after lift-off and decays to rest.
	jc2PadMouseReset(&st);
	assert(jc2PadMouseStep(&st, true, 1, -12000, 0, 1, false, false, 100,
			       64, 85, &dx, &dy));
	assert(jc2PadMouseStep(&st, true, 1, -12000, 0, 2, false, false, 130,
			       64, 85, &dx, &dy));
	for (uint32_t i = 0; i < 6; i++) {
		jc2PadMouseStep(&st, true, 1,
				(int16_t)(-12000 + (i + 1) * 3500), 0, 3 + i,
				false, false, 135 + i * 5, 64, 85, &dx, &dy);
	}
	int32_t coast_sum = 0;
	for (uint32_t t = 170; t <= 1200; t += 10) {
		jc2PadMouseStep(&st, false, 1, 0, 0, 9, false, false, t, 64, 85,
				&dx, &dy);
		coast_sum += dx;
	}
	assert(coast_sum > 20);
	assert(st.coast_vx == 0.0f && st.coast_vy == 0.0f);
}

struct MockBondStore {
	jc2_bond_t saved;
	int load_calls;
	int save_calls;
};

static bool mock_load_bond(jc2_bond_t *out, void *ctx)
{
	struct MockBondStore *s = (struct MockBondStore *)ctx;
	s->load_calls++;
	(void)out;
	return false;
}

static bool mock_save_bond(const jc2_bond_t *bond, void *ctx)
{
	struct MockBondStore *s = (struct MockBondStore *)ctx;
	s->save_calls++;
	s->saved = *bond;
	return true;
}

static void test_vendored_libjoycon2_integration(void)
{
	// Verify libjoycon2 mouse-mode Report 0x08 pins quiet 0x0C IMU across skips.
	static const uint8_t mac[6] = { 0x10, 0x20, 0x30, 0x40, 0x50, 0xC0 };
	jc2_proto_state_t proto;
	jc2_proto_init(&proto, mac, NULL);
	uint8_t cmd03[16] = { 0x03, 0x91, 0x01, 0x0C, 0x00, 0x08, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t rsp[JC2_CMD_RESP_MAX];
	jc2_cmd_effects_t eff;
	uint16_t rlen = jc2_proto_handle_cmd(&proto, JC2_SIDE_RIGHT, cmd03,
					     sizeof(cmd03), rsp, &eff);
	assert(rlen == 8u);

	jc2_input_t in;
	memset(&in, 0, sizeof(in));
	in.mouse_on_surface = true;
	in.mouse_dx = 12;
	in.mouse_dy = -8;
	in.accel_x = 1234;
	in.gyro_z = -5678;

	uint8_t rep[JC2_REPORT0708_SIZE];
	jc2_proto_build_report0708(&proto, JC2_SIDE_RIGHT, 0u, &in, 1000u, true,
				   rep);
	jc2_proto_build_report0708(&proto, JC2_SIDE_RIGHT, 1u, &in, 31000u,
				   true, rep);
	assert(rep[0] == 1u);
	assert(rep[4] == 0x07u);
	assert(jc2_read_s16_le(&rep[9]) == 12);
	assert(jc2_read_s16_le(&rep[11]) == -8);
	assert(rep[13] == 0x16u);
	assert(rep[15] == 0x1Eu);
	assert(rep[19] == 0x0Cu);

	// Verify Switch 2 captured BLE CCM key derivation + encrypt/decrypt.
	static const uint8_t ltk_be[16] = { 0xc1, 0x27, 0x80, 0x67, 0x1a, 0xfd,
					    0x29, 0xb8, 0x00, 0xe1, 0xdd, 0xc5,
					    0x19, 0xb4, 0xf0, 0x54 };
	static const uint8_t skdm[8] = { 0xf0, 0x24, 0x57, 0xb3,
					 0xa7, 0x56, 0x4c, 0x0b };
	static const uint8_t skds[8] = { 0x80, 0xb8, 0xb9, 0xb6,
					 0x10, 0x7b, 0x9b, 0x73 };
	static const uint8_t iv8[8] = { 0x27, 0x78, 0x26, 0x54,
					0xcd, 0x09, 0x87, 0xc8 };
	static const uint8_t f675_exp[5] = { 0xf2, 0xdb, 0x11, 0x88, 0xda };
	static const uint8_t start_enc_rsp[1] = { 0x06u };
	uint8_t ltk_le[16];
	uint8_t sk_be[16];
	uint8_t out_ct[5];
	uint8_t out_pt[1];
	for (int i = 0; i < 16; i++)
		ltk_le[i] = ltk_be[15 - i];
	jc2_ble_derive_sk(ltk_le, skdm, skds, sk_be);
	jc2_ble_ccm_encrypt(sk_be, iv8, 0ULL, false, 0x07u, start_enc_rsp, 1u,
			    out_ct);
	assert(memcmp(out_ct, f675_exp, sizeof(f675_exp)) == 0);
	assert(jc2_ble_ccm_decrypt(sk_be, iv8, 0ULL, false, 0x07u, out_ct,
				   sizeof(out_ct), out_pt));
	assert(out_pt[0] == 0x06u);

	// Verify caller-allocated jc2_device_t + vtable bond callbacks.
	MockBondStore store = {};
	jc2_device_t dev;
	jc2_config_t cfg;
	jc2_config_default(&cfg);
	cfg.cbs.load_bond = mock_load_bond;
	cfg.cbs.save_bond = mock_save_bond;
	cfg.user_ctx = &store;
	jc2_init(&dev, &cfg);
	assert(store.load_calls == 1);

	jc2_start(&dev, 100u);
	assert(jc2_is_started(&dev));
	jc2_start_pairing(&dev, true, 200u);
	jc2_poll(&dev, 300u);
	assert(store.save_calls >= 1);

	jc2_status_t st;
	jc2_get_status(&dev, &st);
	assert(st.started);
	assert(!st.has_ltk[0] && !st.has_ltk[1]);
}

int main(void)
{
	test_mode_constants();
	test_input_mapping_buttons_and_chords();
	test_input_mapping_imu_sticks_and_battery();
	test_pad_mouse_settle_deadband_and_scale();
	test_pad_mouse_click_damp_linger_and_coast();
	test_vendored_libjoycon2_integration();
	printf("PASS: OpenPuck Joy-Con 2 host tests (6/6 suites)\n");
	return 0;
}
