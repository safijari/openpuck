// mode_joycon2_ble.cpp -- Nintendo Switch 2 Joy-Con 2 BLE + Optical Mouse mode.
#include "mode_joycon2_ble.h"
#include "bonds.h"
#include "config.h"
#include "gamepad_util.h"
#include "jc2_esb_coex.h"
#include "rf_link.h"
#include "src/libjoycon2/jc2_ll.h"
#include "triton.h"

#include <stdlib.h>
#include <string.h>

#if !defined(OPK_HOST_TEST)
#include <Adafruit_LittleFS.h>
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <InternalFileSystem.h>
#include <nrf.h>
using namespace Adafruit_LittleFS_Namespace;
#endif

JoyCon2BleController g_joycon2Ble;

// SC2 accelerometer is +/-2g (16384 counts = 1g); Joy-Con 2 expects 4096 = 1g.
#define JC2_SW_ACCEL_DIV 4
#define JC2_BOND_FILE "/jc2_bonds.bin"

#if !defined(OPK_HOST_TEST)
static jc2_device_t s_jc2;
static bool s_jc2_inited = false;
static Jc2PadMouseState s_pad_mouse[NSLOT] = {};
static unsigned long s_last_stream_ms = 0u;
static unsigned long s_lr_hold_start_ms = 0u;
static unsigned long s_pair_chord_start_ms = 0u;
static bool s_pair_chord_fired = false;

static bool save_bond_fs(const jc2_bond_t *bond, void *ctx)
{
	(void)ctx;
	if (!bond)
		return false;
	InternalFS.remove(JC2_BOND_FILE);
	File f(InternalFS);
	if (!f.open(JC2_BOND_FILE, FILE_O_WRITE))
		return false;
	size_t wr = f.write((const uint8_t *)bond, sizeof(*bond));
	f.close();
	return wr == sizeof(*bond);
}

static bool load_bond_fs(jc2_bond_t *out, void *ctx)
{
	(void)ctx;
	if (!out)
		return false;
	File f(InternalFS);
	if (!f.open(JC2_BOND_FILE, FILE_O_READ))
		return false;
	jc2_bond_t tmp;
	size_t rd = f.read((uint8_t *)&tmp, sizeof(tmp));
	f.close();
	if (rd == sizeof(tmp) && tmp.magic == JC2_BOND_MAGIC) {
		*out = tmp;
		return true;
	}
	return false;
}

static void on_slot_drop(int slot)
{
	if (slot >= 0 && slot < NSLOT)
		jc2PadMouseReset(&s_pad_mouse[slot]);
}

static void ensure_ble_started(unsigned long now_ms)
{
	if (jc2_is_started(&s_jc2))
		return;
	bool was_link_up = anySlotLinkUp();
	if (!s_jc2_inited) {
		jc2_config_t cfg;
		jc2_config_default(&cfg);
		cfg.cbs.load_bond = load_bond_fs;
		cfg.cbs.save_bond = save_bond_fs;
		jc2_init(&s_jc2, &cfg);
		s_jc2_inited = true;
	}
	jc2_start(&s_jc2, (uint32_t)now_ms);
	// Keep OpenPuck fault-diag timers below priority-0 BLE radio IRQs.
	NVIC_SetPriority(WDT_IRQn, 2);
	NVIC_SetPriority(TIMER4_IRQn, 2);
	jc2EsbCoexOnBleStart(was_link_up);
}
#endif

static uint32_t code_to_jc2_btn(uint8_t c, uint32_t fA, uint32_t fB,
				uint32_t fX, uint32_t fY)
{
	switch (c) {
	case 1:
		return fA;
	case 2:
		return fB;
	case 3:
		return fX;
	case 4:
		return fY;
	case 5:
		return JC2_BTN_L;
	case 6:
		return JC2_BTN_R;
	case 7:
		return JC2_BTN_L3;
	case 8:
		return JC2_BTN_R3;
	case 9:
		return JC2_BTN_MINUS;
	case 10:
		return JC2_BTN_PLUS;
	case 11:
		return JC2_BTN_HOME;
	case 12:
		return JC2_BTN_UP;
	case 13:
		return JC2_BTN_DOWN;
	case 14:
		return JC2_BTN_LEFT;
	case 15:
		return JC2_BTN_RIGHT;
	case 18:
		return JC2_BTN_CAPTURE;
	case 19:
		return JC2_BTN_ZL;
	case 20:
		return JC2_BTN_ZR;
	default:
		return 0u;
	}
}

jc2_input_t jc2InputFromSlot(uint8_t slot)
{
	if (slot >= NSLOT)
		slot = 0u;
	const PuckInput &in = g_in[slot];
	jc2_input_t out;
	memset(&out, 0, sizeof(out));

	uint32_t raw_b = in.buttons;
	bool qam_mapped = g_qamMap && (raw_b & TB_QAM);
	if ((raw_b & CHORD_BACK4) == CHORD_BACK4)
		raw_b &= ~(uint32_t)(TB_A | TB_B | TB_X | TB_Y | TB_DUP |
				     TB_DDN | TB_DLF | TB_DRT);

	uint32_t fA = g_abSwap ? JC2_BTN_B : JC2_BTN_A;
	uint32_t fB = g_abSwap ? JC2_BTN_A : JC2_BTN_B;
	uint32_t fX = g_abSwap ? JC2_BTN_Y : JC2_BTN_X;
	uint32_t fY = g_abSwap ? JC2_BTN_X : JC2_BTN_Y;

	uint32_t b = 0u;
	if (raw_b & TB_A)
		b |= fA;
	if (raw_b & TB_B)
		b |= fB;
	if (raw_b & TB_X)
		b |= fX;
	if (raw_b & TB_Y)
		b |= fY;
	if (raw_b & TB_LB)
		b |= JC2_BTN_L;
	if (raw_b & TB_RB)
		b |= JC2_BTN_R;
	if ((raw_b & TB_L2) || in.lt >= SW_TRIG_ON)
		b |= JC2_BTN_ZL;
	if ((raw_b & TB_R2) || in.rt >= SW_TRIG_ON)
		b |= JC2_BTN_ZR;
	if (raw_b & TB_VIEW)
		b |= JC2_BTN_PLUS;
	if (raw_b & TB_MENU)
		b |= JC2_BTN_MINUS;
	if (raw_b & TB_L3)
		b |= JC2_BTN_L3;
	if (raw_b & TB_R3)
		b |= JC2_BTN_R3;
	if (raw_b & TB_STEAM)
		b |= JC2_BTN_HOME;
	if (!qam_mapped && (raw_b & TB_QAM))
		b |= JC2_BTN_CHAT;
	if (raw_b & TB_DUP)
		b |= JC2_BTN_UP;
	if (raw_b & TB_DDN)
		b |= JC2_BTN_DOWN;
	if (raw_b & TB_DLF)
		b |= JC2_BTN_LEFT;
	if (raw_b & TB_DRT)
		b |= JC2_BTN_RIGHT;
	if (raw_b & TB_RPADC)
		b |= JC2_BTN_R;
	if (raw_b & TB_LPADC)
		b |= (g_padStick[0] == PS_MOUSE) ? JC2_BTN_R : JC2_BTN_L3;

	if (raw_b & TB_L4)
		b |= code_to_jc2_btn(g_back[0], fA, fB, fX, fY);
	if (raw_b & TB_R4)
		b |= code_to_jc2_btn(g_back[1], fA, fB, fX, fY);
	if (raw_b & TB_L5)
		b |= code_to_jc2_btn(g_back[2], fA, fB, fX, fY);
	if (raw_b & TB_R5)
		b |= code_to_jc2_btn(g_back[3], fA, fB, fX, fY);
	if (qam_mapped)
		b |= code_to_jc2_btn(g_qamMap, fA, fB, fX, fY);
	out.buttons = b;

	int16_t lx = in.lx;
	int16_t ly = in.ly;
	int16_t rx = in.rx;
	int16_t ry = in.ry;
	slotSticks(slot, &lx, &ly, &rx, &ry);
	out.lx = lx;
	out.ly = ly;
	out.rx = rx;
	out.ry = ry;
	out.left_trigger = in.lt;
	out.right_trigger = in.rt;

	// Rotate SC2 IMU into Switch frame (SwitchX <- +gy, SwitchY <- -gx,
	// SwitchZ <- +gz) and divide SC2 +/-2g accel by 4 so 1g == 4096.
	int32_t neg_ax = -(int32_t)in.ax;
	int32_t neg_gx = -(int32_t)in.gx;
	out.accel_x = (int16_t)(in.ay / JC2_SW_ACCEL_DIV);
	out.accel_y = (int16_t)(neg_ax / JC2_SW_ACCEL_DIV);
	out.accel_z = (int16_t)(in.az / JC2_SW_ACCEL_DIV);

	int32_t groll = in.gy;
	int32_t gpitch = neg_gx;
	int32_t gyaw = in.gz;
	if (!g_swGyroLegacy) {
		groll = (groll * 4) / 5;
		gpitch = (gpitch * 9) / 10;
		gyaw = (gyaw * 9) / 10;
	}
	out.gyro_x = (int16_t)groll;
	out.gyro_y = (int16_t)gpitch;
	out.gyro_z = (int16_t)gyaw;

	uint8_t pct = g_battery[slot];
	out.battery_mv = pct ? (uint16_t)(3300u + (uint16_t)pct * 9u) : 4000u;
	out.charge_state = (g_batteryState[slot] == 2u) ? 1u : 0u;
	return out;
}

void jc2TriggerPairing(bool clear_bonds)
{
#if !defined(OPK_HOST_TEST)
	unsigned long now = millis();
	ensure_ble_started(now);
	jc2_start_pairing(&s_jc2, clear_bonds, (uint32_t)now);
#else
	(void)clear_bonds;
#endif
}

void jc2PulseGripLR(uint32_t duration_ms)
{
#if !defined(OPK_HOST_TEST)
	jc2_pulse_grip_lr(&s_jc2, (uint32_t)millis(), duration_ms);
#else
	(void)duration_ms;
#endif
}

#if !defined(OPK_HOST_TEST)
static void print_ble_status(void)
{
	jc2_status_t st;
	jc2_get_status(&s_jc2, &st);

	Serial.printf(
		"# JC2_STATUS ble=%u adv=%u side=%c rec=%u/%u connR=%d connL=%d "
		"in08R=%u in08L=%u sec=%u/%u ltk=%u/%u ep=%u lastCmd=%02X:%02X "
		"cmdCnt=%lu macR=%02X:%02X:%02X:%02X:%02X:%02X "
		"macL=%02X:%02X:%02X:%02X:%02X:%02X\n",
		st.started ? 1u : 0u, st.advertising ? 1u : 0u,
		st.adv_side == JC2_SIDE_RIGHT ? 'R' : 'L',
		st.adv_reconnect[0] ? 1u : 0u, st.adv_reconnect[1] ? 1u : 0u,
		st.connected[0] ? 0 : -1, st.connected[1] ? 1 : -1,
		st.streaming[0] ? 1u : 0u, st.streaming[1] ? 1u : 0u,
		st.secured[0] ? 1u : 0u, st.secured[1] ? 1u : 0u,
		st.has_ltk[0] ? 1u : 0u, st.has_ltk[1] ? 1u : 0u, st.mac_epoch,
		st.last_cmd_id, st.last_sub_id, (unsigned long)st.cmd_count,
		st.mac[0][5], st.mac[0][4], st.mac[0][3], st.mac[0][2],
		st.mac[0][1], st.mac[0][0], st.mac[1][5], st.mac[1][4],
		st.mac[1][3], st.mac[1][2], st.mac[1][1], st.mac[1][0]);

	const jc2_link_diag_t &dr = st.link[0];
	const jc2_link_diag_t &dl = st.link[1];
	Serial.printf(
		"# JC2_LINK R(ci=%u lat=%u sup=%u phy=%u/%u dle=%u mtu=%u "
		"txHz=%u/%u ok=%lu drop=%lu disc=%u:0x%02X) "
		"L(ci=%u lat=%u sup=%u phy=%u/%u dle=%u mtu=%u txHz=%u/%u "
		"ok=%lu drop=%lu disc=%u:0x%02X)\n",
		dr.conn_interval_125, dr.slave_latency, dr.sup_timeout_10ms,
		dr.phy_tx, dr.phy_rx, dr.dle_tx_octets, dr.mtu, dr.tx_hz,
		dr.evt_hz, (unsigned long)dr.hvx_ok, (unsigned long)dr.hvx_drop,
		dr.disc_cnt, dr.last_disc_reason, dl.conn_interval_125,
		dl.slave_latency, dl.sup_timeout_10ms, dl.phy_tx, dl.phy_rx,
		dl.dle_tx_octets, dl.mtu, dl.tx_hz, dl.evt_hz,
		(unsigned long)dl.hvx_ok, (unsigned long)dl.hvx_drop,
		dl.disc_cnt, dl.last_disc_reason);

	jc2EsbCoexPrintStatus();
}
#endif

bool jc2HandleConsoleCmd(const char *line)
{
	if (!line)
		return false;
#if !defined(OPK_HOST_TEST)
	if (!strcmp(line, "JC2_STATUS")) {
		print_ble_status();
		return true;
	}
	if (!strcmp(line, "JC2_PAIR")) {
		jc2TriggerPairing(true);
		jc2PulseGripLR(3000u);
		Serial.println("# JC2_PAIR: fresh L+R pairing + L+R pulse");
		return true;
	}
#endif
	return false;
}

bool JoyCon2BleController::pollRadioCoex()
{
#if !defined(OPK_HOST_TEST)
	if (!jc2_is_started(&s_jc2))
		return false;
	jc2_ll_poll();
	jc2EsbCoexStep(jc2_is_ready(&s_jc2), on_slot_drop);
	return true;
#else
	return false;
#endif
}

void JoyCon2BleController::begin()
{
#if !defined(OPK_HOST_TEST)
	USBDevice.setID(0x057E, 0x2066);
	USBDevice.setManufacturerDescriptor("Nintendo Co., Ltd.");
	USBDevice.setProductDescriptor("OpenPuck Joy-Con 2 BLE");
#endif
}

void JoyCon2BleController::task()
{
#if !defined(OPK_HOST_TEST)
	unsigned long now = millis();
	if (!jc2_is_started(&s_jc2) && now >= 1500u)
		ensure_ble_started(now);
	if (jc2_is_started(&s_jc2))
		jc2_poll(&s_jc2, (uint32_t)now);

	if ((unsigned long)(now - s_last_stream_ms) < 1u)
		return;
	s_last_stream_ms = now;

	uint8_t slot =
		(g_curSlot >= 0 && g_curSlot < NSLOT) ? (uint8_t)g_curSlot : 0u;
	const PuckInput &raw_in = g_in[slot];
	jc2_input_t in = jc2InputFromSlot(slot);

	// L+R hold (80 ms) wakes BLE advertising; Steam+View hold (1 s) re-pairs.
	uint32_t lr_chord = TB_LB | TB_RB;
	if (anySlotLinkUp() && (raw_in.buttons & lr_chord) == lr_chord) {
		if (s_lr_hold_start_ms == 0u)
			s_lr_hold_start_ms = now;
		else if ((unsigned long)(now - s_lr_hold_start_ms) >= 80u)
			ensure_ble_started(now);
	} else {
		s_lr_hold_start_ms = 0u;
	}

	uint32_t pair_chord = TB_STEAM | TB_VIEW;
	if ((raw_in.buttons & pair_chord) == pair_chord) {
		if (s_pair_chord_start_ms == 0u)
			s_pair_chord_start_ms = now;
		else if (!s_pair_chord_fired &&
			 (unsigned long)(now - s_pair_chord_start_ms) >=
				 1000u) {
			s_pair_chord_fired = true;
			jc2TriggerPairing(true);
		}
	} else {
		s_pair_chord_start_ms = 0u;
		s_pair_chord_fired = false;
	}

	// Route whichever trackpad is configured as PS_MOUSE into optical mouse.
	bool r_pad_m = (g_padStick[1] == PS_MOUSE) &&
		       ((raw_in.buttons & (TB_RPADT | TB_RPADC)) != 0u);
	bool l_pad_m = (g_padStick[0] == PS_MOUSE) &&
		       ((raw_in.buttons & (TB_LPADT | TB_LPADC)) != 0u);
	bool pad_touch = r_pad_m || l_pad_m;
	uint8_t pad_id = r_pad_m ? 1u : 0u;
	int16_t pad_rx = r_pad_m ? raw_in.rpx : raw_in.lpx;
	int16_t pad_ry = r_pad_m ? raw_in.rpy : raw_in.lpy;

	// Suppress minor right-stick thumb brushes while using right pad mouse.
	if (r_pad_m && abs((int)in.rx) < 10000 && abs((int)in.ry) < 10000) {
		in.rx = 0;
		in.ry = 0;
	}

	bool pad_click = (raw_in.buttons & (TB_RPADC | TB_LPADC | TB_A | TB_B |
					    TB_RB | TB_R2)) != 0u ||
			 raw_in.rt >= SW_TRIG_ON;

	// Keep Left Stick and D-pad free for movement/hotbar while aiming;
	// cancel surface linger only on deliberate camera stick deflection.
	bool stick_cancel =
		(g_padStick[1] == PS_MOUSE) ?
			(abs((int)in.rx) > 9000 || abs((int)in.ry) > 9000) :
			(abs((int)in.lx) > 9000 || abs((int)in.ly) > 9000);

	if (g_padStick[0] == PS_MOUSE || g_padStick[1] == PS_MOUSE) {
		in.mouse_on_surface = jc2PadMouseStep(
			&s_pad_mouse[slot], pad_touch, pad_id, pad_rx, pad_ry,
			jc2EsbCoexPadSampleSeq(slot), pad_click, stick_cancel,
			(uint32_t)now, g_mDiv, g_mFric, &in.mouse_dx,
			&in.mouse_dy);
	} else {
		jc2PadMouseReset(&s_pad_mouse[slot]);
	}

	jc2_set_input(&s_jc2, &in);
#endif
}
