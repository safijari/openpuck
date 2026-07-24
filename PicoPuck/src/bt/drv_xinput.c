// drv_xinput.c — Xbox One S / Series controller over Bluetooth (HOGP).
//
// Ported from OpenPuck's ble-test drv_xinput_ble.cpp. Also covers 8BitDo pads in
// Xbox mode (same layout). Input report 0x01 = gamepad state, 0x02 = legacy
// separate guide button (pre-fw5). Rumble = output report 0x03.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "bt/input_driver.h"
#include <string.h>
#include <ctype.h>

static bool name_has(const char *hay, const char *needle)
{
	if (!hay)
		return false;
	size_t nl = strlen(needle);
	for (; *hay; hay++) {
		size_t i = 0;
		while (i < nl && hay[i] &&
		       tolower((unsigned char)hay[i]) ==
			       tolower((unsigned char)needle[i]))
			i++;
		if (i == nl)
			return true;
	}
	return false;
}

static bool xi_match(uint16_t vid, uint16_t pid, const char *devname, bool is_ble)
{
	(void)pid;
	if (!is_ble)
		return false;  // Classic Xbox pads are proprietary; BLE only here
	if (vid == 0x045E || vid == 0x2DC8)
		return true;
	return name_has(devname, "xbox") || name_has(devname, "8bitdo");
}

static bool xi_decode(uint8_t rid, const uint8_t *d, uint16_t len,
		      puck_input_t *io)
{
	if (rid == 0x02) {
		if (len < 1)
			return false;
		if (d[0] & 0x01)
			io->buttons |= TB_STEAM;
		else
			io->buttons &= ~TB_STEAM;
		return true;
	}
	if (rid != 0x01 || len < 14)
		return false;

	uint16_t lx = (uint16_t)(d[0] | (d[1] << 8));
	uint16_t ly = (uint16_t)(d[2] | (d[3] << 8));
	uint16_t rx = (uint16_t)(d[4] | (d[5] << 8));
	uint16_t ry = (uint16_t)(d[6] | (d[7] << 8));
	uint16_t lt = (uint16_t)(d[8] | (d[9] << 8));
	uint16_t rt = (uint16_t)(d[10] | (d[11] << 8));
	uint8_t b1 = d[13];
	uint8_t b2 = (len > 14) ? d[14] : 0;
	uint8_t b3 = (len > 15) ? d[15] : 0;

	uint32_t b = triton_hat_bits(d[12]);
	if (b1 & 0x01) b |= TB_A;
	if (b1 & 0x02) b |= TB_B;
	if (b1 & 0x04) b |= TB_X;
	if (b1 & 0x08) b |= TB_Y;
	if (b1 & 0x10) b |= TB_LB;
	if (b1 & 0x20) b |= TB_RB;
	if (b1 & 0x40) b |= TB_VIEW;
	if (b1 & 0x80) b |= TB_MENU;
	if (len > 14) {
		if (b2 & 0x01) b |= TB_STEAM;  // fw 5.x: guide in-band
		if (b2 & 0x02) b |= TB_L3;
		if (b2 & 0x04) b |= TB_R3;
	} else {
		b |= io->buttons & TB_STEAM;  // 14-byte: guide latched from report 0x02
	}
	if (b3 & 0x01) b |= TB_QAM;  // Series share → QAM
	if (lt >= 1000) b |= TB_L2;
	if (rt >= 1000) b |= TB_R2;

	io->buttons = b;
	io->lx = (int16_t)(lx - 32768);
	io->ly = (int16_t)(32767 - ly);  // HID Y down → gamepad Y up
	io->rx = (int16_t)(rx - 32768);
	io->ry = (int16_t)(32767 - ry);
	io->lt = (uint8_t)((lt >> 2) > 255 ? 255 : (lt >> 2));
	io->rt = (uint8_t)((rt >> 2) > 255 ? 255 : (rt >> 2));
	return true;
}

static uint8_t xi_rumble(uint16_t lo, uint16_t hi, uint8_t *out, uint8_t max)
{
	if (max < 8)
		return 0;
	out[0] = 0x0F;  // enable all four motors
	out[1] = 0;     // trigger-left magnitude (impulse triggers)
	out[2] = 0;     // trigger-right magnitude
	out[3] = (uint8_t)((uint32_t)lo * 100 / 0xFFFF);  // strong (low-freq)
	out[4] = (uint8_t)((uint32_t)hi * 100 / 0xFFFF);  // weak (high-freq)
	out[5] = 0xFF;  // sustain ~2.5 s (the stream refreshes long rumbles)
	out[6] = 0x00;  // release
	out[7] = 0x00;  // no loop — a lost "off" must not buzz forever
	return 8;
}

const input_driver_t drv_xinput = {
	.name = "Xbox (BLE)",
	.kind = 2,
	.match = xi_match,
	.decode = xi_decode,
	.build_rumble = xi_rumble,
	.rumble_report_id = 0x03,
};
