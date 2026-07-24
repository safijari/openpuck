// drv_ds4.c — Sony DualShock 4 over Bluetooth Classic (HID).
//
// Report layout ported from joypad-os ds4_bt.c. Reports arrive body-only (the
// report-id byte is stripped by the host before decode): BT report 0x11 carries
// two header bytes before the standard DS4 input block; the basic 0x01 report
// starts the block immediately. Rumble/LED go out as report 0x11 via SET_REPORT
// on the control channel (no CRC needed on that path).
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "bt/input_driver.h"
#include <string.h>

static bool ds4_match(uint16_t vid, uint16_t pid, const char *devname, bool is_ble)
{
	(void)pid;
	if (is_ble)
		return false;  // DS4 pairs over Classic
	if (vid == 0x054C)
		return true;
	return devname && (strstr(devname, "Wireless Controller") ||
			   strstr(devname, "DUALSHOCK") || strstr(devname, "DualShock"));
}

static bool ds4_decode(uint8_t report_id, const uint8_t *d, uint16_t len,
		       puck_input_t *io)
{
	const uint8_t *b;  // start of the DS4 input block
	if (report_id == 0x11 && len >= 2 + 11)
		b = d + 2;  // BT full report: 2 header bytes precede the block
	else if (report_id == 0x01 && len >= 11)
		b = d;
	else
		return false;

	uint8_t x = b[0], y = b[1], z = b[2], rz = b[3];
	uint8_t b0 = b[4], b1 = b[5], b2 = b[6];
	uint8_t dpad = b0 & 0x0F;
	bool up = (dpad == 0 || dpad == 1 || dpad == 7);
	bool right = (dpad >= 1 && dpad <= 3);
	bool down = (dpad >= 3 && dpad <= 5);
	bool left = (dpad >= 5 && dpad <= 7);

	uint32_t bt = 0;
	if (up) bt |= TB_DUP;
	if (down) bt |= TB_DDN;
	if (left) bt |= TB_DLF;
	if (right) bt |= TB_DRT;
	if (b0 & 0x20) bt |= TB_A;    // cross → south
	if (b0 & 0x40) bt |= TB_B;    // circle → east
	if (b0 & 0x10) bt |= TB_X;    // square → west
	if (b0 & 0x80) bt |= TB_Y;    // triangle → north
	if (b1 & 0x01) bt |= TB_LB;   // L1
	if (b1 & 0x02) bt |= TB_RB;   // R1
	if (b1 & 0x04) bt |= TB_L2;   // L2 click
	if (b1 & 0x08) bt |= TB_R2;   // R2 click
	if (b1 & 0x10) bt |= TB_VIEW; // share → View
	if (b1 & 0x20) bt |= TB_MENU; // options → Menu
	if (b1 & 0x40) bt |= TB_L3;
	if (b1 & 0x80) bt |= TB_R3;
	if (b2 & 0x01) bt |= TB_STEAM; // PS → guide
	if (b2 & 0x02) bt |= TB_QAM;   // touchpad click → QAM

	io->buttons = bt;
	io->lx = (int16_t)(((int)x - 128) << 8);
	io->ly = (int16_t)((128 - (int)y) << 8);
	io->rx = (int16_t)(((int)z - 128) << 8);
	io->ry = (int16_t)((128 - (int)rz) << 8);
	io->lt = b[7];
	io->rt = b[8];

	// IMU (full report only): gyro then accel, int16 LE, after timestamp+temp.
	if (report_id == 0x11 && len >= 2 + 24) {
		io->gx = (int16_t)(b[12] | (b[13] << 8));
		io->gy = (int16_t)(b[14] | (b[15] << 8));
		io->gz = (int16_t)(b[16] | (b[17] << 8));
		io->ax = (int16_t)(b[18] | (b[19] << 8));
		io->ay = (int16_t)(b[20] | (b[21] << 8));
		io->az = (int16_t)(b[22] | (b[23] << 8));
	}
	return true;
}

static uint8_t ds4_rumble(uint16_t lo, uint16_t hi, uint8_t *out, uint8_t max)
{
	// Report 0x11 output body (report-id supplied separately to SET_REPORT).
	const uint8_t BODY = 77;
	if (max < BODY)
		return 0;
	memset(out, 0, BODY);
	out[0] = 0x80;  // BT flags
	out[2] = 0xFF;  // enable rumble + LED
	out[5] = (uint8_t)(hi >> 8);  // right / high-freq motor
	out[6] = (uint8_t)(lo >> 8);  // left / low-freq motor
	out[7] = 0x00;  // LED R
	out[8] = 0x00;  // LED G
	out[9] = 0x40;  // LED B (blue idle)
	return BODY;
}

const input_driver_t drv_ds4 = {
	.name = "DualShock 4 (Classic)",
	.kind = 3,
	.match = ds4_match,
	.decode = ds4_decode,
	.build_rumble = ds4_rumble,
	.rumble_report_id = 0x11,
};
