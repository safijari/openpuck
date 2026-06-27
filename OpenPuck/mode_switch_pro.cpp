#include "mode_switch_pro.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_mount.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Arduino.h>
#include <string.h>
using namespace Adafruit_LittleFS_Namespace;

SwitchProController g_switchPro;

// Switch full-report (0x30) cadence. The console integrates the 3 IMU samples per report assuming ~5 ms/sample
// (3 samples / 15 ms genuine); too high a rate over-integrates the gyro. 120 Hz is drift-free and lower-latency
// (default); 66 Hz is the genuine compat fallback; "full" is the 4 ms PC rate (lowest latency).
#define SW_PRO_REPORT_MS 15u // 66 Hz
#define SW_PRO_REPORT_MS_120 8u // ~120 Hz
// SC2 accel is +/-2g (16384/g); /4 -> the genuine Pro +/-8g (4096/g) the Switch cal expects
#define SW_ACCEL_DIV 4

uint8_t g_swProRate =
	2; // 0 = 66Hz, 1 = 120Hz, 2 = full (~250Hz / USB_STREAM_MS, default)
uint8_t g_swGyroScale10 = 10; // gyro sensitivity x10 (10 = 1.0x)

// Persist the two Switch Pro motion settings in their own tiny file so they never trigger a global-Cfg reset.
#define SWPRO_CFG_FILE "/swprocfg.bin"
void swProSaveCfg()
{
	uint8_t b[3] = { 0x01, g_swProRate,
			 g_swGyroScale10 }; // [ver][rate 0/1/2][gyroScale x10]
	InternalFS.remove(SWPRO_CFG_FILE);
	File f(InternalFS);
	if (f.open(SWPRO_CFG_FILE, FILE_O_WRITE)) {
		f.write(b, sizeof b);
		f.close();
	}
}
static void swProLoadCfg()
{
	File f(InternalFS);
	uint8_t b[3];
	if (f.open(SWPRO_CFG_FILE, FILE_O_READ)) {
		if (f.read(b, 3) == 3 && b[0] == 0x01) {
			// 0=66Hz, 1=120Hz, 2=full; bad value -> default full
			g_swProRate = (b[1] <= 2) ? b[1] : 2;
			g_swGyroScale10 =
				(b[2] >= 5 && b[2] <= 30) ?
					b[2] :
					10; // sane bounds (0.5x..3.0x)
		}
		f.close();
	}
}
// Scale a gyro axis by g_swGyroScale10/10, clamped to int16 (3x can saturate at high rates -- expected).
static int16_t gscale(int16_t v)
{
	if (g_swGyroScale10 == 10)
		return v;
	int32_t s = (int32_t)v * (int32_t)g_swGyroScale10 / 10;
	if (s > 32767)
		s = 32767;
	else if (s < -32768)
		s = -32768;
	return (int16_t)s;
}

static const uint8_t SWPRO_HID_DESC[] = {
	0x05, 0x01, 0x15, 0x00, 0x09, 0x04, 0xA1, 0x01, 0x85, 0x30, 0x05, 0x01,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x0A, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
	0x95, 0x0A, 0x55, 0x00, 0x65, 0x00, 0x81, 0x02, 0x05, 0x09, 0x19, 0x0B,
	0x29, 0x0E, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x04, 0x81, 0x02,
	0x75, 0x01, 0x95, 0x02, 0x81, 0x03, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xA1,
	0x00, 0x0B, 0x30, 0x00, 0x01, 0x00, 0x0B, 0x31, 0x00, 0x01, 0x00, 0x0B,
	0x32, 0x00, 0x01, 0x00, 0x0B, 0x35, 0x00, 0x01, 0x00, 0x15, 0x00, 0x27,
	0xFF, 0xFF, 0x00, 0x00, 0x75, 0x10, 0x95, 0x04, 0x81, 0x02, 0xC0, 0x0B,
	0x39, 0x00, 0x01, 0x00, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3B,
	0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x05, 0x09, 0x19,
	0x0F, 0x29, 0x12, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x04, 0x81,
	0x02, 0x75, 0x08, 0x95, 0x34, 0x81, 0x03, 0x06, 0x00, 0xFF, 0x85, 0x21,
	0x09, 0x01, 0x75, 0x08, 0x95, 0x3F, 0x81, 0x03, 0x85, 0x81, 0x09, 0x02,
	0x75, 0x08, 0x95, 0x3F, 0x81, 0x03, 0x85, 0x01, 0x09, 0x03, 0x75, 0x08,
	0x95, 0x3F, 0x91, 0x83, 0x85, 0x10, 0x09, 0x04, 0x75, 0x08, 0x95, 0x3F,
	0x91, 0x83, 0x85, 0x80, 0x09, 0x05, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
	0x85, 0x82, 0x09, 0x06, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83, 0xC0
};
#define JC_BTN_Y (1u << 0)
#define JC_BTN_X (1u << 1)
#define JC_BTN_B (1u << 2)
#define JC_BTN_A (1u << 3)
#define JC_BTN_R (1u << 6)
#define JC_BTN_ZR (1u << 7)
#define JC_BTN_MINUS (1u << 8)
#define JC_BTN_PLUS (1u << 9)
#define JC_BTN_RSTICK (1u << 10)
#define JC_BTN_LSTICK (1u << 11)
#define JC_BTN_HOME (1u << 12)
#define JC_BTN_CAPTURE (1u << 13)
#define JC_BTN_DOWN (1u << 16)
#define JC_BTN_UP (1u << 17)
#define JC_BTN_RIGHT (1u << 18)
#define JC_BTN_LEFT (1u << 19)
#define JC_BTN_L (1u << 22)
#define JC_BTN_ZL (1u << 23)
static uint32_t codeToJc(uint8_t c, uint32_t fA, uint32_t fB, uint32_t fX,
			 uint32_t fY)
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
		return JC_BTN_L;
	case 6:
		return JC_BTN_R;
	case 7:
		return JC_BTN_LSTICK;
	case 8:
		return JC_BTN_RSTICK;
	case 9:
		return JC_BTN_MINUS;
	case 10:
		return JC_BTN_PLUS;
	case 11:
		return JC_BTN_HOME;
	case 18:
		return JC_BTN_CAPTURE; // Capture / Screenshot (Switch-only target)
	case 19:
		return JC_BTN_ZL; // left trigger
	case 20:
		return JC_BTN_ZR; // right trigger
	case 12:
		return JC_BTN_UP;
	case 13:
		return JC_BTN_DOWN;
	case 14:
		return JC_BTN_LEFT;
	case 15:
		return JC_BTN_RIGHT;
	default:
		return 0;
	}
}
// NSLOT Pro-Controller HIDs (one per bond slot) + per-slot handshake state (timer, report-mode gate,
// subcommand-reply FIFO), per-slot reply queue indices, per-slot last-stream millis, per-slot MAC, and
// per-slot user-cal SPI mirror.
static Adafruit_USBD_HID g_swPro[NSLOT];
static unsigned long g_swProLastMs[NSLOT] = { 0 };
// 0 until the host's subcommand 0x03 selects 0x30 -> THEN we stream input
static uint8_t g_swProReportMode[NSLOT] = { 0 };
static uint8_t g_jcTimer[NSLOT] = { 0 };
// Per-slot MAC. The console uses the BT addr it reads from subcommand 0x02 to identify the controller;
// real Pro Controllers each have their own. Last byte varies per slot.
static const uint8_t JC_MAC_BASE[5] = { 0x7C, 0xBB, 0x8A, 0x00, 0x00 };
static uint8_t g_jcMac[NSLOT][6];
static bool g_jcMacInit = false;
static void initJcMacs()
{
	if (g_jcMacInit)
		return;
	for (int s = 0; s < NSLOT; s++) {
		memcpy(g_jcMac[s], JC_MAC_BASE, 5);
		g_jcMac[s][5] = (uint8_t)(0x10 + s);
	}
	g_jcMacInit = true;
}
// Dynamic mount: the handshake/state arrays are keyed by USB slot, but the controller INPUT (and rumble target)
// belongs to the bond slot the USB slot is mapped to. This resolves usbSlot -> bondSlot (identity fallback if
// somehow unmapped, e.g. legacy paths).
static inline uint8_t jcBondOf(uint8_t usbSlot)
{
	int b = (usbSlot < NSLOT) ? g_usbToBond[usbSlot] : -1;
	return (b >= 0) ? (uint8_t)b : usbSlot;
}
static uint16_t jcRumbleAmp(const uint8_t r[4])
{
	// Nintendo packs a high band (r[0],r[1]) and low band (r[2],r[3]) of frequency+amplitude. We only need a
	// magnitude for the Steam motor, so pull the two amplitude fields: HF amp = r[1]>>1, LF amp = r[3]&0x3F.
	// Every canonical idle/neutral frame (00 01 40 40, 00 00 01 40, all-zero) decodes to 0 this way, so a steady
	// idle rumble stream maps to "off" instead of latching the motor on. No forced floor: 0 amplitude -> 0.
	uint8_t hf = r[1] >> 1; // high-band amplitude (neutral 0x01 -> 0)
	uint8_t lf = r[3] & 0x3F; // low-band amplitude  (neutral 0x40 -> 0)
	uint8_t a = hf > lf ? hf : lf; // 0..0x7F
	return (uint16_t)a << 9; // scale to ~16-bit motor speed
}
// Per-slot: each Pro Controller has its own rumble stream, so the "last" relay tracking must be per-slot.
static uint16_t g_jcLastLo[NSLOT] = { 0 };
static uint16_t g_jcLastHi[NSLOT] = { 0 };
static void jcRumble(uint8_t slot, const uint8_t *p, uint16_t pn)
{
	if (pn < 9)
		return; // [timer][left rumble x4][right rumble x4]
	uint16_t lo = jcRumbleAmp(p + 1), hi = jcRumbleAmp(p + 5);
	// only relay on change: the Switch streams rumble every frame; re-sending
	// unchanged values would flood the RF relay and loop the motor
	if (lo == g_jcLastLo[slot] && hi == g_jcLastHi[slot])
		return;
	g_jcLastLo[slot] = lo;
	g_jcLastHi[slot] = hi;
	hapticSteamRumble(lo, hi,
			  jcBondOf(slot)); // route to the mapped controller
}
static int jcStick12(int16_t v, bool inv)
{ // steam int16 (center 0) -> 12-bit (center 0x800), clamped
	int a = 2048 + (inv ? -((int)v >> 4) : ((int)v >> 4));
	return a < 0 ? 0 : (a > 4095 ? 4095 : a);
}
// pack two 12-bit axes into 3 bytes (both normal; Switch Y polarity is opposite DS4)
static void jcPackStick(uint8_t s[3], int16_t x, int16_t y)
{
	int X = jcStick12(x, false), Y = jcStick12(y, false);
	s[0] = (uint8_t)(X & 0xFF);
	s[1] = (uint8_t)(((Y & 0x0F) << 4) | ((X >> 8) & 0x0F));
	s[2] = (uint8_t)((Y >> 4) & 0xFF);
}
// Standard input-report prefix [0..11] (timer, battery/conn, 3 button bytes, both packed sticks, vibrator),
// shared by the streamed 0x30 report and the 0x21 subcommand-reply reports the host reads during init.
static void jcInputPrefix(uint8_t slot, uint8_t *out)
{
	uint8_t bond = jcBondOf(
		slot); // input data comes from the mapped controller; timer/state stay per USB slot
	uint32_t b = g_in[bond].buttons;
	// QAM (3 dots) remap -> applied via codeToJc below like a back paddle (so Capture(18)/any target work).
	bool qam = g_qamMap && (b & TB_QAM);
	if ((b & CHORD_BACK4) == CHORD_BACK4)
		b &= ~(uint32_t)(TB_A | TB_B | TB_X | TB_Y);
	uint32_t fA = g_abSwap ? JC_BTN_B : JC_BTN_A,
		 fB = g_abSwap ? JC_BTN_A : JC_BTN_B;
	uint32_t fX = g_abSwap ? JC_BTN_Y : JC_BTN_X,
		 fY = g_abSwap ? JC_BTN_X : JC_BTN_Y;
	uint32_t jc = 0;
	if (b & TB_Y)
		jc |= fY;
	if (b & TB_B)
		jc |= fB;
	if (b & TB_A)
		jc |= fA;
	if (b & TB_X)
		jc |= fX;
	if (b & TB_LB)
		jc |= JC_BTN_L;
	if (b & TB_RB)
		jc |= JC_BTN_R;
	if ((g_in[bond].lt >= SW_TRIG_ON) || (b & 0x8000000u))
		jc |= JC_BTN_ZL;
	if ((g_in[bond].rt >= SW_TRIG_ON) || (b & 0x800000u))
		jc |= JC_BTN_ZR;
	if (b & TB_VIEW)
		jc |= JC_BTN_PLUS;
	if (b & TB_MENU)
		jc |= JC_BTN_MINUS;
	if (b & TB_L3)
		jc |= JC_BTN_LSTICK;
	if (b & TB_R3)
		jc |= JC_BTN_RSTICK;
	if (b & TB_STEAM)
		jc |= JC_BTN_HOME;
	if (b & TB_DDN)
		jc |= JC_BTN_DOWN;
	if (b & TB_DUP)
		jc |= JC_BTN_UP;
	if (b & TB_DRT)
		jc |= JC_BTN_RIGHT;
	if (b & TB_DLF)
		jc |= JC_BTN_LEFT;
	if (b & TB_L4)
		jc |= codeToJc(g_back[0], fA, fB, fX, fY);
	if (b & TB_R4)
		jc |= codeToJc(g_back[1], fA, fB, fX, fY);
	if (b & TB_L5)
		jc |= codeToJc(g_back[2], fA, fB, fX, fY);
	if (b & TB_R5)
		jc |= codeToJc(g_back[3], fA, fB, fX, fY);
	if (qam)
		jc |= codeToJc(g_qamMap, fA, fB, fX, fY);
	out[0] = g_jcTimer[slot]++;

	// battery full+charging (hi nibble 0x9), connection_info=1 (lo nibble): a wired/charging Pro Controller.
	// A real Switch reads this to show the pad as connected; Steam/hid-nintendo accept it too.
	out[1] = 0x91;
	out[2] = (uint8_t)(jc);
	out[3] = (uint8_t)(jc >> 8);
	out[4] = (uint8_t)(jc >> 16);

	// charging_grip bit (button "common" byte, bit7): genuine Pro Controller always sets it on USB; real
	// Switch uses it to recognise a wired controller. Not a button, so hid-nintendo ignores it.
	out[3] |= 0x80;
	jcPackStick(out + 5, g_in[bond].lx, g_in[bond].ly);
	jcPackStick(out + 8, g_in[bond].rx, g_in[bond].ry);
	// rumble_input_report echo: genuine pad emits 0x09..0x0C; some Switch firmware expects this nonzero.
	out[11] = 0x09;
}
static void switchProBuild(uint8_t slot, uint8_t out[63])
{
	uint8_t bond =
		jcBondOf(slot); // IMU data comes from the mapped controller
	memset(out, 0, 63);
	jcInputPrefix(slot, out);
	// Gyro slot order follows hid-nintendo: raw+6 = ROLL, raw+8 = PITCH, raw+10 = YAW. Source routing (a proper
	// rotation, det +1): SwitchX<-+gy, SwitchY<--gx, SwitchZ<-+gz.
	//
	// The accel MUST use the SAME signed permutation as the gyro. Steam reads the gyro raw and ignores this, but the
	// Switch console FUSES accel (gravity) with gyro to anchor absolute orientation -- if the accel frame has the
	// opposite handedness, the gravity-correction term points the wrong way and the attitude estimate latches into a
	// rotated solution (intermittent ~45deg axis-mixing, per-unit roll bias, cleared only by a replug). So accel Y
	// must be -ax to match the gyro's -gx pitch axis; resting gravity then lands cleanly on Switch Z (the up axis).
	// The SC2 accelerometer is +/-2g full-scale (~16384 counts = 1g). The factory cal we present (sensitivity 0x4000)
	// tells the Switch it's a genuine +/-8g Pro Controller (4096 counts = 1g), so divide by SW_ACCEL_DIV=4 to report
	// ~1g. Without this the console reads gravity as ~4g, REJECTS the accel for drift correction (it must be linear
	// accel, not gravity), and gyro roll error accumulates into a slow ~45deg lean. Gyro is left at native scale.
	// accel X <- +ay, accel Y <- -ax, accel Z <- +az (signs match gyro)
	int16_t aX = (int16_t)(g_in[bond].ay / SW_ACCEL_DIV);
	int16_t aY = (int16_t)((-(int16_t)g_in[bond].ax) / SW_ACCEL_DIV);
	int16_t aZ = (int16_t)(g_in[bond].az / SW_ACCEL_DIV);

	// gyro outputs scaled by the user sensitivity factor
	int16_t groll = gscale((int16_t)g_in[bond].gy);
	int16_t gpitch = gscale((int16_t)(-(int16_t)g_in[bond].gx));
	int16_t gyaw = gscale((int16_t)g_in[bond].gz);
	for (int k = 0; k < 3; k++) {
		int o = 12 + k * 12;
		out[o + 0] = aX & 0xFF;
		out[o + 1] = (aX >> 8) & 0xFF;
		out[o + 2] = aY & 0xFF;
		out[o + 3] = (aY >> 8) & 0xFF;
		out[o + 4] = aZ & 0xFF;
		out[o + 5] = (aZ >> 8) & 0xFF;
		out[o + 6] = groll & 0xFF;
		out[o + 7] = (groll >> 8) & 0xFF;
		out[o + 8] = gpitch & 0xFF;
		out[o + 9] = (gpitch >> 8) & 0xFF;
		out[o + 10] = gyaw & 0xFF;
		out[o + 11] = (gyaw >> 8) & 0xFF;
	}
}
// --- Canonical factory SPI dumps the host reads for calibration. Neutral IMU + centered sticks so a fresh
// "device" calibrates sane; user-cal regions (0x80xx) read 0xFF so the host falls back to these factory blocks.
static const uint8_t SPI_IMU_CAL[24] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					 0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34 };
static const uint8_t SPI_PARAMS1[24] = { 0x50, 0xFD, 0x00, 0x00, 0xC6, 0x0F,
					 0x0F, 0x30, 0x61, 0x96, 0x30, 0xF3,
					 0xD4, 0x14, 0x54, 0x41, 0x15, 0x54,
					 0xC7, 0x79, 0x9C, 0x33, 0x36, 0x63 };
static const uint8_t SPI_PARAMS2[18] = { 0x0F, 0x30, 0x61, 0x96, 0x30, 0xF3,
					 0xD4, 0x14, 0x54, 0x41, 0x15, 0x54,
					 0xC7, 0x79, 0x9C, 0x33, 0x36, 0x63 };
static const uint8_t SPI_COLOR[13] = { 0x32, 0x32, 0x32, 0xE6, 0xE6, 0xE6, 0x32,
				       0x32, 0x32, 0x32, 0x32, 0x32, 0xFF };
// built at boot: left[9]+right[9] factory stick calibration (packed 12-bit). Shared across slots -- it's a
// factory stub the host reads to seed its cal, and we present the same neutral stub for all controllers.
static uint8_t g_spiStickCal[18];
static void jcPack12(uint8_t *o9, const uint16_t v[6])
{ // pack 6 12-bit values into 9 bytes (Switch stick-cal format)
	o9[0] = v[0] & 0xFF;
	o9[1] = ((v[1] & 0x0F) << 4) | ((v[0] >> 8) & 0x0F);
	o9[2] = (v[1] >> 4) & 0xFF;
	o9[3] = v[2] & 0xFF;
	o9[4] = ((v[3] & 0x0F) << 4) | ((v[2] >> 8) & 0x0F);
	o9[5] = (v[3] >> 4) & 0xFF;
	o9[6] = v[4] & 0xFF;
	o9[7] = ((v[5] & 0x0F) << 4) | ((v[4] >> 8) & 0x0F);
	o9[8] = (v[5] >> 4) & 0xFF;
}
static void jcBuildStickCal()
{
	const uint16_t C = 2048, R = 1800; // center, +/- range per axis
	uint16_t L[6] = {
		R, R, C, C, R, R
	}; // left  order: max(x,y), center(x,y), min(x,y)
	uint16_t Rr[6] = {
		C, C, R, R, R, R
	}; // right order: center(x,y), min(x,y), max(x,y)
	jcPack12(&g_spiStickCal[0], L);
	jcPack12(&g_spiStickCal[9], Rr);
}
// Manual-pairing (subcommand 0x01) reply payloads. A real Switch runs this 3-stage BT key exchange over USB so it
// can register the pad (Steam/hid-nintendo do not). The Switch only validates the shape, not the key contents. Each
// is the 31-byte reply body following the 0x21 input prefix + ack(0x81) + echoed-subcommand(0x01).
static const uint8_t BT_PAIR_2[31] = { // type 2: LTK exchange
	0x02, 0xE5, 0xC8, 0xE4, 0x92, 0x05, 0xFF, 0xC9, 0x8A, 0x7D, 0xEA,
	0x15, 0xF6, 0x19, 0xBA, 0x82, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const uint8_t BT_PAIR_3[31] = {
	0x03
}; // type 3: save pairing (all zero body)
// User-calibration SPI mirror (0x8000-0x80FF). A real Pro Controller stores the gyro/accel (and stick) calibration
// the Switch writes during "Calibrate Motion Controls" here, then reads it back and applies it -- that's how a
// resting IMU/neutral offset gets cancelled. Per-slot: each Pro Controller has its own SPI mirror, so each
// gets its own persisted file (NSLOT files; the same 0x100 bytes as before, partitioned by slot index).
#define SWCAL_FILE_BASE "/swimucal"
static uint8_t g_userCal[NSLOT][0x100];
static bool g_userCalLoaded[NSLOT] = { false, false, false, false };
static void swCalFileName(uint8_t slot, char *out)
{
	// "/swimucal0.bin" .. "/swimucal3.bin"
	const char *base = SWCAL_FILE_BASE;
	int i = 0;
	while (base[i]) {
		out[i] = base[i];
		i++;
	}
	out[i++] = (char)('0' + slot);
	out[i++] = '.';
	out[i++] = 'b';
	out[i++] = 'i';
	out[i++] = 'n';
	out[i] = '\0';
}
static void loadUserCal()
{
	for (int s = 0; s < NSLOT; s++) {
		memset(g_userCal[s], 0xFF, sizeof g_userCal[s]);
		char fn[16];
		swCalFileName((uint8_t)s, fn);
		File f(InternalFS);
		if (f.open(fn, FILE_O_READ)) {
			f.read(g_userCal[s], sizeof g_userCal[s]);
			f.close();
		}
		g_userCalLoaded[s] = true;
	}
}
static void saveUserCal(uint8_t slot)
{
	char fn[16];
	swCalFileName(slot, fn);
	InternalFS.remove(fn);
	File f(InternalFS);
	if (f.open(fn, FILE_O_WRITE)) {
		f.write(g_userCal[slot], sizeof g_userCal[slot]);
		f.close();
	}
}
// jcSpiWrite runs in the USB ISR (via jcSet). NEVER do flash I/O here -- a blocking LittleFS erase/write in the
// interrupt wedges USB + the RF poll and corrupts state (device drops into a bad state needing a replug). Only
// update the RAM mirror and flag it dirty; task() (main loop) does the actual save, debounced so a calibration
// write-burst coalesces into one flash write. Per-slot: each Pro Controller's ISR write targets its own mirror.
static volatile bool g_userCalDirty[NSLOT] = { false, false, false, false };
static volatile unsigned long g_userCalDirtyMs[NSLOT] = { 0 };
static void jcSpiWrite(uint8_t slot, uint32_t addr, uint8_t len,
		       const uint8_t *data, uint16_t avail)
{
	bool changed = false;
	for (uint8_t i = 0; i < len && i < avail; i++) {
		uint32_t a = addr + i;
		if (a >= 0x8000 && a < 0x8100) {
			if (g_userCal[slot][a - 0x8000] != data[i]) {
				g_userCal[slot][a - 0x8000] = data[i];
				changed = true;
			}
		}
	}
	if (changed) {
		g_userCalDirty[slot] = true;
		g_userCalDirtyMs[slot] = millis();
	} // defer the flash write to task()
}
static void spiRead(uint8_t slot, uint32_t addr, uint8_t len, uint8_t *dst)
{
	for (uint8_t i = 0; i < len; i++) {
		uint32_t a = addr + i;
		uint8_t v = 0xFF;
		if (a >= 0x6020 && a < 0x6020 + 24)
			v = SPI_IMU_CAL[a - 0x6020];
		else if (a >= 0x603D && a < 0x603D + 18)
			v = g_spiStickCal[a - 0x603D];
		else if (a >= 0x6050 && a < 0x6050 + 13)
			v = SPI_COLOR[a - 0x6050];
		else if (a >= 0x6080 && a < 0x6080 + 24)
			v = SPI_PARAMS1[a - 0x6080];
		else if (a >= 0x6098 && a < 0x6098 + 18)
			v = SPI_PARAMS2[a - 0x6098];
		else if (a >= 0x8000 && a < 0x8100)
			// persisted user cal (0xFF blank -> factory fallback)
			v = g_userCal[slot][a - 0x8000];
		dst[i] = v;
	}
}
// Reply FIFO: the host's handshake/subcommand reports arrive in the USB ISR (jcSet); we enqueue the canonical
// 0x81/0x21 reply and drain it from task() where sendReport is safe. SPSC ring, volatile indices. Per-slot: each
// Pro Controller runs its own handshake state machine, so each has its own reply queue. Every reply is sent as
// a full 63-byte report (64 with the id): macOS IOHIDManager silently DROPS any input report shorter than the
// descriptor-declared length (0x3F), so short 0x81/0x21 replies never reached the host.
#define JC_REPLEN 63
struct JcRep {
	uint8_t rid;
	uint8_t data[JC_REPLEN];
};
#define JCQ_N 8
static volatile uint8_t g_jcQh[NSLOT] = { 0, 0, 0, 0 };
static volatile uint8_t g_jcQt[NSLOT] = { 0, 0, 0, 0 };
static JcRep g_jcQ[NSLOT][JCQ_N];
static void jcEnq(uint8_t slot, uint8_t rid, const uint8_t *d, uint8_t len)
{
	uint8_t t = g_jcQt[slot];
	uint8_t nt = (uint8_t)((t + 1) % JCQ_N);
	if (nt == g_jcQh[slot])
		return; // full -> drop; the host re-requests on timeout
	if (len > JC_REPLEN)
		len = JC_REPLEN;
	g_jcQ[slot][t].rid = rid;
	memset(g_jcQ[slot][t].data, 0, JC_REPLEN);
	memcpy(g_jcQ[slot][t].data, d, len); // zero-pad to full report length
	g_jcQt[slot] = nt;
}
// Build the 0x21 reply (standard input prefix + ACK + echoed subcommand id + reply data) for a report-0x01 subcommand.
static void jcSubcmd(uint8_t slot, uint8_t sub, const uint8_t *args,
		     uint16_t alen)
{
	uint8_t p[JC_REPLEN];
	memset(p, 0, sizeof p);
	jcInputPrefix(slot, p);
	p[13] = sub;
	switch (sub) {
	// manual BT pairing: 3-stage key exchange a real Switch runs over USB
	case 0x01: {
		uint8_t t = (alen >= 1) ? args[0] : 3;
		p[12] = 0x81; // pairing ACK
		if (t == 1) {
			// echo type + this slot's MAC + "Pro Controller" name
			p[14] = 0x01;
			memcpy(&p[15], g_jcMac[slot], 6);
			p[21] = 0x00;
			p[22] = 0x25;
			p[23] = 0x08;
			// "Pro Controller" = 50 72 6F 20 43 6F 6E 74 72 6F 6C 6C 65 72
			static const uint8_t NAME[14] = {
				0x50, 0x72, 0x6F, 0x20, 0x43, 0x6F, 0x6E,
				0x74, 0x72, 0x6F, 0x6C, 0x6C, 0x65, 0x72
			};
			memcpy(&p[24], NAME, 14);
			p[38] = 0x00;
			p[39] = 0x00;
			p[40] = 0x00;
			p[41] = 0x00;
			p[42] = 0x00;
			p[43] = 0x68;
		} else if (t == 2) {
			memcpy(&p[14], BT_PAIR_2, 31);
		} else {
			memcpy(&p[14], BT_PAIR_3, 31);
		}
		break;
	}
	case 0x02: // request device info
		p[12] = 0x82;
		p[14] = 0x03;

		// firmware version 3.72 (genuine Pro Controller value)
		p[15] = 0x48;
		p[16] = 0x03; // controller type: Pro Controller
		p[17] = 0x02; // fixed
		memcpy(&p[18], g_jcMac[slot], 6);
		p[24] = 0x01; // colors stored in SPI (0x6050)
		p[25] = 0x01; // fixed
		break;
	case 0x10: { // SPI flash read -> echo [addr][len] then the data
		if (alen < 5) {
			p[12] = 0x80;
			break;
		}
		uint32_t a = (uint32_t)args[0] | ((uint32_t)args[1] << 8) |
			     ((uint32_t)args[2] << 16) |
			     ((uint32_t)args[3] << 24);
		uint8_t rl = args[4];
		if (rl > 0x1D)
			rl = 0x1D;
		p[12] = 0x90;
		p[14] = args[0];
		p[15] = args[1];
		p[16] = args[2];
		p[17] = args[3];
		p[18] = rl;
		spiRead(slot, a, rl, &p[19]);
		break;
	}
	// SPI flash write -> persist user calibration so Switch motion-cal sticks
	case 0x11:
		if (alen >= 5) {
			uint32_t a = (uint32_t)args[0] |
				     ((uint32_t)args[1] << 8) |
				     ((uint32_t)args[2] << 16) |
				     ((uint32_t)args[3] << 24);
			jcSpiWrite(slot, a, args[4], &args[5],
				   (alen > 5) ? (uint16_t)(alen - 5) : 0);
		}
		p[12] = 0x80; // write ACK
		break;

	// set input report mode (0x30 = standard full) -> begin streaming
	case 0x03:
		if (alen >= 1 && args[0] == 0x30)
			g_swProReportMode[slot] = 0x30;
		p[12] = 0x80;
		break;
	// trigger buttons elapsed time -> canned reply (genuine pad returns data)
	case 0x04:
		p[12] = 0x83;
		p[14] = 0x00;
		p[15] = 0xCC;
		p[16] = 0x00;
		p[17] = 0xEE;
		p[18] = 0x00;
		p[19] = 0xFF;
		break;
	case 0x21:
		p[12] = 0xA0;
		break; // set NFC/IR config
	default:
		p[12] = 0x80;

		// 0x06/0x08/0x30/0x38/0x40/0x41/0x48/... generic positive ACK
		break;
	}
	jcEnq(slot, 0x21, p, JC_REPLEN);
}
// USB-ISR callback for the Pro Controller's OUT endpoint. Per-slot dispatch via per-instance callback
// (jcSet##N closes over the slot index). Report 0x80 = USB handshake; 0x01 = subcommand.
static void jcSetCommon(uint8_t slot, uint8_t rid, hid_report_type_t type,
			uint8_t const *b, uint16_t n)
{
	if (type != HID_REPORT_TYPE_OUTPUT || n < 1)
		return;
	uint8_t id;
	const uint8_t *p;
	uint16_t pn;
	if (rid == 0) {
		id = b[0];
		p = b + 1;
		pn = (uint16_t)(n - 1);
	} // EP-OUT: id is the first payload byte
	else {
		id = rid;
		p = b;
		pn = n;
	} // control SET_REPORT: id already split out
	if (id == 0x80) {
		if (pn < 1)
			return;
		if (p[0] == 0x01) {
			uint8_t d[9] = { 0x01,
					 0x00,
					 0x03,
					 g_jcMac[slot][0],
					 g_jcMac[slot][1],
					 g_jcMac[slot][2],
					 g_jcMac[slot][3],
					 g_jcMac[slot][4],
					 g_jcMac[slot][5] };
			jcEnq(slot, 0x81, d, 9);
		} // device type + MAC
		else if (p[0] == 0x02) {
			uint8_t d[1] = { 0x02 };
			jcEnq(slot, 0x81, d, 1);
		} // handshake
		else if (p[0] == 0x03) {
			uint8_t d[1] = { 0x03 };
			jcEnq(slot, 0x81, d, 1);
		} // set baudrate
		// 0x04 force-USB / 0x05 enable-timeout / 0x06 reset: no reply expected
		return;
	}
	if (id == 0x01) { // [timer][rumble x8][subcmd][args...]
		if (pn < 10)
			return;
		jcRumble(slot, p, pn);
		jcSubcmd(slot, p[9], p + 10,
			 (pn > 10) ? (uint16_t)(pn - 10) : 0);
		return;
	}
	if (id == 0x10) {
		jcRumble(slot, p, pn);
		return;
	}
	// report 0x82: ignored
}
#define JCCB(N)                                                                \
	static void jcSet##N(uint8_t r, hid_report_type_t t, uint8_t const *b, \
			     uint16_t n)                                       \
	{                                                                      \
		jcSetCommon(N, r, t, b, n);                                    \
	}
// clang-format off
JCCB(0)
JCCB(1)
JCCB(2)
JCCB(3)
// clang-format on
typedef void (*jc_setcb_t)(uint8_t, hid_report_type_t, uint8_t const *,
			   uint16_t);
static jc_setcb_t const JC_SETCB[NSLOT] = { jcSet0, jcSet1, jcSet2, jcSet3 };

// Dynamic-mount mode: begin() is unused (setup() calls beginPool()+usbReenumerate instead).
void SwitchProController::begin()
{
}
// Wake mouse (1 HID) is present in Switch mode, leaving CFG_TUD_HID-1 for the Pro Controller pool.
uint8_t SwitchProController::maxSlots() const
{
	uint8_t cap = (uint8_t)(CFG_TUD_HID - 1);
	return cap < NSLOT ? cap : (uint8_t)NSLOT;
}
void SwitchProController::usbIdentity()
{
	USBDevice.setID(0x057E, 0x2009);
	USBDevice.setDeviceVersion(0x0220);
	USBDevice.setManufacturerDescriptor("Nintendo Co., Ltd.");
	USBDevice.setProductDescriptor("Pro Controller");
}
void SwitchProController::beginPool()
{
	jcBuildStickCal();
	loadUserCal();
	swProLoadCfg();
	initJcMacs();
	uint8_t pool = maxSlots();
	for (uint8_t s = 0; s < pool; s++) {
		g_swPro[s].enableOutEndpoint(true);
		g_swPro[s].setReportCallback(NULL, JC_SETCB[s]);
		g_swPro[s].setReportDescriptor(SWPRO_HID_DESC,
					       sizeof SWPRO_HID_DESC);
		g_swPro[s].setPollInterval(1);
		g_swPro[s].begin();
	}
}
void SwitchProController::mountSlots(uint8_t k)
{
	for (uint8_t u = 0; u < k; u++) {
		// Each re-enumeration restarts the host handshake -- clear this USB slot's gate + reply FIFO so we
		// don't stream 0x30 before the (new) host has re-selected report mode.
		g_swProReportMode[u] = 0;
		g_jcQh[u] = g_jcQt[u] = 0;
		USBDevice.addInterface(g_swPro[u]);
	}
}
void SwitchProController::task()
{
	for (uint8_t s = 0; s < g_usbMountCount; s++) {
		if (!g_swPro[s].ready())
			continue;
		// Deferred user-cal flash write (queued by the USB ISR; debounced so a calibration write-burst
		// is one save). Per-slot: each Pro Controller saves its own mirror.
		if (g_userCalDirty[s] &&
		    (unsigned long)(millis() - g_userCalDirtyMs[s]) > 250u) {
			g_userCalDirty[s] = false;
			saveUserCal((uint8_t)s);
		}
		// drain handshake/subcommand replies first (ordered) -- one report per slot per call so a
		// bursty host-init doesn't starve the streamed 0x30
		if (g_jcQh[s] != g_jcQt[s]) {
			JcRep *r = &g_jcQ[s][g_jcQh[s]];
			usbTxHid(&g_swPro[s], r->rid, r->data, JC_REPLEN);
			g_jcQh[s] = (uint8_t)((g_jcQh[s] + 1) % JCQ_N);
			// one report per slot per call; the rest next loop
			continue;
		}
		// not until the host has finished init + selected 0x30
		if (g_swProReportMode[s] != 0x30)
			continue;
		// The Switch integrates the report's 3 IMU samples by SAMPLE COUNT at a fixed ~5 ms/sample (it ignores
		// the timer byte and assumes a 3-samples-per-15ms genuine cadence). Streaming faster (e.g. 250 Hz x 3
		// = 750 samples/s) over-credits gyro rotation ~3.75x, so residual bias accumulates into the slow
		// orientation lean that builds over minutes and resets on replug. 15 ms matches the genuine push,
		// making integration 1:1.
		uint32_t interval = (g_swProRate == 2) ? USB_STREAM_MS :
				    (g_swProRate == 1) ? SW_PRO_REPORT_MS_120 :
							 SW_PRO_REPORT_MS;
		if (millis() - g_swProLastMs[s] < interval)
			continue;
		g_swProLastMs[s] = millis();
		uint8_t p[63];
		switchProBuild((uint8_t)s, p);
		usbTxHid(&g_swPro[s], 0x30, p, sizeof p);
	}
}
