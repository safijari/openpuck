#include "mode_ps5_auth.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "bonds.h"
#include "status_led.h"
#include "haptics.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <SPI.h>
#include <Arduino.h>
#include <string.h>

using namespace Adafruit_LittleFS_Namespace;

Ps5AuthController g_ps5AuthCtl;

// MAX3421E wiring, as Feather-variant Arduino pin numbers (the build uses that variant). The defaults are
// the contiguous D2..D5 run on a Pro Micro / nice!nano clone. The Feather's default SPI pins are avoided on
// purpose: its MISO is P0.15, the SuperMini's user LED.
#ifndef P5_PIN_SCK
#define P5_PIN_SCK 29 // P0.17, Pro Micro D2
#endif
#ifndef P5_PIN_MOSI
#define P5_PIN_MOSI 28 // P0.20, Pro Micro D3
#endif
#ifndef P5_PIN_MISO
#define P5_PIN_MISO 30 // P0.22, Pro Micro D4
#endif
#ifndef P5_PIN_CS
#define P5_PIN_CS 1 // P0.24, Pro Micro D5
#endif

// Adafruit_USBH_Host::begin() insists on an INT pin and hooks it to the GPIOTE ISR, which would run SPI
// transactions in interrupt context and jitter the RF poll. We detach it again and poll the MAX3421E from
// loop() instead, so the pin can stay unconnected.
#ifndef P5_PIN_INT
#define P5_PIN_INT 23 // P0.11, Pro Micro D7
#endif

// TinyUSB addresses the MAX3421E as root port 1; port 0 is the native USBD facing the console.
#define P5_RHPORT 1

#define P5_VID 0x2B81
#define P5_PID 0x0101

#define P5_REPORT_LEN 64
#define P5_TOUCH_H 943

// A dongle control transfer that hasn't completed in this window is treated as lost so a dropped completion
// can't wedge the auth relay until replug.
#define P5_WAIT_MS 500u

// The adapter needs ~500 ms after the final challenge page before its F2 status reads "signed"; asking sooner
// returns not-ready, and the console only polls F2 a few times.
#define P5_F2_DELAY_MS 500u

// A signed-report round trip normally takes a couple of USB frames; past this we assume it was lost.
#define P5_HASH_TIMEOUT_MS 50u

#define P5_CONN_MS 1200u

// The adapter's own HID report descriptor (identical to what it reports when plugged into a PC). Input 0x01
// carries 52 vendor bytes that hold the auth sequence number and hash; F0/F1/F2 are the auth channel.
static const uint8_t P5_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31,
	0x09, 0x32, 0x09, 0x35, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xFF,
	0x00, 0x75, 0x08, 0x95, 0x06, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x20,
	0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07,
	0x35, 0x00, 0x46, 0x3B, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81,
	0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0E, 0x15, 0x00, 0x25,
	0x01, 0x75, 0x01, 0x95, 0x0E, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x21,
	0x95, 0x0E, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x22, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x34, 0x81, 0x02, 0x85, 0x02, 0x09, 0x23,
	0x95, 0x2F, 0x91, 0x02, 0x85, 0x03, 0x0A, 0x21, 0x28, 0x95, 0x2F, 0xB1,
	0x02, 0x06, 0x80, 0xFF, 0x85, 0xE0, 0x09, 0x57, 0x95, 0x02, 0xB1, 0x02,
	0xC0, 0x06, 0xF0, 0xFF, 0x09, 0x40, 0xA1, 0x01, 0x85, 0xF0, 0x09, 0x47,
	0x95, 0x3F, 0xB1, 0x02, 0x85, 0xF1, 0x09, 0x48, 0x95, 0x3F, 0xB1, 0x02,
	0x85, 0xF2, 0x09, 0x49, 0x95, 0x0F, 0xB1, 0x02, 0xC0
};

// Feature 0x03 "controller definition", payload after the report id. Byte-identical to what the adapter
// itself returns; the console reads it at enumeration to classify the pad.
// Controller definition (PS4ControllerConfig layout). Byte 3 is the capability mask the console honours:
// 0x02 motion, 0x04 lightbar, 0x08 vibration, 0x40 touchpad. The adapter's own 0xC3 leaves vibration off,
// so the console never sends motor values; 0xCF turns vibration and lightbar on.
static const uint8_t P5_DEF_03[47] = {
	0x21, 0x28, 0x03, 0xCF, 0x00, 0x2C, 0x56, 0x01, 0x00, 0xD0,
	0x07, 0x00, 0x80, 0x04, 0x00, 0x00, 0x80, 0x0D, 0x0D, 0x84,
};

enum : uint8_t {
	RID_DEF = 0x03,
	RID_CHALLENGE = 0xF0,
	RID_SIGNATURE = 0xF1,
	RID_STATUS = 0xF2,
};

enum : uint8_t {
	A_IDLE,
	A_SEND_F0,
	A_SEND_F0_WAIT,
	A_RECV_F1,
	A_RECV_F1_WAIT,
	A_F2_DELAY,
	A_RECV_F2,
	A_RECV_F2_WAIT,
};

static Adafruit_USBD_HID g_p5;
static SPIClass s_spi(NRF_SPIM2, P5_PIN_MISO, P5_PIN_SCK, P5_PIN_MOSI);
static Adafruit_USBH_Host s_host(&s_spi, P5_PIN_CS, P5_PIN_INT);
static bool s_hostUp = false;
static uint8_t s_chipRev = 0;
// Bring-up diagnostics, readable through the 0xEE report.
static uint8_t s_rev8 = 0, s_oscIrq = 0, s_oscOk = 0, s_oscOk2 = 0;

// Host bring-up progress (0 = not started), readable through the 0xEE diagnostic report.
static volatile uint8_t s_stage = 0;

static uint8_t s_dev = 0xFF, s_inst = 0xFF;
static bool s_ready = false;

// Auth relay. The console's GET/SET_REPORT callbacks run on the usbd task and only ever move s_state out of
// A_IDLE; loop() owns every other transition. s_auth ([0] = report id) is touched from both, under a
// critical section. Mirrors GP2040-CE's one-deep prefetch: the console's read of F1/F2 is answered from
// s_auth and triggers fetching the next F1 page from the adapter.
static volatile uint8_t s_state = A_IDLE;
static uint8_t s_auth[P5_REPORT_LEN];
static uint8_t s_hostBuf[P5_REPORT_LEN];
static uint8_t s_f1Left = 0;
static uint32_t s_waitMs = 0, s_f2DueMs = 0;

// Signing pipeline: one input report in flight to the adapter at a time.
static uint8_t s_plain[P5_REPORT_LEN];
static bool s_hashInFlight = false;
static uint32_t s_hashMs = 0, s_lastSendMs = 0, s_ledMs = 0;

// Signing latency stats for the 0xEE diagnostic, in us, reset on every read: adapter round trip
// (send -> signed copy back) and the longest gap between task() calls (how promptly the host is serviced).
static uint32_t s_sendUs, s_taskUs;
static volatile uint32_t s_rttMin = 0xFFFFFFFF, s_rttMax, s_rttSum, s_rttN,
			 s_taskGapMax;

static int p5ActiveSlot()
{
	unsigned long now = millis();
	for (int s = 0; s < NSLOT; s++)
		if (g_slot[s].used && g_connReplyMs[s] &&
		    (now - g_connReplyMs[s]) < P5_CONN_MS)
			return s;
	return -1;
}

static bool p5IsAdapter(uint8_t dev, uint8_t inst)
{
	return s_ready && dev == s_dev && inst == s_inst;
}

// ---- console side (usbd task) ----

static uint16_t p5AuthRead(uint8_t *buf, uint16_t reqlen, uint16_t n)
{
	if (n > reqlen)
		n = reqlen;
	taskENTER_CRITICAL();
	memcpy(buf, s_auth + 1, n);
	if (s_state == A_IDLE)
		s_state = A_RECV_F1;
	taskEXIT_CRITICAL();
	return n;
}

// ---- console output capture (diagnostic) ----
// Records what the console sends besides the F0 challenge (output reports, other SET_REPORTs) so a
// rumble layout can be read back on a computer after the puck has been on a PS5. Slots 0-3 keep the
// first reports, 4-7 a ring of the latest distinct ones. Saved to flash only on boots where a console
// challenged us, so a later bench boot doesn't overwrite it; GET_REPORT 0xD0-0xD7 (entries) and 0xD8
// (counters) return the copy loaded at boot.
#define P5_CAP_N 8
#define P5_CAP_DATA 48
#define P5_CAP_FILE "/p5cap.bin"
#define P5_CAP_SAVE_MS 3000u

struct P5CapEnt {
	uint8_t rid, type, len, used;
	uint8_t d[P5_CAP_DATA];
};
struct P5Cap {
	uint32_t nOut, nOther;
	uint8_t ring;
	P5CapEnt e[P5_CAP_N];
};
static P5Cap s_cap, s_capSaved;
static volatile bool s_capDirty, s_sawChallenge;
static unsigned long s_capSaveMs;

static void p5CapStore(P5CapEnt *e, uint8_t rid, hid_report_type_t type,
		       uint8_t const *b, uint16_t n)
{
	e->rid = rid;
	e->type = (uint8_t)type;
	e->len = n > 255 ? 255 : (uint8_t)n;
	e->used = 1;
	memset(e->d, 0, P5_CAP_DATA);
	memcpy(e->d, b, n < P5_CAP_DATA ? n : P5_CAP_DATA);
}

static void p5Capture(uint8_t rid, hid_report_type_t type, uint8_t const *b,
		      uint16_t n)
{
	taskENTER_CRITICAL();
	if (type == HID_REPORT_TYPE_OUTPUT)
		s_cap.nOut++;
	else
		s_cap.nOther++;
	int k;
	for (k = 0; k < 4 && s_cap.e[k].used; k++)
		;
	if (k < 4) {
		p5CapStore(&s_cap.e[k], rid, type, b, n);
		s_capDirty = true;
	} else {
		P5CapEnt *last = &s_cap.e[4 + (s_cap.ring + 3) % 4];
		uint16_t m = n < P5_CAP_DATA ? n : P5_CAP_DATA;
		if (!last->used || last->rid != rid || last->len != n ||
		    memcmp(last->d, b, m)) {
			p5CapStore(&s_cap.e[4 + s_cap.ring], rid, type, b, n);
			s_cap.ring = (s_cap.ring + 1) % 4;
			s_capDirty = true;
		}
	}
	taskEXIT_CRITICAL();
}

static void p5CapLoad()
{
	File f(InternalFS);
	if (f.open(P5_CAP_FILE, FILE_O_READ)) {
		if (f.read(&s_capSaved, sizeof s_capSaved) != sizeof s_capSaved)
			memset(&s_capSaved, 0, sizeof s_capSaved);
		f.close();
	}
}

static void p5CapSave()
{
	if (!s_capDirty || !s_sawChallenge ||
	    millis() - s_capSaveMs < P5_CAP_SAVE_MS)
		return;
	static P5Cap snap;
	taskENTER_CRITICAL();
	snap = s_cap;
	s_capDirty = false;
	taskEXIT_CRITICAL();
	s_capSaveMs = millis();
	InternalFS.remove(P5_CAP_FILE);
	File f(InternalFS);
	if (f.open(P5_CAP_FILE, FILE_O_WRITE)) {
		f.write((const uint8_t *)&snap, sizeof snap);
		f.close();
	}
}

static uint16_t p5CapRead(uint8_t rid, uint8_t *buf, uint16_t reqlen)
{
	uint8_t tmp[4 + P5_CAP_DATA];
	uint16_t n;
	if (rid == 0xD8) {
		memcpy(tmp, &s_capSaved.nOut, 4);
		memcpy(tmp + 4, &s_capSaved.nOther, 4);
		n = 8;
	} else {
		P5CapEnt const *e = &s_capSaved.e[rid - 0xD0];
		tmp[0] = e->used;
		tmp[1] = e->rid;
		tmp[2] = e->type;
		tmp[3] = e->len;
		memcpy(tmp + 4, e->d, P5_CAP_DATA);
		n = sizeof tmp;
	}
	if (n > reqlen)
		n = reqlen;
	memcpy(buf, tmp, n);
	return n;
}

static uint16_t p5GetReport(uint8_t rid, hid_report_type_t type, uint8_t *buf,
			    uint16_t reqlen)
{
	if (type != HID_REPORT_TYPE_FEATURE || !buf || reqlen == 0)
		return 0;
	switch (rid) {
	case RID_DEF: {
		uint16_t n = sizeof P5_DEF_03 < reqlen ? sizeof P5_DEF_03 :
							 reqlen;
		memcpy(buf, P5_DEF_03, n);
		return n;
	}
	case RID_SIGNATURE:
		return p5AuthRead(buf, reqlen, 63);
	case RID_STATUS:
		return p5AuthRead(buf, reqlen, 15);
	case 0xD0 ... 0xD8:
		return p5CapRead(rid, buf, reqlen);
	case 0xEE:
		// Bench diagnostic for tools/p5_check.py; not in the descriptor, so a console never asks for it.
		buf[0] = s_chipRev;
		if (reqlen < 3)
			return 1;
		buf[1] = s_hostUp;
		buf[2] = s_ready;
		if (reqlen < 4)
			return 3;
		buf[3] = s_stage;
		if (reqlen < 8)
			return 4;
		buf[4] = s_rev8;
		buf[5] = s_oscIrq;
		buf[6] = s_oscOk;
		buf[7] = s_oscOk2;
		if (reqlen < 28)
			return 8;
		{
			uint32_t st[5] = { s_rttN, s_rttN ? s_rttMin : 0,
					   s_rttN ? s_rttSum / s_rttN : 0,
					   s_rttMax, s_taskGapMax };
			memcpy(buf + 8, st, sizeof st);
			s_rttMin = 0xFFFFFFFF;
			s_rttMax = s_rttSum = s_rttN = s_taskGapMax = 0;
		}
		return 28;
	default:
		return 0;
	}
}

// Output report 0x02 has the DualSense's 47-byte layout: [0] valid_flag0, [2] high-frequency (right) motor,
// [3] low-frequency (left) motor. flag0 bits 0/1 mark the motor bytes valid; a report without them is a
// lightbar/LED-only update whose zero motor bytes must not cancel a running rumble. Some hosts never set
// the flags at all, so the gate only engages once a flagged report has been seen.
static bool s_rumbleFlagsSeen = false;

static void p5Rumble(uint8_t rid, uint8_t const *b, uint16_t n)
{
	if (rid == 0) {
		if (n < 1 || b[0] != 0x02)
			return;
		b++;
		n--;
	} else if (rid != 0x02) {
		return;
	}
	if (n < 4)
		return;
	if (b[0] & 0x03)
		s_rumbleFlagsSeen = true;
	else if (s_rumbleFlagsSeen)
		return;
	int slot = p5ActiveSlot();
	if (slot >= 0)
		hapticSteamRumble((uint16_t)b[3] * 257u, (uint16_t)b[2] * 257u,
				  (uint8_t)slot);
}

static void p5SetReport(uint8_t rid, hid_report_type_t type, uint8_t const *b,
			uint16_t n)
{
	if (type == HID_REPORT_TYPE_OUTPUT) {
		p5Capture(rid, type, b, n);
		p5Rumble(rid, b, n);
		return;
	}
	if (type != HID_REPORT_TYPE_FEATURE || rid != RID_CHALLENGE) {
		p5Capture(rid, type, b, n);
		return;
	}
	if (n != 63)
		return;
	s_sawChallenge = true;
	taskENTER_CRITICAL();
	if (s_state == A_IDLE) {
		s_auth[0] = RID_CHALLENGE;
		memcpy(s_auth + 1, b, 63);
		s_state = A_SEND_F0;
	}
	taskEXIT_CRITICAL();
}

// ---- adapter side (TinyUSB host callbacks, dispatched from s_host.task() in loop) ----

extern "C" {

void tuh_hid_mount_cb(uint8_t dev, uint8_t inst, uint8_t const *desc,
		      uint16_t len)
{
	(void)desc;
	(void)len;
	if (s_ready || g_usbMode != MODE_PS5_AUTH)
		return;
	uint16_t vid = 0, pid = 0;
	tuh_vid_pid_get(dev, &vid, &pid);
	if (vid != P5_VID || pid != P5_PID)
		return;
	s_dev = dev;
	s_inst = inst;
	s_ready = true;
	s_hashInFlight = false;
	s_state = A_IDLE;
	tuh_hid_receive_report(dev, inst);
}

void tuh_hid_umount_cb(uint8_t dev, uint8_t inst)
{
	if (!p5IsAdapter(dev, inst))
		return;
	s_ready = false;
	s_dev = s_inst = 0xFF;
	s_hashInFlight = false;
	s_state = A_IDLE;
}

void tuh_hid_report_received_cb(uint8_t dev, uint8_t inst,
				uint8_t const *report, uint16_t len)
{
	if (!p5IsAdapter(dev, inst))
		return;
	// The adapter's only input report is the signed copy of what we sent; byte 0 is its report id.
	if (s_hashInFlight && len == P5_REPORT_LEN && report[0] == 0x01) {
		uint32_t rtt = micros() - s_sendUs;
		if (rtt < s_rttMin)
			s_rttMin = rtt;
		if (rtt > s_rttMax)
			s_rttMax = rtt;
		s_rttSum += rtt;
		s_rttN++;
		usbTxHid(&g_p5, report[0], report + 1, len - 1);
		s_hashInFlight = false;
	}
	tuh_hid_receive_report(dev, inst);
}

void tuh_hid_set_report_complete_cb(uint8_t dev, uint8_t inst, uint8_t rid,
				    uint8_t type, uint16_t len)
{
	(void)type;
	if (!p5IsAdapter(dev, inst) || rid != RID_CHALLENGE || len == 0 ||
	    s_state != A_SEND_F0_WAIT)
		return;
	// s_auth[1] is the challenge kind and s_auth[3] its page index. Kind 1 is a 4-page challenge whose
	// signature comes back as 4 F1 pages; kind 3 has a 1-page answer; kind 2 has only a status.
	uint8_t kind = s_auth[1];
	s_f1Left = kind == 0x01 ? 4 : kind == 0x03 ? 1 : 0;
	bool lastPage = (kind == 0x01 && s_auth[3] == 3) || kind == 0x02 ||
			kind == 0x03;
	if (lastPage) {
		s_f2DueMs = millis() + P5_F2_DELAY_MS;
		s_state = A_F2_DELAY;
	} else {
		s_state = A_IDLE;
	}
}

void tuh_hid_get_report_complete_cb(uint8_t dev, uint8_t inst, uint8_t rid,
				    uint8_t type, uint16_t len)
{
	(void)type;
	if (!p5IsAdapter(dev, inst) || len == 0)
		return;
	bool want = (rid == RID_SIGNATURE && s_state == A_RECV_F1_WAIT) ||
		    (rid == RID_STATUS && s_state == A_RECV_F2_WAIT);
	if (!want)
		return;
	if (len > P5_REPORT_LEN)
		len = P5_REPORT_LEN;
	taskENTER_CRITICAL();
	memcpy(s_auth, s_hostBuf, len);
	s_state = A_IDLE;
	taskEXIT_CRITICAL();
}

} // extern "C"

static void p5AuthStep()
{
	uint32_t now = millis();
	switch (s_state) {
	case A_SEND_F0:
		memcpy(s_hostBuf, s_auth, P5_REPORT_LEN);
		if (tuh_hid_set_report(s_dev, s_inst, RID_CHALLENGE,
				       HID_REPORT_TYPE_FEATURE, s_hostBuf,
				       P5_REPORT_LEN)) {
			s_state = A_SEND_F0_WAIT;
			s_waitMs = now;
		}
		break;
	case A_RECV_F1:
		if (!s_f1Left) {
			s_state = A_IDLE;
			break;
		}
		if (tuh_hid_get_report(s_dev, s_inst, RID_SIGNATURE,
				       HID_REPORT_TYPE_FEATURE, s_hostBuf,
				       P5_REPORT_LEN)) {
			s_state = A_RECV_F1_WAIT;
			s_waitMs = now;
			s_f1Left--;
		}
		break;
	case A_F2_DELAY:
		if ((int32_t)(now - s_f2DueMs) < 0)
			break;
		s_state = A_RECV_F2;
		// fall through
	case A_RECV_F2:
		if (tuh_hid_get_report(s_dev, s_inst, RID_STATUS,
				       HID_REPORT_TYPE_FEATURE, s_hostBuf,
				       16)) {
			s_state = A_RECV_F2_WAIT;
			s_waitMs = now;
		}
		break;
	case A_SEND_F0_WAIT:
	case A_RECV_F1_WAIT:
	case A_RECV_F2_WAIT:
		// A late completion is ignored: its callback only acts while still in the matching wait state.
		if (now - s_waitMs >= P5_WAIT_MS)
			s_state = A_IDLE;
		break;
	default:
		break;
	}
}

// The plain report the adapter signs. Bytes 12..15 (sequence) and 56..63 (hash) are filled in by the adapter.
static void p5Build(int slot, uint8_t r[P5_REPORT_LEN])
{
	memset(r, 0, P5_REPORT_LEN);
	r[0] = 0x01;
	r[30] = 0x1A;
	if (slot < 0) {
		r[1] = r[2] = r[3] = r[4] = 0x80;
		r[8] = 0x0F;
		touchPackPoint(r + 32, 0, false, 0, 0);
		touchPackPoint(r + 32, 1, false, 0, 0);
		return;
	}
	uint32_t b = psButtonsFromSteam(g_in[slot].buttons);
	psPadClickEdge((uint8_t)slot, (b & (TB_LPADC | TB_RPADC)) != 0);
	int16_t lx, ly, rx, ry;
	slotSticks((uint8_t)slot, &lx, &ly, &rx, &ry);
	r[1] = swStick(lx, false);
	r[2] = swStick(ly, true);
	r[3] = swStick(rx, false);
	r[4] = swStick(ry, true);
	r[5] = g_in[slot].lt;
	r[6] = g_in[slot].rt;

	// GP2040-CE's proven P5General reports use 0xF for a released hat, not the DS4's 8.
	uint8_t hat = psHatNibble(b);
	r[8] = (hat == 8 ? 0x0F : hat) | psFaceNibble(b);
	r[9] = psShouldersByte(b, g_in[slot].lt, g_in[slot].rt);
	r[10] = ((b & TB_STEAM) ? 0x01 : 0) |
		((b & (TB_TOUCH | TB_LPADC | TB_RPADC)) ? 0x02 : 0);

	// Little-endian, as on a DualSense. GP2040-CE byte-swaps here, but big-endian sends motion haywire on a
	// real PS5.
	PsImuFrame imu = psImuFromSteam(g_in[slot]);
	// P5_DEF_03 declares accel as 4 g full scale (8192/g); the SC2 reports 16384/g, so halve it or the
	// console sees 2 g of gravity. The gyro (16/deg/s) already matches the declared 2000 deg/s.
	const int16_t v[6] = { imu.gx,
			       imu.gy,
			       imu.gz,
			       (int16_t)(imu.ax / 2),
			       (int16_t)(imu.ay / 2),
			       (int16_t)(imu.az / 2) };
	for (int i = 0; i < 6; i++) {
		r[16 + 2 * i] = (uint8_t)(v[i] & 0xFF);
		r[17 + 2 * i] = (uint8_t)((uint16_t)v[i] >> 8);
	}

	// Bytes 28-29 are a 16-bit IMU timestamp the console integrates the gyro over (GP2040-CE leaves it zero,
	// which kills rotation). Ticks are assumed to be 1 us; the DualSense's 0.33 us ticks spun rotation ~3x
	// too fast. Byte 30 is the battery/status byte, 0x1A = charging, full, as the P5General sends it.
	uint16_t ts = (uint16_t)g_in[slot].imuTimestampUs;
	r[28] = (uint8_t)ts;
	r[29] = (uint8_t)(ts >> 8);

	bool lTouch = g_padStick[0] == PS_OFF &&
		      ((b & TB_LPADT) || (b & TB_LPADC)),
	     rTouch = g_padStick[1] == PS_OFF &&
		      ((b & TB_RPADT) || (b & TB_RPADC));
	uint16_t tlx = 0, tly = 0, trx = 0, tryy = 0;
	if (lTouch || rTouch)
		steamPadsToTouch(b, P5_TOUCH_H, g_in[slot].lpx, g_in[slot].lpy,
				 g_in[slot].rpx, g_in[slot].rpy, &tlx, &tly,
				 &trx, &tryy);
	touchPackPadsStateful((uint8_t)slot, r + 32, lTouch, rTouch, tlx, tly,
			      trx, tryy);
}

static void p5SignStep()
{
	uint32_t now = millis();
	if (s_hashInFlight) {
		if (now - s_hashMs < P5_HASH_TIMEOUT_MS)
			return;
		s_hashInFlight = false;
	}
	// Sign a report as soon as a new controller frame lands (each carries a fresh IMU timestamp, so the
	// struct always differs) instead of on a free-running 4 ms tick, which added up to 4 ms of wait on
	// top of the adapter's ~2 ms signing round trip. The tick stays as the keep-alive for an idle link.
	static PuckInput lastIn[NSLOT];
	int slot = p5ActiveSlot();
	bool fresh = slot >= 0 &&
		     memcmp(&g_in[slot], &lastIn[slot], sizeof(PuckInput)) != 0;
	if ((!fresh && now - s_lastSendMs < USB_STREAM_MS) ||
	    !tuh_hid_send_ready(s_dev, s_inst))
		return;
	if (slot >= 0)
		lastIn[slot] = g_in[slot];
	p5Build(slot, s_plain);
	if (tuh_hid_send_report(s_dev, s_inst, 0, s_plain, P5_REPORT_LEN)) {
		s_hashInFlight = true;
		s_hashMs = s_lastSendMs = now;
		s_sendUs = micros();
	}
}

// Bring-up status on the user LED (no CDC/WebUSB in this clean mode). Each pulse lasts 500 ms, so:
// even 1 Hz blink = MAX3421E not answering on SPI, slow blink = chip up but no adapter, solid = adapter up.
static void p5Led()
{
	uint32_t now = millis();
	uint32_t period = s_ready ? 250u : s_hostUp ? 2000u : 1000u;
	if (now - s_ledMs >= period) {
		s_ledMs = now;
		ledWakePulse();
	}
}

void Ps5AuthController::usbIdentity()
{
	USBDevice.setID(P5_VID, P5_PID);
	USBDevice.setVersion(0x0200);
	USBDevice.setDeviceVersion(0x0001);
	USBDevice.setManufacturerDescriptor("Activtor");
	USBDevice.setProductDescriptor("P5General");
}

// MAX3421E SPI command byte = reg << 3, bit 1 set for a write. After reset the chip is in half-duplex SPI
// and doesn't drive MISO, so PINCTL (17) must get FDUPSPI before REVISION (18) can be read back.
static uint8_t p5Reg(uint8_t reg, bool write, uint8_t val,
		     uint32_t hz = 4000000)
{
	s_spi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
	digitalWrite(P5_PIN_CS, LOW);
	s_spi.transfer((uint8_t)(reg << 3 | (write ? 0x02 : 0)));
	uint8_t r = s_spi.transfer(val);
	digitalWrite(P5_PIN_CS, HIGH);
	s_spi.endTransaction();
	return r;
}

// Polls USBIRQ (13) for OSCOKIRQ (bit 0) for up to 50 ms after a chip reset.
static bool p5WaitOsc()
{
	uint32_t t0 = millis();
	do {
		s_oscIrq = p5Reg(13, false, 0);
		if (s_oscIrq & 0x01)
			return true;
	} while (millis() - t0 < 50);
	return false;
}

static uint8_t p5ProbeChip()
{
	pinMode(P5_PIN_CS, OUTPUT);
	digitalWrite(P5_PIN_CS, HIGH);
	s_spi.begin();
	// FDUPSPI | INTLEVEL
	p5Reg(17, true, 0x18);
	uint8_t rev = p5Reg(18, false, 0);
	s_rev8 = p5Reg(18, false, 0, 8000000);
	// Same reset TinyUSB's hcd_init does (USBCTL.CHIPRES pulse), then the wait it does without a timeout.
	p5Reg(15, true, 0x20);
	p5Reg(15, true, 0x00);
	s_oscOk = p5WaitOsc();
	if (!s_oscOk) {
		p5Reg(17, true, 0x18);
		s_oscOk2 = p5WaitOsc();
	}
	s_spi.end();
	return rev;
}

void Ps5AuthController::begin()
{
	usbIdentity();
	p5CapLoad();
	g_p5.enableOutEndpoint(true);
	g_p5.setReportCallback(p5GetReport, p5SetReport);
	g_p5.setReportDescriptor(P5_HID_DESC, sizeof P5_HID_DESC);
	g_p5.setPollInterval(1);
	g_p5.begin();
}

// Host bring-up runs from loop() a few seconds after boot rather than in begin(): begin() runs before the
// console-side USB attach, so anything that stalls here would leave the puck invisible to the console.
// TinyUSB's MAX3421E init also spins forever waiting for the oscillator-OK bit, so the chip's revision
// register is read first and the chip is only handed to TinyUSB if it is really there.
#define P5_HOST_START_MS 3000u

static void p5HostStart()
{
	s_stage = 1;
	s_chipRev = p5ProbeChip();
	s_stage = 2;
	if ((s_chipRev != 0x12 && s_chipRev != 0x13) || !s_oscOk)
		return;
	s_stage = 3;
	s_hostUp = s_host.begin(P5_RHPORT);
	s_stage = 4;
	if (s_hostUp)
		detachInterrupt(P5_PIN_INT);
	s_stage = 5;
}

void Ps5AuthController::task()
{
	uint32_t nowUs = micros();
	if (s_taskUs && nowUs - s_taskUs > s_taskGapMax)
		s_taskGapMax = nowUs - s_taskUs;
	s_taskUs = nowUs;
	p5Led();
	p5CapSave();
	if (!s_stage && millis() >= P5_HOST_START_MS)
		p5HostStart();
	if (!s_hostUp)
		return;
	tuh_int_handler(P5_RHPORT, false);
	s_host.task(0, false);
	if (!s_ready)
		return;
	p5AuthStep();
	p5SignStep();
}
