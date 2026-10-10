#include "mode_ps5_audio.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_mount.h"
#include "fault_diag.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

// One audio function of three interfaces, like a real DualSense: control, 4ch speaker/haptics out, 2ch mic in.
// The mic carries only silence, but Linux needs it: Sony's stock UCM profile opens the capture PCM and
// PipeWire rejects the whole profile (and its Direct 4ch sink) when that fails.
#define UAC1_IAD_LEN 8
// IAD(8)+AC_std(9)+CS_hdr(10)+speaker IT(12)+FU(12)+OT(9)+mic IT(12)+mic FU(9)+mic OT(9)
#define UAC1_AC_DESC_LEN 90
// Each AS interface: alt0(9)+alt1(9)+CS_general(7)+format(11)+std_ep(9)+CS_ep(7)
#define UAC1_AS_DESC_LEN 52
#define UAC1_DESC_LEN (UAC1_AC_DESC_LEN + 2 * UAC1_AS_DESC_LEN)
// Max packet sizes as a real pad declares them: one 8-byte frame (4ch * 16 bit) and one 4-byte frame (2ch) more
// than the 48 frames of a 1 ms packet at 48 kHz.
#define UAC1_ISO_EP_BUFSIZE 392
#define UAC1_ISO_IN_EP_BUFSIZE 196
// The mic sends exactly 48 frames per packet.
#define UAC1_ISO_IN_PACKET 192

static uint8_t g_uac1ItfAc = 0xFF;
static uint8_t g_uac1ItfAs = 0xFF;
static uint8_t g_uac1ItfAsIn = 0xFF;
static uint8_t g_uac1EpOut = 0x08;
static uint8_t g_uac1EpIn = 0x88;
static uint8_t g_uac1AltSetting = 0;
static uint8_t g_uac1AltSettingIn = 0;

CFG_TUD_MEM_SECTION static uint32_t g_isoOutBuf32[UAC1_ISO_EP_BUFSIZE / 4];
static uint8_t *g_isoOutBuf = (uint8_t *)g_isoOutBuf32;
// Never written: every mic packet is silence.
CFG_TUD_MEM_SECTION static uint32_t g_isoInBuf32[UAC1_ISO_IN_EP_BUFSIZE / 4];
static uint8_t *g_isoInBuf = (uint8_t *)g_isoInBuf32;

Adafruit_USBD_Audio_UAC1 g_ps5Audio;

Adafruit_USBD_Audio_UAC1::Adafruit_USBD_Audio_UAC1()
{
}

uint16_t Adafruit_USBD_Audio_UAC1::getInterfaceDescriptor(uint8_t itfnum,
							  uint8_t *buf,
							  uint16_t bufsize)
{
	(void)itfnum;
	if (!buf)
		return UAC1_DESC_LEN;
	if (bufsize < UAC1_DESC_LEN)
		return 0;

	uint8_t ac_itf = TinyUSBDevice.allocInterface(3);
	uint8_t as_out = (uint8_t)(ac_itf + 1);
	uint8_t as_in = (uint8_t)(ac_itf + 2);
	g_uac1ItfAc = ac_itf;
	g_uac1ItfAs = as_out;
	g_uac1ItfAsIn = as_in;

	const uint8_t desc[UAC1_DESC_LEN] = {
		// Interface Association Descriptor (IAD) - 8 bytes
		8, TUSB_DESC_INTERFACE_ASSOCIATION, ac_itf, 3, TUSB_CLASS_AUDIO,
		0x00, 0x00, 0,

		// Audio Control (AC) Standard Interface Descriptor - 9 bytes
		9, TUSB_DESC_INTERFACE, ac_itf, 0, 0, TUSB_CLASS_AUDIO, 0x01,
		0x00, 0,

		// The terminals and units below are a real pad's, IDs and associations included: speaker path
		// IT 1 -> FU 2 -> OT 3, mic path IT 4 -> FU 5 -> OT 6, with IT 1/OT 6 and OT 3/IT 4 paired.

		// AC Class-Specific Header Descriptor - 10 bytes
		// wTotalLength = CS_hdr(10)+IT(12)+FU(12)+OT(9)+mic IT(12)+mic FU(9)+mic OT(9) = 73
		10, 0x24, 0x01, 0x00, 0x01, 73, 0x00, 2, as_out, as_in,

		// Input Terminal Descriptor (USB Streaming, 4ch) - 12 bytes
		// wChannelConfig 0x0033: FL + FR + BL(haptic-L) + BR(haptic-R)
		12, 0x24, 0x02, 0x01, 0x01, 0x01, 0x06, 4, 0x33, 0x00, 0x00, 0,

		// Feature Unit Descriptor - 12 bytes
		// bControlSize=1: master mute + volume (0x03), no per-channel controls
		12, 0x24, 0x06, 0x02, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00,
		0,

		// Output Terminal Descriptor (Speaker 0x0301) - 9 bytes
		9, 0x24, 0x03, 0x03, 0x01, 0x03, 0x04, 0x02, 0,

		// Input Terminal Descriptor (Headset 0x0402, 2ch FL+FR) - 12 bytes
		12, 0x24, 0x02, 0x04, 0x02, 0x04, 0x03, 2, 0x03, 0x00, 0x00, 0,

		// Feature Unit Descriptor (mic) - 9 bytes
		// master mute + volume only, the same 9-byte form a real pad sends
		9, 0x24, 0x06, 0x05, 0x04, 0x01, 0x03, 0x00, 0,

		// Output Terminal Descriptor (USB Streaming, source = mic FU) - 9 bytes
		9, 0x24, 0x03, 0x06, 0x01, 0x01, 0x01, 0x05, 0,

		// Speaker/haptics AS Standard Interface Descriptor (Alt 0) - 9 bytes
		9, TUSB_DESC_INTERFACE, as_out, 0, 0, TUSB_CLASS_AUDIO, 0x02,
		0x00, 0,

		// Speaker/haptics AS Standard Interface Descriptor (Alt 1) - 9 bytes
		9, TUSB_DESC_INTERFACE, as_out, 1, 1, TUSB_CLASS_AUDIO, 0x02,
		0x00, 0,

		// AS Class-Specific General Descriptor (terminal link = IT 1) - 7 bytes
		7, 0x24, 0x01, 0x01, 0x01, 0x01, 0x00,

		// AS Class-Specific Format Type I Descriptor (PCM 4ch 16-bit 48kHz) - 11 bytes
		11, 0x24, 0x02, 0x01, 4, 2, 16, 1, 0x80, 0xBB, 0x00,

		// Standard Isochronous Audio Data Endpoint Descriptor - 9 bytes (EP 0x08)
		// bmAttributes 0x09: isochronous (01b) + adaptive sync (10b), as on a real pad. The nRF52840's only
		// isochronous endpoint is EP8, so the address can't match a real pad's 0x01.
		9, TUSB_DESC_ENDPOINT, 0x08, 0x09,
		U16_TO_U8S_LE(UAC1_ISO_EP_BUFSIZE), 1, 0, 0,

		// Class-Specific Audio Data Endpoint Descriptor - 7 bytes
		// bmAttributes 0: no sampling-frequency control, as on a real pad
		7, 0x25, 0x01, 0x00, 0, 0, 0,

		// Mic AS Standard Interface Descriptor (Alt 0) - 9 bytes
		9, TUSB_DESC_INTERFACE, as_in, 0, 0, TUSB_CLASS_AUDIO, 0x02,
		0x00, 0,

		// Mic AS Standard Interface Descriptor (Alt 1) - 9 bytes
		9, TUSB_DESC_INTERFACE, as_in, 1, 1, TUSB_CLASS_AUDIO, 0x02,
		0x00, 0,

		// AS Class-Specific General Descriptor (terminal link = OT 6) - 7 bytes
		7, 0x24, 0x01, 0x06, 0x01, 0x01, 0x00,

		// AS Class-Specific Format Type I Descriptor (PCM 2ch 16-bit 48kHz) - 11 bytes
		11, 0x24, 0x02, 0x01, 2, 2, 16, 1, 0x80, 0xBB, 0x00,

		// Standard Isochronous Audio Data Endpoint Descriptor - 9 bytes (EP 0x88)
		// bmAttributes 0x05: isochronous (01b) + asynchronous sync (01b)
		9, TUSB_DESC_ENDPOINT, 0x88, 0x05,
		U16_TO_U8S_LE(UAC1_ISO_IN_EP_BUFSIZE), 1, 0, 0,

		// Class-Specific Audio Data Endpoint Descriptor - 7 bytes
		7, 0x25, 0x01, 0x00, 0, 0, 0
	};

	memcpy(buf, desc, UAC1_DESC_LEN);
	return UAC1_DESC_LEN;
}

bool Adafruit_USBD_Audio_UAC1::begin()
{
	return TinyUSBDevice.addInterface(*this);
}

#include <math.h>

// Rumble style: an envelope below this (~1.2% of full scale) is dither, and even a tiny 0x80 speed still
// runs the controller's motor imitation.
#define HAPTIC_GATE 400
// Tone style: plays down to TONE_FLOOR_DB, so its gate sits lower (~0.3% of full scale). Idle haptic tracks
// are exact zeros, so this still separates an effect from silence.
#define TONE_GATE 100
// One strength update per 20 ms, the 0x80 rumble relay's throttle (RUMBLE_THROTTLE_MS in haptics.cpp).
#define HAPTIC_TICK_MS 20u
// A zero crossing counts once a sample passes this far beyond zero, so dither does not read as a high tone.
#define ZC_HYST 64

// Per haptic channel since the last tick: sum of squared samples, the same split below/above SPLIT_HZ, and
// zero crossings (of the part above SPLIT_HZ in the split style) with the frames that carried signal for them,
// plus the frames summed. Added per 1 ms ISO packet in the usbd task, taken and cleared by ps5AudioTask in the
// loop task.
static uint64_t s_winSqL = 0;
static uint64_t s_winSqR = 0;
static uint64_t s_winSqLoL = 0;
static uint64_t s_winSqLoR = 0;
static uint64_t s_winSqHiL = 0;
static uint64_t s_winSqHiR = 0;
static uint16_t s_winZcL = 0;
static uint16_t s_winZcR = 0;
static uint16_t s_winActL = 0;
static uint16_t s_winActR = 0;
static uint32_t s_winFrames = 0;

// Split style crossover: 2nd-order Butterworth low-pass at SPLIT_HZ = 80 (RBJ cookbook, fs 48 kHz, Q 0.7071).
// Stellar Blade's dash has 38% of its energy below 80 Hz, yet its zero crossings read 125-300 Hz, so tone
// style alone plays it high and the deep part is lost.
#define SPLIT_B0 2.721380799e-05f
#define SPLIT_B1 5.442761598e-05f
#define SPLIT_A1 (-1.985190658f)
#define SPLIT_A2 0.985299513f

struct Biquad {
	float x1, x2, y1, y2;
};

static inline float splitLow(Biquad *f, float x)
{
	float y = SPLIT_B0 * (x + f->x2) + SPLIT_B1 * f->x1 - SPLIT_A1 * f->y1 -
		  SPLIT_A2 * f->y2;
	f->x2 = f->x1;
	f->x1 = x;
	f->y2 = f->y1;
	f->y1 = y;
	return y;
}

// Wave style filter: 2nd-order Butterworth sections (RBJ cookbook, Q 0.7071), -6 dB at the corner.
struct BiquadCoef {
	float b0, b1, b2, a1, a2;
};

// RBJ low-pass at f0 Hz for a stream at fs Hz, Q 0.7071.
static BiquadCoef lowPass(float f0, float fs)
{
	float w = 6.2831853f * f0 / fs, c = cosf(w), a = sinf(w) * 0.70710678f,
	      a0 = 1.0f + a;
	return { (1.0f - c) / 2.0f / a0, (1.0f - c) / a0,
		 (1.0f - c) / 2.0f / a0, -2.0f * c / a0, (1.0f - a) / a0 };
}

// Wave style: the haptic channels low-passed at OPK_HAPTIC_LP_HZ (two sections at 48 kHz, which is also the
// anti-alias ahead of the decimation). The grip actuators turn content above a few hundred Hz into clicks and
// buzz, so sharp effects felt harsh through the old 1.6 kHz anti-alias alone. In a feel test (Stellar Blade)
// 300 Hz was smooth on strong hits and 500 Hz still a little harsh.
#ifndef OPK_HAPTIC_LP_HZ
#define OPK_HAPTIC_LP_HZ 300
#endif
static const BiquadCoef HAP_LP = lowPass(OPK_HAPTIC_LP_HZ, 48000.0f);

static inline float biquad(Biquad *f, const BiquadCoef &c, float x)
{
	float y = c.b0 * x + c.b1 * f->x1 + c.b2 * f->x2 - c.a1 * f->y1 -
		  c.a2 * f->y2;
	f->x2 = f->x1;
	f->x1 = x;
	f->y2 = f->y1;
	f->y1 = y;
	return y;
}

// Wave style grip stream, set by ps5AudioTask: bond slots streaming (bit per slot) and the sample scale.
static volatile uint8_t s_waveMask = 0;
static volatile float s_waveScale = 0;

// Frequency is crossings per frame with signal, not per tick: a step starting late in a tick, or silence
// after one, otherwise reads far too low. Stellar Blade's running steps then played at 40 Hz, where the
// controller's IMU measured a quarter of the 125 Hz response.
static inline void zeroCross(int32_t x, int8_t *sign, uint16_t *zc,
			     uint16_t *act)
{
	if (x > ZC_HYST || x < -ZC_HYST)
		(*act)++;
	if ((*sign > 0 && x < -ZC_HYST) || (*sign < 0 && x > ZC_HYST)) {
		*sign = (int8_t) - *sign;
		(*zc)++;
	}
}

static void processAudioSamples(const uint8_t *data, uint32_t len)
{
	// 4-channel 16-bit PCM: 8 bytes per frame (FL, FR, RL, RR)
	// ch3 (s[2]) = Left LRA, ch4 (s[3]) = Right LRA (dedicated haptic tracks)
	if (len < 8)
		return;

	// which side of zero each haptic channel was last on, and the crossover state (usbd task only)
	static int8_t s_signL = 1, s_signR = 1;
	static Biquad s_lpL = {}, s_lpR = {};
	bool split = g_audioHapticStyle == AUDIO_STYLE_SPLIT;
	// grip stream: decimation phase, fill of the frame being built, samples sent, and that frame
	static Biquad s_aaL[2] = {}, s_aaR[2] = {};
	static uint8_t s_dec = 0, s_fill = 0;
	static uint16_t s_queued = 0;
	static uint8_t s_pcmL[PCM_SAMPLES], s_pcmR[PCM_SAMPLES];
	uint8_t waveMask = s_waveMask;
	float waveScale = s_waveScale;
	if (!waveMask) {
		s_fill = 0;
		s_queued = 0;
	}

	uint32_t num_frames = len / 8;
	uint64_t sq_l = 0, sq_r = 0;
	float lo_l = 0, lo_r = 0, hi_l = 0, hi_r = 0;
	uint16_t zc_l = 0, zc_r = 0, act_l = 0, act_r = 0;
	for (uint32_t i = 0; i < num_frames; i++) {
		const int16_t *s = (const int16_t *)(data + i * 8);
		sq_l += (int32_t)s[2] * s[2];
		sq_r += (int32_t)s[3] * s[3];
		float ll = splitLow(&s_lpL, s[2]), lr = splitLow(&s_lpR, s[3]);
		float hl = s[2] - ll, hr = s[3] - lr;
		lo_l += ll * ll;
		lo_r += lr * lr;
		hi_l += hl * hl;
		hi_r += hr * hr;
		zeroCross(split ? (int32_t)hl : s[2], &s_signL, &zc_l, &act_l);
		zeroCross(split ? (int32_t)hr : s[3], &s_signR, &zc_r, &act_r);
		if (!waveMask)
			continue;
		float al = biquad(&s_aaL[1], HAP_LP,
				  biquad(&s_aaL[0], HAP_LP, s[2])),
		      ar = biquad(&s_aaR[1], HAP_LP,
				  biquad(&s_aaR[0], HAP_LP, s[3]));
		if (++s_dec < 48000u / PCM_RATE_HZ)
			continue;
		s_dec = 0;
		// soft limit, not hapticUlaw's clamp: a sharp hit peaks above the 20 ms envelope the haptic gain
		// follows, and a clipped peak plays as a pop
		s_pcmL[s_fill] = hapticUlaw(hapticSoftLimit(al * waveScale));
		s_pcmR[s_fill] = hapticUlaw(hapticSoftLimit(ar * waveScale));
		if (++s_fill < hapticPcmFrameLen(s_queued, PCM_RATE_HZ))
			continue;
		for (uint8_t b = 0; b < NSLOT; b++)
			if (waveMask & (1u << b))
				hapticPcmSend(b, s_pcmL, s_pcmR, s_fill);
		if (s_queued < 1000u)
			s_queued += s_fill;
		s_fill = 0;
	}

	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	s_winSqL += sq_l;
	s_winSqR += sq_r;
	s_winSqLoL += (uint64_t)lo_l;
	s_winSqLoR += (uint64_t)lo_r;
	s_winSqHiL += (uint64_t)hi_l;
	s_winSqHiR += (uint64_t)hi_r;
	s_winZcL += zc_l;
	s_winZcR += zc_r;
	s_winActL += act_l;
	s_winActR += act_r;
	s_winFrames += num_frames;
	__set_PRIMASK(pm);
}

// Level of one haptic channel over a tick: RMS scaled by sqrt(2), so a sine reads as its peak. It must span
// the whole tick -- one 1 ms packet holds a tenth of a 100-150 Hz haptic cycle, so a per-packet level swings
// with phase and reads as random strength and stop/start chatter.
static uint16_t hapticLevel(uint64_t sumSq, uint32_t frames)
{
	if (!frames)
		return 0;
	float level = sqrtf(2.0f * (float)(sumSq / frames));
	return level > 32767.0f ? 32767 : (uint16_t)level;
}

// Envelope of one haptic channel: instant rise, halving fall. Noise-like textures (Stellar Blade's sprint)
// swing +-25 points tick to tick even as RMS, felt as a 50 Hz stutter; the halving fall cuts that jitter to
// about half while a hit still lands at full strength on its first tick and fades within ~100 ms.
static uint16_t hapticEnvelope(uint16_t level, uint16_t *env, uint16_t gate)
{
	*env = level >= *env ? level : (uint16_t)((*env + level) / 2);
	if (*env < gate)
		*env = 0;
	return *env;
}

// Auto gain reference: the envelope that plays at full strength. It jumps to any louder peak and sinks by
// 1/1024 per tick (~20 s time constant), so a long quiet stretch is lifted gradually instead of footsteps
// snapping to full after one hit. The floor caps the input boost at 4x, so dither-level games stay faint.
#define AUTO_REF_MIN 8192u
#define AUTO_REF_SINK_SHIFT 10

static uint16_t autoGainRef(uint16_t loudest)
{
	static uint16_t s_ref = AUTO_REF_MIN;
	if (loudest > s_ref)
		s_ref = loudest;
	else
		s_ref -= s_ref >> AUTO_REF_SINK_SHIFT;
	if (s_ref < AUTO_REF_MIN)
		s_ref = AUTO_REF_MIN;
	return s_ref;
}

// Drive level, 1.0 = full strength: square root of the envelope against `ref`, then the fixed gain. Linear
// left Stellar Blade's footsteps (40 ms pulses at ~5% of full scale) at 10-17% rumble / -15..-20 dB tone,
// which is not felt; the root puts them at ~35-45%.
static float hapticDrive(uint16_t env, uint16_t ref, uint16_t gainPct)
{
	return sqrtf((float)env / ref) * gainPct / 100.0f;
}

static uint16_t hapticStrength(uint16_t env, uint16_t ref, uint16_t gainPct)
{
	float s = hapticDrive(env, ref, gainPct) * 65535.0f;
	return s > 65535.0f ? 65535u : (uint16_t)s;
}

// Bond slot fed by USB slot u; single-controller setups fall back to the first bonded slot.
static int audioBond(uint8_t u)
{
	int bond = (u < NSLOT) ? g_usbToBond[u] : -1;
	if (bond >= 0 && g_slot[bond].used)
		return bond;
	for (int s = 0; s < NSLOT; s++)
		if (g_slot[s].used)
			return s;
	return -1;
}

// ---- Tone style: each haptic channel plays as a 0x83 tone on its own actuator (side 0 = left, 1 = right),
// at the channel's dominant frequency and strength. The limits below come from the controller's IMU: a tone
// re-sent every 60 ms plays as smoothly as one long tone, and a 200 ms duration rides out a lost RF frame;
// strength steps of a few dB a few times a second trace cleanly where 1 dB steps every 20 ms stutter; and
// every retune swings the vibration, so the frequency moves only between coarse steps.
#define TONE_DUR_MS 200u
#define TONE_REFRESH_MS 60u
#define TONE_STEP_MS 40u
#define TONE_STEP_DB 2
// A rise this large is a hit: sent at once, past the step limit.
#define TONE_HIT_DB 6
// Quietest tone played; the controller still renders it, and a more sensitive hand than the tester's can
// feel below where the tester lost it (~-48 dB).
#define TONE_FLOOR_DB (-60)
// gain_db that cuts a playing tone within ~25-50 ms; ending it any other way fades out over ~100-150 ms
#define TONE_CUT_DB (-128)
#define TONE_CUT_MS 30u

// A third of an octave apart across the actuators' useful 40-250 Hz (350 Hz is barely felt).
static const uint16_t TONE_BANDS[] = { 40, 50, 63, 80, 100, 125, 160, 200, 250 };
#define TONE_NBANDS (sizeof TONE_BANDS / sizeof TONE_BANDS[0])

static uint8_t toneBand(float hz)
{
	uint8_t best = 0;
	float bestDist = 1e9f;
	for (uint8_t i = 0; i < TONE_NBANDS; i++) {
		float dist = fabsf(logf(hz / TONE_BANDS[i]));
		if (dist < bestDist) {
			bestDist = dist;
			best = i;
		}
	}
	return best;
}

struct ToneSide {
	bool on;
	int8_t gain; // last sent, dB
	uint8_t band; // TONE_BANDS index being played
	uint8_t pendBand; // a different band the estimate moved to, and for how many ticks in a row
	uint8_t pendTicks;
	uint32_t sentMs;
	// zero crossings and frames with signal of the last three ticks that crossed: a 60 ms frequency estimate
	uint16_t zc[3];
	uint16_t frames[3];
	uint8_t n;
};

static void toneSend(uint8_t side, int8_t gainDb, uint16_t hz, uint16_t durMs)
{
	for (uint8_t u = 0; u < g_usbMountCount; u++) {
		int bond = audioBond(u);
		if (bond >= 0)
			hapticAudioTone(side, gainDb, hz, durMs, (uint8_t)bond);
	}
}

static void toneUpdate(ToneSide *t, uint8_t side, uint16_t env, uint16_t ref,
		       uint16_t gainPct, uint16_t zc, uint16_t frames,
		       uint32_t now)
{
	if (!env) {
		if (t->on) {
			// Twice: the relay does not retry, and a lost cut lets the tone play out its duration.
			toneSend(side, TONE_CUT_DB, TONE_BANDS[t->band],
				 TONE_CUT_MS);
			toneSend(side, TONE_CUT_DB, TONE_BANDS[t->band],
				 TONE_CUT_MS);
			t->on = false;
		}
		t->n = 0;
		return;
	}

	// A tick without a crossing adds nothing; until one has, the tone keeps the band it last played (the
	// previous footstep's, for a repeating effect).
	if (zc) {
		t->zc[t->n % 3] = zc;
		t->frames[t->n % 3] = frames;
		t->n++;
	}
	uint8_t band = t->band;
	if (t->n) {
		uint32_t zcSum = 0, frameSum = 0;
		for (uint8_t k = 0; k < (t->n < 3 ? t->n : 3); k++) {
			zcSum += t->zc[k];
			frameSum += t->frames[k];
		}
		// two crossings per cycle at 48 kHz; a crossing always comes with a frame of signal
		band = toneBand((float)zcSum * 24000.0f / (float)frameSum);
	}
	float db = 20.0f * log10f(hapticDrive(env, ref, gainPct));
	int8_t gain = db >= 0.0f		 ? 0 :
		      db <= (float)TONE_FLOOR_DB ? (int8_t)TONE_FLOOR_DB :
						   (int8_t)lroundf(db);
	uint32_t since = now - t->sentMs;

	bool send;
	if (!t->on) {
		t->band = band;
		t->pendTicks = 0;
		send = true;
	} else {
		if (band == t->band)
			t->pendTicks = 0;
		else if (band == t->pendBand)
			t->pendTicks++;
		else {
			t->pendBand = band;
			t->pendTicks = 1;
		}
		bool retune = t->pendTicks >= 2;
		if (retune) {
			t->band = band;
			t->pendTicks = 0;
		}
		int step = gain - t->gain;
		bool change = retune || step >= TONE_STEP_DB ||
			      step <= -TONE_STEP_DB;
		send = step >= TONE_HIT_DB ||
		       (change && since >= TONE_STEP_MS) ||
		       since >= TONE_REFRESH_MS;
	}
	if (!send)
		return;
	toneSend(side, gain, TONE_BANDS[t->band], TONE_DUR_MS);
	t->on = true;
	t->gain = gain;
	t->sentMs = now;
}

// Wave style grip stream: stream while the haptic channels are above the gate and for WAVE_HANG_MS after, so a
// short pause does not pay the controller's pre-buffer again. The format is re-sent every WAVE_FMT_MS: it
// persists on the controller, but one that power-cycled mid-stream comes back without it.
#define WAVE_HANG_MS 300u
#define WAVE_FMT_MS 1000u

static void waveUpdate(bool active, float waveScale, uint32_t now)
{
	static bool s_on = false;
	static uint32_t s_lastActive = 0, s_fmtMs[NSLOT] = {};
	if (active) {
		s_on = true;
		s_lastActive = now;
	} else if (s_on && now - s_lastActive >= WAVE_HANG_MS) {
		s_on = false;
	}
	uint8_t mask = 0;
	for (uint8_t u = 0; s_on && u < g_usbMountCount; u++) {
		int bond = audioBond(u);
		if (bond >= 0)
			mask |= (uint8_t)(1u << bond);
	}
	for (uint8_t b = 0; b < NSLOT; b++) {
		if (!(mask & (1u << b)))
			continue;
		if (!(s_waveMask & (1u << b)) ||
		    now - s_fmtMs[b] >= WAVE_FMT_MS) {
			hapticPcmStart(b);
			s_fmtMs[b] = now;
		}
	}
	s_waveScale = waveScale;
	s_waveMask = mask;
}

void ps5AudioTask(void)
{
	static uint16_t s_envL = 0, s_envR = 0;
	static uint16_t s_envLoL = 0, s_envLoR = 0, s_envHiL = 0, s_envHiR = 0;
	static uint16_t s_lastL = 0, s_lastR = 0;
	static uint32_t s_lastTickMs = 0;
	static ToneSide s_toneL = {}, s_toneR = {};

	uint32_t now = millis();
	if ((uint32_t)(now - s_lastTickMs) < HAPTIC_TICK_MS)
		return;
	s_lastTickMs = now;

	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint64_t sqL = s_winSqL, sqR = s_winSqR;
	uint64_t sqLoL = s_winSqLoL, sqLoR = s_winSqLoR;
	uint64_t sqHiL = s_winSqHiL, sqHiR = s_winSqHiR;
	uint16_t zcL = s_winZcL, zcR = s_winZcR;
	uint16_t actL = s_winActL, actR = s_winActR;
	uint32_t frames = s_winFrames;
	s_winSqL = s_winSqR = 0;
	s_winSqLoL = s_winSqLoR = s_winSqHiL = s_winSqHiR = 0;
	s_winZcL = s_winZcR = 0;
	s_winActL = s_winActR = 0;
	s_winFrames = 0;
	__set_PRIMASK(pm);

	bool tone = g_audioHapticStyle == AUDIO_STYLE_TONE;
	bool split = g_audioHapticStyle == AUDIO_STYLE_SPLIT;
	bool wave = g_audioHapticStyle == AUDIO_STYLE_WAVE;
	uint16_t gate = (tone || wave) ? TONE_GATE : HAPTIC_GATE;
	uint16_t envL = hapticEnvelope(hapticLevel(sqL, frames), &s_envL, gate);
	uint16_t envR = hapticEnvelope(hapticLevel(sqR, frames), &s_envR, gate);
	// Split style: below SPLIT_HZ drives the rumble, the rest the tones, each with that output's gate.
	uint16_t loL = hapticEnvelope(hapticLevel(sqLoL, frames), &s_envLoL,
				      HAPTIC_GATE);
	uint16_t loR = hapticEnvelope(hapticLevel(sqLoR, frames), &s_envLoR,
				      HAPTIC_GATE);
	uint16_t hiL = hapticEnvelope(hapticLevel(sqHiL, frames), &s_envHiL,
				      TONE_GATE);
	uint16_t hiR = hapticEnvelope(hapticLevel(sqHiR, frames), &s_envHiR,
				      TONE_GATE);
	// Auto gain runs even while a fixed gain is set, so switching back to auto starts from a settled level.
	uint16_t ref = autoGainRef(envL > envR ? envL : envR);
	uint16_t gain = g_audioHapticGain;
	if (gain)
		ref = 32767;
	else
		gain = 100;
	if (!g_audioHaptics)
		envL = envR = loL = loR = hiL = hiR = 0;
	uint16_t toneL = tone ? envL : split ? hiL : 0;
	uint16_t toneR = tone ? envR : split ? hiR : 0;
	uint16_t rumL = (tone || wave) ? 0 : split ? loL : envL;
	uint16_t rumR = (tone || wave) ? 0 : split ? loR : envR;
	// wave plays the haptic channels as sent, like a real DualSense, so it skips the auto gain. 100% (and Auto)
	// is 40% of full scale: half matched a real pad in FFXIV, but felt too strong across other games (2026-10-05).
	waveUpdate(wave && (envL || envR),
		   (wave && g_audioHaptics) ?
			   (g_audioHapticGain ? g_audioHapticGain : 100) /
				   (250.0f * 32768.0f) :
			   0.0f,
		   now);

	// A style that stops using an output stops whatever it left playing (a tone cuts once; a rumble stops below).
	toneUpdate(&s_toneL, 0, toneL, ref, gain, zcL, actL, now);
	toneUpdate(&s_toneR, 1, toneR, ref, gain, zcR, actR, now);
	uint16_t l = hapticStrength(rumL, ref, gain);
	uint16_t r = hapticStrength(rumR, ref, gain);

	// Silent and already stopped: send nothing, so a game's HID report 0x02 motor rumble is not overridden.
	// While active, resend every tick: hapticUpdateRumble drops identical frames itself, and re-offering the
	// current strength recovers an update its own 20 ms throttle refused.
	if (!l && !r && !s_lastL && !s_lastR)
		return;
	s_lastL = l;
	s_lastR = r;
	for (uint8_t u = 0; u < g_usbMountCount; u++) {
		int bond = audioBond(u);
		if (bond >= 0)
			hapticAudioRumble(l, r, (uint8_t)bond);
	}
}

// TinyUSB class driver implementation for UAC1

// Endpoint descriptor for the ISO OUT endpoint on Alt 1.
// Must match the AS descriptor emitted by getInterfaceDescriptor.
static const tusb_desc_endpoint_t s_iso_ep_out = {
	.bLength = sizeof(tusb_desc_endpoint_t),
	.bDescriptorType = TUSB_DESC_ENDPOINT,
	.bEndpointAddress = 0x08,
	// bmAttributes 0x09: isochronous (01b) + adaptive sync (10b)
	.bmAttributes = { .xfer = TUSB_XFER_ISOCHRONOUS, .sync = 2, .usage = 0 },
	.wMaxPacketSize = UAC1_ISO_EP_BUFSIZE,
	.bInterval = 1,
};

static const tusb_desc_endpoint_t s_iso_ep_in = {
	.bLength = sizeof(tusb_desc_endpoint_t),
	.bDescriptorType = TUSB_DESC_ENDPOINT,
	.bEndpointAddress = 0x88,
	.bmAttributes = { .xfer = TUSB_XFER_ISOCHRONOUS, .sync = 1, .usage = 0 },
	.wMaxPacketSize = UAC1_ISO_IN_EP_BUFSIZE,
	.bInterval = 1,
};

// SET_CUR data for controls we don't apply (sampling rate, unknown selectors). Kept apart from g_isoOutBuf,
// which the ISO OUT DMA may be filling when the host configures the mic mid-stream.
static uint8_t s_ctrlScratch[8];

static void uac1_init(void)
{
	// USBD_ISOSPLIT_SPLIT_OneDir (0x0000) is the hardware reset default;
	// writing it here races with the USBD power-on sequence and causes the
	// host's first GET_DESCRIPTOR to stall (-71 EPROTO). TinyUSB's own
	// dcd_nrf5x.c sets ISOSPLIT when ISO endpoints are opened.
}

static void uac1_reset(uint8_t rhport)
{
	(void)rhport;
	g_uac1AltSetting = 0;
	g_uac1AltSettingIn = 0;
}

static uint16_t uac1_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
			  uint16_t max_len)
{
	(void)max_len;
	if (itf_desc->bInterfaceClass != TUSB_CLASS_AUDIO)
		return 0;

	if (itf_desc->bInterfaceSubClass == 0x01) {
		usbd_edpt_open(rhport, &s_iso_ep_out);
		usbd_edpt_open(rhport, &s_iso_ep_in);
		/*
		 * IAD has bInterfaceCount=3, so TinyUSB pre-binds AC and both
		 * AS interfaces to this driver before calling open(). We must
		 * consume all three interfaces' bytes here so the scanner
		 * doesn't hit an AS interface again and fail the already-bound
		 * slot assertion. open() gets the pointer just past the IAD.
		 */
		return UAC1_DESC_LEN - UAC1_IAD_LEN;
	}
	return 0;
}

static bool uac1_control_xfer_cb(uint8_t rhport, uint8_t stage,
				 tusb_control_request_t const *request)
{
	if (stage != CONTROL_STAGE_SETUP)
		return true;

	if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
		if (request->bRequest != TUSB_REQ_SET_INTERFACE)
			return false;
		uint8_t itf = tu_u16_low(request->wIndex);
		uint8_t alt = tu_u16_low(request->wValue);
		if (itf == g_uac1ItfAs) {
			g_uac1AltSetting = alt;
			if (alt == 1) {
				usbd_edpt_xfer(rhport, g_uac1EpOut, g_isoOutBuf,
					       UAC1_ISO_EP_BUFSIZE);
			}
		} else if (itf == g_uac1ItfAsIn) {
			g_uac1AltSettingIn = alt;
			// A transfer left queued by an earlier alt 1 keeps the silence loop running on its own.
			if (alt == 1 && !usbd_edpt_busy(rhport, g_uac1EpIn))
				usbd_edpt_xfer(rhport, g_uac1EpIn, g_isoInBuf,
					       UAC1_ISO_IN_PACKET);
		}
		return tud_control_status(rhport, request);
	}

	if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS)
		return false;

	// Endpoint-directed: sampling frequency
	if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) {
		if (request->bRequest == 0x01) { // SET_CUR
			return tud_control_xfer(rhport, request, s_ctrlScratch,
						tu_min16(request->wLength,
							 sizeof s_ctrlScratch));
		}
		// GET_CUR / GET_MIN / GET_MAX / GET_RES: return 48 kHz
		static uint8_t s_freq[3] = { 0x80, 0xBB, 0x00 };
		return tud_control_xfer(rhport, request, s_freq, sizeof s_freq);
	}

	// Interface-directed: master mute / volume on Feature Unit 2 (speaker) or 5 (mic), never applied.
	// Volumes are 16-bit signed 1/256 dB. The ranges give what Linux shows for a real pad: speaker
	// -100..0 dB in 1 dB steps, mic 0..+48 dB in 101 steps.
	static const int16_t s_volRange[2][3] = {
		{ -100 * 256, 0, 256 }, // speaker: min, max, res
		{ 0, 48 * 256, 121 }, // mic
	};
	static uint8_t s_curMute[2] = { 0, 0 };
	static int16_t s_curVol[2] = { 0, 0 };
	uint8_t cs = tu_u16_high(request->wValue);
	uint8_t u = tu_u16_high(request->wIndex) == 0x05 ? 1 : 0;

	if (request->bRequest == 0x01) { // SET_CUR
		if (cs == 0x01)
			return tud_control_xfer(rhport, request, &s_curMute[u],
						1);
		if (cs == 0x02)
			return tud_control_xfer(rhport, request, &s_curVol[u],
						sizeof(int16_t));
		return tud_control_xfer(rhport, request, s_ctrlScratch,
					tu_min16(request->wLength,
						 sizeof s_ctrlScratch));
	}

	if (cs == 0x01)
		return tud_control_xfer(rhport, request, &s_curMute[u], 1);

	if (cs == 0x02) {
		switch (request->bRequest) {
		case 0x81: // GET_CUR
			return tud_control_xfer(rhport, request, &s_curVol[u],
						sizeof(int16_t));
		case 0x82: // GET_MIN
		case 0x83: // GET_MAX
		case 0x84: // GET_RES
			return tud_control_xfer(
				rhport, request,
				(void *)&s_volRange[u][request->bRequest - 0x82],
				sizeof(int16_t));
		default:
			return false;
		}
	}
	return false;
}

static bool uac1_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
			 uint32_t xferred_bytes)
{
	if (ep_addr == g_uac1EpOut && result == XFER_RESULT_SUCCESS) {
		if (xferred_bytes > 0 && g_uac1AltSetting == 1)
			processAudioSamples(g_isoOutBuf, xferred_bytes);
		if (g_uac1AltSetting == 1)
			usbd_edpt_xfer(rhport, g_uac1EpOut, g_isoOutBuf,
				       UAC1_ISO_EP_BUFSIZE);
		return true;
	}
	if (ep_addr == g_uac1EpIn) {
		if (g_uac1AltSettingIn == 1)
			usbd_edpt_xfer(rhport, g_uac1EpIn, g_isoInBuf,
				       UAC1_ISO_IN_PACKET);
		return true;
	}
	return false;
}

static const usbd_class_driver_t g_uac1Driver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "UAC1",
#endif
	.init = uac1_init,
	.reset = uac1_reset,
	.open = uac1_open,
	.control_xfer_cb = uac1_control_xfer_cb,
	.xfer_cb = uac1_xfer_cb,
	.sof = NULL
};

const usbd_class_driver_t *uac1_get_driver(void)
{
	return &g_uac1Driver;
}
