#include "audio.h"
#include "haptics.h" // relayEnqueue / hapticLinkUp
#include "bonds.h" // NSLOT
#include "config.h" // g_usbMode / g_announceEnabled / MODE_*
#include <Arduino.h>

// 0x83 LFO_TONE report id + fixed frame fields (see audio.h / reference SteamHapticsPlayer).
#define RID_LFO_TONE 0x83
#define AUD_TONE_CH \
	0 // haptic channel to sing on (0 = the channel the tone test proved clean)
#define AUD_TONE_DUR \
	0x64 // hold byte -- a note rings until the next note/stop is sent

// ---- jingle sequencer ----
static const AudNote *g_jNotes = nullptr;
static uint8_t g_jN = 0, g_jIdx = 0, g_jSlot = 0;
static bool g_jOn = false;
static uint32_t g_jNextMs = 0;

// ---- on-boot announce ----
static bool g_wasLink[NSLOT] = { false };
static bool g_annBootDone = false; // fire once per boot
static uint32_t g_annPendingAtMs = 0; // 0 = none; else scheduled start

// ---- per-mode jingles ------------------------------------------------------------------------------------
// Steam: a simple monotone. Switch: a crest then a valley. PS3/PS4/PS5: three notes, the first two identical
// across all three, the third distinct and ascending with the console number. Lizard: two tones back and forth
// a couple of times. Xbox: a short rising pair. Frequencies kept in the ~440-1050 Hz range that reproduces
// cleanly on the actuator.
static const AudNote J_STEAM[] = { { 659, 360 } };
static const AudNote J_XBOX[] = { { 392, 150 }, { 523, 220 } };
static const AudNote J_SWITCH[] = { { 660, 150 },
				    { 990, 170 },
				    { 440,
				      240 } }; // crest (990) then valley (440)
static const AudNote J_LIZARD[] = { { 700, 120 },
				    { 440, 120 },
				    { 700, 120 },
				    { 440, 150 } }; // back and forth x2
// PlayStation: shared first two notes, distinct ascending third (PS3 < PS4 < PS5).
static const AudNote J_PS3[] = { { 660, 160 }, { 660, 160 }, { 523, 240 } };
static const AudNote J_PS4[] = { { 660, 160 }, { 660, 160 }, { 784, 240 } };
static const AudNote J_PS5[] = { { 660, 160 }, { 660, 160 }, { 1047, 240 } };

static void jingleForMode(uint8_t mode, const AudNote **notes, uint8_t *n)
{
#define JSET(arr)                                    \
	do {                                         \
		*notes = (arr);                      \
		*n = sizeof(arr) / sizeof((arr)[0]); \
	} while (0)
	switch (mode) {
	case MODE_XBOX:
		JSET(J_XBOX);
		break;
	case MODE_SW_HORI:
	case MODE_SW_PRO:
		JSET(J_SWITCH);
		break;
	case MODE_LIZARD:
		JSET(J_LIZARD);
		break;
	case MODE_PS3:
		JSET(J_PS3);
		break;
	case MODE_HIDGYRO: // DS4 layout = "PS4"
	case MODE_DS4_GAME:
		JSET(J_PS4);
		break;
	case MODE_PS5:
	case MODE_PS5_GAME:
		JSET(J_PS5);
		break;
	case MODE_STEAM:
	default:
		JSET(J_STEAM);
		break;
	}
#undef JSET
}

int audioFirstLinkedSlot()
{
	for (int s = 0; s < NSLOT; s++)
		if (hapticLinkUp(s))
			return s;
	return -1;
}

static void toneSend(uint8_t slot, uint16_t freq)
{
	// [ch][gain][freqLo][freqHi][0xFF][dur][0][0][0]; freq 0 => the all-zero form that silences the channel.
	uint8_t d[9] = { AUD_TONE_CH,
			 (uint8_t)(freq ? AUD_TONE_GAIN : 0),
			 (uint8_t)(freq & 0xFF),
			 (uint8_t)(freq >> 8),
			 (uint8_t)(freq ? 0xFF : 0),
			 (uint8_t)(freq ? AUD_TONE_DUR : 0),
			 0,
			 0,
			 0 };
	relayEnqueue(RID_LFO_TONE, d, sizeof d, slot);
}

void audioInit()
{
	g_jOn = false;
	g_annBootDone = false;
	g_annPendingAtMs = 0;
	for (int s = 0; s < NSLOT; s++)
		g_wasLink[s] = false;
}

void audioPlayJingle(const AudNote *notes, uint8_t n, uint8_t slot)
{
	if (!notes || !n)
		return;
	if (slot == 0xFF) {
		int s = audioFirstLinkedSlot();
		if (s < 0)
			return;
		slot = (uint8_t)s;
	} else if (slot >= NSLOT || !hapticLinkUp(slot)) {
		return;
	}
	g_jNotes = notes;
	g_jN = n;
	g_jIdx = 0;
	g_jSlot = slot;
	g_jNextMs = millis(); // fire the first note immediately
	g_jOn = true;
}

void audioAnnounceNow(uint8_t mode, uint8_t slot)
{
	const AudNote *notes;
	uint8_t n;
	jingleForMode(mode, &notes, &n);
	audioPlayJingle(notes, n, slot);
}

void audioTask()
{
	// One-shot on-boot announce: arm on the first controller to come up, then play the active mode's jingle
	// after a short settle (once its haptic engine is ready).
	for (int s = 0; s < NSLOT; s++) {
		bool up = hapticLinkUp(s);
		if (up && !g_wasLink[s] && !g_annBootDone &&
		    g_announceEnabled) {
			g_annBootDone = true;
			g_annPendingAtMs = millis() + ANN_SETTLE_MS;
		}
		g_wasLink[s] = up;
	}
	if (g_annPendingAtMs && (int32_t)(millis() - g_annPendingAtMs) >= 0 &&
	    !g_jOn) {
		g_annPendingAtMs = 0;
		audioAnnounceNow(g_usbMode); // first connected controller
	}

	// Jingle sequencer: emit the next note when the current one's duration elapses; final tick silences.
	if (g_jOn) {
		if (!hapticLinkUp(g_jSlot)) {
			g_jOn = false;
		} else if ((int32_t)(millis() - g_jNextMs) >= 0) {
			if (g_jIdx >= g_jN) {
				toneSend(g_jSlot, 0);
				g_jOn = false;
			} else {
				const AudNote &nt = g_jNotes[g_jIdx++];
				toneSend(g_jSlot, nt.freq);
				g_jNextMs = millis() + nt.durMs;
			}
		}
	}
}
