// audio.h -- mode-announcement jingles played through the controller's haptic actuators.
//
// Streamed PCM audio was abandoned: over the wireless puck<->controller link the controller drops most PCM
// frames while driving its actuator (noRx spikes, playback breaks up). Single 0x83 LFO_TONE *commands*, by
// contrast, are cheap and rock-solid over RF -- one packet tells the controller to synthesize a tone locally.
// So mode announcements are short, distinct tone jingles: reliable and instantly recognizable.
//
//   0x83 LFO_TONE  [ch][gain][freqLo][freqHi][0xFF][dur][0][0][0]  -- play a tone until the next tone/stop.
//
// A jingle is a list of notes; audioTask() paces one 0x83 per note into the haptic relay ring (haptics.cpp).
// On the first controller connect after boot (i.e. after a mode switch, which reboots the puck) the active
// mode's jingle plays once, so the puck "announces" its mode with no host/panel involvement.
#pragma once
#include <stdint.h>

// 0x83 tone loudness (gain byte) and the settle delay between a controller (re)connecting and the on-boot
// announce (lets its haptic engine come up first).
#define AUD_TONE_GAIN 0x7Fu
#define ANN_SETTLE_MS 900u

struct AudNote {
	uint16_t freq; // Hz; 0 = rest (silence)
	uint16_t durMs;
};

void audioInit();
// per-loop upkeep: advances the jingle sequencer and fires the one-shot on-boot mode announce.
void audioTask();

// Play an explicit jingle. slot = bond slot, or 0xFF for the first connected controller.
void audioPlayJingle(const AudNote *notes, uint8_t n, uint8_t slot = 0xFF);
// Play a mode's announcement jingle now (used by the on-boot announce and the panel test button).
void audioAnnounceNow(uint8_t mode, uint8_t slot = 0xFF);

// first connected controller slot, or -1 if none.
int audioFirstLinkedSlot();
