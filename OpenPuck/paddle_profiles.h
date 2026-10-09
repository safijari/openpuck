// paddle_profiles.h -- back-paddle profiles you can switch and reprogram from the controller itself.
//
// Each emulated controller type (Xbox / Switch / DS4 / DS5) gets PP_COUNT paddle layouts. The ACTIVE layout
// always lives in g_type[et].back (cfg.bin), so every mode builder and the WebUSB panel keep working
// unchanged -- the panel simply edits whichever profile is active. The other layouts are kept in their own
// flash file (/paddles.bin), so cfg.bin's layout is untouched and an older firmware still reads it.
//
// Controller gestures (emulated modes only; Steam / Lizard / DirectInput / SInput are left alone):
//   all 4 back paddles + RB      -> next profile        (buzzes 1-4 times = the profile number)
//   all 4 back paddles + LB      -> previous profile
//   all 4 back paddles + Start   -> enter / leave paddle-assign mode (one long buzz)
// In paddle-assign mode nothing reaches the console. Tap a paddle (1 buzz), then tap the button it should
// act as (2 buzzes = saved). Tapping the same paddle twice turns that paddle off (3 buzzes). The mode also
// closes by itself after PP_LEARN_TIMEOUT_MS without input.
#pragma once
#include <stdint.h>

#define PP_COUNT 4
#define PP_LEARN_TIMEOUT_MS 30000u

// boot: load /paddles.bin (or seed presets). Call AFTER loadCfg() so g_type[] is already loaded.
void paddleProfilesLoad();
// loop context: feedback buzz sequencing, learn-mode timeout, deferred flash save.
void paddleProfilesTask();
// rf_link, once per decoded input report, after g_in[slot] is filled and BEFORE the report is handed to the
// active controller. May mask g_in[slot] and the raw report (rep) so gesture presses never reach the host.
void paddleProfilesOnInput(uint8_t slot, uint8_t *rep);
// Write /paddles.bin if it changed. saveCfg() calls this, so every path that persists the config (mode
// switch, panel edit) persists the profiles alongside it.
void paddleProfilesSaveIfDirty();
// active profile index (0..PP_COUNT-1) for an emulated type, 0xFF for an invalid type
uint8_t paddleProfileActive(uint8_t et);
