// config.h -- USB-presentation modes + persisted, runtime-tunable settings.
//
// One puck speaks the SAME RF protocol no matter which USB face it wears; only the USB enumeration and the
// input-report mapping change. The active face is g_usbMode, persisted to flash (cfg.bin). Tunables here are
// set over the CDC console (serial_console.cpp) and the WebUSB panel (webusb_config.cpp) and likewise persist.
#pragma once
#include <stdint.h>

// Compile-time logging / diagnostics. 0 (default) = PRODUCTION build: no host->controller capture ring
// (reclaims ~80KB RAM), no per-event logging, no main-loop section timing, no WebUSB capture channel.
// Functional bits (haptic reconnect block, buzz-clear re-init, rate stats) stay. -DOPK_LOG=1 for diagnostics.
#ifndef OPK_LOG
#define OPK_LOG 0
#endif

// Build-time FACTORY RESET (recovery build). 0 (default) = normal. -DOPK_FACTORY_RESET=1 wipes ALL persistent
// storage (cfg.bin + bonds.bin) ONCE -- on the first boot after flashing -- then persists normally. A git-hash
// tag file (written after the wipe) records that this build already reset, so subsequent boots skip it. Flashing
// a DIFFERENT build re-triggers the one-time wipe. Re-pair the controller after the reset. See factoryResetOnce().
#ifndef OPK_FACTORY_RESET
#define OPK_FACTORY_RESET 0
#endif

// ---- USB presentation modes (g_usbMode). RF poll/relay is identical across all; only USB enumeration +
//      report mapping differ. ----
#define MODE_STEAM 0 // Valve puck; auto-lizard when Steam closed
#define MODE_XBOX 1 // XInput + right-pad mouse
#define MODE_SW_HORI 2 // HORIPAD (Switch console whitelist)
#define MODE_LIZARD \
	3 // Puck HID; always keyboard+mouse (ignores Steam heartbeat)
#define MODE_SW_PRO 4 // Nintendo Switch Pro Controller (057E:2009) + gyro
// Sony DualSense (054C:0CE6) + gyro + split trackpad (+ wake mouse + WebUSB panel)
#define MODE_PS5 5
// DS4-layout generic HID gamepad + gyro (+ wake mouse + WebUSB panel)
#define MODE_HIDGYRO 6
// DualSense, CLEAN single-HID (no wake/WebUSB) so PC games classify it as PlayStation (Fortnite)
#define MODE_PS5_GAME 7
// DS4, CLEAN single-HID (no wake/WebUSB) for game PlayStation classification
#define MODE_DS4_GAME 8
#define MODE_MAX 8

// The two "game" personalities drop the wake-mouse + WebUSB interfaces so the device is a genuine single-HID PS
// controller (some PC games -- e.g. Fortnite/UE GameInput -- refuse PS classification when extra interfaces are
// present). Cost: no config panel / host-wake while in these modes; chord back to Steam (back4 + A) for the panel.
static inline bool modeIsCleanPS(uint8_t m)
{
	return m == MODE_PS5_GAME || m == MODE_DS4_GAME;
}

static inline bool modeIsPuck(uint8_t m)
{
	return m == MODE_STEAM || m == MODE_LIZARD;
}
static inline bool modeValid(uint8_t m)
{
	return m <= MODE_MAX;
}

extern uint8_t g_usbMode; // loaded from flash at boot

// true for all non-puck presentations (cached !modeIsPuck(g_usbMode))
extern bool g_xbox;
extern uint8_t g_chordBtn[3]; // back4+B/X/Y -> these modes (A always STEAM)

// Mode persistence policy: by DEFAULT every fresh power-on/reconnect lands in STEAM mode (0). An explicit
// mode switch still works for the session via a ONE-SHOT bootMode (honored once, then cleared). g_persistMode
// instead remembers the last selected mode across reboots.
// false (default) = always boot Steam; true = boot into last mode
extern bool g_persistMode;
// one-shot: boot into this mode once then clear (!persistMode + explicit switch)
extern uint8_t g_bootMode;

// One-shot debug CDC. Puck mode normally DROPS the CDC serial console to free the USB endpoint its wake-mouse
// interface needs (to wake a sleeping Windows host). Arming this keeps CDC for the NEXT boot only -- dropping
// the wake mouse that boot -- to attach the serial debugger; the boot after reverts automatically.
// decision for THIS boot: true => keep CDC, skip the wake interface
extern bool g_debugCdcThisBoot;
void armDebugCdcNextBoot(); // persist the one-shot (caller reboots)

// persisted, runtime-tunable config:
extern int g_mDiv, g_mFric; // xbox/lizard mouse sensitivity divisor / friction%
extern uint8_t g_abSwap; // 1 = swap A/B and X/Y (Nintendo face-button layout)

// back paddles L4,R4,L5,R5 -> button codes (0..15 standard, 16=PS Touch Click, 17=PS5 Mute)
extern uint8_t g_back[4];
// QAM (3 dots) physical button -> same code space (0 = default/unmapped)
extern uint8_t g_qamMap;
// rumble strength, percent of decoded amplitude (100 = 1x, 200 = 2x default), all modes
extern uint8_t g_rumbleScale;
// Switch Pro motion settings. Persisted in their OWN flash file (mode_switch_pro.cpp), NOT in Cfg -- so changing
// them never resets the rest of the config. Set from the WebUSB panel.
// Switch Pro report cadence: 0 = 66Hz (15ms, compat), 1 = 120Hz (8ms, DEFAULT), 2 = full (~250Hz)
extern uint8_t g_swProRate;
// Switch Pro gyro sensitivity x10 (10 = 1.0x default; 5/15/20/25/30 = 0.5..3.0x)
extern uint8_t g_swGyroScale10;

// persist g_swProRate + g_swGyroScale10 to their flash file
void swProSaveCfg();

// 250 Hz -- matches SC2 input report rate (1000000/250 = 4000 us)
#define POLL_US_DEFAULT 4000u
// host-side HID stream cadence for translated modes (~250 Hz)
#define USB_STREAM_MS 4u
// RF poll cadence (us). FIXED -- not configurable (see loadCfg).
extern const uint32_t g_pollUs;

// loop-timing diagnostics (defined in OpenPuck.ino) -- surfaced in the WebUSB status blob to find what caps
// the poll rate: avg loop period, slowest section index, and that section's avg us/iteration.
extern uint16_t g_loopPeriodUs;
extern uint8_t g_loopWorst;
extern uint16_t g_loopWorstUs;

void loadCfg();
void saveCfg();
// FULL factory wipe: reformat the internal LittleFS, erasing cfg.bin (modes/tunables/chords) AND bonds.bin
// (paired controller). Irreversible; the caller reboots so the next boot comes up on clean defaults and the
// controller must be re-paired. Gated behind explicit confirmation at every call site.
void factoryErase();
// One-time factory reset for the -DOPK_FACTORY_RESET recovery build: wipe ONCE on the first boot after flashing
// (tracked by a git-hash tag file so it doesn't wipe on every boot), then persist normally. buildTag = OPK_GIT_HASH.
void factoryResetOnce(const char *buildTag);
// Mode switch (chord / WebUI): persist mode if the toggle is on, else arm a one-shot so this reboot lands in
// the new mode but the next cold boot returns to Steam. Either way saveCfg + caller reboots.
void saveMode(uint8_t m);
