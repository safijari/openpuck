// config.h -- USB-presentation modes + persisted, runtime-tunable settings.
//
// One puck speaks the SAME RF protocol no matter which USB face it wears; only the USB enumeration and the
// input-report mapping change. The active face is g_usbMode, persisted to flash (cfg.bin). Tunables here
// (mouse sensitivity, back-paddle mapping, A/B swap, chord assignments, QoS) are set over the CDC console
// (serial_console.cpp) and the WebUSB panel (webusb_config.cpp) and likewise persist.
#pragma once
#include <stdint.h>

// ---- Compile-time logging / diagnostics ----
// 0 (default) = PRODUCTION build: no host->controller capture ring (reclaims ~80KB RAM), no per-event
// logging, no main-loop section timing, no WebUSB capture channel. The functional bits (haptic reconnect
// block, the buzz-clear re-init, rate stats) stay. Build the diagnostic firmware with -DOPK_LOG=1.
// The WebUSB panel reads this back in the status blob and hides its logging UI when it's 0.
#ifndef OPK_LOG
#define OPK_LOG 0
#endif

// Build-time FACTORY RESET (recovery build). 0 (default) = normal. Build with -DOPK_FACTORY_RESET=1 to produce a
// firmware that wipes ALL persistent storage (cfg.bin + bonds.bin) ONCE -- on the first boot after flashing --
// then behaves exactly like a normal build that persists settings. It is NOT a wipe-every-boot image: a git-hash
// tag file (written after the wipe) records that this build already did its one-time reset, so subsequent boots
// skip it. This makes it a safe recovery image for a bad config/bond: flash it, boot once (cleaned), keep using
// it. Flashing a DIFFERENT build re-triggers the one-time wipe; for an on-demand wipe use the serial "ERASE-ALL"
// or WebUSB "Factory erase". Re-pair the controller after the reset. See factoryResetOnce() in config.cpp.
#ifndef OPK_FACTORY_RESET
#define OPK_FACTORY_RESET 0
#endif

// ---- USB presentation modes (g_usbMode). RF poll/relay is identical across all; only USB enumeration +
//      report mapping differ. ----
#define MODE_STEAM   0   // Valve puck; auto-lizard when Steam closed
#define MODE_XBOX    1   // XInput + right-pad mouse
#define MODE_SW_HORI 2   // HORIPAD (Switch console whitelist)
#define MODE_LIZARD  3   // Puck HID; always keyboard+mouse (ignores Steam heartbeat)
#define MODE_SW_PRO  4   // Nintendo Switch Pro Controller (057E:2009) + gyro
#define MODE_PS5     5   // Sony DualSense (054C:0CE6) + gyro + split trackpad
#define MODE_HIDGYRO 6   // DS4-layout generic HID gamepad + gyro (Fortnite-friendly)
#define MODE_MAX     6

static inline bool modeIsPuck(uint8_t m){ return m==MODE_STEAM || m==MODE_LIZARD; }
static inline bool modeValid(uint8_t m){ return m<=MODE_MAX; }

extern uint8_t g_usbMode;       // loaded from flash at boot
extern bool    g_xbox;          // true for all non-puck presentations (cached !modeIsPuck(g_usbMode))
extern uint8_t g_chordBtn[3];   // back4+B/X/Y -> these modes (A always STEAM)

// Mode persistence policy: by DEFAULT every fresh power-on/reconnect lands in STEAM mode (0). An explicit
// mode switch still works for the session via a ONE-SHOT bootMode (honored once, then cleared, so the next
// cold boot reverts to Steam). The WebUI "persist last mode" toggle (g_persistMode) instead remembers the
// last selected mode across reboots.
extern bool    g_persistMode;   // false (default) = always boot Steam; true = boot into last mode
extern uint8_t g_bootMode;      // one-shot: boot into this mode once then clear (!persistMode + explicit switch)

// One-shot debug CDC. Puck mode normally DROPS the CDC serial console to free the USB endpoint its wake-mouse
// interface needs (so puck can wake a sleeping Windows host). Arming this keeps CDC for the NEXT boot only --
// dropping the wake mouse that boot -- so someone can attach the serial debugger; the boot after reverts to
// normal automatically. Mirrors the g_bootMode one-shot. Armed from the WebUSB panel or CDC 'D' command.
extern bool    g_debugCdcThisBoot;   // decision for THIS boot: true => keep CDC, skip the wake interface
void armDebugCdcNextBoot();          // persist the one-shot (caller reboots)

// persisted, runtime-tunable config:
extern int     g_mDiv, g_mFric; // xbox/lizard mouse sensitivity divisor / friction%

extern uint8_t g_abSwap;        // 1 = swap A/B and X/Y (Nintendo face-button layout)
extern uint8_t g_back[4];       // back paddles L4,R4,L5,R5 -> button codes (0..15 standard, 16=PS Touch Click, 17=PS5 Mute)

#define POLL_US_DEFAULT 4000u   // 250 Hz -- matches SC2 input report rate (1000000/250 = 4000 us)
#define USB_STREAM_MS   4u      // host-side HID stream cadence for translated modes (~250 Hz)
extern const uint32_t g_pollUs; // RF poll cadence (us). FIXED -- not configurable (see loadCfg).

// loop-timing diagnostics (defined in OpenPuck.ino) -- surfaced in the WebUSB status blob to find what caps
// the poll rate: avg loop period, slowest section index, and that section's avg us/iteration.
extern uint16_t g_loopPeriodUs;
extern uint8_t  g_loopWorst;
extern uint16_t g_loopWorstUs;

void loadCfg();
void saveCfg();
// FULL factory wipe: reformat the internal LittleFS, erasing cfg.bin (modes/tunables/chords) AND bonds.bin
// (paired controller). Irreversible; the caller reboots so the next boot comes up on clean defaults and the
// controller must be re-paired. Gated behind explicit confirmation at every call site.
void factoryErase();
// One-time factory reset for the -DOPK_FACTORY_RESET recovery build: wipe ONCE on the first boot after flashing
// (tracked by a git-hash tag file so it doesn't wipe on every boot), then persist normally. buildTag = OPK_GIT_HASH.
void factoryResetOnce(const char* buildTag);
// Mode switch (chord / WebUI): persist mode if the toggle is on, else arm a one-shot so this reboot lands in
// the new mode but the next cold boot returns to Steam. Either way saveCfg + caller reboots.
void saveMode(uint8_t m);
