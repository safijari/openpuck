// fault_diag.h -- "why did we reboot?" diagnostic for issue #72 (intermittent disconnects).
//
// Field reports conflate THREE different events that all look identical from the host (USB re-enumerates, the
// WebUSB panel shows "RF Link down"):
//   1. a genuine RF link drop -- NO reboot, the link self-recovers in seconds (handled in rf_link, not here);
//   2. a loop() hang -- the ~8s hardware watchdog fires and resets the MCU (red LED);
//   3. an MCU fault -- a HardFault triggers an immediate reset (the Adafruit core's default handler resets).
// Without recording which one happened, (2) and (3) are indistinguishable in the field, so the thread keeps
// guessing. This module classifies the cause of each boot and surfaces it (boot banner + WebUSB panel).
//
// How: the nRF52 RESETREAS register already distinguishes watchdog (DOG) / software (SREQ) / pin (RESETPIN) /
// lockup resets; the Adafruit core latches it at boot (readResetReason()). A SREQ alone can't tell "we rebooted
// on purpose" (mode-switch chord, config reboot, DFU) from "a HardFault reset us" -- so we stamp the GPREGRET2
// retention register (retained across soft/watchdog/pin reset, cleared only on power-on; GPREGRET itself is
// reserved by the bootloader for the DFU magic): faultDiagArmIntentionalReset() before every deliberate
// NVIC_SystemReset, and the HardFault handler stamps a distinct fault marker.
#pragma once
#include <stdint.h>

// Boot-cause classification (kept in sync with REASON_STR[] in fault_diag.cpp).
enum {
	RR_UNKNOWN = 0,
	RR_POWERON, // cold power-on / brownout
	RR_PIN, // reset pin / cable replug
	RR_WATCHDOG, // WDT fired -> loop() stopped feeding it (a hang)
	RR_LOCKUP, // CPU lockup
	RR_HARDFAULT, // HardFault -> our handler reset us
	RR_REBOOT, // intentional NVIC_SystemReset (mode switch / config / DFU)
	RR_SOFT, // software reset, unattributed
	RR_WAKE, // wake from System OFF
	RR_COUNT,
};

// Classify + log the cause of THIS boot. Call once, early in setup() (after Serial.begin so the line lands).
void faultDiagBoot();
// Stamp "the reset about to happen is intentional" -- call immediately before a deliberate NVIC_SystemReset so
// the next boot classifies it as RR_REBOOT, not RR_HARDFAULT.
void faultDiagArmIntentionalReset();

// ---- hang-stage breadcrumb -------------------------------------------------------------------------------
// loop() writes the stage it is ENTERING to GPREGRET2 (encoded 0x80|stage, distinct from the intentional/fault
// markers and from 0 during normal SREQ). If loop() wedges, the ~8s watchdog fires; GPREGRET2 survives that
// reset, so the next boot can report WHICH loop stage was stuck -- the missing piece for the clone hangs.
// Stage indices match the OPK_LOG timing order: 0 webusb,1 ctrl.task,2 serial,3 rfdiag,4 rflink,5 haptic,
// 6 led,7 usbmount,8 usbtx.
void faultDiagSetStage(uint8_t stage);
// The stage loop() was in when the last WATCHDOG/LOCKUP reset fired (0xFF = last boot wasn't a hang, or the
// breadcrumb didn't survive -- some clone bootloaders clear GPREGRET2). String form for the panel/console.
uint8_t faultDiagHangStage();
const char *faultDiagHangStageStr();

// Live hang localization (does NOT need a reset to survive): faultDiagBeat() is called once per loop()
// iteration; faultDiagStallMs() is ms since that last beat (~0 healthy, grows while loop() is wedged), and
// faultDiagCurStage() is the stage loop() is currently in. The SOF-driven blob reports these so the panel can
// show "STALLED in <stage>" live, before the watchdog even fires.
void faultDiagBeat();
uint8_t faultDiagCurStage();
uint32_t faultDiagStallMs();
const char *faultDiagStageStr(uint8_t s);

// Watchdog pre-reset PC capture ("software SWD"): arm in setup() right after NRF_WDT->TASKS_START. After a
// watchdog hang, faultDiagHangPC()/LR() return the PC/LR of the stuck code (0 if the hang hard-masked
// interrupts so the capture ISR couldn't run). Map the PC with addr2line on the build .elf.
void faultDiagArmHangCapture();
// Flash black box: 4 Hz TIMER4 watchdog-of-the-watchdog; at ~6s of loop stall it dumps both tasks' stacked
// PCs + vitals to a reserved flash page (survives ANY reset, incl. boards that wipe .noinit/GPREGRET2).
// Reported + consumed by faultDiagBoot on the next boot. Arm once from setup(), after the WDT starts.
void faultDiagBlackBoxArm();
uint32_t faultDiagHangPC();
uint32_t faultDiagHangLR();

// Per-task stack headroom (words of stack never used = free). Call faultDiagStackTick() from loop() (self-gated
// to ~1 Hz). faultDiagUsbdStackFree() trending toward 0 under haptic load confirms the usbd-task overflow.
void faultDiagStackTick();
uint16_t faultDiagUsbdStackFree();
uint16_t faultDiagLoopStackFree();
// Last-boot classification (RR_*) + the raw RESETREAS, for the WebUSB panel / console.
uint8_t faultDiagReason();
uint32_t faultDiagResetReas();
const char *faultDiagReasonStr();

// ---- flight recorder (catch a watchdog hang post-mortem) -------------------------------------------------
// A .noinit ring of recent high-level events + a live vitals snapshot, BOTH surviving a watchdog reset (same
// mechanism as the hang PC frame: .noinit RAM isn't zeroed by the C runtime, so it rides through the reset even
// on clone boards whose bootloader wipes GPREGRET2). A hang drops USB the instant it resets, so live CDC
// logging can never catch the moment itself -- instead we record the trail continuously and DUMP IT ON THE NEXT
// BOOT when the last reset was a hang. Every push is a few stores under PRIMASK: safe to call from the fragile
// 800B usbd task and from ISRs (NO printf, NO allocation). This is the "what was going on when it wedged" the
// cheap boards (Teyleten hangs ~every 4 min) need. `arg` is event-specific, usually packed (high<<8)|low.
enum {
	FR_NONE = 0,
	FR_BEAT, // periodic loop heartbeat (arg = stall ms) -- the gap between beats localizes the wedge in time
	FR_SET, // usbd handleSet feature write     (arg = (rid<<8)|cmd)
	FR_GET, // usbd handleGet feature read      (arg = (rid<<8)|cmd)
	FR_RELAY, // host->controller relay enqueued (arg = (slot<<8)|rid)
	FR_RFUP, // RF (re)connect                   (arg = slot)
	FR_RFDN, // RF link down                     (arg = slot)
	FR_HEAL, // RF-stall radio self-heal (power-cycle)
	FR_MOUNT, // USB HID mount/unmount changed   (arg = mounted-slot count)
	FR_SUSP, // USB bus suspend edge
	FR_RESUME, // USB bus resume edge
	FR_OFF, // controller power-off (0x9F) relayed (arg = slot)
	FR_RINGF, // relay-ring fault caught          (arg = running count)
	FR_SAVE, // bond flash write (saveBonds)
};

// Push one event into the flight recorder. Cheap + ISR/usbd-safe (a few stores under PRIMASK, no printf).
void faultDiagTrace(uint8_t evt, uint16_t arg);
// Per-loop upkeep: refresh the persistent vitals snapshot + emit a heartbeat (~4x/s), and stream the live
// per-second vitals line over CDC when g_vitals is on. Call once per loop() iteration.
void faultDiagFlightTick(void);
// Print the pre-reset flight trail + vitals-at-wedge over CDC (from the boot-time copy of the surviving ring).
// Called automatically from faultDiagBoot() after a hang; also re-dumpable on demand from the console ("FR").
void faultDiagDumpFlight(void);

// ---- WebUSB panel access to the saved (pre-reset) trail ---------------------------------------------------
// The same boot-time copy the console "FR" dumps, exposed for the panel to stream as 0xA8 frames. Mirrors the
// hapLog capture drain: reset the cursor to the oldest saved event, then pull entries until exhausted, so a
// full 96-event trail streams across several panel polls without a single blocking write.
bool faultDiagHaveFlight(
	void); // true if a pre-reset trail was captured at this boot
uint16_t faultDiagFlightCount(void); // saved events available to stream (<= 96)
uint16_t faultDiagFlightTotal(
	void); // total events pushed before the reset (for "showing N of M")
void faultDiagFlightDrainReset(
	void); // rewind the pull cursor to the oldest saved event
// Pull the next saved event; false when exhausted. *dtMs = ms the event fired BEFORE the last recorded crumb.
bool faultDiagFlightPull(uint32_t *dtMs, uint8_t *evt, uint8_t *stage,
			 uint16_t *arg);
// Vitals as of the last ~4Hz refresh before the reset (the "@wedge" snapshot). false if no trail was captured.
struct FaultFlightVitals {
	uint16_t loopPerSec;
	uint16_t usbdStkFree, loopStkFree;
	uint32_t heapUsed;
	uint16_t pollsps, relayps, crcps, norxps, rfHeal, ringFault;
	uint8_t curStage, stallMs;
};
bool faultDiagFlightVitals(struct FaultFlightVitals *out);
// 1 = stream the live per-second vitals line over CDC (console "VIT" toggles). Watch usbd stack / heap / relay
// rate trend on a flaky board in the seconds before it wedges.
extern bool g_vitals;

// ---- clock fingerprint (clone-board stability diagnostic) -------------------------------------------------
// nice!nano clones vary in their crystals. The bare-metal radio needs HFXO (32 MHz xtal); millis()/RTC and the
// watchdog run on LFCLK (32.768 kHz xtal or RC). If a clone runs HFCLK on the internal RC, or its LFCLK is
// off-rate, the two time bases diverge -- which inflates the micros()-gated poll rate, shortens the RX window
// (delivered drops), and skews the LFCLK-based watchdog. These let the panel show, per board: which source
// each clock is actually on, and the measured micros()/millis() ratio (ideal 1000 us per ms).
//   LF code: 0=stopped, 1=RC, 2=crystal, 3=synth.  HF code: 0=RC(running or not), 2=crystal.
void clockDiagBoot(); // read+log the clock sources once at boot
void clockDiagTick(); // call from loop(); recomputes usPerMs about once a second
uint8_t clockLfSrc();
uint8_t clockHfSrc();
uint16_t
clockUsPerMs(); // measured micros() advanced per millis() tick (ideal 1000; 0 until first sample)
