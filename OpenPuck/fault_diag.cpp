#include "fault_diag.h"
#include <Arduino.h> // readResetReason(), NRF_POWER, NVIC_SystemReset, POWER_RESETREAS_*_Msk
#include <FreeRTOS.h>
#include <task.h> // uxTaskGetSystemState -> per-task stack high-water (overflow hypothesis check)
#include <string.h>
#include "rf_link.h" // g_pollsps/g_relayps/g_crcps/g_norxps/g_rfStallRecover -- flight-recorder vitals
#include "haptics.h" // g_ringFault, relayPending() -- relay-load vitals

// GPREGRET2 markers. GPREGRET (id 0) is reserved by the Adafruit bootloader for the DFU magic; GPREGRET2 is
// free for the application and is retained across soft/watchdog/pin reset, cleared only on power-on/brownout.
// Arbitrary non-zero tags -- any value distinct from each other and from the power-on default (0) works.
#define G2_INTENT 0xB1u
#define G2_FAULT 0xFAu

static uint8_t g_reason = RR_UNKNOWN;
static uint32_t g_resetReas = 0;
static uint8_t g_hangStage = 0xFF;

// ---- watchdog pre-reset PC capture ("software SWD") --------------------------------------------------------
// The nRF52 WDT raises a TIMEOUT interrupt ~2 LFCLK cycles BEFORE it resets the chip. We take that interrupt at
// the highest NVIC priority (0) -- above FreeRTOS/TinyUSB BASEPRI critical sections -- read the stacked PC of
// whatever code was wedged, and stash it in a .noinit RAM struct that survives the reset. Next boot reports it
// on the panel; map it with addr2line to name the stuck function. Caveat: this fires only if interrupts aren't
// hard-masked (PRIMASK/CPSID). If a watchdog reset reports NO pc, the hang was a PRIMASK-off / flash-stall type
// (a different, narrower class) -- itself a useful signal. .noinit isn't zeroed by the C runtime, so it rides
// through the watchdog reset (unlike GPREGRET2, which this board's bootloader wipes).
struct HangFrame {
	uint32_t pc, lr, magic;
};
#define HANGPC_MAGIC 0x48414E47u // "HANG"
__attribute__((section(".noinit"))) volatile struct HangFrame g_hangFrame;
static uint32_t g_reportHangPC = 0, g_reportHangLR = 0;

__attribute__((naked)) void WDT_IRQHandler(void)
{
	__asm volatile(
		"tst lr, #4            \n" // EXC_RETURN bit2: 0=frame on MSP, 1=on PSP
		"ite eq                \n"
		"mrseq r0, msp         \n"
		"mrsne r0, psp         \n"
		"ldr r1, [r0, #24]     \n" // stacked PC  (frame[6])
		"ldr r3, [r0, #20]     \n" // stacked LR  (frame[5])
		"ldr r2, =g_hangFrame  \n"
		"str r1, [r2, #0]      \n"
		"str r3, [r2, #4]      \n"
		"ldr r1, =0x48414E47   \n"
		"str r1, [r2, #8]      \n"
		"b .                   \n" // hold until the WDT reset lands (microseconds away)
		".ltorg                \n");
}

// Enable the WDT TIMEOUT interrupt at top priority. Call once, right after NRF_WDT->TASKS_START in setup().
void faultDiagArmHangCapture()
{
	NRF_WDT->INTENSET = WDT_INTENSET_TIMEOUT_Msk;
	NVIC_SetPriority(
		WDT_IRQn,
		0); // above FreeRTOS configMAX_SYSCALL priority -> fires through BASEPRI guards
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);
}
uint32_t faultDiagHangPC()
{
	return g_reportHangPC;
}
uint32_t faultDiagHangLR()
{
	return g_reportHangLR;
}

// Hang breadcrumb: loop() stage written to GPREGRET2 as 0x80|stage. 0x80..0x9F can never collide with the
// intentional-reboot (0xB1) / HardFault (0xFA) markers or the power-on default (0), so the same register
// disambiguates all of them at boot.
#define G2_STAGE_FLAG 0x80u
static const char *const STAGE_STR[] = { "webusb", "ctrl.task", "serial",
					 "rfdiag", "rflink",	"haptic",
					 "led",	   "usbmount",	"usbtx" };
#define STAGE_COUNT ((uint8_t)(sizeof STAGE_STR / sizeof STAGE_STR[0]))

static const char *const REASON_STR[RR_COUNT] = {
	"unknown",   "power-on", "pin/replug", "WATCHDOG (hang)", "CPU lockup",
	"HARDFAULT", "reboot",	 "soft reset", "wake-from-off",
};

// HardFault override. The Adafruit core's default handler (cores/.../debug.cpp) already does NVIC_SystemReset();
// we replace it to first stamp the fault marker so the NEXT boot classifies this SREQ as RR_HARDFAULT rather
// than an intentional reboot. The core's strong symbol is only linked to satisfy the vector table, so our own
// strong definition takes its place. Keep this MINIMAL: we are in fault context on a possibly-corrupt stack --
// no Serial, no allocations, just stamp and reset.
extern "C" void HardFault_Handler(void)
{
	NRF_POWER->GPREGRET2 = G2_FAULT;
	NVIC_SystemReset();
	while (1) {
	}
}

void faultDiagArmIntentionalReset()
{
	NRF_POWER->GPREGRET2 = G2_INTENT;
}

// Live stage + loop heartbeat. The post-reset GPREGRET2 breadcrumb only works if the bootloader preserves
// GPREGRET2 across a watchdog reset -- some clone boards clear it, so it reads blank. These LIVE values don't
// depend on surviving a reset: the WebUSB blob send runs on the USB SOF interrupt (independent of loop()), so
// when loop() wedges, the SOF drain can still report the current stage + how long loop() has been stuck,
// straight to the panel during the ~8s before the watchdog fires.
static volatile uint8_t g_curStage = 0xFF;
static volatile uint32_t g_loopBeatMs = 0;

void faultDiagSetStage(uint8_t stage)
{
	g_curStage = stage;
	NRF_POWER->GPREGRET2 = (uint8_t)(G2_STAGE_FLAG | (stage & 0x1Fu));
}

void faultDiagBeat()
{
	g_loopBeatMs = millis();
}
uint8_t faultDiagCurStage()
{
	return g_curStage;
}
uint32_t faultDiagStallMs()
{
	// ms since loop() last beat. ~0 when healthy (loop beats ~250x/s); grows while loop() is stuck.
	return (uint32_t)(millis() - g_loopBeatMs);
}
const char *faultDiagStageStr(uint8_t s)
{
	return s < STAGE_COUNT ? STAGE_STR[s] : "?";
}

uint8_t faultDiagHangStage()
{
	return g_hangStage;
}
const char *faultDiagHangStageStr()
{
	return g_hangStage < STAGE_COUNT ? STAGE_STR[g_hangStage] : "n/a";
}

// ---- flight recorder (data) --------------------------------------------------------------------------------
// The live ring lives in .noinit so it survives a watchdog reset (like g_hangFrame). At boot we copy the
// pre-reset trail into g_flightSaved (ordinary RAM -- no need to survive twice) so the live session can start a
// fresh ring while the console can still re-dump the old trail on demand. Declared here (above faultDiagBoot)
// because the boot classifier snapshots + dumps it; the push/tick/dump functions are implemented further down.
struct FRRec {
	uint32_t ms; // millis() at the push
	uint16_t arg; // event-specific
	uint8_t evt; // FR_*
	uint8_t stage; // loop stage current at the push (g_curStage)
};
#define FR_RING 96u
#define FR_MAGIC 0x464C4954u // "FLIT"
struct FlightRec {
	uint32_t magic;
	uint16_t head; // next write slot
	uint16_t count; // total pushed this session (saturating) -> "showing last N of M"
	FRRec ring[FR_RING];
	// live vitals, refreshed ~4x/s -- the last values captured before a wedge
	uint32_t vMs; // millis() at last refresh
	uint32_t loopPerSec; // loop() iterations in the last full second
	uint32_t heapUsed; // mallinfo().uordblks (bytes) -- trend up = leak/fragmentation
	uint16_t usbdStackFree, loopStackFree; // words (0 = overflowed)
	uint16_t pollsps, relayps, crcps, norxps;
	uint16_t rfHeal, ringFault;
	uint8_t curStage;
	uint8_t stallMs; // ms since last loop beat, capped 255
};
__attribute__((section(".noinit"))) static volatile struct FlightRec g_flight;
static struct FlightRec
	g_flightSaved; // boot-time copy of the pre-reset trail (BSS)
static bool g_haveSaved = false;
bool g_vitals = true; // live per-second CDC vitals line (console "VIT" toggles)

static const char *const FR_STR[] = { "none",  "beat",	  "SET",    "GET",
				      "relay", "rf-up",	  "rf-DN",  "HEAL!",
				      "mount", "SUSPEND", "resume", "OFF",
				      "RINGF", "save" };
#define FR_STR_COUNT ((uint8_t)(sizeof FR_STR / sizeof FR_STR[0]))
static const char *frEvtStr(uint8_t e)
{
	return e < FR_STR_COUNT ? FR_STR[e] : "?";
}

// ---- flash black box (pre-watchdog dump) -------------------------------------------------------------------
// On boards whose bootloader wipes .noinit + GPREGRET2 across a watchdog reset (Teyleten-class), EVERY
// post-mortem channel above is destroyed -- hangs on those boards were unforensicable. Flash survives any
// reset, so: a 4 Hz TIMER4 interrupt (priority 1, above FreeRTOS BASEPRI masking, below the WDT capture's 0)
// watches the loop heartbeat; when loop() has been frozen ~6 s (the WDT resets at 8 s), it walks the loop and
// usbd tasks' TCBs for their stacked PCs (both tasks BLOCKED = the suspected mutual-deadlock class) plus the
// PC it interrupted (covers the one-task-spinning class), and writes one record to a reserved raw flash page.
// Next boot reports it on CDC and feeds the existing hangPC/hangStage panel channels. Caveats: fires only if
// priority-1 interrupts still run (PRIMASK-off hangs leave no record -- same caveat as the WDT capture, and
// itself a signal); a task that is RUNNING (not blocked) has a stale TCB pxTopOfStack, so cross-check its PC
// against irqPC.
#define BB_ADDR \
	0xE8000UL // raw page: app image (~170 KB, ends < 0x60000) < here < InternalFS (0xED000)
#define BB_MAGIC 0x62627831u // "bbx1"
#define BB_WORDS 12
static TaskHandle_t g_hLoop,
	g_hUsbd; // captured by faultDiagStackTick's 1 Hz sweep
// non-static (like g_hangFrame): the naked ISR's `ldr =symbol` needs external linkage to resolve
extern "C" {
volatile uint32_t g_bbIrqPC,
	g_bbIrqLR; // stacked frame of whatever TIMER4 preempted
}
static void bbBootReport(
	bool isHang); // defined below faultDiagBlackBoxArm; called from faultDiagBoot
static void bbErase(void); // dump-time lazy erase (see bbTimerBody)

static void bbFlashWord(uint32_t addr, uint32_t v)
{
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
	__DMB();
	*(volatile uint32_t *)addr = v;
	while (!NRF_NVMC->READY) {
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
	__DMB();
}
// Stacked PC/LR of a BLOCKED FreeRTOS task. pxTopOfStack is the first TCB member (portable-layer invariant);
// the CM4F PendSV push order is [r4-r11, EXC_RETURN] then (s16-s31 iff the FPU frame was live) then the
// hardware frame [r0-r3, r12, lr, pc, xpsr]. EXC_RETURN bit4 = 0 means the FPU frame is present.
static void bbTaskPc(TaskHandle_t h, uint32_t *pc, uint32_t *lr)
{
	*pc = *lr = 0;
	if (!h)
		return;
	uint32_t *tos = *(uint32_t **)h;
	uint32_t a = (uint32_t)tos;
	if (a < 0x20000000u || a >= 0x20040000u - (26 + 8) * 4)
		return; // corrupt TCB/stack -> don't fault inside the black box
	uint32_t excret = tos[8];
	uint32_t hw = 9 + ((excret & 0x10) ? 0 : 16);
	*pc = tos[hw + 6];
	*lr = tos[hw + 5];
}
extern "C" void bbTimerBody(void)
{
	NRF_TIMER4->EVENTS_COMPARE[0] = 0;
	(void)NRF_TIMER4->EVENTS_COMPARE[0]; // readback: event write is posted
	static uint32_t lastBeat;
	static uint16_t stuck;
	static bool dumped;
	uint32_t b = g_loopBeatMs;
	if (b != lastBeat) {
		lastBeat = b;
		stuck = 0;
		return;
	}
	if (dumped)
		return;
	if (++stuck <
	    24) { // 24 ticks @ 4 Hz = 6 s frozen; WDT reset lands at 8 s
		// USBD self-heal kick before giving up. The black-boxed wedge (2026-07-03, loopPC=start_dma,
		// usbdPC=idle xQueueReceive) is a USB EasyDMA whose END event/interrupt got swallowed (clone
		// USBD silicon under concurrent RADIO EasyDMA): TinyUSB's dma_running never clears, every next
		// transfer re-defers on the TASK_PRIO_HIGH usbd task (livelock), and TASK_PRIO_LOW loop starves.
		// If the END EVENT is latched but its IRQ was lost, re-pending the USBD IRQ makes the ISR run
		// edpt_dma_end -> the defer chain unwinds -> FULL recovery, no reset (the panel trail shows a
		// recovered stall episode). Harmless when nothing is pending: a spurious USBD ISR just returns.
		// >=500ms of frozen loop heartbeat is never legitimate (bond flash saves are ms-class).
		if (stuck >= 2) {
			NVIC_SetPendingIRQ(USBD_IRQn);
			faultDiagTrace(
				FR_HEAL,
				0x05Bu); // arg 0x5B = USBD kick (radio heal passes its own count)
		}
		return;
	}
	dumped = true;
	// Lazy erase: the page still holds the PREVIOUS hang's (already-reported) record -- boot no longer
	// erases it, so its CDC banner stays re-printable on any later debug boot. An in-ISR page erase is
	// ~85ms of CPU stall, irrelevant here: the system is already dead and the WDT reset is ~2s away.
	bbErase();
	// Record v2, written in TWO PHASES: phase 1 (PCs + vitals) commits BEFORE any USBD register is
	// touched -- if the USBD peripheral has wedged the AHB matrix, reading its registers could bus-stall
	// this ISR forever (the WDT still resets: its reset path is hardware, not CPU). A record with phase 1
	// present and phase 2 erased (0xFFFFFFFF) is therefore itself the proof of an AHB/USBD bus wedge.
	uint32_t lpc, llr, upc, ulr;
	bbTaskPc(g_hLoop, &lpc, &llr);
	bbTaskPc(g_hUsbd, &upc, &ulr);
	uint32_t w1[8] = {
		BB_MAGIC,
		(2u << 24) | ((uint32_t)g_curStage << 16), // ver 2
		lpc, // w2: loop task stacked PC (where it starved/blocked)
		g_bbIrqPC, // w3: what TIMER4 preempted = the code RUNNING at dump time
		upc, // w4: usbd task stacked PC
		((uint32_t)g_flight.usbdStackFree << 16) |
			g_flight.loopStackFree, // w5
		((uint32_t)g_flight.relayps << 16) | g_flight.pollsps, // w6
		g_flight.vMs, // w7: millis at last healthy vitals ~= wedge time
	};
	(void)llr;
	(void)ulr;
	for (int i = 0; i < 8; i++)
		bbFlashWord(BB_ADDR + 4u * i, w1[i]);
	// phase 2: USBD peripheral state -- did the DMA END event ever latch? are its interrupts enabled?
	uint32_t endmask = 0;
	for (int i = 0; i < 8; i++) {
		if (NRF_USBD->EVENTS_ENDEPIN[i])
			endmask |= (1u << i);
		if (NRF_USBD->EVENTS_ENDEPOUT[i])
			endmask |= (1u << (8 + i));
	}
	if (NRF_USBD->EVENTS_EPDATA)
		endmask |= (1u << 16);
	if (NRF_USBD->EVENTS_USBEVENT)
		endmask |= (1u << 17);
	if (NRF_USBD->EVENTS_EP0DATADONE)
		endmask |= (1u << 18);
	if (NRF_USBD->EVENTS_SOF)
		endmask |= (1u << 19);
	if (NVIC_GetPendingIRQ(USBD_IRQn))
		endmask |= (1u << 20); // IRQ pending but not being serviced
	if (NVIC_GetEnableIRQ(USBD_IRQn))
		endmask |= (1u << 21);
	uint32_t w2[4] = {
		endmask, // w8: latched-event bitmap (ENDEPIN 0-7, ENDEPOUT 8-15, EPDATA/USBEVENT/EP0DD/SOF, NVIC)
		NRF_USBD->INTEN, // w9: which USBD interrupts are enabled
		NRF_USBD->EPDATASTATUS, // w10: per-endpoint data-ready flags
		((uint32_t)NRF_USBD->EPINEN << 16) |
			(NRF_USBD->EPOUTEN & 0xFFFF), // w11
	};
	for (int i = 0; i < 4; i++)
		bbFlashWord(BB_ADDR + 4u * (8 + i), w2[i]);
}
// Naked shim: grab the interrupted context's stacked PC/LR (like the WDT capture), then tail-branch to the C
// body -- LR still holds EXC_RETURN, so the body's return IS the exception return.
extern "C" __attribute__((naked)) void TIMER4_IRQHandler(void)
{
	__asm volatile("tst lr, #4            \n"
		       "ite eq                \n"
		       "mrseq r0, msp         \n"
		       "mrsne r0, psp         \n"
		       "ldr r1, [r0, #24]     \n"
		       "ldr r2, =g_bbIrqPC    \n"
		       "str r1, [r2]          \n"
		       "ldr r1, [r0, #20]     \n"
		       "ldr r2, =g_bbIrqLR    \n"
		       "str r1, [r2]          \n"
		       "b bbTimerBody         \n"
		       ".ltorg                \n");
}
void faultDiagBlackBoxArm()
{
	NRF_TIMER4->TASKS_STOP = 1;
	NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit
			      << TIMER_BITMODE_BITMODE_Pos;
	NRF_TIMER4->PRESCALER = 9; // 16 MHz / 512 = 31250 Hz
	NRF_TIMER4->CC[0] = 31250u / 4u; // 4 Hz
	NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
	NRF_TIMER4->EVENTS_COMPARE[0] = 0;
	NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
	NVIC_SetPriority(TIMER4_IRQn,
			 1); // above BASEPRI masking, below the WDT capture
	NVIC_ClearPendingIRQ(TIMER4_IRQn);
	NVIC_EnableIRQ(TIMER4_IRQn);
	NRF_TIMER4->TASKS_START = 1;
}
static void bbErase()
{
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;
	__DMB();
	NRF_NVMC->ERASEPAGE = BB_ADDR;
	while (!NRF_NVMC->READY) {
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
	__DMB();
}
// Boot-side: report the record. Runs from faultDiagBoot AFTER the .noinit hangPC/hangStage recovery, so on
// boards where those survive the black box only fills the gaps; on RAM-wiping boards it IS the evidence.
// Lifecycle: the record is NOT erased here -- a normal boot often has no CDC attached, so erasing on first
// boot would destroy the register words unread. Instead it persists (the banner re-prints on ANY later boot,
// e.g. a debug-CDC reboot) and the NEXT dump erases the page itself. A "seen" marker at word 12 keeps the
// panel adoption one-shot so a stale record can't masquerade as a later, unrelated hang's evidence.
static void bbBootReport(bool isHang)
{
	const volatile uint32_t *w = (const volatile uint32_t *)BB_ADDR;
	bool fresh = (w[12] == 0xFFFFFFFFu);
	if (w[0] != BB_MAGIC) {
		// garbage/partial page (interrupted dump or pre-feature content): clean it so the next dump
		// writes onto erased flash. All-blank needs no erase.
		bool blank = true;
		for (int i = 0; i < BB_WORDS; i++)
			if (w[i] != 0xFFFFFFFFu)
				blank = false;
		// keep a half-written phase-1 record (magic present handled below); anything else non-blank
		// without magic is garbage
		if (!blank)
			bbErase();
		return;
	}
	{
		uint8_t stage = (uint8_t)(w[1] >> 16);
		Serial.printf(
			"# BLACK BOX v%lu (flash dump written ~6s into the last hang):\n",
			(unsigned long)(w[1] >> 24));
		Serial.printf(
			"#   stage=%s  loopPC=%08lX  irqPC=%08lX (running at dump)  usbdPC=%08lX\n",
			faultDiagStageStr(stage), (unsigned long)w[2],
			(unsigned long)w[3], (unsigned long)w[4]);
		Serial.printf(
			"#   stkFree usbd=%luw loop=%luw  polls/s=%lu relay/s=%lu  tWedge~%lums\n",
			(unsigned long)(w[5] >> 16),
			(unsigned long)(w[5] & 0xFFFF),
			(unsigned long)(w[6] & 0xFFFF),
			(unsigned long)(w[6] >> 16), (unsigned long)w[7]);
		// phase 2 = USBD peripheral state; all-FF here with phase 1 present = reading USBD registers
		// bus-stalled the dump ISR = the peripheral wedged the AHB matrix (decisive by itself).
		if (w[8] == 0xFFFFFFFFu && w[9] == 0xFFFFFFFFu)
			Serial.printf(
				"#   USBD regs: UNREADABLE (dump died reading them -> AHB/USBD bus wedge)\n");
		else
			Serial.printf(
				"#   USBD: events=%08lX (ENDEPIN0-7,ENDEPOUT8-15,EPDATA16,USBEVT17,EP0DD18,SOF19,NVICpend20,NVICen21)"
				" INTEN=%08lX EPDATASTATUS=%08lX EPINEN=%04lX EPOUTEN=%04lX\n",
				(unsigned long)w[8], (unsigned long)w[9],
				(unsigned long)w[10],
				(unsigned long)(w[11] >> 16),
				(unsigned long)(w[11] & 0xFFFF));
		// feed the existing panel channels (hangLog / Last-reset tile) when the .noinit path came back
		// blank -- i.e. exactly the RAM-wiping boards this exists for. loopPC is the primary suspect
		// location; irqPC (the code actually RUNNING while loop starved) rides the LR slot. One-shot:
		// only a FRESH record (seen-marker still erased) may claim this boot's hang.
		if (isHang && fresh) {
			if (!g_reportHangPC && w[2]) {
				g_reportHangPC = w[2];
				g_reportHangLR = w[3];
			}
			if (g_hangStage == 0xFF && stage < STAGE_COUNT)
				g_hangStage = stage;
		}
		if (fresh)
			bbFlashWord(BB_ADDR + 4u * 12, 1u); // stamp "seen"
	}
}

void faultDiagBoot()
{
	uint32_t rr =
		readResetReason(); // latched + cleared by the core's init()
	uint8_t g2 = (uint8_t)NRF_POWER->GPREGRET2;
	NRF_POWER->GPREGRET2 = 0; // consume the marker for this boot cycle
	g_resetReas = rr;

	// If this boot follows a watchdog/lockup reset and GPREGRET2 holds a stage breadcrumb (0x80|stage), recover
	// which loop stage was stuck. Only meaningful for hangs -- an intentional reboot/HardFault stamps its own
	// marker over the breadcrumb, and a clean power-on zeroes it.
	bool isHang = (rr & POWER_RESETREAS_DOG_Msk) ||
		      (rr & POWER_RESETREAS_LOCKUP_Msk);
	if (isHang && (g2 & G2_STAGE_FLAG) && g2 < G2_INTENT)
		g_hangStage = (uint8_t)(g2 & 0x1Fu);
	else
		g_hangStage = 0xFF;

	// Recover the PC the WDT pre-reset ISR captured (if it fired -- i.e. the hang wasn't PRIMASK-off). Consume
	// the magic so a later watchdog reset that DIDN'T capture (PRIMASK-off) can't report a stale address.
	if (isHang && g_hangFrame.magic == HANGPC_MAGIC) {
		g_reportHangPC = g_hangFrame.pc;
		g_reportHangLR = g_hangFrame.lr;
	} else {
		g_reportHangPC = 0;
		g_reportHangLR = 0;
	}
	g_hangFrame.magic = 0;

	uint8_t reason;
	// Precedence: a physical pin reset / watchdog / lockup is unambiguous from RESETREAS; only a SREQ needs the
	// GPREGRET2 marker to split intentional reboot vs HardFault.
	if (rr & POWER_RESETREAS_RESETPIN_Msk)
		reason = RR_PIN;
	else if (rr & POWER_RESETREAS_DOG_Msk)
		reason = RR_WATCHDOG;
	else if (rr & POWER_RESETREAS_LOCKUP_Msk)
		reason = RR_LOCKUP;
	else if (rr & POWER_RESETREAS_SREQ_Msk)
		reason = (g2 == G2_FAULT)  ? RR_HARDFAULT :
			 (g2 == G2_INTENT) ? RR_REBOOT :
					     RR_SOFT;
	else if (rr & POWER_RESETREAS_OFF_Msk)
		reason = RR_WAKE;
	else if (rr == 0)
		reason = RR_POWERON; // all bits clear == power-on / brownout
	else
		reason = RR_UNKNOWN;
	g_reason = reason;

	Serial.printf(
		"# reset cause: %s (RESETREAS=0x%08lX gpregret2=0x%02X)\n",
		REASON_STR[reason], (unsigned long)rr, g2);
	if (g_hangStage != 0xFF)
		Serial.printf("# hang stage: %s (%u)\n",
			      faultDiagHangStageStr(), g_hangStage);
	if (g_reportHangPC)
		Serial.printf("# hang PC=0x%08lX LR=0x%08lX\n",
			      (unsigned long)g_reportHangPC,
			      (unsigned long)g_reportHangLR);

	// Flash black box: report + consume last hang's pre-watchdog dump (fills hangPC/stage on boards whose
	// bootloader wiped the .noinit/GPREGRET2 evidence above).
	bbBootReport(isHang);

	// Flight recorder: if the pre-reset trail survived (.noinit), snapshot it into ordinary RAM so the live
	// session can start a fresh ring while the console can still re-dump the old trail ("FR"). On a hang, dump
	// it now -- this is the post-mortem of what the firmware was doing in the seconds before it wedged.
	if (g_flight.magic == FR_MAGIC) {
		memcpy(&g_flightSaved, (const void *)&g_flight,
		       sizeof g_flightSaved);
		g_haveSaved = true;
	}
	g_flight.magic = FR_MAGIC; // start this session's trail fresh
	g_flight.head = 0;
	g_flight.count = 0;
	if (isHang)
		faultDiagDumpFlight();
}

// ---- flight recorder (implementation; data declared above faultDiagBoot) ----------------------------------
void faultDiagTrace(uint8_t evt, uint16_t arg)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (g_flight.magic !=
	    FR_MAGIC) { // lazy (re)init: power-on garbage or first push this session
		g_flight.magic = FR_MAGIC;
		g_flight.head = 0;
		g_flight.count = 0;
	}
	uint16_t h = g_flight.head;
	g_flight.ring[h].ms = millis();
	g_flight.ring[h].arg = arg;
	g_flight.ring[h].evt = evt;
	g_flight.ring[h].stage = g_curStage;
	g_flight.head = (uint16_t)((h + 1u) % FR_RING);
	if (g_flight.count < 0xFFFFu)
		g_flight.count++;
	if (!pm)
		__enable_irq();
}

void faultDiagFlightTick(void)
{
	static unsigned long beatMs = 0, secMs = 0;
	static uint32_t loops = 0;
	loops++;
	unsigned long now = millis();

	// ~4x/s: refresh the persistent vitals snapshot + drop a heartbeat crumb (its cadence in the trail shows
	// exactly when loop() stopped advancing).
	if ((uint32_t)(now - beatMs) >= 250u) {
		beatMs = now;
		uint32_t stall = faultDiagStallMs();
		g_flight.vMs = now;
		g_flight.usbdStackFree = faultDiagUsbdStackFree();
		g_flight.loopStackFree = faultDiagLoopStackFree();
		// NOTE: mallinfo() removed here -- it suspended the FreeRTOS scheduler + walked the heap free list 4x/s,
		// which is new loop latency and a new wedge surface on a slow clone, for zero payoff (heap was flat). If
		// a heap-usage trend is ever needed, sample it lazily from loop, not on this hot path.
		g_flight.heapUsed = 0;
		g_flight.pollsps = g_pollsps;
		g_flight.relayps = g_relayps;
		g_flight.crcps = g_crcps;
		g_flight.norxps = g_norxps;
		g_flight.rfHeal = g_rfStallRecover;
		g_flight.ringFault = g_ringFault;
		g_flight.curStage = g_curStage;
		g_flight.stallMs = (uint8_t)(stall > 255 ? 255 : stall);
		faultDiagTrace(FR_BEAT, (uint16_t)g_flight.stallMs);
	}

	// once per second: latch loop rate + emit the live CDC vitals line (loop context -> Serial is safe here,
	// unlike the usbd task). Reads as a trend on a flaky board: usbd stack falling toward 0 = the overflow;
	// heapUsed climbing = a leak; poll==0 = the RF loop stalled; relay high = Steam flooding the link.
	if ((uint32_t)(now - secMs) >= 1000u) {
		g_flight.loopPerSec = loops;
		loops = 0;
		secMs = now;
		// Guard on FIFO space: a CDC write to a host that has the port open but isn't draining blocks the loop --
		// itself a watchdog-hang source (same failure the WebUSB blob send hit). If there's no room this second,
		// skip the line rather than stall; the persistent trail still captured the vitals via the beat above.
		if (g_vitals && Serial && Serial.availableForWrite() >= 200) {
			// Clock fingerprint on every line: the radio waits time out on micros() (HFCLK-derived). If a clone's
			// HFCLK drops to the internal RC (or micros() freezes), usPerMs drifts off 1000 and a "bounded" RX/
			// disable wait can spin forever -- the prime suspect for a healthy-then-sudden hard lock. lf/hf: R=RC,
			// X=xtal, S=synth, -=stopped.
			const char clkc[] = { '-', 'R', 'X', 'S' };
			uint8_t lf = clockLfSrc(), hf = clockHfSrc();
			Serial.printf(
				"# vit up=%lus loop=%lu/s stall=%ums stage=%s usbdStk=%u loopStk=%u heapUsed=%lu "
				"poll=%u relay=%u crc=%u norx=%u heal=%u ringF=%u clk=%c/%c usPerMs=%u\n",
				(unsigned long)(now / 1000),
				(unsigned long)g_flight.loopPerSec,
				(unsigned)g_flight.stallMs,
				faultDiagStageStr(g_flight.curStage),
				(unsigned)g_flight.usbdStackFree,
				(unsigned)g_flight.loopStackFree,
				(unsigned long)g_flight.heapUsed,
				(unsigned)g_flight.pollsps,
				(unsigned)g_flight.relayps,
				(unsigned)g_flight.crcps,
				(unsigned)g_flight.norxps,
				(unsigned)g_flight.rfHeal,
				(unsigned)g_flight.ringFault,
				lf < 4 ? clkc[lf] : '?',
				hf < 4 ? clkc[hf] : '?',
				(unsigned)clockUsPerMs());
		}
	}
}

// Wait (bounded) for room in the CDC TX FIFO so a multi-line dump to a slow host neither drops lines nor
// blocks forever. Caps each line's wait at ~20ms -> the whole ~100-line dump can't approach the ~8s watchdog.
static void frWaitTx(uint16_t need)
{
	for (uint8_t w = 0;
	     w < 20 && Serial && Serial.availableForWrite() < need; w++)
		delay(1);
}

void faultDiagDumpFlight(void)
{
	if (!Serial) // no host attached (e.g. the automatic boot dump before the panel connects) -- use "FR" later
		return;
	if (!g_haveSaved || g_flightSaved.magic != FR_MAGIC) {
		Serial.println(
			"# flight: no pre-reset trail (recorder lost across this reset, or last boot wasn't a hang)");
		return;
	}
	const struct FlightRec *f = &g_flightSaved;
	Serial.printf(
		"# ---- FLIGHT RECORDER (trail up to the last hang) ----\n"
		"# vitals@wedge: loop=%lu/s stall=%ums stage=%s usbdStkFree=%u loopStkFree=%u heapUsed=%lu\n"
		"#               poll=%u relay=%u crc=%u norx=%u heal=%u ringFault=%u  (age %lums before reset)\n",
		(unsigned long)f->loopPerSec, (unsigned)f->stallMs,
		faultDiagStageStr(f->curStage), (unsigned)f->usbdStackFree,
		(unsigned)f->loopStackFree, (unsigned long)f->heapUsed,
		(unsigned)f->pollsps, (unsigned)f->relayps, (unsigned)f->crcps,
		(unsigned)f->norxps, (unsigned)f->rfHeal,
		(unsigned)f->ringFault, (unsigned long)0);

	uint16_t total = f->count;
	uint16_t cnt = total < FR_RING ? total : (uint16_t)FR_RING;
	if (!cnt) {
		Serial.println("# flight: trail empty");
		return;
	}
	uint16_t start = (uint16_t)((f->head + FR_RING - cnt) % FR_RING);
	uint16_t lastIdx = (uint16_t)((f->head + FR_RING - 1u) % FR_RING);
	uint32_t last = f->ring[lastIdx].ms;
	Serial.printf(
		"# %u events (of %u total); t = ms BEFORE the last recorded crumb:\n",
		cnt, total);
	for (uint16_t i = 0; i < cnt; i++) {
		uint16_t idx = (uint16_t)((start + i) % FR_RING);
		const FRRec *r = &f->ring[idx];
		frWaitTx(64);
		Serial.printf("#  -%6lu  %-8s stage=%-9s arg=0x%04X\n",
			      (unsigned long)(last - r->ms), frEvtStr(r->evt),
			      faultDiagStageStr(r->stage), r->arg);
	}
	Serial.println("# ---- end flight recorder ----");
}

// ---- WebUSB panel access to the saved trail ----------------------------------------------------------------
static uint16_t g_frDrain =
	0; // pull cursor into g_flightSaved, 0 = oldest saved event

bool faultDiagHaveFlight(void)
{
	return g_haveSaved && g_flightSaved.magic == FR_MAGIC;
}
uint16_t faultDiagFlightCount(void)
{
	if (!faultDiagHaveFlight())
		return 0;
	uint16_t t = g_flightSaved.count;
	return t < FR_RING ? t : (uint16_t)FR_RING;
}
uint16_t faultDiagFlightTotal(void)
{
	return faultDiagHaveFlight() ? g_flightSaved.count : 0;
}
void faultDiagFlightDrainReset(void)
{
	g_frDrain = 0;
}
bool faultDiagFlightPull(uint32_t *dtMs, uint8_t *evt, uint8_t *stage,
			 uint16_t *arg)
{
	uint16_t cnt = faultDiagFlightCount();
	if (g_frDrain >= cnt)
		return false;
	const struct FlightRec *f = &g_flightSaved;
	uint16_t start = (uint16_t)((f->head + FR_RING - cnt) % FR_RING);
	uint16_t idx = (uint16_t)((start + g_frDrain) % FR_RING);
	uint16_t lastIdx = (uint16_t)((f->head + FR_RING - 1u) % FR_RING);
	*dtMs = (uint32_t)(f->ring[lastIdx].ms - f->ring[idx].ms);
	*evt = f->ring[idx].evt;
	*stage = f->ring[idx].stage;
	*arg = f->ring[idx].arg;
	g_frDrain++;
	return true;
}
bool faultDiagFlightVitals(struct FaultFlightVitals *out)
{
	if (!faultDiagHaveFlight())
		return false;
	const struct FlightRec *f = &g_flightSaved;
	out->loopPerSec =
		(uint16_t)(f->loopPerSec > 0xFFFFu ? 0xFFFFu : f->loopPerSec);
	out->usbdStkFree = f->usbdStackFree;
	out->loopStkFree = f->loopStackFree;
	out->heapUsed = f->heapUsed;
	out->pollsps = f->pollsps;
	out->relayps = f->relayps;
	out->crcps = f->crcps;
	out->norxps = f->norxps;
	out->rfHeal = f->rfHeal;
	out->ringFault = f->ringFault;
	out->curStage = f->curStage;
	out->stallMs = f->stallMs;
	return true;
}

// ---- clock fingerprint -------------------------------------------------------------------------------------
static uint8_t g_clkLf = 0, g_clkHf = 0;
static uint16_t g_clkUsPerMs = 0;

void clockDiagBoot()
{
	// LFCLKSTAT: bit16 STATE(running), bits1:0 SRC (0=RC,1=Xtal,2=Synth). HFCLKSTAT: bit16 STATE, bit0 SRC
	// (0=RC,1=Xtal). The bare-metal radio needs HFXO; if HF shows RC here, RF runs on the 64 MHz internal RC.
	uint32_t lf = NRF_CLOCK->LFCLKSTAT;
	uint32_t hf = NRF_CLOCK->HFCLKSTAT;
	bool lfrun = lf & CLOCK_LFCLKSTAT_STATE_Msk;
	uint8_t lfsrc = (uint8_t)(lf & CLOCK_LFCLKSTAT_SRC_Msk);
	g_clkLf = lfrun ? (uint8_t)(lfsrc + 1) :
			  0; // 1=RC,2=Xtal,3=Synth, 0=stopped
	bool hfxtal = hf & CLOCK_HFCLKSTAT_SRC_Msk;
	g_clkHf = hfxtal ? 2 : 0; // 2=crystal, 0=RC
	Serial.printf("# clock: LFCLK=%s HFCLK=%s\n",
		      g_clkLf == 2 ? "xtal" :
		      g_clkLf == 1 ? "RC" :
		      g_clkLf == 3 ? "synth" :
				     "stopped",
		      g_clkHf == 2 ? "xtal" : "RC");
}

void clockDiagTick()
{
	// Cross-check the two time bases: count micros() (HFCLK-derived) elapsed over a ~1s millis() (LFCLK/RTC)
	// window. usPerMs = micros-delta / millis-delta; 1000 = the clocks agree. A clone whose HFCLK runs fast vs
	// its LFCLK reads >1000 here -- the exact signature behind "poll rate too high, delivered too low".
	static uint32_t u0 = 0;
	static unsigned long m0 = 0;
	static bool init = false;
	unsigned long m = millis();
	if (!init) {
		u0 = micros();
		m0 = m;
		init = true;
		return;
	}
	unsigned long dm = m - m0;
	if (dm >= 1000) {
		uint32_t du = (uint32_t)(micros() - u0);
		g_clkUsPerMs = (uint16_t)(du / dm);
		u0 = micros();
		m0 = m;
	}
}

uint8_t clockLfSrc()
{
	return g_clkLf;
}
uint8_t clockHfSrc()
{
	return g_clkHf;
}
uint16_t clockUsPerMs()
{
	return g_clkUsPerMs;
}

// ---- per-task stack headroom (usbd-overflow hypothesis check) ---------------------------------------------
// uxTaskGetSystemState (configUSE_TRACE_FACILITY=1) reports each task's stack high-water mark = the LEAST free
// stack it ever had, in words. The "usbd" task (200 words / 800 B, core-fixed) runs handleSet->relayEnqueue;
// if this trends toward 0 under haptic load, the stack overflow is confirmed. Repo-scoped, read-only, no core
// changes -- just observing FreeRTOS.
static uint16_t g_usbdStackMin = 0xFFFF;
static uint16_t g_loopStackMin = 0xFFFF;
void faultDiagStackTick()
{
	static unsigned long ms = 0;
	if ((uint32_t)(millis() - ms) < 1000)
		return;
	ms = millis();
	static TaskStatus_t st[12];
	UBaseType_t n = uxTaskGetSystemState(st, 12, NULL);
	for (UBaseType_t i = 0; i < n; i++) {
		uint16_t hw = (uint16_t)st[i].usStackHighWaterMark;
		if (!strcmp(st[i].pcTaskName, "usbd")) {
			g_hUsbd =
				st[i].xHandle; // black box: TCB to walk at dump time
			if (hw < g_usbdStackMin)
				g_usbdStackMin = hw;
		} else if (!strcmp(st[i].pcTaskName, "loop")) {
			g_hLoop = st[i].xHandle;
			if (hw < g_loopStackMin)
				g_loopStackMin = hw;
		}
	}
}
uint16_t faultDiagUsbdStackFree()
{
	return g_usbdStackMin == 0xFFFF ? 0 : g_usbdStackMin;
}
uint16_t faultDiagLoopStackFree()
{
	return g_loopStackMin == 0xFFFF ? 0 : g_loopStackMin;
}

uint8_t faultDiagReason()
{
	return g_reason;
}
uint32_t faultDiagResetReas()
{
	return g_resetReas;
}
const char *faultDiagReasonStr()
{
	return REASON_STR[g_reason < RR_COUNT ? g_reason : 0];
}
