/*
 * MakerDiary nRF52840 Connect Kit variant for OpenPuck.
 *
 * Identity digital-pin map (Arduino pin N -> nRF pin index N) and an
 * initVariant() that parks the LEDs off. See variant.h for the pin sourcing
 * (MakerDiary's official DTS).
 */

#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "nrf.h"

const uint32_t g_ADigitalPinMap[PINS_COUNT] =
{
    // P0.00 .. P0.31  ->  Arduino 0 .. 31
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
    // P1.00 .. P1.15  ->  Arduino 32 .. 47
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
};

// This board runs nosd: its UF2 bootloader hands the application control at 0x1000 with no
// SoftDevice, and does NOT point the CPU's vector table at the app. So the app must set VTOR to
// its own vector table itself -- and it has to happen BEFORE main()/init(), because init() enables
// the first interrupts (USB power detect, systick); if one fires while VTOR still points at stale
// handlers the chip faults and resets. A constructor runs during __libc_init_array, ahead of
// main(), so it is early enough. (Doing this only in initVariant(), which runs AFTER init(), is
// too late and reset-loops.)
__attribute__((constructor(101))) static void ck_set_vtor(void)
{
    SCB->VTOR = 0x1000;
}

void initVariant()
{
    SCB->VTOR = 0x1000; // redundant with the constructor above; harmless belt-and-suspenders.

    // Park all LEDs off. They are common-anode / active LOW, so OFF = drive HIGH.
    // (Raw registers so this doesn't depend on the Arduino pin layer being up yet.)
    NRF_P1->PIN_CNF[10] = 1; // RGB red   P1.10, DIR=output
    NRF_P1->PIN_CNF[11] = 1; // RGB green P1.11
    NRF_P1->PIN_CNF[12] = 1; // RGB blue  P1.12
    NRF_P1->PIN_CNF[15] = 1; // green LED P1.15
    NRF_P1->OUTSET = (1u << 10) | (1u << 11) | (1u << 12) | (1u << 15);
}
