// status_led.h -- LED indication of wake activity and, on RGB boards, the
// current USB mode.
//
// Plain (single-LED) boards: the LED is DARK in all steady states -- including while wake is armed (host
// suspended) -- and flashes for half a second when a wake is actually sent (USBDevice.remoteWakeup()). It's a
// wake debugger: flash + PC stays asleep = resume signal was sent and the HOST ignored it (fix host-side:
// powercfg /deviceenablewake); no flash = firmware never fired (didn't see the gesture, or didn't consider
// the bus suspended).
//
// RGB boards (XIAO nRF52840 and anything else whose variant defines LED_RED/LED_GREEN/LED_BLUE): the LED
// additionally shows a steady color for the current USB mode -- white = Steam/Lizard, green = Xbox,
// red = Switch, blue = PlayStation -- toggleable from the WebUSB panel (g_modeLed; off = the dark steady
// state above). The wake pulse rides the same channels: 500 ms white, or dark when the steady color is
// already white.
//
// Board note: built with the Feather nRF52840 variant, but the usual hardware is a SuperMini "Pro Micro"
// clone. The Feather's user LED is P1.15 (D3, active high); the SuperMini's blue user LED is P0.15 (= D24 in
// the Feather pin map -- SPI MISO, unused here). We drive BOTH pins so the indicator works on either board.
// Override the pins/polarity below if your board differs.
#pragma once

// the RGB gate below keys off the variant's LED_* pin macros
#include <Arduino.h>

// Seeed XIAO nRF52840: its user LED is a common-anode RGB (a channel lights on a
// LOW level), and the SuperMini default pin 24 lands on a QSPI flash line in this
// board's pin map. Drive the blue channel for the wake pulse (there's no second
// user LED, so A == B). When built with the RGB mode-LED feature (OPK_RGB_LED,
// auto-on for this variant) that path owns the LED instead; this keeps a plain
// -DOPK_RGB_LED=0 build correct.
#if defined(ARDUINO_XIAO_NRF52840) && !defined(WAKE_LED_PIN_A)
#define WAKE_LED_PIN_A LED_BLUE
#define WAKE_LED_PIN_B LED_BLUE
#define WAKE_LED_ON LOW
#endif

#ifndef WAKE_LED_PIN_A

// Feather: P1.15 user LED (harmless unconnected pad on SuperMini clones)
#define WAKE_LED_PIN_A LED_BUILTIN
#endif
#ifndef WAKE_LED_PIN_B

// SuperMini "Pro Micro" clone: P0.15 blue user LED (D24 in the Feather map)
#define WAKE_LED_PIN_B 24
#endif
#ifndef WAKE_LED_ON
#define WAKE_LED_ON HIGH // set LOW if your board's LED is wired active-low
#endif

// RGB mode-color support. Auto-enabled when the board variant defines all three channel pins (the Seeed XIAO
// variant does; the Adafruit Feather variant lacks LED_GREEN, so Feather/SuperMini builds keep the plain
// wake LED). Force with -DOPK_RGB_LED=0/1.
#ifndef OPK_RGB_LED
#if defined(LED_RED) && defined(LED_GREEN) && defined(LED_BLUE)
#define OPK_RGB_LED 1
#else
#define OPK_RGB_LED 0
#endif
#endif

#if OPK_RGB_LED
#ifndef RGB_LED_PIN_R
#define RGB_LED_PIN_R LED_RED
#endif
#ifndef RGB_LED_PIN_G
#define RGB_LED_PIN_G LED_GREEN
#endif
#ifndef RGB_LED_PIN_B
#define RGB_LED_PIN_B LED_BLUE
#endif
#ifndef RGB_LED_ON

// The XIAO's RGB LED is common-anode: a channel lights when its pin is driven
// LOW (the Seeed variant's LED_STATE_ON=1 is wrong for this hardware).
#define RGB_LED_ON LOW
#endif
#endif

void ledInit(); // call once from setup(): pins to output, LED off

// call at each USBDevice.remoteWakeup() site: LED on now, off after 500ms
void ledWakePulse();
void ledTask(); // call every loop(): times out the pulse

// apply the steady mode color (dark when g_modeLed=0); no-op without RGB.
// Call after the boot mode is final and whenever g_modeLed changes.
void ledShowMode();
