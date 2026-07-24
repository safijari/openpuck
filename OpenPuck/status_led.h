// status_led.h -- LED indication of wake activity.
//
// The LED is DARK in all steady states -- including while wake is armed (host suspended) -- and flashes for
// half a second when a wake is actually sent (USBDevice.remoteWakeup()). It's a wake debugger: flash + PC
// stays asleep = resume signal was sent and the HOST ignored it (fix host-side: powercfg /deviceenablewake);
// no flash = firmware never fired (didn't see the gesture, or didn't consider the bus suspended).
//
// Board note: built with the Feather nRF52840 variant, but the usual hardware is a SuperMini "Pro Micro"
// clone. The Feather's user LED is P1.15 (D3, active high); the SuperMini's blue user LED is P0.15 (= D24 in
// the Feather pin map -- SPI MISO, unused here). We drive BOTH pins so the indicator works on either board.
// Override the pins/polarity below if your board differs.
#pragma once

// Seeed XIAO nRF52840: its user LED is a common-anode RGB (a channel lights on a
// LOW level), and the SuperMini default pin 24 lands on a QSPI flash line in this
// board's pin map. Drive the blue channel for the wake pulse (there's no second
// user LED, so A == B). When built with the RGB mode-LED feature
// (status_led.cpp OPK_RGB_LED) that path owns the LED instead; this keeps a plain
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

void ledInit(); // call once from setup(): pins to output, LED off

// call at each USBDevice.remoteWakeup() site: LED on now, off after 500ms
void ledWakePulse();
void ledTask(); // call every loop(): times out the pulse
