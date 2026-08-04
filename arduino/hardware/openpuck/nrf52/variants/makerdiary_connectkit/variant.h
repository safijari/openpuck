/*
 * MakerDiary nRF52840 Connect Kit variant for OpenPuck.
 *
 * Pin assignments are taken from MakerDiary's official Zephyr board definition
 * (boards/makerdiary/nrf52840_connectkit/nrf52840_connectkit_nrf52840.dts):
 *   - Green status LED (led0)    = P1.15  (active LOW)
 *   - RGB red   (led1)           = P1.10  (active LOW)
 *   - RGB green                  = P1.11  (active LOW)
 *   - RGB blue                   = P1.12  (active LOW)
 *   - USER button (sw0)          = P1.00  (pull-up, active LOW)
 *   - Reset                      = P0.18  (gpio-as-nreset)
 *   - 32.768 kHz LF crystal (Y2) present -> USE_LFXO
 *
 * The digital pin map (variant.cpp) is an identity map: Arduino pin N maps to
 * the nRF pin index N (P0.00..P0.31 = 0..31, P1.00..P1.15 = 32..47), so the LED
 * / button macros below are just the nRF pin indices. OpenPuck drives the radio
 * and USB at register/TinyUSB level and doesn't lean on the Arduino pin
 * abstraction, so a straight identity map is the least surprising choice.
 */

#ifndef _MAKERDIARY_CONNECTKIT_H_
#define _MAKERDIARY_CONNECTKIT_H_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

#define USE_LFXO      // Connect Kit has a 32.768 kHz crystal (Y2) for LFCLK
//#define USE_LFRC    // (would be RC for LFCLK)

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif

// nRF pin index N == Arduino pin N (identity map in variant.cpp)
#define PINS_COUNT              (48)
#define NUM_DIGITAL_PINS        (48)
#define NUM_ANALOG_INPUTS       (8)
#define NUM_ANALOG_OUTPUTS      (0)

// LEDs (all common-anode / active LOW: a channel lights when driven LOW)
#define LED_GREEN0              (47)    // P1.15 standalone green status LED (led0)
#define LED_RED                 (42)    // P1.10 RGB red   (led1)
#define LED_GREEN               (43)    // P1.11 RGB green
#define LED_BLUE                (44)    // P1.12 RGB blue

#define PIN_LED                 (LED_GREEN0)
#define LED_BUILTIN             (PIN_LED)
#define LED_STATE_ON            (0)     // active LOW

#define PIN_NEOPIXEL            (PINS_COUNT)
#define NEOPIXEL_NUM            (0)

// Buttons
#define PIN_BUTTON1             (32)    // P1.00 USER button (sw0), pull-up/active LOW

// Analog inputs (AINx)
#define PIN_A0                  (2)     // P0.02 AIN0
#define PIN_A1                  (3)     // P0.03 AIN1
#define PIN_A2                  (4)     // P0.04 AIN2
#define PIN_A3                  (5)     // P0.05 AIN3
#define PIN_A4                  (28)    // P0.28 AIN4
#define PIN_A5                  (29)    // P0.29 AIN5
#define PIN_A6                  (30)    // P0.30 AIN6
#define PIN_A7                  (31)    // P0.31 AIN7
static const uint8_t A0 = PIN_A0, A1 = PIN_A1, A2 = PIN_A2, A3 = PIN_A3;
static const uint8_t A4 = PIN_A4, A5 = PIN_A5, A6 = PIN_A6, A7 = PIN_A7;
#define ADC_RESOLUTION          (12)

// NFC antenna pins
#define PIN_NFC1                (9)     // P0.09
#define PIN_NFC2                (10)    // P0.10

// Serial1 (UART) -- unused by OpenPuck; valid placeholders.
#define PIN_SERIAL1_RX          (8)
#define PIN_SERIAL1_TX          (6)

// SPI -- unused by OpenPuck; valid placeholders (not begun, so never configured).
#define SPI_INTERFACES_COUNT    (1)
#define PIN_SPI_MISO            (14)
#define PIN_SPI_MOSI            (15)
#define PIN_SPI_SCK             (13)
static const uint8_t SS   = 7;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// Wire (I2C) -- unused by OpenPuck; valid placeholders.
#define WIRE_INTERFACES_COUNT   (1)
#define PIN_WIRE_SDA            (4)     // P0.04
#define PIN_WIRE_SCL            (5)     // P0.05
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// NOTE: no EXTERNAL_FLASH_* here. The Connect Kit has an MX25R64 QSPI flash, but
// OpenPuck stores config/bonds in the nRF52840's INTERNAL flash (LittleFS), so
// the external flash is intentionally left unconfigured.

#ifdef __cplusplus
}
#endif

#endif
