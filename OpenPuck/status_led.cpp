#include "status_led.h"
#include "rf_link.h"
#include "bonds.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

#if defined(OPK_BOARD_MDBT50Q_CX_40)
#include <nrf_gpio.h>

// The borrowed RX variant does not map the CX-40's active-low P0.08 LED.
#define WAKE_LED_PIN NRF_GPIO_PIN_MAP(0, 8)
#undef WAKE_LED_ON
#define WAKE_LED_ON LOW
#endif

#define PULSE_MS 500u

uint8_t g_ledMode = LED_MODE_STATUS;
uint8_t g_ledModeB = LED_MODE_OFF;
uint8_t g_ledPinA = WAKE_LED_PIN_A;
uint8_t g_ledPinB = WAKE_LED_PIN_B;
uint8_t g_ledActiveLevel = (WAKE_LED_ON == HIGH) ? 1 : 0;
uint8_t g_ledActiveLevelB = (WAKE_LED_ON == HIGH) ? 1 : 0;

static unsigned long g_pulseMs = 0;
static bool g_lit = false;
static unsigned long g_testUntilMs = 0;
static int s_lastLevelA = -1;
static int s_lastLevelB = -1;

static void ledSetPin(uint8_t pin, int level, int &lastLevel,
		      uint8_t activeLevel)
{
	if (pin == LED_PIN_NONE)
		return;
	if (level == lastLevel)
		return;
	lastLevel = level;
	int active = activeLevel ? HIGH : LOW;
	int inactive = activeLevel ? LOW : HIGH;
	int val = (level == HIGH) ? active : inactive;
#if defined(OPK_BOARD_MDBT50Q_CX_40)
	nrf_gpio_pin_write(WAKE_LED_PIN, val);
#else
	digitalWrite(pin, val);
#endif
}

static int computeLedLevel(uint8_t mode)
{
	if (mode == LED_MODE_OFF)
		return LOW;
	if (mode == LED_MODE_ON)
		return HIGH;
	if (mode == LED_MODE_WAKE_ONLY)
		return LOW;
	if (mode == LED_MODE_HEARTBEAT) {
		unsigned long cycle = millis() % 1200u;
		bool beat = (cycle < 70u) || (cycle >= 180u && cycle < 250u);
		return beat ? HIGH : LOW;
	}
	if (anySlotLinkUp())
		return HIGH;
	if (g_pairing || anySlotConnecting()) {
		bool on = (millis() % (LED_FAST_BLINK_MS * 2)) <
			  LED_FAST_BLINK_MS;
		return on ? HIGH : LOW;
	}
	bool on = (millis() % (LED_SLOW_BLINK_MS * 2)) < LED_SLOW_BLINK_MS;
	return on ? HIGH : LOW;
}

void ledApplyPins(uint8_t pinA, uint8_t pinB, uint8_t activeLevelA,
		  uint8_t activeLevelB)
{
	ledSetPin(g_ledPinA, LOW, s_lastLevelA, g_ledActiveLevel);
	ledSetPin(g_ledPinB, LOW, s_lastLevelB, g_ledActiveLevelB);

	g_ledPinA = pinA;
	g_ledPinB = pinB;
	g_ledActiveLevel = activeLevelA ? 1 : 0;
	g_ledActiveLevelB = activeLevelB ? 1 : 0;

#if !defined(OPK_BOARD_MDBT50Q_CX_40)
	if (g_ledPinA != LED_PIN_NONE)
		pinMode(g_ledPinA, OUTPUT);
	if (g_ledPinB != LED_PIN_NONE)
		pinMode(g_ledPinB, OUTPUT);
#endif

	s_lastLevelA = -1;
	s_lastLevelB = -1;
	ledSetPin(g_ledPinA, LOW, s_lastLevelA, g_ledActiveLevel);
	ledSetPin(g_ledPinB, LOW, s_lastLevelB, g_ledActiveLevelB);
}

void ledInit()
{
#if defined(OPK_BOARD_MDBT50Q_CX_40)
	nrf_gpio_cfg_output(WAKE_LED_PIN);
#else
	if (g_ledPinA != LED_PIN_NONE)
		pinMode(g_ledPinA, OUTPUT);
	if (g_ledPinB != LED_PIN_NONE)
		pinMode(g_ledPinB, OUTPUT);
#endif
	s_lastLevelA = -1;
	s_lastLevelB = -1;
	ledSetPin(g_ledPinA, LOW, s_lastLevelA, g_ledActiveLevel);
	ledSetPin(g_ledPinB, LOW, s_lastLevelB, g_ledActiveLevelB);
}

void ledWakePulse()
{
	g_pulseMs = millis();
	g_lit = true;

	int lvlA = (g_ledMode != LED_MODE_OFF) ? HIGH : LOW;
	int lvlB = (g_ledModeB != LED_MODE_OFF) ? HIGH : LOW;
	ledSetPin(g_ledPinA, lvlA, s_lastLevelA, g_ledActiveLevel);
	ledSetPin(g_ledPinB, lvlB, s_lastLevelB, g_ledActiveLevelB);
}

void ledTriggerTest(uint32_t ms)
{
	g_testUntilMs = millis() + ms;
	ledSetPin(g_ledPinA, HIGH, s_lastLevelA, g_ledActiveLevel);
	ledSetPin(g_ledPinB, HIGH, s_lastLevelB, g_ledActiveLevelB);
}

void ledTask()
{
	if (g_testUntilMs != 0) {
		if (millis() < g_testUntilMs) {
			ledSetPin(g_ledPinA, HIGH, s_lastLevelA,
				  g_ledActiveLevel);
			ledSetPin(g_ledPinB, HIGH, s_lastLevelB,
				  g_ledActiveLevelB);
			return;
		}
		g_testUntilMs = 0;
	}

	if (g_lit) {
		if (millis() - g_pulseMs >= PULSE_MS)
			g_lit = false;
		else {
			int lvlA = (g_ledMode != LED_MODE_OFF) ? HIGH : LOW;
			int lvlB = (g_ledModeB != LED_MODE_OFF) ? HIGH : LOW;
			ledSetPin(g_ledPinA, lvlA, s_lastLevelA,
				  g_ledActiveLevel);
			ledSetPin(g_ledPinB, lvlB, s_lastLevelB,
				  g_ledActiveLevelB);
			return;
		}
	}

	if (USBDevice.suspended()) {
		ledSetPin(g_ledPinA, LOW, s_lastLevelA, g_ledActiveLevel);
		ledSetPin(g_ledPinB, LOW, s_lastLevelB, g_ledActiveLevelB);
		return;
	}

	int lvlA = computeLedLevel(g_ledMode);
	int lvlB = computeLedLevel(g_ledModeB);
	ledSetPin(g_ledPinA, lvlA, s_lastLevelA, g_ledActiveLevel);
	ledSetPin(g_ledPinB, lvlB, s_lastLevelB, g_ledActiveLevelB);
}
