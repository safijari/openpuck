#include "status_led.h"
#include <Arduino.h>

#if defined(OPK_BOARD_MDBT50Q_CX_40) || defined(OPK_BOARD_PCA10059)
#define OPK_RAW_WAKE_LED 1
#include <nrf_gpio.h>

// The borrowed RX variant maps neither board's LED; both are active-low.
#if defined(OPK_BOARD_PCA10059)
#define WAKE_LED_PIN NRF_GPIO_PIN_MAP(0, 6) // LD1, green
#else
#define WAKE_LED_PIN NRF_GPIO_PIN_MAP(0, 8)
#endif
#undef WAKE_LED_ON
#define WAKE_LED_ON LOW
#endif

#define WAKE_LED_OFF ((WAKE_LED_ON) == HIGH ? LOW : HIGH)
#define PULSE_MS 500u // wake flash duration

static unsigned long g_pulseMs = 0;
static bool g_lit = false;

static void ledWrite(int level)
{
#if defined(OPK_RAW_WAKE_LED)
	nrf_gpio_pin_write(WAKE_LED_PIN, level);
#else
	digitalWrite(WAKE_LED_PIN_A, level);
	digitalWrite(WAKE_LED_PIN_B, level);
#endif
}

void ledInit()
{
#if defined(OPK_RAW_WAKE_LED)
	nrf_gpio_cfg_output(WAKE_LED_PIN);
#else
	pinMode(WAKE_LED_PIN_A, OUTPUT);
	pinMode(WAKE_LED_PIN_B, OUTPUT);
#endif
	ledWrite(WAKE_LED_OFF);
}

void ledWakePulse()
{
	g_pulseMs = millis();
	g_lit = true;

	// light immediately at the remoteWakeup() call site, not on the next loop
	ledWrite(WAKE_LED_ON);
}

void ledTask()
{
	if (g_lit && millis() - g_pulseMs >= PULSE_MS) {
		g_lit = false;
		ledWrite(WAKE_LED_OFF);
	}
}
