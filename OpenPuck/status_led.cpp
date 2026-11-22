#include "status_led.h"
#include <Arduino.h>

#if defined(OPK_BOARD_MDBT50Q_CX_40) || defined(OPK_BOARD_MDK_USB_DONGLE)
#include <nrf_gpio.h>
#endif

#if defined(OPK_BOARD_MDBT50Q_CX_40)

// The borrowed RX variant does not map the CX-40's active-low P0.08 LED.
#define WAKE_LED_PIN NRF_GPIO_PIN_MAP(0, 8)
#undef WAKE_LED_ON
#define WAKE_LED_ON LOW
#endif

#if defined(OPK_BOARD_MDK_USB_DONGLE)
// The MDK RGB channels are active LOW. Keep blue for normal operation and
// replace it with red during the 500 ms USB remote-wake pulse.
#define OPK_MDK_LED_RED_PIN NRF_GPIO_PIN_MAP(0, 23)
#define OPK_MDK_LED_GREEN_PIN NRF_GPIO_PIN_MAP(0, 22)
#define OPK_MDK_LED_BLUE_PIN NRF_GPIO_PIN_MAP(0, 24)
#define OPK_MDK_LED_ON LOW
#define OPK_MDK_LED_OFF HIGH
#undef WAKE_LED_ON
#define WAKE_LED_ON OPK_MDK_LED_ON
#endif
#define WAKE_LED_OFF ((WAKE_LED_ON) == HIGH ? LOW : HIGH)
#define PULSE_MS 500u // wake flash duration

static unsigned long g_pulseMs = 0;
static bool g_lit = false;

static void ledWrite(int level)
{
#if defined(OPK_BOARD_MDK_USB_DONGLE)
	const bool active = (level == WAKE_LED_ON);
	nrf_gpio_pin_write(OPK_MDK_LED_BLUE_PIN,
			   active ? OPK_MDK_LED_OFF : OPK_MDK_LED_ON);
	nrf_gpio_pin_write(OPK_MDK_LED_RED_PIN,
			   active ? OPK_MDK_LED_ON : OPK_MDK_LED_OFF);
	nrf_gpio_pin_write(OPK_MDK_LED_GREEN_PIN, OPK_MDK_LED_OFF);
#elif defined(OPK_BOARD_MDBT50Q_CX_40)
	nrf_gpio_pin_write(WAKE_LED_PIN, level);
#else
	digitalWrite(WAKE_LED_PIN_A, level);
	digitalWrite(WAKE_LED_PIN_B, level);
#endif
}

void ledInit()
{
#if defined(OPK_BOARD_MDK_USB_DONGLE)
	nrf_gpio_cfg_output(OPK_MDK_LED_BLUE_PIN);
	nrf_gpio_cfg_output(OPK_MDK_LED_RED_PIN);
	nrf_gpio_cfg_output(OPK_MDK_LED_GREEN_PIN);
#elif defined(OPK_BOARD_MDBT50Q_CX_40)
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
