#include "xinput_auth.h"
#include "src/libxsm3/xsm3.h"
#include <Arduino.h>
#include <stdio.h>

static TaskHandle_t g_authWorker;

uint32_t xinputAuthLock()
{
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	return mask;
}

void xinputAuthUnlock(uint32_t mask)
{
	__set_PRIMASK(mask);
}

void xinputAuthBoardSerial(char serial[13])
{
	snprintf(serial, 13, "%04lX%08lX",
		 (unsigned long)(NRF_FICR->DEVICEID[1] & 0xFFFF),
		 (unsigned long)NRF_FICR->DEVICEID[0]);
}

static void authWorker(void *)
{
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		xinputAuthProcess();
	}
}

bool xinputAuthStartWorker()
{
	if (g_authWorker)
		return true;
	// Adafruit's USB task has only 200 words of stack. Crypto gets its own
	// stack below the RF loop's priority, leaving USB polling responsive.
	return xTaskCreate(authWorker, "xsm3", 2048, nullptr, tskIDLE_PRIORITY,
			   &g_authWorker) == pdPASS;
}

void xinputAuthNotify()
{
	if (g_authWorker)
		xTaskNotifyGive(g_authWorker);
}

void xinputAuthYield()
{
	// taskYIELD() cannot schedule below the continuously runnable RF loop.
	// Lend the worker one tick between polls, never inside an RF RX window.
	if (xinputAuthBusy())
		delay(1);
}

extern "C" bool xsm3_random_bytes(uint8_t *buffer, size_t length)
{
	// OpenPuck uses the radio directly, without enabling the SoftDevice;
	// no other firmware subsystem owns RNG. Do not use Arduino random().
	NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Enabled << RNG_CONFIG_DERCEN_Pos;
	NRF_RNG->EVENTS_VALRDY = 0;
	NRF_RNG->TASKS_START = 1;
	unsigned long started = millis();
	for (size_t i = 0; i < length;) {
		if (NRF_RNG->EVENTS_VALRDY) {
			buffer[i++] = (uint8_t)NRF_RNG->VALUE;
			NRF_RNG->EVENTS_VALRDY = 0;
		} else if (millis() - started > 100) {
			NRF_RNG->TASKS_STOP = 1;
			return false;
		}
	}
	NRF_RNG->TASKS_STOP = 1;
	return true;
}