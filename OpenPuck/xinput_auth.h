#pragma once
#include <Adafruit_TinyUSB.h>

void xinputAuthPrepare();
void xinputAuthReset();
const char *xinputAuthSerial();
bool xinputAuthControl(uint8_t rhport, uint8_t stage,
		       const tusb_control_request_t *request);

// Only the dedicated worker may call the singleton crypto library.
void xinputAuthProcess();
bool xinputAuthBusy();
void xinputAuthYield();
bool xinputAuthReceived(uint16_t length);
bool xinputAuthStartWorker();
void xinputAuthNotify();
void xinputAuthBoardSerial(char serial[13]);
uint32_t xinputAuthLock();
void xinputAuthUnlock(uint32_t mask);