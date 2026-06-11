#include "status_led.h"
#include <Arduino.h>

#define WAKE_LED_OFF ((WAKE_LED_ON)==HIGH ? LOW : HIGH)
#define PULSE_MS 500u   // wake flash duration

static unsigned long g_pulseMs = 0;
static bool g_lit = false;

static void ledWrite(int level){
  digitalWrite(WAKE_LED_PIN_A, level);
  digitalWrite(WAKE_LED_PIN_B, level);
}

void ledInit(){
  pinMode(WAKE_LED_PIN_A, OUTPUT);
  pinMode(WAKE_LED_PIN_B, OUTPUT);
  ledWrite(WAKE_LED_OFF);
}

void ledWakePulse(){
  g_pulseMs = millis(); g_lit = true;
  ledWrite(WAKE_LED_ON);   // light immediately at the remoteWakeup() call site, not on the next loop
}

void ledTask(){
  if (g_lit && millis()-g_pulseMs >= PULSE_MS){ g_lit = false; ledWrite(WAKE_LED_OFF); }
}
