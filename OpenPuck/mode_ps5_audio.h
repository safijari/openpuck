// mode_ps5_audio.h -- USB Audio Class 1 (UAC1) interface for DualSense audio-haptics.
#pragma once
#include <Adafruit_TinyUSB.h>
#include <stdint.h>

extern "C" {
#include "device/usbd_pvt.h"
}

// UAC1 Audio Control interface for DualSense PS5 personality.
class Adafruit_USBD_Audio_UAC1 : public Adafruit_USBD_Interface {
    public:
	Adafruit_USBD_Audio_UAC1();

	uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf,
					uint16_t bufsize) override;

	bool begin();
};

// UAC1 Audio Streaming interface helper to register the second interface with USBDevice.
class Adafruit_USBD_Audio_UAC1_AS : public Adafruit_USBD_Interface {
    public:
	Adafruit_USBD_Audio_UAC1_AS();

	uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf,
					uint16_t bufsize) override;

	bool begin();
};

extern Adafruit_USBD_Audio_UAC1 g_ps5Audio;
extern Adafruit_USBD_Audio_UAC1_AS g_ps5AudioAs;

// Background task to rate-limit audio haptics to the RF relay queue.
void ps5AudioTask(void);

// Application class driver registration for TinyUSB.
const usbd_class_driver_t *uac1_get_driver(void);
