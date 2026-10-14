#include "config.h"
#include <Adafruit_TinyUSB.h>

extern "C" const uint16_t *__real_tud_descriptor_string_cb(uint8_t index,
							   uint16_t langid);

extern "C" const uint16_t *__wrap_tud_descriptor_string_cb(uint8_t index,
							   uint16_t langid)
{
	// Adafruit truncates interface strings at 32 characters. The XSM3
	// descriptor is 88 UTF-16 code units and must survive a multi-packet GET.
	static const char security[] =
		"Xbox Security Method 3, Version 1.00, \xa9 2005 Microsoft "
		"Corporation. All rights reserved.";
	static uint16_t descriptor[sizeof security / sizeof security[0]];
	if (g_usbMode != MODE_XBOX || index != 4)
		return __real_tud_descriptor_string_cb(index, langid);
	constexpr size_t count = sizeof security / sizeof security[0] - 1;
	descriptor[0] = (TUSB_DESC_STRING << 8) | sizeof descriptor;
	for (size_t i = 0; i < count; i++)
		descriptor[i + 1] = (uint8_t)security[i];
	return descriptor;
}