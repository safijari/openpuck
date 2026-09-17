#pragma once
#include <stdint.h>

#define CFG_TUD_MEM_ALIGN __attribute__((aligned(4)))

struct tusb_control_request_t {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
};

enum {
	CONTROL_STAGE_SETUP,
	CONTROL_STAGE_DATA,
	CONTROL_STAGE_ACK,
};

bool tud_control_xfer(uint8_t rhport, const tusb_control_request_t *request,
		      void *buffer, uint16_t length);