#pragma once
#include <Adafruit_TinyUSB.h>

struct usbd_class_driver_t {
	void (*init)();
	bool (*deinit)();
	void (*reset)(uint8_t rhport);
	uint16_t (*open)(uint8_t rhport, const tusb_desc_interface_t *interface,
			 uint16_t max_len);
	bool (*control_xfer_cb)(uint8_t rhport, uint8_t stage,
				const tusb_control_request_t *request);
	bool (*xfer_cb)(uint8_t rhport, uint8_t ep, xfer_result_t result,
			uint32_t length);
	void (*sof)(uint8_t rhport, uint32_t frame_count);
};

bool usbd_edpt_open(uint8_t rhport, const tusb_desc_endpoint_t *descriptor);
void usbd_edpt_close(uint8_t rhport, uint8_t ep);
bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep, uint8_t *buffer,
		    uint16_t length);
bool usbd_edpt_busy(uint8_t rhport, uint8_t ep);
bool usbd_edpt_claim(uint8_t rhport, uint8_t ep);
bool usbd_edpt_release(uint8_t rhport, uint8_t ep);