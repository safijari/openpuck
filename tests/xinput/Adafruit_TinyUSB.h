#pragma once
#include <stddef.h>
#include <stdint.h>

#define CFG_TUD_MEM_ALIGN __attribute__((aligned(4)))
#define CFG_TUSB_DEBUG 0

enum xfer_result_t {
	XFER_RESULT_SUCCESS,
	XFER_RESULT_FAILED,
	XFER_RESULT_STALLED,
	XFER_RESULT_TIMEOUT,
	XFER_RESULT_INVALID,
};

enum {
	TUSB_DESC_STRING = 3,
	TUSB_DESC_ENDPOINT = 5,
	TUSB_DIR_OUT = 0,
	TUSB_DIR_IN = 1,
};

struct __attribute__((packed)) tusb_desc_interface_t {
	uint8_t bLength, bDescriptorType, bInterfaceNumber, bAlternateSetting;
	uint8_t bNumEndpoints, bInterfaceClass, bInterfaceSubClass;
	uint8_t bInterfaceProtocol, iInterface;
};

struct __attribute__((packed)) tusb_desc_endpoint_t {
	uint8_t bLength, bDescriptorType, bEndpointAddress, bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};

struct __attribute__((packed)) tusb_desc_device_t {
	uint8_t bLength, bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass, bDeviceSubClass, bDeviceProtocol, bMaxPacketSize0;
	uint16_t idVendor, idProduct, bcdDevice;
	uint8_t iManufacturer, iProduct, iSerialNumber, bNumConfigurations;
};

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

extern "C" {
bool tud_control_xfer(uint8_t rhport, const tusb_control_request_t *request,
		      void *buffer, uint16_t length);

bool tud_mounted();
const uint8_t *tud_descriptor_device_cb();
}

class Adafruit_USBD_Interface {
    protected:
	uint8_t _strid = 0;

    public:
	virtual ~Adafruit_USBD_Interface() = default;
	virtual uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf,
						uint16_t bufsize) = 0;
	void setStringDescriptor(const char *str);
};

class Adafruit_USBD_Device {
    public:
	uint8_t allocInterface(uint8_t count);
	uint8_t allocEndpoint(uint8_t direction);
	bool addInterface(Adafruit_USBD_Interface &interface);
	void setID(uint16_t vendor, uint16_t product);
	void setVersion(uint16_t version);
	void setDeviceVersion(uint16_t version);
	void setManufacturerDescriptor(const char *str);
	void setProductDescriptor(const char *str);
	void setSerialDescriptor(const char *str);
	void setConfigurationAttribute(uint8_t attributes);
	void setConfigurationMaxPower(uint16_t milliamps);
};

extern Adafruit_USBD_Device TinyUSBDevice;
#define USBDevice TinyUSBDevice