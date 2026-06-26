// Adafruit_TinyUSB.h -- compatibility shim mapping the Adafruit TinyUSB API the
// firmware was written against onto Zephyr's USB device stack (USBD "next").
//
// The Arduino tree builds its USB presentation imperatively: construct
// Adafruit_USBD_HID/WebUSB/XInput objects, set descriptors + callbacks, then
// USBDevice.attach(). Zephyr's USBD registers class instances (HID devices are
// devicetree nodes) and brings the whole stack up at once. This shim bridges the
// two: each HID object binds to a pre-declared Zephyr HID device node on begin()
// in registration order, sendReport()->hid_device_submit_report(), and the
// TinyUSB get/set report callbacks are dispatched from Zephyr's hid_device_ops.
//
// One behavioral concession (documented in ARCHITECTURE.zephyr.md): Zephyr's
// descriptor set is fixed once the stack is enabled, so the Arduino dynamic
// "re-enumerate without reboot" path collapses to reboot-to-reenumerate, which
// was already the firmware's mode-switch path.
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---- HID report types (TinyUSB values; identical to Zephyr's). Guarded so this
//      never collides with <zephyr/usb/class/usbd_hid.h> when both are pulled
//      into the shim implementation TU. ----
typedef uint8_t hid_report_type_t;
#ifndef ZEPHYR_INCLUDE_USBD_HID_CLASS_DEVICE_H_
enum {
	HID_REPORT_TYPE_INVALID = 0,
	HID_REPORT_TYPE_INPUT,
	HID_REPORT_TYPE_OUTPUT,
	HID_REPORT_TYPE_FEATURE,
};
#endif

// ---- USB descriptor constants used when hand-building the XInput interface ----
enum {
	TUSB_DESC_INTERFACE = 0x04,
	TUSB_DESC_ENDPOINT = 0x05,
};
enum {
	TUSB_XFER_CONTROL = 0,
	TUSB_XFER_ISOCHRONOUS,
	TUSB_XFER_BULK,
	TUSB_XFER_INTERRUPT,
};
enum {
	TUSB_DIR_OUT = 0,
	TUSB_DIR_IN = 1,
};
#define HID_ITF_PROTOCOL_NONE 0
#define HID_ITF_PROTOCOL_KEYBOARD 1
#define HID_ITF_PROTOCOL_MOUSE 2

// Split a uint16 into two little-endian bytes (TinyUSB descriptor helper).
#define U16_TO_U8S_LE(x) (uint8_t)((x) & 0xFF), (uint8_t)(((x) >> 8) & 0xFF)

// Max HID interface instances (Arduino build set -DCFG_TUD_HID=4).
#ifndef CFG_TUD_HID
#define CFG_TUD_HID 4
#endif

// HID keyboard modifier bitmasks (TinyUSB values).
enum {
	KEYBOARD_MODIFIER_LEFTCTRL = 0x01,
	KEYBOARD_MODIFIER_LEFTSHIFT = 0x02,
	KEYBOARD_MODIFIER_LEFTALT = 0x04,
	KEYBOARD_MODIFIER_LEFTGUI = 0x08,
	KEYBOARD_MODIFIER_RIGHTCTRL = 0x10,
	KEYBOARD_MODIFIER_RIGHTSHIFT = 0x20,
	KEYBOARD_MODIFIER_RIGHTALT = 0x40,
	KEYBOARD_MODIFIER_RIGHTGUI = 0x80,
};

// Standard boot mouse report (TinyUSB layout): used by lizard + the wake jiggle.
typedef struct __attribute__((packed)) {
	uint8_t buttons;
	int8_t x;
	int8_t y;
	int8_t wheel;
	int8_t pan;
} hid_mouse_report_t;

// ---- HID keyboard usage IDs (lizard keyboard map) ----
enum {
	HID_KEY_NONE = 0x00,
	HID_KEY_A = 0x04,
	HID_KEY_O = 0x12,
	HID_KEY_ENTER = 0x28,
	HID_KEY_ESCAPE = 0x29,
	HID_KEY_DELETE = 0x4C,
	HID_KEY_TAB = 0x2B,
	HID_KEY_PAGE_UP = 0x4B,
	HID_KEY_PAGE_DOWN = 0x4E,
	HID_KEY_ARROW_RIGHT = 0x4F,
	HID_KEY_ARROW_LEFT = 0x50,
	HID_KEY_ARROW_DOWN = 0x51,
	HID_KEY_ARROW_UP = 0x52,
};

// ---- packed descriptor structs used by the XInput class ----
typedef struct __attribute__((packed)) {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
} tusb_desc_interface_t;

typedef struct __attribute__((packed)) {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
} tusb_desc_endpoint_t;

typedef struct __attribute__((packed)) {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} tusb_control_request_t;

// ---- HID report callback types (TinyUSB signatures) ----
typedef uint16_t (*hid_get_report_cb_t)(uint8_t report_id,
					hid_report_type_t type, uint8_t *buffer,
					uint16_t reqlen);
typedef void (*hid_set_report_cb_t)(uint8_t report_id, hid_report_type_t type,
				    uint8_t const *buffer, uint16_t bufsize);

// ---- base interface (XInput subclasses it to emit its own descriptor) ----
class Adafruit_USBD_Interface
{
    public:
	virtual ~Adafruit_USBD_Interface()
	{
	}
	virtual uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf,
						uint16_t bufsize) = 0;
	void setStringDescriptor(const char *s)
	{
		_desc_str = s;
	}

    protected:
	const char *_desc_str = nullptr;
	uint8_t _strid = 0; // string-descriptor index assigned by the stack
};

// ---- HID device ----
// Binds to one Zephyr "zephyr,hid-device" node (claimed in begin(), in order).
// Inherits Adafruit_USBD_Interface so it can be passed to USBDevice.addInterface
// (the Zephyr backing builds the real interface from the registered HID node).
class Adafruit_USBD_HID : public Adafruit_USBD_Interface
{
    public:
	Adafruit_USBD_HID();

	uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf,
					uint16_t bufsize) override
	{
		(void)itfnum;
		(void)buf;
		(void)bufsize;
		return 0; // Zephyr emits the HID interface descriptor itself
	}

	void setReportDescriptor(const uint8_t *desc, uint16_t len);
	void setReportCallback(hid_get_report_cb_t get_cb,
			       hid_set_report_cb_t set_cb);
	void setStringDescriptor(const char *s);
	void setPollInterval(uint8_t interval_ms);
	void setBootProtocol(uint8_t protocol);
	void enableOutEndpoint(bool en);
	bool begin();
	bool ready();
	bool sendReport(uint8_t report_id, const void *report, uint16_t len);

	// Convenience boot-mouse report (wake jiggle). buttons=0 moves without clicking.
	bool mouseReport(uint8_t report_id, uint8_t buttons, int8_t x, int8_t y,
			 int8_t wheel, int8_t pan)
	{
		hid_mouse_report_t m = { buttons, x, y, wheel, pan };
		return sendReport(report_id, &m, sizeof m);
	}

	// internal: invoked by the shim's Zephyr hid_device_ops dispatcher
	hid_get_report_cb_t _get_cb = nullptr;
	hid_set_report_cb_t _set_cb = nullptr;

    private:
	const uint8_t *_desc = nullptr;
	uint16_t _desc_len = 0;
	const char *_str = nullptr;
	uint8_t _poll_ms = 1;
	uint8_t _boot_proto = 0;
	bool _has_out = false;
	int _node = -1; // index into the Zephyr HID device pool, -1 = unbound
};

// ---- WebUSB vendor interface ----
class Adafruit_USBD_WebUSB : public Adafruit_USBD_Interface
{
    public:
	Adafruit_USBD_WebUSB();
	uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf,
					uint16_t bufsize) override
	{
		(void)itfnum;
		(void)buf;
		(void)bufsize;
		return 0;
	}
	bool begin();
	bool connected();
	int available();
	int read();
	uint32_t write(const void *buffer, uint32_t bufsize);
	void flush();
	void setLandingPage(const char *url)
	{
		_url = url;
	}

    private:
	const char *_url = nullptr;
};

// Standard TinyUSB boot-mouse report descriptor (no report ID): 5 buttons +
// 3-bit pad, then X/Y/wheel/AC-pan as 8-bit relative — matching hid_mouse_report_t.
#define TUD_HID_REPORT_DESC_MOUSE(...)                                        \
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00, 0x05,    \
		0x09, 0x19, 0x01, 0x29, 0x05, 0x15, 0x00, 0x25, 0x01, 0x95, \
		0x05, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x03, 0x81, \
		0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38, 0x15, \
		0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06, 0x05, \
		0x0C, 0x0A, 0x38, 0x02, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, \
		0x95, 0x01, 0x81, 0x06, 0xC0, 0xC0

// DFU entry (Adafruit nRF52 bootloader): stash the magic in GPREGRET and reset
// into UF2 mass-storage or serial DFU. Implemented in the USB backing.
void enterUf2Dfu(void);
void enterSerialDfu(void);

// ---- the device singleton ----
class Adafruit_USBD_Device
{
    public:
	void setID(uint16_t vid, uint16_t pid);
	void setVersion(uint16_t bcd);
	void setDeviceVersion(uint16_t bcd);
	void setManufacturerDescriptor(const char *s);
	void setProductDescriptor(const char *s);
	void setSerialDescriptor(const char *s);

	void setConfigurationBuffer(uint8_t *buf, uint32_t buflen);
	void setConfigurationAttribute(uint8_t attr);
	bool addInterface(Adafruit_USBD_Interface &itf);
	uint8_t allocInterface(uint8_t count = 1);
	uint8_t allocEndpoint(uint8_t dir);

	void clearConfiguration();
	bool detach();
	bool attach();
	bool mounted();
	bool suspended();
	bool remoteWakeup();
};
// Adafruit exposes the device object under both names; some call sites use
// TinyUSBDevice (e.g. wake_hid), others USBDevice. Both refer to one instance.
extern Adafruit_USBD_Device USBDevice;
extern Adafruit_USBD_Device &TinyUSBDevice;
