// usb_glue.cpp -- Zephyr USBD backing for the Adafruit_TinyUSB shim.
//
// Brings up a single usbd_context, exposes device state (mounted/suspended) to
// the shim, and binds Adafruit_USBD_HID objects to a pool of devicetree
// "zephyr,hid-device" nodes. Report callbacks from Zephyr's hid_device_ops are
// dispatched to the per-object TinyUSB-style get/set callbacks.
//
// NOTE: this is the bring-up backing. The pieces that map cleanly to Zephyr
// (device state, HID submit/callbacks) are functional; the WebUSB vendor pipe
// and the XInput custom class are scaffolded to compile and are completed in
// their own glue units (see usb_webusb_glue / mode_xinput port).
// Zephyr USB headers FIRST so their HID enums/keys are defined; the shim header
// then suppresses its duplicate definitions via the matching include guards.
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#include "Adafruit_TinyUSB.h"

// ---- device state, updated from the USBD message callback ----
static atomic_t s_mounted = ATOMIC_INIT(0);
static atomic_t s_suspended = ATOMIC_INIT(0);

extern "C" bool opk_usb_mounted(void)
{
	return atomic_get(&s_mounted) != 0;
}
extern "C" bool opk_usb_suspended(void)
{
	return atomic_get(&s_suspended) != 0;
}

// Set by the USBD init code (usb_device_setup.cpp) so this TU can drive resume.
// Weak fallbacks let this glue link before that unit exists (bring-up); the real
// usbd setup unit strongly overrides them.
extern "C" __attribute__((weak)) struct usbd_context *opk_usbd_ctx(void)
{
	return nullptr;
}
extern "C" __attribute__((weak)) int opk_usbd_enable(void)
{
	return 0;
}
extern "C" __attribute__((weak)) int opk_usbd_disable(void)
{
	return 0;
}

// ---- HID device pool, from devicetree ----
// Each "zephyr,hid-device" node is one HID interface. Objects claim them in
// begin() order, mirroring TinyUSB's instance-index assignment.
#define HID_NODE(inst) DEVICE_DT_GET(DT_NODELABEL(hid_dev_##inst)),
static const struct device *const s_hid_pool[] = {
#if DT_NODE_EXISTS(DT_NODELABEL(hid_dev_0))
	DEVICE_DT_GET(DT_NODELABEL(hid_dev_0)),
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(hid_dev_1))
	DEVICE_DT_GET(DT_NODELABEL(hid_dev_1)),
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(hid_dev_2))
	DEVICE_DT_GET(DT_NODELABEL(hid_dev_2)),
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(hid_dev_3))
	DEVICE_DT_GET(DT_NODELABEL(hid_dev_3)),
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(hid_dev_4))
	DEVICE_DT_GET(DT_NODELABEL(hid_dev_4)),
#endif
};
#define HID_POOL_N ((int)(sizeof(s_hid_pool) / sizeof(s_hid_pool[0])))

// Reverse map: Zephyr device -> owning HID object, for the ops dispatcher.
static Adafruit_USBD_HID *s_hid_owner[HID_POOL_N];
static int s_hid_claimed;
static atomic_t s_hid_ready[HID_POOL_N];

static int pool_index(const struct device *dev)
{
	for (int i = 0; i < HID_POOL_N; i++)
		if (s_hid_pool[i] == dev)
			return i;
	return -1;
}

// ---- Zephyr hid_device_ops -> TinyUSB-style callback dispatch ----
static void op_iface_ready(const struct device *dev, const bool ready)
{
	int i = pool_index(dev);
	if (i >= 0)
		atomic_set(&s_hid_ready[i], ready ? 1 : 0);
}

static int op_get_report(const struct device *dev, const uint8_t type,
			 const uint8_t id, const uint16_t len, uint8_t *const buf)
{
	int i = pool_index(dev);
	if (i < 0 || !s_hid_owner[i] || !s_hid_owner[i]->_get_cb)
		return -ENOTSUP;
	uint16_t n = s_hid_owner[i]->_get_cb(id, type, buf, len);
	return n ? (int)n : -ENOTSUP;
}

static int op_set_report(const struct device *dev, const uint8_t type,
			 const uint8_t id, const uint16_t len,
			 const uint8_t *const buf)
{
	int i = pool_index(dev);
	if (i < 0 || !s_hid_owner[i] || !s_hid_owner[i]->_set_cb)
		return -ENOTSUP;
	s_hid_owner[i]->_set_cb(id, type, buf, len);
	return 0;
}

// Output reports delivered via the interrupt OUT pipe: the firmware's set
// callbacks treat OUTPUT and the interrupt-OUT path identically, so forward to
// the same callback with the OUTPUT type. The first byte is the report ID when
// the descriptor uses IDs.
static void op_output_report(const struct device *dev, const uint16_t len,
			     const uint8_t *const buf)
{
	int i = pool_index(dev);
	if (i < 0 || !s_hid_owner[i] || !s_hid_owner[i]->_set_cb || len == 0)
		return;
	s_hid_owner[i]->_set_cb(buf[0], HID_REPORT_TYPE_OUTPUT, buf + 1,
				len - 1);
}

static const struct hid_device_ops s_hid_ops = {
	.iface_ready = op_iface_ready,
	.get_report = op_get_report,
	.set_report = op_set_report,
	.output_report = op_output_report,
};

// ===================== Adafruit_USBD_HID =====================
Adafruit_USBD_HID::Adafruit_USBD_HID()
{
}

void Adafruit_USBD_HID::setReportDescriptor(const uint8_t *desc, uint16_t len)
{
	_desc = desc;
	_desc_len = len;
}
void Adafruit_USBD_HID::setReportCallback(hid_get_report_cb_t get_cb,
					  hid_set_report_cb_t set_cb)
{
	_get_cb = get_cb;
	_set_cb = set_cb;
}
void Adafruit_USBD_HID::setStringDescriptor(const char *s)
{
	_str = s;
}
void Adafruit_USBD_HID::setPollInterval(uint8_t interval_ms)
{
	_poll_ms = interval_ms;
}
void Adafruit_USBD_HID::setBootProtocol(uint8_t protocol)
{
	_boot_proto = protocol;
}
void Adafruit_USBD_HID::enableOutEndpoint(bool en)
{
	_has_out = en;
}

bool Adafruit_USBD_HID::begin()
{
	if (s_hid_claimed >= HID_POOL_N)
		return false;
	_node = s_hid_claimed++;
	s_hid_owner[_node] = this;
	const struct device *dev = s_hid_pool[_node];
	if (!device_is_ready(dev))
		return false;
	return hid_device_register(dev, _desc, _desc_len, &s_hid_ops) == 0;
}

bool Adafruit_USBD_HID::ready()
{
	if (_node < 0)
		return false;
	return atomic_get(&s_hid_ready[_node]) != 0;
}

bool Adafruit_USBD_HID::sendReport(uint8_t report_id, const void *report,
				   uint16_t len)
{
	if (_node < 0 || !ready())
		return false;
	const struct device *dev = s_hid_pool[_node];
	// Zephyr expects the report buffer; when the descriptor uses report IDs
	// the ID must lead the buffer. The firmware passes the body separately,
	// so prepend the ID here when nonzero.
	if (report_id) {
		uint8_t buf[80];
		if ((size_t)len + 1 > sizeof buf)
			return false;
		buf[0] = report_id;
		memcpy(buf + 1, report, len);
		return hid_device_submit_report(dev, len + 1, buf) == 0;
	}
	return hid_device_submit_report(dev, len,
					(const uint8_t *)report) == 0;
}

// ===================== TinyUSBDevice =====================
// Device identity / descriptor mutators feed the usbd descriptor built in
// usb_device_setup.cpp; here they are recorded into shared state it reads.
#include "usb_identity.h"
opk_usb_identity g_opk_usb_id;

void Adafruit_USBD_Device::setID(uint16_t vid, uint16_t pid)
{
	g_opk_usb_id.vid = vid;
	g_opk_usb_id.pid = pid;
}
void Adafruit_USBD_Device::setVersion(uint16_t bcd)
{
	g_opk_usb_id.bcd_usb = bcd;
}
void Adafruit_USBD_Device::setDeviceVersion(uint16_t bcd)
{
	g_opk_usb_id.bcd_device = bcd;
}
void Adafruit_USBD_Device::setManufacturerDescriptor(const char *s)
{
	g_opk_usb_id.manufacturer = s;
}
void Adafruit_USBD_Device::setProductDescriptor(const char *s)
{
	g_opk_usb_id.product = s;
}
void Adafruit_USBD_Device::setSerialDescriptor(const char *s)
{
	g_opk_usb_id.serial = s;
}
void Adafruit_USBD_Device::setConfigurationBuffer(uint8_t *buf, uint32_t buflen)
{
	(void)buf;
	(void)buflen; // Zephyr builds its own descriptor set
}
void Adafruit_USBD_Device::setConfigurationAttribute(uint8_t attr)
{
	g_opk_usb_id.cfg_attr = attr;
}

// Interface/endpoint allocation + dynamic add: the Zephyr descriptor set is
// fixed at init, so these are recorded for the XInput class glue and are no-ops
// against the live stack (see the reboot-to-reenumerate concession).
uint8_t Adafruit_USBD_Device::allocInterface(uint8_t count)
{
	uint8_t base = g_opk_usb_id.next_itf;
	g_opk_usb_id.next_itf += count;
	return base;
}
uint8_t Adafruit_USBD_Device::allocEndpoint(uint8_t dir)
{
	uint8_t n = ++g_opk_usb_id.next_ep;
	return (dir ? 0x80 : 0x00) | n;
}
bool Adafruit_USBD_Device::addInterface(Adafruit_USBD_Interface &itf)
{
	(void)itf;
	return true;
}

void Adafruit_USBD_Device::clearConfiguration()
{
	s_hid_claimed = 0;
	g_opk_usb_id.next_itf = 0;
	g_opk_usb_id.next_ep = 0;
}

// detach/attach map to disabling/enabling the usbd stack (opk_usbd_*, above).
bool Adafruit_USBD_Device::detach()
{
	return opk_usbd_disable() == 0;
}
bool Adafruit_USBD_Device::attach()
{
	return opk_usbd_enable() == 0;
}
bool Adafruit_USBD_Device::mounted()
{
	return opk_usb_mounted();
}
bool Adafruit_USBD_Device::suspended()
{
	return opk_usb_suspended();
}
bool Adafruit_USBD_Device::remoteWakeup()
{
	struct usbd_context *ctx = opk_usbd_ctx();
	if (!ctx)
		return false;
	return usbd_wakeup_request(ctx) == 0;
}

Adafruit_USBD_Device USBDevice;
Adafruit_USBD_Device &TinyUSBDevice = USBDevice;

// USBD message callback: track mount/suspend. Registered by usb_device_setup.
extern "C" void opk_usb_msg(const enum usbd_msg_type type)
{
	switch (type) {
	case USBD_MSG_CONFIGURATION:
		atomic_set(&s_mounted, 1);
		break;
	case USBD_MSG_RESET:
		atomic_set(&s_mounted, 0);
		break;
	case USBD_MSG_SUSPEND:
		atomic_set(&s_suspended, 1);
		break;
	case USBD_MSG_RESUME:
		atomic_set(&s_suspended, 0);
		break;
	default:
		break;
	}
}

// ===================== WebUSB (vendor) =====================
// Backed by the Zephyr custom vendor class in usb_webusb_class.cpp.
extern "C" bool opk_webusb_connected(void);
extern "C" int opk_webusb_available(void);
extern "C" int opk_webusb_read(void);
extern "C" uint32_t opk_webusb_write(const void *data, uint32_t n);
extern "C" void opk_webusb_flush(void);

static bool s_want_webusb;
extern "C" bool opk_want_webusb(void)
{
	return s_want_webusb;
}
extern "C" int opk_hid_claimed(void)
{
	return s_hid_claimed;
}

Adafruit_USBD_WebUSB::Adafruit_USBD_WebUSB()
{
}
bool Adafruit_USBD_WebUSB::begin()
{
	// The firmware calls this exactly for modes that expose the panel (i.e.
	// not the clean-PS modes); used to gate per-mode class registration.
	s_want_webusb = true;
	return true;
}
bool Adafruit_USBD_WebUSB::connected()
{
	return opk_webusb_connected();
}
int Adafruit_USBD_WebUSB::available()
{
	return opk_webusb_available();
}
int Adafruit_USBD_WebUSB::read()
{
	return opk_webusb_read();
}
uint32_t Adafruit_USBD_WebUSB::write(const void *data, uint32_t n)
{
	return opk_webusb_write(data, n);
}
void Adafruit_USBD_WebUSB::flush()
{
	opk_webusb_flush();
}

// ===================== DFU entry =====================
#include "Arduino.h"
void enterUf2Dfu(void)
{
	NRF_POWER->GPREGRET = 0x57; // Adafruit bootloader: UF2 mass-storage
	NVIC_SystemReset();
}
void enterSerialDfu(void)
{
	NRF_POWER->GPREGRET = 0x4E; // Adafruit bootloader: serial (OTA) DFU
	NVIC_SystemReset();
}

// ===================== XInput custom-class endpoint stubs =====================
// XInput needs a real Zephyr custom USBD class to enumerate (see mode_xinput).
// These keep the class driver's report logic compiling/inert until then.
#include "device/usbd_pvt.h"
extern "C" bool tud_mounted(void)
{
	return opk_usb_mounted();
}
bool usbd_edpt_open(uint8_t, const tusb_desc_endpoint_t *)
{
	return false;
}
bool usbd_edpt_xfer(uint8_t, uint8_t, uint8_t *, uint16_t)
{
	return false;
}
bool usbd_edpt_busy(uint8_t, uint8_t)
{
	return false;
}
bool usbd_edpt_claim(uint8_t, uint8_t)
{
	return false;
}
void usbd_edpt_release(uint8_t, uint8_t)
{
}
