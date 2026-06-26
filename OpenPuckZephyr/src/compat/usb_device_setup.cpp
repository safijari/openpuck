// usb_device_setup.cpp -- the Zephyr usbd_context for the OpenPuck port.
//
// Builds one full-speed device, registers all class instances picked up from
// devicetree (the HID pool in app.overlay + the board CDC ACM console), and
// brings the stack up. The TinyUSB shim's USBDevice.attach()/detach() call
// opk_usbd_enable()/disable(); the per-mode VID/PID/bcdDevice the firmware set
// through the shim (g_opk_usb_id) are applied here just before enable.
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/bos.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include "Adafruit_TinyUSB.h"
#include "usb_identity.h"

LOG_MODULE_REGISTER(opk_usbd, LOG_LEVEL_INF);

extern "C" void opk_usb_msg(const enum usbd_msg_type type);
// per-mode class-selection inputs (set by the shim / modes before attach)
extern "C" int opk_hid_claimed(void);
extern "C" bool opk_want_webusb(void);
extern "C" bool opk_xinput_want(void);

USBD_DEVICE_DEFINE(opk_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x28DE,
		   0x1304);

USBD_DESC_LANG_DEFINE(opk_lang);
USBD_DESC_MANUFACTURER_DEFINE(opk_mfr, "OpenPuck");
USBD_DESC_PRODUCT_DEFINE(opk_product, "OpenPuck");
USBD_DESC_SERIAL_NUMBER_DEFINE(opk_sn);

USBD_DESC_CONFIG_DEFINE(opk_fs_cfg_desc, "OpenPuck FS");

// ---- WebUSB platform capability (BOS) + landing-page URL ----
// Lets a browser discover the vendor interface (usb_webusb_class.cpp). The URL
// is served via the WebUSB vendor request.
#define OPK_WEBUSB_VENDOR_CODE 0x01U
#define OPK_WEBUSB_LANDING 0x01U
#define WEBUSB_REQ_GET_URL 0x02U
#define WEBUSB_DESC_TYPE_URL 0x03U

struct opk_bos_webusb {
	struct usb_bos_platform_descriptor platform;
	struct usb_bos_capability_webusb cap;
} __packed;

static const struct opk_bos_webusb opk_bos_webusb = {
	.platform = {
		.bLength = sizeof(struct usb_bos_platform_descriptor) +
			   sizeof(struct usb_bos_capability_webusb),
		.bDescriptorType = USB_DESC_DEVICE_CAPABILITY,
		.bDevCapabilityType = USB_BOS_CAPABILITY_PLATFORM,
		.bReserved = 0,
		.PlatformCapabilityUUID = {
			0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47,
			0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,
		},
	},
	.cap = {
		.bcdVersion = sys_cpu_to_le16(0x0100),
		.bVendorCode = OPK_WEBUSB_VENDOR_CODE,
		.iLandingPage = OPK_WEBUSB_LANDING,
	},
};

// Landing page (the browser config panel). UTF-8, scheme byte 0x01 = https.
static const uint8_t opk_webusb_url[] = {
	0x16, WEBUSB_DESC_TYPE_URL, 0x01,
	'o', 'p', 'e', 'n', 'p', 'u', 'c', 'k', '.', 'l', 'o', 'c', 'a', 'l',
	'/', 'p', 'a', 'n', 'e', 'l',
};

static int webusb_to_host(const struct usbd_context *const ctx,
			  const struct usb_setup_packet *const setup,
			  struct net_buf *const buf)
{
	(void)ctx;
	if (setup->wIndex == WEBUSB_REQ_GET_URL &&
	    USB_GET_DESCRIPTOR_INDEX(setup->wValue) == OPK_WEBUSB_LANDING) {
		net_buf_add_mem(buf, opk_webusb_url,
				MIN(net_buf_tailroom(buf),
				    sizeof(opk_webusb_url)));
		return 0;
	}
	return -ENOTSUP;
}

USBD_DESC_BOS_VREQ_DEFINE(opk_bos_vreq_webusb, sizeof(opk_bos_webusb),
			  &opk_bos_webusb, OPK_WEBUSB_VENDOR_CODE, webusb_to_host,
			  NULL);

// Bus-powered + remote-wakeup (the firmware always arms remote wakeup).
static const uint8_t opk_attr = USB_SCD_REMOTE_WAKEUP;
USBD_CONFIGURATION_DEFINE(opk_fs_config, opk_attr, 250, &opk_fs_cfg_desc);

static bool s_setup_done;
static bool s_enabled;

static void msg_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	(void)ctx;
	opk_usb_msg(msg->type);
}

static int do_setup(void)
{
	int err;

	if (s_setup_done)
		return 0;

	if ((err = usbd_add_descriptor(&opk_usbd, &opk_lang)) ||
	    (err = usbd_add_descriptor(&opk_usbd, &opk_mfr)) ||
	    (err = usbd_add_descriptor(&opk_usbd, &opk_product)) ||
	    (err = usbd_add_descriptor(&opk_usbd, &opk_sn))) {
		LOG_ERR("descriptor add failed (%d)", err);
		return err;
	}

	err = usbd_add_configuration(&opk_usbd, USBD_SPEED_FS, &opk_fs_config);
	if (err) {
		LOG_ERR("add configuration failed (%d)", err);
		return err;
	}

	// Register only the classes this mode actually used. Zephyr's descriptor
	// set is fixed once enabled, so per-mode interface selection happens here
	// (the reboot-to-reenumerate concession). The nRF52840 has 8 IN endpoints,
	// so registering every class at once would also overflow the budget.
	// Names are the USBD_DEFINE_CLASS instance names: HID pool = hid_0..hid_4,
	// the board CDC console = cdc_acm_0, plus our opk_webusb_0 / opk_xinput_0.
	static const char *const HIDN[] = { "hid_0", "hid_1", "hid_2", "hid_3",
					    "hid_4" };
	const char *block[8];
	int bi = 0;
	int hc = opk_hid_claimed();
	for (int i = hc; i < 5; i++)
		block[bi++] = HIDN[i]; // unused HID pool nodes
	if (!opk_want_webusb()) {
		block[bi++] = "opk_webusb_0";
		block[bi++] = "cdc_acm_0"; // CDC rides with the panel (clean-PS drops both)
	}
	if (!opk_xinput_want())
		block[bi++] = "opk_xinput_0";
	block[bi] = NULL;

	err = usbd_register_all_classes(&opk_usbd, USBD_SPEED_FS, 1, block);
	if (err) {
		LOG_ERR("register classes failed (%d)", err);
		return err;
	}

	usbd_device_set_code_triple(&opk_usbd, USBD_SPEED_FS, 0, 0, 0);

	// WebUSB BOS (browser discovers the vendor interface); best-effort.
	(void)usbd_device_set_bcd_usb(&opk_usbd, USBD_SPEED_FS, 0x0210);
	if (usbd_add_descriptor(&opk_usbd, &opk_bos_vreq_webusb))
		LOG_WRN("WebUSB BOS add failed");

	usbd_msg_register_cb(&opk_usbd, msg_cb);
	s_setup_done = true;
	return 0;
}

extern "C" struct usbd_context *opk_usbd_ctx(void)
{
	return &opk_usbd;
}

extern "C" int opk_usbd_enable(void)
{
	int err = do_setup();
	if (err)
		return err;

	// Apply the per-mode identity the firmware staged through the shim.
	usbd_device_set_vid(&opk_usbd, g_opk_usb_id.vid);
	usbd_device_set_pid(&opk_usbd, g_opk_usb_id.pid);
	usbd_device_set_bcd_device(&opk_usbd, g_opk_usb_id.bcd_device);

	if (s_enabled)
		return 0;
	err = usbd_init(&opk_usbd);
	if (err && err != -EALREADY) {
		LOG_ERR("usbd_init failed (%d)", err);
		return err;
	}
	err = usbd_enable(&opk_usbd);
	if (err && err != -EALREADY) {
		LOG_ERR("usbd_enable failed (%d)", err);
		return err;
	}
	s_enabled = true;
	return 0;
}

extern "C" int opk_usbd_disable(void)
{
	if (!s_enabled)
		return 0;
	int err = usbd_disable(&opk_usbd);
	s_enabled = false;
	return err;
}
