// usb_xinput_class.cpp -- Zephyr custom class for the Xbox 360 (XInput) interface.
//
// XInput is a vendor interface (class 0xFF / sub 0x5D / proto 0x01) carrying a
// magic 0x21 descriptor blob plus an interrupt IN/OUT endpoint pair, serving the
// 20-byte XInput report. mode_xinput.cpp builds the report and rumble decode;
// this class is the Zephyr USB plumbing (the TinyUSB class driver in that file
// is unused under Zephyr).
//
// Single interface (one controller). Multi-controller XInput (one interface per
// connected pad) is a follow-up — see ARCHITECTURE.zephyr.md.
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(opk_xinput, LOG_LEVEL_ERR);

#define XI_IN_EP 0x83
#define XI_OUT_EP 0x03
#define XI_ENABLED 0

NET_BUF_POOL_FIXED_DEFINE(xi_pool, 4, 32, sizeof(struct udc_buf_info), NULL);
static atomic_t xi_state;
static struct usbd_class_data *xi_cd;

// rumble decode lives in mode_xinput; it provides this (weak no-op fallback).
extern "C" __attribute__((weak)) void opk_xinput_rumble(uint8_t big, uint8_t sml)
{
	(void)big;
	(void)sml;
}

// The XInput descriptor: 9-byte interface (FF/5D/01) + 17-byte magic 0x21 blob
// (embeds the endpoint addresses) + interrupt IN + interrupt OUT. Endpoint
// addresses are fixed so the magic blob stays consistent with the descriptors.
struct xi_desc {
	struct usb_if_descriptor if0;
	uint8_t magic[17];
	struct usb_ep_descriptor in_ep;
	struct usb_ep_descriptor out_ep;
	struct usb_desc_header nil;
} __packed;

static struct xi_desc xi_desc_0 = {
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = 0xFF,
		.bInterfaceSubClass = 0x5D,
		.bInterfaceProtocol = 0x01,
		.iInterface = 0,
	},
	.magic = { 0x11, 0x21, 0x00, 0x01, 0x01, 0x25, XI_IN_EP, 0x14, 0x00,
		   0x00, 0x00, 0x00, 0x13, XI_OUT_EP, 0x08, 0x00, 0x00 },
	.in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = XI_IN_EP,
		.bmAttributes = USB_EP_TYPE_INTERRUPT,
		.wMaxPacketSize = sys_cpu_to_le16(0x20),
		.bInterval = 1,
	},
	.out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = XI_OUT_EP,
		.bmAttributes = USB_EP_TYPE_INTERRUPT,
		.wMaxPacketSize = sys_cpu_to_le16(0x20),
		.bInterval = 8,
	},
	.nil = { .bLength = 0, .bDescriptorType = 0 },
};

const static struct usb_desc_header *xi_fs_desc_0[] = {
	(struct usb_desc_header *)&xi_desc_0.if0,
	(struct usb_desc_header *)&xi_desc_0.magic,
	(struct usb_desc_header *)&xi_desc_0.in_ep,
	(struct usb_desc_header *)&xi_desc_0.out_ep,
	(struct usb_desc_header *)&xi_desc_0.nil,
};

static struct net_buf *xi_alloc(uint8_t ep)
{
	struct net_buf *buf = net_buf_alloc(&xi_pool, K_NO_WAIT);
	if (!buf)
		return NULL;
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	memset(bi, 0, sizeof(*bi));
	bi->ep = ep;
	return buf;
}

static void xi_queue_out(struct usbd_class_data *c)
{
	struct net_buf *buf = xi_alloc(XI_OUT_EP);
	if (buf && usbd_ep_enqueue(c, buf))
		net_buf_unref(buf);
}

static int xi_request(struct usbd_class_data *c, struct net_buf *buf, int err)
{
	struct usbd_context *ctx = usbd_class_get_ctx(c);
	struct udc_buf_info *bi = (struct udc_buf_info *)net_buf_user_data(buf);

	if (err == 0 && bi->ep == XI_OUT_EP) {
		// XInput rumble packet: [00][08][00][big][small]...
		if (buf->len >= 5 && buf->data[0] == 0x00 && buf->data[1] == 0x08)
			opk_xinput_rumble(buf->data[3], buf->data[4]);
		usbd_ep_buf_free(ctx, buf);
		if (atomic_test_bit(&xi_state, XI_ENABLED))
			xi_queue_out(c);
		return 0;
	}
	usbd_ep_buf_free(ctx, buf);
	return 0;
}

static void *xi_get_desc(struct usbd_class_data *const c,
			 const enum usbd_speed speed)
{
	(void)c;
	(void)speed;
	return xi_fs_desc_0;
}
static void xi_enable(struct usbd_class_data *const c)
{
	xi_cd = c;
	if (!atomic_test_and_set_bit(&xi_state, XI_ENABLED))
		xi_queue_out(c);
}
static void xi_disable(struct usbd_class_data *const c)
{
	(void)c;
	atomic_clear_bit(&xi_state, XI_ENABLED);
}
static int xi_init(struct usbd_class_data *c)
{
	xi_cd = c;
	return 0;
}

static struct usbd_class_api xi_api = {
	.request = xi_request,
	.enable = xi_enable,
	.disable = xi_disable,
	.init = xi_init,
	.get_desc = xi_get_desc,
};

USBD_DEFINE_CLASS(opk_xinput_0, &xi_api, NULL, NULL);

// ---- bridge for mode_xinput ----
// Whether the XInput interface should be registered this boot (Xbox mode only).
static bool s_xi_want;
extern "C" void opk_xinput_begin(void)
{
	s_xi_want = true;
}
extern "C" bool opk_xinput_want(void)
{
	return s_xi_want;
}
extern "C" bool opk_xinput_ready(void)
{
	return atomic_test_bit(&xi_state, XI_ENABLED);
}
extern "C" bool opk_xinput_send(const uint8_t *rep, uint16_t n)
{
	if (!xi_cd || !opk_xinput_ready())
		return false;
	struct net_buf *buf = xi_alloc(XI_IN_EP);
	if (!buf)
		return false;
	net_buf_add_mem(buf, rep, n > 32 ? 32 : n);
	if (usbd_ep_enqueue(xi_cd, buf)) {
		net_buf_unref(buf);
		return false;
	}
	return true;
}
