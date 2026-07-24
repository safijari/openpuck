// usb_descriptors.c — TinyUSB descriptor callbacks for the puck identity.
//
// Presents VID/PID 28DE:1304 with four HID slot interfaces (each the cloned
// puck report descriptor) plus a WebUSB vendor interface (WinUSB-bound via an
// MS OS 2.0 descriptor so Chrome can open it without a manual driver). bcdDevice
// is 0x03xx to distinguish PicoPuck from the nRF puck in the panel and in
// Windows' per-(VID,PID,bcdDevice) descriptor cache.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include <string.h>
#include "tusb.h"
#include "usb/usb_descriptors.h"
#include "usb/puck_desc.h"
#include "puck/identity.h"

// Set by the panel's class request (0x22) on the vendor interface.
void webusb_set_connected(bool connected);

// ---- device descriptor -----------------------------------------------------
#if CFG_TUD_CDC
#define PP_BCD PP_BCD_DEVICE_CDC
#else
#define PP_BCD PP_BCD_DEVICE
#endif

static const tusb_desc_device_t desc_device = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0210,  // 2.1 → device advertises a BOS descriptor
#if CFG_TUD_CDC
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
#else
	.bDeviceClass = 0x00,
	.bDeviceSubClass = 0x00,
	.bDeviceProtocol = 0x00,
#endif
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = PP_USB_VID,
	.idProduct = PP_USB_PID,
	.bcdDevice = PP_BCD,
	.iManufacturer = 0x01,
	.iProduct = 0x02,
	.iSerialNumber = 0x03,
	.bNumConfigurations = 0x01,
};

const uint8_t *tud_descriptor_device_cb(void)
{
	return (const uint8_t *)&desc_device;
}

// ---- HID report descriptor (all four instances share the puck descriptor) --
const uint8_t *tud_hid_descriptor_report_cb(uint8_t instance)
{
	(void)instance;
	return PUCK_HID_DESC;
}

// ---- configuration descriptor ----------------------------------------------
#define CONFIG_TOTAL_LEN                                                       \
	(TUD_CONFIG_DESC_LEN + 4 * TUD_HID_DESC_LEN + TUD_VENDOR_DESC_LEN +     \
	 CFG_TUD_CDC * TUD_CDC_DESC_LEN)

static const uint8_t desc_configuration[] = {
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 250),

	// Four puck slot interfaces: IN-only, 1 ms poll, sharing PUCK_HID_DESC.
	TUD_HID_DESCRIPTOR(ITF_NUM_HID0, 0, HID_ITF_PROTOCOL_NONE,
			   PUCK_HID_DESC_SIZE, EPNUM_HID0, CFG_TUD_HID_EP_BUFSIZE, 1),
	TUD_HID_DESCRIPTOR(ITF_NUM_HID1, 0, HID_ITF_PROTOCOL_NONE,
			   PUCK_HID_DESC_SIZE, EPNUM_HID1, CFG_TUD_HID_EP_BUFSIZE, 1),
	TUD_HID_DESCRIPTOR(ITF_NUM_HID2, 0, HID_ITF_PROTOCOL_NONE,
			   PUCK_HID_DESC_SIZE, EPNUM_HID2, CFG_TUD_HID_EP_BUFSIZE, 1),
	TUD_HID_DESCRIPTOR(ITF_NUM_HID3, 0, HID_ITF_PROTOCOL_NONE,
			   PUCK_HID_DESC_SIZE, EPNUM_HID3, CFG_TUD_HID_EP_BUFSIZE, 1),

	// WebUSB config channel (bulk IN/OUT). String index 4 names it.
	TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 4, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64),

#if CFG_TUD_CDC
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 5, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT,
			   EPNUM_CDC_IN, 64),
#endif
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
	(void)index;
	return desc_configuration;
}

// ---- BOS + MS OS 2.0 (WinUSB binding for the vendor interface) -------------
#define MS_OS_20_DESC_LEN 0xB2

#define BOS_TOTAL_LEN                                                          \
	(TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN +                          \
	 TUD_BOS_MICROSOFT_OS_DESC_LEN)

static const uint8_t desc_bos[] = {
	TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),
	// WebUSB: vendor request code, landing-page index 0 = no landing page
	// (the panel opens the device directly; avoids a Chrome popup).
	TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 0),
	TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

const uint8_t *tud_descriptor_bos_cb(void)
{
	return desc_bos;
}

static const uint8_t desc_ms_os_20[] = {
	// Set header
	U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
	U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
	// Configuration subset header
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
	0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),
	// Function subset header (binds the vendor interface)
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
	ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),
	// Compatible ID: WINUSB
	U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
	'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	// Registry property: DeviceInterfaceGUIDs
	U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14),
	U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
	U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
	'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00,
	'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00,
	'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
	U16_TO_U8S_LE(0x0050),
	'{', 0x00, '9', 0x00, '7', 0x00, '5', 0x00, 'F', 0x00, '4', 0x00, '4', 0x00,
	'D', 0x00, '9', 0x00, '-', 0x00, '0', 0x00, 'D', 0x00, '0', 0x00, '8', 0x00,
	'-', 0x00, '4', 0x00, '3', 0x00, 'F', 0x00, 'D', 0x00, '-', 0x00, '8', 0x00,
	'B', 0x00, '3', 0x00, 'E', 0x00, '-', 0x00, '1', 0x00, '2', 0x00, '7', 0x00,
	'C', 0x00, 'A', 0x00, '8', 0x00, 'A', 0x00, 'F', 0x00, 'F', 0x00, 'F', 0x00,
	'9', 0x00, 'D', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00,
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "MS OS 2.0 size");

// Single vendor control-transfer callback: WebUSB/MS-OS descriptor requests and
// the panel's class "connect" request (0x22, Adafruit convention).
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
				tusb_control_request_t const *request)
{
	if (stage != CONTROL_STAGE_SETUP)
		return true;

	switch (request->bmRequestType_bit.type) {
	case TUSB_REQ_TYPE_VENDOR:
		switch (request->bRequest) {
		case VENDOR_REQUEST_MICROSOFT:
			if (request->wIndex == 7) {
				uint16_t total_len;
				memcpy(&total_len, desc_ms_os_20 + 8, 2);
				return tud_control_xfer(
					rhport, request,
					(void *)(uintptr_t)desc_ms_os_20,
					total_len);
			}
			return false;
		default:
			break;
		}
		break;

	case TUSB_REQ_TYPE_CLASS:
		if (request->bRequest == 0x22) {
			webusb_set_connected(request->wValue != 0);
			return tud_control_status(rhport, request);
		}
		break;

	default:
		break;
	}
	return false;
}

// ---- string descriptors ----------------------------------------------------
enum { STRID_LANGID = 0, STRID_MANUFACTURER, STRID_PRODUCT, STRID_SERIAL, STRID_VENDOR };

static const char *const string_desc_arr[] = {
	[STRID_LANGID] = (const char[]){ 0x09, 0x04 },  // English (0x0409)
	[STRID_MANUFACTURER] = "Valve Software",
	[STRID_PRODUCT] = "Steam Controller Puck",
	[STRID_SERIAL] = NULL,  // filled from g_usb_serial at runtime
	[STRID_VENDOR] = "PicoPuck WebUSB",
};

static uint16_t desc_str[32];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	(void)langid;
	size_t chr_count;

	if (index == STRID_LANGID) {
		memcpy(&desc_str[1], string_desc_arr[0], 2);
		chr_count = 1;
	} else {
		const char *str;
		if (index == STRID_SERIAL)
			str = g_usb_serial;
		else if (index < TU_ARRAY_SIZE(string_desc_arr))
			str = string_desc_arr[index];
		else
			return NULL;
		if (!str)
			return NULL;

		chr_count = strlen(str);
		if (chr_count > 31)
			chr_count = 31;
		for (size_t i = 0; i < chr_count; i++)
			desc_str[1 + i] = str[i];
	}

	desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
	return desc_str;
}
