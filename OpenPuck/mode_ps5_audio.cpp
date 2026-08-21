#include "mode_ps5_audio.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_mount.h"
#include "fault_diag.h"
#include <Arduino.h>
#include <string.h>

// 4-channel layout: IAD(8)+AC_std(9)+CS_hdr(9)+input_term(12)+feat_unit(12)+out_term(9) = 59
#define UAC1_AC_DESC_LEN 59
#define UAC1_AS_DESC_LEN 52
// 4ch * 2 bytes * 48 frames/ms = 384 bytes per 1ms isochronous packet
#define UAC1_ISO_EP_BUFSIZE 384

static uint8_t g_uac1ItfAc = 0xFF;
static uint8_t g_uac1ItfAs = 0xFF;
static uint8_t g_uac1EpOut = 0x08;
static uint8_t g_uac1AltSetting = 0;

CFG_TUD_MEM_SECTION static uint32_t g_isoOutBuf32[UAC1_ISO_EP_BUFSIZE / 4];
static uint8_t *g_isoOutBuf = (uint8_t *)g_isoOutBuf32;

Adafruit_USBD_Audio_UAC1 g_ps5Audio;
Adafruit_USBD_Audio_UAC1_AS g_ps5AudioAs;

Adafruit_USBD_Audio_UAC1::Adafruit_USBD_Audio_UAC1()
{
}

uint16_t Adafruit_USBD_Audio_UAC1::getInterfaceDescriptor(uint8_t itfnum,
							  uint8_t *buf,
							  uint16_t bufsize)
{
	if (!buf)
		return UAC1_AC_DESC_LEN;
	if (bufsize < UAC1_AC_DESC_LEN)
		return 0;

	uint8_t ac_itf = TinyUSBDevice.allocInterface(1);
	uint8_t as_itf = (uint8_t)(ac_itf + 1);
	g_uac1ItfAc = ac_itf;

	const uint8_t desc[UAC1_AC_DESC_LEN] = {
		// Interface Association Descriptor (IAD) - 8 bytes
		8, TUSB_DESC_INTERFACE_ASSOCIATION, ac_itf, 2, TUSB_CLASS_AUDIO,
		0x00, 0x00, 0,

		// Audio Control (AC) Standard Interface Descriptor - 9 bytes
		9, TUSB_DESC_INTERFACE, ac_itf, 0, 0, TUSB_CLASS_AUDIO, 0x01,
		0x00, 0,

		// AC Class-Specific Header Descriptor - 9 bytes
		// wTotalLength = CS_hdr(9)+input_term(12)+feat_unit(12)+out_term(9) = 42
		9, 0x24, 0x01, 0x00, 0x01, 42, 0x00, 1, as_itf,

		// Input Terminal Descriptor (USB Streaming, 4ch) - 12 bytes
		// wChannelConfig 0x0033: FL + FR + BL(haptic-L) + BR(haptic-R)
		12, 0x24, 0x02, 0x01, 0x01, 0x01, 0x00, 4, 0x33, 0x00, 0x00, 0,

		// Feature Unit Descriptor (Mute / Volume, 4ch) - 12 bytes
		// bControlSize=1: master mute(0x01), ch1-4 volume(0x02 each)
		12, 0x24, 0x06, 0x02, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02,
		0,

		// Output Terminal Descriptor (Speaker) - 9 bytes
		9, 0x24, 0x03, 0x03, 0x01, 0x03, 0x00, 0x02, 0
	};

	memcpy(buf, desc, UAC1_AC_DESC_LEN);
	return UAC1_AC_DESC_LEN;
}

bool Adafruit_USBD_Audio_UAC1::begin()
{
	return TinyUSBDevice.addInterface(*this);
}

Adafruit_USBD_Audio_UAC1_AS::Adafruit_USBD_Audio_UAC1_AS()
{
}

uint16_t Adafruit_USBD_Audio_UAC1_AS::getInterfaceDescriptor(uint8_t itfnum,
							     uint8_t *buf,
							     uint16_t bufsize)
{
	if (!buf)
		return UAC1_AS_DESC_LEN;
	if (bufsize < UAC1_AS_DESC_LEN)
		return 0;

	uint8_t as_itf = TinyUSBDevice.allocInterface(1);
	g_uac1ItfAs = as_itf;

	const uint8_t desc[UAC1_AS_DESC_LEN] = {
		// Audio Streaming (AS) Standard Interface Descriptor (Alt 0) - 9 bytes
		9, TUSB_DESC_INTERFACE, as_itf, 0, 0, TUSB_CLASS_AUDIO, 0x02,
		0x00, 0,

		// Audio Streaming (AS) Standard Interface Descriptor (Alt 1) - 9 bytes
		9, TUSB_DESC_INTERFACE, as_itf, 1, 1, TUSB_CLASS_AUDIO, 0x02,
		0x00, 0,

		// AS Class-Specific General Descriptor - 7 bytes
		7, 0x24, 0x01, 0x01, 0x01, 0x01, 0x00,

		// AS Class-Specific Format Type I Descriptor (PCM 4ch 16-bit 48kHz) - 11 bytes
		11, 0x24, 0x02, 0x01, 4, 2, 16, 1, 0x80, 0xBB, 0x00,

		// Standard Isochronous Audio Data Endpoint Descriptor - 9 bytes (EP 0x08)
		// bmAttributes 0x05: isochronous (01b) + asynchronous sync (01b)
		9, TUSB_DESC_ENDPOINT, 0x08, 0x05,
		U16_TO_U8S_LE(UAC1_ISO_EP_BUFSIZE), 1, 0, 0,

		// Class-Specific Audio Data Endpoint Descriptor - 7 bytes
		7, 0x25, 0x01, 0x01, 0, 0, 0
	};

	memcpy(buf, desc, UAC1_AS_DESC_LEN);
	return UAC1_AS_DESC_LEN;
}

bool Adafruit_USBD_Audio_UAC1_AS::begin()
{
	return TinyUSBDevice.addInterface(*this);
}

static volatile uint16_t s_audioLeftAmp = 0;
static volatile uint16_t s_audioRightAmp = 0;
static volatile uint32_t s_audioLastPktMs = 0;

static void processAudioSamples(const uint8_t *data, uint32_t len)
{
	// 4-channel 16-bit PCM: 8 bytes per frame.
	// ch1(s[0]), ch2(s[1]) = speaker (discarded).
	// ch3(s[2]) = left LRA, ch4(s[3]) = right LRA.
	if (len < 8)
		return;

	uint32_t num_frames = len / 8;
	int32_t peak_l = 0, peak_r = 0;
	for (uint32_t i = 0; i < num_frames; i++) {
		const int16_t *s = (const int16_t *)(data + i * 8);
		int32_t l = s[2] < 0 ? -s[2] : s[2];
		int32_t r = s[3] < 0 ? -s[3] : s[3];
		if (l > peak_l)
			peak_l = l;
		if (r > peak_r)
			peak_r = r;
	}

	s_audioLeftAmp = (uint16_t)((peak_l * 65535u) / 32767u);
	s_audioRightAmp = (uint16_t)((peak_r * 65535u) / 32767u);
	s_audioLastPktMs = millis();
}

void ps5AudioTask(void)
{
	static uint16_t s_lastL = 0, s_lastR = 0;
	static uint32_t s_lastSendMs = 0;

	uint32_t now = millis();
	// Silence timeout: if no audio packet in 50ms, force amplitude to 0
	uint16_t l = (uint32_t)(now - s_audioLastPktMs) > 50u ? 0 :
								s_audioLeftAmp;
	uint16_t r = (uint32_t)(now - s_audioLastPktMs) > 50u ? 0 :
								s_audioRightAmp;

	if ((l != s_lastL || r != s_lastR) &&
	    ((uint32_t)(now - s_lastSendMs) >= 8u ||
	     (l == 0 && r == 0 && (s_lastL > 0 || s_lastR > 0)))) {
		s_lastL = l;
		s_lastR = r;
		s_lastSendMs = now;
		for (uint8_t u = 0; u < g_usbMountCount; u++) {
			int bond = g_usbToBond[u];
			if (bond >= 0)
				hapticSteamRumble(l, r, (uint8_t)bond);
		}
	}
}

// TinyUSB class driver implementation for UAC1

// Endpoint descriptor for the ISO OUT endpoint on Alt 1.
// Must match the AS descriptor emitted by getInterfaceDescriptor.
static const tusb_desc_endpoint_t s_iso_ep_out = {
	.bLength = sizeof(tusb_desc_endpoint_t),
	.bDescriptorType = TUSB_DESC_ENDPOINT,
	.bEndpointAddress = 0x08,
	// bmAttributes 0x05: isochronous (01b) + asynchronous sync (01b)
	.bmAttributes = { .xfer = TUSB_XFER_ISOCHRONOUS, .sync = 1, .usage = 0 },
	.wMaxPacketSize = UAC1_ISO_EP_BUFSIZE,
	.bInterval = 1,
};

static void uac1_init(void)
{
	// USBD_ISOSPLIT_SPLIT_OneDir (0x0000) is the hardware reset default;
	// writing it here races with the USBD power-on sequence and causes the
	// host's first GET_DESCRIPTOR to stall (-71 EPROTO). TinyUSB's own
	// dcd_nrf5x.c sets ISOSPLIT when ISO endpoints are opened.
}

static void uac1_reset(uint8_t rhport)
{
	(void)rhport;
	g_uac1AltSetting = 0;
}

static uint16_t uac1_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
			  uint16_t max_len)
{
	(void)max_len;
	if (itf_desc->bInterfaceClass != TUSB_CLASS_AUDIO)
		return 0;

	if (itf_desc->bInterfaceSubClass == 0x01) {
		usbd_edpt_open(rhport, &s_iso_ep_out);
		/*
		 * IAD has bInterfaceCount=2, so TinyUSB pre-binds both AC
		 * and AS to this driver before calling open(). We must
		 * consume both interfaces' bytes here so the scanner doesn't
		 * hit AS again and fail the already-bound slot assertion.
		 * AC (51) + AS (52) = 103 bytes past the standard-interface ptr.
		 * AC = std(9)+CS_hdr(9)+input_term(12)+feat_unit(12)+out_term(9).
		 */
		return (9 + 9 + 12 + 12 + 9) + UAC1_AS_DESC_LEN;
	}
	return 0;
}

static bool uac1_control_xfer_cb(uint8_t rhport, uint8_t stage,
				 tusb_control_request_t const *request)
{
	if (stage != CONTROL_STAGE_SETUP)
		return true;

	if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
		if (request->bRequest != TUSB_REQ_SET_INTERFACE)
			return false;
		uint8_t itf = tu_u16_low(request->wIndex);
		uint8_t alt = tu_u16_low(request->wValue);
		if (itf == g_uac1ItfAs) {
			uint8_t prev_alt = g_uac1AltSetting;
			g_uac1AltSetting = alt;
			if (alt == 1 && prev_alt == 0) {
				usbd_edpt_xfer(rhport, g_uac1EpOut, g_isoOutBuf,
					       UAC1_ISO_EP_BUFSIZE);
			}
		}
		return tud_control_status(rhport, request);
	}

	if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS)
		return false;

	// Endpoint-directed: sampling frequency
	if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) {
		if (request->bRequest == 0x01) { // SET_CUR
			return tud_control_xfer(rhport, request, g_isoOutBuf,
						request->wLength);
		}
		// GET_CUR / GET_MIN / GET_MAX / GET_RES: return 48 kHz
		static uint8_t s_freq[3] = { 0x80, 0xBB, 0x00 };
		return tud_control_xfer(rhport, request, s_freq, sizeof s_freq);
	}

	// Interface-directed: mute / volume on the Feature Unit
	static uint8_t s_cur_mute = 0;
	static int16_t s_cur_vol[2] = { 0, 0 }; // 0 dB default
	uint8_t cs = tu_u16_high(request->wValue);

	if (request->bRequest == 0x01) { // SET_CUR
		if (cs == 0x01) { // Mute
			return tud_control_xfer(rhport, request, &s_cur_mute,
						1);
		}
		if (cs == 0x02) { // Volume
			uint8_t cn = tu_u16_low(request->wValue);
			uint8_t ch = (cn > 0 && cn <= 2) ? (cn - 1) : 0;
			return tud_control_xfer(rhport, request, &s_cur_vol[ch],
						sizeof(int16_t));
		}
		return tud_control_xfer(rhport, request, g_isoOutBuf,
					request->wLength);
	}

	if (cs == 0x01) {
		// Mute: 1-byte boolean
		return tud_control_xfer(rhport, request, &s_cur_mute, 1);
	}

	if (cs == 0x02) {
		// Volume: 16-bit signed 1/256-dB, little-endian
		uint8_t cn = tu_u16_low(request->wValue);
		uint8_t ch = (cn > 0 && cn <= 2) ? (cn - 1) : 0;
		switch (request->bRequest) {
		case 0x81: // GET_CUR
			return tud_control_xfer(rhport, request, &s_cur_vol[ch],
						sizeof(int16_t));
		case 0x82: { // GET_MIN: -46 dB (0xD200 LE)
			static const int16_t s_min = (int16_t)0xD200;
			return tud_control_xfer(rhport, request, (void *)&s_min,
						sizeof s_min);
		}
		case 0x83: { // GET_MAX: 0 dB
			static const int16_t s_max = 0;
			return tud_control_xfer(rhport, request, (void *)&s_max,
						sizeof s_max);
		}
		case 0x84: { // GET_RES: 1 dB (0x0100 LE)
			static const int16_t s_res = (int16_t)0x0100;
			return tud_control_xfer(rhport, request, (void *)&s_res,
						sizeof s_res);
		}
		default:
			return false;
		}
	}
	return false;
}

static bool uac1_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
			 uint32_t xferred_bytes)
{
	if (ep_addr == g_uac1EpOut && result == XFER_RESULT_SUCCESS) {
		if (xferred_bytes > 0 && g_uac1AltSetting == 1)
			processAudioSamples(g_isoOutBuf, xferred_bytes);
		if (g_uac1AltSetting == 1)
			usbd_edpt_xfer(rhport, g_uac1EpOut, g_isoOutBuf,
				       UAC1_ISO_EP_BUFSIZE);
		return true;
	}
	return false;
}

static const usbd_class_driver_t g_uac1Driver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "UAC1",
#endif
	.init = uac1_init,
	.reset = uac1_reset,
	.open = uac1_open,
	.control_xfer_cb = uac1_control_xfer_cb,
	.xfer_cb = uac1_xfer_cb,
	.sof = NULL
};

const usbd_class_driver_t *uac1_get_driver(void)
{
	return &g_uac1Driver;
}
