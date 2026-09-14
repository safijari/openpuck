#include "puck_hid_driver.h"

#include "bonds.h"
#include "config.h"
#include "rf_link.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

extern "C" {
#include "class/hid/hid_device.h"
}

// An inactive slot must reject FEATURE report 1 before accepting the control
// transfer. Accepting it makes host discovery treat an empty slot as a live
// controller; the ordinary set-report callback runs too late to reject SETUP.
static int8_t g_puckSlotByInterface[CFG_TUD_INTERFACE_MAX];
static uint8_t g_nextPuckSlot;

static void puckHidMapReset(void)
{
	for (uint8_t i = 0; i < CFG_TUD_INTERFACE_MAX; i++)
		g_puckSlotByInterface[i] = -1;
	g_nextPuckSlot = 0;
}

static void puckHidDriverInit(void)
{
	puckHidMapReset();
}

static bool puckHidDriverDeinit(void)
{
	return true;
}

static void puckHidDriverReset(uint8_t rhport)
{
	(void)rhport;
	puckHidMapReset();
}

static bool puckHidMode(void)
{
	return g_usbMode == MODE_STEAM || g_usbMode == MODE_LIZARD;
}

static bool puckHidSlotLive(int slot)
{
	if (slot < 0 || slot >= NSLOT || !g_slot[slot].used ||
	    !g_connReplyMs[slot])
		return false;

	return (uint32_t)(millis() - g_connReplyMs[slot]) < 1200u;
}

static uint16_t puckHidDriverOpen(uint8_t rhport,
				  tusb_desc_interface_t const *itf,
				  uint16_t maxLen)
{
	if (!puckHidMode() || itf->bInterfaceClass != TUSB_CLASS_HID ||
	    itf->bInterfaceSubClass != HID_SUBCLASS_NONE ||
	    itf->bInterfaceProtocol != HID_ITF_PROTOCOL_NONE)
		return 0;

	if (g_nextPuckSlot >= NSLOT ||
	    itf->bInterfaceNumber >= CFG_TUD_INTERFACE_MAX)
		return 0;

	uint16_t used = hidd_open(rhport, itf, maxLen);
	if (!used)
		return 0;

	g_puckSlotByInterface[itf->bInterfaceNumber] = g_nextPuckSlot++;
	return used;
}

static int puckHidSlotForRequest(tusb_control_request_t const *request)
{
	uint16_t itf = request->wIndex;
	if (itf >= CFG_TUD_INTERFACE_MAX)
		return -1;
	return g_puckSlotByInterface[itf];
}

static bool puckHidDriverControl(uint8_t rhport, uint8_t stage,
				 tusb_control_request_t const *request)
{
	if (stage == CONTROL_STAGE_SETUP &&
	    request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
	    request->bRequest == HID_REQ_CONTROL_SET_REPORT) {
		uint8_t reportType = tu_u16_high(request->wValue);
		uint8_t reportId = tu_u16_low(request->wValue);
		int slot = puckHidSlotForRequest(request);

		if (slot >= 0 && reportType == HID_REPORT_TYPE_FEATURE &&
		    reportId == 1 && !puckHidSlotLive(slot))
			return false;
	}

	return hidd_control_xfer_cb(rhport, stage, request);
}

static const usbd_class_driver_t g_puckHidDriver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "PUCK-HID",
#endif
	.init = puckHidDriverInit,
	.deinit = puckHidDriverDeinit,
	.reset = puckHidDriverReset,
	.open = puckHidDriverOpen,
	.control_xfer_cb = puckHidDriverControl,
	.xfer_cb = hidd_xfer_cb,
	.sof = NULL,
};

const usbd_class_driver_t *puckHidClassDriver(void)
{
	return &g_puckHidDriver;
}
