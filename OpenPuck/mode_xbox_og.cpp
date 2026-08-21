#include "mode_xbox_og.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_tx.h"
#include "usb_app_drivers.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

#define XBOX_OG_DESC_LEN (9 + 7 + 7)
#define XBOX_OG_PACKET_SIZE 32
#define XBOX_OG_CONN_MS 1200u

enum {
	XBOX_OG_DUP = 0x01,
	XBOX_OG_DDOWN = 0x02,
	XBOX_OG_DLEFT = 0x04,
	XBOX_OG_DRIGHT = 0x08,
	XBOX_OG_START = 0x10,
	XBOX_OG_BACK = 0x20,
	XBOX_OG_L3 = 0x40,
	XBOX_OG_R3 = 0x80,
};

#pragma pack(push, 1)
struct XboxOgInputReport {
	uint8_t reserved;
	uint8_t length;
	uint8_t buttons;
	uint8_t reserved2;
	uint8_t a;
	uint8_t b;
	uint8_t x;
	uint8_t y;
	uint8_t black;
	uint8_t white;
	uint8_t left_trigger;
	uint8_t right_trigger;
	int16_t left_x;
	int16_t left_y;
	int16_t right_x;
	int16_t right_y;
};

struct XboxOgOutputReport {
	uint8_t reserved;
	uint8_t length;
	uint16_t left_motor;
	uint16_t right_motor;
};
#pragma pack(pop)

static const uint8_t XID_DESCRIPTOR[] = {
	0x10, 0x42, 0x00, 0x01, 0x01, 0x02, 0x14, 0x06,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};
static const uint8_t XID_CAPABILITIES_IN[] = {
	0x00, 0x14, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};
static const uint8_t XID_CAPABILITIES_OUT[] = {
	0x00, 0x06, 0xFF, 0xFF, 0xFF, 0xFF,
};

static_assert(sizeof(XboxOgInputReport) == 20,
	      "Original Xbox input report must be 20 bytes");
static_assert(sizeof(XboxOgOutputReport) == 6,
	      "Original Xbox output report must be 6 bytes");
static_assert(sizeof(XID_DESCRIPTOR) == 16, "XID descriptor must be 16 bytes");
static_assert(sizeof(XID_CAPABILITIES_IN) == 20,
	      "XID input capabilities must be 20 bytes");
static_assert(sizeof(XID_CAPABILITIES_OUT) == 6,
	      "XID output capabilities must be 6 bytes");

struct XboxOgSlot {
	uint8_t interface_number;
	uint8_t endpoint_in;
	uint8_t endpoint_out;
	// The loop owns bond selection and is its only reader -- it snapshots
	// bond_slot when relaying rumble; lifecycle callbacks only clear it.
	volatile int8_t bond_slot;
	// TinyUSB callbacks publish endpoint readiness from the usbd task; the
	// loop owns pending input state.
	volatile bool in_use;
	volatile bool tx_pending;
	// TinyUSB ignores xfer_cb's return value, so the loop drain must remember
	// and recover a failed runtime OUT rearm.
	volatile bool output_rearm_pending;

	// The loop publishes complete reports under PRIMASK; USB only snapshots it.
	XboxOgInputReport latest_report;
	// These four are owned by the USB peripheral, not by a lock, so the
	// rules below hold only while the endpoints are live -- bus reset and
	// deinit clear them wholesale. See xboxOgClearSlot().
	// EP0 IN owns this snapshot until the control transfer completes.
	uint8_t control_in_buffer[sizeof(XboxOgInputReport)];
	// The loop may replace this only while the interrupt IN endpoint is idle.
	CFG_TUD_MEM_ALIGN uint8_t tx_buffer[XBOX_OG_PACKET_SIZE];
	// Interrupt OUT DMA owns this buffer between arm and completion.
	CFG_TUD_MEM_ALIGN uint8_t output_buffer[XBOX_OG_PACKET_SIZE];
	// EP0 OUT owns this buffer from SET_REPORT setup through its data stage.
	uint8_t control_out_buffer[sizeof(XboxOgOutputReport)];

	// USB output callbacks publish this mailbox; the loop consumes it and
	// relays to whichever slot is currently bonded when it does.
	volatile uint16_t rumble_low;
	volatile uint16_t rumble_high;
	volatile bool rumble_pending;
};

static XboxOgSlot g_xboxOgSlot;
static volatile uint32_t g_xboxOgGeneration;
static unsigned long g_xboxOgNeutralMs;

static void xboxOgNeutralReport(XboxOgInputReport &report)
{
	memset(&report, 0, sizeof report);
	report.length = sizeof report;
}

// Wholesale clear, DMA buffers included. Every caller runs at a point where
// no transfer is armed against them: mode setup runs before attach, init /
// deinit / bus reset arrive after the peripheral has dropped the endpoints,
// and xboxOgOpen clears before it arms the OUT endpoint (or after closing
// both on the failure path).
static void xboxOgClearSlot(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	g_xboxOgGeneration++;
	memset(&g_xboxOgSlot, 0, sizeof g_xboxOgSlot);
	g_xboxOgSlot.bond_slot = -1;
	xboxOgNeutralReport(g_xboxOgSlot.latest_report);
	__set_PRIMASK(primask);
}

static void xboxOgSnapshotReport(uint8_t *destination)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	memcpy(destination, &g_xboxOgSlot.latest_report,
	       sizeof g_xboxOgSlot.latest_report);
	__set_PRIMASK(primask);
}

static void xboxOgPublishReport(const XboxOgInputReport &report)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	memcpy(&g_xboxOgSlot.latest_report, &report, sizeof report);
	g_xboxOgSlot.tx_pending = true;
	__set_PRIMASK(primask);
}

static bool xboxOgPublishBondReport(const XboxOgInputReport &report, int slot,
				    uint32_t generation)
{
	uint32_t primask = __get_PRIMASK();
	bool published = false;

	__disable_irq();
	if (g_xboxOgGeneration == generation &&
	    g_xboxOgSlot.bond_slot == slot) {
		memcpy(&g_xboxOgSlot.latest_report, &report, sizeof report);
		g_xboxOgSlot.tx_pending = true;
		published = true;
	}
	__set_PRIMASK(primask);
	return published;
}

static bool xboxOgApplyRumble(XboxOgSlot &slot, const uint8_t *data,
			      uint16_t length)
{
	if (!data || length < sizeof(XboxOgOutputReport) || data[0] != 0x00 ||
	    data[1] != sizeof(XboxOgOutputReport))
		return false;

	uint16_t low = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
	uint16_t high = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	slot.rumble_low = low;
	slot.rumble_high = high;
	slot.rumble_pending = true;
	__set_PRIMASK(primask);
	return true;
}

static void xboxOgRetryOutputRearm(void)
{
	XboxOgSlot &slot = g_xboxOgSlot;
	uint32_t generation = g_xboxOgGeneration;
	uint8_t endpoint = slot.endpoint_out;

	if (!tud_mounted() || !slot.in_use || !slot.output_rearm_pending ||
	    endpoint == 0 || usbd_edpt_busy(0, endpoint) ||
	    !usbd_edpt_claim(0, endpoint))
		return;
	if (usbd_edpt_xfer(0, endpoint, slot.output_buffer,
			   sizeof slot.output_buffer)) {
		uint32_t primask = __get_PRIMASK();

		__disable_irq();
		if (g_xboxOgGeneration == generation && slot.in_use &&
		    slot.endpoint_out == endpoint)
			slot.output_rearm_pending = false;
		__set_PRIMASK(primask);
	} else {
		usbd_edpt_release(0, endpoint);
	}
}

static void xboxOgTxDrain(void)
{
	XboxOgSlot &slot = g_xboxOgSlot;

	xboxOgRetryOutputRearm();

	if (!tud_mounted() || !slot.in_use || !slot.tx_pending ||
	    slot.endpoint_in == 0 || usbd_edpt_busy(0, slot.endpoint_in))
		return;
	if (!usbd_edpt_claim(0, slot.endpoint_in))
		return;

	xboxOgSnapshotReport(slot.tx_buffer);
	if (usbd_edpt_xfer(0, slot.endpoint_in, slot.tx_buffer,
			   sizeof(XboxOgInputReport))) {
		slot.tx_pending = false;
	} else {
		usbd_edpt_release(0, slot.endpoint_in);
	}
}

// The console asks for these three before it accepts the controller: 0x06
// with wValue 0x4200 is the XID descriptor, 0x01 with 0x0100/0x0200 the
// input and output capabilities. Stalling them makes it refuse the pad.
static bool xboxOgVendorRequest(const tusb_control_request_t *request)
{
	if (!g_xboxOgSlot.in_use || request->bmRequestType != 0xC1 ||
	    request->wIndex != g_xboxOgSlot.interface_number)
		return false;

	if (request->bRequest == 0x06 && request->wValue == 0x4200)
		return true;
	if (request->bRequest != 0x01)
		return false;
	return request->wValue == 0x0100 || request->wValue == 0x0200;
}

bool xboxOgVendorControlXfer(uint8_t rhport, uint8_t stage,
			     const tusb_control_request_t *request)
{
	if (!request || !xboxOgVendorRequest(request))
		return false;
	if (stage != CONTROL_STAGE_SETUP)
		return true;

	if (request->bRequest == 0x06)
		return tud_control_xfer(rhport, request,
					const_cast<uint8_t *>(XID_DESCRIPTOR),
					sizeof XID_DESCRIPTOR);
	if (request->wValue == 0x0100)
		return tud_control_xfer(
			rhport, request,
			const_cast<uint8_t *>(XID_CAPABILITIES_IN),
			sizeof XID_CAPABILITIES_IN);
	return tud_control_xfer(rhport, request,
				const_cast<uint8_t *>(XID_CAPABILITIES_OUT),
				sizeof XID_CAPABILITIES_OUT);
}

// Controller S also carries input and rumble over EP0, in parallel with the
// interrupt endpoints: GET_REPORT (0xA1/0x01, wValue 0x0100) and SET_REPORT
// (0x21/0x09, wValue 0x0200).
static bool xboxOgClassControlXfer(uint8_t rhport, uint8_t stage,
				   const tusb_control_request_t *request)
{
	if (!request || !g_xboxOgSlot.in_use ||
	    request->wIndex != g_xboxOgSlot.interface_number)
		return false;

	if (request->bmRequestType == 0xA1 && request->bRequest == 0x01 &&
	    request->wValue == 0x0100) {
		if (stage == CONTROL_STAGE_SETUP) {
			xboxOgSnapshotReport(g_xboxOgSlot.control_in_buffer);
			return tud_control_xfer(
				rhport, request, g_xboxOgSlot.control_in_buffer,
				sizeof g_xboxOgSlot.control_in_buffer);
		}
		return true;
	}

	if (request->bmRequestType != 0x21 || request->bRequest != 0x09 ||
	    request->wValue != 0x0200 ||
	    request->wLength != sizeof(XboxOgOutputReport))
		return false;

	if (stage == CONTROL_STAGE_SETUP) {
		memset(g_xboxOgSlot.control_out_buffer, 0,
		       sizeof g_xboxOgSlot.control_out_buffer);
		return tud_control_xfer(rhport, request,
					g_xboxOgSlot.control_out_buffer,
					sizeof g_xboxOgSlot.control_out_buffer);
	}
	if (stage == CONTROL_STAGE_DATA)
		return xboxOgApplyRumble(
			g_xboxOgSlot, g_xboxOgSlot.control_out_buffer,
			sizeof g_xboxOgSlot.control_out_buffer);
	return true;
}

static void xboxOgInit(void)
{
	xboxOgClearSlot();
}

static bool xboxOgDeinit(void)
{
	xboxOgClearSlot();
	return true;
}

static void xboxOgReset(uint8_t rhport)
{
	(void)rhport;
	xboxOgClearSlot();
}

static bool xboxOgEndpointValid(const tusb_desc_endpoint_t *endpoint)
{
	return endpoint->bLength == sizeof(tusb_desc_endpoint_t) &&
	       endpoint->bDescriptorType == TUSB_DESC_ENDPOINT &&
	       endpoint->bmAttributes.xfer == TUSB_XFER_INTERRUPT &&
	       endpoint->bmAttributes.sync == 0 &&
	       endpoint->bmAttributes.usage == 0 &&
	       (endpoint->bEndpointAddress & 0x70) == 0 &&
	       tu_edpt_number(endpoint->bEndpointAddress) != 0 &&
	       tu_le16toh(endpoint->wMaxPacketSize) == XBOX_OG_PACKET_SIZE &&
	       endpoint->bInterval == 4;
}

static uint16_t xboxOgOpen(uint8_t rhport,
			   const tusb_desc_interface_t *interface,
			   uint16_t max_length)
{
	if (!interface || g_xboxOgSlot.in_use ||
	    max_length < sizeof(tusb_desc_interface_t) ||
	    interface->bLength != sizeof(tusb_desc_interface_t) ||
	    interface->bDescriptorType != TUSB_DESC_INTERFACE ||
	    interface->bAlternateSetting != 0 || interface->iInterface != 0 ||
	    interface->bNumEndpoints != 2 ||
	    interface->bInterfaceClass != 0x58 ||
	    interface->bInterfaceSubClass != 0x42 ||
	    interface->bInterfaceProtocol != 0x00)
		return 0;

	const uint8_t *start = reinterpret_cast<const uint8_t *>(interface);
	const uint8_t *cursor = start + interface->bLength;
	const uint8_t *end = start + max_length;
	const tusb_desc_endpoint_t *endpoint_in = nullptr;
	const tusb_desc_endpoint_t *endpoint_out = nullptr;
	uint8_t endpoint_count = 0;

	while (cursor < end) {
		uint16_t remaining = (uint16_t)(end - cursor);

		if (remaining < 2 || cursor[0] < 2 || cursor[0] > remaining)
			return 0;
		if (cursor[1] == TUSB_DESC_INTERFACE) {
			if (cursor[0] < sizeof(tusb_desc_interface_t))
				return 0;
			break;
		}
		if (cursor[1] == TUSB_DESC_INTERFACE_ASSOCIATION) {
			if (cursor[0] < sizeof(tusb_desc_interface_assoc_t))
				return 0;
			break;
		}
		if (cursor[1] == TUSB_DESC_ENDPOINT) {
			if (cursor[0] < sizeof(tusb_desc_endpoint_t))
				return 0;
			const tusb_desc_endpoint_t *endpoint =
				reinterpret_cast<const tusb_desc_endpoint_t *>(
					cursor);

			if (!xboxOgEndpointValid(endpoint) ||
			    ++endpoint_count > interface->bNumEndpoints)
				return 0;
			if (tu_edpt_dir(endpoint->bEndpointAddress) ==
			    TUSB_DIR_IN) {
				if (endpoint_in)
					return 0;
				endpoint_in = endpoint;
			} else {
				if (endpoint_out)
					return 0;
				endpoint_out = endpoint;
			}
		}
		cursor += cursor[0];
	}

	if (endpoint_count != 2 || !endpoint_in || !endpoint_out)
		return 0;
	if (!usbd_edpt_open(rhport, endpoint_in))
		return 0;
	if (!usbd_edpt_open(rhport, endpoint_out)) {
		usbd_edpt_close(rhport, endpoint_in->bEndpointAddress);
		return 0;
	}

	xboxOgClearSlot();
	g_xboxOgSlot.interface_number = interface->bInterfaceNumber;
	g_xboxOgSlot.endpoint_in = endpoint_in->bEndpointAddress;
	g_xboxOgSlot.endpoint_out = endpoint_out->bEndpointAddress;
	g_xboxOgSlot.in_use = true;
	if (!usbd_edpt_xfer(rhport, g_xboxOgSlot.endpoint_out,
			    g_xboxOgSlot.output_buffer,
			    sizeof g_xboxOgSlot.output_buffer)) {
		usbd_edpt_close(rhport, g_xboxOgSlot.endpoint_in);
		usbd_edpt_close(rhport, g_xboxOgSlot.endpoint_out);
		xboxOgClearSlot();
		return 0;
	}

	return (uint16_t)(cursor - start);
}

static bool xboxOgXfer(uint8_t rhport, uint8_t endpoint, xfer_result_t result,
		       uint32_t transferred)
{
	if (!g_xboxOgSlot.in_use)
		return false;
	if (endpoint == g_xboxOgSlot.endpoint_in)
		return true;
	if (endpoint != g_xboxOgSlot.endpoint_out)
		return false;

	if (result == XFER_RESULT_SUCCESS)
		xboxOgApplyRumble(g_xboxOgSlot, g_xboxOgSlot.output_buffer,
				  (uint16_t)transferred);
	if (!g_xboxOgSlot.in_use)
		return true;
	if (!usbd_edpt_xfer(rhport, g_xboxOgSlot.endpoint_out,
			    g_xboxOgSlot.output_buffer,
			    sizeof g_xboxOgSlot.output_buffer)) {
		uint32_t primask = __get_PRIMASK();

		__disable_irq();
		if (g_xboxOgSlot.in_use &&
		    endpoint == g_xboxOgSlot.endpoint_out)
			g_xboxOgSlot.output_rearm_pending = true;
		__set_PRIMASK(primask);
	}
	return true;
}

static const usbd_class_driver_t g_xboxOgDriver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "XID",
#endif
	.init = xboxOgInit,
	.deinit = xboxOgDeinit,
	.reset = xboxOgReset,
	.open = xboxOgOpen,
	.control_xfer_cb = xboxOgClassControlXfer,
	.xfer_cb = xboxOgXfer,
	.sof = nullptr,
};

const usbd_class_driver_t *xboxOgClassDriver(void)
{
	return &g_xboxOgDriver;
}

class Adafruit_USBD_XboxOg : public Adafruit_USBD_Interface {
    public:
	uint16_t getInterfaceDescriptor(uint8_t, uint8_t *buffer,
					uint16_t buffer_size) override
	{
		if (!buffer)
			return XBOX_OG_DESC_LEN;
		if (buffer_size < XBOX_OG_DESC_LEN)
			return 0;

		uint8_t interface_number = TinyUSBDevice.allocInterface(1);
		uint8_t endpoint_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
		uint8_t endpoint_out =
			TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		const uint8_t descriptor[XBOX_OG_DESC_LEN] = {
			9,
			TUSB_DESC_INTERFACE,
			interface_number,
			0,
			2,
			0x58,
			0x42,
			0x00,
			0,
			7,
			TUSB_DESC_ENDPOINT,
			endpoint_in,
			TUSB_XFER_INTERRUPT,
			U16_TO_U8S_LE(XBOX_OG_PACKET_SIZE),
			4,
			7,
			TUSB_DESC_ENDPOINT,
			endpoint_out,
			TUSB_XFER_INTERRUPT,
			U16_TO_U8S_LE(XBOX_OG_PACKET_SIZE),
			4,
		};

		memcpy(buffer, descriptor, sizeof descriptor);
		return sizeof descriptor;
	}

	bool begin()
	{
		return TinyUSBDevice.addInterface(*this);
	}
};

static Adafruit_USBD_XboxOg g_xboxOgInterface;

XboxOgController g_xboxOgCtl;

enum XboxOgRemapTarget : uint8_t {
	XBOX_OG_TARGET_NONE,
	XBOX_OG_TARGET_A,
	XBOX_OG_TARGET_B,
	XBOX_OG_TARGET_X,
	XBOX_OG_TARGET_Y,
	XBOX_OG_TARGET_WHITE,
	XBOX_OG_TARGET_BLACK,
	XBOX_OG_TARGET_L3,
	XBOX_OG_TARGET_R3,
	XBOX_OG_TARGET_BACK,
	XBOX_OG_TARGET_START,
	XBOX_OG_TARGET_DUP,
	XBOX_OG_TARGET_DDOWN,
	XBOX_OG_TARGET_DLEFT,
	XBOX_OG_TARGET_DRIGHT,
};

// Remaps target Controller S outputs independently of the RF button masks.
// Codes come from the shared remap code space: 5/6 (LB/RB) drive White/Black,
// 9/10 are semantic Back/Start, and 11 (Guide) has no Controller S target.
static constexpr XboxOgRemapTarget XBOX_OG_REMAP[] = {
	XBOX_OG_TARGET_NONE,   XBOX_OG_TARGET_A,     XBOX_OG_TARGET_B,
	XBOX_OG_TARGET_X,      XBOX_OG_TARGET_Y,     XBOX_OG_TARGET_WHITE,
	XBOX_OG_TARGET_BLACK,  XBOX_OG_TARGET_L3,    XBOX_OG_TARGET_R3,
	XBOX_OG_TARGET_BACK,   XBOX_OG_TARGET_START, XBOX_OG_TARGET_NONE,
	XBOX_OG_TARGET_DUP,    XBOX_OG_TARGET_DDOWN, XBOX_OG_TARGET_DLEFT,
	XBOX_OG_TARGET_DRIGHT,
};

static_assert(sizeof XBOX_OG_REMAP / sizeof XBOX_OG_REMAP[0] == 16,
	      "Xbox remap table must cover codes 0 through 15");

static void xboxOgApplyRemap(XboxOgInputReport &report, uint8_t code)
{
	uint32_t mapped = tritonFromCode(code);

	// LT/RT are codes 19/20, outside the table -- these two must stay above
	// the bounds check or a paddle mapped to a trigger does nothing.
	if (mapped & TB_L2)
		report.left_trigger = 0xFF;
	if (mapped & TB_R2)
		report.right_trigger = 0xFF;
	if (code >= sizeof XBOX_OG_REMAP / sizeof XBOX_OG_REMAP[0])
		return;

	switch (XBOX_OG_REMAP[code]) {
	case XBOX_OG_TARGET_A:
		report.a = 0xFF;
		break;
	case XBOX_OG_TARGET_B:
		report.b = 0xFF;
		break;
	case XBOX_OG_TARGET_X:
		report.x = 0xFF;
		break;
	case XBOX_OG_TARGET_Y:
		report.y = 0xFF;
		break;
	case XBOX_OG_TARGET_WHITE:
		report.white = 0xFF;
		break;
	case XBOX_OG_TARGET_BLACK:
		report.black = 0xFF;
		break;
	case XBOX_OG_TARGET_L3:
		report.buttons |= XBOX_OG_L3;
		break;
	case XBOX_OG_TARGET_R3:
		report.buttons |= XBOX_OG_R3;
		break;
	case XBOX_OG_TARGET_BACK:
		report.buttons |= XBOX_OG_BACK;
		break;
	case XBOX_OG_TARGET_START:
		report.buttons |= XBOX_OG_START;
		break;
	case XBOX_OG_TARGET_DUP:
		report.buttons |= XBOX_OG_DUP;
		break;
	case XBOX_OG_TARGET_DDOWN:
		report.buttons |= XBOX_OG_DDOWN;
		break;
	case XBOX_OG_TARGET_DLEFT:
		report.buttons |= XBOX_OG_DLEFT;
		break;
	case XBOX_OG_TARGET_DRIGHT:
		report.buttons |= XBOX_OG_DRIGHT;
		break;
	case XBOX_OG_TARGET_NONE:
	default:
		break;
	}
}

static void xboxOgBuildReport(XboxOgInputReport &report, const uint8_t *raw)
{
	uint32_t buttons = btnsOf(raw);

	xboxOgNeutralReport(report);
	if (buttons & TB_DUP)
		report.buttons |= XBOX_OG_DUP;
	if (buttons & TB_DDN)
		report.buttons |= XBOX_OG_DDOWN;
	if (buttons & TB_DLF)
		report.buttons |= XBOX_OG_DLEFT;
	if (buttons & TB_DRT)
		report.buttons |= XBOX_OG_DRIGHT;
	// The bit names are the reverse of the physical labels: the left
	// button (View) arrives as TB_MENU and the right one as TB_VIEW.
	// Back is the left button on an Original Xbox, so this is positional.
	if (buttons & TB_VIEW)
		report.buttons |= XBOX_OG_START;
	if (buttons & TB_MENU)
		report.buttons |= XBOX_OG_BACK;
	if (buttons & TB_L3)
		report.buttons |= XBOX_OG_L3;
	if (buttons & TB_R3)
		report.buttons |= XBOX_OG_R3;

	// A/B/X/Y and White/Black are analog pressure bytes, so a digital press
	// writes full scale; the shoulders have no button bit at all.
	if (buttons & TB_A)
		(g_abSwap ? report.b : report.a) = 0xFF;
	if (buttons & TB_B)
		(g_abSwap ? report.a : report.b) = 0xFF;
	if (buttons & TB_X)
		(g_abSwap ? report.y : report.x) = 0xFF;
	if (buttons & TB_Y)
		(g_abSwap ? report.x : report.y) = 0xFF;
	if (buttons & TB_LB)
		report.white = 0xFF;
	if (buttons & TB_RB)
		report.black = 0xFF;

	report.left_trigger = trigU8(u16off(raw, 4));
	report.right_trigger = trigU8(u16off(raw, 6));
	if (buttons & TB_L2)
		report.left_trigger = 0xFF;
	if (buttons & TB_R2)
		report.right_trigger = 0xFF;

	const uint32_t paddle_buttons[] = { TB_L4, TB_R4, TB_L5, TB_R5 };

	for (int i = 0; i < 4; i++)
		if (buttons & paddle_buttons[i])
			xboxOgApplyRemap(report, g_back[i]);
	if ((buttons & TB_QAM) && g_qamMap)
		xboxOgApplyRemap(report, g_qamMap);

	// Raw-report offsets, not slotSticks(): like mode_xinput this mode decodes 0x45 in place and never
	// touches g_in. 16/18 = left pad X/Y, 22/24 = right pad X/Y.
	int16_t lx = (int16_t)s16off(raw, 8), ly = (int16_t)s16off(raw, 10),
		rx = (int16_t)s16off(raw, 12), ry = (int16_t)s16off(raw, 14);
	padStickBlend(buttons, (int16_t)s16off(raw, 16),
		      (int16_t)s16off(raw, 18), (int16_t)s16off(raw, 22),
		      (int16_t)s16off(raw, 24), &lx, &ly, &rx, &ry);
	le16(reinterpret_cast<uint8_t *>(&report.left_x), lx);
	le16(reinterpret_cast<uint8_t *>(&report.left_y), ly);
	le16(reinterpret_cast<uint8_t *>(&report.right_x), rx);
	le16(reinterpret_cast<uint8_t *>(&report.right_y), ry);
}

static bool xboxOgBondRecent(int slot, unsigned long now)
{
	return slot >= 0 && slot < NSLOT && g_slot[slot].used &&
	       g_connReplyMs[slot] != 0 &&
	       (unsigned long)(now - g_connReplyMs[slot]) < XBOX_OG_CONN_MS;
}

static void xboxOgUpdateBond(void)
{
	XboxOgSlot &state = g_xboxOgSlot;
	unsigned long now = millis();
	int current = state.bond_slot;

	if (xboxOgBondRecent(current, now))
		return;

	if (current >= 0) {
		XboxOgInputReport neutral;

		state.bond_slot = -1;
		xboxOgNeutralReport(neutral);
		xboxOgPublishReport(neutral);
		g_xboxOgNeutralMs = now;
	}
	for (int slot = 0; slot < NSLOT; slot++)
		if (xboxOgBondRecent(slot, now)) {
			state.bond_slot = (int8_t)slot;
			break;
		}
}

// Fire-and-forget: no stop is tracked or sent here. The shared relay in
// haptics.cpp (burst, reconnect scrub, per-slot watchdog) stops the motor.
static void xboxOgHandleRumble(void)
{
	XboxOgSlot &state = g_xboxOgSlot;
	uint16_t low, high;
	bool pending;
	int8_t target;
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	pending = state.rumble_pending;
	low = state.rumble_low;
	high = state.rumble_high;
	target = state.bond_slot;
	state.rumble_pending = false;
	__set_PRIMASK(primask);

	if (!pending || target < 0 || !g_rumble)
		return;
	hapticSteamRumble(low, high, (uint8_t)target);
}

void XboxOgController::begin()
{
	xboxOgClearSlot();
	g_xboxOgNeutralMs = 0;
	usbIdentity();
	USBDevice.setConfigurationMaxPower(100);
	g_xboxOgInterface.begin();
	usbTxRegisterDrain(xboxOgTxDrain);
}

void XboxOgController::onReport45(int slot, const uint8_t *rep, bool fresh,
				  uint8_t bodyTlen)
{
	(void)fresh;
	if (!rep || slot < 0 || slot >= NSLOT || bodyTlen < 18 ||
	    (rep[0] != 0x45 && rep[0] != 0x42) ||
	    g_xboxOgSlot.bond_slot != slot)
		return;

	uint32_t generation = g_xboxOgGeneration;
	XboxOgInputReport report;

	xboxOgBuildReport(report, rep);
	xboxOgPublishBondReport(report, slot, generation);
}

void XboxOgController::task()
{
	xboxOgUpdateBond();
	xboxOgHandleRumble();

	if (g_xboxOgSlot.bond_slot >= 0 ||
	    (unsigned long)(millis() - g_xboxOgNeutralMs) < USB_STREAM_MS)
		return;

	XboxOgInputReport neutral;

	g_xboxOgNeutralMs = millis();
	xboxOgNeutralReport(neutral);
	xboxOgPublishReport(neutral);
}

void XboxOgController::usbIdentity()
{
	USBDevice.setID(0x045E, 0x0289);
	USBDevice.setDeviceVersion(0x0121);
	// A real Controller S sends no string descriptors (iManufacturer /
	// iProduct / iSerial = 0); hosts name it from usb.ids. TinyUSB pins
	// those indices non-zero, so mirror what usb.ids shows for 045E:0289.
	USBDevice.setManufacturerDescriptor("Microsoft Corp.");
	USBDevice.setProductDescriptor("Xbox Controller S");
}
