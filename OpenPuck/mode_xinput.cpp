// One wired Xbox 360 pad, independent of RF controller joins and departures.
#include "mode_xinput.h"
#include "xinput_auth.h"
#include "xinput_descriptors.h"
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

XboxController g_xboxCtl;

enum {
	XB_DUP = 0x0001,
	XB_DDOWN = 0x0002,
	XB_DLEFT = 0x0004,
	XB_DRIGHT = 0x0008,
	XB_START = 0x0010,
	XB_BACK = 0x0020,
	XB_L3 = 0x0040,
	XB_R3 = 0x0080,
	XB_LB = 0x0100,
	XB_RB = 0x0200,
	XB_GUIDE = 0x0400,
	XB_A = 0x1000,
	XB_B = 0x2000,
	XB_X = 0x4000,
	XB_Y = 0x8000
};

#define XINPUT_CONN_MS 1200u
#define RUMBLE_STUCK_MS 2500u

struct XiState {
	volatile bool inUse;
	volatile int8_t bond;
	volatile bool txPending;
	volatile bool neutralPending;
	volatile bool outRearm;
	uint8_t claimedInterfaces;
	uint8_t rhport;
	uint8_t latest[20];
	CFG_TUD_MEM_ALIGN uint8_t inBuf[32];
	CFG_TUD_MEM_ALIGN uint8_t outBuf[32];
	volatile uint16_t rumbleLow, rumbleHigh;
	volatile unsigned long rumbleMs;
	volatile bool rumblePending;
	volatile int8_t rumbleBond;
	volatile uint8_t player;
};

static XiState g_xi;
static volatile uint32_t g_xiGeneration;
// Reset callbacks must stop the old RF destination later, never a replacement.
static volatile uint8_t g_stopBonds;

static void xiNeutral(uint8_t *report)
{
	memset(report, 0, 20);
	report[1] = 20;
}

// Only used before attach or after TinyUSB has cancelled endpoint DMA.
static void xiClear(void)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (g_xi.inUse && g_xi.bond >= 0)
		g_stopBonds |= 1u << g_xi.bond;
	g_xiGeneration++;
	memset(&g_xi, 0, sizeof g_xi);
	g_xi.bond = g_xi.rumbleBond = -1;
	xiNeutral(g_xi.latest);
	xiNeutral(g_xi.inBuf);
	g_xi.neutralPending = true;
	__set_PRIMASK(pm);
	xinputAuthReset();
}

static void xi_init(void)
{
	xiClear();
}

static bool xi_deinit(void)
{
	xiClear();
	return true;
}

static void xi_reset(uint8_t rhport)
{
	(void)rhport;
	xiClear();
}

static uint16_t xi_open(uint8_t rhport, tusb_desc_interface_t const *itf,
			uint16_t max_len)
{
	if (!itf || max_len < sizeof(tusb_desc_interface_t) ||
	    itf->bInterfaceNumber >= 4)
		return 0;
	static const uint16_t starts[] = { 0, 40, 104, 129, 144 };
	uint8_t number = itf->bInterfaceNumber;
	uint16_t start = starts[number];
	uint16_t length = starts[number + 1] - start;
	if (max_len < length || (g_xi.claimedInterfaces & (1u << number)))
		return 0;

	const uint8_t *body = (const uint8_t *)itf;
	const tusb_desc_endpoint_t *epIn = nullptr, *epOut = nullptr;
	uint16_t offset = 0;
	while (offset < length) {
		if (length - offset < 2)
			return 0;
		uint8_t size = body[offset], type = body[offset + 1];
		if (size < 2 || size > length - offset ||
		    size != XINPUT_CONFIG_BODY[start + offset])
			return 0;
		// The fixed personality needs exact interface/endpoint numbers and
		// vendor blobs. Only the security string is allocated by Adafruit.
		for (uint16_t i = offset; i < offset + size; i++) {
			if (start + i != XINPUT_SECURITY_STRING_OFFSET &&
			    body[i] != XINPUT_CONFIG_BODY[start + i])
				return 0;
		}
		if (type == TUSB_DESC_ENDPOINT && number == 0) {
			const tusb_desc_endpoint_t *ep =
				(const tusb_desc_endpoint_t *)(body + offset);
			if (ep->bEndpointAddress == 0x81)
				epIn = ep;
			else if (ep->bEndpointAddress == 0x02)
				epOut = ep;
		}
		offset += size;
	}

	// The accessory interfaces identify the genuine pad layout, but no
	// headset or chatpad is present. Do not open or feed their endpoints;
	// input's reserved bytes stay zero (no accessory-presence flags).
	// Claim separately: older TinyUSB versions bind only the first interface
	// of a returned descriptor span when there is no IAD.
	if (number != 0) {
		g_xi.claimedInterfaces |= 1u << number;
		return length;
	}
	if (!epIn || !epOut)
		return 0;
	if (!usbd_edpt_open(rhport, epIn))
		return 0;
	if (!usbd_edpt_open(rhport, epOut)) {
		usbd_edpt_close(rhport, 0x81);
		return 0;
	}
	g_xi.claimedInterfaces |= 1u << number;
	g_xi.rhport = rhport;
	g_xi.inUse = true;
	g_xi.neutralPending = true;
	g_xi.outRearm =
		!usbd_edpt_xfer(rhport, 0x02, g_xi.outBuf, sizeof g_xi.outBuf);
	return length;
}

static bool xi_ctrl(uint8_t rhport, uint8_t stage,
		    tusb_control_request_t const *request)
{
	return xinputAuthControl(rhport, stage, request);
}

static bool xi_xfer(uint8_t rhport, uint8_t ep, xfer_result_t result,
		    uint32_t length)
{
	if (!g_xi.inUse || rhport != g_xi.rhport)
		return false;
	if (ep == 0x81) {
		if (result != XFER_RESULT_SUCCESS)
			g_xi.neutralPending = true;
		return true;
	}
	if (ep != 0x02)
		return false;
	if (result == XFER_RESULT_SUCCESS) {
		const uint8_t *data = g_xi.outBuf;
		if (length == 8 && data[0] == 0 && data[1] == 8 &&
		    data[2] == 0 && data[5] == 0 && data[6] == 0 &&
		    data[7] == 0) {
			g_xi.rumbleLow = (uint16_t)data[3] * 257u;
			g_xi.rumbleHigh = (uint16_t)data[4] * 257u;
			g_xi.rumbleMs = millis();
			g_xi.rumbleBond = g_xi.bond;
			g_xi.rumblePending = true;
		} else if (length == 3 && data[0] == 1 && data[1] == 3) {
			// 2..5 blink then settle; 6..9 immediately select a player.
			uint8_t led = data[2];
			g_xi.player = led >= 2 && led <= 5 ? led - 1 :
				      led >= 6 && led <= 9 ? led - 5 :
							     0;
		}
	}
	g_xi.outRearm =
		!usbd_edpt_xfer(rhport, ep, g_xi.outBuf, sizeof g_xi.outBuf);
	return true;
}

static const usbd_class_driver_t g_xiDriver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "XINPUT",
#endif
	.init = xi_init,
	.deinit = xi_deinit,
	.reset = xi_reset,
	.open = xi_open,
	.control_xfer_cb = xi_ctrl,
	.xfer_cb = xi_xfer,
	.sof = NULL
};

const usbd_class_driver_t *xinputClassDriver(void)
{
	return &g_xiDriver;
}

class Adafruit_USBD_XInput : public Adafruit_USBD_Interface {
    public:
	uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf,
					uint16_t bufsize) override
	{
		if (!buf)
			return sizeof XINPUT_CONFIG_BODY;
		if (itfnum != 0 || bufsize < sizeof XINPUT_CONFIG_BODY)
			return 0;
		TinyUSBDevice.allocInterface(4);
		// Reserve through endpoint 6 in each direction, including holes.
		// No other USB interfaces may be added to this personality.
		for (uint8_t ep = 1; ep <= 6; ep++) {
			TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
			TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		}
		memcpy(buf, XINPUT_CONFIG_BODY, sizeof XINPUT_CONFIG_BODY);
		buf[XINPUT_SECURITY_STRING_OFFSET] = _strid;
		return sizeof XINPUT_CONFIG_BODY;
	}
	bool begin()
	{
		setStringDescriptor(XINPUT_SECURITY_STRING);
		return TinyUSBDevice.addInterface(*this);
	}
};

static Adafruit_USBD_XInput g_xinput;

static void xinputSend(uint8_t slot, uint32_t generation, uint16_t buttons,
		       uint8_t lt, uint8_t rt, int16_t lx, int16_t ly,
		       int16_t rx, int16_t ry)
{
	uint8_t report[20] = { 0, 20 };
	report[2] = buttons & 0xFF;
	report[3] = buttons >> 8;
	report[4] = lt;
	report[5] = rt;
	report[6] = lx & 0xFF;
	report[7] = lx >> 8;
	report[8] = ly & 0xFF;
	report[9] = ly >> 8;
	report[10] = rx & 0xFF;
	report[11] = rx >> 8;
	report[12] = ry & 0xFF;
	report[13] = ry >> 8;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (g_xiGeneration == generation && g_xi.bond == slot) {
		memcpy(g_xi.latest, report, sizeof report);
		g_xi.txPending = true;
	}
	__set_PRIMASK(pm);
}

// usbTxPump calls this from the loop with the USB priority-inversion guard.
static void xiTxDrain(void)
{
	if (!tud_mounted() || !g_xi.inUse)
		return;
	uint32_t generation = g_xiGeneration;
	uint8_t rhport = g_xi.rhport;
	if (g_xi.outRearm && !usbd_edpt_busy(rhport, 0x02) &&
	    usbd_edpt_claim(rhport, 0x02)) {
		g_xi.outRearm = false;
		if (!usbd_edpt_xfer(rhport, 0x02, g_xi.outBuf,
				    sizeof g_xi.outBuf)) {
			if (generation == g_xiGeneration)
				g_xi.outRearm = true;
			usbd_edpt_release(rhport, 0x02);
		}
	}
	if (generation != g_xiGeneration || !g_xi.inUse ||
	    (!g_xi.txPending && !g_xi.neutralPending) ||
	    usbd_edpt_busy(rhport, 0x81) || !usbd_edpt_claim(rhport, 0x81))
		return;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (generation != g_xiGeneration) {
		__set_PRIMASK(pm);
		usbd_edpt_release(rhport, 0x81);
		return;
	}
	bool neutral = g_xi.neutralPending;
	if (neutral) {
		xiNeutral(g_xi.inBuf);
		g_xi.neutralPending = false;
	} else {
		memcpy(g_xi.inBuf, g_xi.latest, sizeof g_xi.latest);
		g_xi.txPending = false;
	}
	__set_PRIMASK(pm);
	if (!usbd_edpt_xfer(rhport, 0x81, g_xi.inBuf, 20)) {
		pm = __get_PRIMASK();
		__disable_irq();
		if (generation == g_xiGeneration) {
			if (neutral)
				g_xi.neutralPending = true;
			else
				g_xi.txPending = true;
		}
		__set_PRIMASK(pm);
		usbd_edpt_release(rhport, 0x81);
	}
}

static uint16_t codeToXB(uint8_t c)
{
	switch (c) {
	case 1:
		return XB_A;
	case 2:
		return XB_B;
	case 3:
		return XB_X;
	case 4:
		return XB_Y;
	case 5:
		return XB_LB;
	case 6:
		return XB_RB;
	case 7:
		return XB_L3;
	case 8:
		return XB_R3;
	case 9:
		return XB_BACK;
	case 10:
		return XB_START;
	case 11:
		return XB_GUIDE;
	case 12:
		return XB_DUP;
	case 13:
		return XB_DDOWN;
	case 14:
		return XB_DLEFT;
	case 15:
		return XB_DRIGHT;
	default:
		return 0;
	}
}

static void rfXboxGamepad(uint8_t slot, uint32_t generation, const uint8_t *r)
{
	uint32_t b = btnsOf(r);
	if (g_qamMap && (b & TB_QAM)) {
		b &= ~(uint32_t)TB_QAM;
		b |= tritonFromCode(g_qamMap);
	}
	uint16_t btn = 0;
	if (b & TB_DUP)
		btn |= XB_DUP;
	if (b & TB_DDN)
		btn |= XB_DDOWN;
	if (b & TB_DLF)
		btn |= XB_DLEFT;
	if (b & TB_DRT)
		btn |= XB_DRIGHT;
	// TB_VIEW is the physical Menu/Start side; see triton.h.
	if (b & TB_VIEW)
		btn |= XB_START;
	if (b & TB_MENU)
		btn |= XB_BACK;
	if (b & TB_STEAM)
		btn |= XB_GUIDE;
	if (b & TB_LB)
		btn |= XB_LB;
	if (b & TB_RB)
		btn |= XB_RB;
	if (b & TB_L3)
		btn |= XB_L3;
	if (b & TB_R3)
		btn |= XB_R3;
	uint16_t fA = g_abSwap ? XB_B : XB_A, fB = g_abSwap ? XB_A : XB_B,
		 fX = g_abSwap ? XB_Y : XB_X, fY = g_abSwap ? XB_X : XB_Y;
	if (b & TB_A)
		btn |= fA;
	if (b & TB_B)
		btn |= fB;
	if (b & TB_X)
		btn |= fX;
	if (b & TB_Y)
		btn |= fY;
	if (b & TB_L4)
		btn |= codeToXB(g_back[0]);
	if (b & TB_R4)
		btn |= codeToXB(g_back[1]);
	if (b & TB_L5)
		btn |= codeToXB(g_back[2]);
	if (b & TB_R5)
		btn |= codeToXB(g_back[3]);
	uint8_t lt = trigU8(u16off(r, 4)), rt = trigU8(u16off(r, 6));
	// Remapped triggers pull the analog byte full; they have no button bit.
	if (b & TB_L2)
		lt = 0xFF;
	if (b & TB_R2)
		rt = 0xFF;
	const uint8_t bc[4] = { g_back[0], g_back[1], g_back[2], g_back[3] };
	const uint32_t bm[4] = { TB_L4, TB_R4, TB_L5, TB_R5 };
	for (int i = 0; i < 4; i++) {
		if (!(b & bm[i]))
			continue;
		if (bc[i] == 19)
			lt = 0xFF;
		else if (bc[i] == 20)
			rt = 0xFF;
	}
	int16_t lx = (int16_t)s16off(r, 8), ly = (int16_t)s16off(r, 10),
		rx = (int16_t)s16off(r, 12), ry = (int16_t)s16off(r, 14);
	padStickBlend(b, (int16_t)s16off(r, 16), (int16_t)s16off(r, 18),
		      (int16_t)s16off(r, 22), (int16_t)s16off(r, 24), &lx, &ly,
		      &rx, &ry);
	xinputSend(slot, generation, btn, lt, rt, lx, ly, rx, ry);
}

static bool xiBondAlive(int slot, unsigned long now)
{
	return slot >= 0 && slot < NSLOT && g_slot[slot].used &&
	       g_connReplyMs[slot] &&
	       now - g_connReplyMs[slot] <= XINPUT_CONN_MS;
}

static void xiDisconnect(unsigned long now)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	int slot = g_xi.bond;
	if (slot >= 0 && !xiBondAlive(slot, now)) {
		g_stopBonds |= 1u << slot;
		g_xi.bond = g_xi.rumbleBond = -1;
		g_xi.rumbleLow = g_xi.rumbleHigh = 0;
		g_xi.rumbleMs = 0;
		g_xi.rumblePending = false;
		g_xi.txPending = false;
		g_xi.neutralPending = true;
		xiNeutral(g_xi.latest);
	}
	__set_PRIMASK(pm);
}

void XboxController::usbIdentity()
{
	USBDevice.setID(0x045E, 0x028E);
	USBDevice.setVersion(0x0200);
	USBDevice.setDeviceVersion(0x0114);
	USBDevice.setManufacturerDescriptor("\xc2\xa9Microsoft Corporation");
	USBDevice.setProductDescriptor("Controller");
	USBDevice.setSerialDescriptor(xinputAuthSerial());
	USBDevice.setConfigurationAttribute(0xA0);
	USBDevice.setConfigurationMaxPower(500);
	// Adafruit exposes no class setter; its callback returns mutable backing
	// storage. Identity is set only while detached, before enumeration.
	tusb_desc_device_t *device =
		(tusb_desc_device_t *)tud_descriptor_device_cb();
	device->bDeviceClass = 0xFF;
	device->bDeviceSubClass = 0xFF;
	device->bDeviceProtocol = 0xFF;
}

void XboxController::begin()
{
	xiClear();
	xinputAuthPrepare();
	usbIdentity();
	g_xinput.begin();
	usbTxRegisterDrain(xiTxDrain);
}

void XboxController::onReport45(int slot, const uint8_t *rep, bool fresh,
				uint8_t bodyTlen)
{
	if (slot < 0 || slot >= NSLOT || !rep || bodyTlen < 26 ||
	    (rep[0] != 0x45 && rep[0] != 0x42))
		return;
	unsigned long now = millis();
	xiDisconnect(now);
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	// rf_link also injects a synthetic neutral after 300ms of silence; it
	// may release input, but must never acquire an inactive RF destination.
	if (g_xi.inUse && g_xi.bond < 0 && fresh && xiBondAlive(slot, now) &&
	    now - g_connReplyMs[slot] < 300u) {
		g_xi.bond = slot;
		g_xi.rumblePending = false;
		g_xi.rumbleBond = -1;
		g_xi.rumbleLow = g_xi.rumbleHigh = 0;
	}
	bool selected = g_xi.bond == slot;
	uint32_t generation = g_xiGeneration;
	__set_PRIMASK(pm);
	if (selected)
		rfXboxGamepad((uint8_t)slot, generation, rep);
}

void XboxController::task()
{
	unsigned long now = millis();
	xiDisconnect(now);
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t stops = g_stopBonds;
	g_stopBonds = 0;
	int slot = g_xi.bond;
	bool pending = g_xi.rumblePending && g_xi.rumbleBond == slot;
	uint16_t low = g_xi.rumbleLow, high = g_xi.rumbleHigh;
	if (!g_rumble || now - g_xi.rumbleMs > RUMBLE_STUCK_MS) {
		pending |= low || high;
		low = high = 0;
		g_xi.rumbleLow = g_xi.rumbleHigh = 0;
	}
	g_xi.rumblePending = false;
	uint32_t generation = g_xiGeneration;
	__set_PRIMASK(pm);
	for (uint8_t s = 0; s < NSLOT; s++) {
		if (stops & (1u << s))
			hapticSteamRumble(0, 0, s);
	}
	if (pending && xiBondAlive(slot, now) && g_xiGeneration == generation &&
	    g_xi.bond == slot)
		hapticSteamRumble(low, high, (uint8_t)slot);
	xinputAuthYield();
}
