#include "mode_xinput.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>
// custom class-driver API for the XInput interface
extern "C" {
#include "device/usbd_pvt.h"
}

XboxController g_xboxCtl;

// XInput button bits
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

// ===================== XInput custom TinyUSB class driver =====================
// Custom class driver + an Adafruit_USBD_Interface subclass emitting the interface + 0x21-blob + 2 endpoints.
#define XINPUT_DESC_LEN \
	(9 + 17 + 7 + 7) // interface(9)+vendor0x21(17)+IN ep(7)+OUT ep(7) = 40
static uint8_t g_xiItf = 0xFF, g_xiEpIn = 0, g_xiEpOut = 0;
static uint8_t g_xiInBuf[32], g_xiOutBuf[32];

// XInput rumble strengths from the host's OUT packet; translated to Steam/Triton report 0x80.
static volatile uint16_t g_rumbleLow = 0, g_rumbleHigh = 0;
static volatile unsigned long g_rumbleMs =
	0; // millis of last rumble OUT packet (stuck-rumble watchdog)

// release a held rumble if no OUT packet refreshes it for this long (covers a lost stop)
#define RUMBLE_STUCK_MS 2500u

static void xi_init(void)
{
}
static bool xi_deinit(void)
{
	return true;
}
static void xi_reset(uint8_t rhport)
{
	(void)rhport;
	g_xiEpIn = g_xiEpOut = 0;
}
static uint16_t xi_open(uint8_t rhport, tusb_desc_interface_t const *itf,
			uint16_t max_len)
{
	if (!(itf->bInterfaceClass == 0xFF && itf->bInterfaceSubClass == 0x5D &&
	      itf->bInterfaceProtocol == 0x01))
		return 0;
	g_xiItf = itf->bInterfaceNumber;
	uint8_t const *p = (uint8_t const *)itf;
	uint8_t const *end = p + max_len;
	uint16_t used = itf->bLength;
	p += itf->bLength;
	uint8_t opened = 0;
	while (p < end && opened < itf->bNumEndpoints) {
		uint8_t blen = p[0], btype = p[1];
		if (btype == TUSB_DESC_ENDPOINT) {
			tusb_desc_endpoint_t const *ep =
				(tusb_desc_endpoint_t const *)p;
			usbd_edpt_open(rhport, ep);
			if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN)
				g_xiEpIn = ep->bEndpointAddress;
			else
				g_xiEpOut = ep->bEndpointAddress;
			opened++;
		}
		used += blen;
		p += blen;
	}
	if (g_xiEpOut)
		usbd_edpt_xfer(rhport, g_xiEpOut, g_xiOutBuf,
			       sizeof g_xiOutBuf); // arm OUT (rumble/LED)
	return used;
}
static bool xi_ctrl(uint8_t rhport, uint8_t stage,
		    tusb_control_request_t const *req)
{
	(void)rhport;
	(void)req;
	return stage != CONTROL_STAGE_SETUP;
}
static bool xi_xfer(uint8_t rhport, uint8_t ep, xfer_result_t res, uint32_t n)
{
	(void)res;
	if (ep == g_xiEpOut) {
		// XInput rumble packet: [00][08][00][bigMotor][smallMotor][00][00][00]; LED pkt is [01][03][led]
		if (n >= 5 && g_xiOutBuf[0] == 0x00 && g_xiOutBuf[1] == 0x08) {
			uint8_t big = g_xiOutBuf[3], sml = g_xiOutBuf[4];
			g_rumbleLow = (uint16_t)big * 257u;
			g_rumbleHigh = (uint16_t)sml * 257u;
			g_rumbleMs =
				millis(); // stamp for the stuck-rumble watchdog
			hapticSteamRumble(g_rumbleLow, g_rumbleHigh);
		}
		usbd_edpt_xfer(rhport, g_xiEpOut, g_xiOutBuf,
			       sizeof g_xiOutBuf); // re-arm OUT
	}
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
extern "C" const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *count)
{
	*count = 1;
	return &g_xiDriver;
}

class Adafruit_USBD_XInput : public Adafruit_USBD_Interface {
    public:
	uint16_t getInterfaceDescriptor(uint8_t, uint8_t *buf,
					uint16_t bufsize) override
	{
		if (!buf)
			return XINPUT_DESC_LEN;
		if (bufsize < XINPUT_DESC_LEN)
			return 0;
		uint8_t itfnum = TinyUSBDevice.allocInterface(1);
		uint8_t epin = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN),
			epout = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		const uint8_t t[XINPUT_DESC_LEN] = {
			9, TUSB_DESC_INTERFACE, itfnum, 0x00, 0x02, 0xFF, 0x5D,
			0x01, _strid, 0x11, 0x21, 0x00, 0x01, 0x01, 0x25, epin,
			0x14, 0x00, 0x00, 0x00, 0x00, 0x13, epout, 0x08, 0x00,
			0x00, 7, TUSB_DESC_ENDPOINT, epin, TUSB_XFER_INTERRUPT,
			U16_TO_U8S_LE(0x20),

			// bInterval 1ms (1000Hz) so the RF rate is the only limit
			1, 7, TUSB_DESC_ENDPOINT, epout, TUSB_XFER_INTERRUPT,
			U16_TO_U8S_LE(0x20), 8
		};
		memcpy(buf, t, XINPUT_DESC_LEN);
		return XINPUT_DESC_LEN;
	}
	bool begin()
	{
		return TinyUSBDevice.addInterface(*this);
	}
};
static Adafruit_USBD_XInput g_xinput;
static void xinputSend(uint16_t buttons, uint8_t lt, uint8_t rt, int16_t lx,
		       int16_t ly, int16_t rx, int16_t ry)
{
	if (!tud_mounted() || g_xiEpIn == 0 || usbd_edpt_busy(0, g_xiEpIn))
		return;
	uint8_t *r = g_xiInBuf;
	r[0] = 0x00;
	r[1] = 0x14;
	r[2] = buttons & 0xFF;
	r[3] = buttons >> 8;
	r[4] = lt;
	r[5] = rt;
	r[6] = lx & 0xFF;
	r[7] = lx >> 8;
	r[8] = ly & 0xFF;
	r[9] = ly >> 8;
	r[10] = rx & 0xFF;
	r[11] = rx >> 8;
	r[12] = ry & 0xFF;
	r[13] = ry >> 8;
	memset(r + 14, 0, 6);
	if (usbd_edpt_claim(0, g_xiEpIn)) {
		if (!usbd_edpt_xfer(0, g_xiEpIn, r, 20))
			usbd_edpt_release(0, g_xiEpIn);
	}
}

// ===================== right-pad mouse interface =====================
static const uint8_t MOUSE_HID_DESC[] = { TUD_HID_REPORT_DESC_MOUSE() };
static Adafruit_USBD_HID g_mouse; // Xbox-mode mouse interface (right trackpad)

// ===================== report 0x45 -> XInput + mouse =====================
// button code (g_back[], g_abSwap targets) -> legacy XInput bit. 0=none 1=A 2=B 3=X 4=Y 5=LB 6=RB 7=L3 8=R3 9=Back 10=Start 11=Guide 12=Dup 13=Ddown 14=Dleft 15=Dright
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
static void rfXboxGamepad(const uint8_t *r)
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
	// face buttons, with optional A/B + X/Y swap (Nintendo layout)
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
	// back paddles -> configurable mapping (default L4->LB, R4->RB, L5->L3, R5->R3)
	if (b & TB_L4)
		btn |= codeToXB(g_back[0]);
	if (b & TB_R4)
		btn |= codeToXB(g_back[1]);
	if (b & TB_L5)
		btn |= codeToXB(g_back[2]);
	if (b & TB_R5)
		btn |= codeToXB(g_back[3]);
	uint8_t lt = trigU8(u16off(r, 4)),
		rt = trigU8(u16off(
			r, 6)); // triggers u16 (half-scale) -> full-range u8
	xinputSend(btn, lt, rt, (int16_t)s16off(r, 8),
		   (int16_t)s16off(r, 10), // L stick X/Y
		   (int16_t)s16off(r, 12),
		   (int16_t)s16off(r, 14)); // R stick X/Y
}
// Right pad -> mouse on a second HID-mouse interface alongside the XInput gamepad. Same glide model as Lizard's
// right pad; RPad click = left button, LPad click = right.
static void rfXboxMouse(const uint8_t *r)
{
	uint32_t b = btnsOf(r);
	static int prx = 0, pry = 0;
	static bool prt = false;
	static float vx = 0, vy = 0, rmx = 0, rmy = 0;
	static uint8_t pmb = 0;
	bool rtouch = b & TB_RPADT;
	int rx = s16off(r, 22), ry = s16off(r, 24);
	if (rtouch) {
		if (prt) {
			vx += (rx - prx);
			vy += (ry - pry);
		}
		prx = rx;
		pry = ry;
	}
	prt = rtouch;
	float mxf = vx / (float)(g_mDiv * 10) + rmx,
	      myf = -(vy / (float)(g_mDiv * 10)) +
		    rmy; // Y inverted for screen coords
	int dx = (int)mxf, dy = (int)myf;
	rmx = mxf - dx;
	rmy = myf - dy; // sub-pixel carry
	if (dx > 127)
		dx = 127;
	if (dx < -127)
		dx = -127;
	if (dy > 127)
		dy = 127;
	if (dy < -127)
		dy = -127;
	float f = g_mFric / 100.0f;
	vx *= f;
	vy *= f;
	if (vx > -1 && vx < 1)
		vx = 0;
	if (vy > -1 && vy < 1)
		vy = 0; // friction = glide/decay
	uint8_t mb = ((b & TB_RPADC) ? 1 : 0) |
		     ((b & TB_LPADC) ?
			      2 :
			      0); // RPad click = left, LPad click = right
	if (dx || dy || mb != pmb) {
		pmb = mb;
		hid_mouse_report_t m;
		m.buttons = mb;
		m.x = (int8_t)dx;
		m.y = (int8_t)dy;
		m.wheel = 0;
		m.pan = 0;
		if (g_mouse.ready())
			g_mouse.sendReport(0, &m, sizeof m);
	}
}

// ===================== IController =====================
void XboxController::begin()
{
	g_rumbleLow = g_rumbleHigh = 0;

	// 045E:028E -> Windows xusb / SDL / Linux xpad all bind it
	USBDevice.setID(0x045E, 0x028E);
	// Windows caches the config descriptor by VID:PID:bcdDevice, so any interface change here MUST bump bcdDevice
	// or Windows serves a stale descriptor.
	USBDevice.setDeviceVersion(0x0115);
	USBDevice.setManufacturerDescriptor("Microsoft");
	USBDevice.setProductDescriptor("Controller");
	g_xinput.setStringDescriptor("Controller");
	g_xinput.begin();
	g_mouse.setStringDescriptor("OpenPuck Mouse");
	g_mouse.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
	g_mouse.setReportDescriptor(MOUSE_HID_DESC, sizeof MOUSE_HID_DESC);
	g_mouse.setPollInterval(1);
	g_mouse.begin();
}
void XboxController::onReport45(const uint8_t *rep, bool fresh,
				uint8_t bodyTlen)
{
	(void)fresh;
	(void)bodyTlen;
	rfXboxGamepad(rep);
	rfXboxMouse(rep);
}
// Lost-stop watchdog: Steam/Triton rumble is latched, so force a zero report if the host stops refreshing.
void XboxController::task()
{
	if ((g_rumbleLow || g_rumbleHigh) &&
	    millis() - g_rumbleMs > RUMBLE_STUCK_MS) {
		g_rumbleLow = g_rumbleHigh = 0;
		hapticSteamRumble(0, 0);
	}
}
