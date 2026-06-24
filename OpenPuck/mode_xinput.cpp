// mode_xinput.cpp -- Xbox 360 wired (XInput) personality (MODE_XBOX): one XInput gamepad per bonded controller.
//
// Real Xbox 360 pads are NOT HID: they use a vendor interface (class 0xFF / sub 0x5D / proto 0x01) carrying a
// 20-byte XInput report. A custom TinyUSB class driver serves that interface plus a second boot-mouse interface
// for the right trackpad. Host binds by VID/PID (045E:028E) + the FF/5D/01 interface. The OUT endpoint carries
// rumble, relayed to the controller as a haptic by task().
//
// Multi-controller: one XInput interface per CONNECTED controller (dynamic mount -- see usb_mount.h; the device
// re-enumerates without rebooting as controllers connect/disconnect). The single class driver routes xfer/open
// callbacks by endpoint address into the per-bond-slot state; xi_open claims slots in connection order. The
// right-pad mouse is a single shared interface (only bond slot 0's right-pad drives it).
#include "mode_xinput.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_mount.h"
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
// Per-slot state lives in g_xiSlot[]: each XInput interface gets one. The class driver dispatches open/xfer
// by endpoint address; xinputSend() targets a specific slot's IN endpoint.
#define XINPUT_DESC_LEN \
	(9 + 17 + 7 + 7) // interface(9)+vendor0x21(17)+IN ep(7)+OUT ep(7) = 40

// per-slot state. `inUse` is set in xi_open from the USB ISR and read in xinputSend from the loop -- mark
// volatile so the loop doesn't observe a stale 0 across a freshly-attached interface.
struct XiSlot {
	uint8_t itf, epIn, epOut;
	uint8_t inBuf[32], outBuf[32];
	volatile uint16_t rumbleLow,
		rumbleHigh; // last host rumble values, x 257
	volatile unsigned long
		rumbleMs; // millis of last OUT packet (stuck-rumble watchdog)
	volatile bool inUse;
};
static XiSlot g_xiSlot[NSLOT];
// Dynamic mount: xi_open is called once per XInput interface in descriptor order; this counts them so the
// u-th interface claims the u-th connected controller (g_usbToBond[u]). Reset on every (re)enumeration.
static uint8_t g_xiOpenU = 0;
// release a held rumble if no OUT packet refreshes it for this long (covers a lost stop)
#define RUMBLE_STUCK_MS 2500u

static void xi_init(void)
{
	for (int s = 0; s < NSLOT; s++)
		g_xiSlot[s].inUse = false;
}
static bool xi_deinit(void)
{
	return true;
}
static void xi_reset(uint8_t rhport)
{
	(void)rhport;
	g_xiOpenU =
		0; // restart per-interface claim ordering for this enumeration
	for (int s = 0; s < NSLOT; s++) {
		g_xiSlot[s].itf = 0;
		g_xiSlot[s].epIn = g_xiSlot[s].epOut = 0;
		g_xiSlot[s].inUse = false;
	}
}
// TinyUSB calls xi_open once per XInput interface in the config descriptor. Claim the first free BONDED
// slot so the xiSlot index matches the bond-slot index (begin() registers them in ascending bond-slot order).
// Fallback to any free slot when there are no bonds (fresh device).
static uint16_t xi_open(uint8_t rhport, tusb_desc_interface_t const *itf,
			uint16_t max_len)
{
	if (!(itf->bInterfaceClass == 0xFF && itf->bInterfaceSubClass == 0x5D &&
	      itf->bInterfaceProtocol == 0x01))
		return 0;
	// The u-th XInput interface in the descriptor serves the u-th connected controller. g_xiSlot is keyed by
	// BOND slot so onReport45(bond)/rumble route straight through; only connected bonds get an interface.
	uint8_t u = g_xiOpenU++;
	int slot = (u < g_usbMountCount) ? g_usbToBond[u] : -1;
	if (slot < 0) { // fallback (e.g. map not built): first free slot
		for (int s = 0; s < NSLOT; s++)
			if (!g_xiSlot[s].inUse) {
				slot = s;
				break;
			}
	}
	if (slot < 0 || slot >= NSLOT || g_xiSlot[slot].inUse)
		return 0;
	XiSlot &S = g_xiSlot[slot];
	S.itf = itf->bInterfaceNumber;
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
				S.epIn = ep->bEndpointAddress;
			else
				S.epOut = ep->bEndpointAddress;
			opened++;
		}
		used += blen;
		p += blen;
	}
	S.inUse = true;
	// arm OUT (rumble/LED)
	if (S.epOut)
		usbd_edpt_xfer(rhport, S.epOut, S.outBuf, sizeof S.outBuf);
	return used;
}
static bool xi_ctrl(uint8_t rhport, uint8_t stage,
		    tusb_control_request_t const *req)
{
	(void)rhport;
	(void)req;
	return stage != CONTROL_STAGE_SETUP;
}
// Route an endpoint xfer callback to the slot that owns that endpoint.
static bool xi_xfer(uint8_t rhport, uint8_t ep, xfer_result_t res, uint32_t n)
{
	(void)res;
	for (int s = 0; s < NSLOT; s++) {
		XiSlot &S = g_xiSlot[s];
		if (!S.inUse)
			continue;
		if (ep == S.epOut) {
			// XInput rumble packet: [00][08][00][bigMotor][smallMotor][00][00][00]; LED pkt is [01][03][led]
			if (n >= 5 && S.outBuf[0] == 0x00 &&
			    S.outBuf[1] == 0x08) {
				uint8_t big = S.outBuf[3], sml = S.outBuf[4];
				uint16_t lo = (uint16_t)big * 257u;
				uint16_t hi = (uint16_t)sml * 257u;
				S.rumbleLow = lo;
				S.rumbleHigh = hi;
				S.rumbleMs = millis();
				hapticSteamRumble(lo, hi, (uint8_t)s);
			}
			usbd_edpt_xfer(rhport, S.epOut, S.outBuf,
				       sizeof S.outBuf);
			return true;
		}
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
// NSLOT instances: each registers its own XInput interface (own itfnum + IN/OUT ep pair) during begin().
// begin() in XboxController::begin() runs in order 0..NSLOT-1, so xi_open's first-free assignment lines up.
static Adafruit_USBD_XInput g_xinput[NSLOT];
static void xinputSend(uint8_t slot, uint16_t buttons, uint8_t lt, uint8_t rt,
		       int16_t lx, int16_t ly, int16_t rx, int16_t ry)
{
	if (slot >= NSLOT || !g_xiSlot[slot].inUse)
		return;
	XiSlot &S = g_xiSlot[slot];
	if (!tud_mounted() || S.epIn == 0 || usbd_edpt_busy(0, S.epIn))
		return;
	uint8_t *r = S.inBuf;
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
	if (usbd_edpt_claim(0, S.epIn)) {
		if (!usbd_edpt_xfer(0, S.epIn, r, 20))
			usbd_edpt_release(0, S.epIn);
	}
}

// ===================== right-pad mouse interface =====================
static const uint8_t MOUSE_HID_DESC[] = { TUD_HID_REPORT_DESC_MOUSE() };
static Adafruit_USBD_HID
	g_mouse; // Xbox-mode mouse interface (right trackpad, slot 0 only)

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
static void rfXboxGamepad(uint8_t slot, const uint8_t *r)
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
	// Trigger remaps (codes 19=LT, 20=RT): XInput triggers are analog bytes, not buttons, so a back paddle /
	// QAM mapped to a trigger pulls it full. QAM arrives as TB_L2/TB_R2 (folded into b via tritonFromCode);
	// back paddles are matched by their configured code.
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
	xinputSend(slot, btn, lt, rt, (int16_t)s16off(r, 8),
		   (int16_t)s16off(r, 10), // L stick X/Y
		   (int16_t)s16off(r, 12),
		   (int16_t)s16off(r, 14)); // R stick X/Y
}
// Right pad -> mouse on a second HID-mouse interface alongside the XInput gamepad. Same glide model as Lizard's
// right pad; RPad click = left button, LPad click = right. SINGLE shared mouse: the desktop can only consume
// one mouse. Slot 0's right-pad drives it; other slots' right-pad input is intentionally ignored here.
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
// Dynamic-mount mode: begin() is unused (setup() calls beginPool()+usbReenumerate instead).
void XboxController::begin()
{
}
// XInput slot interfaces are a custom (non-HID) class, so they don't draw on the CFG_TUD_HID budget (only the
// wake mouse + right-pad mouse do). Slots are limited by NSLOT / USB endpoints, not HID instances.
uint8_t XboxController::maxSlots() const
{
	return (uint8_t)NSLOT;
}
void XboxController::usbIdentity()
{
	// 045E:028E -> Windows xusb / SDL / Linux xpad all bind it
	USBDevice.setID(0x045E, 0x028E);
	USBDevice.setDeviceVersion(0x0120);
	USBDevice.setManufacturerDescriptor("Microsoft");
	USBDevice.setProductDescriptor("Controller");
}
void XboxController::beginPool()
{
	// Lock the right-pad mouse HID instance (wake mouse was begun first by setup -> this is HID instance 1).
	g_mouse.setStringDescriptor("OpenPuck Mouse");
	g_mouse.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
	g_mouse.setReportDescriptor(MOUSE_HID_DESC, sizeof MOUSE_HID_DESC);
	g_mouse.setPollInterval(1);
	g_mouse.begin();
	for (int s = 0; s < NSLOT; s++)
		g_xinput[s].setStringDescriptor("Controller");
}
void XboxController::mountSlots(uint8_t k)
{
	// Right-pad mouse first (fixed HID), then one XInput interface per connected controller (claimed in
	// order by xi_open). The wake mouse + WebUSB are added around this by usbReenumerate.
	USBDevice.addInterface(g_mouse);
	for (uint8_t u = 0; u < k; u++)
		USBDevice.addInterface(g_xinput[u]);
}
void XboxController::onReport45(int slot, const uint8_t *rep, bool fresh,
				uint8_t bodyTlen)
{
	(void)fresh;
	(void)bodyTlen;
	if (slot < 0 || slot >= NSLOT)
		return;
	rfXboxGamepad((uint8_t)slot, rep);
	// Shared desktop mouse: slot 0's right-pad only. Other slots' right-pad is still part of the
	// per-slot XInput (TB_RPADT in the gamepad report is unused today but kept for forward-compat).
	if (slot == 0)
		rfXboxMouse(rep);
}
// Lost-stop watchdog per slot: Steam/Triton rumble is latched, so force a zero report if the host stops
// refreshing on a particular XInput. The haptics.cpp global watchdog fires too, but it skips while the slot
// is in the post-reconnect block -- this catches a host that kept streaming while we were blocked.
void XboxController::task()
{
	for (int s = 0; s < NSLOT; s++) {
		if (!g_xiSlot[s].inUse)
			continue;
		if ((g_xiSlot[s].rumbleLow || g_xiSlot[s].rumbleHigh) &&
		    millis() - g_xiSlot[s].rumbleMs > RUMBLE_STUCK_MS) {
			g_xiSlot[s].rumbleLow = g_xiSlot[s].rumbleHigh = 0;
			hapticSteamRumble(0, 0, (uint8_t)s);
		}
	}
}
