#include "xinput_auth.h"

static uint32_t g_received;
static bool g_receiveOk;

bool xinputAuthReceived(uint16_t length)
{
	return g_receiveOk && g_received == length;
}

extern "C" bool __real_usbd_control_xfer_cb(uint8_t rhport, uint8_t ep,
					    xfer_result_t result,
					    uint32_t length);

extern "C" bool __wrap_usbd_control_xfer_cb(uint8_t rhport, uint8_t ep,
					    xfer_result_t result,
					    uint32_t length)
{
	// TinyUSB's DATA callback omits actual length and also runs for a short
	// OUT packet. XSM3 challenges fit in one EP0 packet (34/22 < 64).
	g_received = length;
	g_receiveOk = ep == 0 && result == XFER_RESULT_SUCCESS;
	bool ok = __real_usbd_control_xfer_cb(rhport, ep, result, length);
	g_receiveOk = false;
	return ok;
}