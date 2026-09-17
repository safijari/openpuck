#include "xinput_auth.h"
#include "src/libxsm3/xsm3.h"
#include <string.h>

enum AuthState : uint8_t {
	AUTH_IDLE,
	AUTH_RECEIVE_INIT,
	AUTH_PENDING_INIT,
	AUTH_WORKING_INIT,
	AUTH_INIT_READY,
	AUTH_RECEIVE_VERIFY,
	AUTH_PENDING_VERIFY,
	AUTH_WORKING_VERIFY,
	AUTH_VERIFY_READY,
	AUTH_FAILED,
};

static volatile AuthState g_state = AUTH_IDLE;
static volatile uint32_t g_generation;
static bool g_workerReady;
static char g_serial[13];
static uint8_t g_identification[29];
static uint8_t g_challenge[34];
static uint8_t g_response[46];
static uint8_t g_responseLength;
// EP0 owns this buffer until DATA/ACK or the next SETUP. The worker never
// modifies it, even if a reset or new challenge arrives during crypto.
CFG_TUD_MEM_ALIGN static uint8_t g_control[46];

bool xinputAuthBusy()
{
	AuthState state = g_state;
	return state == AUTH_PENDING_INIT || state == AUTH_PENDING_VERIFY ||
	       state == AUTH_WORKING_INIT || state == AUTH_WORKING_VERIFY;
}

void xinputAuthReset()
{
	uint32_t mask = xinputAuthLock();
	g_generation++;
	g_state = AUTH_IDLE;
	g_responseLength = 0;
	memset(g_response, 0, sizeof g_response);
	xinputAuthUnlock(mask);
}

const char *xinputAuthSerial()
{
	return g_serial;
}

void xinputAuthPrepare()
{
	xinputAuthBoardSerial(g_serial);
	memcpy(g_identification, xsm3_id_data_ms_controller,
	       sizeof g_identification);
	memcpy(g_identification + 5, g_serial, 12);
	g_identification[28] = 0;
	for (unsigned i = 5; i < 28; i++)
		g_identification[28] ^= g_identification[i];
	xinputAuthReset();
	g_workerReady = xinputAuthStartWorker();
}

void xinputAuthProcess()
{
	uint8_t challenge[34];
	uint32_t mask = xinputAuthLock();
	AuthState state = g_state;
	if (state != AUTH_PENDING_INIT && state != AUTH_PENDING_VERIFY) {
		xinputAuthUnlock(mask);
		return;
	}
	uint32_t generation = g_generation;
	memcpy(challenge, g_challenge, sizeof challenge);
	g_state = state == AUTH_PENDING_INIT ? AUTH_WORKING_INIT :
					       AUTH_WORKING_VERIFY;
	xinputAuthUnlock(mask);

	static uint32_t cryptoGeneration;
	static bool initialized;
	bool ok;
	if (state == AUTH_PENDING_INIT) {
		xsm3_initialise_state();
		xsm3_set_identification_data(g_identification);
		ok = xsm3_do_challenge_init(challenge);
		initialized = ok;
		cryptoGeneration = generation;
	} else {
		ok = initialized && cryptoGeneration == generation &&
		     xsm3_do_challenge_verify(challenge);
		if (!ok)
			initialized = false;
	}

	mask = xinputAuthLock();
	if (generation == g_generation) {
		if (ok) {
			g_responseLength = state == AUTH_PENDING_INIT ? 46 : 22;
			memcpy(g_response, xsm3_challenge_response,
			       g_responseLength);
			g_state = state == AUTH_PENDING_INIT ?
					  AUTH_INIT_READY :
					  AUTH_VERIFY_READY;
		} else {
			g_responseLength = 0;
			memset(g_response, 0, sizeof g_response);
			g_state = AUTH_FAILED;
		}
	}
	xinputAuthUnlock(mask);
}

static bool authRecipient(const tusb_control_request_t *request)
{
	uint8_t type = request->bmRequestType & 0x7F;
	// Retail requests use interface 3, sometimes with 0x01 in the high
	// byte of wIndex. Device-recipient requests have no interface index.
	return (type == 0x41 &&
		(request->wIndex == 3 || request->wIndex == 0x0103)) ||
	       (type == 0x40 && request->wIndex == 0);
}

bool xinputAuthControl(uint8_t rhport, uint8_t stage,
		       const tusb_control_request_t *request)
{
	if (!g_workerReady || !authRecipient(request))
		return false;
	bool input = request->bmRequestType & 0x80;
	uint8_t command = request->bRequest;
	// The console's OUT keepalive has no data stage. Acknowledge it without
	// resetting the session; subsequent verify challenges reuse its keys.
	if (!input && command == 0x84) {
		if (request->wLength != 0)
			return false;
		if (stage == CONTROL_STAGE_SETUP)
			return tud_control_xfer(rhport, request, g_control, 0);
		return true;
	}
	if (input) {
		if (command != 0x81 && command != 0x83 && command != 0x84 &&
		    command != 0x86)
			return false;
		if (stage != CONTROL_STAGE_SETUP)
			return true;
		uint16_t length = 0;
		uint32_t mask = xinputAuthLock();
		switch (command) {
		case 0x81:
			xinputAuthReset();
			memcpy(g_control, g_identification,
			       sizeof g_identification);
			length = sizeof g_identification;
			break;
		case 0x83:
			if (g_state != AUTH_INIT_READY &&
			    g_state != AUTH_VERIFY_READY) {
				xinputAuthUnlock(mask);
				return false;
			}
			length = g_responseLength;
			memcpy(g_control, g_response, length);
			break;
		case 0x86:
			if (g_state == AUTH_FAILED) {
				xinputAuthUnlock(mask);
				return false;
			}
			g_control[0] = (g_state == AUTH_INIT_READY ||
					g_state == AUTH_VERIFY_READY) ?
					       2 :
					       1;
			g_control[1] = 0;
			length = 2;
			break;
		case 0x84:
			break;
		}
		xinputAuthUnlock(mask);
		return tud_control_xfer(rhport, request, g_control, length);
	}

	bool init = command == 0x82;
	if ((!init && command != 0x87) || request->wLength != (init ? 34 : 22))
		return false;
	if (stage == CONTROL_STAGE_SETUP) {
		uint32_t mask = xinputAuthLock();
		if (!init && g_state != AUTH_INIT_READY &&
		    g_state != AUTH_VERIFY_READY) {
			xinputAuthUnlock(mask);
			return false;
		}
		if (init)
			xinputAuthReset();
		g_state = init ? AUTH_RECEIVE_INIT : AUTH_RECEIVE_VERIFY;
		g_responseLength = 0;
		memset(g_response, 0, sizeof g_response);
		memset(g_control, 0, sizeof g_control);
		xinputAuthUnlock(mask);
		return tud_control_xfer(rhport, request, g_control,
					request->wLength);
	}
	if (stage == CONTROL_STAGE_DATA) {
		uint32_t mask = xinputAuthLock();
		if (!xinputAuthReceived(request->wLength)) {
			g_generation++;
			g_state = AUTH_FAILED;
			xinputAuthUnlock(mask);
			return false;
		}
		if (g_state !=
		    (init ? AUTH_RECEIVE_INIT : AUTH_RECEIVE_VERIFY)) {
			xinputAuthUnlock(mask);
			return false;
		}
		memcpy(g_challenge, g_control, request->wLength);
		g_state = init ? AUTH_PENDING_INIT : AUTH_PENDING_VERIFY;
		xinputAuthUnlock(mask);
		xinputAuthNotify();
	}
	return true;
}