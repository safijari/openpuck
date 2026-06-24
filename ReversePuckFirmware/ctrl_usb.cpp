#include "ctrl_usb.h"
#include "ctrl_bonds.h"
#include "ctrl_link.h"
#include "identity.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

// Same Valve HID descriptor shape the puck uses (mouse 0x40 + keyboard 0x41 + vendor input reports +
// the 63-byte FEATURE command reports on id 1/2). The first collection is Generic Desktop (usagePage
// 0x01), so the host classifies this as the usagePage-0x01 control interface Steam/PairTUI write to.
static const uint8_t CTRL_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x40, 0x09, 0x01, 0xA1, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
	0x95, 0x02, 0x81, 0x02, 0x75, 0x06, 0x95, 0x01, 0x81, 0x01, 0x05, 0x01,
	0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x02,
	0x81, 0x06, 0x95, 0x01, 0x09, 0x38, 0x81, 0x06, 0x05, 0x0C, 0x0A, 0x38,
	0x02, 0x95, 0x01, 0x81, 0x06, 0xC0, 0xC0, 0x05, 0x01, 0x09, 0x06, 0xA1,
	0x01, 0x85, 0x41, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25,
	0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x81, 0x01, 0x19, 0x00, 0x29,
	0x65, 0x15, 0x00, 0x25, 0x65, 0x75, 0x08, 0x95, 0x06, 0x81, 0x00, 0xC0,
	0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x42, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x35, 0x09, 0x42, 0x81, 0x02, 0x85, 0x44,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x05, 0x09, 0x44, 0x81,
	0x02, 0x85, 0x79, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x01,
	0x09, 0x79, 0x81, 0x02, 0x85, 0x43, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x0E, 0x09, 0x43, 0x81, 0x02, 0x85, 0x7B, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x0C, 0x09, 0x7B, 0x81, 0x02, 0x85, 0x45,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x2D, 0x09, 0x45, 0x81,
	0x02, 0x85, 0x80, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x09,
	0x09, 0x80, 0x91, 0x02, 0x85, 0x81, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x07, 0x09, 0x81, 0x91, 0x02, 0x85, 0x82, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x03, 0x09, 0x82, 0x91, 0x02, 0x85, 0x83,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x09, 0x09, 0x83, 0x91,
	0x02, 0x85, 0x84, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x08,
	0x09, 0x84, 0x91, 0x02, 0x85, 0x85, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x03, 0x09, 0x85, 0x91, 0x02, 0x85, 0x86, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x03, 0x09, 0x86, 0x91, 0x02, 0x85, 0x87,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F, 0x09, 0x87, 0x91,
	0x02, 0x85, 0x89, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F,
	0x09, 0x89, 0x91, 0x02, 0x85, 0x88, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x3F, 0x09, 0x88, 0x91, 0x02, 0x85, 0x01, 0x95, 0x3F, 0x09,
	0x01, 0xB1, 0x02, 0x85, 0x02, 0x95, 0x3F, 0x09, 0x01, 0xB1, 0x02, 0xC0
};

static Adafruit_USBD_HID g_hid;
static uint8_t g_resp[63];
static uint16_t g_resp_len = 0;
static volatile bool g_rebootPending = false;

// Select bond slot from a Zephyr settings key string: "esb/bond" -> 0, "esb/bond_2" -> 1.
static int slotForKey(const char *key)
{
	if (strcmp(key, "esb/bond") == 0)
		return 0;
	if (strcmp(key, "esb/bond_2") == 0)
		return 1;
	return -1;
}

static void writeBond(int slot, const uint8_t *rec24)
{
	if (slot < 0 || slot >= NBOND)
		return;
	if (ctrlRecEmpty(rec24)) {
		g_bond[slot].used = false;
		memset(g_bond[slot].rec, 0, 24);
	} else {
		memcpy(g_bond[slot].rec, rec24, 24);
		g_bond[slot].used = true;
	}
	g_bondDirty = true;
}

// 0xAE string attribute by index, matching a real controller: 0=board serial, 1=unit serial (both our
// unique generated values), 3=the Valve constant "7054257d2da7" Steam checks, everything else "NA".
static const char *aeString(uint8_t idx)
{
	return (idx == 0) ? g_board :
	       (idx == 1) ? g_unit :
	       (idx == 3) ? "7054257d2da7" :
			    "NA";
}

// Build the controller's reply to a feature/command GET as [cmd][len][payload] -- the SAME shape a real
// controller returns over USB (`83 19 …`, `AE 14 idx …`), confirmed by scmd. The inner length byte IS
// required (these are command-channel replies, NOT the 0x45 input report). The puck relays this value
// straight back to Steam as the feature response. ctrl_link wraps it in the F1 type-6 TLV. Returns the
// byte count written to out (out must hold >= 63).
uint8_t ctrlFeatureResp(uint8_t cmd, const uint8_t *param, uint8_t plen,
			uint8_t *out)
{
	switch (cmd) {
	case 0x83: { // product/attribute blob: [0x83][len][ATTR83...]
		uint8_t alen = ATTR83_LEN > 58 ? 58 : (uint8_t)ATTR83_LEN;
		out[0] = 0x83;
		out[1] = alen;
		memcpy(out + 2, ATTR83, alen);
		return (uint8_t)(2 + alen);
	}
	case 0xAE: { // string attr: [0xAE][0x14][idx][serial 19]
		uint8_t idx = plen > 0 ? param[0] : 1;
		const char *s = aeString(idx);
		out[0] = 0xAE;
		out[1] = 0x14;
		out[2] = idx;
		memset(out + 3, 0, 0x13);
		memcpy(out + 3, s, strlen(s) < 0x13 ? strlen(s) : 0x13);
		return (uint8_t)(2 + 0x14);
	}
	case 0xB4: // wireless transport state: [0xB4][len=1][state]
		out[0] = 0xB4;
		out[1] = 0x01;
		out[2] = ctrlLinkUp() ? 0x02 : 0x01;
		return 3;
	default: // unknown query -> [cmd][len=0]
		out[0] = cmd;
		out[1] = 0;
		return 2;
	}
}

// Feature SET: framing [cmd][len][payload...]. rid (1 or 2) is not significant for our commands.
static void handleSet(uint8_t rid, hid_report_type_t type, uint8_t const *b,
		      uint16_t n)
{
	(void)rid;
	if (type != HID_REPORT_TYPE_FEATURE || n < 1)
		return;
	uint8_t cmd = b[0], len = (n > 1) ? b[1] : 0;
	const uint8_t *pl = b + 2;
	uint16_t pln = (n >= 2) ? n - 2 : 0;
	(void)len;

	// Plain-text diagnostics for a serial monitor during pairing. The Deck app resyncs on the frame
	// sync word, so these unframed lines are ignored by it -- only a human monitor sees them.
	if (Serial.availableForWrite() > 40) {
		Serial.printf("# SET rid=%u cmd=%02X len=%u pln=%u:", rid, cmd,
			      len, (unsigned)pln);
		for (uint16_t i = 0; i < n && i < 12; i++)
			Serial.printf(" %02X", b[i]);
		Serial.println();
	}

	memset(g_resp, 0, sizeof g_resp);
	g_resp_len = 0;
	switch (cmd) {
	case 0x83: { // product attributes (0x1302)
		uint16_t alen = ATTR83_LEN > 60 ? 60 : ATTR83_LEN;
		g_resp[0] = 0x83;
		g_resp[1] = (uint8_t)alen;
		memcpy(g_resp + 2, ATTR83, alen);
		g_resp_len = 63;
		break;
	}
	// string attributes: 0=board, 1=unit serial, 3="7054257d2da7"
	case 0xAE: {
		uint8_t idx = pln > 0 ? pl[0] : 1;
		const char *s = aeString(idx);
		g_resp[0] = 0xAE;
		g_resp[1] = 0x14;
		g_resp[2] = idx;
		memcpy(g_resp + 3, s, strlen(s));
		g_resp_len = 63;
		break;
	}
	case 0xB4: // wireless transport state (1 = ok)
		g_resp[0] = 0xB4;
		g_resp[1] = 0x01;
		g_resp[2] = ctrlLinkUp() ? 0x02 : 0x01;
		g_resp_len = 63;
		break;
	case 0xEE: { // keyed bond write: ["key"\0][r1 4][r2 4][puck_serial 16]
		char key[24];
		uint16_t k = 0;
		while (k < pln && k < sizeof(key) - 1 && pl[k] != 0)
			key[k] = (char)pl[k], k++;
		key[k] = 0;
		const uint8_t *rec = pl + k + 1; // skip the NUL
		uint16_t reclen =
			(pln > (uint16_t)(k + 1)) ? (uint16_t)(pln - k - 1) : 0;
		// Find-or-allocate by the record's uuid key rather than the Zephyr settings key, so we can
		// bond an arbitrary number of pucks (not just the two legacy esb/bond[_2] keys).
		int sk = (reclen >= 24) ? ctrlBondFindOrAlloc(rec) :
					  slotForKey(key);
		if (reclen >= 24)
			writeBond(sk, rec);
		if (Serial.availableForWrite() > 40)
			Serial.printf(
				"# EE key='%s' slot=%d reclen=%u stored=%d\n",
				key, sk, (unsigned)reclen,
				(sk >= 0 && reclen >= 24) ? 1 : 0);
		g_resp[0] = 0xEE;
		g_resp_len = 63;
		break;
	}
	case 0xEF: // commit keyed value (we write-through on 0xEE, so just ack)
		g_resp[0] = 0xEF;
		g_resp_len = 63;
		break;
	case 0xA2: // direct 24-byte bond write to slot 0 (fallback shape)
		if (pln >= 24)
			writeBond(0, pl);
		g_resp[0] = 0xA2;
		g_resp_len = 63;
		break;
	case 0xA3: // read slot 0 bond
		g_resp[0] = 0xA3;
		g_resp[1] = 0x18;
		if (g_bond[0].used)
			memcpy(g_resp + 2, g_bond[0].rec, 24);
		g_resp_len = 63;
		break;
	// reboot into wireless mode (magic 0xA427AF52); the reset happens after the flash flush
	case 0x95:
		g_rebootPending = true;
		if (Serial.availableForWrite() > 40)
			Serial.println("# 0x95 reboot-to-wireless requested");
		g_resp[0] = 0x95;
		g_resp_len = 63;
		break;
	default:
		g_resp[0] = cmd;
		g_resp[1] = len;
		if (pln)
			memcpy(g_resp + 2, pl, pln > 60 ? 60 : pln);
		g_resp_len = 63;
		break;
	}
}

static uint16_t handleGet(uint8_t rid, hid_report_type_t type, uint8_t *buf,
			  uint16_t reqlen)
{
	(void)rid;
	if (type != HID_REPORT_TYPE_FEATURE)
		return 0;
	uint16_t nn = g_resp_len ? g_resp_len : 63;
	if (nn > reqlen)
		nn = reqlen;
	memcpy(buf, g_resp, nn);
	return nn;
}

void ctrlUsbBegin()
{
	USBDevice.setID(0x28DE, 0x1302);
	USBDevice.setDeviceVersion(0x0101);
	USBDevice.setManufacturerDescriptor("Valve Software");
	USBDevice.setProductDescriptor("Steam Controller");
	g_hid.setReportDescriptor(CTRL_HID_DESC, sizeof CTRL_HID_DESC);
	g_hid.setReportCallback(handleGet, handleSet);
	g_hid.setPollInterval(1);
	g_hid.begin();
}

void ctrlUsbPoll()
{
	if (g_bondDirty) {
		g_bondDirty = false;
		saveCtrlBonds();
	}
	// reboot only AFTER the freshly-written bond is on flash (else it reloads a stale bond on boot).
	if (g_rebootPending && !g_bondDirty) {
		delay(40);
		NVIC_SystemReset();
	}
}
