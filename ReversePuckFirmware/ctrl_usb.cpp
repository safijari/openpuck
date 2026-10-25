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
// 0x95 reboots into wireless mode, which drops USB. If we reset before the host has read back the
// command's ACK, Steam sees the disconnect mid-transaction and reports "pairing failed" even though the
// bond is already on flash. So we wait for the host to GET the response (g_respRead), or REBOOT_GRACE_MS
// for a host that fires 0x95 without reading back (scpair.py does this), before resetting.
static volatile bool g_respRead = false;
static uint32_t g_rebootAt = 0;
static const uint32_t REBOOT_GRACE_MS = 400;

// Select bond slot from a Zephyr settings key string: "esb/bond" -> 0, "esb/bond_2" -> 1.
static int slotForKey(const char *key)
{
	if (strcmp(key, "esb/bond") == 0)
		return 0;
	if (strcmp(key, "esb/bond_2") == 0)
		return 1;
	return -1;
}

// Exact-slot key for pairing tools: "rp/bond/0" .. "rp/bond/7" -> 0..7, else -1. Unlike Steam's
// esb/bond[_2] writes (find-or-alloc by uuid), a write to one of these lands in exactly that slot.
static int rpSlotForKey(const char *key)
{
	if (strncmp(key, "rp/bond/", 8) != 0 || key[8] < '0' || key[9] != 0)
		return -1;
	int slot = key[8] - '0';
	return slot < NBOND ? slot : -1;
}

// Read-back slot for 0xED / 0xF0: an exact rp/bond/N key, or Steam's esb/bond (0) / esb/bond_2 (1).
static int anySlotForKey(const char *key)
{
	int slot = rpSlotForKey(key);
	return slot >= 0 ? slot : slotForKey(key);
}

// Identity marker read via 0xED "rp/info": ['R']['P'][protocol version][bond slot count]. A real
// controller answers an unknown key with an empty value, so this tells the two apart.
#define RP_INFO_VERSION 1

// Copy the NUL-terminated settings key at the start of a 0xED/0xEE/0xF0 payload into key[24].
// Returns the key length (the record, if any, starts after the NUL at pl[len]).
static uint16_t readKey(const uint8_t *pl, uint16_t pln, char *key)
{
	uint16_t k = 0;
	while (k < pln && k < 23 && pl[k] != 0)
		key[k] = (char)pl[k], k++;
	key[k] = 0;
	return k;
}

// 0xED reply [0xED][len][value] for the key in pl: a bond record, the rp/info marker, or len 0 when the
// slot is empty or the key unknown (as a real controller answers). out must hold >= 26. Returns length.
static uint8_t settingResp(const uint8_t *pl, uint16_t pln, uint8_t *out)
{
	char key[24];
	readKey(pl, pln, key);
	int sk = anySlotForKey(key);
	out[0] = 0xED;
	out[1] = 0;
	if (sk >= 0 && g_bond[sk].used) {
		out[1] = 24;
		memcpy(out + 2, g_bond[sk].rec, 24);
	} else if (strcmp(key, "rp/info") == 0) {
		out[1] = 4;
		out[2] = 'R';
		out[3] = 'P';
		out[4] = RP_INFO_VERSION;
		out[5] = NBOND;
	}
	return (uint8_t)(2 + out[1]);
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
	case 0xED: // keyed value read: ["key"\0] -> [0xED][len][value], len 0 when empty/unknown
		settingResp(pl, pln, g_resp);
		g_resp_len = 63;
		break;
	case 0xF0: { // keyed value delete: ["key"\0] -> clear that bond slot
		char key[24];
		readKey(pl, pln, key);
		ctrlBondClear(anySlotForKey(key));
		g_resp[0] = 0xF0;
		g_resp_len = 63;
		break;
	}
	case 0xEE: { // keyed bond write: ["key"\0][r1 4][r2 4][puck_serial 16]
		char key[24];
		uint16_t k = readKey(pl, pln, key);
		const uint8_t *rec = pl + k + 1; // skip the NUL
		uint16_t reclen =
			(pln > (uint16_t)(k + 1)) ? (uint16_t)(pln - k - 1) : 0;
		// rp/bond/N writes exactly slot N. Steam's keys find-or-allocate by the record's uuid key
		// rather than the Zephyr settings key, so we can bond an arbitrary number of pucks (not just
		// the two legacy esb/bond[_2] keys).
		int sk = rpSlotForKey(key);
		if (sk < 0)
			sk = (reclen >= 24) ? ctrlBondFindOrAlloc(rec) :
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
	// reboot into wireless mode (magic 0xA427AF52); the reset happens after the flash flush AND once the
	// host has read the ACK back (see g_respRead / REBOOT_GRACE_MS above)
	case 0x95:
		g_rebootPending = true;
		g_respRead = false;
		g_rebootAt = millis();
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
	// The host read the staged response -> the SET/GET transaction completed. Used to know a 0x95 ACK
	// landed before we drop USB, and as a serial-monitor trace of Steam's pairing read sequence (the SET
	// side is logged in handleSet; a bare GET with no preceding SET shows up here as a stale g_resp[0]).
	g_respRead = true;
	if (Serial.availableForWrite() > 40)
		Serial.printf("# GET rid=%u -> %02X %02X req=%u\n", rid,
			      g_resp[0], g_resp[1], (unsigned)reqlen);
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
	// reboot only AFTER the freshly-written bond is on flash (else it reloads a stale bond on boot) AND
	// the host has read the 0x95 ACK back (or the grace window lapsed for a fire-and-forget host) so the
	// USB drop never lands mid-transaction and reads to Steam as a pairing failure.
	if (g_rebootPending && !g_bondDirty &&
	    (g_respRead ||
	     (uint32_t)(millis() - g_rebootAt) > REBOOT_GRACE_MS)) {
		delay(20);
		NVIC_SystemReset();
	}
}
