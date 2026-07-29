#include "webusb_ctrl.h"
#include "ctrl_bonds.h"
#include "ctrl_link.h" // g_linkSlot, ctrlLinkUp()
#include "deck_input.h" // deckForwarding()
#include "fw_update.h"
#include <Arduino.h>
#include <string.h>

Adafruit_USBD_WebUSB usb_web;

// The bond-list (0xAC) frame can carry all NBOND=8 pucks: 5-byte header + 8*26 = 213 B. It must fit the
// vendor TX FIFO whole (we send drop-on-full, never spin), so the build sets CFG_TUD_VENDOR_TX_BUFSIZE=256.
// Guard it so a build without that flag fails loudly instead of shipping a dongle whose panel never lists a
// puck. (The core default is 64 -- see arduino/ports/nrf/tusb_config_nrf.h.)
static_assert(
	CFG_TUD_VENDOR_TX_BUFSIZE >= 5 + NBOND * 26,
	"CFG_TUD_VENDOR_TX_BUFSIZE too small for the 0xAC bond list -- build "
	"the dongle with -DCFG_TUD_VENDOR_TX_BUFSIZE=256 (see Makefile).");

// ---- deferred replies (both sent from webusbCtrlPoll, gated on FIFO room so usb_web.write() never spins) ----
// usb_web.write() SPINS until the IN FIFO drains or the panel disconnects; a panel that holds the interface
// open but stops reading would hang loop() -> watchdog reset. So EVERY send here is gated on
// tud_vendor_write_available(): the bond list is drop-on-full (the panel re-requests), the fw-update ack is
// kept pending until there is room (the panel's strict ping-pong drains the IN pipe, so that is immediate).
static volatile bool g_bondListPend = false;
static volatile bool g_fwupAckPend = false;
static volatile uint8_t g_fwupAckStatus = 0;
static volatile uint32_t g_fwupAckOff = 0;

static void fwupAckPost(uint8_t status)
{
	g_fwupAckStatus = status;
	g_fwupAckOff = fwupNextOff();
	g_fwupAckPend = true;
}

// Build + send the paired-pucks list. Mirrors deckStatusTask()'s per-bond record so the two management
// surfaces (the Deck app over CDC, the browser panel over WebUSB) show the same thing.
static void sendBondList()
{
	if (!usb_web.connected())
		return;
	static uint8_t p[5 + NBOND * 26];
	bool linkUp = ctrlLinkUp();
	p[0] = 0xAC;
	p[2] = 1; // format version
	p[3] = (uint8_t)((deckForwarding() ? 1 : 0) | (linkUp ? 2 : 0));
	uint8_t count = 0;
	uint8_t *q = p + 5;
	for (int i = 0; i < NBOND; i++) {
		if (!g_bond[i].used)
			continue;
		q[0] = (uint8_t)
			i; // firmware slot index (target for 0x30 remove)
		q[1] = (i == g_linkSlot && linkUp) ? 1 : 0; // alive
		memcpy(q + 2, g_bond[i].rec + 0, 4); // proteus_uuid
		memcpy(q + 6, g_bond[i].rec + 4, 4); // ibex_uuid
		memcpy(q + 10, g_bond[i].rec + 8, 16); // puck serial
		q += 26;
		count++;
	}
	p[4] = count;
	uint16_t frame = (uint16_t)(5 + count * 26);
	p[1] = (uint8_t)(3 + count * 26); // payload length after [marker][len]
	if (tud_vendor_write_available() >= frame) {
		usb_web.write(p, frame);
		usb_web.flush();
		g_bondListPend = false;
	} else {
		g_bondListPend = true; // no room now -> retry next poll
	}
}

static void flushDeferred()
{
	if (g_fwupAckPend && tud_vendor_write_available() >= 7) {
		uint8_t f[7] = { 0xAB,
				 5,
				 g_fwupAckStatus,
				 (uint8_t)(g_fwupAckOff),
				 (uint8_t)(g_fwupAckOff >> 8),
				 (uint8_t)(g_fwupAckOff >> 16),
				 (uint8_t)(g_fwupAckOff >> 24) };
		usb_web.write(f, sizeof f);
		usb_web.flush();
		g_fwupAckPend = false;
	}
	if (g_bondListPend)
		sendBondList();
}

void webusbCtrlBegin()
{
	usb_web.begin(); // adds the vendor interface + bumps bcdUSB to 2.1; no landing page (no Chrome popup)
}

void webusbCtrlPoll()
{
	flushDeferred();
	if (!usb_web.connected())
		return;

	// 134 B holds the largest command: a firmware-update data chunk 0x21 = [op][off u32][len][<=128 data].
	// Byte-wise accumulation reassembles a command that spans USB-FS packets; a bad byte costs a 1-byte resync.
	static uint8_t buf[160];
	static uint8_t n = 0;
	while (usb_web.available()) {
		int c = usb_web.read();
		if (c < 0)
			break;
		if (n < sizeof buf)
			buf[n++] = (uint8_t)c;
		for (;;) {
			if (n == 0)
				break;
			uint8_t op = buf[0];
			// valid opcodes: 0x01, 0x0B, 0x0C, 0x20..0x25, 0x30. Anything else -> drop one byte + resync.
			bool known = (op == 0x01 || op == 0x0B || op == 0x0C ||
				      op == 0x30 || (op >= 0x20 && op <= 0x25));
			if (!known) {
				memmove(buf, buf + 1, --n);
				continue;
			}
			// 0x21 carries its own data length at [5]; sanity-gate it before trusting it as a length so a
			// corrupt byte can only cost a one-byte resync, never wedge the parser.
			if (op == 0x21 && n >= 6) {
				uint8_t dl = buf[5];
				if (dl == 0 || dl > 128 || (dl & 3)) {
					memmove(buf, buf + 1, --n);
					continue;
				}
			}
			uint8_t need =
				(op == 0x20) ?
					9 :
				(op == 0x21) ?
					(uint8_t)(6 + (n >= 6 ? buf[5] : 0)) :
				(op == 0x25) ? 5 :
				(op == 0x30) ? 2 :
					       1;
			if (n < need)
				break; // wait for more bytes

			if (op == 0x01) {
				sendBondList();
			} else if (op == 0x30) {
				// un-bond one paired puck by firmware slot, persist, then re-report the list.
				ctrlBondClear(buf[1]);
				if (g_bondDirty) {
					g_bondDirty = false;
					saveCtrlBonds();
				}
				sendBondList();
			} else if (op == 0x0B) {
				usb_web.flush();
				delay(40);
				enterSerialDfu();
			} else if (op == 0x0C) {
				usb_web.flush();
				delay(40);
				enterUf2Dfu();
			} else if (op == 0x20) {
				uint32_t sz = (uint32_t)buf[1] |
					      ((uint32_t)buf[2] << 8) |
					      ((uint32_t)buf[3] << 16) |
					      ((uint32_t)buf[4] << 24);
				uint32_t crc = (uint32_t)buf[5] |
					       ((uint32_t)buf[6] << 8) |
					       ((uint32_t)buf[7] << 16) |
					       ((uint32_t)buf[8] << 24);
				fwupAckPost(fwupBegin(sz, crc));
			} else if (op == 0x21) {
				uint32_t off = (uint32_t)buf[1] |
					       ((uint32_t)buf[2] << 8) |
					       ((uint32_t)buf[3] << 16) |
					       ((uint32_t)buf[4] << 24);
				fwupAckPost(fwupChunk(off, buf + 6, buf[5]));
			} else if (op == 0x22) {
				fwupAckPost(fwupEnd());
			} else if (op == 0x23) {
				// clean reboot. After a committed 0x22 the boot path applies the staged image;
				// unarmed it is just a normal restart.
				usb_web.flush();
				delay(40);
				NVIC_SystemReset();
			} else if (op == 0x24) {
				fwupAbort();
				fwupAckPost(FWUP_OK);
			} else if (op == 0x25) {
				// FULL BOARD WIPE (debug). Guarded by the 4-byte magic "WIPE" so no stray byte can
				// trigger it. Arms the wipe marker + reboots; fwupWipeIfArmed() erases at next boot.
				if (buf[1] == 0x57 && buf[2] == 0x49 &&
				    buf[3] == 0x50 && buf[4] == 0x45) {
					usb_web.flush();
					fwupArmFullWipe();
					delay(40);
					NVIC_SystemReset();
				}
			}
			// consume the processed command from the front of buf
			if (n > need)
				memmove(buf, buf + need, n - need);
			n -= need;
		}
	}
}
