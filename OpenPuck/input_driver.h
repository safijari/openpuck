// input_driver.h -- the input-side controller abstraction (the mirror of controllers.h's IController).
//
// controllers.h answers "how do we PRESENT input to the host" (one IController per USB personality). This
// header answers "where does input COME FROM and how is it normalized". The contract in the middle is
// PuckInput/g_in[slot] (triton.h): every input source decodes its native reports into g_in[slot] and stamps
// g_connReplyMs[slot], and every USB personality reads g_in -- so a new input source lights up ALL of the
// existing output modes (XInput, Switch, PS5, DS4...) with no per-mode work.
//
// Sources today:
//   - the SC2 RF link (rf_link.cpp) -- decodes report 0x45. Not an IInputDriver; it predates this seam and
//     stays as-is.
//   - BLE HID controllers (ble_host.cpp) -- each connected device is matched to an IInputDriver that knows its
//     report layout. ble_host owns the transport (scan/pair/bond/subscribe); the driver owns byte meaning.
//
// Slot model: the four bond slots (bonds.h) are shared. RF bonds own the slots marked g_slot[s].used; a BLE
// controller claims a FREE slot at connect (highest index first, so RF pairing -- which fills from interface 0
// -- collides last) and releases it at disconnect. g_slotSrc tags who drives each slot so the RF poll, the
// haptic relay, and usb_mount route correctly.
//
// Adding a driver: subclass IInputDriver in a new drv_*.cpp, decode into the PuckInput fields you have (leave
// the rest alone -- absent sensors stay zero), map buttons onto the TB_* Triton masks (triton.h), and add the
// singleton to the registry in input_driver.cpp. See drv_xinput_ble.cpp for the exemplar.
#pragma once
#include <stdint.h>
#include "bonds.h" // NSLOT

struct PuckInput;

// ---- slot input-source tags ----
#define SRC_RF 0 // SC2 over the ESB link (default; all RF-bonded slots)
#define SRC_BLE 1 // BLE HID controller (runtime claim by ble_host)
extern volatile uint8_t g_slotSrc[NSLOT];
static inline bool slotIsBle(int s)
{
	return s >= 0 && s < NSLOT && g_slotSrc[s] == SRC_BLE;
}

// ---- the driver interface ----
class IInputDriver {
    public:
	virtual ~IInputDriver()
	{
	}
	virtual const char *name() const = 0;
	// Claim a device by USB-IF ids (from the BLE Device Information Service PnP ID; 0/0 if absent) and/or
	// its advertised name (never NULL, possibly empty).
	virtual bool match(uint16_t vid, uint16_t pid,
			   const char *devName) const = 0;
	// The BLE HID client subscribes to the device's input-report characteristics in service-declaration
	// (attribute-handle) order but cannot read their Report Reference descriptors (no raw-descriptor API in
	// the BLE stack wrapper) -- so the driver declares which HID report id the i-th input characteristic
	// carries. Default: sequential ids starting at 1 (matches most single/dual-report gamepads).
	virtual uint8_t inputRidByIndex(uint8_t idx) const
	{
		return (uint8_t)(idx + 1);
	}
	// Decode one input report into io (the slot's live PuckInput). Update only the fields this report
	// carries -- io persists between reports, which is how multi-report devices (e.g. a separate guide-
	// button report) merge state. Return true if io changed (gates the host dispatch).
	virtual bool decode(int slot, uint8_t rid, const uint8_t *d,
			    uint16_t len, PuckInput *io) = 0;
	// Rumble: build the device's output report payload (WITHOUT the report id) for the given 16-bit
	// low/high-frequency motor amplitudes. Return the payload length, 0 = no rumble support.
	virtual uint8_t buildRumble(uint16_t lo, uint16_t hi, uint8_t out[16])
	{
		(void)lo;
		(void)hi;
		(void)out;
		return 0;
	}
};

// Registry lookup (input_driver.cpp). NULL = no driver claims this device (ble_host refuses the connection).
IInputDriver *inputDriverFor(uint16_t vid, uint16_t pid, const char *devName);
