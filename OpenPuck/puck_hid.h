// puck_hid.h -- the Steam Controller puck personality (MODE_STEAM + MODE_LIZARD).
//
// This is the controller that wears Valve's Proteus puck identity (28DE:1304) and exposes the four puck HID slot interfaces
// (interface N owns bond slot N). It implements:
//   - the puck feature COMMAND CHANNEL (handleSet/handleGet): reports 0x83/0xAE/0xB4/0xAD/0xA2/0xA3 the host
//     uses to read attributes/serials and read/write/clear bond slots, plus the host->controller haptic relay
//     ride-along on OUTPUT reports 0x80-0x86 and feature passthrough (report 0x01).
//   - the seamless LIZARD decision: when Steam is driving the gamepad (recent OUTPUT/heartbeat) we forward the
//     gamepad report 0x45; when Steam is closed we present keyboard+mouse on the SAME interface. MODE_LIZARD
//     forces lizard always. lizardActive() is the single source of truth, shared with the haptic gate so we
//     never relay haptics while presenting lizard (Steam isn't reading 0x45 back -> would buzz-loop).
//   - the USB connection-state presentation (reports 0x79 / 0x7B) Steam reads to mark the controller connected.
#pragma once
#include "controllers.h"
#include <stdint.h>

// Forward report 0x45 only when the seq advanced (dedupe like the real puck); sending stale
// repeats makes Steam's trackpad smoothing stair-step. (WebUSB field 14)
extern uint8_t g_fwdNewOnly;
// Content dedup: forward a 0x45/0x42 report only when its body EXCLUDING the free-running counter byte
// changed, so the delivered rate tracks the controller's real distinct-report rate (not the poll rate).
// On by default; console "CD" toggles for A/B. (see puck_hid.cpp for the why)
extern uint8_t g_fwdContentDedup;

// True while we are presenting the desktop keyboard/mouse instead of forwarding the gamepad to Steam
// (MODE_LIZARD always, or MODE_STEAM once Steam's heartbeat has stopped). Exposed so the haptic layer can
// tell "lizard is active" without duplicating the heartbeat logic -- the lizard-suppression keepalive must
// NOT run while lizard is active, or it disables the controller's autonomous touchpad haptic ticks.
bool puckLizardActive();

// Feature-command capture (diagnostic): when g_cmdCapture is on, Steam's USB feature SET/GET commands are
// logged to serial (# FC lines) from loop context and the high-rate I45 input stream is suppressed, so the
// connect handshake is readable. puckCmdLogDrain() must be called once per loop(). Console "FC" toggles it.
extern bool g_cmdCapture;
void puckCmdLogDrain(void);

// Drop Steam's relayed 0x81 CLEAR_DIGITAL_MAPPINGS in Steam mode (the amp-clicker in Steam's per-connect
// config; OpenPuck doesn't need it). On by default; console "S81" toggles for A/B.
extern bool g_drop81;

class SteamPuckController : public IController {
    public:
	void begin() override;
	void onReport45(int slot, const uint8_t *rep, bool fresh,
			uint8_t bodyTlen) override;
	void onAuxReport(int slot, uint8_t rid, const uint8_t *data, uint8_t n)
		override; // forward controller 0x43/0x44 status to Steam
	void task() override;

	void wakeEvent() override;
	bool isPuck() const override
	{
		return true;
	}
};
extern SteamPuckController g_steamPuck;
