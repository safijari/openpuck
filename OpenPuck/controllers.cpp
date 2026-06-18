#include "controllers.h"
#include "config.h"
#include "puck_hid.h"
#include "mode_xinput.h"
#include "mode_switch_hori.h"
#include "mode_switch_pro.h"
#include "mode_ps5.h"
#include "mode_hidgyro.h"

IController *g_active = nullptr;

// Map a USB-presentation mode to its singleton controller. Steam and Lizard share the puck controller (the
// lizard-vs-forward decision is made per-report inside it).
IController *controllerFor(uint8_t mode)
{
	switch (mode) {
	case MODE_STEAM:
		return &g_steamPuck;
	case MODE_LIZARD:
		return &g_steamPuck;
	case MODE_XBOX:
		return &g_xboxCtl;
	case MODE_SW_HORI:
		return &g_switchHori;
	case MODE_SW_PRO:
		return &g_switchPro;
	case MODE_PS5:
		return &g_ps5Ctl;
	case MODE_HIDGYRO:
		return &g_hidGyroCtl;
	case MODE_PS5_GAME:

		// same DualSense controller; setup() drops wake/WebUSB for clean enum
		return &g_ps5Ctl;
	case MODE_DS4_GAME:

		// same DS4 controller; setup() drops wake/WebUSB for clean enum
		return &g_hidGyroCtl;
	default:
		return &g_steamPuck;
	}
}
