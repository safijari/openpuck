// mode_xbox_og.h -- Original Xbox Controller S XID personality.
#pragma once

#include "controllers.h"

class XboxOgController : public IController {
    public:
	void begin() override;
	void onReport45(int slot, const uint8_t *rep, bool fresh,
			uint8_t bodyTlen) override;
	void task() override;
	void usbIdentity() override;
};

extern XboxOgController g_xboxOgCtl;
