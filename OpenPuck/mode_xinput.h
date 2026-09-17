// Static wired Xbox 360 personality; one selected RF bond supplies input.
#pragma once
#include "controllers.h"
#include <stdint.h>

class XboxController : public IController {
    public:
	void begin() override;
	void onReport45(int slot, const uint8_t *rep, bool fresh,
			uint8_t bodyTlen) override;
	void task() override;
	bool dynamicMount() const override
	{
		return false;
	}
	void usbIdentity() override;
};
extern XboxController g_xboxCtl;
