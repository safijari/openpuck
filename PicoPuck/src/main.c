// PicoPuck main — core0 cooperative loop.
//
// Phase 1: bring up the puck USB identity (four HID slots + WebUSB) and answer
// Steam's command channel with no controllers connected. Bluetooth (BTstack
// host) is layered on in later phases; the loop already reserves its slot.
//
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "config/picopuck_config.h"
#include "puck/identity.h"
#include "puck/slots.h"
#include "puck/personality.h"
#include "usb/usb_tx.h"
#include "usb/webusb.h"
#include "bt/bt_host.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"
#include "tusb.h"

int main(void)
{
	stdio_init_all();

	identity_init();
	slots_init();
	puck_personality_init();
	webusb_init();

	// USB device stack (puck HID slots + WebUSB vendor interface).
	tud_init(0);

	// The user LED hangs off the CYW43 chip; this init also hands the radio to
	// BTstack in a later phase (btstack_cyw43_init on the async_context).
	bool have_radio = (cyw43_arch_init() == 0);

	// Bring up the Bluetooth host on the radio (dual-mode Classic + BLE).
	bool have_bt = have_radio && bt_host_init();

	printf("\n[picopuck] boot commit=%s board=%d radio=%d bt=%d\n",
	       PICOPUCK_GIT_COMMIT, PP_BOARD, have_radio, have_bt);

	watchdog_enable(PP_WATCHDOG_MS, /*pause_on_debug=*/true);

	absolute_time_t next_led = make_timeout_time_ms(PP_STATUS_LED_MS);
	bool led_on = false;

	while (1) {
		tud_task();               // USB device events
		usb_tx_pump();            // drain queued HID reports
		puck_personality_task();  // 0x79 / 0x7B / 0x43 status
		webusb_task();            // panel commands

		if (have_bt) {
			cyw43_arch_poll();  // services BTstack on its async_context
			bt_host_task();     // scan timeout + rumble flush
		} else if (have_radio) {
			cyw43_arch_poll();
		}

		if (absolute_time_diff_us(get_absolute_time(), next_led) <= 0) {
			led_on = !led_on;
			if (have_radio)
				cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
			next_led = make_timeout_time_ms(PP_STATUS_LED_MS);
		}

		watchdog_update();
	}

	return 0;
}
