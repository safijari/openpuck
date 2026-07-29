#include "usb_app_drivers.h"

extern "C" const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *count)
{
	static const usbd_class_driver_t drivers[] = {
		*xinputClassDriver(),
		*xboxOgClassDriver(),
	};

	*count = sizeof(drivers) / sizeof(drivers[0]);
	return drivers;
}
