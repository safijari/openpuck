// usb_app_drivers.h -- internal registry for custom TinyUSB class drivers.
#pragma once

extern "C" {
#include "device/usbd_pvt.h"
}

const usbd_class_driver_t *xinputClassDriver(void);
const usbd_class_driver_t *xboxOgClassDriver(void);
