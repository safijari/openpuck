# OpenPuck — Zephyr RTOS port.
#
# Migrated from the Arduino/Adafruit-nRF52/TinyUSB sketch in ../OpenPuck.
# Build (from a west workspace with this repo reachable):
#   west build -b adafruit_feather_nrf52840 OpenPuckZephyr
#
# The bare-metal RADIO peripheral is driven directly (no BLE/802.15.4), so the
# Bluetooth/IEEE802154/MPSL stacks are disabled in prj.conf and the RADIO IRQ is
# owned by this app.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(openpuck C CXX)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/compat)

# status_led uses Arduino logical pin numbers; under the absolute-nRF-pin
# digitalWrite shim, the SuperMini's P0.15 blue LED is absolute pin 15 (the
# header's default 24 is the Adafruit logical pin). LED_BUILTIN (P1.15 = 47) is
# set in the Arduino.h shim.
target_compile_definitions(app PRIVATE WAKE_LED_PIN_B=15)

target_sources(app PRIVATE
  src/main.cpp

  # compatibility layer
  src/compat/arduino_compat.cpp
  src/compat/littlefs_shim.cpp
  src/compat/serial_cdc_glue.cpp
  src/compat/usb_glue.cpp           # Zephyr USBD backing for the TinyUSB shim
  src/compat/usb_device_setup.cpp   # usbd_context (HID pool + CDC) bring-up
  src/compat/usb_webusb_class.cpp   # WebUSB vendor class (config panel pipe)
  src/compat/usb_xinput_class.cpp   # XInput (Xbox) vendor class

  # ported firmware modules
  src/config.cpp
  src/bonds.cpp
  src/identity.cpp
  src/triton.cpp
  src/gamepad_util.cpp
  src/radio.cpp
  src/rf_link.cpp
  src/rf_diag.cpp
  src/haptics.cpp
  src/status_led.cpp
  src/controllers.cpp
  src/puck_hid.cpp
  src/mode_lizard.cpp
  src/mode_switch_hori.cpp
  src/mode_switch_pro.cpp
  src/mode_ps5.cpp
  src/mode_hidgyro.cpp
  src/mode_xinput.cpp
  src/webusb_config.cpp
  src/serial_console.cpp
  src/usb_mount.cpp
  src/wake_hid.cpp
)
