# PicoPuck — a Steam-Controller-2 puck emulator for Raspberry Pi Pico W / Pico 2 W.
#
# Presents the same Valve puck USB identity as OpenPuck (28DE:1304) while
# connecting to controllers over Bluetooth (Classic + BLE) via the CYW43 radio
# and BTstack. See README.md for the build instructions and PLAN/phase notes.
#
# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.13)

# Board must be chosen on the command line: -DPICO_BOARD=pico_w | pico2_w.
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico_w CACHE STRING "Target board (pico_w or pico2_w)")
endif()

# Pull in the SDK (PICO_SDK_PATH env var or -DPICO_SDK_PATH=...) before project().
include(pico_sdk_import.cmake)

project(picopuck C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# --- version / build id -----------------------------------------------------
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE PICOPUCK_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT PICOPUCK_GIT_COMMIT)
    set(PICOPUCK_GIT_COMMIT "unknown")
endif()

add_executable(picopuck)

target_sources(picopuck PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb/usb_descriptors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb/puck_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb/usb_tx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb/webusb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/puck/identity.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/puck/slots.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/puck/personality.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/puck/relay.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/puck/triton.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bt/bt_control_stub.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bt/bt_host.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bt/bt_valve.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bt/input_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bt/drv_xinput.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bt/drv_sc2.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bt/drv_ds4.c
)

target_include_directories(picopuck PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config
)

target_compile_definitions(picopuck PRIVATE
    CYW43_LWIP=0                 # no networking; we only use the radio for BT
    BTSTACK_USE_CYW43=1          # BTstack runs on the CYW43 async_context
    ENABLE_BTSTACK=1
    PICOPUCK_GIT_COMMIT="${PICOPUCK_GIT_COMMIT}"
)

# LED blink + BT radio need cyw43_arch. The _poll variant (no background IRQ
# thread) is what BTstack-on-CYW43 uses; it is serviced by cyw43_arch_poll().
# USB (tinyusb_device) and BTstack libraries are added in later phases.
target_link_libraries(picopuck PRIVATE
    pico_stdlib
    pico_unique_id
    pico_cyw43_arch_poll
    pico_btstack_ble
    pico_btstack_classic
    pico_btstack_cyw43
    tinyusb_device
    tinyusb_board
)

# Debug console on UART (GP0/GP1); USB is reserved for the puck interfaces.
pico_enable_stdio_uart(picopuck 1)
pico_enable_stdio_usb(picopuck 0)

pico_add_extra_outputs(picopuck)

# Name the UF2 per board so both artifacts can live side by side.
set_target_properties(picopuck PROPERTIES OUTPUT_NAME "picopuck_${PICO_BOARD}")

message(STATUS "PicoPuck: board=${PICO_BOARD} commit=${PICOPUCK_GIT_COMMIT}")
