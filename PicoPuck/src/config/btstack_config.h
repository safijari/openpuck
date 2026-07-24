// btstack_config.h — BTstack configuration for PicoPuck (CYW43, dual-mode).
//
// Derived from joypad-os src/bt/btstack/btstack_config.h with PicoPuck deltas:
// four controller slots instead of two, and no BLE-peripheral / HIDS-device
// role (we are a central/host only). Not compiled until the BTstack libraries
// are linked (Phase 2).
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// ---- port features ---------------------------------------------------------
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_BTSTACK_STDIN
#define HAVE_MALLOC              // CYW43 path provides malloc
#define HAVE_PRINTF
#define ENABLE_PRINTF_HEXDUMP

// ---- BTstack features ------------------------------------------------------
#ifndef ENABLE_BLE
#define ENABLE_BLE
#endif
#define ENABLE_LE_CENTRAL
// LE peripheral is enabled even though we never advertise: parts of BTstack's
// hci.c reference advertisement state that only exists under this flag, so the
// central-only build fails to compile without it. Matches joypad-os's config.
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_SECURE_CONNECTIONS

#ifndef ENABLE_CLASSIC
#define ENABLE_CLASSIC
#endif

#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO
// #define ENABLE_LOG_DEBUG      // verbose; enable when debugging BT bring-up

#define ENABLE_SOFTWARE_AES128
#define ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS

// ---- buffer sizes ----------------------------------------------------------
#define HCI_ACL_PAYLOAD_SIZE 256
#define HCI_INCOMING_PRE_BUFFER_SIZE 14

#ifdef BTSTACK_USE_CYW43
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#endif

// ---- memory pools (sized for four controller slots) ------------------------
// Whether the CYW43 firmware actually schedules four concurrent links well is
// an open hardware question (see README risks); the pools allow it so the
// ceiling is measured, not baked in below the target.
#define MAX_NR_HCI_CONNECTIONS 4
#define MAX_NR_L2CAP_CHANNELS 16
#define MAX_NR_L2CAP_SERVICES 3
#define MAX_NR_GATT_CLIENTS 5
#define MAX_NR_WHITELIST_ENTRIES 4
#define MAX_NR_LE_DEVICE_DB_ENTRIES 4
#define NVM_NUM_LINK_KEYS 4
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 8
#define NVM_NUM_DEVICE_DB_ENTRIES 8

// ---- HID host --------------------------------------------------------------
#define ENABLE_HID_HOST
#define MAX_NR_HID_HOST_CONNECTIONS 4
#define MAX_NR_HIDS_CLIENTS 4
#define MAX_NR_BATTERY_SERVICE_CLIENTS 4

#endif // BTSTACK_CONFIG_H
