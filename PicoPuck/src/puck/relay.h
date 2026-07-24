// relay.h — host→controller relay seam.
//
// The personality hands Steam's actuator/config writes (haptics 0x80-0x86,
// settings 0x87, power-off 0x9F, and feature-id-1 passthrough) to relay_enqueue.
// Where they go depends on what is bound to the slot:
//   - Phase 2: a generic BLE/Classic pad → mapped to the pad's rumble output.
//   - Phase 4: a Steam Controller 2 → GATT-written verbatim to the Valve report
//     characteristic (transparent forwarding).
// In Phase 1 there is nothing bound, so this is a no-op.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_RELAY_H
#define PICOPUCK_RELAY_H

#include <stdint.h>

// Queue one command (cmd byte + payload) for the controller bound to `slot`.
void relay_enqueue(int slot, uint8_t cmd, const uint8_t *payload, uint16_t len);

#endif // PICOPUCK_RELAY_H
