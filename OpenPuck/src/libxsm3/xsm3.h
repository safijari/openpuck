/*
    xsm3.h - part of libxsm3
    Copyright (C) 2022 InvoxiPlayGames

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* OpenPuck modifications, 2026-09-17: RNG hook, bool results and C++ linkage.
   See NOTICE.md. */

#ifndef XSM3_H_
#define XSM3_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Identification data taken from an official wired controller. (Serial number is static.)
extern const uint8_t xsm3_id_data_ms_controller[0x1D];

// The response data from the previously completed challenge.
extern uint8_t xsm3_challenge_response[0x30];

// The console ID fetched from the console after request 0x82.
extern uint8_t xsm3_console_id[0x8];

// Supplied by firmware: fill the entire buffer with fresh cryptographic random bytes.
// Return false on failure; never substitute predictable data.
bool xsm3_random_bytes(uint8_t *buffer, size_t length);

// Clears the state of the XSM3 internal variables, including imported/generated keys.
void xsm3_initialise_state(void);

// Caller supplies 29 bytes. Invalid length field/checksum clears identification data.
void xsm3_set_identification_data(const uint8_t id_data[0x1D]);

// Imports the console-specific keys (GetKey index 0x23 and 0x24 for wired controllers) into the current state.
void xsm3_import_kv_keys(const uint8_t key1[0x10], const uint8_t key2[0x10]);

// Initialises the XSM3 state using information from the challenge init packet (0x82) and places a response in xsm3_challenge_response.
// Caller must supply 34 actual bytes. Requires packet[4] == 28; false clears response.
bool xsm3_do_challenge_init(uint8_t challenge_packet[0x22]);

// Completes a verify challenge passed from request 0x87 and places the response data in xsm3_challenge_response.
// Caller must supply 22 actual bytes. Requires packet[4] == 16; false clears response.
// Caller must enforce a successful init before verify and abort authentication on false.
bool xsm3_do_challenge_verify(uint8_t challenge_packet[0x16]);

#ifdef __cplusplus
}
#endif

#endif // XSM3_H_
