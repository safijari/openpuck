# Vendored libxsm3

Upstream: https://github.com/InvoxiPlayGames/libxsm3

Pinned git revision: **a085709cce5fe0da19f4a5213f7b6f325d3aff7a**

Vendored and locally modified for OpenPuck on **2026-09-17**. This is not an
unmodified upstream release. All 12 upstream C source/header files and
[LICENSE.txt](LICENSE.txt) are included; upstream README and git metadata are
not required for compilation. Original source formatting is retained rather
than applying the parent project's formatter. Final newlines are normalized.

## Licensing and attribution

libxsm3 is licensed under **GNU LGPL version 2.1 or (at your option) any later
version**. See the complete [LICENSE.txt](LICENSE.txt) and the original notices
in [xsm3.c](xsm3.c), [xsm3.h](xsm3.h), [usbdsec.c](usbdsec.c) and
[usbdsec.h](usbdsec.h):

- Copyright (C) 2022-2023 InvoxiPlayGames (xsm3.c).
- Copyright (C) 2022 InvoxiPlayGames (xsm3.h, usbdsec.c, usbdsec.h).
- Copyright (C) 2013 oct0xor (usbdsec.c).

The files prefixed `excrypt` originate from
[emoose/ExCrypt](https://github.com/emoose/ExCrypt) and are licensed under the
**BSD 3-Clause License**, reproduced in full below. The authoritative notice is
from [ExCrypt revision b2e037c3102de22d1107d1e362df4ce407d964ac](https://github.com/emoose/ExCrypt/blob/b2e037c3102de22d1107d1e362df4ce407d964ac/LICENSE),
the exact license revision linked by libxsm3's upstream README. This identifies
the license source, not an independently established revision for every
vendored ExCrypt source file; the source pin is the libxsm3 revision above.

Upstream credits retained here:

- [oct0xor](https://github.com/oct0xor) for reversing, documenting and
  implementing much of XSM3 ([write-up](https://oct0xor.github.io/2017/05/03/xsm3/),
  [implementation](https://github.com/oct0xor/xbox_security_method_3)).
- [emoose](https://github.com/emoose) for reimplementing XeCrypt functions in
  ExCrypt.
- [sanjay900](https://github.com/sanjay900) and an anonymous contributor for
  helping discover the retail keys.
- Upstream DES and SHA source attribution links to
  [fffaraz/cppDES](https://github.com/fffaraz/cppDES) and
  [mohaps/TinySHA1](https://github.com/mohaps/TinySHA1) are retained in source.

Distributions must retain these notices and the license texts. Distributing a
statically linked firmware also requires satisfying the LGPL's applicable
source/relinking requirements; this notice alone is not a substitute for them.

## Local changes and integration contract

- [xsm3.c](xsm3.c), [xsm3.h](xsm3.h): `xsm3_initialise_state(void)` resets
  `xsm3_has_kv_keys` along with the key buffers. Call this when starting a new
  authentication session, then set identification data and optionally import
  console-specific keys. The library retains upstream singleton state and is
  not reentrant; firmware must serialize calls.
- Firmware must provide `bool xsm3_random_bytes(uint8_t *buffer, size_t length);`
  with C linkage (declared in [xsm3.h](xsm3.h)). It must fill the entire requested
  buffer with fresh cryptographically suitable random bytes and return true;
  return false on failure, which aborts the challenge without a response.
  There is no predictable fallback and no `srand`, `rand`, `time` or debug
  printing dependency.
- `bool xsm3_do_challenge_init(uint8_t challenge_packet[0x22]);` and
  `bool xsm3_do_challenge_verify(uint8_t challenge_packet[0x16]);` reject incorrect
  length fields, XOR checksums and MACs with `false`. Both clear all 48 response
  bytes before validation, so no stale or partial response is published on
  failure. Init checks the MAC before changing session data. Verify uses a local
  salt copy and commits it only after the incoming MAC passes.
- **Firmware must check actual received lengths:** exactly 34 bytes for init
  and 22 for verify. C array parameter declarations do not enforce lengths.
  The library checks byte 4 equals 28 (init) or 16 (verify) *before* calling the
  checksum routine; it does not validate USB transfer lengths or impose an
  authentication state machine. Inputs must be non-NULL and must not alias the
  global response buffer. The caller must require successful init before verify,
  abort authentication on `false`, and never report completion or transmit a
  response on failure. Successful responses occupy 46 bytes (init) or 22 bytes
  (verify), including header and checksum; the remaining response bytes are zero.
- Identification data likewise requires an actual 29-byte buffer; invalid
  length field (not 23) or checksum clears stored identification data and returns
  without importing it. Its existing `void` API is retained. Checksum indexing
  uses `size_t` rather than an overflowing 8-bit temporary.
- [usbdsec.c](usbdsec.c), [usbdsec.h](usbdsec.h): salt increment and MAC/ACR XORs
  use bytes, preserving big-endian counter carry/wraparound and in-place salt
  mutation. The declaration now correctly marks salt mutable and includes
  `<stdint.h>`; both XSM3 and USB security headers have C++ linkage guards.
- [excrypt_des.c](excrypt_des.c): key/IV loads and IV stores use `memcpy` instead
  of integer dereferences. The existing byte-safe ECB/CBC input/output copies
  are retained, including in-place decryption behavior.
- [excrypt_parve.c](excrypt_parve.c), [excrypt_parve.h](excrypt_parve.h):
  `ExCryptChainAndSumMac` now accepts byte pointers for `cd`, `ab`, `input` and
  `output` (its count is still in 32-bit words). Big-endian loads use `memcpy`
  plus byte swap; output uses exact 32-bit stores. ACR no longer casts byte
  buffers (including response + 5) to integer pointers. Other Parve accesses
  already use byte operations or `memcpy` and are retained.
- [excrypt_sha.c](excrypt_sha.c): remove the unused duplicate `state`
  declaration that prevents upstream compilation. Its word buffer is accessed
  through permitted byte aliases and `memcpy`. The upstream single-block SHA-1
  implementation is retained: **only inputs of 0 through 55 bytes are supported**
  (XSM3 uses 8 and 32); it is not a general-purpose SHA-1 API. DES/CBC and MAC
  helpers likewise require whole 8-byte blocks and adequately sized buffers.

These changes target GCC/Clang and the little-endian ARM Cortex-M4. They do not
add hardware RNG glue, USB request handling, firmware authentication state,
console tests or changes to parent build/format configuration.

## Native validation (2026-09-17)

Validation used transient compiler-stdin harnesses, not firmware glue or
committed test sources:

- GCC 16 C11 compilation at `-O2` with `-Wall -Wextra -Werror`, strict aliasing,
  `-Wcast-align=strict` and `-Wstrict-aliasing=2`; C++11 linkage and bool API
  type checks; object symbols confirm no printing, libc PRNG or clock dependency.
- Standard DES known-answer vector and all SHA-1 lengths 0..55 against Python
  `hashlib`; 128 differential cases each for 3DES, salted/unsalted MAC, ACR,
  Parve ECB/CBC and chain-and-sum, including unaligned buffers, in-place CBC
  decryption and salt carry/wraparound.
- 24 init responses and 96 successive verify responses matched upstream
  byte-for-byte. Both harnesses used identical deterministic test-only random
  bytes. The upstream baseline required removing its duplicate SHA declaration
  in compiler input and explicitly refreshing its KV keys between sessions
  because its reset flag is broken; no upstream files were edited.
- UBSan trap-mode execution at offsets 0..7 covered exact-size packet buffers,
  every header-length byte, bad checksums, each ciphertext/MAC byte corrupted
  with recomputed checksums, all-zero failed responses, unchanged session data
  on rejection, RNG invocation, imported/generated key reset, repeated valid
  challenges and crypto buffer alignment. All checks passed.
- ASan execution was unavailable because the host's GCC sanitizer shared
  libraries are missing. UBSan trap mode needs no runtime and passed; it is not
  a substitute for ASan heap-bound checking. ARM firmware builds and physical
  console validation remain integration tasks.

## ExCrypt BSD 3-Clause notice

BSD 3-Clause License

Copyright (c) 2020, emoose
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
