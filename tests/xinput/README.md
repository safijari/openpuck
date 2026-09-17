# Native XInput regressions

From the repository root:

```sh
python3 tools/test-xinput.py
```

Requires Python 3, a C11 compiler (`CC`, default `cc`), and a C++17 compiler
(`CXX`, default `c++`). Compiler variables accept arguments, such as a compiler
wrapper. No Arduino SDK, hardware, Python packages, OpenSSL, or shared UBSan
runtime is needed for the default run. All objects, the generated fixture
header, and the executable live in a `TemporaryDirectory` that is cleaned up
on success or failure. Compiler, linker, sanitizer, or assertion failures
produce a nonzero exit status. UBSan uses trap mode for both C and C++.

Optional checks:

```sh
python3 tools/test-xinput.py --check-fixtures
CC=clang CXX=clang++ python3 tools/test-xinput.py --asan
```

Fixture regeneration requires the `openssl` executable with two-key 3DES-CBC
support. ASan requires a compiler with a usable host ASan runtime and a matching
C++ standard library. Neither optional check is silently skipped on failure.

## What is compiled and simulated

The runner compiles the on-disk production `xinput_auth.cpp` and all five
vendor crypto C sources, without patching or copying their implementations.
`xinput_descriptors.h` is compiled into the test binary. The only substituted
firmware interfaces are TinyUSB EP0 and the platform hooks from `xinput_auth.h`
and `xsm3_random_bytes` (the latter returns `bool`).

The USB stub retains the actual transfer buffer. OUT writes happen between
SETUP and DATA, followed by ACK. IN snapshots are limited to the smaller of
the offered length and `wLength`. Nested locks are checked for LIFO restoration;
RNG and worker notification hooks must execute outside the lock. Scheduling is
explicit, not a mock crypto completion: `xinputAuthProcess()` performs real
DES/3DES, SHA-1, MAC and ACR operations.

Coverage includes:

- Identification framing/checksum and stable 12-byte serial across resets.
- Interface recipients 3 and 0x0103, device recipient 0, wrong directions,
  recipients, indices and commands; 21 selected lengths from 0 through 65535.
- Exact 34/22-byte OUT lengths; 29/46/22/2/0-byte IN offers and truncation.
- No reply before completion; state 1 while queued, state 2 after processing;
  DATA/ACK notification behavior, duplicate DATA rejection and idempotent work.
- Two distinct console IDs/nonces, reinitialization without an explicit reset,
  explicit resets, three consecutive verify challenges, and repeated IN reads.
- Invalid packet lengths, checksum and MAC (with a repaired checksum), failure
  without stale readiness, RNG/worker startup failure, recovery on a new init.
- Reset before DATA, while queued, and inside the RNG crypto callback; a new
  console challenge injected during crypto must supersede the old result.
- EP0 response ownership across reset and a worker finishing during an IN poll.
- Descriptor body/total lengths, four interfaces, class descriptor lengths,
  all seven endpoint addresses, packet sizes, intervals and security string.

## Independent oracle

`fixtures.json` contains synthetic retail-key exchanges, **not hardware
captures**. `reference.py` reproduces them without loading any production C
code: OpenSSL supplies DES/3DES, hashlib supplies SHA-1, and Python implements
framing, key derivation, salted MAC and PARVE/ACR arithmetic. Public constants
and algorithm provenance are recorded in that script. A standard DES known
answer checks its backend before generation.

The native console fixture constructs its own encrypted init, derives session
keys from its console ID and nonce, decrypts the **actual init response**, then
uses the recovered controller nonce and plaintext hash to construct successive
verify requests. It never calls the production XSM3 state machine as the
console. Although it uses the low-level vendor primitives, every challenge and
response must also match the independently generated JSON oracle byte for
byte, including ACRs and checksums. Thus shared primitive bugs cannot pass
merely by round-tripping against themselves. `--check-fixtures` regenerates
and compares the JSON in memory; it never overwrites the committed fixture.

Standalone known answers cover DES, SHA-1 (empty, `abc`, 32- and 55-byte input),
and salted MAC with full 64-bit counter wrap. DES parity conversion is checked
for preservation of the effective key schedule.

## Limits

The vendor SHA-1 helper supports only single-block inputs (at most 55 bytes);
the standard 56-byte SHA-1 test fails. Auth hashes 8 or 32 bytes. The vendor DES
parity helper preserves effective key bits but does not consistently produce
odd parity; DES ignores those bits. Tests document these existing constraints
rather than changing firmware or pretending the helpers are general-purpose.

This harness does not validate actual USB enumeration, vendor callback routing
outside `xinputAuthControl()`, FreeRTOS scheduling, board identity derivation,
hardware RNG, timing, or compatibility with a physical Xbox 360. It is not a
thread-race detector. The descriptor total-length check is body + the standard
nine-byte configuration header, not an Arduino-generated configuration.

Only these tests and `tools/test-xinput.py` are owned by this change; Makefile,
CI, and firmware sources are intentionally untouched. Handwritten C++ and the
stub follow the repository's clang-format 18 style.