# ESB backend migration (adopt Nordic `nrf_esb`)

The default radio path (`radio.cpp` + the poll loop in `rf_link.cpp`) is a hand-rolled, synchronous,
busy-wait half-duplex RADIO driver. It works, but it has **no automatic retransmission** and runs entirely
inside `loop()`, so a single lost poll/reply surfaces to the app as a reply gap. Those gaps are what trip the
reconnect/connect-edge paths that cause the connect-time buzz. A real ESB stack hides single-packet loss in
hardware (auto-ack + auto-retransmit) and runs off RADIO IRQ + PPI + a dedicated timer, independent of
`loop()`.

This change vendors Nordic's `nrf_esb` (nRF5 SDK 17.1.0) and adds an `nrf_esb`-based backend behind a
build flag. **It is foundational, not finished** — see Status.

## Build it

```sh
make compile                                             # default raw backend (unchanged, 151260 B)
make compile EXTRA_FLAGS="-DNRF52840_XXAA {build.flags.usb} -DCFG_TUD_HID=4 -DOPK_RADIO_ESB=1"
```

`OPK_RADIO_ESB=0` (default) leaves the production build byte-for-byte unchanged. `=1` builds and links the
ESB backend (+~1.5 KB flash, +2 KB RAM) and calls `esbBackendInit()` at boot.

## What's in the tree

| file | role |
|------|------|
| `OpenPuck/nrf_esb.{c,h}`, `nrf_esb_error_codes.h`, `nrf_esb_resources.h` | **vendored verbatim** from nRF5 SDK 17.1.0 `components/proprietary_rf/esb`. The only edits: `nrf_esb.c` is wrapped in `#if OPK_RADIO_ESB` and its SDK includes (`nrf_error.h`/`sdk_common.h`/`sdk_macros.h`/`app_util.h`/`nrf_gpio.h`/`nrf_delay.h`) point at the shim. Excluded from `make format` (Nordic style). |
| `OpenPuck/nrf_esb_shim.h` | tiny shim providing the only SDK symbols `nrf_esb` needs: `nrf_error.h` (re-included from the BSP), the `VERIFY_*` macros, `STATIC_ASSERT`, `ret_code_t`, and `NRF_ESB_MAX_PAYLOAD_LENGTH=96`. Avoids vendoring the whole `sdk_common` chain. |
| `OpenPuck/esb_backend.{h,cpp}` | the puck-side glue: configures `nrf_esb` as a PTX poll master and bridges its event-driven model to the synchronous `poll → reply` shape `rf_link` expects. |

## Link parameters (must match the puck — from `radio.cpp` / `docs/PROTOCOL.md`)

| puck link | `nrf_esb` config | note |
|-----------|------------------|------|
| PHY = `Ble_2Mbit` | `bitrate = NRF_ESB_BITRATE_2MBPS_BLE` | BLE 2M = 16-bit preamble. **Not** `NRF_ESB_BITRATE_2MBPS` (that is `Nrf_2Mbit`, 8-bit preamble — would not sync.) |
| CRC16 poly 0x11021 init 0xFFFF, addr included | `crc = NRF_ESB_CRC_16BIT` | `nrf_esb`'s 16-bit CRC is exactly this. |
| dynamic length, S1LEN=3 (PID+noack) | `protocol = NRF_ESB_PROTOCOL_ESB_DPL` | ESB DPL framing. |
| 5-byte address (BALEN=4) | `set_base_address_0` + `set_prefixes` | `nrf_esb` bit-reverses to the wire itself — pass the **stored** bytes (as in `g_rfBase`/the bond), not the reversed form. |
| reply in the ACK window | PTX + `selective_auto_ack`, poll TX with `noack=0` | controller is the PRX; its F1/F3 reply rides back as the ACK payload. The E1 beacon uses `noack=1`. |
| F1 up to ~66 B | `NRF_ESB_MAX_PAYLOAD_LENGTH = 96` | default 32 would truncate the 0x43-augmented F1. |

## Resource ownership (the runtime risk)

`nrf_esb` takes exclusive ownership of **RADIO**, **TIMER2** (sys timer) and **TIMER3** (nRF52 anomaly-132
workaround), **SWI0_EGU0** (event IRQ), and **PPI channels 7–13**. There is **no SoftDevice** in this build
(BLE is never started), so RADIO/PPI are free. The link test confirms `nrf_esb`'s `RADIO_IRQHandler`,
`TIMER2/3_IRQHandler` and `SWI0_IRQHandler` do **not** collide with the Adafruit core (the image links
clean). Before runtime bring-up, confirm the core/TinyUSB does not also drive TIMER2/TIMER3/PPI 7–13 at
runtime; reassign via `nrf_esb`'s `#define`s (overridable) if it does.

## Status

Done and verified (compiles + links, no symbol conflicts; default build unaffected):
- vendored `nrf_esb` builds against the Adafruit BSP via the shim;
- `esb_backend` configures it for the puck's exact link and is linked into the `OPK_RADIO_ESB=1` image;
- `esbBackendInit()` is the boot entry point.

Remaining (hardware-gated — cannot be done without a puck + controller on the bench):
1. **Wire the data path.** Route `rfConnStep`/`rfConnTx` (poll) through `esbBackendPoll()` and
   `rfHostFrameOnce` (E1 beacon) through `esbBackendSendNoAck()`, and **gate off** the raw `rfConfig()`/
   `NRF_RADIO` path in `rf_link.cpp`/`radio.cpp` under `OPK_RADIO_ESB` (the two cannot both own RADIO). Today
   with the flag on, `esbBackendInit()` and the raw poller both touch RADIO, so the link does not function —
   this is why the flag is experimental/off.
2. **Channel hopping.** Re-point `rfHopTo`/QoS at `esbBackendSetChannel()`.
3. **On-air A/B.** With a real controller: confirm it answers the PTX poll with F1 in the ACK payload on the
   bonded address; tune `retransmit_count`/`retransmit_delay`; verify the E1 discovery beacon still lands.
4. **Pacing.** `esbBackendPoll` busy-waits for the TX-done edge to mirror `rfConnTx`; once stable, consider
   making the poll loop fully event-driven (TX-done callback drives the next poll) so it no longer blocks
   `loop()` at all — the real payoff of moving to `nrf_esb`.

Once 1–3 are validated, the buzz-relevant win is automatic: a lost poll is retransmitted in hardware instead
of becoming the reply gap that the (separately fixed) reconnect/connect-edge logic reacts to.
