# Vendored libjoycon2

Upstream: https://github.com/bmmund/libjoycon2

Pinned release: **v0.1.0**
Pinned git revision: **aef70cb3a4ff42c65b887a62afebdc56895eeea9**

Vendored unmodified into OpenPuck on **2026-11-22** via `tools/sync-libjoycon2.sh`.
Only `src/*.c`, `src/*.h`, and [LICENSE](LICENSE) are vendored; upstream tests,
examples, and protocol docs live in the upstream repository.

## Licensing and attribution

libjoycon2 is licensed under the **MIT License**. See [LICENSE](LICENSE).

- Copyright (c) 2026 Braden Mund

Protocol notes and Switch 2 Joy-Con 2 BLE / GATT / report documentation live in
[https://github.com/bmmund/libjoycon2/blob/v0.1.0/docs/protocol-map.md](https://github.com/bmmund/libjoycon2/blob/v0.1.0/docs/protocol-map.md).

## Integration contract

- OpenPuck instantiates a single static `jc2_device_t` and injects LittleFS
  bond persistence (`/jc2_bonds.bin`) via `jc2_callbacks_t`.
- `jc2_ll_nrf52.c` owns `RADIO_IRQHandler`, `TIMER0_IRQHandler`, and
  `SWI0_EGU0_IRQHandler` while Joy-Con 2 BLE mode is active and exposes
  `jc2_ll_acquire_gap()` / `jc2_ll_remaining_gap_us()` / `jc2_ll_release_gap()`
  for 2.4 GHz Steam Controller ESB coexistence.
- To update to a newer upstream release, run:
  `./tools/sync-libjoycon2.sh https://github.com/bmmund/libjoycon2.git <tag>`
