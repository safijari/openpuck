# OpenPuck RF Sniffer

A **passive** 2.4 GHz sniffer for the link between a real Valve SC2 puck and its controller. It never
transmits, so it cannot disturb the pairing — it only listens, and streams every frame to a browser app in
real time with **Start Cap / End Cap**.

Use it to capture the actual puck↔controller traffic for any function (shutdown, LED, battery, …), isolate
the relevant frames, and export them.

## What you need
- A **second** nRF52840 board (e.g. the same Pro Micro / Feather you flash OpenPuck onto) — *not* the puck.
- The real Valve puck + controller, paired and working.
- Chrome or Edge (WebUSB).

## 1. Flash the sniffer firmware
```
arduino-cli compile -b adafruit:nrf52:feather52840 \
  --build-property "build.extra_flags=-DNRF52840_XXAA {build.flags.usb}" \
  --upload -p <PORT> puck_sniffer
```
(Same manual-DFU/double-tap-reset dance as OpenPuck if the port won't auto-reset.) It enumerates as
`28DE:534E "OpenPuck Sniffer"`.

## 2. Open the app
WebUSB needs a secure context, so **not** a `file://` path. Either:
- **Hosted:** https://safijari.github.io/OpenPuck/sniffer.html (once `docs/` is pushed to gh-pages), or
- **Local:** `cd docs && python3 -m http.server` → open http://localhost:8000/sniffer.html

Click **Connect sniffer**, pick *OpenPuck Sniffer*.

## 3. Capture — the exact sequence (do it in this order)

Why order matters: the link is Nordic **Gazell** — the on-air address is random per session, and **a *connected*
slot never beacons on ch2; only a *reconnecting* one does.** So you can't lock onto an already-connected
controller — you must catch its **reconnect**. The bond's **ibex_uuid is stable** across reconnects (only the
address changes), so we anchor the lock on that uuid and force a reconnect.

### Setup (once)
1. **Unplug the copycat** (the OpenPuck dev board) if it's plugged in — it beacons its own `E1`s and pollutes
   ch2. Leave only: the **real Valve puck**, the **controller**, and the **sniffer board**.
2. Controller connected to the **real puck** (Steam sees it). With `pairtui` (see `../pairtui/`), note the
   connected slot's **ibex_uuid** (e.g. `EF7171B4`) — it's the bond id, stable across sessions.

### Arm the sniffer
3. Flash the sniffer, open the app (`docs/sniffer.html` over localhost/https — not `file://`), **Connect** →
   *OpenPuck Sniffer*. Confirm **rx** is climbing.
4. In **lock ibex**, type the uuid (`EF7171B4`), click **Lock**. Status → **ACQUIRE** (waiting on ch2 for *only*
   that slot's `E1`; ignores other slots and the copycat).
5. Click **● Start Cap**.

### Trigger (the step that's easy to miss)
6. **Power-cycle the controller** — hold its power button until it turns **off**, wait ~2 s, turn it back **on**.
   On reconnect it re-runs ch2 discovery and broadcasts slot 3's `E1` (matching the uuid) with a **fresh** session
   address → the sniffer matches the uuid, locks that session, and hunts its channels.
7. Watch the app: link → **CAPTURE (session locked)** and **`C→P` climbs**. Wiggle sticks / trackpad to keep
   traffic flowing.

### Capture the action
8. With `C→P` climbing, do the thing in Steam (change brightness / LED, or turn off).
9. **■ End Cap → Download.**

**Always: Lock the uuid → Start Cap → *then* power-cycle.** The most common failure is the controller silently
auto-reconnecting *before* the lock is armed, so its `E1` is never caught.

Direction: `P→C` (opcode `0xE_`, puck→controller — LED/shutoff/brightness commands ride here) vs `C→P` (`0xF_`,
controller→puck — input + battery/telemetry). The `filter` box matches payload hex (e.g. `9f`).

### If `C→P` won't climb after the power-cycle
- **Survey ch2** and look at the `op e1` rows during the reconnect: one payload should contain your uuid in
  little-endian (`EF7171B4` → `… b4 71 71 ef …`). If you only ever see a *different* uuid, the copycat or another
  bonded slot is the only thing reconnecting — make sure the real controller is re-pairing to the **real** puck.
- Re-check you did **Lock → Start Cap → power-cycle** in that order.

> Bond info (which controller/slot, its serial + uuids) is read over USB, not the air — see `../pairtui/`. Useful
> for the uuid lock and labelling a capture; it does **not** give the on-air address (random per session).

## How it works
After RX the radio buffer is `[LENGTH][S1][payload…]`; `payload[0]` is the opcode. The puck advertises its
per-session base/prefix/channel inside the `E1` host frame on the shared `ibex`/ch2 rendezvous, so the sniffer
reads those, retunes to the session, and receives both directions (same ESB base — and it now listens on **all
8 ESB pipes** of that base, so a controller reply on a different prefix is still caught; the **pipe** column /
status `last pipe` shows which `RXMATCH` each frame hit). A QoS channel-hop is followed via the `E1` keepalive.
When the session goes silent it **sweeps clean candidate channels keeping the learned address** (rather than
abandoning to ch2) and only falls back to a full discovery scan after two dry sweeps. Radio parameters are
OpenPuck's CRC-validated config (`radio.cpp`).

### If you only see `E2`/`E1` on ch2 and never the controller's `0xF_` replies
That means it never camps on the session. Read a real `E1` row in the table (a `P→C`, op `e1` frame): its
payload bytes are `e1 [proteus:4] [ibex:4] [CH] 00 00 00 [BASE:4] [PFX]` — i.e. **byte 9 = session channel,
bytes 13–16 = session base, byte 17 = prefix** (the "E1 advertises" stat shows the sniffer's own read of these).
Cross-check that against where traffic actually is, then type the six values into **pin session**
(`b0 b1 b2 b3 pfx ch`, address bytes hex) and click **Pin** to force the sniffer onto that exact session — this
bypasses auto-acquire entirely, which is the reliable path if the real puck's `E1` offsets differ from ours.

## Stream protocol (WebUSB bulk)
- packet: `C0 DE [N] [t_us:4 LE] [ch] [flags] [rssi] [match] [N raw bytes]`  (flags bit0 = CRC ok; match = RXMATCH pipe; raw = `[LEN][S1][payload]`)
- status: `C1 DE [state] [curCh] [base:4] [prefix] [cap] [hb:2] [advCh] [advBase:4] [advPfx] [lastMatch]`  (19 B; adv* = session parsed from the last `E1`)
- commands (bulk OUT): `01` start · `02` stop · `03` re-acquire · `04 <ch>` pin channel · `05 <b0 b1 b2 b3 pfx ch>` pin full session
