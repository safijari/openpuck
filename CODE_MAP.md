# OpenPuck firmware — architectural map

nRF52840 / Adafruit Arduino core / TinyUSB. The device impersonates a Valve Steam
Controller 2 "puck" over USB and talks to SC2 controllers over a bare-metal nRF52
RADIO (ESB-style, **no SoftDevice**) protocol. Built with `-DCFG_TUD_HID=6`.

> This map describes what the **code actually does**. Source comments in this repo
> are known to be misleading and were ignored.

---

## 0. Execution contexts (read this first)

Three contexts matter for every piece of state below:

| Context | Priority / stack | What runs here |
|---|---|---|
| **loop task** | Arduino FreeRTOS "loop" task, low priority, ~4 KB stack | Everything called from `loop()`: `webusbPoll`, `g_active->task()`, `serialConsolePoll`, `rfDiagTask`, `rfLinkTask`, `hapticTask`, `ledTask`, `usbMountTask`. Also all of the RF radio code (it is driven synchronously from `rfLinkTask`/`rfDiagTask`). Also `setup()`. |
| **usbd task** | TinyUSB "usbd" task, **high priority, ~800-byte stack** | All TinyUSB HID `set_report`/`get_report` callbacks, the custom XInput class-driver xfer callbacks, and (in the Adafruit core) HID `sendReport` queuing context. This is where host→device control transfers are decoded. **Stack is tiny — a `Serial.printf` here can overflow it (the suspected LOCKUP cause), which is why all per-report logging is `#if OPK_LOG` gated.** |
| **ISR** | Hardware exception | `HardFault_Handler` (fault_diag.cpp). The RADIO is **polled, not interrupt-driven** — there is no radio ISR. |

Key cross-task rule the code relies on: producers that touch the relay rings and the
capture ring run under `__disable_irq()`/PRIMASK; the per-slot HID reply rings are
single-producer (usbd) / single-consumer (loop) with `volatile` head/tail and no lock.

---

## 1. Top level — `OpenPuck.ino`

**Entry point + main loop.** No input/output logic of its own.

### `setup()` (loop task, boot)
- `genSerial()`, `ledInit()`, seed per-slot RF session addresses, `InternalFS.begin()`,
  optional one-time `factoryResetOnce()`, `loadCfg()`, `loadBonds()`, regenerate session
  addresses for used slots.
- Decides USB presentation: `g_xbox = !modeIsPuck`, `g_active = controllerFor(g_usbMode)`.
- Builds the USB descriptor. Two paths:
  - **Dynamic-mount modes** (emulated controllers): `wakeHidBegin()` (HID instance 0,
    unless clean-PS), `g_active->beginPool()`, optional `usb_web.begin()`,
    `usbMountEnable()`, `usbMountRebuildMap()`, `usbReenumerate(g_usbMountCount)`.
  - **Static modes** (puck/lizard, or any non-dynamic): conditional CDC keep
    (`keepCdc`), per-mode serial suffix, ordered interface registration
    (`usb_web` IF0 → wake mouse IF1 → 4 puck slots IF2..5 for puck), then `attach()`.
- Busy-waits up to **3 s** for USB mount: `for (i<300 && !mounted()) delay(10)`.
- `hapticInit()`, `faultDiagBoot()`, prints diagnostics.
- **Arms the hardware watchdog**: `NRF_WDT` CRV = `8*32768-1` (~8 s), RUN-in-sleep.
  If `loop()` ever stops feeding it, the chip resets.

### `usbReenumerate(uint8_t k)` (loop task)
Tears down and rebuilds the USB config descriptor for `k` connected slots **with no MCU
reboot**: `detach()`, `delay(20)`, `clearConfiguration()`, restore identity/serial,
re-add wake mouse + `g_active->mountSlots(k)` + WebUSB in locked instance order,
`attach()`. Called at boot and by `usbMountTask`.

### `loop()` (loop task)
1. Feeds the watchdog (`NRF_WDT->RR[0]`).
2. If `g_dirty`, `saveBonds()` (flash write in loop context).
3. Calls, in order: `webusbPoll`, `g_active->task`, `serialConsolePoll`, `rfDiagTask`,
   `rfLinkTask`, `hapticTask`, `ledTask`, `usbMountTask`.
   `#if OPK_LOG` wraps each in `micros()` timing for the WebUSB panel.

### State owned
- `g_usbCfgDesc[512]` — config-descriptor build buffer (puck composite + WebUSB exceeds 256 B).
- `MODE_SUFFIX[8]`, `s_dynWantWebusb`, `s_dynWantWakeMouse`.
- `g_loopPeriodUs`, `g_loopWorst`, `g_loopWorstUs` — loop-timing diagnostics (read by webusb).

---

## 2. Configuration & identity

### `config.cpp` / `config.h` — persisted settings (loop task)
- **Owns**: `g_usbMode`, `g_xbox`, `g_chordBtn[3]`, `g_persistMode`, `g_bootMode`,
  `g_debugCdcThisBoot`, `g_mDiv`, `g_mFric`, `g_type[ET_COUNT]` (per-emulated-type
  button config), `g_etype`, live mirrors `g_abSwap`/`g_back[4]`/`g_qamMap`/
  `g_padHaptics`/`g_ledBright`, `g_rumbleScale`, `g_swProRate`, `g_swGyroScale10`,
  and the constant `g_pollUs = 4000` (250 Hz, **fixed**).
- `struct Cfg` is serialized to `/cfg.bin` (LittleFS), magic `0xCD`. `rxWin10`,
  `hapticBlockOn`, `hapticBlockS` and the per-type table travel with it; `rsvd0` is the
  one-shot debug-CDC arm.
- `loadCfg()` resolves the boot mode policy: one-shot `bootMode` wins once then clears;
  else `persistMode ? last mode : STEAM(0)`. Calls `applyActiveType()`.
- `saveCfg`, `saveMode`, `armDebugCdcNextBoot`, `factoryErase` (LittleFS `format()`),
  `factoryResetOnce` (git-hash-tagged one-time wipe). All flash I/O — **loop context only**.
- `applyActiveType()` copies `g_type[g_etype]` into the hot-path live mirrors.

### `identity.cpp` / `identity.h` (loop task, boot)
- `genSerial()` derives `g_unit[16]` ("FXB...") and `g_board[16]` ("MXB...") from
  `NRF_FICR->DEVICEID`. `g_usbSerial[18]` holds the per-mode suffixed serial.
- `ATTR83[25]` — the constant 0x83-attribute blob (advertises product 0x1304).

### `build_info.h` / `git_version.h` — compile-time tags only.

---

## 3. Bonds & shared link state — `bonds.cpp` / `bonds.h`

- `NSLOT = 4`. `struct Slot { uint8_t rec[24]; bool used; uint8_t resp[63]; uint16_t resp_len; }`.
  `rec` = `[8 uuid][16 serial]`. `resp`/`resp_len` is the staging buffer for that
  interface's pending feature-report reply.
- **`g_slot[NSLOT]`** — bond records + per-interface reply staging. Written by the puck
  feature command channel (usbd task, `handleSet` 0xA2) and by `loadBonds` (loop). Read
  by RF code, webusb, serial console.
- **`g_connReplyMs[NSLOT]`** — `millis()` of the last F-type reply per slot (**the live
  "controller is alive" timestamp**). Written by the RF decode (loop task,
  `rfConnTx`). Read everywhere: puck_hid connection presentation, haptic link-up gate,
  usb_mount connectivity watcher, webusb, serial console.
- **`g_dirty`** (`volatile`) — set by `handleSet` (usbd) when a bond is written; consumed
  by `loop()` → `saveBonds()` (defers flash off the usbd task).
- `g_pairing` — set by `handleSet` 0xAD.
- `loadBonds`/`saveBonds` — `/bonds.bin` LittleFS, loop context.

---

## 4. RF stack (bare-metal RADIO)

### `radio.cpp` / `radio.h` — hardware layer (loop task)
- Programs `NRF_RADIO` directly (PHY=Ble_2Mbit, ENDIAN=Big, no whitening, CRC16
  poly 0x11021/init 0xFFFF, ESB-DPL framing). No SoftDevice.
- **DMA buffers `rfrx[100]`, `rftx[100]`** (declared >= MAXLEN+2 = 66+). Shared by
  `rf_link.cpp` AND `rf_diag.cpp` with **no guarding** (see §13).
- `g_rfBase[4]="ibex"`, `g_rfPrefix=0x10`, `g_rfCh=2` — shared discovery rendezvous.
- `g_sessCh=18` — connected-session channel (global across slots).
- **`g_sessBase[NSLOT][4]`, `g_sessPrefix[NSLOT]`** — per-bond unique session address,
  derived in `rfGenSessionAddr(slot)` from bond UUID + FICR DEVICEID (deterministic,
  stable across reboots; degenerate bytes scrubbed; never equals "ibex").
- Tunables (`g_crcinit`, `g_whiteiv`, `g_mode`, `g_pcnf0/1`, `g_crcpoly`, `g_crccnf`,
  `g_pid`, `g_balen`, `g_prefixRaw`, `g_statlen`) — mutated by the CDC console and by
  rf_diag's `applyCfg`/sweeps. `rfConfig()` reads them on every (re)program.
- `RWAIT_DISABLED()` macro: **bounded** spin-wait for `EVENTS_DISABLED`, **3000 µs**
  timeout, then bails (never hangs the loop forever).
- `rfSetAddr` bit-reverses the base/prefix into BASE0/PREFIX0.

### `rf_link.cpp` / `rf_link.h` — operational puck protocol (loop task)
The "dongle" role. All driven synchronously from `rfLinkTask()` in `loop()`.

- **`rfLinkTask()`**: host-frame beacons (session keepalive E1 on `g_sessCh`, discovery
  E1 on ch2), `rfConnStep()` poll loop, USB remote-wakeup on a fresh RF connection, QoS
  channel hop, per-second stat aggregation. Gated off during a 2.5 s post-disconnect
  cooldown (`g_connCooldown`).
- **`rfConnStep()`**: cycle gate fires once per `g_pollUs` (250 Hz); on each fire it
  polls **every warm slot back-to-back** (so N controllers all run at full rate). Cold
  slots (silent > 5 s) retry only every 2 s. Per slot it flushes one relay entry and
  sends a bare E3 poll (real-puck behavior; legacy E7/GET path is toggle-able).
- **`rfConnTx(ch, s1, payload, plen, rxWinUs)`**: the core TX-then-RX primitive. TXes
  one frame on the current slot's session address, then busy-waits for a reply
  (`while(!EVENTS_END && micros()-t0 < win)`, `win` = `g_rxWin` default **2000 µs**, or
  a tiny override like **80 µs** for no-ACK relays). On a CRC-good F-type reply it:
  samples RSSI, updates `g_connReplyMs`/`g_linkRssi`, calls `hapticOnReconnect` on a
  gap, and **decodes the 0x45 input report into `g_in[g_curSlot]`** (buttons, sticks,
  triggers, trackpads, IMU). Also handles: Steam-button short-press remote wakeup,
  Steam+Y 2 s power-off chord, back-4 mode-switch chord (`saveMode` + `NVIC_SystemReset`),
  and status reports 0x43 (battery → `g_battery`/`g_batteryState`) / 0x44 → dispatched
  via `g_active->onAuxReport`.
- **`g_curSlot`** — the slot the poll loop is currently driving. Set by `rfConnStep`
  before each TX; read by the decode, the relay flush, haptics, stats, webusb.
- **Shared output state** (all `volatile`, written here, read by puck_hid/webusb):
  `g_linkRssi[NSLOT]`, `g_battery[NSLOT]`, `g_batteryState[NSLOT]`.
- Per-second readouts: `g_f1ps`, `g_newps`, `g_pollsps`, `g_pollPeriodUs`.
- Many runtime toggles (`g_pollGet`, `g_e7announce`, `g_e1keepalive`, `g_rxWin`,
  `g_e3mode`, `g_qos`, …) set from the console/webusb.
- **Buffers**: uses the shared `rftx`/`rfrx` (radio.h). The 0x45 TLV walk bounds every
  read against the received length and `sizeof(rfrx)` (explicit guards against runaway
  `idx` and reads past the buffer — the documented stack-smash/LOCKUP class).

### `triton.cpp` / `triton.h` — decoded controller input
- **`g_in[NSLOT]`** (`struct PuckInput`): the decoded SC2 input per slot — buttons,
  `lx/ly/rx/ry`, `lt/rt`, `lpx/lpy/rpx/rpy`, accel `ax/ay/az`, gyro `gx/gy/gz`.
  **Written by `rf_link` (loop task) once per fresh 0x45; read by every USB
  personality's `task()` (loop) and by webusb.** No lock (single writer, loop-context
  readers; the usbd-task readers in mode_switch_pro read it too — benign).
- TB_* button masks, inline decoders `s16off`/`u16off`/`trigU8`/`btnsOf`, `imuFrom45`.

### `rf_diag.cpp` / `rf_diag.h` — RE / calibration tooling (loop task)
Channel sniffers, raw capture/replay, beacon/respond, config sweeps — all driven from
`rfDiagTask()` and CDC-console start hooks. **Reprograms the same `NRF_RADIO` and shares
`rfrx`/`rftx` with rf_link with zero mutual exclusion** (relies on the design assumption
that diag modes are off during normal operation). Buffers: `g_replay[48]`,
`g_hostTx[48]` (len 0x22=34), `pl[48]` in `rfBeaconOnce` (copy clamped to 48 into
`rftx`). Spin-waits: `RWAIT_DISABLED()` (3000 µs) ×1–3 per call; a 500 µs RX window in
`rfBeaconOnce`. `rfReplayOnce` runs every loop with no rate gate while armed.
Notable: `rfCapPoll` copies 48 bytes into `g_replay` but sets `g_replayLen=32`.

---

## 5. Controller abstraction — `controllers.cpp` / `controllers.h`

- `IController` interface: `begin`, `onReport45`, `onAuxReport`, `task`, `wakeEvent`,
  `isPuck`, and the dynamic-mount hooks `dynamicMount`/`maxSlots`/`usbIdentity`/
  `beginPool`/`mountSlots`.
- **`g_active`** — the one controller for this boot.
- `controllerFor(mode)` maps `g_usbMode` → singleton (Steam & Lizard share `g_steamPuck`;
  PS5/PS5_GAME share `g_ps5Ctl`; HIDGYRO/DS4_GAME share `g_hidGyroCtl`).
- **Dispatch model**: PUSH modes (puck/lizard, Xbox) build+send their host report inside
  `onReport45` (loop task, called from rf_link). STREAM modes (Switch Hori/Pro, PS5,
  HIDGYRO) ignore `onReport45` and emit from `task()` reading `g_in[]`.

---

## 6. Puck personality — `puck_hid.cpp` / `puck_hid.h`  (MODE_STEAM + MODE_LIZARD)

The 28DE:1304 puck identity with four HID slot interfaces (interface N = bond slot N).

- **`PUCK_HID_DESC[]`** / **`PUCK_LIZARD_HID_DESC[]`** — the cloned puck report
  descriptors (mouse 0x40 + keyboard 0x41 + vendor FF00 with 63-byte feature reports).
- **`hid[NSLOT]`** — the four `Adafruit_USBD_HID` slot objects.

### usbd-task code (HID callbacks, 800-byte stack)
- **`handleSet(slot, rid, type, b, n)`** (via `setcb0..3`): host→device reports.
  - OUTPUT reports 0x80–0x89: stamp `g_steamAliveMs` (Steam-alive heartbeat),
    `hapLogAdd`, and for 0x80–0x86 relay to the controller via **`relayEnqueue`**
    (gated by `hapticRelaySlotOk`, `lizardActive`, post-resume mute, `haptic82Blocked`).
  - FEATURE report 0x01 = raw passthrough → `relayEnqueue(cmd,…)`; 0x9F → `hapticSendShutdown`.
  - FEATURE command channel: 0x83 (attrs), 0xAE (strings), 0xB4 (connection state),
    0xAD (pairing), 0xA2 (write/clear `g_slot[slot]` → sets `g_dirty`), 0xA3 (read slot).
    Each writes the reply into `S.resp[63]` / `S.resp_len`.
- **`handleGet(slot, rid, type, buf, reqlen)`** (via `getcb0..3`): returns the staged
  `S.resp` (≤63 bytes), stamps `g_steamAliveMs` in Steam mode, `hapLogAdd(0xFC,…)`.

### loop-task code
- **`onReport45(slot, rep, fresh, bodyTlen)`** (from rf_link): suspended/post-resume
  gating, then either `rfLizard` (lizard, slot 0 only) or `hid[slot].sendReport(0x45,
  rep+1, blen)` (`blen` clamped ≤ 45).
- **`onAuxReport(slot, rid, data, n)`**: forwards controller 0x43/0x44 verbatim to
  `hid[slot]` (how Steam reads battery).
- **`task()`**: post-resume edge stamp; per-slot connection presentation —
  edge-triggered `0x79` (1-byte conn state), periodic `0x7B` (12-byte status with RSSI
  patched from `g_linkRssi`), and a synthesized 14-byte `0x43` battery report from
  `g_battery`/`g_batteryState` every 2 s (for SDL's full-length requirement). Also
  `wakeNudgeTask()`.
- **`wakeEvent()`** / `wakeNudgeTask()`: queue + deliver a net-zero mouse jiggle to wake
  the host (prefers the boot wake-mouse; falls back to per-slot gamepad jiggle).
  `g_nudgeStep[NSLOT]`, `g_nudgeMs[NSLOT]`.

### Shared state owned here
- `g_steamAliveMs` (last Steam OUTPUT/heartbeat; written in usbd & loop) and the derived
  `lizardActive()` — **the single source of truth gating both the USB input path and the
  haptic relay**. `g_resumeMs` (post-resume mute). `g_fwdNewOnly`. `g_autoLizard`.
- **Cross-task**: `S.resp`/`S.resp_len` written in `handleSet` (usbd) and read in
  `handleGet` (usbd) — same task. `g_slot[slot].rec`/`used` + `g_dirty` written in usbd,
  flushed in loop. `g_steamAliveMs`/`g_resumeMs` written in both contexts, no lock.

---

## 7. Lizard desktop driver — `mode_lizard.cpp` / `mode_lizard.h`  (loop task)

- **`rfLizard(r, mdev, kdev, mrid, krid)`** — not an IController; called from
  `puck_hid::onReport45` (loop task). Turns a raw 0x45 into keyboard+mouse:
  - Right pad → mouse motion (velocity/friction/sub-pixel glide; dx/dy clamped ±127).
    `mdev->sendReport(mrid, &m, 5)` (`hid_mouse_report_t` = 5 bytes).
  - Left pad → wheel (dw clamped ±15). Buttons from pad-click / trigger thresholds.
  - Keyboard: modifiers + up to 6 keycodes; `krep[8]` (boot-keyboard layout),
    `kdev->sendReport(krid, krep, 8)`.
  - MODE_LIZARD-only consumer-control: `mdev->sendReport(0x03, &cc, 1)`.
- All state is loop-local statics; no USB callbacks; no shared cross-task state.

---

## 8. XInput personality — `mode_xinput.cpp` / `mode_xinput.h`  (`g_xboxCtl`)

Dynamic-mount, PUSH-style. Custom TinyUSB **vendor class driver** (0xFF/0x5D/0x01) plus a
boot mouse for the right pad.

- **usbd-task code (custom class driver `g_xiDriver`, returned via
  `usbd_app_driver_get_cb`)**: `xi_init`, `xi_reset`, `xi_open` (claims interface →
  `g_usbToBond[u]`, arms OUT endpoint with a 32-byte xfer), `xi_ctrl`, **`xi_xfer`**.
  `xi_xfer` decodes the host rumble OUT report (`outBuf[0]==0x00 && [1]==0x08`) and
  **calls `hapticSteamRumble(lo, hi, slot)` directly from the usbd task**, then re-arms
  the 32-byte OUT xfer. No HID feature/get/set callbacks in this mode.
- **loop-task code**: `onReport45` → `rfXboxGamepad(slot, rep)` (+ `rfXboxMouse` for slot
  0); `xinputSend` fills `S.inBuf` (**20 bytes** filled of the 32-byte buffer) and
  `usbd_edpt_xfer(0, epIn, r, 20)`; `task()` is the stuck-rumble watchdog (2.5 s →
  `hapticSteamRumble(0,0,s)`).
- **Buffers**: `XiSlot.inBuf[32]`, `XiSlot.outBuf[32]`; XInput report = 20 bytes;
  descriptor = `XINPUT_DESC_LEN` 40 bytes; mouse report = 5 bytes.
- **Cross-task**: `g_xiSlot[NSLOT]` — `inUse`/`rumbleLow`/`rumbleHigh`/`rumbleMs`
  `volatile`, but `itf`/`epIn`/`epOut` **not** volatile. usbd writes endpoint/rumble
  fields, loop reads them and touches the IN endpoint. No lock. `hapticSteamRumble`
  reached from **both** usbd (`xi_xfer`) and loop (`task`) — safety relies on the relay
  ring's PRIMASK guard.

---

## 9. Switch personalities

### `mode_switch_hori.cpp` / `mode_switch_hori.h`  (`g_switchHori`)
HORIPAD (0F0D:0092). Dynamic-mount, STREAM-style, plain 8-byte report, **no report
callbacks registered → no usbd-task user code at all**.
- `task()` (loop): rate-gated, `switchBuildHoripad(slot, p[8])` from `g_in[]`,
  `sendReport(0, p, 8)` (8 bytes, no report id). All build state loop-local.

### `mode_switch_pro.cpp` / `mode_switch_pro.h`  (`g_switchPro`)
Switch Pro (057E:2009). Dynamic-mount, STREAM-style + a full handshake/subcommand state
machine. **This mode registers an OUT/set-report callback** (`setReportCallback(NULL,
JC_SETCB[s])`).

- **usbd-task code (800-byte stack, the deepest USB-callback path in the firmware)**:
  `jcSet0..3` → `jcSetCommon` → `jcRumble` (decodes amplitudes, dedupes, calls
  **`hapticSteamRumble(lo,hi,bond)` from usbd**) and `jcSubcmd` (builds a 63-byte 0x21
  reply: BT pairing, device info, SPI read via `spiRead`, SPI write via `jcSpiWrite`,
  report-mode set). `jcSubcmd`'s local `p[63]` plus `jcInputPrefix`/`spiRead` is the
  heaviest stack user on the 800-byte stack. `jcEnq` (usbd) is the **ring producer**.
  `jcSpiWrite` is **RAM-only** (writes `g_userCal[slot][]`, sets a dirty flag) —
  deliberately no flash I/O in the callback.
- **loop-task code**: `beginPool` (LittleFS loads, descriptor setup), `task()` —
  deferred flash save (`saveUserCal` when dirty + 250 ms), drains **one** queued reply
  per slot via `sendReport(rid, data, 63)`, else rate-gated `switchProBuild(slot, p[63])`
  + `sendReport(0x30, p, 63)`.
- **Buffers**: `JC_REPLEN = 63` (all input/reply reports are 63 payload bytes);
  reply ring `g_jcQ[NSLOT][8]` of `{rid, data[63]}`; `g_userCal[NSLOT][0x100]`;
  `g_spiStickCal[18]`; `g_jcMac[NSLOT][6]`; config files `b[3]`.
- **Cross-task shared data**:
  - Reply ring `g_jcQ` / `g_jcQh` / `g_jcQt` — SPSC (usbd produces, loop consumes);
    head/tail `volatile`, ring body **not** volatile.
  - `g_swProReportMode[]` — written usbd (`jcSubcmd`) + loop (`mountSlots`), read loop;
    **not volatile**.
  - `g_userCal[][]` + `g_userCalDirty`/`g_userCalDirtyMs` — `g_userCal` written/read in
    usbd, read in loop (`saveUserCal`); only the dirty flag/ts are `volatile` → a
    loop-side save can race a usbd-side write (no lock).
  - `g_jcTimer[]` — incremented in both contexts, not atomic (benign).
  - `g_in[bond]` — read from both `jcInputPrefix` (usbd) and `switchProBuild` (loop).

---

## 10. PlayStation personalities

### `mode_ps5.cpp` / `mode_ps5.h`  (`g_ps5Ctl`, MODE_PS5 + MODE_PS5_GAME)
DualSense (054C:0CE6). Dynamic-mount, STREAM-style. **Registers get+set report
callbacks** (`setReportCallback(PS5_GETCB[s], PS5_SETCB[s])`).
- **usbd task**: `ps5GetCommon` (GET_FEATURE: 0x03→47 B, 0x05 motion-calib via
  `psNeutralCalib` writing through buf[33]→returns 40, 0x09 MAC 6 B→19, 0x20 fw→63 B;
  each `memset(buf,0,reqlen)` first); `ps5SetCommon` (OUTPUT 0x02 rumble →
  `hapticSteamRumble(p[3]*257, p[2]*257, bond)` from usbd, reads `g_usbToBond[slot]`).
- **loop task**: `task()` rate-gated → `ps5Build(u, bond, p[63])` from `g_in[bond]` (per-slot
  `seq[]`), `sendReport(0x01, p, 63)`. Touch packed at `out+32` (8 bytes).
- **Buffers**: input report `p[63]`; `g_ps5Mac[NSLOT][6]`.

### `mode_hidgyro.cpp` / `mode_hidgyro.h`  (`g_hidGyroCtl`, MODE_HIDGYRO + MODE_DS4_GAME)
DS4-layout (054C:05C4) + gyro. Same structure as PS5.
- **usbd task**: `hidGyroGetCommon` (GET_FEATURE: 0x02 calib via `psNeutralCalib`→36,
  0x12 MAC 6→15, 0x81 MAC 6→6, 0xA3 fw→48); `hidGyroSetCommon` (OUTPUT 0x05 rumble →
  `hapticSteamRumble(p[4]*257, p[3]*257, bond)` from usbd).
- **loop task**: `task()` → `hidGyroBuild(u, bond, p[63])` from `g_in[bond]`,
  `sendReport(0x01, p, 63)`; touch via `touchPackPads(out+34,…)` (8 bytes).
- **Buffers**: input report `p[63]`; `g_ds4Mac[NSLOT][6]`.

**Both PS modes**: `g_usbToBond[]` read in usbd without a lock (bounds-checked); the
calib helper `psNeutralCalib` writes 34 bytes (buf[6..33]) while callers return 36/40
(remainder zeroed by the prior memset).

### `gamepad_util.cpp` / `gamepad_util.h` — shared report-builders (called from loop task)
`swStick`, `psNeutralCalib` (writes through `buf[33]`), trackpad→touch mappers
(`touchPackPads` writes 8 bytes = two 4-byte points), `psButtonsFromSteam` (back-paddle/
QAM/chord remap), `psShouldersByte`, `psHatNibble`, `psFaceNibble`, `tritonFromCode`.
Reads `g_qamMap`/`g_abSwap`/`g_back[]`. Pure transforms, no buffers beyond callers'.

---

## 11. Haptics & host→controller relay — `haptics.cpp` / `haptics.h`

**The central cross-task data structure: the relay rings.**

- **`g_rq[NSLOT][RELAY_QLEN]`** (`RELAY_QLEN = 32`), `struct RelayMsg { uint8_t rid, len;
  uint8_t data[RELAY_MAXP=60]; }`, with **`volatile g_rqHead[NSLOT]` / `g_rqTail[NSLOT]`**.
  One ring per bond slot. **Multi-producer** (usbd-task HID callbacks: puck_hid
  `handleSet`, the mode rumble callbacks; plus loop-context console/test injectors),
  **single-consumer** per slot (`rfConnFlushRelay` on that slot's poll turn, loop task).
- **`relayEnqueue(rid, payload, plen, slot)`**: producers serialize under
  `__get_PRIMASK()` / `__disable_irq()` / `__set_PRIMASK`. `slot==0xFF` broadcasts to all
  rings. Full ring → evict the **oldest** (keeps the most recent meaningful frame).
- **`rfConnFlushRelay(ch, s1)`** (loop, from `rfConnStep`): drains **one** entry from
  `g_curSlot`'s ring, frames it as an E3 sub-TLV (type-01 form for 0x9F/LED-0x2D,
  legacy type-05 otherwise), `hapLogAdd(0xFE,…)`, TXes via `rfConnTx(…, 80)` (no-ACK).
- **`rfConnQueueHapticRelay()`** (loop): injects test/stop haptics only on an idle cycle.

### Other haptic state (mostly loop task, some `volatile` flags)
- `g_testHaptic`, `g_hapticStop` (`volatile`), `g_hapticBlockOn`, `g_hapticBlockMs`,
  `g_hapticBlockUntil[NSLOT]`, `g_relayOp`, `g_relaySub`.
- `hapticSendShutdown()` — bursts 0x9F "off!" (`{6f 66 66 21}`) ×3 broadcast.
- `hapticSteamRumble(low, high, slot)` — builds a 9-byte 0x80 report (×`g_rumbleScale`),
  `relayEnqueue(0x80, p, 9, slot)`; **called from usbd (mode rumble callbacks) and loop**.
  Per-slot stuck-rumble tracking `g_rumble80On/Ms[NSLOT]`.
- `haptic82Blocked`/`hapticLinkUp`/`hapticRelaySlotOk` — link-up + block gates (read
  `g_connReplyMs`, `g_slot`).
- `hapticOnReconnect(slot)` — arms the per-slot block + schedules `g_reinitLeft` re-init
  shots; `hapticCancelPendingOn` voids queued ON entries under PRIMASK.
- `hapticReinit(slot)` — enqueues the captured 0x81/0x87 settings re-init sequence.
- **`hapticTask()`** (loop): per-slot link-edge detect → `hapticOnReconnect`; fires
  scheduled re-init; **autonomous controller power-off on a persisted USB suspend**
  (`SUSPEND_OFF_MS = 4000` ms, VBUS-gated) → `hapticSendShutdown`; stuck-rumble watchdog
  (2.5 s); idle-clear re-init.
- **`hapInit()`** resets all rings/flags at boot.

### Diagnostic capture ring (`#if OPK_LOG` only)
- `g_hapLog[HAPLOG_N=4096]` of `{ms, slot, rid, n, b[16]}` (~80 KB). `hapLogAdd` written
  from **both usbd and loop**, guarded by PRIMASK; `g_hapHead` is `uint16_t`. Drained to
  WebUSB via `hapLogResetDrain`/`hapLogPull` (loop) and `'H'` console dump. Compiled out
  entirely in production (`OPK_LOG=0`), reclaiming the RAM.

---

## 12. USB infrastructure

### `usb_mount.cpp` / `usb_mount.h` — dynamic re-enumeration watcher (loop task)
- **`g_usbMountCount`**, **`g_usbToBond[NSLOT]`** (usbSlot→bond, -1 unused),
  **`g_bondToUsb[NSLOT]`** (inverse). `g_usbToBond` is the routing map the usbd-task
  rumble callbacks read to map a USB output back to a bond slot.
- **No TinyUSB callbacks here** despite the name — it's a poll-driven state machine.
- `usbMountTask()` (loop): `connectedMask()` (slots with a reply < `CONN_UP_MS=1200` ms),
  debounce `MOUNT_DEBOUNCE_MS=2000` ms, and on a steady change `commitOrder()` (rewrites
  both maps + count) then **`usbReenumerate(g_usbMountCount)`** (detach/rebuild/attach,
  no reboot). Order-preserving so existing instances keep their index.
- No `delay()` here (the detach delay lives in `usbReenumerate`). Maps rewritten
  non-atomically just before re-attach; consumers tolerate the brief window.

### `wake_hid.cpp` / `wake_hid.h` — boot-mouse wake source
- `WAKE_HID_DESC` = boot mouse, `g_wakeHid`, `g_wakeHidPresent`.
- `wakeHidBegin` (setup), `wakeHidAddInterface` (re-enumerate build), `wakeHidPresent`,
  `wakeHidReady`, `wakeHidMove(dx,dy)` → `g_wakeHid.mouseReport(0,0,dx,dy,0,0)`
  (buttons always 0). Poll interval 10 ms. No buffers/delays/loops; only shared state is
  the one flag set once at begin.

### `webusb_config.cpp` / `webusb_config.h` — browser config channel (loop task)
- `Adafruit_USBD_WebUSB usb_web`. **No vendor callbacks** — fully **polled from
  `webusbPoll()` in `loop()`** (the library buffers the vendor RX). So nothing here runs
  on the usbd 800-byte stack.
- `webusbSendBlob()` builds a **115-byte** status blob (`p[2+WB_PAYLEN(113)]`, header
  `A5 113 11`) reading a large set of diagnostic globals (`g_curSlot`, `g_connReplyMs`,
  `g_battery`, `g_linkRssi`, `g_in[0]` IMU, `g_slot`, `g_type`, loop/poll stats, …) —
  unsynchronized reads of RF-owned state, but cosmetic (torn reads at most).
- `webusbPoll()` command parser (`static buf[16]`): `0x01` send blob; `0x02 field value`
  mutates config tunables (writes `g_type`, `g_mDiv/Fric`, `g_rxWin` (clamped 600..3000),
  `g_hapticBlockMs` (≤60 s), etc., then `applyActiveType`/`saveCfg`/`swProSaveCfg`);
  `0x03 mode`→`saveMode`+reset; `0x07`→`hapticReinit`; `0x08`→`hapticSendShutdown`;
  `0x0A` ("ERS")→`factoryErase`+reset; `0x0B/0x0C`→DFU; capture drain (`OPK_LOG`).
  Each reset/DFU path does `delay(40)` then `NVIC_SystemReset()` (harmless — reboots).

### `fw_update.cpp` / `fw_update.h` — staged firmware update (loop task + boot)
- WebUSB ops `0x20 begin / 0x21 chunk / 0x22 verify+commit / 0x24 abort` stage a new app
  image into unused flash below the meta page at `0xEC000` (top of the app region);
  strictly sequential offsets, duplicate chunks re-acked without rewriting (flash words
  are written once per erase). One page erase max per `loop()` pass (~85 ms CPU stall).
- `fwupEnd()` CRC32s the **staged flash** (heartbeat-beaten so the wedge reporter stays
  quiet), sanity-checks the vector table, then commits the one-page meta record — the
  single arming action. Idempotent on retry (lost-ack safe).
- `fwupApplyIfArmed()` (first line of `setup()`): re-validates everything, then
  `ramApply()` — a `.data`-section (RAM-resident) copier, IRQs off, WDT force-started
  and fed — erases the app (vector page FIRST), writes it (vector page LAST, first word
  dead-last), verifies word-for-word, rewrites the bootloader settings page to the
  drag-and-drop state (`BANK_VALID_APP`, crc-check off), erases meta, resets. Any
  interruption leaves the board app-less → UF2 bootloader, never a half image.

### `serial_console.cpp` / `serial_console.h` — CDC debug CLI (loop task)
- `serialConsolePoll()` drains available CDC bytes into `static char line[24]` (bounded),
  dispatches single-letter commands on newline (first match wins). Commands mutate the
  radio tunables, rf_link toggles, rf_diag flags, config (`E/F/W/K/Q` → `saveCfg`), and
  inject relays (`J`/`t`). No blocking reads; `delay()` only before resets.
- Note: `C` and `H` each appear twice in the chain — the **second occurrence of each is
  dead code** (unreachable).

---

## 13. Status & fault

### `status_led.cpp` / `status_led.h` (loop task)
Drives two GPIO pins (LED_BUILTIN + pin 24) together. `ledWakePulse()` lights them and
stamps `g_pulseMs`; `ledTask()` clears after `PULSE_MS=500`. No buffers/delays/radio.

### `fault_diag.cpp` / `fault_diag.h`
- **`HardFault_Handler()` (ISR / exception context)** — a **strong override** of the
  weak default: sets `GPREGRET2 = 0xFA` then `NVIC_SystemReset()`. The only ISR/handler
  in the codebase.
- `faultDiagArmIntentionalReset()` sets `GPREGRET2 = 0xB1` before a deliberate reset.
- `faultDiagBoot()` (loop) reads `RESETREAS` + `GPREGRET2`, classifies the last boot
  (PIN / WATCHDOG / LOCKUP / HARDFAULT / REBOOT / SOFT / WAKE / POWERON / UNKNOWN),
  clears `GPREGRET2`. Accessors `faultDiagReason`/`…ResetReas`/`…ReasonStr` feed WebUSB.

---

## 14. Cross-task shared state — consolidated

State touched by **both** the loop task and the usbd task (the synchronization-critical set):

| State | Writer(s) | Reader(s) | Synchronization |
|---|---|---|---|
| **`g_rq[NSLOT][32]` relay rings** + `g_rqHead/Tail` (haptics) | usbd HID callbacks (`handleSet`, mode rumble cbs) + loop (console/test/`hapticTask`) | loop `rfConnFlushRelay` | **Producers under PRIMASK**; `volatile` head/tail; ring body not volatile; SPSC-drain per slot |
| **`g_slot[NSLOT].rec/used/resp`** (bonds) | usbd `handleSet` (0xA2 / reply staging) + loop (`loadBonds`) | loop RF/webusb/console, usbd `handleGet` | `resp` is same-task (usbd); `g_dirty` (`volatile`) defers the flash write to loop |
| **`g_dirty`** | usbd `handleSet` | loop `loop()` → `saveBonds` | `volatile` flag |
| **`g_in[NSLOT]`** (decoded input) | loop `rf_link` decode | loop mode `task()`/`onReport45`; **usbd `jcInputPrefix` (switch pro)**; webusb | none (single writer; readers tolerate staleness) |
| **`g_connReplyMs[NSLOT]`** | loop `rf_link` | loop puck_hid/usb_mount/haptics/webusb; usbd haptic gates | none (plain `unsigned long`) |
| **`g_battery/g_batteryState/g_linkRssi[NSLOT]`** | loop `rf_link` | loop puck_hid `task`/webusb | declared `volatile` |
| **`g_usbToBond[NSLOT]`** (usb_mount) | loop `usbMountTask`/`commitOrder` | **usbd rumble callbacks** (ps5/hidgyro/xinput) + loop modes | none; bounds-checked; non-atomic rewrite just before re-attach |
| **`g_jcQ`/`g_jcQh`/`g_jcQt`** (switch pro reply ring) | usbd `jcEnq` | loop `task()` | `volatile` head/tail, SPSC; body not volatile |
| **`g_userCal[NSLOT][0x100]`** + dirty flags (switch pro) | usbd `jcSpiWrite` | usbd `spiRead`, loop `saveUserCal` | only dirty flag/ts `volatile`; `g_userCal` itself unguarded (loop save can race usbd write) |
| **`g_xiSlot[NSLOT]`** (xinput) | usbd class driver (eps, rumble) | loop `xinputSend`/`task` | `inUse`/`rumble*` `volatile`; `itf`/`epIn`/`epOut` not volatile; no lock |
| **`g_swProReportMode[NSLOT]`** | usbd `jcSubcmd` + loop `mountSlots` | loop `task` | **not volatile** |
| **`g_steamAliveMs` / `g_resumeMs`** (puck_hid) | usbd `handleSet`/`handleGet` + loop `task` | both | none |
| **`g_hapLog[4096]`** (`OPK_LOG`) | usbd + loop `hapLogAdd` | loop drain | PRIMASK in `hapLogAdd` |
| **`rfrx[100]`/`rftx[100]` + `NRF_RADIO`** | loop `rf_link` AND loop `rf_diag` | same | **no guard** — relies on diag-modes-off assumption (both are loop task, so no preemption between them, but no arbitration of radio state) |

**`hapticSteamRumble()` is the one app function legitimately entered from the usbd task**
(via the ps5/hidgyro/xinput/switch-pro rumble callbacks) as well as the loop task; its
safety rests entirely on `relayEnqueue`'s PRIMASK guard.
