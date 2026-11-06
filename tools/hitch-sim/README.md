# hitch-sim

Host simulator for the stutter-aware channel policy (`OpenPuck/hitch_policy.cpp`). It runs the same policy
code the firmware ships against a model of the puck-to-controller link at the 250 Hz poll rate, in nine kinds
of room (Wi-Fi traffic, microwaves, bursty devices, weak signal). Each room's parameters are randomised per
seed, so no policy is judged on one hand-picked layout.

It compares four policies:

- **0.9.59 (old)**: fixed channel, no retry
- **0.9.67 (current)**: one immediate retry, plus recovery when average delivery stays under 55% for 3 s
- **Hitch-aware (new)**: retry + `HitchPolicy` with its own total-failure backstop
- **Hitch-aware + 0.9.67 backstop (firmware)**: what this firmware runs: `HitchPolicy` for stutters, the
  existing average-delivery recovery as the backstop

Metrics are what a player feels: stutters (gaps of 40 ms or more between fresh reports), freezes (100 ms or
more) and the worst freeze per session.

## Run

```sh
g++ -O2 -std=c++17 -I OpenPuck tools/hitch-sim/sim.cpp OpenPuck/hitch_policy.cpp -o hitch-sim
./hitch-sim [seeds=30] [minutes=30] [room index]   # CSV on stdout, about a minute with the defaults
```

Environment knobs for sensitivity checks: `CAPTURE_DB` (co-channel rejection, default 8), `HOP_GAP_MS`
(cost of a channel move), `RANDOM_START` (start on a random channel instead of 18).

## Results (defaults: 30 seeds x 30 minutes per room)

| Room | Policy | Stutters/min | Freezes/min (≥100 ms) | Worst freeze p90 (ms) | Channel moves |
|---|---|---:|---:|---:|---:|
| Quiet house | Fixed channel | 0.00 | 0.000 | 28 | 0.0 |
| Quiet house | Current (retry + recovery) | 0.00 | 0.000 | 24 | 0.0 |
| Quiet house | This PR | 0.00 | 0.000 | 24 | 0.0 |
| Home with someone streaming | Fixed channel | 1.60 | 0.000 | 76 | 0.0 |
| Home with someone streaming | Current (retry + recovery) | 0.30 | 0.000 | 60 | 0.1 |
| Home with someone streaming | This PR | 0.10 | 0.000 | 60 | 0.1 |
| Crowded apartment | Fixed channel | 9.98 | 0.097 | 152 | 0.0 |
| Crowded apartment | Current (retry + recovery) | 1.18 | 0.003 | 88 | 0.8 |
| Crowded apartment | This PR | 0.15 | 0.000 | 68 | 0.4 |
| Every channel busy | Fixed channel | 45.46 | 0.784 | 200 | 0.0 |
| Every channel busy | Current (retry + recovery) | 4.58 | 0.019 | 116 | 8.9 |
| Every channel busy | This PR | 0.64 | 0.001 | 92 | 2.3 |
| Quiet house + microwave | Fixed channel | 0.00 | 0.000 | 28 | 0.0 |
| Quiet house + microwave | Current (retry + recovery) | 0.00 | 0.000 | 20 | 0.0 |
| Quiet house + microwave | This PR | 0.00 | 0.000 | 20 | 0.0 |
| Weak signal (far/blocked) | Fixed channel | 40.03 | 4.440 | 768 | 0.0 |
| Weak signal (far/blocked) | Current (retry + recovery) | 3.20 | 0.191 | 352 | 2.2 |
| Weak signal (far/blocked) | This PR | 3.24 | 0.192 | 352 | 2.3 |
| Neighbor starts streaming | Fixed channel | 2.91 | 0.009 | 100 | 0.0 |
| Neighbor starts streaming | Current (retry + recovery) | 0.50 | 0.000 | 80 | 0.5 |
| Neighbor starts streaming | This PR | 0.24 | 0.000 | 68 | 0.4 |
| Weak signal + crowded | Fixed channel | 50.11 | 2.906 | 584 | 0.0 |
| Weak signal + crowded | Current (retry + recovery) | 4.10 | 0.150 | 348 | 5.4 |
| Weak signal + crowded | This PR | 2.88 | 0.119 | 268 | 3.8 |
| Bursty device (camera/backup) | Fixed channel | 2.01 | 1.110 | 772 | 0.0 |
| Bursty device (camera/backup) | Current (retry + recovery) | 1.91 | 1.084 | 772 | 0.0 |
| Bursty device (camera/backup) | This PR | 0.09 | 0.019 | 264 | 0.3 |

Where every move would be wasted (weak signal, nothing better to move to) it stays at parity with the
current recovery rather than adding moves. These are model results; the firmware also reports the same
measures live (WebUSB 0xB5, the panel's Connection quality card), so they can be checked on real hardware.
