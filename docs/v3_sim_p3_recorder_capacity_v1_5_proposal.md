# P3 contract `p3-acceptance-v1.5` — proposal (raw depth recorder capacity 8 → 32)

**Status: PROPOSAL ONLY — NOT APPROVED, NOT APPLIED.** Nothing was changed, built, frozen or
run for this proposal. P3 remains INCOMPLETE, qualification 0/3.

Basis: HEAD after `aa64445` and the retained v1.4 Gate B run
`20261025T133925Z-p3-flight-b4436533` (acceptance report section AC).

v1.5 would change exactly one sizing value. Everything in v1.4 stays, including
`recorder_overflow_allowed = 0`, `recorded_fraction_required = 1.0`, `stall_wall_s = 1.0`, every
other numeric limit, the sensors' SDF, resolution and rate, the renderer and shutdown profile,
the RGB scope, the P2 controller, mission and configuration, and gate sequencing.

## Evidence from v1.4 Gate B

* The flight completed (`COMPLETE`) and everything except depth recording passed: P2 flight
  acceptance, `vehicle_land_detected` maximum gap 1.076 s, RTF 0.995, MC verified, RGB absence
  verified, native commands under policy A, including the six v1.4 landing self-commands.
* From monotonic 138811.3 to 138814.4 s (sim ≈ 44.4–47.5 s, `EAST_TEST`), the CPU rate of every
  owned process rose 1.5–1.8× together. That includes Gazebo, the adapter, PX4, the
  controller, the bridge, QGC and the probe, on 24 cores with nothing saturated and memory
  steady. It looks like a host-level slowdown. The v1.3 flight shows no such event at the same
  mission point.
* The recorder's writer thread fell behind. One write took 0.209 s, and the time from receipt
  to write reached 0.544 s (p50 0.013 s over the run, the same as in v1.3). The queue filled
  (`max_queue` 8), and **9 frames overflowed between sim 45.180 and 45.676 s**. ROS delivered
  all 3,301 frames (maximum receipt gap 0.139 s wall). The loss was only between adapter receipt
  and the raw file. The adapter correctly reported `raw_recorder_overflow` and exited 3.

## Proposed change

| File | Change |
|---|---|
| `configs/p3_sensors.yaml` | `"recorder_capacity": 8` → `"recorder_capacity": 32` (currently `6c3038fe…`; the only edit) |
| `configs/p3_validation.yaml` | `contract: p3-acceptance-v1.5` and a `contract_revision` sentence; no other key changes |
| tests | Contract identity is v1.5; `recorder_capacity == 32`; every other `p3_sensors.yaml` key and every numeric limit in `p3_validation.yaml` is identical; `recorder_overflow_allowed == 0`; a recorder unit test shows that 32 queued frames are accepted and the 33rd overflows (the overflow stays sticky and marks the recording incomplete) |

**Sizing.** The capacity equals the existing 1.0 s `stall_wall_s` at the measured source rate:
⌈1.0 s × 30.31 Hz⌉ = 31 frames, plus 1 for the frame being compressed, gives **32**. A writer
stall longer than the stall limit still overflows and still fails, so no loss can be hidden. The
observed 0.544 s backlog needed about 17 frames, so 32 leaves about 1.9× headroom.

**Cost.** Each queued item holds one 640×480 float32 payload (1,228,800 bytes), so the worst case
is about 39 MB more resident memory than the 8-frame queue. The adapter's RSS was about 182 MB in
the failed run. The writer, zlib-1 compression, hashing and fsync-on-close are unchanged.

**Build and candidate impact.** `p3_sensors.yaml` is outside the sensor package, so the package
hash (`c5d37413…`) does not change and the package needs no rebuild. The adapter validates the
count range 1–1000 at load. The candidate source hash does change, so the sequence is: approval,
apply and regress, commit, Policy A re-approval for the new commit (policy content unchanged),
freeze, then Gates A, B and C under separate authorizations.

## What v1.5 does not fix (operator attention)

The same slowdown also delayed the PX4 state messages. Four in-window observations (sim
45.212–45.476 s) were associated at processing time as `attitude:pending_newer_state` ×3 and
`position:pending_newer_state` ×1. The evaluator correctly failed them
(`state_association_incomplete`). **Under v1.5 the same run would still fail Gate B on that
check.** The capacity change removes only the recorder loss.

Options for that, **not proposed here** and each requiring its own decision:
1. No change: treat a repeat host slowdown of this size as a legitimate failure.
2. An adapter behaviour change: defer association of a processed observation until a newer
   state sample arrives, bounded by the existing `observation_age_sim_s` (0.25 s), instead of
   associating immediately. This is a functional change to the adapter and needs its own proposal
   and tests. `state_mismatch_s` (0.05 s) would remain unchanged.

## What v1.5 does not change or claim

* No threshold, freshness, sensor rate or resolution, renderer, shutdown, MC, cleanup, RGB or
  native-command rule changes. `recorder_overflow_allowed` stays 0.
* The v1.4 Gate B run stays failed and is not re-evaluated. The cause of the host slowdown was
  not identified from run evidence.
