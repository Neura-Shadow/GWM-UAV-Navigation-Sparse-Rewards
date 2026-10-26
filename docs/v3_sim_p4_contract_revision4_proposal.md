# P4 contract `p4-acceptance-v1` — revision 4 proposal (transit speed versus the tracking bound)

**Status: APPROVED by the operator (2026-10-26) exactly as written in `65b99e4`; APPLIED** to the
[contract](v3_sim_p4_contract.md). It amends revisions 2 and 3 (commits
`d3c06b4`, `ea2c9e3`). It changes one speed: P4's own `TRANSIT` target speed becomes 0.20 m/s.
P2's 0.3 m/s ramp, every threshold, limit, scenario and gate are unchanged. Nothing was launched to
prepare this proposal.

## Problem

Gate B of candidate `c932b212` failed on one check, with everything else correct
([plan §32](v3_sim_p4_plan.md)): horizontal tracking during `TRANSIT` was 0.342 m against the
contract's ≤ 0.30 m (§6). The S2 flight itself completed its detour and reached the goal (0.069 m),
with 0 collisions and 0.457 m minimum truth clearance.

The bound cannot be met at the current transit speed:

* P4 advances the target at P2's `ramp_m_s` of 0.3 m/s (L7 "≤ 0.3 m/s", implemented at the
  maximum). It publishes position-only `trajectory_setpoint`s from the accepted P2 helper, so
  velocity and acceleration are NaN.
* PX4's horizontal position loop is proportional. The stock `MPC_XY_P` is 0.95, and the x500
  airframe does not override it. A position target moving at constant speed `v` therefore settles
  at a lag of **`v / MPC_XY_P`**, which at 0.3 m/s is 0.316 m.
* The flight matches that exactly. The error plateaus at 0.31–0.33 m in every straight segment,
  peaks at 0.342 m in turns, and falls to about 0.03 m whenever the target holds.

## Evidence

**SITL (Gate B S2, run `20261026T033114Z-p4-flight-S2-8215374e`):** 382 of 620 `TRANSIT` decisions
were above 0.30 m. The steady plateau was ≈ 0.316 m, and the turn overshoot was about 0.026 m above it.

**Offline speed study** (scratch, no runtime). This is the production perception, planner and
supervisor on the independent renderer, with the vehicle following the target through a
first-order horizontal loop `v = 0.95 · error`. At 0.3 m/s the model reproduces the measured
plateau (0.308 m against 0.316 m in SITL):

| Scenario | Expected | 0.3 m/s: outcome, transit, max tracking | 0.20 m/s: outcome, transit, max tracking | Min clearance at 0.20 m/s |
|---|---|---|---|---|
| S1 | goal | goal, 28.5 s, 0.308 m | goal, 41.4 s, **0.205 m** | — |
| S2 | goal with detour | detour → goal, 31.7 s, 0.308 m | detour → goal, 45.4 s, **0.205 m** | 0.625 m |
| S3 | goal with detour | detour → goal, 29.2 s, 0.308 m | detour → goal, 42.5 s, **0.205 m** | 0.641 m |
| S4 | goal with detour | detour → goal, 33.9 s, 0.308 m | detour → goal, 45.1 s, **0.205 m** | 0.625 m |
| S5 | safe refusal | HOVER → ABORT (hover timeout) | HOVER → ABORT (hover timeout) | 0.975 m |

S6 is not in the study. Its depth-loss path does not depend on speed: the bridge stop is injected at
a travelled distance, so a slower target only injects it later, and the stale-input `HOVER` freezes
translation.

At 0.20 m/s the expected SITL tracking is 0.20 / 0.95 = 0.211 m steady. With the measured turn
overshoot scaled to that speed (0.342 × 0.2 / 0.3) it is **≈ 0.23 m**, leaving about 0.07 m of margin
to the unchanged 0.30 m bound. The supervisor transitions are identical at both speeds, and the
clearances are equal or larger.

## Proposed changes (exact)

1. **New pinned P4 parameter:** `mission.transit_speed_m_s` = **0.20** in `p4_avoidance.yaml`,
   pinned in `contracts.PINNED`. It is the speed at which a `TRANSLATE` target advances during
   `TRANSIT`.
2. **§5 L7** becomes: "A translating target advances along the selected segment at the pinned
   `TRANSIT` speed of 0.20 m/s (`mission.transit_speed_m_s`; never above P2's 0.3 m/s ramp). Any
   action is re-validated each tick, and a failed re-validation yields `HOLD`."
3. **§3, P2 timing and safety:** append to "setpoint ramp 0.3 m/s": "(P2 phases; P4 `TRANSIT` uses
   the slower 0.20 m/s of L7)."
4. **§6 table:** add the row "`TRANSIT` target speed | 0.20 m/s (steady PX4 lag 0.20 / `MPC_XY_P`
   0.95 ≈ 0.21 m, inside the 0.30 m tracking bound)".
5. **Contract header and `p4_validation.yaml` `contract_revision`:** these will name revisions 2, 3
   and 4.

## Unchanged

* P2's `p2_control.yaml` (including `ramp_m_s` 0.3 and `yaw_ramp_deg_s` 10), and every P2/P3 file
  and manifest. The P3 candidate `1e205fec…` stays verifiable.
* Every §6 threshold: tracking ≤ 0.30 m, minimum clearance ≥ 0.20 m, 0 collisions, response latency
  ≤ 0.25 s, setpoint validity, and §6.1 performance. Also `r_infl`, `d_stop`, the trigger range,
  the free cap, the geofence, the goal, the `MAP_SCAN`, scenarios S1–S6, Gates A/B/S/C and the
  deadlines.
* The claim in §1 ("S1–S6 at ≤ 0.3 m/s") still holds; 0.20 m/s is inside it. §10 decision 2 ("P2
  … speed and yaw-rate limits stay") still holds: the P2 limit is unchanged and P4 runs below it.
* **Conservative by construction:** a slower target lowers the distance travelled within the 0.25 s
  response latency and between truth samples. The truth-sample allowance of 0.015 m stays, although
  0.20 m/s needs only 0.010 m. The `d_stop` margin and perception coverage per metre (one frame per
  0.02 m instead of 0.03 m) also improve.

## Timing impact

`TRANSIT` grows by about 13 s in the offline study (S2: 31.7 → 45.4 s; longest 55 s for S5 before
its hover timeout). The 150 s `TRANSIT` deadline and P2's 240 s simulation deadline are unchanged
and not approached. The worst case is still capped by the deadline, as in revision 3.

## Implementation on approval (offline, then the usual gates)

* `p4_avoidance.yaml` and `contracts.PINNED`: add `mission.transit_speed_m_s` = 0.20.
* `mission.py`: `TRANSIT` passes `p4["mission"]["transit_speed_m_s"]` to `supervisor.apply_action`
  instead of P2's `ramp_m_s`. Every P2 phase is untouched.
* `tests/p4_synthetic.py` closed loop: take the pinned speed. Tests will pin the value, prove that
  `TRANSIT` uses it and P2 phases do not, and add the first-order tracking study as a regression:
  max tracking ≤ 0.30 m at the pinned speed, and above 0.30 m at 0.3 m/s.
* `p4_validation.yaml` `contract_revision` names revisions 2, 3 and 4.
* The package is rebuilt (because `mission.py` changes), then a new commit, Policy A approval, a
  freeze, and the gates from **Gate A** onward. Candidate `c932b212` has consumed its Gate B, and a
  new manifest needs its own Gate A.
