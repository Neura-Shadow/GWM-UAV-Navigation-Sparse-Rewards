# v3-SIM P6 — final acceptance (fault injection and safety validation)

**P6 COMPLETE (2026-11-15).** Contract `p6-acceptance-v1` ([contract](v3_sim_p6_contract.md),
[plan](v3_sim_p6_plan.md), [fault matrix](v3_sim_p6_fault_matrix.md)), with the operator decisions recorded in
contract §10.

The real candidate `22292775` passed Gates A, B, S and C:
* each gate ran once, under its own operator authorization;
* there were no retries;
* PX4 SITL only, with no hardware.

The accepted P5 stack ran unchanged throughout. The P3, P4 and P5 baselines still verify. No file under the P2–P5
trees has changed since P5 was accepted (`git diff 1f06f39 HEAD` over those trees is empty, across 60 commits).

M7 (plan §7) is this report. It covers:
* the coverage report (`docs/evidence/v3_sim_p6_22292775_coverage.json`);
* the A/B/C comparison and coverage by class and tier (`docs/evidence/v3_sim_p6_22292775_m7_comparison.json`, made
  by `diagnostics/p6_m7/m7_arms.py` from the retained evidence alone);
* the baseline re-verification.

## Accepted candidate

| Item | Identity |
|---|---|
| P6 manifest (canonical) | `222927752a507fdb2fb0ef13f44d6ce8163d50b7d22f405ebbf27df6c21bfa41` (`~/uav_autonomy/state/p6-candidate-8e29f3f.json`, file sha256 `f4474805…`) |
| Frozen at | git `8e29f3f`. The 41 bound P6 files (sources, specs, injectors, tests) are those of `c08bfc8`, canonical digest `68b45c83…`. |
| Specs | faults `97fc5eef…`, scenarios `4e06ab00…` |
| Baselines | P3 `1e205fec…` (`p3-acceptance-v1.6`), P4 `c110bd30…`, P5 `2becfbbb…`. The P5 model `ee006a80…` and build `e7a59f35…` are unchanged. |
| Policy A | `bf1ef9b1…b3b0`. Production approval `state/p6-policy-A-approval-c08bfc8.json` (sha256 `994e4801…`): Gate A on the ground, and Gates B, S and C in simulated flight only. |
| Runtime | PX4 v1.17.0 with the MC causal-trace overlay; QGC unattended `82994283…`; Gazebo Harmonic; ROS 2 Jazzy; WSL2 (24 vCPU); `hardware:d3d12`. The Windows AC overlay was Best performance (`ded574b5`) and the host was on AC, checked read-only before every gate. |

## Gates (operator Neura-Shadow, 2026-11-15)

| Gate | Rows | Ran (UTC) | Authorization (sha256) | Result record (sha256) |
|---|---|---|---|---|
| A: ground readiness and fault fixtures | 18 of 18 | 07:40:35–08:00:45 | `7d014f10…` | `p6-gate-A-22292775….result.json` (`358ecf5d…`) |
| B: smoke | 2 of 2 | 09:08:24–09:14:32 | `8316fac1…` | `p6-gate-B-22292775….result.json` (`7bf387d1…`) |
| S: safety-critical faults | 5 of 5 | 10:12:01–10:25:10 | `0f854af0…` | `p6-gate-S-22292775….result.json` (`169b5d34…`) |
| C: fault-tolerance qualification | 5 of 5 | 12:03:37–12:19:49 | `53e6e23b…` | `p6-gate-C-22292775….result.json` (`1aef8379…`) |

Every result records:
* `accepted` true and `acceptance_credit` true;
* `dry_run` false and `development` false;
* no first failure, and nothing not run.

Each authorization (`max_attempts` 1) was consumed by a permanent marker before the first row. The candidate was
unchanged at every row and at every gate's end, and each gate checked that its predecessor had been accepted.
The records are in plan §16.21–16.24.

## Claims (contract §1) and the evidence

* **Each required fault scenario shows its specified detection and safe response**, under the declared injection
  conditions, with the accepted P5 stack unchanged. All 26 live P6 evaluations passed: 14 in Gate A, 2 in B, 5 in S
  and 5 in C. Each one's sha256 matches its gate result. Every required class item is covered (see Coverage).
* **Every flight kept:**
  * 0 collisions;
  * truth clearance ≥ 0.20 m (the minimum was 0.370 m, in C-2);
  * 0 unsafe commands;
  * a verified landed-and-disarmed end.

  All 12 flights did.
* **Each scenario was observed once.** These are existence demonstrations. With 0 unsafe outcomes in 12 flights, the
  exact one-sided 95 % upper bound on the per-flight unsafe-outcome probability is 22.1 %, for these conditions only.

## Flights (simulated flight, with acceptance credit)

Arms follow plan §8:
* **A:** pass-through nominal.
* **B:** single faults.
* **C:** compound faults.

Latencies are in sim seconds from the row's trigger.

| Row | Arm | World | Fault | Detection / safe action | Mission outcome | Min truth clearance | Time in HOVER | Trigger → LAND | Availability |
|---|---|---|---|---|---|---|---|---|---|
| B-0 | A | S2 | none | — | goal with detour, 0.015 m | 0.453 m | 0 | (goal) | 1.000 |
| B-1 | B | S2 | F3.08 policy timeout ×3 | 0.048 / 0.048 | ABORT `policy_unavailable`, L45 | 2.095 m | 0 | 0.148 | 0.979 |
| S-1 | B | S2 | F1.01 depth loss | 0.284 / 0.284 | HOVER, ABORT `stale_inputs:snapshot_stale`, L45 | 0.634 m | 10.04 | 10.33 | 0.604 |
| S-2′ | B | S2 | F2.09 pose withheld from perception | 0.248 / 0.248 | HOVER, ABORT `stale_inputs:snapshot_stale`, L45 | 1.926 m | 10.05 | 10.30 | 0.425 |
| S-3 | B | S2 | F3.04 non-finite scores ×3 | 0.052 / 0.052 | ABORT `policy_unavailable`, L45 | 2.078 m | 0 | 0.152 | 0.979 |
| S-4 | B | S3 | F4.01 map loss 1.0 s | 0.152 / 0.152 | 1 recovery (§6.5), goal, 0.066 m | 1.064 m | 0.90 | (goal) | 0.980 |
| S-5′ | B | S1 | F6.03 one 60 ms mission-tick stall | 0.048 / 0.048 | P2 `stale_action`, one LAND, L45R | — (no obstacle) | 0 | 0.048 | 1.000 |
| C-1 | B | S3 | F1.03 depth bursts 0.1 / 0.5 / 1.5 s | 4.252 / 4.252 ¹ | 2 recoveries (§6.5), goal, 0.044 m | 1.012 m | 1.75 | (goal) | 0.961 |
| C-2 | B | S4 | F3.07 five isolated 25 ms ticks | 5.0 / 5.0 ¹ | HOLD on exactly the 5 ticks, no ABORT, goal, 0.012 m | 0.370 m | 0 | (goal) | 0.994 |
| C-3 | B | S2 | F3.13 adversarial valid choice | — ² / 149.964 | contained (S0); ABORT `transit_deadline`, L45 | 3.352 m | 0 | 149.96 | 1.000 |
| C-4 | C | S2 | F8.01 bridge stop + inference delay | 0.052 / 0.052 | ABORT `policy_unavailable`, 0 invocations after stale onset, L45 | 2.051 m | 0 | 0.152 | 0.979 |
| C-5 | C | S4 | F8.03 map loss 1.0 s + 24 CPU workers | 0.152 / 0.152 | recovered from the map loss, then the §5.3 P2 abort (55.4 ms callback), L45R | 1.981 m | 0.88 | 1.068 | 0.814 |

¹ By design, the first effective injection comes after the trigger:
* C-1: the 0.1 s burst never ages a snapshot past 0.25 s, so the first stale onset is in the 0.5 s burst at +4 s.
* C-2: the first injected tick is at +5 s.

² C-3's fault is a valid but adversarial choice, so no fault label is expected. The row requires containment (S0)
and reports its outcome.

**The other numbers for these flights:**
* **Tracking:** max ≤ 0.254 m (limit 0.30 m).
* **Setpoints:** no setpoint inside an inflated truth obstacle.
* **Unsafe commands:** 0 in every flight (contract §6.3).
* **Landing:**
  * After REQUEST_LAND, the L45 landings reached COMPLETE in 6.5–6.7 s sim.
  * The L45R landings were landed and disarmed 4.58 s (S-5′) and 8.04 s (C-5) wall after their one LAND (limit
    45 s wall).
* **Recovery stability** (a return to a fault label within 2.0 s after a recovery): 0 of 4 recoveries.
* **Deadlines:** 2 of all control callbacks went over the 0.050 s dispatch budget, and each ended in its declared
  P2 abort:
  * S-5′'s injected 65.5 ms stall;
  * C-5's 55.4 ms callback under load.

  B-0's slowest callback was 20.8 ms.
* **Inference** on non-failing ticks: p99 ≤ 3.0 ms and max ≤ 4.2 ms (limits 5 ms and 10 ms). In C-5, inference
  outside the load window was 3.0 ms.
* **RTF:** 0.976–0.999.

**Mission success and safety success are separate** (contract §1):
* all 12 flights are safety passes;
* 4 reached the goal (B-0, S-4, C-1, C-2);
* 6 landed after a supervisor ABORT (B-1, S-1, S-2′, S-3, C-3, C-4);
* 2 landed through P2 RECOVERY (S-5′, C-5).

The 8 that did not reach the goal are mission failures by design, never reported as mission completion.

## Gate A: ground and offline

| Rows | Content | Result |
|---|---|---|
| A-1 | P3 readiness, unchanged | passed |
| A-2 | P3, P4 and P5 manifests verify and are bound by the P6 manifest | passed |
| A-3 | P6 readiness under load, every injector pass-through, decisions bit-identical to the P5 replay | passed |
| A-4 | the M3 offline suite, hash-bound: 89 of 89 tests, every R-O item | passed |
| A-5 | 12 ground fixtures: F1.01, F1.02, F1.04, F1.06, F1.07, F1.11, F2.09, F3.01, F3.02, F3.03, F4.07, F4.13 | 12 of 12 passed |
| A-6, A-7 | injector activation and cleanup; one PX4-input owner, Policy A, graceful cleanup | passed |

## Coverage (fault matrix, 86 IDs: 77 faults and 9 aliases)

| Class | Faults | Covered by | Result |
|---|---|---|---|
| R-O (offline) | 32 | the A-4 offline suite (89/89) | 32 of 32 |
| R-A (ground) | 12 | the A-5 fixtures | 12 of 12 |
| R-B (smoke) | 1 | B-1 (F3.08) | 1 of 1 |
| R-S (safety) | 6 | S-1, S-2′, S-3, S-4, S-5′; F7.09 through F4.01 in S-4 | 6 of 6 |
| R-C (qualification) | 7 | C-1 to C-5; F6.01 through F8.03 in C-5; F7.09 through F1.03 in C-1 | 7 of 7 |
| C (characterization) | 35 | reported, not gated | 12 with offline characterization data from M3/A-4; 23 without P6 data |
| cov (accepted evidence reused) | 2 | F3.02 (also re-run in A-5), F7.01 (P4/P5 S5) | — |
| D (deferred) | 3 | F2.10, F5.08, F5.12 | — |

The tiers, over the 77 faults (a fault can be in more than one): G 19, S 34, O 57.

The M4 coverage report (`p6_coverage_report.py`, bound and unchanged) lists F6.01 and F7.09 as "required faults
without a passing run". That's because it keys on each run's own fault ID and ignores the specs' `realized_by`
field. Both faults are declared as realized through other rows, and those rows passed with the check each one
requires:
* **F6.01:** C-5's LOAD check, which implements §5.3.
* **F7.09:**
  * S-4 and C-1's RECOVERY checks: every return from HOVER happened on a decision with no stale reason from the
    accepted supervisor (snapshot, adapter health, calibration, unusable frames, estimator reset) and an
    independently computed snapshot age ≤ 0.25 s;
  * plus UNSAFE.

The M7 comparison file counts `realized_by`.

## A/B/C comparison (descriptive, paired by world)

| Arm | Flights | Collisions | Min clearance | Unsafe | Goal | Supervisor ABORT + L45 | P2 RECOVERY + L45R | Recoveries (relapses) | Over-budget callbacks | Median detection |
|---|---|---|---|---|---|---|---|---|---|---|
| A | 1 | 0 | 0.453 m | 0 | 1 | 0 | 0 | 0 (0) | 0 | — |
| B | 9 | 0 | 0.370 m | 0 | 3 | 5 | 1 | 3 (0) | 1 (injected) | 0.200 s |
| C | 2 | 0 | 1.981 m | 0 | 0 | 1 | 1 | 1 (0) | 1 (§5.3) | 0.102 s |

**False alarms in arm A:** B-0, 136 s sim, had 0 stale ticks, 0 HOVER and 0 policy failures. That's 0 fault labels
in one flight, not a rate.

**Paired by world:**

| World | P5 reference (accepted P5 Gate flight) | P6 rows |
|---|---|---|
| S1 | goal, no obstacle | S-5′ |
| S2 | goal with detour, 0.420 m, 42.9 s to goal | B-0 (goal with detour, 0.453 m, arm A); B-1, S-1, S-2′, S-3, C-3 (B); C-4 (C) |
| S3 | goal with detour, 1.032 m | S-4, C-1 (B) |
| S4 | goal with detour, 0.419 m | C-2 (B); C-5 (C) |

Only S2 has an arm-A flight, so S1, S3 and S4 use the accepted P5 flights as a reference, not as arm A. Clearance in
the P6 S3 and S4 rows that reached the goal, against their P5 reference flights:
* S-4 and C-1 had 1.064 m and 1.012 m, against 1.032 m;
* C-2 had 0.370 m, against 0.419 m. That's lower, but still above the 0.20 m limit.

The comparison makes no significance claim (plan §8).

## Operator decisions applied in the accepted rows (contract §10)

**No threshold was relaxed or changed.** Each exception below replaces exact items on named rows with a stricter or
equal proof. In every accepted row, the items that were excepted were exactly that row's declared ones, and none was
refused.

* **M5, 2026-11-07 ("Adopt both") and 2026-11-13: F4.07 and F4.13.**
  * They trigger at `ground_prestream`.
  * Exactly `cleanup:ros-controller:nonzero_exit_1` and `p2:timing` are replaced by the fail-closed proof.
  * F4.13 is also detected as `clock_epoch_mismatch`, with proof in the controller's own ledger.
* **M6, 2026-11-14 ("Adopt"): S-5′.** `RECOVERY_LANDING` replaces the four items that the accepted code produces for
  any P2 RECOVERY landing.
* **M6, 2026-11-14 ("Adopt both"): C-3.** `FAIL_CLOSED_LANDING` is recorded only for a P2 RECOVERY landing. The real
  C-3 ended in a transit-deadline ABORT and L45, so it was not used.
* **M6, 2026-11-14 ("Adopt as recommended", "Adopt the extension"): C-5.**
  * The load starts at the map-loss trigger.
  * The §5.3 `LOAD` check, `LANDING`, and `LOAD_RECORDER` apply.
  * `LOAD_RECORDER` narrows an accepted evidence-completeness check, only inside a declared load window. In the real
    C-5 it replaced the three recorder items for a 36-frame overflow inside the load window, which is reported. The
    delivery items did not arise.
* **M6, 2026-11-15 ("Adopt the proof fix"): S-4 and C-5.** The map-drop proof judges the controller's own decisions in
  [trigger, trigger + window).

## Scope and limitations

* **Simulation only:** PX4 SITL with Gazebo Harmonic. There was no hardware flight, and no claim is made about real
  flight, formal fault tolerance or certification.
* **Existence demonstrations only:** one run per required scenario. No detection-rate, false-alarm-rate or reliability
  claim is made (P7).
* **App-side faults:** the shim changes what the app sees, not PX4's own estimate. PX4-native sensor and estimator
  failures are deferred (D5). Shim runs are "accepted code plus a declared, hashed shim", bounded by pass-through bit
  identity (A-3, B-0).
* **Characterization:**
  * No characterization flight was run, including the D4a PX4-managed cases S-2 (F2.01) and S-5 (F5.01). Each needs
    its own authorization and is never counted as a pass.
  * 23 C-class items have no P6 characterization data.
* **Under 24 CPU workers (C-5),** the accepted P2 controller can overrun its 0.050 s dispatch budget and abort, which
  is the declared §5.3 response, and the P3 raw depth recorder can overflow. The real C-5 shows a safe landing, not
  mission completion under that load.
* **Operational sensitivities** seen during the M5–M6 development dry runs, none in the real gates, none involving P6
  code, and none excepted:
  * one MicroXRCE-DDS agent SIGSEGV at teardown (the first in 137 SIGINT stops since P3);
  * one Gazebo 15 s shutdown hang;
  * the host power overlay: on "Best power efficiency" the accepted stack's RTF fell to about 0.5–0.6 and broke the
    accepted P2 freshness rule. The overlay is the operator's setting, and it was verified read-only before every
    live run.

## Development history (no credit, preserved)

The development candidates were:
* `524f0714` (M5 Gate A content);
* `33f33c5a`, `23d513ee`, `75289deb`, `0db5c60a`, `6b1d952d` and `57b9af72` (M6 content dry runs, plan §16.5–16.18);
* `06d08d98`, superseded because of a relative approval path, never used.

`57b9af72` passed all four gates' content, with no credit. The real candidate `22292775` binds the same 41 sources
and specs. Every failed and superseded dry run's evidence is kept (D10).

## Retained evidence

* **Runs:** the 27 run directories of the four real gates (the 26 evaluated rows plus A-4's offline suite) are in
  `~/uav_autonomy/runs`, named `20261115T*-p6-<row>-*`. Each flight holds:
  * its ULog, rosbag and MAVLink capture;
  * the P4/P5 ledgers;
  * the P6 fault declaration, fault ledgers and witness;
  * the load-harness ledger (C-5);
  * `p5-evaluation.json` and `p6/p6-evaluation.json`.
* **State:** `~/uav_autonomy/state` holds:
  * the manifest and the production Policy A approval;
  * each gate's authorization, consumption marker and result;
  * every development candidate and dry-run approval.
* **Reports:** `docs/evidence/v3_sim_p6_22292775_coverage.json` (M4 coverage report) and
  `docs/evidence/v3_sim_p6_22292775_m7_comparison.json` (per-flight metrics, arms, coverage by class and tier, P5
  references).
* **Frozen files:** changing any of the 41 manifest-bound P6 files, or any P3, P4 or P5 bound file, breaks `verify`.
  Verify with:

```bash
GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 python3 simulation/px4_gazebo_p6/scripts/p6_manifest.py verify "$HOME/uav_autonomy/state/p6-candidate-8e29f3f.json"
```

The baselines verify with `p3_manifest.py` (`p3-candidate-087722b.json`), `p4_manifest.py`
(`p4-candidate-50d5002.json`) and `p5_manifest.py` (`p5-candidate-40a109b.json`). All four returned their accepted
SHA-256s on 2026-11-15, after Gate C.
