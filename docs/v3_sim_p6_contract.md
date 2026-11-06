# v3-SIM P6 contract `p6-acceptance-v1` — fault injection and safety validation

**Status: APPROVED by the operator on 2026-11-06 ("Approve contract").** The approval covers the
whole contract, including:

* the new thresholds §5.1, §5.2 and §5.4 (§5.3 was already approved under D8);
* rule C11;
* decisions D1–D10, D1a and D4a (§10).

The operator answered D1–D10 on 2026-11-06, all as recommended. M0 is complete
([audit](v3_sim_p6_m0_audit.md)); it raised D1a, D4a and C11, also approved as recommended.

**What approval does not authorize.** Implementation, freezing, simulator use and gates are still
not authorized. Each milestone (M1 onward), each freeze and each gate needs its own operator
authorization.

All limits not introduced in §5 are reused unchanged from P2–P5. See the
[P6 plan](v3_sim_p6_plan.md) and the [fault matrix](v3_sim_p6_fault_matrix.md).

## 1. Scope and claims

* **Claimed on acceptance:**
  * In PX4 SITL (v1.17.0, Gazebo Harmonic, ROS 2 Jazzy, WSL2, D3D12), the accepted P5 stack
    (manifest `2becfbbb794006833d0a224b819db702135b0a0c9ff25da6577a7c1dd5238632`), unchanged,
    shows the specified detection and safe response for each required fault scenario of the
    matrix, under the declared injection conditions.
  * Every flight keeps 0 collisions, truth clearance ≥ 0.20 m, 0 unsafe commands and a verified
    landed-and-disarmed end, or the declared terminal outcome for a characterized PX4-managed case.
  * Each scenario was observed once.
* **Not claimed:**
  * formal fault tolerance, safety certification or real-hardware behaviour;
  * fault classes the accepted code cannot detect (characterization only);
  * PX4-native sensor or estimator failures (deferred, D5);
  * detection, false-alarm or reliability *rates* (P7);
  * any change to P2–P5 behaviour.
* **Mission and safety outcomes are reported separately.** An ABORT and landing is a safety pass
  and a mission failure.

## 2. Boundaries (normative)

* **C1.** No file in the P3, P4 or P5 manifest scopes is created, changed or deleted (plan §1.1).
  The P6 runner refuses to start unless all three manifests verify to the SHA-256s in plan §0.
* **C2.** The P5 model, build install `e7a59f35…`, configs, thresholds and controller interpreter
  are used unchanged. No P5 rebuild happens in P6.
* **C3.** No undeclared fallback. P5's `fallback: none` stays. A fallback would need a separate
  design, tests and contract approval, and a new P5 candidate.
* **C4.** No P6 process publishes on `/px4_71/fmu/in/*`. The F4.07 fixture advertises a publisher
  and never publishes a message.
* **C5.** No MAVLink or `vehicle_command` traffic outside Policy A content. There is no PX4-native
  failure injection unless D5 is approved and Policy A is amended.
* **C6.** No PX4 parameter, Windows setting or WSL setting changes. Load workers run inside the
  owned namespace and are bounded in count, duration and memory.
* **C7.** Every injector has `off`, `pass-through` and `active` modes. It is declared in the P6
  manifest, with its source hashes and fault parameters, before a run.
* **C8.** A fault scenario cannot pass unless the evaluator independently confirms that the
  intended fault occurred (§7.2). The injector's own success record is never sufficient.
* **C9.** Simulation only. No hardware. All live runs need operator authorization.
* **C10.** Code lives in `simulation/px4_gazebo_p6/`, plus tests in `tests/test_px4_gazebo_p6_*.py`.
* **C11 (proposed by M0; approved 2026-11-06).** Every run-local config must equal its frozen original byte for
  byte, except the keys declared in the run's fault declaration (for example the two
  `ros_topic_name` values of `p3_sensor_bridge.yaml`). Each declared key's before and after values
  are recorded. The P3–P5 evaluators do not make this check (M0 Finding C), so the P6 evaluator
  owns it.

## 3. Limits carried over unchanged

| Source | Limits |
|---|---|
| P2 | 20 Hz; dispatch budget 0.050 s wall; graph fresh 1.5 s wall; position and attitude freshness 0.25 s sim, flags 1.5 s sim, state 2.0 s wall; clock stall 1.0 s wall; sample gap 0.2 s sim; ACK 3.0 s sim / 5.0 s wall, one attempt; recovery 45 s wall |
| P3 | depth stream limits (≥ 25 Hz, source gap ≤ 0.12 s, receipt stall ≤ 1.0 s wall, observation age ≤ 0.25 s sim, processed ≥ 8 Hz, recorded 1.0, overflow 0); interruption detection 1.2 s wall; association; RGB absent; renderer D3D12; MC causality; graceful cleanup; Policy A |
| P4 | 0 collisions; truth clearance ≥ 0.20 m; tracking ≤ 0.30 m; response latency ≤ 0.25 s sim; setpoint validity; snapshot latency max ≤ 0.25 s, p95 ≤ 0.15 s; perception compute p99 ≤ 0.050 s, max ≤ 0.090 s; snapshot rate ≥ 8 Hz; supervisor table and timeouts (HOVER 20 s, stale 10 s, transit 150 s); §9.1 F1–F4 fault-window rules |
| P5 | inference p99 ≤ 0.005 s, max ≤ 0.010 s; 0 policy failures outside declared fault ticks; participation 1.0; 0 outside the mask; 0 fallbacks; checkpoint identity; bit-identical score replay |

## 4. Fault windows and the applicability of normal-operation limits

Each run declares `t_inj` (activation) and, where applicable, `t_clear`.

* **`W_pre`** = [sensor-window start, `t_inj` − 0.12 s]: every limit in §3 applies unchanged.
* **`W_fault`** = (`t_inj`, run end]: the table below applies.
* **No silent exemptions.** A limit not listed as replaced applies in `W_fault` too.

| Limit | In `W_fault` | Replaced by (stricter or equal fault-response check) |
|---|---|---|
| P3 delivery and rate, for depth faults only (F1.01–F1.04) | not applied | P4 §9.1 F1 attributable, F2 detected; H12 |
| P4 tracking ≤ 0.30 m | applied while the app commands; **not applied** during PX4-managed failsafe (no app setpoints) | PX4-managed: horizontal drift from the last app target **reported, not limited**, because under D4a these runs are characterization only; clearance ≥ 0.20 m and 0 collisions still evaluated |
| P5 inference p99 and max | applied to every non-injected tick | injected tick IDs (from the shim ledger, cross-checked by `inference_s` in the accepted ledger) excluded, and only those |
| P5 0 policy failures | applied outside declared failing ticks | declared failing ticks must show exactly the declared failure reason |
| P4 perception compute and latency, inside a declared load window (F6, F8.03) | evaluated and reported | §5.3: every breach is followed by a safe response within the stated latency, or the run fails |
| P2 dispatch budget | always applied | an injected breach (F3.09, F6.03) must produce the P2 abort it implies |
| Clearance, collisions, unsafe commands, Policy A, ownership, cleanup | **always applied** | — |

### 4.1 Clocks

| Requirement | Clock | Source |
|---|---|---|
| staleness, HOVER, ABORT timing, policy tick counting, response latency | simulation time (`/clock`) | `p4/decisions.jsonl`, `p4/local_map.jsonl`, policy ledger |
| adapter health detection, dispatch budget, clock stall, state receipt, graph check, landing within 45 s | wall `CLOCK_MONOTONIC` | adapter health records, `timing-trace.jsonl`, `ros-events.jsonl` |
| PX4 failsafe onset, nav_state, land detection, arming | PX4 boot time, correlated with sim by the P2 ROS/ULog position pairing (v1.6) | ULog |
| truth clearance and collisions | Gazebo sim time | `p4/truth_poses.jsonl`, `p4/contacts.jsonl` |
| injection activation | sim and wall both recorded by the injector; **confirmed** by the witness or ULog in their own clocks | fault ledger, witness |

## 5. New thresholds (approved with the contract, 2026-11-06)

### 5.1 Policy-fault timing (derived from P5 §5 and the 20 Hz rate)

* **Failing tick.** The target and yaw are unchanged on every failing tick: ≤ 0.01 m and ≤ 0.5°,
  which is the P4 §9.1 HOVER bound.
* **ABORT.** `policy_unavailable` comes on the 3rd consecutive failing tick: 0.10 s sim after the
  first, ± 0.05 s, one tick of tolerance for timer jitter.
* **LAND.** The first LAND command follows in the same tick as the ABORT (P4 revision 5
  `_land_after_abort`), within ≤ 0.05 s sim.

### 5.2 PX4-managed outcomes (D4)

**Derived in M0** ([audit](v3_sim_p6_m0_audit.md) §5–§6) and approved with the contract on
2026-11-06. Under D4a, they are the reporting thresholds for the PX4-managed characterization
flights. No gate counts them as a pass.

* **What PX4 does.** An offboard loss gives an immediate **Return** (`AUTO_RTL`), with no 5 s Hold.
  It returns at `max(d, 4 m)` for 2 m < d ≤ 10 m from home, flies straight home, lands and
  disarms. One retained run shows it: 1.05 s to detection, 6.3 s from onset to disarm.
* **Proposed limits:**
  * onset ≤ 1.0 + 0.2 s after the last `offboard_control_mode`;
  * `AUTO_RTL` within 0.1 s of onset;
  * landed and disarmed within 45 s wall of onset (P2 `recovery_wall_s`), ULog only;
  * 0 collisions and truth clearance ≥ 0.20 m, with no exemption.
* **What cannot hold.** The P4 3.0 m ceiling cannot hold in this window. Under D4a (approved), the
  PX4-managed runs are characterization only, and these values are reported, not gated.

The text below was written before M0 and is kept for the record.

* **To be derived in M0.** The values come from the pinned PX4 v1.17 source and the parameters
  read from the accepted ULog: `COM_OF_LOSS_T` 1.0 s, `COM_FAIL_ACT_T` 5.0 s, `COM_OBL_RC_ACT` 0,
  `MPC_LAND_SPEED` 0.7 m/s, `COM_DISARM_LAND` 2.0 s.
* **Values to set:**
  * failsafe onset tolerance;
  * expected nav_state sequence;
  * landing and disarm deadline;
  * permitted drift.
* **If PX4 does not land** with these parameters and no RC, then:
  * the scenario's expected outcome is *terminal failure, vehicle airborne in PX4 hold*;
  * that is a documented gap;
  * no required gate may count it as a pass.

### 5.3 Load-window rule (D8; approved by the operator 2026-11-06)

The approved bounds are at most 24 workers, at most 60 s per window, and memory ≤ 25 % of free RAM.
Inside a declared load window, a breach of a P4 perception or P5 inference limit is acceptable
only if the declared safe response for the resulting input condition follows:

* stale snapshot → H35;
* inference above 0.020 s → HOLD;
* dispatch above 0.050 s → P2 abort.

There must also be 0 unsafe commands. A breach with no matching safe response fails the run.

### 5.4 Injection-proof timing

* **Witness.** It must record the first faulted message within 0.10 s sim of the injector's
  `t_inj`.
* **Process signals.** The `/proc` state change, or the exit, must follow within 1.0 s wall.

## 6. Metric definitions (normative for evaluation)

### 6.1 Latencies

* **Detection latency.** `t_detect − t_inj`, where `t_detect` is the first record produced by the
  accepted code that names the fault class:
  * an adapter health transition;
  * a supervisor `stale` reason;
  * a policy ledger `failure`;
  * a P2 `abort` event;
  * a PX4 `failsafe` flag (ULog).
* **Fault-to-safe-action latency.** `t_safe − t_inj`. For staleness it is `t_safe − t_last`, per
  P4 §9.1. `t_safe` is the first publication with a frozen target and yaw, the first LAND command,
  or the PX4 failsafe nav_state change.

### 6.2 Degraded operation

* **Time in degraded mode.** From the first fault-detected label to recovery (a progress state on
  fresh inputs) or to ABORT.
* **Abort initiation delay.** `t_ABORT − t_rule`, where `t_rule` is the rule's due time (stale
  onset + 10 s, hover onset + 20 s, the 3rd failing tick).

### 6.3 Unsafe command

Any one of these counts as an unsafe command:

* a `trajectory_setpoint` in `W_fault` that moves the target more than 0.01 m or the yaw more than
  0.5° while the applicable rule requires HOLD or HOVER;
* a setpoint target inside a truth obstacle inflated by `R_body` + 0.10 m;
* a `vehicle_command` outside Policy A, or more than one LAND;
* any setpoint not from the owner (graph record and ULog);
* motion resumed without the full revalidation set (§6.5).

The required count is **0**.

### 6.4 Landing completion

The controller reaches `COMPLETE`, **and** the ULog shows `landed` and `arming_state` disarmed.
For PX4-managed cases, only the ULog counts.

### 6.5 Recovery eligibility

All of these must hold on the same tick:

* a usable snapshot ≤ 0.25 s old with matched association;
* adapter health `fresh` and ≤ 2.0 s old;
* calibration identity equal;
* no estimator reset since the last usable frame;
* the policy healthy on that tick;
* a collision-valid progress candidate.

**Recovery stability** is the count of returns to a fault label within 2.0 s sim after a recovery.
It is reported only; it is not gated.

### 6.6 Repeated-failure handling

The consecutive-failure counter must equal the recomputation from the ledger on every tick.
Alternating patterns are reported as F7.02.

### 6.7 Research metrics (reported)

* detection rate (detectable classes only) and false-alarm rate (fault labels in arm-A runs per
  flight-hour);
* recovery success and safe-abort rate;
* collision rate and minimum clearance;
* time in HOVER and time to LAND;
* availability (TRANSIT fraction not held);
* deadline violations;
* coverage by class and tier.

Plan §8 states what the sample sizes do and do not support.

## 7. Evidence

### 7.1 Retained per run

* everything P5 retains;
* `p6/fault-ledger.jsonl`, `p6/fault-declaration.json` (hashed in the manifest);
* `p6/witness/` (raw and injected topics);
* the load harness `resources.jsonl` and schedstat (F6);
* the shim ledger (CS);
* the injector process records;
* the cleanup classification.

### 7.2 Independent injection proof, by mechanism

| Mechanism | Independent proof required |
|---|---|
| Depth relay | native source probe versus witness |
| Map/health relay | witness versus `p4/local_map.jsonl` and the decisions' snapshot stamps |
| Controller shim | accepted-code ledgers (policy ledger `failure` and `inference_s`, timing trace) and the bagged true PX4 messages versus shim-modified values |
| Process signal | `/proc` state, exit codes and the P3 cleanup classification |
| Load harness | per-process CPU, schedstat and `/proc/pressure` |

### 7.3 Truth and PX4

* **Truth:** the P4 truth observer, unchanged.
* **PX4:** the ULog is the only source for failsafe, nav_state, landed and armed claims.

## 8. Gates

**Rules for every gate:**
* Each gate is separately authorized and bound to the exact P6 manifest SHA-256.
* Each runs once, stops at the first failure and is never retried.
* Each gate after A needs the accepted predecessor for the same manifest.
* No P2–P5 gate result grants P6 credit.
* A Policy A approval is required per P6 candidate commit.

### 8.1 Gate A — ground readiness and fault fixtures

**Rows** (vehicle never armed):

1. P3 readiness unchanged.
2. P3, P4 and P5 manifests verify, and the P6 manifest binds them.
3. **P6 readiness under load with every injector in pass-through:**
   * the full stack in ground-diagnostic mode, with every §3 limit applied;
   * relay latency and CPU recorded;
   * decisions on the recorded ground snapshots bit-identical to the P5 replay.
4. **The M3 offline suite**, run in its declared scope and bound by hash: every R-O item of the
   matrix, including precedence (F8.04, F8.05) and the F7 properties.
5. **Ground fault fixtures, 13 live runs:**
   * F1.01, F1.02, F1.04, F1.06, F1.07, F1.11;
   * F3.01, F3.02, F3.03;
   * F4.02, F4.06, F4.07, F4.13.

   Each needs its detection in the live component and the replayed supervisor/policy response.
6. **Injector activation and cleanup proven** for every mechanism used later: off leaves no
   process, pass-through causes no topology change beyond the declared relay, and active matches
   the declaration.
7. One PX4-input owner; no unintended vehicle commands (Policy A); graceful cleanup.

### 8.2 Gate B — smoke

* **B-0:** S2 with every injector armed in pass-through and no fault. All P2–P5 limits apply, and
  the decisions are reproduced bit for bit.
* **B-1:** F3.08 on S2: three consecutive 25 ms inference ticks in `AVOIDING`, giving PH3 and then
  L45, with S0.

### 8.3 Gate S — safety-critical faults (fixed order)

| Row | Scenario | Fault | Required outcome |
|---|---|---|---|
| S-1 | S2 | F1.01 depth loss in `AVOIDING` | P4 §9.1 F1–F4; H35, A10, L45, S0 |
| S-2′ | S2 | F2.09 pose withheld from perception (`--px4-prefix` state relay) in TRANSIT | H35, then `ABORT stale_inputs:snapshot_stale` (A10), then L45; S0 |
| S-3 | S2 | F3.04 non-finite scores × 3 in `AVOIDING` | PH3, L45, S0 |
| S-4 | S3 | F4.01 local-map loss 1.0 s in `AVOIDING` | H35; recovery only with §6.5; goal reached; S0 |
| S-5′ | S1 | F6.03 one 60 ms mission-tick stall in NORMAL | P2 dispatch-budget abort with a valid Offboard sample, then exactly one LAND, then L45; S0 |

**D4a, approved by the operator 2026-11-06.** M0 found that PX4 lands through Return, but breaches
the 3.0 m ceiling and can pass within 0.20 m over obstacle tops. The original rows are therefore
moved out of Gate S:

* **S-2:** F2.01, stale pose to the controller.
* **S-5:** F5.01, controller SIGSTOP 1.5 s.

Both are now **separately authorized characterization flights**, with full truth-safety
evaluation and the §5.2 values reported. They are never counted as passes, and no ceiling
exemption is granted.

The earlier replacements are withdrawn:
* S-2′ = F2.04, because no SITL use is allowed before M3;
* S-5′ = F5.02, because its outcome depends on the callback order (M0 §7).

### 8.4 Gate C — fault-tolerance qualification (fixed order, all required)

| Row | Scenario | Fault | Required outcome |
|---|---|---|---|
| C-1 | S3 | F1.03 depth bursts 0.1 / 0.5 / 1.5 s | stale ⇔ age > 0.25 s on every tick; recovery per §6.5; goal reached; S0 |
| C-2 | S4 | F3.07 five isolated 25 ms ticks | HOLD on exactly the injected ticks; no ABORT; P5 limits on every other tick; goal; S0 |
| C-3 | S2 | F3.13 adversarial valid choice | S0 (shield containment); outcome reported |
| C-4 | S2 | F8.01 bridge stop + inference delay | staleness precedence: H35, A10, L45; 0 policy records after stale onset; S0 |
| C-5 | S4 | F8.03 map loss 1.0 s + 24 CPU workers | H35, then recovery or A10; §5.3; S0 |

P5's S2/S3/S4 qualification is not reused as P6 acceptance. Every Gate C row requires a detected
fault and its safe handling.

**P6 is complete only with A, B, S and C accepted for the same P6 manifest.**

## 9. Manifest

The P6 manifest binds:

* the P6 sources, configs, fault declarations and scenarios;
* the P3, P4 and P5 manifest SHA-256s, re-verified at freeze and at every run start;
* the P5 build receipt and model identity, through the P5 manifest;
* the injector and witness hashes;
* Policy A plus a per-candidate approval;
* the runtime receipts, as in P5;
* the gate definitions.

Any change after freeze is a new P6 candidate.

## 10. Decisions requiring operator approval (recommendation first)

**Decision record.** The operator answered all ten in chat on 2026-11-06, each as recommended:

| Decision | Answer | Effect |
|---|---|---|
| D1 | relays + controller shim + process signals + bounded load harness | plan §4.2 rows M-DR, M-MR, M-CS, M-PS, M-LH; no PX4-native row |
| D2 | shim runs accepted as "accepted code plus a declared, hashed shim" | pass-through bit identity is required (Gate A row 3, B-0) |
| D3 | gates require only implemented behaviour | the matrix C items stay characterization |
| D4 | derive PX4-managed expectations in M0, then approve | §5.2 derived in M0 and approved with the contract; S-2 and S-5 resolved by D4a (§8.3) |
| D5 | no PX4-native failure injection in v1 | F2.10 and F5.08 deferred; Policy A unchanged |
| D6 | Gate B = B-0 pass-through + B-1 smoke | §8.2 as written |
| D7 | GCS link loss by source analysis only, never flown | F5.12 stays D |
| D8 | bounded load windows | §5.3 approved with its bounds |
| D9 | two backup copies: external disk + versioned cloud store | the paths and destination still need naming before any copy or upload |
| D10 | keep all accepted, failed and cited diagnostic evidence indefinitely | no deletion |

**Raised by M0 ([audit](v3_sim_p6_m0_audit.md) §7) and approved by the operator in chat on
2026-11-06, all as recommended ("Approve as recommended"):**

| Decision | Answer | Effect |
|---|---|---|
| D1a | depth relay through a declared 2-key run-local bridge config; map and health input faults inside the controller shim; pose faults to perception through `--px4-prefix` and a state relay | plan §4.2 rows M-DR, M-MR (shim), M-SR |
| D4a | the PX4-managed S-2 and S-5 become characterization; Gate S uses S-2′ = F2.09 and S-5′ = F6.03 | §8.3 as now written; no ceiling exemption |
| C11 | run-local config equality, except declared keys | §2 C11 in force for P6 evaluation |

The options as presented:

* **D1a.**
  * **Recommended:** the depth relay through a declared 2-key run-local bridge config (C11); map
    and health input faults inside the controller shim; pose faults to perception through
    `--px4-prefix` and a state relay.
  * *Alternative:* a second run-local `p4_avoidance.yaml` for a map relay.
* **D4a.**
  * **Recommended:** PX4-managed S-2 and S-5 become characterization, with S-2′ = F2.09 and
    S-5′ = F6.03 in Gate S.
  * *Alternative:* keep them required with a narrow ceiling exemption, "≤ the Return altitude +
    0.5 m" inside the PX4-managed window only.
* **C11.** Run-local config equality (§2).

**The contract as a whole**, including §5.1, §5.2 and §5.4, was approved on 2026-11-06. Still to
authorize, each separately:

* each milestone from M1 onward (M0 is complete);
* any freeze, Policy A approval or gate;
* each characterization run;
* the backup paths and the backup step.

The original options are kept below.

* **D1 — injection architecture.**
  * **Recommended:** producer-side relays (DR, MR) where M0 confirms remap support, the in-process
    controller shim (CS) for policy and app-side state and ACK faults, process signals and the
    bounded load harness.
  * *Alternatives:* DDS-level proxies for everything (not possible for the controller inputs
    without source changes); PX4-native injection (D5).
* **D2 — shim identity.**
  * **Recommended:** accept that CS runs are "accepted code plus a declared, hashed shim", with
    pass-through bit identity as the bridge.
  * *Alternative:* no in-process faults. F2, F3 and F5 then become offline-only.
* **D3 — required versus characterization.**
  * **Recommended:** gates require only behaviours the accepted code implements. Undetectable
    faults (F1.05, F1.09, F2.02, F2.04 map reuse, F3.13 mission outcome, F4.03, F4.08, F4.10,
    F7.02) are characterized and reported. Remedies are new P5 candidates.
* **D4 — PX4-managed outcomes.**
  * **Recommended:** derive the §5.2 expectations in M0 from the PX4 source and the recorded
    parameters, then approve them. If PX4 does not land, S-2/S-5 switch to their §8.3
    replacements.
* **D5 — PX4-native failure injection.**
  * **Recommended:** not in P6 v1. It needs a command outside Policy A content and a PX4 failure
    workflow.
* **D6 — Gate B.**
  * **Recommended:** B-0 pass-through plus B-1 smoke.
  * *Alternative:* the smoke alone, with the pass-through proof on the ground only.
* **D7 — GCS link loss (F5.12).**
  * **Recommended:** source analysis only. Never flown while `NAV_DLL_ACT` 2 and
    `RTL_RETURN_ALT` 30 m imply a climb above the P4 ceiling.
* **D8 — load windows.**
  * **Recommended:** the §5.3 rule with the declared bounds: at most 24 workers, at most 60 s,
    memory ≤ 25 % of free RAM.
* **D9 — evidence backup target.** The operator chooses the storage location; see the
  [backup plan](v3_sim_p3_p5_evidence_backup_plan.md).
* **D10 — retention.**
  * **Recommended:** keep the raw evidence of every accepted and every failed gate indefinitely.
    No deletion is proposed.
