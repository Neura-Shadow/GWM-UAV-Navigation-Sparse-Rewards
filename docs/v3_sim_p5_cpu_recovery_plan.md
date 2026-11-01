# P5 recovery plan: CPU tail-latency optimization, host diagnostics and new-candidate readiness

**Status: PROPOSAL (2026-11-01).** Offline evidence, plus 3 operator-approved diagnostic S2 SITL flights
that earn no acceptance credit (§9.1). **After those flights the recommendation is Path A** (§9.1); §8 and
the start of §9 record the earlier, pre-diagnostic Path B reasoning.

* P5 remains **stopped and not qualified** (plan §13.16).
* Nothing here changes a frozen file, the model, a threshold or the failed Gate C result.
* No gate, simulator or flight was run. No host setting was changed. Nothing was pushed.
* The optimization below exists only as an isolated diagnostic module; it is **not** integrated.

Builds on the benchmark report [v3_sim_p5_inference_benchmark.md](v3_sim_p5_inference_benchmark.md)
(commit `a4b8f8f`). Code: [`diagnostics/p5_inference_benchmark/`](../diagnostics/p5_inference_benchmark/).
Raw JSON results stay local (`results/` is ignored by the repository).

## 1. Verified repository and manifest status (2026-11-01, read-only)

| Check | Result |
|---|---|
| Branch / HEAD | `v3/gwm-uav-c2-agentops-planning` / `a4b8f8f` (benchmark commit); only `.codegraph/` untracked before this work |
| Benchmark report and code | present (`docs/v3_sim_p5_inference_benchmark.md`, `diagnostics/p5_inference_benchmark/`) |
| P3 manifest `p3_manifest.py verify` | `1e205feca482e11f…` ✔ |
| P4 manifest `p4_manifest.py verify` | `c110bd30be22fb43…` ✔ |
| P5 baselines (C4) | P3 and P4 verify byte for byte ✔ |
| Frozen P5 manifest `db4c4779…` `p5_manifest.py verify` | `db4c477901671fdf…` ✔ (manifest file SHA-256 `ac456eb9…`) |
| P5 sources since candidate `ab7b0c6` | unchanged (`git diff ab7b0c6 HEAD -- simulation/px4_gazebo_p5` is empty) |
| Model `p5-ranker-ee006a80….npz` | SHA-256 `ee006a805ba9c327…` = file name = frozen policy SHA ✔; unchanged since `44811f2` |
| Gate C result `p5-gate-C-db4c4779….result.json` | `accepted: false`, `first_failure: qualification-1:policy:inference_max`, 1 run; SHA-256 `018e03e5…`; mtime 2026-10-31 20:20:18 (the gate run) — unchanged |
| Gate C consumption marker | present (2026-10-31 20:17:00) |
| Simulator processes | none |

## 2. Host power mode

Read only, on 2026-11-01:
* plan *Balanced* (`381b4222…`);
* power-mode overlay **`961cc777-2547-4f9d-8174-7d86181b8a7a` = "Best power efficiency"**, set for both
  AC and DC;
* on AC power (battery status 2, 79 %).

This is unchanged from 2026-10-31. **Every measurement in this plan and in the benchmark report was taken
in "Best power efficiency" mode**, except §5.1. There the operator set PredatorSense *Performance*, but
Windows still reported the power-efficiency overlay. **A verified Windows Best performance measurement
still does not exist.**

**Operator action to enable a performance-mode test.** Windows Settings → System → Power & battery →
*Power mode* → **Best performance**, with the laptop on AC. Then confirm in chat that the mode is set and
the offline comparison may run (§9, approval 1). The mode can be checked read-only with
`reg query HKLM\SYSTEM\CurrentControlSet\Control\Power\User\PowerSchemes /v ActiveOverlayAcPowerScheme`.
Best performance is `ded574b5-45a0-4f42-8737-46345c09c238`.

## 3. The optimization under evaluation (CPU-D)

[`optimized.polar_profile`](../diagnostics/p5_inference_benchmark/optimized.py) replaces the production
per-heading loop in `observation_adapter.polar_profile` (13 headings x ~15 small NumPy calls) with one 2-D
array expression over all headings:
* `math.cos` and `math.sin` per heading, as in production;
* then the same multiply, add, subtract, divide, floor, bounds test, cell lookup, traversed lookup and
  first-hit search per element, in the same order.

Per element the IEEE operations are identical, so the output is identical; only the NumPy call overhead
changes. Nothing else in the decision path changes:
* the model, weights, schema `(26, 46)`, candidate order, feature scaling and occupancy semantics;
* the mask, selection, shield, supervisor and timeout.

The preallocated-buffer forward pass also measured in the benchmark is **not** proposed: it gave no
measurable gain.

## 4. Numerical and decision equivalence (CPU-A production vs CPU-D)

[`differential.py`](../diagnostics/p5_inference_benchmark/differential.py), controller interpreter, no tolerance anywhere:

| Test | Set | Result |
|---|---|---|
| Scores on exact flight tensors | every invoked record of all 5 retained P5 flight ledgers | **4,267 / 4,267** bit-exact (frozen and CPU-D forward); recorded choice 4,267 / 4,267 |
| Observation tensors | 5,713 reproduced TRANSIT ticks of 12 retained P4 and P5 flights (S1-S6) | **5,713 / 5,713** bit-identical `(26, 46)` tensors |
| Allowed masks | same | 5,713 / 5,713 |
| Scores and selected slot | same | 5,713 / 5,713 |
| Executed action (`plan_with_ranker`: action, heading, length, end, reason, best, counts) | same | 5,713 / 5,713 |
| Supervisor decisions (sequential `PolicySupervisor` replay per run) | 5,713 steps, 12 runs | **5,713 / 5,713** steps (state, action, heading, end, frozen_yaw, reason, policy record); transitions equal in **12 / 12** runs |
| Ground snapshots (Gate A item 4 frame) | 2,945 usable snapshots of 12 retained G2/G4/G6 and readiness runs | features, masks and decisions 2,945 / 2,945 (policy never invoked on the ground, as revision 2 found) |
| Polar-profile fuzz on real grids | 200,000 cases on 48 recorded grids: random and cell-boundary starts (36,413 outside the grid), axis and heading-step angles, traversed density 0-90 % | **200,000 / 200,000** bit-exact |
| Checkpoint identity | before and after | `ee006a80…` unchanged; equals the frozen policy SHA |
| No ground truth or I/O | audit hook over 2,000 calls; static imports | 0 file, directory, socket or subprocess events; imports only `math`, `numpy`, `gwm_obstacle_avoidance`, `gwm_obstacle_avoidance.local_map` (FREE, OCCUPIED, planner.wrap) |

**No numerical or behavioral mismatch was found.**

## 5. Timing: matched CPU-A vs CPU-D

**Method** ([`recovery_bench.py`](../diagnostics/p5_inference_benchmark/recovery_bench.py)):
* Both variants run through the **unchanged production `PolicyRanker.choose`**, measured by its own
  `inference_s` (the gated quantity). Only the polar-profile function is swapped, inside the benchmark
  process.
* Calls come in pairs on the same tick, in alternating order (A,D then D,A), so data, order and time of run
  are matched.
* 20 Hz pacing uses an absolute 50 ms schedule. A missed slot is a wake-up lateness of at least 50 ms.
* Per call the harness records:
  * thread CPU time;
  * **run-queue wait** from `/proc/thread-self/schedstat`: time runnable but not running, the direct
    signature of guest-scheduler preemption;
  * voluntary and involuntary context switches;
  * the Linux CPU before and after;
  * the guest's runnable tasks (`procs_running`);
  * polar-profile and `FrozenPolicy.scores` time.

**Sample counts.** Pooled p50 is the median of per-trial p50s. p95, p99 and max are the largest per-trial
value.

| Condition | Trials x s | Calls per variant | A p50 / p95 / p99 / max (ms) | D p50 / p95 / p99 / max (ms) | > 10 ms A / D | > 20 ms A / D |
|---|---|---|---|---|---|---|
| Back to back | 3 x 30 | 102,675 / 102,674 | 0.428 / 0.784 / 1.200 / 5.615 | **0.228 / 0.430 / 0.644 / 4.156** | 0 / 0 | 0 / 0 |
| 20 Hz, idle | 5 x 120 | 6,005 / 6,000 | 0.669 / 2.618 / 2.910 / 4.955 | **0.464 / 1.552 / 1.787 / 3.200** | 0 / 0 | 0 / 0 |
| 20 Hz, pinned to vCPU 4 | 2 x 120 | 2,402 / 2,400 | 1.665 / 2.591 / 2.939 / 5.900 | 1.004 / 1.680 / 1.837 / **9.908** | 0 / 0 | 0 / 0 |
| 20 Hz, host contention (24 Windows busy processes) | 1 x 120 | 1,201 / 1,200 | 1.494 / 2.255 / 2.688 / 3.431 | 0.891 / 1.380 / 1.660 / 3.417 | 0 / 0 | 0 / 0 |
| 20 Hz, guest oversubscription (30 busy processes on 24 vCPUs), run 1 | 1 x 120 | 1,201 / 1,200 | 1.332 / 2.796 / 7.754 / 16.356 | 0.802 / 1.421 / 4.209 / 12.211 | 6 / 1 | 0 / 0 |
| same, run 2 | 1 x 120 | 1,201 / 1,200 | 1.316 / 2.614 / 7.385 / 29.354 | 0.791 / 1.358 / 3.081 / 8.219 | 2 / 0 | 1 / 0 |
| same, run 3 | 1 x 120 | 1,201 / 1,200 | 1.335 / 3.212 / 8.679 / 26.006 | 0.805 / 1.558 / 5.272 / 31.906 | 7 / 7 | 1 / 1 |

**Phase medians, 20 Hz idle** (ms):

| Variant | polar profile | `FrozenPolicy.scores` (validation + forward) | rest (candidate features, mask, rank, select) |
|---|---|---|---|
| A | 0.378 | 0.112 | 0.182 |
| D | 0.164 | 0.115 | 0.183 |

**Wake-up lateness at 20 Hz** (scheduling delay before each call):
* Idle: p99 0.62 ms, max 4.73 ms, 0 missed slots.
* Host contention: max 28.0 ms.
* Guest oversubscription: p99 6.2-7.5 ms, max 75.3 and 77.7 ms, **5 missed slots** (the thread could not
  even start on time).

**Repeatability.** Absolute values drift between trials of the same condition: the A median ranged from
0.63 to 1.40 ms over the five idle 20 Hz trials. D was below A on every quantile in every idle and
host-contention trial. The matched design is what makes the A-to-D comparison valid despite the drift.

### 5.1 Approval 1: comparison in the operator-set mode (2026-11-01)

**What the operator did.**
* Set Windows *Best performance*, re-applied it once.
* On this **Acer Predator PHN16-71**, also set **PredatorSense** to *Performance*.

**What Windows reported throughout** (read only): the power overlay stayed
`961cc777…` (*Best power efficiency*). `PowerGetEffectiveOverlayScheme` and `PowerGetActualOverlayScheme`
agree with the registry. A PredatorSense mode can change CPU power limits through firmware without moving
the Windows overlay. This condition is therefore labelled **"operator-set PredatorSense Performance; Windows
overlay unchanged"**, not "Windows Best performance".

**Run.** The same matched chain was repeated (`recovery-P-*` results).

| Condition | Mode | A p50 / p95 / p99 / max | D p50 / p95 / p99 / max | > 10 ms A / D | Slow calls with run-queue wait / charged as CPU |
|---|---|---|---|---|---|
| Back to back (3 x 30 s) | power-efficiency | 0.428 / 0.784 / 1.200 / 5.615 | 0.228 / 0.430 / 0.644 / 4.156 | 0 / 0 | 0 / 11 |
| | operator-set | 0.408 / 0.705 / 1.048 / 9.656 | 0.217 / 0.388 / 0.573 / 6.414 | 0 / 0 | 0 / 40 |
| 20 Hz (5 x 120 s) | power-efficiency | 0.669 / 2.618 / 2.910 / 4.955 | 0.464 / 1.552 / 1.787 / 3.200 | 0 / 0 | 0 / 17 |
| | operator-set | 1.380 / 1.914 / 2.602 / 4.791 | 0.888 / 1.195 / 1.695 / 2.972 | 0 / 0 | 0 / 12 |
| 20 Hz, guest oversubscription (3 x 120 s) | power-efficiency | 1.332 / 3.212 / 8.679 / 29.354 | 0.802 / 1.558 / 5.272 / 31.906 | 15 / 8 | 217 / 0 |
| | operator-set | 1.335 / 5.596 / 10.913 / 21.661 | 0.809 / 2.514 / 7.938 / 17.827 | 38 / 15 | 379 / 2 |
| 20 Hz, host contention (1 x 120 s) | power-efficiency | 1.494 / 2.255 / 2.688 / 3.431 | 0.891 / 1.380 / 1.660 / 3.417 | 0 / 0 | 0 / 3 |
| | operator-set | 1.392 / 1.899 / 2.675 / 5.540 | 0.874 / 1.189 / 1.564 / 2.764 | 0 / 0 | 0 / 8 |
| 20 Hz, pinned to vCPU 4 (2 x 120 s) | power-efficiency | 1.665 / 2.591 / 2.939 / 5.900 | 1.004 / 1.680 / 1.837 / **9.908** | 0 / 0 | 0 / 14 |
| | operator-set | 1.314 / 2.052 / 3.307 / **16.594** | 0.845 / 1.307 / 1.740 / 2.625 | **2** / 0 | 0 / 15 |

Findings:

* **No measurable benefit from the operator-set mode.**
  * The 20 Hz median of the production path was *higher* (per-trial A p50 1.18-1.56 ms, against
    0.63-1.40 ms before).
  * p95 and p99 were somewhat tighter at 20 Hz.
  * Maxima did not improve.
  * The Windows overlay never changed, so whether the intended host setting was in force cannot be
    confirmed from inside the machine.
* **The charged-slowness class alone can exceed 10 ms.**
  * In the pinned idle run, the production path had two calls over 10 ms (max 16.6 ms).
  * Both had zero run-queue wait, no involuntary switch, and CPU time ≥ wall time.
  * This is the first offline reproduction of an in-flight-sized breach **without any guest load**.
  * The optimized path had no breach in that run.
* **Guest oversubscription again dominated breaches**: 379 of 381 slow calls had run-queue wait.
  * D halved the exceedances again (15 vs 38).
  * D still reached 17.8 ms.
* **Pooled over all paced conditions without guest oversubscription in both modes**
  (20 Hz, host contention, pinned; 9,600 calls per variant per mode, 19,200 per variant in total):
  * production: **2** calls over 10 ms;
  * optimized: **0**, worst 9.91 ms.
  * Zero in 19,200 bounds D's rate below about 1.6 in 10,000 at 95 % confidence. That is an upper bound,
    not a guarantee, and the flight's observed production rate (1 in 4,267) is of the same order.

**What D buys:**
* *Algorithmic speedup:* polar profile 2.3x faster at 20 Hz, 4x back to back.
* *End-to-end decision speedup:*
  * median -31 % at 20 Hz, -47 % back to back;
  * p99 -39 % at 20 Hz, -46 % back to back;
  * maximum -35 % at 20 Hz idle.
* *Under guest oversubscription:* fewer calls over 10 ms in total (8 vs 15), but not zero, and one D call
  reached 31.9 ms.

## 6. Scheduling and tail-latency analysis

The run-queue wait separates every slow call (> 3 ms) into one of two classes, with no exceptions:

| Class | Where | Count (A + D) | Signature | Interpretation |
|---|---|---|---|---|
| **Guest preemption** | guest oversubscription only | 217 / 217 slow calls | run-queue wait > 0 (up to 30.4 ms), wall ≫ CPU, involuntary switches, `procs_running` 31-47 | **Measured:** the WSL guest scheduler ran something else while the decision was runnable |
| **Charged slowness** | idle, back to back, host contention, affinity | 45 / 45 slow calls | run-queue wait = 0, no involuntary switch, CPU time ≥ wall time | The guest believes the thread was running the whole time, yet the same work took up to 10x longer. **Hypothesis** (not proven): host frequency or power state ("Best power efficiency"), cold caches after sleep, or hypervisor vCPU descheduling charged to the guest (WSL reports no steal time) |

Consequences:

1. **The optimization cannot bound either class.** A stall of either class is not computation, so making
   the computation shorter only lowers the chance that a stall lands inside it. In matched trials D halved
   the class-1 exceedances (8 vs 15 over 10 ms). In class 2, one D call reached **9.91 ms** on an idle,
   pinned process (10.2 ms CPU, zero wait), 0.09 ms below the limit.
   §5.1 then showed the class-2 mechanism alone pushing the *production* path to 16.6 ms on an idle,
   pinned process. Either class can therefore produce a breach of the in-flight size.
2. **Which class caused the in-flight 10.84 ms is unknown.** The flight recorded neither run-queue wait
   nor `procs_running` for policy calls. Earlier analysis (plan §13.14) found guest load averages of
   1.3-6.2. That is a one-minute mean, and it can hide short bursts above 24 runnable tasks.
3. **A finite clean trial is not proof.** About 120,000 offline calls per variant had no breach outside
   guest oversubscription. That bounds the rate only loosely: with zero events in about 6,000 idle 20 Hz
   calls per variant, the 95 % upper bound is about 5 in 10,000 calls. The flight's 1 in 4,267 is within
   reach of that.

## 7. Production integration feasibility

**Smallest change: one function in one manifest-bound file.**

| File | Change | Manifest / receipt effect |
|---|---|---|
| `simulation/px4_gazebo_p5/ros2_ws/src/gwm_policy_control/gwm_policy_control/observation_adapter.py` | replace the body of `polar_profile` with the 2-D form in §3. Drop the diagnostic step cache so the function stays stateless; computing the ~40-element step array per call costs about 1 µs. Signature, docstring contract and return value unchanged | changes `p5_source_files`; changes the P5 package hash, so `state/p5-built.json` must be regenerated with `build_p5.sh --build` |
| `tests/test_px4_gazebo_p5_policy.py` | add a differential test that keeps the current loop as an in-test oracle and checks bit equality on recorded and fuzzed inputs, as in §4 | test file only; not manifest-bound |
| `simulation/px4_gazebo_p5/configs/p5_validation.yaml` | `contract_revision` string, only if revision 3 (§10) is approved | bound by the manifest (`p5_contract_revision`) |

**Unchanged:** the model and checkpoint; `policy_runtime.py`, `candidate_ranker.py`, `decision.py`,
`safety_shield.py`, `mission.py` and `control_node.py`; every P2, P3 and P4 file; the model schema; all
thresholds; PX4 ownership. The function has no ROS import and creates no publisher, so **no additional
PX4 control path is possible**.

The training code (`training/p5_dataset.py`) builds features through the same adapter. Its outputs would
be bit-identical. The model's dataset, training and readiness reports do not bind the adapter source.

**Candidate identity.** Any source change produces a new candidate. Required sequence:
1. a new commit;
2. `build_p5.sh --build`;
3. Policy A approval for that commit;
4. `p5_manifest.py freeze`, giving a new manifest SHA;
5. fresh, separately authorized Gates A, B, S and C.

The Gate A, B and S results of `db4c4779…` cannot be transferred. The failed Gate C stays preserved.

## 8. New-candidate readiness assessment

| # | Required evidence | Status |
|---|---|---|
| 1 | P3 manifest verifies | ✔ (§1) |
| 2 | P4 manifest verifies | ✔ (§1) |
| 3 | P5 checkpoint unchanged | ✔ (§1, §4) |
| 4 | Bit-exact scoring and decision equivalence | ✔ (§4: 4,267 tensors, 5,713 flight ticks, 2,945 ground snapshots, 200,000 fuzz cases, 12 supervisor replays) |
| 5 | No additional direct PX4 control publishers | ✔ (pure function; no ROS import; §7) |
| 6 | Safety and failure behaviour unchanged | ✔ by construction and replay (supervisor 5,713 / 5,713); the 20 ms timeout and HOLD/ABORT rules untouched |
| 7 | Focused P5 regression tests pass with the optimization | see §8.1 |
| 8 | Linux and Windows suites remain compatible | see §8.1 |
| 9 | Offline performance improves under representative pacing | ✔ at 20 Hz: p99 -39 %, max -35 % (matched, 5 trials) |
| 10 | Scheduling and tail-latency risks documented | ✔ (§6) — **the risk is not removed** |

### 8.1 Regression results with the optimization patched in

The existing suites ran unmodified, with
[`polar_patch_plugin.py`](../diagnostics/p5_inference_benchmark/polar_patch_plugin.py) swapping in the
CPU-D polar profile for the whole session. The plugin counts calls, so a session that never exercised the
optimized function cannot pass silently.

| Suite | Interpreter | Result | Optimized calls |
|---|---|---|---|
| P5 offline suites (8 files: contract, policy, shield, training, mission, runtime, comparison, …; ROS harness excluded) | Linux `/usr/bin/python3` (controller env) | **198 passed** | 2,767 |
| ROS harness, P4 + P5 (real rclpy, offline) | Linux controller env | **8 passed** | 0: the harness runs never reach a TRANSIT policy decision, so they cover start-up integrity, not this path |
| Windows full suite (`tests/`) | Windows Python 3.13.5 | **1,538 passed, 48 skipped**, the same counts as the accepted baseline | 2,767 |

No source file was modified for these runs (`git status` clean apart from the new diagnostics and docs).

**Readiness conclusion.**
* The CPU-D change is **technically ready to integrate** as a bit-exact, minimal source change.
* It is **not sufficient evidence for another acceptance attempt**:
  * The measured tail mechanisms are external to the computation.
  * One of them reached 9.91 ms on the optimized path even with no load.
  * The in-flight cause is unattributed.
* A new candidate built on CPU-D alone would face the same failure mode at a lower, but unquantified,
  rate. It would need about 7 flights plus Gate A at 0 exceedances.

**Evidence still missing, and obtainable only with a loaded simulator:**
* which class (guest preemption or charged slowness) produces policy-call spikes during a real flight;
* how often each occurs;
* whether CPU-D plus a host setting keeps those calls under 10 ms.

## 9. Recommendation: Path B — host scheduling investigation, with the Path A change prepared

**Why not Path A now.** Its conditions on equivalence, improvement, safety and documentation are met.
But significant spikes remain under load, the in-flight mechanism is unattributed, and an idle optimized
call came within 0.09 ms of the limit. Another acceptance attempt now would consume a candidate on an
unquantified risk.

**Why not Path C.** Equivalence holds, nothing unsafe was introduced, and credible timing validation is
feasible: the run-queue instrumentation cleanly attributes stalls.

**Proposed Path B sequence (none started; each step needs explicit operator approval):**

1. **Performance-mode offline comparison (approval 1): done, §5.1.**
   * With PredatorSense *Performance* set by the operator, the charged-slowness class did **not** shrink.
   * The production path even showed two idle calls over 10 ms in that class.
   * The Windows overlay stayed at power efficiency.
   * **A host power setting is therefore not supported as a fix.** If one is to be pursued, it first needs
     a setting the machine itself reports, for example a firmware or PredatorSense state readable by the
     runner.
2. **Loaded-SITL scheduling diagnostic (approval 2; not an acceptance gate).**
   * Run a diagnostic copy of `p5_control` outside the frozen sources, under a separate diagnostic
     namespace and run directory, never bound to a manifest or gate.
   * The copy has CPU-D integrated and records, for each policy call:
     * `inference_s`;
     * thread CPU time;
     * run-queue wait and timeslices from schedstat, read just before and after the timed region and never
       inside it;
     * context switches;
     * the Linux CPU;
     * guest `procs_running`.
   * In parallel it samples per-process CPU for Gazebo, PX4 and perception at 10 Hz.
   * Flights: S2 x 3 in the current power mode and, if approval 1 showed a benefit, S2 x 3 in Best
     performance.
   * Outputs:
     * the spike class distribution in real flight;
     * the D-path margin under real load;
     * whether the simulator's thread bursts oversubscribe the 24 vCPUs.
   * Same safety envelope as the accepted stack. No gate credit, and no change to any frozen candidate.
3. **Then decide:**
   * **Path A (new candidate)** if loaded flights show the D path with comfortable margin, for example no
     call above about 5 ms over several flights, under a documented host configuration.
   * **Path A plus a runner mitigation** if guest preemption dominates, for example reserving vCPUs for
     the controller or bounding simulator worker threads. That is a P5 runner change with its own
     evidence; P3 and P4 stay untouched.
   * **Path C** if neither brings the spike rate to a level where about 7 flights pass with high
     confidence.

The 10 ms maximum is not proposed for change at any step.

### 9.1 Approval 2: loaded-SITL scheduling diagnostic — results (2026-11-01)

**Authorization and scope.**
* Operator approval in chat ("Authorize diagnostic"), recorded in
  `~/uav_autonomy/diag/p5-sched-diag-approval.json`: S2 only, variant D, at most 3 flights, no acceptance
  credit, no automatic retry.
* The approval record itself enforces the flight cap.

**Implementation** ([`diagnostics/p5_sched_diag/`](../diagnostics/p5_sched_diag/)):
* `diag_runner.py` subclasses the accepted `P5Runner`, imported unchanged from its file. Through it, the
  accepted P4 and P3 runners launch the identical stack: private namespace, loopback only, exclusive
  workspace lock, the same PX4, Gazebo, QGC, DDS, sensor and P4 perception processes, and ordered shutdown.
* Preconditions keep every environment, namespace, lock, disk and WSL check, and re-verify manifest
  `db4c4779…` (model, schema, P3 and P4 baselines). No gate authorization, marker or result is read or
  written.
* `diag_control.py` runs the installed `control_node.main` unchanged, with two in-process changes:
  * the CPU-D polar profile is swapped in;
  * `PolicyRanker.choose` is wrapped to log scheduler evidence read before and after each call, never
    inside its timed region.
* A 10 Hz host sampler records the guest's runnable tasks, steal time and per-process runnable threads.
* The namespace owner is a byte-identical copy (`0d1c2d27…`) of the owner the QGC build pins.
* The offline dry check (`dry_check.py`) proved:
  * the refusals outside the namespace;
  * the wrapper logs every call;
  * scores and choices stay bit-identical before any flight.

**Flights.** Three S2 flights, sequential: all `status: passed`, no residual processes, no retry needed.

| Run | Evaluator (standard `p5_evaluate`, in memory) | Clearance / collisions / tracking | Policy calls, failures, outside mask, fallbacks, participation, replay mismatches | Inference p50 / p99 / max (ms) |
|---|---|---|---|---|
| `20261101T071454Z-p5diag-flight-S2-c95389ca` | passed, no failures; `goal_reached_with_detour` | 0.410 m / 0 / 0.246 m | 834, 0, 0, 0, 1.0, 0 | 0.354 / 1.366 / **2.898** |
| `20261101T071758Z-p5diag-flight-S2-d8202341` | passed, no failures; `goal_reached_with_detour` | 0.489 m / 0 / 0.242 m | 832, 0, 0, 0, 1.0, 0 | 0.354 / 1.212 / **3.040** |
| `20261101T072058Z-p5diag-flight-S2-de524fca` | passed, no failures; `goal_reached_with_detour` | 0.407 m / 0 / 0.255 m | 832, 0, 0, 0, 1.0, 0 | 0.353 / 1.366 / **3.040** |

Previous S2 flights of the production path, for reference: Gate B S2 p50 0.70 ms, max 6.00 ms; Gate C S2
p50 0.57 ms, max **10.84 ms**.

**Scheduling evidence over the decision window** (first to last policy call, about 42 s per flight;
2,498 calls; 1,246 host samples):
* Guest runnable tasks peaked at 14, 16 and 21. No sample exceeded the 24 vCPUs.
* Run-queue wait occurred on 3 of 2,498 calls, at most 0.027 ms. Involuntary switches: 3 calls.
* Steal time: 0.
* Calls over 3 ms: 2 (3.04 ms each). Both were CPU-charged: zero run-queue wait, CPU ≥ wall.
* Calls over 5 ms: **0**.
* In each flight, the only host sample above 24 runnable tasks came from `gst-plugin-scan` (the GStreamer
  plugin scanner, up to 24 runnable threads). It ran for 0.1-0.5 s at stack start-up, about 82 s
  before the first policy decision, and never overlapped decisions.
* The Gazebo server (`ruby`) peaked at 11-19 runnable threads, PX4 at 6-8.

**Interpretation.**
1. **In these loaded S2 flights, the guest was not oversubscribed while decisions ran.** Guest
   preemption, the class 1 mechanism of §6, was effectively absent from the decision path.
2. **The optimized path's in-flight tail is small:** max 3.04 ms over 2,498 calls, about 3.3x below the
   limit. The production path's per-flight maxima on S2 were 6.0 ms and 10.84 ms.
3. **By elimination, the Gate C 10.84 ms breach was most likely class 2 (CPU-charged slowness).** §5.1
   reproduced that class offline at 16.6 ms on the production path. This remains a **hypothesis**: Gate
   C was not instrumented, and its host conditions on 2026-10-31 may have differed.
4. **Limits of this evidence.**
   * Zero calls over 5 ms in 2,498 bounds the per-call rate below about 1.2 in 1,000 at 95 % confidence.
     That is far from proving zero over a full gate sequence of about 5,900 policy calls.
   * Offline, class 2 produced a 9.91 ms call on the optimized path once (§5) in about 19,200 paced calls.
   * The residual risk is lower than for the production path, but not zero, and no measure inside the
     policy code removes it.

**Updated recommendation: Path A is now supported**, with the residual risk documented. The criteria are
met:
* bit-exact equivalence (§4, plus 0 replay mismatches in all three flights);
* improved p99 and maximum under representative load (in flight: max 3.04 ms against 6.0-10.84 ms);
* no safety or controller regression (evaluator passed, clearance ≥ 0.41 m);
* the remaining tail risk characterized (class 2, rare, external to the computation).

The next step is approval 3, below.

## 10. Contract revision 3 proposal (draft for review; NOT approved, nothing applied)

Needed only when a new candidate is pursued (Path A). It changes no threshold.

1. **Root-cause statement** (§6): two measured stall classes external to the computation; in-flight
   attribution pending the approval-2 diagnostic. The remaining uncertainty is stated in the revision.
2. **Implementation:** `observation_adapter.polar_profile` in the vectorized form of §3. It must be
   bit-identical to the revision 1 loop, enforced by a regression test that keeps the loop as an oracle.
   The model, weights, normalization, schema `(26, 46)`, candidate order, mask, selection, shield,
   supervisor and timeout are unchanged.
3. **Thresholds unchanged:** inference p99 ≤ 5 ms and max ≤ 10 ms (wall, feature build + forward +
   selection, as measured today); 20 ms runtime timeout; 0 policy failures; participation 1.0; and all
   P2, P3 and P4 limits.
4. **Identity:** new commit, `build_p5.sh --build` receipt, `p5_source_files`, Policy A approval for the
   new commit, and a fresh freeze with a new manifest SHA.
5. **Runtime prerequisites: none proposed.**
   * Approval 1 found no benefit from the host power setting, and Windows never reported a change (§5.1).
   * Approval 2 found no guest oversubscription during decisions (§9.1).
   * Neither justifies a host or runner prerequisite. The Windows power overlay and PredatorSense state
     are documented as observed conditions only.
6. **Optional evidence addition (operator's choice; not part of the minimal change):**
   * Per policy call, record the run-queue wait, timeslices and thread CPU time, read outside the timed
     region, in a `p5-policy-ledger-v2`. Reported only, never limited.
   * It would make any future breach attributable, as §9.1 did for the diagnostic.
   * Cost: it touches `candidate_ranker.py` or `mission.py`, `policy_ledger.py` and the evaluator, which
     widens the change and its regression scope.
   * Recommendation: keep the minimal change. The diagnostic runner remains available for attribution if
     a gate fails again.
7. **Required regression:**
   * the §4 differential tests;
   * the P5 offline suites;
   * the ROS harness (P4 + P5);
   * the Windows full suite;
   * the P3 and P4 manifest verification;
   * a drift guard proving no P2, P3 or P4 file changed.
8. **Sequence:** revision 3 approval, then implementation and tests, Policy A approval, freeze, and Gates
   A → B → S → C, each separately authorized, one run each, no retries. No credit from `db4c4779…`; its
   failed Gate C stays preserved.

## 11. Explicit operator approvals needed next

| # | Decision | What happens on approval |
|---|---|---|
| 1 | ~~Set Windows *Best performance* and authorize the offline comparison~~ **Done (§5.1).** The Windows overlay never changed; PredatorSense Performance showed no measurable benefit | — |
| 2 | ~~Authorize the loaded-SITL scheduling diagnostic~~ **Done (§9.1).** 3 S2 flights, all passed; max 3.04 ms; no guest oversubscription during decisions | — |
| 3 | **Approve revision 3 (§10) and production integration of CPU-D**: the one-function change in `observation_adapter.py`, plus the in-test oracle differential test | implementation, the §10 regression set, `build_p5.sh --build`. Then Policy A approval and freeze are each asked separately, and Gates A → B → S → C each need their own authorization |

Nothing in this plan authorizes integration, re-freezing or any gate.
