# v3-SIM P4 — final acceptance (deterministic non-learning obstacle avoidance)

**P4 COMPLETE (2026-10-30).** Contract `p4-acceptance-v1` with revisions 2–5
([contract](v3_sim_p4_contract.md), [plan](v3_sim_p4_plan.md)). Candidate `c110bd30` passed Gates A,
B, S and C, each run once under its own operator authorization, with no retries. PX4 SITL only (no
hardware); depth-only flights; no learning. The P3 v1.6 baseline still verifies unchanged.

## Accepted candidate

| Item | Identity |
|---|---|
| P4 manifest (canonical) | `c110bd30be22fb43e596cf1f4b01397cb97145138f8912d6535a7dc4c15da42b` (`~/uav_autonomy/state/p4-candidate-50d5002.json`) |
| Source commit | `50d5002e9670d70f07c43792415494ae70f4efe0`; P4 sources identical to `b7e4de4` (revision 5) |
| P4 package build | `c4bd5a55abd997207f74fc019759f820ef97b8d07dd2ab103c1e54b44faf2486` |
| P3 baseline | `1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c` (`p3-acceptance-v1.6`) |
| Policy A | `bf1ef9b1…b3b0`, unchanged; approval `state/p4-policy-A-approval-50d5002.json` |
| Runtime | PX4 v1.17.0 with the MC causal-trace overlay `7542919c…`; QGC unattended `82994283…`; Gazebo Harmonic; ROS 2 Jazzy; WSL2; `hardware:d3d12` |

## Gates (operator Neura-Shadow, 2026-10-30)

| Gate | Result record (sha256) | Runs |
|---|---|---|
| A — readiness, fixtures, faults | `p4-gate-A-c110bd30….result.json` (`ffbd4ee9…f4fe9`) | P3 readiness, P4 readiness under load, G2/G4/G6, adapter_stop, bridge_stop: 7/7 passed |
| B — smoke | `p4-gate-B-c110bd30….result.json` (`b2fe75c5…dc26`) | S2: passed |
| S — safety | `p4-gate-S-c110bd30….result.json` (`8ef9cad4…0ebf`) | S1, S5, S6: 3/3 passed |
| C — qualification | `p4-gate-C-c110bd30….result.json` (`c7ce0752…de3a`) | S2, S3, S4: 3/3 passed |

## Flight evidence (all with acceptance credit; every owned process stopped gracefully)

| Run | Outcome | Collisions | Min truth clearance | Max tracking | Compute p99 / max |
|---|---|---|---|---|---|
| B S2 `20261030T141506Z…9b8ef2b0` | goal with detour, goal 0.067 m | 0 | 0.410 m | 0.252 m | 0.025 / 0.037 s |
| S S1 `20261030T143104Z…24276ada` | goal, 0.081 m, no supervisor transitions | 0 | — (no obstacle) | 0.233 m | 0.027 / 0.043 s |
| S S5 `20261030T143402Z…e0f9816e` | safe refusal: HOVER → ABORT (`hover_timeout`), landed to `COMPLETE` 6.7 s after LAND | 0 | 0.903 m | 0.236 m | 0.024 / 0.039 s |
| S S6 `20261030T143657Z…6f435d5e` | safe stop on sensor loss: HOVER 0.28 s after the last usable snapshot, 0.0 m / 0.0° while stale, ABORT at 10 s, landed in 6.7 s | 0 | 1.187 m | 0.237 m | 0.029 / 0.030 s |
| C S2 `20261030T144136Z…abeea2d5` | goal with detour, 0.023 m | 0 | 0.446 m | 0.256 m | 0.025 / 0.033 s |
| C S3 `20261030T144442Z…7907931e` | goal with detour, 0.047 m | 0 | 0.881 m | 0.235 m | 0.026 / 0.043 s |
| C S4 `20261030T144757Z…2fc2af47` | goal with detour (two obstacles), 0.024 m | 0 | 0.474 m | 0.239 m | 0.023 / 0.032 s |

Limits (contract §6, §6.1): 0 collisions, clearance ≥ 0.20 m, tracking ≤ 0.30 m, response latency
≤ 0.25 s, compute p99 ≤ 0.050 s and max ≤ 0.090 s.

Every flight also passed the unchanged P2/P3 checks: P2 timing with its full flight requirements,
the estimator reference and yaw ownership, ROS/ULog pairing, landed and disarmed, RGB absence, MC
causality, Policy A (401 × 1, 512 × 34) and the D3D12 renderer.

## How it got here (contract revisions and fixes; details in the plan)

* **Revisions:**
  * 2 (`d3c06b4`): fault window, collision validity, performance limits.
  * 3 (`ea2c9e3`): 360° map scan and initial feasibility.
  * 4 (`65b99e4`): 0.20 m/s transit speed, so PX4's steady lag is about 0.21 m, inside 0.30 m.
  * 5 (`ae0e037`): a supervisor abort lands through P2's nominal sequence to `COMPLETE`.
* **Defects found by gates and fixed offline before re-freezing:**
  * The runner's SITL gate environment.
  * `p4_control` trace spans and BEST_EFFORT QoS.
  * A world plugin that disabled PX4's server config.
  * Perception compute; vectorized with identical outputs.
  * The truth-observer shutdown race.
  * Graph-check tracing.
* **Intermittent environment events, re-frozen unchanged by operator decision:** a Gazebo forced stop
  (§20), and one extra QGC request (§46).

## Retained evidence

The accepted runs above and their gate, authorization, consumption, manifest and approval records
are in `~/uav_autonomy/{runs,state}`. Earlier candidates' records also remain there as history; no
cleanup has been done. Changing any manifest-bound P4 file (`simulation/px4_gazebo_p4/{configs,
scripts,validation,ros2_ws/src/gwm_obstacle_avoidance}`) or P3 file breaks `verify`.
