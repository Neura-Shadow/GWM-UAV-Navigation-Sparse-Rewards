# P3 renderer contract revision — proposal `p3-acceptance-v1` → `p3-acceptance-v1.1`

**Status: PROPOSAL ONLY — NOT APPLIED, NOT APPROVED.** P3 remains INCOMPLETE, qualification
0/3. Validation V gives no acceptance credit.

Date: 2026-10-25. Basis: the operator-authorized ground-only llvmpipe validation V of
candidate `1e439b44caf24bdb4d1a5386046d84c3a9fb34da0a0bcf43d565b52461edc68f` (commit `6e80422`).

## 1. Why

On this host (WSL2, Ubuntu 24.04.3) the hardware path cannot pass the unchanged cleanup rule.
With D3D12 on the RTX 4060, Gazebo's own SIGINT shutdown faults when the sensors render
thread exits and glibc runs a GPU-stack thread-specific-data destructor. The exact owning
module is **unconfirmed** (acceptance report sections L–M). The same shutdown on llvmpipe
exits 0.

## 2. Evidence (validation V, 2026-10-25 08:11–08:18 UTC)

Result `state/p3-gate-V-1e439b44….result.json`, `accepted: true`,
`acceptance_credit: false`. The plan was identical to Gate A, with no flight.

| Run | Gazebo exit | source Hz (≥25) | max gap s (≤0.12) | processed Hz (≥8) | max age s (≤0.25) | max stall s wall (≤1.0) | wall delivery Hz |
|---|---|---|---|---|---|---|---|
| readiness `967ce4d8` | 0 | 30.30 | 0.036 | 9.93 | 0.072 | 0.174 | 17.1 |
| plane2 `28317f3c` | 0 | 30.30 | 0.036 | 10.0 | 0.060 | 0.056 | 30.3 |
| plane4 `8468ee6b` | 0 | 30.30 | 0.036 | 10.0 | 0.056 | 0.058 | 30.3 |
| plane6 `a052b5aa` | 0 | 30.30 | 0.036 | 10.0 | 0.060 | 0.055 | 30.3 |
| oblique `8ec85ff3` | 0 | 30.30 | 0.036 | 10.0 | 0.056 | 0.066 | 30.3 |
| asymmetric `504b08ad` | 0 | 30.30 | 0.036 | 10.0 | 0.068 | 0.088 | 30.2 |
| out_of_range `0f556e94` | 0 | 30.30 | 0.036 | 10.0 | 0.064 | 0.056 | 30.3 |
| interruption `dbd51338` | 0 | 30.31 | 0.036 | 10.0 | 0.052 | 0.059 | 30.3 |

Every independent evaluation passed: geometry accuracy, valid fraction, out-of-range,
interruption detection, recorder completeness, association, native commands (policy A),
MC causality and P2 ground timing. Graceful cleanup passed for every process in all 8 runs.
Renderer `Mesa / llvmpipe (LLVM 20.1.2, 256 bits)`; readiness RTF 0.556.

## 3. Proposed change (exactly this, nothing else)

| Item | v1 | v1.1 (proposed) |
|---|---|---|
| Manifest `renderer_requirement` | `hardware` | `software:llvmpipe` (Mesa llvmpipe, recorded strings) |
| Runner environment for A/B/C | D3D12 + NVIDIA adapter | no D3D12 variables (Mesa default llvmpipe), as in V |
| Evaluator renderer check for A/B/C | `renderer_unidentified` unless non-software | `renderer_not_declared_llvmpipe` unless the recorded renderer is llvmpipe |
| Every sensor, geometry, freshness, timing, RTF-measurement, recorder, command-policy, MC and cleanup limit | — | **unchanged** |

A v1.1 candidate would be frozen fresh and still needs Gate A, then B, then C, each with its
own authorization. Validation V does not substitute for any gate.

## 4. Residual risk (not removed by this revision)

* **Flight timing is unproven on llvmpipe.** P2 flight limits (including the 2 s
  `vehicle_land_detected` wall freshness, which an archived smoke missed at 2.008 s) are
  judged unchanged at Gate B/C. If they fail, the recommendation is a native Linux GPU host,
  not a relaxation.
* Readiness wall delivery is 17.1 Hz with the full stack, against the 25 Hz limit, which
  applies to source and simulation-time delivery and passes. Margin to real time is lower
  than with D3D12 (RTF 0.56 vs 0.82).
* The archived llvmpipe −11 crashes (older runner, SIGTERM) remain unexplained; zero
  occurred in the 10 llvmpipe shutdowns since the pause + SIGINT sequence (8 in V, the Gate A
  `8feaa076` readiness run, and the section M diagnostic).

## 5. Approval required

An explicit operator approval of this revision (`p3-acceptance-v1.1`, this document's
commit) before any code or manifest change. Without it, Gate A stays `hardware`-only and
cannot pass on this host.
