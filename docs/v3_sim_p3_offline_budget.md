# P3-R2 offline evaluation budget

`p3-offline-budget-v1` freezes **90 seconds for control and 120 seconds for
sensor evaluation**, with a separate maximum 10-second owned-child reap.
There is no mid-job extension. Flight's 600-second outer deadline, simulation
deadlines, 0.2-second consumption gap, 50 ms dispatch age, ACKs, freshness,
rates, dwell periods and physical tolerances are unchanged.

## Measurement method and scope

Measurements use the existing Ubuntu-24.04 environment and dependencies.
Control runs `verify_p2_evidence.sh`, sources the same ROS Jazzy/px4_msgs
overlay, and executes `venv-px4/bin/python`. Sensor runs
`verify_p3_coexistence.sh` and `/usr/bin/python3`. No simulator, ROS node,
flight publisher, replay, hardware access or new recording is involved.
Each run uses a fresh interpreter and exclusive analysis filenames. The
fixed representative set is the original P3 smoke and latest R1 smoke, two
repetitions each; second repetitions reuse warm filesystem caches. Initial
cache state is uncontrolled, not claimed cold. No caches are dropped, host
scheduling changed or unrelated workloads terminated.

The original recording has 4,148,864,486 bytes in the enumerated legacy
input set, including 4,032,921,600 depth bytes. It predates R1's runtime seal;
its existing ULog/bag/sensor manifests remain the integrity basis. The latest
R1 seal covers 4,371,651,598 bytes, including 4,051,353,600 depth bytes. Every
required raw hash and frame hash remains checked. R1's complete runtime seal,
callback ledger, exact source identity, ordering and full window coverage
remain required. A newer seal is not manufactured for the original smoke.

The measurement supervisor starts GNU `time` around the production wrapper
and records elapsed time, user/system CPU and peak RSS. It also records its
own monotonic elapsed time around the bounded subprocess. Those two elapsed
measurements differ on this host; both are retained without substituting one
for the other. Deadline enforcement uses Python subprocess timeout. Budget
assessment conservatively checks both elapsed values. No offline clock is
used to reinterpret PX4 source age or flight timing.

Historical control results are intentionally `unknown` under the new causal
contract, and exit nonzero. Historical sensor diagnostics still complete all
sensor checks and retain failed control acceptance. This demonstrates bounded
evaluation of a complete recording, **not successful flight acceptance**.
Production qualification never launches sensor evaluation after that result.

## Baseline profiling and measured repair

The preserved R1 arithmetic completed latest/original control evaluation in
281.475/246.657 supervisor seconds, CPU 300.705/264.580 seconds. Only forwarding
the existing exclusive output-name argument and binding the old run to its
recorded frozen inputs were adapted in the diagnostic snapshot. R1 source
and all original evaluations were preserved first. Latest baseline peak RSS
was 1,057,580 KiB; the original measurement's RUSAGE_CHILDREN RSS is a
cumulative upper bound from that same supervisor, not a separate exact peak.

A complete latest-recording cProfile run measured 646.722 profiled seconds
(647.772 wrapper seconds). These profiler-inflated times are not used as the
production budget. Its finalized report is byte-identical to the unprofiled
baseline report, SHA256
`fcfe5b208c242ab84b37154ea7b15652edb15017b1701785a7ba91c7f78bf5d5`.
The cProfile CLI swallowed the application's nonzero SystemExit and returned
zero; the report remained failed. That profiler exit code earns no acceptance.

| Profiled function/stage | Calls | Inclusive seconds |
|---|---:|---:|
| Fixed-window joins | 16 | 454.469 |
| Stream validation, including repeated window validation | 124 | 289.174 |
| Strict-prior as-of lookup | 39,550 | 258.301 |
| Source identity/payload computation | 688,045 | 158.152 |
| Additional full payload fingerprint | 657,311 | 73.139 |
| Reference classification | 1 | 26.085 |
| Yaw/reference-source and prior-state evaluation | 1 | 42.476 |
| Exact ROS/ULog overlap | 2 | 24.946 |
| Execution timing reconciliation | 1 | 13.500 |
| Bag loading/deserialization | 1 | 11.305 |
| Runtime finalization/artifact verification | 1 | 11.172 |
| ULog loading/native reconstruction | 1 | 9.030 |
| Callback source-delivery verification | 1 | 8.400 |
| Actual controller-source reconstruction | 1 | 7.674 |
| Window arithmetic after joins | 24 | 0.159 |
| Final report JSON generation (main's two dumps) | 2 | 0.085 |

Inclusive values overlap and must not be summed. The remaining 1.050 seconds
between wrapper elapsed and profiled total include unprofiled startup,
profiler finalization and exit; they do not isolate shell startup alone.
Payload hashing accounts for most of the separate aggregate JSON-dumps cost;
final report serialization was not the measured hotspot.

A separate full 10-second INITIAL_HOVER ULog window localized the same cost:
1,249 position rows, 6,245 as-of calls, 65.099 profiled seconds; as-of work
was 45.561 seconds and six stream validations 19.027 seconds. It overlapped
the full diagnostic profile and is not a budget measurement.

The repair validates and snapshots the six streams once per recording view,
then reuses their immutable ordered timestamp indexes across fixed windows.
Lookup retains `bisect_left(t)-1`, every collision member and the last validated
sample within the selected strict-prior group. Every native health, reset,
tracking and extreme-value check remains separate and unchanged. There is no
global cache, verdict reuse, downsampling, skipped hash, nearest-angle search,
interpolation or mutable evaluator parallelism. Independent reference/yaw/
delivery verification still performs its own required checks.

Performance-only replays kept the R1 yaw gate. Original/latest full reports
matched every baseline field except the two changed implementation hashes;
both still failed. They took 68.77/80.09 GNU elapsed seconds, CPU 71.49/81.39
seconds, peak RSS 717,612/1,149,520 KiB. This differential comparison separates
the performance repair from the deliberate A/B/C/D/E evidence revision.

## Final wrapper measurements

All **8/8** required offline jobs finalized within the unchanged budgets.

| Recording | Repetition | Job | GNU elapsed s | Supervisor monotonic s | CPU s | Peak RSS KiB |
|---|---:|---|---:|---:|---:|---:|
| Original P3 | 1 | control | 55.51 | 53.675 | 54.42 | 717,376 |
| Original P3 | 1 | sensor | 33.77 | 31.875 | 25.98 | 179,360 |
| Latest R1 | 1 | control | 88.43 | 84.670 | 85.97 | 1,149,180 |
| Latest R1 | 1 | sensor | 37.65 | 33.824 | 27.79 | 429,108 |
| Original P3 | 2 | control | 53.31 | 51.417 | 52.17 | 716,628 |
| Original P3 | 2 | sensor | 21.83 | 19.976 | 15.84 | 178,920 |
| Latest R1 | 2 | control | 73.09 | 69.286 | 71.35 | 1,146,612 |
| Latest R1 | 2 | sensor | 26.17 | 24.279 | 20.83 | 429,188 |

The largest control measurement is 88.43 s (1.57 s below 90); the largest
sensor measurement is 37.65 s. This limited headroom is a measured result,
not a worst-case guarantee. Ordinary regression work overlapped some first
repetition measurements; no workload was killed or host scheduling adjusted.
Both repetitions produced byte-identical control and sensor reports for each
recording. All control results are unknown and all sensor composites fail only
control acceptance. See the [complete measurement/evidence index](evidence/v3_sim_p3_r2_offline.json)
for input identities, exact commands, per-report hashes, retained baseline
profiles and unchanged-field differential results. No extra favorable run was
selected, and no timeout was extended.

## Qualification failure behavior

The actual qualification entry point calls `offline_jobs.evaluate_trial`.
It rejects preexisting output, executes control within the frozen limit,
then verifies complete strict JSON, enclosing run, full frozen arithmetic
identity, finalized marker, absence of historical credit and passed control
acceptance. A/B/C/D must all be verified before sensor starts. Sensor has its
own deadline and must identify the exact completed control-report bytes;
those bytes are rechecked before any count advances. Another flight cannot
start until all required results pass.

Timeout is `infrastructure_timeout` with `failure_stage=control` or `sensor`.
Other subprocess/JSON/hash/identity/unknown failures are
`offline_evaluation_failure`. They invalidate qualification progression while
retaining the independent launcher/flight behavior. Counts are incremented
only after both results pass. Owned session groups are terminated even when
the original leader has exited, so surviving descendants cannot hold an
evaluation open. No unrelated process is signalled.

Pure tests exercise the production helper and actual qualification main with
fake subprocesses: control timeout/nonzero, partial/stale output, other run,
missing/changed arithmetic hash, unknown/contradictory causal evidence,
sensor sequencing/timeout, stale output before launch, group cleanup after
leader exit and no next flight after failure. They do not launch simulation.

Reproduce the fixed historical measurements only under a **new** output
directory and label:

```bash
python3 simulation/px4_gazebo/scripts/measure_p3_offline.py \
  --runs-root "$HOME/uav_autonomy/runs" \
  --output-directory "$HOME/uav_autonomy/state/UNIQUE_MEASUREMENT_DIRECTORY" \
  --label UNIQUE-LABEL
```

New binaries, inputs outside the measured size/profile, or changed evaluators
require a new offline qualification. This measured workload is not a universal
execution-time guarantee. Exact MC observability remains blocked independently;
the budget cannot authorize another nominal flight.
