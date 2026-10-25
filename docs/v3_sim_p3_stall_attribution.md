# P3 flight-timing stall attribution (offline, no gate credit)

Read-only analysis of retained runs. No runtime was started; no source, threshold or
retained artifact was modified. Evidence:
[`evidence/v3_sim_p3_stall_attribution.json`](evidence/v3_sim_p3_stall_attribution.json)
(53 input files with SHA-256). Host: i7-13700HX, 24 logical CPUs, WSL2.

Inputs: D3D12 Gate B flight `20261025T100510Z-p3-flight-5a2b42b8`, llvmpipe Gate B flight
`20261025T091152Z-p3-flight-c287b08f`, accepted Gate A readiness runs
`20261025T094817Z-p3-readiness-adc2a21a` (D3D12) and `20261025T085945Z-p3-readiness-93f53ed2`
(llvmpipe), and as a no-camera reference the 20 accepted P2-R3 flights of batch
`20260912T035329Z-p2-repeat-84bb288d` (same PX4, Gazebo, controller and 2 s rule; x500
without cameras).

## Observed facts

Local RTF is from the Gazebo-native `/clock` probe (`source/clock.jsonl`, ~10 Hz wall),
binned over 1 s of wall time. CPU is from `resources.jsonl` (1 Hz, cores = Δcpu_s/Δt).

| Measurement | D3D12 flight | llvmpipe flight | D3D12 readiness | llvmpipe readiness | P2 (20 flights) |
|---|---:|---:|---:|---:|---:|
| mean local RTF | **0.577** | 0.532 | 0.647 | 0.577 | ≈1.0 |
| p05 / min local RTF (1 s) | 0.405 / 0.109 | 0.456 / 0.259 | 0.555 / 0.104 | 0.543 / 0.520 | — |
| 1 s bins with RTF < 0.55 | 35 / 121 | 124 / 158 | 1 / 30 | 2 / 30 | — |
| land_detected wall gap p50 / max | 1.688 / **2.725** s | 1.848 / 2.658 s | 1.647 / 1.799 s | 1.712 / 1.798 s | 1.000 / **1.025** s |
| gaps > 1.8 s / > 2.0 s | 15 / 5 | 65 / 4 | 0 / 0 | 0 / 0 | 0 / 0 |
| Gazebo CPU median (cores) | 1.34 | 4.36 | 1.42 | 4.43 | — |
| all owned processes median / max (cores of 24) | 3.11 / 3.57 | 5.81 / 6.53 | 2.96 / 3.36 | 5.72 / 6.18 | — |
| loadavg-1 max | 5.84 | 8.88 | 2.94 | 3.83 | — |
| MemAvailable min | 6.4 GB | 6.1 GB | 6.6 GB | 5.9 GB | — |

1. **The stall is in simulation progress itself, not in the controller's receipt path.** For
   every land_detected gap over 1.8 s, the RTF implied by the gap (1.000 s sim divided by
   the wall gap) equals the independently measured `/clock` RTF over the same interval
   (D3D12 worst gap: 0.367 implied vs 0.369 clock; across all 15 D3D12 and 65 llvmpipe gaps
   the largest difference is 0.017). The controller recorded **zero** `bag_write`/`text_write_flush`
   spans over 50 ms in either flight.
2. **The deficit is mostly mean throughput, not rare stalls on a fast baseline.** The
   earlier "≈0.82" was one readiness sample (section L of the acceptance report). The
   failing D3D12 flight averaged 0.577 and D3D12 readiness 0.647, so the median 1 Hz gap
   is already 1.65–1.69 s. Dips on top of that cross 2 s. The same PX4/Gazebo/controller
   without cameras (P2) runs at real time (gaps 1.000–1.025 s).
3. **No P3 evidence-path component saturates or leads.** Recorder write latency is
   p50 0.016 s / max 0.071 s (D3D12). In each over-limit window, Gazebo-native depth and RGB
   receipt gaps rise with the sim slowdown (0.18–0.23 s at the worst gap) rather than
   preceding it, and MAVLink gaps behave the same way. Adapter CPU is uncorrelated with RTF
   (r = −0.06 D3D12, 0.04 llvmpipe). Consumer CPU (probe r = 0.78, bridge, controller)
   correlates **positively** with RTF: consumers do more work when the sim produces more,
   which is the signature of downstream load, not of a cause.
4. **On D3D12, Gazebo waits rather than computes in slow seconds.** Gazebo CPU averages 1.31
   cores in RTF < 0.55 seconds against 1.36 otherwise, and non-Gazebo CPU also drops
   (1.58 vs 1.73). Meanwhile loadavg rises as RTF falls (r = −0.55). Linux loadavg also
   counts uninterruptible (D-state) tasks, which is consistent with threads blocked in the
   kernel, but the 1 Hz `resources.jsonl` cannot show thread states.
5. **On llvmpipe, Gazebo is compute-bound** at about 4.4 cores (CPU rasterisation), RTF is
   steady around 0.53, and Gazebo CPU barely changes in slow seconds: a throughput ceiling.
6. **Structure:**
   * No clean periodicity. The RTF autocorrelation peaks at a 2 s lag (r 0.56 D3D12,
     0.62 llvmpipe), which means slow episodes persist for seconds. Episode spacing is
     irregular (D3D12 starts at 0, 22, 27, 38, 62, 66, 71, 73, 76, 79, 86, 100 s).
   * Phase: D3D12 over-limit gaps begin in STABILIZE_REFERENCE/INITIAL_HOVER at about 0.53
     RTF; the four worst (2.30–2.73 s) cluster at NORTH_TEST and the following RECOVERY.
     llvmpipe gaps are spread through every phase.
   * Drift: RTF falls through each run (D3D12 flight 0.618 → 0.536 between halves,
     −0.09/min; llvmpipe −0.03/min). This is compatible with thermal or cumulative load but
     confounded with mission phase.
   * Readiness shows the same mean deficit with fewer dips, so it is not flight-specific.

## Ranked hypotheses

Falsifiable predictions in the style of `v3_sim_p2_timing_contract.md`:

1. **GPU/WSL path inside Gazebo's step (D3D12 rendering and readback through dxg).**
   *For:* the deficit appears only when cameras render (P2 is at real time). On D3D12 Gazebo
   is not compute-bound in slow seconds (CPU falls), loadavg rises, and the host has more
   than 20 idle cores. llvmpipe fails differently, as a compute ceiling.
   *Against or unknown:* there is no thread-state, wchan or GPU timing data, so a GPU or
   kernel wait is inferred, not observed.
2. **P3 evidence-path CPU/IO (probe RGB metadata, recorder zlib + SHA-256, adapter decode,
   MAVLink capture, rosbag/ULog/event writes).**
   *For:* these run concurrently and their volume (RGB at about 186 MB/s through
   gz-transport) is large.
   *Against:* no saturation (3.1 of 24 cores in total), the adapter is uncorrelated, consumer
   CPU correlates positively with RTF, recorder latency stays small, controller writes are
   never slow, and the stall is in sim time (finding 1). Disfavoured, but **not excluded**:
   a subscriber can still cost Gazebo inside its own process (gz-transport
   serialisation/copy of 6.2 MB RGB per frame per subscriber, or render-on-demand when a
   topic has subscribers), and the retained data cannot isolate that.
3. **Host scheduling/thermal.**
   *For:* within-run RTF decline. *Against:* no memory pressure (≥ 5.9 GB available),
   moderate loadavg, idle cores. CPU frequency, temperature and Windows-host load were not
   recorded, so this cannot be excluded.
4. **PX4/Gazebo lockstep itself.**
   *Against:* the same lockstep runs at real time in P2. PX4 CPU is low (0.3 cores) and
   positively correlated with RTF (it follows the clock). Weakest hypothesis.

**What the retained data cannot distinguish:** whether the in-Gazebo wait is GPU/dxg render
or readback (environment, H1) or Gazebo's own sensor publication to the P3 subscribers (a P3
workload effect inside Gazebo, part of H2). No retained run has the cameras present with
P3 consumers removed, and no run has thread-level Gazebo state.

## Step 2 proposal — bounded labelled diagnostics (NOT run; requires explicit authorization)

* **Label and credit:** `diagnostic`; never gate credit, never a Gate B/C predecessor, no
  retries. A failed or interrupted start is retained and counted.
* **Shape:** ground-only (readiness-shaped: full PX4 + Gazebo + sensor stack, vehicle on the
  ground, never armed). No flight is needed, because the deficit and dips appear in
  readiness. Fixed **120 s** measurement window after the unchanged readiness settle.
* **Budget:** at most **6 starts**, D3D12 v1.2 profile, fixed order, declared up front:
  1. baseline (full frozen workload);
  2. source-probe RGB metadata observer disabled;
  3. raw recorder compression `none` (every frame still hashed and recorded);
  4. source probe disabled entirely;
  5. baseline repeat, to measure run-to-run spread before attributing any difference;
  6. reserved: `.wslconfig` CPU or power-plan variant, only if separately approved
     (a host change); otherwise unused.
* **Added passive instrumentation (all variants):** a 10 Hz sampler of every Gazebo thread's
  `/proc/<pid>/task/*/{stat,wchan}` (state R/S/D, wait channel, utime) and of host CPU
  frequency. This directly separates "blocked in a GPU/dxg kernel wait" from "running" and
  from "waiting on a subscriber".
* **Metrics per variant:** mean, p05 and minimum 1 s local RTF; the land_detected wall-gap
  distribution (same 1 Hz family); per-process and Gazebo CPU; Gazebo thread-state
  histogram during RTF < 0.55.
* **Implementation needed (reviewed, diagnostic-only):** a `D` scope in the runner with these
  explicit toggles and the sampler. The frozen candidates are not modified, and none of the
  toggles is available to Gates A/B/C.

## Decision

**Unattributed** between (1) the WSL GPU/dxg path inside Gazebo's step and (2) a P3
workload cost incurred inside Gazebo by its subscribers. P3 evidence-path CPU/IO outside
Gazebo is disfavoured, and controller-side IO is excluded. A host change is **not**
recommended yet.

**Smallest next observation:** the Step 2 ground-only diagnostic set above (at most 6
starts). Gazebo thread states during slow seconds decide H1 vs H2. Variants 2–4 then show
whether removing a specific P3 consumer restores RTF:
* if it does, the result is **P3-load-bound** and an off-critical-path redesign is proposed;
* if it does not, and threads sit in D-state on dxg, the result is **environment-bound** and
  a native Linux GPU host is recommended.

## Step 2 results — labelled diagnostics (operator-authorized, run 2026-10-25)

Gate `D` (commit `c9be22c`), diagnostic candidate `state/p3-candidate-c9be22c-diagnostic.json`,
canonical SHA-256 `85bdd2a4b7538a89dcd777079170853908dfc125f0fc472a20482447a4d10559`, frozen with
the default `strict_zero` policy because diagnostics are never acceptance-evaluated.
Authorization `p3-auth-D-85bdd2a4b753.json` (Neura-Shadow), consumed. Result
`p3-gate-D-85bdd2a4….result.json`: **5 of 5 starts completed**, none failed; the 6th
(host-change) start was not used. D3D12 v1.2 profile (RTX 4060) in every run, vehicle on the
ground and never armed, 120 s wall window each, `acceptance_credit: false`.
Measurement: [`evidence/v3_sim_p3_stall_diagnostic.json`](evidence/v3_sim_p3_stall_diagnostic.json)
(`p3_stall_diagnostic.py`, input hashes included).

| Variant (run) | mean / p05 / min 1 s RTF | wall time per 1.000 s sim p50 / p95 / max | > 1.8 s / > 2.0 s | Gazebo CPU | total CPU |
|---|---|---|---|---|---|
| baseline `ac764f9f` | 0.572 / 0.422 / 0.255 | 1.695 / 2.186 / 2.891 | 15 / 5 | 1.33 | 2.84 |
| **no RGB observer** `337a7122` | **1.000 / 0.999 / 0.988** | **1.000 / 1.001 / 1.010** | **0 / 0** | 1.03 | 2.85 |
| recorder compression `none` `7aa47bb7` | 0.607 / 0.537 / 0.494 | 1.636 / 1.793 / 1.873 | 4 / 0 | 1.33 | 2.92 |
| probe clock-only `a2044e0b` | 0.997 / 0.992 / 0.895 | 1.000 / 1.007 / 1.104 | 0 / 0 | 0.97 | 2.74 |
| baseline repeat `5070451c` | 0.559 / 0.413 / 0.349 | 1.761 / 2.226 / 2.610 | 24 / 12 | 1.31 | 2.71 |

* **Removing the single RGB subscriber restores real time.** The source-probe RGB observer is
  the only subscriber of the 1920×1080 RGB topic; the sensor bridge carries depth only. With
  it removed (variant 2, and variant 4, which also drops it), RTF is 1.0 and the 1 Hz family
  sits at 1.000 s, matching P2. Both baselines reproduce the Gate B deficit.
* **The observer's own work is not the cost** (about 0.13 cores). The recorder and adapter are
  not the cost either: compression `none` gives a small, within-noise improvement (0.607 vs
  0.559–0.572). The cost is Gazebo producing the 1080p RGB frames (render, readback,
  serialise) inside its step, which Gazebo does only while the topic has a subscriber.
* **Thread states:** Gazebo is not CPU-bound (1.3 of 24 cores, 2.3 GHz throughout). Over 95%
  of its thread samples are sleeping (S) in both slow and normal seconds. D-state
  (`dxgvmb_send_sync_msg`) is ≤ 0.06% of samples in every variant, including the real-time
  ones. The step therefore *waits* while RGB is produced, consistent with GPU completion or
  readback waits, which appear as S in user space. That cannot be proven here: all Gazebo
  threads are named `ruby`, so the render thread cannot be isolated, and no GPU timing is
  available.

## Decision

**P3-load-bound — specifically the frozen 1920×1080 @ 30 Hz RGB workload, produced inside
Gazebo's simulation step whenever it is consumed.** It is not the P3 evidence-path CPU/IO
(observer, recorder, adapter, probe ledgers, controller writes): removing that work
separately does not restore real time, while removing the RGB consumer does.

No reviewed change can move this work off the timing-critical path while keeping the frozen
workload and evidence completeness. The work is Gazebo's own synchronous sensor rendering,
not a P3 process; any RGB consumer, observer or not, triggers it; and the observer itself is
already cheap. Two directions remain, and each needs an operator decision:

1. **Keep the frozen workload; test the GPU path (recommended next step).** On this host the
   RGB production cost behaves like a wait on the WSL D3D12 (and llvmpipe) render/readback
   path. Run the identical two-start diagnostic (baseline, no RGB observer; 120 s, ground,
   no credit) on a native Ubuntu 24.04 + NVIDIA host, for example an external-SSD install on
   this laptop. If the baseline reaches RTF about 1.0 there, migrate: own build receipts,
   GPU-library pins, a new candidate and a fresh Gate A. If it does not, direction 2 is the
   only remaining path.
2. **Revise the RGB workload in the contract** (rate, resolution or capture strategy). That
   is a material acceptance change and is not proposed here without operator direction.

No candidate, threshold, P2 file or earlier gate result was changed by this analysis.

## Direction 1 prepared (operator choice, 2026-10-25)

The identical two-start diagnostic (baseline, then no RGB observer; 120 s; ground; no credit)
is prepared as gate `N` for a native Ubuntu 24.04 + NVIDIA host. The setup checklist,
decision rule (fixed in advance) and run commands are in
[`v3_sim_p3_native_gpu_diagnostic_setup.md`](v3_sim_p3_native_gpu_diagnostic_setup.md).

On that host, gate N uses `configs/p3_native_gpu_profile.json` (NVIDIA EGL/GLX vendor, no D3D12
variables, no preload or shim). The manifest records, rather than pins, the native NVIDIA
libraries and driver version. Gate N refuses to run on WSL, and gates A/B/C/D/V refuse to run
off WSL. The RGB workload and every limit are unchanged. Nothing was run.
