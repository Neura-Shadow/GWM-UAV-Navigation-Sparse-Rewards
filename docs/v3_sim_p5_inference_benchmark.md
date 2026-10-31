# P5 CPU vs GPU inference benchmark and tail-latency investigation

**Status: diagnostic only (2026-10-31).** This report does not change the accepted model, the frozen P5
sources, the acceptance thresholds or the failed Gate C result (plan §13.13). No gate was run, no
simulator was started, nothing was installed, and no Windows or WSL setting was changed. Offline numbers
here do not establish SITL acceptance.

Code and raw results: [`diagnostics/p5_inference_benchmark/`](../diagnostics/p5_inference_benchmark/)
(outside every manifest scope; the controller never imports it).

**Result.**
* **The GPU makes the decision slower and its tail worse.**
  * At 20 Hz, torch CUDA end to end reaches p99 13.0 ms and max 14.8 ms (float64), against 2.3 / 4.5 ms
    on the CPU.
  * Its scores are never bit-identical, although every decision matches.
* **A bit-exact CPU optimization halves the decision** (vectorizing the polar profile). At 20 Hz it cuts
  the tail to max 2.0 ms.
* **Neither removes the cause.** The policy's computation is 0.2-0.4 ms. The 10-22 ms calls are
  reproduced offline only when the WSL guest is oversubscribed, with the thread preempted mid-call.
  Paced decisions also run 3x slower, on a host found in "Best power efficiency" mode.
* **Recommendation: Option C** (§11). Keep the CPU NumPy backend. Test the host power mode, and measure
  guest preemption in a separately authorized loaded-simulator diagnostic.

## 1. Verified hardware and runtime inventory

All entries were read with read-only commands on 2026-10-31.

| Item | Value |
|---|---|
| CPU | 13th Gen Intel Core i7-13700HX: 8 performance cores (16 threads) + 8 efficiency cores = 24 logical processors |
| Seen inside WSL2 | 24 vCPUs, presented as 12 cores x 2 threads (not the real hybrid topology); no `hybrid_cpu` flag; no `cpufreq`; affinity mask `ffffff`; cgroup `cpu.max` unlimited |
| WSL2 | kernel 6.18.40.1-microsoft-standard-WSL2, Ubuntu 24.04.3 LTS, 15.9 GB RAM visible; no `.wslconfig` (defaults); kernel reports no steal time; `CONFIG_PREEMPT_DYNAMIC`, booted `PREEMPT_NONE` |
| Windows power | plan *Balanced* (`381b4222…`); power-mode overlay **`961cc777…` = "Best power efficiency"** on both AC and DC; on AC power. Not changed |
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU, 8 GB, compute capability 8.9, driver 571.96, CUDA 12.8; visible in WSL through `/dev/dxg` and `/usr/lib/wsl/lib/libcuda.so` |
| Controller interpreter | `/usr/bin/python3` 3.12.3, NumPy 1.26.4 linked to the **reference (netlib) BLAS** `libblas.so.3` 3.12.0, not OpenBLAS, so `OPENBLAS_NUM_THREADS=1` has no effect; no torch, ONNX Runtime, TensorRT or CuPy |
| Evaluator venv `venv-px4` | NumPy 2.5.3; no GPU runtime |
| GPU-capable env (pre-existing) | WSL conda env `thesis`: Python 3.13.0, NumPy 2.3.5 with OpenBLAS 0.3.30, **torch 2.9.1+cu128, CUDA available** |
| Windows Pythons | anaconda (torch 2.12 CPU only), Python 3.13 (torch 2.13 CPU only), envs `UAV-PY13`, `thesis_win` (no torch) |
| ONNX Runtime, TensorRT | not installed in any environment checked |

## 2. Benchmark procedure (reproducible)

* **Inputs.**
  * Policy: the frozen checkpoint `p5-ranker-ee006a80…`, loaded through the production `FrozenPolicy.load`
    (SHA-256, architecture, schema hash and `trained` checks).
  * Ticks: the first 120 TRANSIT ticks of the failed Gate C S2 flight
    (`20261031T121719Z-p5-flight-S2-2e9825f4`) that the evaluator's `reproduced_ticks` rebuilds from the
    run's own records with P4's recorded plan reproduced field for field. This is the plan §13.15
    selection.
  * Spike ticks: 491 (10.84 ms in flight) and 337 (4.22 ms) cannot be rebuilt that way. Their exact
    feature tensor, mask and scores come from the policy ledger, which the equivalence checks use.
    All 833 invoked ledger records are used for score equivalence.
* **Reconstruction fidelity.** Against the flight ledger, 64 of the 120 rebuilt tensors are bit-exact.
  The rest differ by ≤ 2.8e-16 in a context feature, giving score differences ≤ 1.0e-14. All 120
  policy choices are equal. Timing does not depend on these last-bit differences.
* **CPU harness** (`cpu_bench.py`, run under the controller interpreter by `run_controller_python.sh`).
  Each trial alternates two calls on the same tick sequence:
  * production `PolicyRanker.choose`, timed by its own clock (`inference_s`, the gated quantity);
  * a phased replica of the same steps in the same order:
    * `build_candidates`: `observation_adapter.build` minus its polar profile, i.e. candidate features
      with their scaling (P5 has no separate normalization stage) and the mask;
    * `polar_profile`: the context polar profile inside `build`, timed through a wrapper installed in
      the benchmark process only;
    * `validate`: mask equals the progress set, then the input shape, dtype and finite checks;
    * `forward`: the three dense layers;
    * `rank`: `policy_choice`, the masked argmax with the P4 tie order;
    * `select`: the mask re-check and the slot lookup.

  Every phased call must reproduce the production scores bit for bit and the same choice, or the trial
  aborts. Per call the harness also records thread CPU time, the Linux CPU before and after, and the
  thread's voluntary and involuntary context switches. Per trial it records process CPU utilization and
  peak RSS.
* **Pacing.** *Back to back*: no sleep between calls. *20 Hz*: 50 ms sleep between calls, like the control
  timer. Three trials per configuration (30 s back to back, 120 s paced) unless stated otherwise.
* **GPU harness** (`gpu_bench.py`, in the conda env `thesis` via `run_gpu_python.sh`). Torch CUDA
  float64 and float32 run the same three layers with the frozen weights.
  * *GPU-A*: forward only, with the input already on the device and `torch.cuda.synchronize()` before
    and after each call.
  * *GPU-B*: end to end, timing CPU feature build, host-to-device copy, forward plus synchronize,
    device-to-host copy, `policy_choice` and the mask re-check.
  * Cold calls (first after load) are reported separately.
* **Stress** (`load.py`, bounded, each worker exits by itself):
  * guest oversubscription: 30 busy processes inside WSL;
  * host contention: 24 busy processes in a Windows Python;
  * GPU contention: continuous 4096² float32 matmuls in a second process.
* **Affinity** (`--affinity`): process-scoped `sched_setaffinity` for one run only.

## 3. CPU baseline (CPU-A): the unchanged production decision path

Controller interpreter, no affinity, power mode as found. All times are in ms. "Production" is the gated
`inference_s`; phases come from the phased replica. Pooling across trials uses the median of the per-trial
p50 and the largest per-trial p95, p99 and max.

| Pacing | Production calls | p50 | p95 | p99 | Max | > 5 ms | > 10 ms | Process CPU |
|---|---|---|---|---|---|---|---|---|
| Back to back (3 x 30 s) | 100,091 | 0.391 | 0.736 | 1.097 | 5.242 | 1 | 0 | 107 % |
| 20 Hz (3 x 120 s) | 3,499 | 1.272 | 1.922 | 2.312 | 4.478 | 0 | 0 | 3 % |

Phase breakdown (p50 / p99 / max):

| Phase | Back to back | 20 Hz |
|---|---|---|
| build_candidates | 0.048 / 0.150 / 4.433 | 0.246 / 0.472 / 1.364 |
| polar_profile | 0.240 / 0.702 / 4.802 | 0.747 / 1.438 / 5.413 |
| validate | 0.010 / 0.038 / 0.908 | 0.041 / 0.133 / 0.199 |
| forward (MLP) | 0.078 / 0.222 / 2.487 | 0.186 / 0.442 / 0.563 |
| rank (masked argmax) | 0.010 / 0.050 / 3.511 | 0.040 / 0.130 / 0.795 |
| select | 0.000 / 0.001 / 0.124 | 0.001 / 0.002 / 0.007 |
| **total (phased)** | 0.389 / 1.106 / 5.004 | 1.277 / 2.277 / 7.122 |
| thread CPU time | 0.415 / 1.183 / 5.320 | 1.385 / 2.459 / 7.611 |

Measured facts:

* **Feature construction is most of the decision.** The polar profile alone is about 60 % of the median
  decision. The MLP forward pass is about 20 % (78 µs back to back, through the reference BLAS).
  Validation, ranking and selection together are about 5 %.
* **Pacing makes the same work 3.3x slower at the median** (0.39 ms back to back, 1.27 ms at 20 Hz). Every
  phase slows by 2.4-5x, and thread CPU time rises with wall time. The process sleeps 97 % of the time at
  20 Hz.
* **Rare multi-millisecond stalls land in whichever phase is running.** That includes `rank`, whose median
  is 10 µs but whose maximum was 3.5 ms. The slowest production calls were charged in full as thread CPU
  time (5.24 ms wall, 5.58 ms CPU), with no context switch and no change of Linux CPU during the call.
* **No call exceeded 10 ms on the idle host** in 103,590 production calls. The in-flight 10.84 ms has not been
  reproduced without the simulator running.
* Peak RSS 210 MB. Calls that migrated between Linux CPUs: ≤ 0.04 %.
* **Not established from these measurements:** *why* the core runs slower. Plausible mechanisms include
  host frequency and power-state management under the "Best power efficiency" mode, cold caches after
  sleep, and hypervisor vCPU scheduling or efficiency-core placement. CPU time ≥ wall time on a slow call
  shows the guest charged the thread as running. It does not by itself prove any of these mechanisms
  (§9).

## 4. CPU-B: performance mode — not measured

Windows was found in the "Best power efficiency" power mode, and this task may not change host settings.
No performance-mode measurement exists. Section 3's numbers are all in that mode.

## 5. CPU-C: P-core affinity

**Mapping check** (`pe_map.py`). A fixed single-thread integer workload was pinned to each Linux CPU in
turn, in 4 rounds of 0.25 s per CPU.
* A given vCPU's speed changed from round to round: median spread 11.6 %, maximum 43.6 %.
* The 24 mean speeds form one continuous band, 0.80-0.94 of the fastest sample. There are no two clusters,
  as fixed P- and E-core bindings would produce (E-cores ~0.7x P-cores at this CPU's turbo limits).
* Conclusion: WSL2 vCPU numbers do not identify host P-cores. vCPUs are scheduled across host logical
  processors by Hyper-V, so Linux affinity cannot request a P-core. Host-side affinity would require
  changing Windows or WSL configuration, which is not authorized here.

**Affinity measurement** (`--affinity 4`, process-scoped, 20 Hz, 3 x 120 s; production path unchanged):

| Configuration (20 Hz) | Calls | p50 | p95 | p99 | Max | > 10 ms |
|---|---|---|---|---|---|---|
| CPU-A, no affinity | 3,499 | 1.272 | 1.922 | 2.312 | 4.478 | 0 |
| CPU-C, pinned to vCPU 4 | 3,513 | 0.761 | 1.822 | 2.035 | 3.155 | 0 |

* Pinning to one vCPU lowered the median by 40 % and the maximum by 30 %, with p95 and p99 nearly
  unchanged.
* Because vCPU 4 is not a fixed host core (mapping check above), this is **not** P-core affinity. A
  plausible reading, not proven here, is that keeping the thread on one vCPU keeps that vCPU warmer and its
  caches populated between ticks. It is one pinned vCPU, measured in one session.

## 6. CPU-D: lightweight optimizations (isolated variant `optimized.py`)

Two changes, both bit-identical by construction and checked on every call: the benchmark aborts if any
score bit or choice differs from production. The polar profile becomes one 2-D array expression with the
same per-element IEEE operations in the same order, and the forward pass uses preallocated buffers through
the same BLAS routine. Production calls run in the same trials as the reference.

| Decision total (phased) | Back to back p50 / p99 / max | 20 Hz p50 / p99 / max |
|---|---|---|
| CPU-A production code | 0.389 / 1.106 / 5.004 | 1.277 / 2.277 / 7.122 |
| CPU-D optimized | **0.201 / 0.553 / 4.361** | **0.924 / 1.475 / 2.036** |
| polar_profile alone, A → D | 0.240 → 0.055 (p50) | 0.747 → 0.327 (p50) |
| forward alone, A → D | 0.078 → 0.077 (p50) | 0.186 → 0.227 (p50) |

* **Every one of 144,037 optimized calls was bit-exact** (plus 2,601 in the §9 stress runs), with identical
  choices.
* Vectorizing the polar profile halves the decision. Preallocating the MLP buffers gives no measurable
  gain: the forward pass is about 77 µs either way and is dominated by the BLAS calls.
* The rare multi-millisecond stall remains in back-to-back mode (max 4.36 ms), landing again in arbitrary
  phases (`select` max 1.16 ms, median < 1 µs). Optimization shortens the exposure window; it cannot
  remove an external stall.
* Other CPU-D items:
  * **OpenBLAS thread count:** not applicable, because the controller uses the reference BLAS.
  * **A different BLAS** (OpenBLAS, MKL): changes summation order and so breaks bit-exact replay (§8).
  * **Feature recomputation:** none found. Features are built once per decision; the shadow P4
    selection is outside the timed region.

## 7. GPU inference benchmark (experimental backend, isolated)

Backend: torch 2.9.1+cu128 on the RTX 4060 Laptop GPU, in the pre-existing conda env `thesis` (Python
3.13.0, NumPy 2.3.5). The controller environment was not touched. Weights are the frozen checkpoint
loaded through `FrozenPolicy.load`. Batch = the 26 candidate slots of one decision. TF32 matmul was off.
Every timed GPU region is bracketed by `torch.cuda.synchronize()`, and the device-to-host copy is
synchronous, so no time is asynchronous submission only.

| Workload (ms) | p50 | p95 | p99 | Max | > 10 ms | Calls |
|---|---|---|---|---|---|---|
| GPU-A forward only, f64, back to back | 0.177 | 0.445 | 0.963 | 3.252 | 0 | 60,677 |
| GPU-A forward only, f64, 20 Hz | 2.534 | 4.652 | 5.609 | 9.425 | 0 | 1,661 |
| GPU-A forward only, f32, 20 Hz | 1.865 | 2.443 | 3.384 | 4.193 | 0 | 1,121 |
| GPU-B end to end, f64, back to back | 0.661 | 1.456 | 2.046 | **17.353** | 1 | 60,677 |
| GPU-B end to end, f64, 20 Hz | 5.363 | 8.340 | **13.035** | **14.804** | **28** | 1,661 |
| GPU-B end to end, f32, 20 Hz | 4.784 | 6.941 | 8.087 | **20.645** | 2 | 1,121 |
| *CPU-A forward only (reference BLAS), 20 Hz* | *0.186* | | *0.442* | *0.563* | *0* | *3,499* |
| *CPU-A full decision, 20 Hz* | *1.272* | *1.922* | *2.312* | *4.478* | *0* | *3,499* |

GPU-B parts at 20 Hz, f64 (p50 / p99 / max):

| Part | Time (ms) |
|---|---|
| CPU feature build | 1.181 / 2.645 / 5.153 |
| host-to-device copy | 1.416 / 4.681 / 11.952 |
| forward + synchronize | 2.091 / 6.256 / 10.039 |
| device-to-host copy | 0.333 / 1.855 / 4.756 |
| selection | 0.170 / 0.375 / 2.796 |

Cold forward (the first synchronized call after the weights are uploaded): 987 ms, 160 ms and 57 ms in
the three runs. Context creation and weight upload: 0.44-0.58 s.

Measured facts:

* **The MLP is too small for a GPU.** It is 26 x 46 → 64 → 64 → 1, about 0.2 MFLOP. Back to back, the
  GPU forward (0.177 ms) is no faster than the CPU forward (0.078 ms back to back, through the reference
  BLAS). At 20 Hz the GPU is 13x slower (2.53 vs 0.19 ms): an idle GPU is slow to respond to a single
  small request, in addition to kernel-launch and synchronization overhead.
* **End to end, the GPU adds two copies and a synchronization to a decision that is ~80 % CPU feature
  construction.** At 20 Hz, GPU-B is 4.2x slower at the median than the CPU decision. It exceeded 10 ms
  in 28 of 1,661 calls (f64) and reached 20.6 ms (f32), against no CPU calls over 10 ms. Individual copies
  stalled for up to 12-17 ms.
* No configuration measured improves worst-case latency, p99, timing stability or end-to-end latency
  with the GPU. CPU use per decision is not lower either, because feature construction stays on the CPU
  and synchronization spins.

## 8. Numerical and decision equivalence

**Reference:** the policy ledger of the failed Gate C S2 flight, 833 invoked records, each with the
exact flight feature tensor, mask, recorded scores and choice.

| Backend | Scores bit-exact | Max abs diff | Max rel diff | Choice equal | New near-ties (≤ 1e-9) |
|---|---|---|---|---|---|
| Frozen `FrozenPolicy.scores`, controller interpreter (reference BLAS) | **833 / 833** | 0 | 0 | 833 / 833 | 0 |
| CPU-D `optimized.Forward`, controller interpreter | **833 / 833** | 0 | 0 | 833 / 833 | 0 |
| NumPy 2.3.5 + OpenBLAS 0.3.30, CPU (env `thesis`) | 0 / 833 | 1.2e-14 | 5.3e-12 | 833 / 833 | 0 |
| torch CUDA float64 | 0 / 833 | 1.1e-14 | 5.9e-12 | 833 / 833 | 0 |
| torch CUDA float32 | 0 / 833 | 6.2e-6 | 5.3e-3 | 833 / 833 | 0 |

* The spike ticks 491 and 337 behave like every other tick:
  * the frozen CPU path is bit-exact on both;
  * GPU f64 differs by up to 7.1e-15 and keeps the recorded choice (slot 13 and slot 18);
  * the forward pass on tick 491 takes 0.085 ms median over 2,000 repetitions.

  Nothing in the inputs of tick 491 makes it expensive.
* **Decisions on the 120 reconstructed ticks**, with the production `PolicyRanker` and the decision
  replica `plan_with_ranker` driven by the GPU policy versus the frozen CPU policy:
  * f64: 120 / 120 equal choices and 120 / 120 equal decisions (action, heading, length, end, reason,
    best candidate, valid and progress counts);
  * f32: likewise 120 / 120 and 120 / 120;
  * the feature tensor and mask are identical in both paths, so the masks match.
* **Shield and supervisor.** The safety shield and supervisor are deterministic functions of the decision
  and the same inputs. With identical decisions on every tick, their outputs and transitions are identical
  by construction. No backend produced a different decision, so no divergence was observed to trace.
* **Contract compatibility.**
  * Contract §8 and Gate A item 4 require scores that are bit-identical to the frozen runtime under the
    controller interpreter.
  * **Every GPU variant, and any CPU BLAS other than the controller's reference BLAS, fails that
    requirement**, even with 833 / 833 equal decisions.
  * Adopting one would need a proposed contract revision that replaces bit-exact replay with a
    decision-equivalence criterion, plus explicit operator approval. This report does not propose or
    assume one.
  * CPU-D is compatible: bit-exact on all 833 tensors.

## 9. Tail-latency stress tests

Each stress run is 20 Hz, one trial of 90 s, with the load generators bounded to 100 s and verified gone
afterwards. CPU runs carry the production path (CPU-A) and the optimized phased path (CPU-D) in the same
trial. "Timeouts" are production calls over the runtime's 20 ms `failure_wall_s`, which in flight would
be a policy failure and a HOLD.

| Condition (20 Hz) | Calls | CPU-A p50 / p95 / p99 / max | > 10 ms | Timeouts | CPU-D p50 / p99 / max |
|---|---|---|---|---|---|
| Idle (§3, 3 trials) | 3,499 | 1.272 / 1.922 / 2.312 / 4.478 | 0 | 0 | 0.924 / 1.475 / 2.036 (§6) |
| Guest oversubscription: 30 busy processes on 24 vCPUs | 852 | 1.495 / 2.642 / **8.016 / 21.636** | **6** | **1** | 0.890 / 2.804 / **12.467** |
| Host contention: 24 busy Windows processes | 877 | 1.348 / 1.774 / 2.486 / 2.962 | 0 | 0 | 0.845 / 1.598 / 1.975 |
| Host contention + affinity vCPU 4 | 872 | 1.433 / 2.384 / 2.786 / 5.551 | 0 | 0 | 0.928 / 1.711 / 2.187 |
| GPU contention, GPU-B f64 end to end | 576 | 2.674 / 3.514 / 5.131 / 11.444 | 1 | n/a | (GPU-A forward 0.833 / 1.532 / 5.107) |

* **Guest CPU oversubscription reproduces calls of the in-flight size and larger.** Six production calls
  exceeded 10 ms; one took 21.6 ms and would have tripped the runtime timeout.
  * Each of the five slowest calls had an involuntary context switch and no change of Linux CPU.
  * On each, wall time was far above thread CPU time (21.6 ms wall vs 12.9 ms CPU; 19.7 vs 5.7).
  * That is the guest scheduler taking the CPU away mid-call. Its signature differs from the idle-host
    stalls of §3, where CPU time ≥ wall time and there was no context switch.
  * The optimized path still reached 12.5 ms: a shorter computation is hit less often, not never.
* **Host CPU contention did not worsen anything.** It made timing slightly steadier (max 2.96 ms vs 4.48 ms
  idle).
* **Affinity is not a robust improvement.** The 40 % median gain of the idle affinity run (§5) did not
  reappear under host contention (p50 1.433 ms).
* **Under GPU contention, the GPU path was faster than with an idle GPU** (end-to-end p50 2.67 vs 5.36 ms),
  but still had an 11.4 ms call.

### Separating the causes

| Component | Evidence | Status |
|---|---|---|
| Algorithmic computation | Fixed work per tick (0.20-0.39 ms back to back); the spike ticks cost the same as other ticks (tick 491 forward 0.085 ms median); polar profile ~60 % of it | **Measured.** Cannot produce 10 ms |
| Memory allocation and copying | Preallocating MLP buffers changes nothing (77 vs 78 µs); RSS flat at 210 MB; GC ruled out earlier (plan §13.14) | **Measured.** Not a tail source on CPU |
| Runtime and library overhead | Reference BLAS forward 78 µs; validation, ranking and selection ~5 % | **Measured.** Small |
| GPU launch and synchronization | GPU forward alone 0.18 ms back to back, 2.5 ms at 20 Hz; host-to-device copies up to 12-17 ms | **Measured.** Large; worse than CPU |
| CPU frequency and power state | Same work 3.3x slower when paced; a busy host makes timing steadier; host in "Best power efficiency" mode | **Hypothesis, consistent with measurements.** Performance mode not tested |
| Affinity and core placement | vCPU numbers do not map to P/E-cores; pinning helped once, not under contention | **Measured inconclusive** |
| Guest (WSL) scheduling | Oversubscription gives 10-22 ms calls with involuntary switches and wall ≫ CPU | **Measured mechanism.** Whether the simulator's bursts oversubscribe the guest in flight is a **hypothesis** (load average 1.3-6.2 is a 1-minute mean that can hide bursts) |
| Host / hypervisor scheduling | Idle-host stalls of 3-9 ms charged as CPU with no guest switch; WSL reports no steal time | **Hypothesis.** CPU ≥ wall is consistent with it but is not proof |

## 10. Comparison

**Full decision, 20 Hz** (the gated quantity; ms):

| Backend | p50 | p95 | p99 | Max | Decision parity | Contract compatible |
|---|---|---|---|---|---|---|
| CPU NumPy baseline (production) | 1.272 | 1.922 | 2.312 | 4.478 | reference | yes |
| CPU performance mode | not measured (host setting not changed) | | | | | |
| CPU affinity, vCPU 4 (production code) | 0.761 | 1.822 | 2.035 | 3.155 | identical code | code yes; affinity is a runner change |
| CPU optimized (CPU-D, isolated) | 0.924 | 1.329 | 1.475 | 2.036 | 833 / 833 bit-exact scores; identical choices | yes (bit-exact), but a new candidate |
| GPU experimental, torch CUDA f64 | 5.363 | 8.340 | 13.035 | 14.804 | choices 833 / 833, decisions 120 / 120; scores 0 / 833 bit-exact | **no** |
| GPU experimental, torch CUDA f32 | 4.784 | 6.941 | 8.087 | 20.645 | choices 833 / 833, decisions 120 / 120; scores 0 / 833 (max diff 6.2e-6) | **no** |

**Back to back** (ms, p50 / p95 / p99 / max):

| Backend | Full decision | Forward only |
|---|---|---|
| CPU NumPy baseline | 0.391 / 0.736 / 1.097 / 5.242 | 0.078 / 0.142 / 0.222 / 2.487 |
| CPU optimized | 0.201 / 0.373 / 0.553 / 4.361 | 0.077 / 0.140 / 0.211 / 1.946 |
| GPU f64 | 0.661 / 1.456 / 2.046 / 17.353 | 0.177 / 0.445 / 0.963 / 3.252 |

**Forward only, 20 Hz** (ms, p50 / p95 / p99 / max):

| Backend | Forward only |
|---|---|
| CPU NumPy baseline | 0.186 / 0.359 / 0.442 / 0.563 |
| GPU f64 | 2.534 / 4.652 / 5.609 / 9.425 |
| GPU f32 | 1.865 / 2.443 / 3.384 / 4.193 |

**GPU against the criteria:**

* *Worst case:* worse (14.8-20.6 ms vs 4.5 ms).
* *p99:* worse (8-13 ms vs 2.3 ms).
* *Stability:* worse (copy stalls up to 17 ms).
* *CPU usage:* not lower, because feature construction stays on the CPU.
* *Reproducibility:* fails bit-exact replay.
* *End-to-end latency:* 4.2x slower at the median.

**The GPU gives no meaningful improvement on any criterion.**

## 11. Recommendation: Option C — retain the current CPU NumPy backend and investigate further

* **GPU (Option B) is rejected on measurements.** It is slower end to end, has worse tails, adds a
  dependency and a near-one-second cold start, and its scores are not bit-identical. Adopting it would
  also require a contract revision, which this report does not propose.
* **CPU-only optimization does not resolve the spike on its own (so not Option A).**
  * The bit-exact CPU-D variant halves the decision (0.39 → 0.20 ms back to back). At 20 Hz it cuts the
    tail (p99 2.28 → 1.48 ms, max 7.12 → 2.04 ms) with 833 / 833 bit-exact scores.
  * Under guest oversubscription it still reached 12.5 ms.
  * The measured tail source is time the policy thread does not get or runs slowly: guest preemption, and
    probably host power and scheduling behaviour. The computation is not the source, so making it faster
    narrows the exposure window without bounding it.
* **The 10 ms maximum is not proposed for weakening.** Nothing here supports doing that to obtain a pass.
* **No offline result here validates anything in SITL.** P5 remains stopped and not qualified (plan
  §13.16). Any production change needs a separately authorized candidate and gate sequence.

## 12. Proposed next steps (none started; each needs operator authorization)

1. **Host power mode (operator action, then offline).** The operator switches Windows to *Best
   performance* on AC. Then repeat §3, §6 and §9 (about 30 minutes, no simulator). If paced medians fall
   toward back-to-back values and idle stalls disappear, the host setting is a material factor.
2. **A loaded-simulator diagnostic run (separately authorized, not an acceptance gate).** A non-frozen
   diagnostic copy of the controller would record, for every policy call, thread CPU time, involuntary
   context switches and the Linux CPU, alongside the guest runnable-task count
   (`/proc/loadavg` running field, sampled at 10 Hz). This would show whether the in-flight 10 ms class of
   spike comes with guest preemption, as §9 reproduces, or not.
3. **If P5 is ever reopened with a new candidate:** the bit-exact CPU-D polar-profile vectorization is
   the only inference change these measurements support. Any guest-contention mitigation (for example
   bounding simulator worker threads, or giving the controller a dedicated vCPU budget) would be a runner
   change to the P5 stack with its own evidence, not an inference-backend change.
4. **Not recommended:** GPU inference, a different BLAS, or float32. Each breaks bit-exact replay and none
   improved the tail.

## Reproduce

Inside WSL, from `diagnostics/p5_inference_benchmark` (each command runs on its own):

```bash
./run_controller_python.sh cpu_bench.py --label A-baseline-20hz --pause 0.05 --duration 120 --trials 3
./run_controller_python.sh cpu_bench.py --label D-optimized-20hz --variant optimized --pause 0.05 --duration 120 --trials 3
./run_controller_python.sh pe_map.py 4 0.25
./run_controller_python.sh equivalence_cpu.py
./run_gpu_python.sh gpu_bench.py --label f64-20hz --precision f64 --pause 0.05 --duration 60 --trials 3
python3 summarize.py
```

Guest oversubscription: start `python3 load.py cpu 30 100 &`, then run `cpu_bench.py` with
`--label S-guest-oversub-20hz --variant optimized --duration 90 --trials 1`.
