"""Experimental GPU backend benchmark for the P5 ranker (diagnostics only; never part of the controller).

Runs in the existing WSL conda env `thesis` (torch + CUDA), not in the controller interpreter; nothing is
installed. Weights come from the frozen checkpoint through the production `FrozenPolicy.load`.

* Equivalence: every invoked record of the Gate C S2 policy ledger (exact flight feature tensor, mask and
  recorded scores, which the controller interpreter reproduces bit for bit) scored by torch CUDA float64 and
  float32 and by NumPy in this env, against the recorded scores: bit-exact count, max abs/rel difference,
  masked-argmax choice equality. On the reconstructed ticks, the production `PolicyRanker` and the decision
  replica `plan_with_ranker` are run with the GPU policy and with the frozen CPU policy and compared.
* GPU-A forward only: inputs already on the device; torch.cuda.synchronize() before and after each call.
  Cold calls (first after load) are reported apart from warm calls.
* GPU-B end to end: production CPU feature build, host-to-device copy, forward, synchronize, device-to-host
  copy, masked selection (`policy_choice`) and the final mask re-check, each timed.
Usage: gpu_bench.py --label NAME [--precision f64|f32] [--pause 0.05] [--duration 60] [--trials 3]
"""
import argparse
import json
import platform
import time

import numpy as np
import torch

import common


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--label", required=True)
    parser.add_argument("--precision", choices=("f64", "f32"), default="f64")
    parser.add_argument("--pause", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--skip-equivalence", action="store_true")
    args = parser.parse_args()

    from gwm_policy_control import decision, observation_adapter
    from gwm_policy_control.candidate_ranker import PolicyRanker, policy_choice
    from gwm_policy_control.contracts import FEATURES, SLOTS

    device = torch.device("cuda")
    t_load = time.perf_counter()
    torch.cuda.init()
    policy, schema = common.load_policy()
    dtypes = {"f64": torch.float64, "f32": torch.float32}

    def upload(dtype):
        return {k: torch.tensor(np.asarray(v), dtype=dtype, device=device) for k, v in policy._weights.items()}
    weights = {name: upload(dt) for name, dt in dtypes.items()}
    torch.cuda.synchronize()
    load_s = time.perf_counter() - t_load

    def forward(xt, w):
        h = torch.relu(xt @ w["W1"] + w["b1"])
        h = torch.relu(h @ w["W2"] + w["b2"])
        return (h @ w["W3"] + w["b3"])[:, 0]

    out = {"label": args.label, "precision": args.precision, "pause_s": args.pause, "duration_s": args.duration,
           "inventory": {"python": platform.python_version(), "numpy": np.__version__, "torch": torch.__version__,
                         "cuda_runtime": torch.version.cuda, "cudnn": torch.backends.cudnn.version(),
                         "device": torch.cuda.get_device_name(0), "capability": torch.cuda.get_device_capability(0),
                         "tf32_matmul": torch.backends.cuda.matmul.allow_tf32},
           "context_and_upload_s": load_s}

    # ---- cold calls: the first forwards after upload, each synchronized -------------------------------
    flight = common.ledger_ticks(None)
    first = next(iter(flight.values()))
    w = weights[args.precision]
    cold = []
    for _ in range(3):
        xt = torch.from_numpy(first[0]).to(device=device, dtype=dtypes[args.precision])
        torch.cuda.synchronize()
        t = time.perf_counter_ns()
        forward(xt, w)
        torch.cuda.synchronize()
        cold.append((time.perf_counter_ns() - t) / 1e6)
    out["cold_forward_ms"] = cold

    # ---- equivalence on the exact flight tensors -------------------------------------------------------
    if not args.skip_equivalence:
        def masked_choice(scores, allowed):
            best = scores[allowed].max()
            tied = [int(i) for i in np.flatnonzero(allowed) if scores[i] >= best - 1e-9]
            return tied
        eq = {}
        backends = {"torch_cuda_f64": lambda F: forward(torch.from_numpy(F).to(device), weights["f64"]).cpu().numpy(),
                    "torch_cuda_f32": lambda F: forward(torch.from_numpy(F).to(device, torch.float32), weights["f32"]).double().cpu().numpy(),
                    "numpy_this_env_cpu": lambda F: policy.scores(F)}
        for name, fn in backends.items():
            exact = same_choice = ties_changed = 0
            max_abs = max_rel = 0.0
            for tick, (F, allowed, recorded, choice) in flight.items():
                s = np.asarray(fn(F), dtype=np.float64)
                exact += bool(np.array_equal(s, recorded))
                diff = np.abs(s - recorded)
                max_abs = max(max_abs, float(diff.max()))
                max_rel = max(max_rel, float((diff / np.maximum(np.abs(recorded), 1e-300)).max()))
                tied = masked_choice(s, allowed)
                same_choice += len(tied) == 1 and tied[0] == choice
                ties_changed += len(tied) != 1
            eq[name] = {"records": len(flight), "scores_bit_exact": exact, "max_abs_diff": max_abs,
                        "max_rel_diff": max_rel, "choice_equal": same_choice, "new_near_ties": ties_changed}
        out["equivalence_flight_tensors"] = eq
        print(json.dumps(eq), flush=True)
        spikes = {}
        for tick in common.SPIKE_TICKS:  # the Gate C S2 tail ticks, exact flight tensors
            F, allowed, recorded, choice = flight[tick]
            g = backends["torch_cuda_f64"](F)
            spikes[tick] = {"recorded_choice": choice, "numpy_this_env_bit_exact": bool(np.array_equal(policy.scores(F), recorded)),
                            "gpu_f64_bit_exact": bool(np.array_equal(g, recorded)),
                            "gpu_f64_choice": masked_choice(g, allowed), "gpu_f64_max_abs_diff": float(np.abs(g - recorded).max())}
        out["spike_ticks"] = spikes
        print(json.dumps(spikes), flush=True)

        # decisions on the reconstructed ticks: PolicyRanker + plan_with_ranker, GPU policy vs frozen CPU policy
        class GpuPolicy:
            sha256 = policy.sha256

            def __init__(self, precision):
                self.w, self.dtype = weights[precision], dtypes[precision]

            def scores(self, features):
                x = np.asarray(features)
                if x.shape != (SLOTS, FEATURES) or x.dtype != np.float64 or not np.isfinite(x).all():
                    raise ValueError("input_schema")
                return forward(torch.from_numpy(x).to(device, self.dtype), self.w).double().cpu().numpy()
        ticks = common.reconstruct_ticks(120)
        decisions = {}
        for precision in ("f64", "f32"):
            gpu_ranker, cpu_ranker = PolicyRanker(GpuPolicy(precision), schema, 0.020), PolicyRanker(policy, schema, 0.020)
            same_choice = same_decision = features_equal = 0
            for tick, sim, state, rows, progress, ctx, grid, traversed, settings, recorded in ticks:
                config = common.CONFIGS[tick]
                _, rg = gpu_ranker.choose(rows, progress, ctx, grid, traversed, state, settings)
                _, rc = cpu_ranker.choose(rows, progress, ctx, grid, traversed, state, settings)
                same_choice += rg["choice_slot"] == rc["choice_slot"] and rg["failure"] is None
                features_equal += bool(np.array_equal(rg["features"], rc["features"]))
                args_plan = (grid, traversed, ctx, settings, config["envelope"], config["perception"]["snapshot_max_age_sim_s"])
                dg = decision.plan_with_ranker(*args_plan, gpu_ranker, state)
                dc = decision.plan_with_ranker(*args_plan, cpu_ranker, state)
                keys = ("action", "heading", "length", "end", "reason", "best", "candidates_valid", "candidates_progress")
                same_decision += all(dg[k] == dc[k] for k in keys)
            decisions[precision] = {"ticks": len(ticks), "choice_equal": same_choice, "decision_equal": same_decision,
                                    "feature_tensor_equal_cpu_vs_gpu_path": features_equal}
        out["equivalence_reconstructed_decisions"] = decisions
        print(json.dumps(decisions), flush=True)

    # ---- timing --------------------------------------------------------------------------------------------
    ticks = common.reconstruct_ticks(120)
    dtype = dtypes[args.precision]
    reference = {}
    for tick, sim, state, rows, progress, ctx, grid, traversed, settings, recorded in ticks:
        F, allowed = observation_adapter.build(rows, ctx, settings, grid, traversed, state, schema)
        reference[tick] = (F, allowed, int(np.argmax(np.where(allowed, policy.scores(F), -np.inf))))
    on_device = {tick: torch.from_numpy(v[0]).to(device, dtype) for tick, v in reference.items()}
    torch.cuda.synchronize()
    trials = []
    for trial in range(args.trials):
        fwd, e2e = [], []
        end, i = time.monotonic() + args.duration, 0
        while time.monotonic() < end:
            tick, sim, state, rows, progress, ctx, grid, traversed, settings, _ = ticks[(i // 2) % len(ticks)]
            if i % 2 == 0:  # GPU-A forward only
                xt = on_device[tick]
                torch.cuda.synchronize()
                t0 = time.perf_counter_ns()
                forward(xt, w)
                torch.cuda.synchronize()
                fwd.append((time.perf_counter_ns() - t0) / 1e6)
            else:  # GPU-B end to end
                t0 = time.perf_counter_ns()
                F, allowed = observation_adapter.build(rows, ctx, settings, grid, traversed, state, schema)
                t1 = time.perf_counter_ns()
                xt = torch.from_numpy(F).to(device, dtype)
                torch.cuda.synchronize()
                t2 = time.perf_counter_ns()
                yt = forward(xt, w)
                torch.cuda.synchronize()
                t3 = time.perf_counter_ns()
                scores = yt.double().cpu().numpy()
                t4 = time.perf_counter_ns()
                by_slot = {observation_adapter.slot_of(r, settings): r for r in rows}
                choice = policy_choice(scores, allowed, by_slot)
                if not allowed[choice]:
                    raise SystemExit("selection_outside_mask")
                t5 = time.perf_counter_ns()
                e2e.append({"tick": tick, "build_ms": (t1 - t0) / 1e6, "h2d_ms": (t2 - t1) / 1e6, "forward_sync_ms": (t3 - t2) / 1e6,
                            "d2h_ms": (t4 - t3) / 1e6, "select_ms": (t5 - t4) / 1e6, "total_ms": (t5 - t0) / 1e6,
                            "choice_equal_cpu": choice == reference[tick][2]})
            i += 1
            if args.pause > 0:
                time.sleep(args.pause)
        summary = {"trial": trial, "calls": i, "forward_only_ms": common.quantiles(fwd),
                   "end_to_end": {k: common.quantiles([r[k] for r in e2e]) for k in
                                  ("build_ms", "h2d_ms", "forward_sync_ms", "d2h_ms", "select_ms", "total_ms")},
                   "end_to_end_over_5ms": sum(r["total_ms"] > 5 for r in e2e),
                   "end_to_end_over_10ms": sum(r["total_ms"] > 10 for r in e2e),
                   "choice_mismatches_vs_cpu": sum(not r["choice_equal_cpu"] for r in e2e),
                   "slowest_end_to_end": sorted(e2e, key=lambda r: -r["total_ms"])[:5]}
        trials.append(summary)
        print(json.dumps({"label": args.label, "trial": trial,
                          "forward_only_ms": {k: round(v, 3) for k, v in summary["forward_only_ms"].items() if k != "n"},
                          "end_to_end_ms": {k: round(v, 3) for k, v in summary["end_to_end"]["total_ms"].items() if k != "n"},
                          "over_10ms": summary["end_to_end_over_10ms"], "mismatch": summary["choice_mismatches_vs_cpu"]}), flush=True)
    out["trials"] = trials
    common.RESULTS.mkdir(exist_ok=True)
    path = common.RESULTS / f"gpu-{args.label}.json"
    path.write_text(json.dumps(out, indent=1, default=str))
    print(path)


if __name__ == "__main__":
    main()
