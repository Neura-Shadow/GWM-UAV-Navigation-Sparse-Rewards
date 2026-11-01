"""Differential test: production P5 decision path (CPU-A) vs the same path with the vectorized polar profile
(CPU-D, `optimized.polar_profile`). Diagnostics only; the production module file is never modified — the
optimized function is swapped in for the module attribute inside this process only.

Everything below is a bit-for-bit or field-for-field comparison; no tolerance is used anywhere.

1. scores: every invoked record of every retained P5 flight ledger (exact flight tensors): frozen
   `FrozenPolicy.scores` and `optimized.Forward` vs the recorded scores.
2. flight decisions: every TRANSIT tick of every retained P4 and P5 flight that `reproduced_ticks` rebuilds
   (P4's recorded plan reproduced field for field): feature tensor, allowed mask, scores, selected slot,
   `plan_with_ranker` decision (action/heading/length/end/reason/best), and a sequential `PolicySupervisor`
   replay over the run's reproduced ticks (state, action, heading, end, frozen_yaw, reason, transitions).
3. ground decisions: every usable snapshot of every retained P4/P5 G2/G4/G6 and readiness run, in the
   Gate A item 4 frame (`snapshot_context`, state NORMAL): features, mask, decision.
4. polar-profile fuzz: random poses, headings (including exact multiples of the heading step and the axes),
   start points on cell boundaries and outside the grid, random traversed masks, on real recorded grids.
5. identity and isolation: checkpoint SHA-256 before and after; an audit hook proves the optimized
   function opens no file (no ground truth, no run data) while it runs.
Usage (controller interpreter): differential.py [fuzz_cases]
"""
import hashlib
import json
import math
import sys
import time

import numpy as np

import common
import optimized

FUZZ = int(sys.argv[1]) if len(sys.argv) > 1 else 200000


def main():
    from gwm_obstacle_avoidance import local_map, planner
    from gwm_policy_control import decision, observation_adapter
    from gwm_policy_control.candidate_ranker import PolicyRanker
    from gwm_policy_control.safety_shield import PolicySupervisor
    import p5_policy_eval

    sha_before = hashlib.sha256(common.MODEL.read_bytes()).hexdigest()
    policy, schema = common.load_policy()
    production_polar = observation_adapter.polar_profile
    fast_forward = optimized.Forward(policy)
    report = {"model_sha256_before": sha_before}

    def use(variant):
        observation_adapter.polar_profile = production_polar if variant == "A" else optimized.polar_profile

    def canon(value):
        """JSON-stable form with exact float bits (repr round-trips float64)."""
        if isinstance(value, np.ndarray):
            return [canon(v) for v in value.tolist()]
        if isinstance(value, dict):
            return {k: canon(v) for k, v in value.items() if k != "inference_s"}
        if isinstance(value, (list, tuple)):
            return [canon(v) for v in value]
        if isinstance(value, (float, np.floating)):
            return repr(float(value))
        if isinstance(value, (np.integer, np.bool_)):
            return value.item()
        return value

    # ---- 1. scores on exact flight tensors --------------------------------------------------------------
    flights = sorted(p.name for p in common.RUNS.glob("2026103*-p5-flight-S*") if (p / "p5/policy-ledger.jsonl").exists())
    s = {"records": 0, "frozen_bit_exact": 0, "optimized_forward_bit_exact": 0, "recorded_choice_equal": 0}
    for run in flights:
        for tick, (F, allowed, recorded, choice) in common.ledger_ticks(None, run).items():
            scores = policy.scores(F)
            s["records"] += 1
            s["frozen_bit_exact"] += bool(np.array_equal(scores, recorded))
            s["optimized_forward_bit_exact"] += bool(np.array_equal(fast_forward(policy, F), recorded))
            s["recorded_choice_equal"] += int(np.argmax(np.where(allowed, scores, -np.inf))) == choice
    report["scores_on_flight_tensors"] = {"runs": flights, **s}
    print("1", json.dumps(report["scores_on_flight_tensors"]), flush=True)

    # ---- 2. flight decisions and sequential supervisor replay -------------------------------------------
    flight_runs = sorted(p for p in common.RUNS.glob("2026103*-p[45]-flight-S*") if (p / "controller-result.json").exists())
    f = {"runs": 0, "ticks": 0, "features_equal": 0, "mask_equal": 0, "scores_equal": 0, "choice_equal": 0,
         "decision_equal": 0, "supervisor_steps": 0, "supervisor_equal": 0, "transitions_equal_runs": 0, "mismatches": []}
    for run in flight_runs:
        ticks = list(p5_policy_eval.reproduced_ticks(run))
        if not ticks:
            continue
        f["runs"] += 1
        outs = {}
        for variant in ("A", "B"):
            use("A" if variant == "A" else "D")
            config = ticks[0][6]
            ranker = PolicyRanker(policy, schema, float("inf"))
            sup = PolicySupervisor(config, ticks[0][1]["sim_s"] - 10.0, ranker, 3)
            sup.state = ticks[0][1]["state"]
            seq, per_tick = [], []
            for _, event, row, grid, traversed, ctx, config in ticks:
                settings, max_age = config["planner"], config["perception"]["snapshot_max_age_sim_s"]
                rows = planner.evaluate_candidates(grid, traversed, ctx, settings, max_age)
                progress = [r for r in rows if r["valid"] and observation_adapter.progress_qualified(r, ctx, settings)[0]]
                F, allowed = observation_adapter.build(rows, ctx, settings, grid, traversed, event["state"], schema)
                _, record = ranker.choose(rows, progress, ctx, grid, traversed, event["state"], settings)
                d = decision.plan_with_ranker(grid, traversed, ctx, settings, config["envelope"], max_age, ranker, event["state"])
                identity = row.get("calibration_identity")
                o = sup.step({"sim": event["sim_s"], "snapshot": {"acquisition_sim_s": event["snapshot_acquisition_sim_s"],
                                                                 "calibration_identity": identity},
                              "grid": grid, "traversed": traversed,
                              "adapter_health": {"status": "fresh", "calibration_identity": identity, "sim_s": event["sim_s"]},
                              "consecutive_unusable": 0, "reset_sim": None, "vehicle": ctx.vehicle, "vehicle_yaw": ctx.vehicle_yaw,
                              "target": ctx.target, "goal": ctx.goal, "origin": ctx.origin, "geofence": config["geofence"]})
                per_tick.append((F, allowed, record["scores"], record["choice_slot"], canon(d)))
                seq.append(canon({k: o[k] for k in ("state", "action", "heading", "end", "length", "frozen_yaw", "reason", "policy")}))
            outs[variant] = (per_tick, seq, canon(sup.transitions))
        a, b = outs["A"], outs["B"]
        for (Fa, ma, sa, ca, da), (Fb, mb, sb, cb, db) in zip(a[0], b[0]):
            f["ticks"] += 1
            f["features_equal"] += bool(np.array_equal(Fa, Fb))
            f["mask_equal"] += bool(np.array_equal(ma, mb))
            f["scores_equal"] += bool(np.array_equal(sa, sb))
            f["choice_equal"] += ca == cb
            f["decision_equal"] += da == db
            if not (np.array_equal(Fa, Fb) and da == db):
                f["mismatches"].append(run.name)
        f["supervisor_steps"] += len(a[1])
        f["supervisor_equal"] += sum(x == y for x, y in zip(a[1], b[1]))
        f["transitions_equal_runs"] += a[2] == b[2]
        print("2", run.name, len(ticks), "ticks", flush=True)
    report["flight_decisions"] = f
    print("2", json.dumps({k: v for k, v in f.items() if k != "mismatches"}), "mismatches", f["mismatches"][:5], flush=True)

    # ---- 3. ground snapshots (Gate A item 4 frame) -------------------------------------------------------
    ground_runs = sorted(p for p in common.RUNS.glob("2026103*-p[45]-*")
                         if any(t in p.name for t in ("-G2-", "-G4-", "-G6-", "readiness")) and (p / "p4/local_map.jsonl").exists())
    g = {"runs": 0, "snapshots": 0, "features_equal": 0, "mask_equal": 0, "decision_equal": 0, "policy_invoked": 0}
    for run in ground_runs:
        config = json.loads((run / "p4_avoidance.yaml").read_text())
        settings, max_age = config["planner"], config["perception"]["snapshot_max_age_sim_s"]
        rows = [r for r in p5_policy_eval.jsonl(run / "p4/local_map.jsonl") if r.get("usable") and r.get("grid")]
        g["runs"] += 1
        for row in rows:
            grid, traversed, ctx = p5_policy_eval.snapshot_context(row, config)
            cands = planner.evaluate_candidates(grid, traversed, ctx, settings, max_age)
            res = {}
            for variant in ("A", "D"):
                use(variant)
                F, allowed = observation_adapter.build(cands, ctx, settings, grid, traversed, "NORMAL", schema)
                d = decision.plan_with_ranker(grid, traversed, ctx, settings, config["envelope"], max_age,
                                              PolicyRanker(policy, schema, float("inf")), "NORMAL")
                res[variant] = (F, allowed, canon(d))
            g["snapshots"] += 1
            g["features_equal"] += bool(np.array_equal(res["A"][0], res["D"][0]))
            g["mask_equal"] += bool(np.array_equal(res["A"][1], res["D"][1]))
            g["decision_equal"] += res["A"][2] == res["D"][2]
            g["policy_invoked"] += res["A"][2]["policy"] is not None
    report["ground_snapshots"] = g
    print("3", json.dumps(g), flush=True)

    # ---- 4. polar-profile fuzz on real grids -------------------------------------------------------------
    use("A")
    grids = []
    for run in flight_runs[:4]:
        for i, row in enumerate(p5_policy_eval.jsonl(run / "p4/local_map.jsonl")):
            if row.get("usable") and row.get("grid") and i % 97 == 0:
                grids.append(local_map.Grid.decode(row["grid"]))
    config = json.loads((flight_runs[0] / "p4_avoidance.yaml").read_text())
    settings, profile = config["planner"], schema["polar_profile"]
    rng = np.random.default_rng(20261101)
    step = math.radians(settings["heading_step_deg"])
    z = {"cases": 0, "equal": 0, "grids": len(grids), "start_outside_grid": 0, "first_mismatch": None}
    for k in range(FUZZ):
        grid = grids[k % len(grids)]
        mode = k % 4
        if mode == 0:    # anywhere around the grid, including well outside it
            x = rng.uniform(grid.x_min - 2.0, grid.x_min + grid.nx * grid.resolution + 2.0)
            y = rng.uniform(grid.y_min - 2.0, grid.y_min + grid.ny * grid.resolution + 2.0)
        else:            # exactly on cell boundaries
            x = grid.x_min + rng.integers(-3, grid.nx + 3) * grid.resolution
            y = grid.y_min + rng.integers(-3, grid.ny + 3) * grid.resolution
        heading = (rng.integers(-24, 25) * step) if mode in (1, 2) else rng.uniform(-math.pi, math.pi)
        if mode == 2:
            heading = rng.choice([0.0, math.pi / 2, -math.pi / 2, math.pi, -math.pi])
        traversed = rng.random((grid.nx, grid.ny)) < (0.0, 0.05, 0.3, 0.9)[k % 4]
        outside = not (grid.x_min <= x < grid.x_min + grid.nx * grid.resolution and grid.y_min <= y < grid.y_min + grid.ny * grid.resolution)
        a = production_polar(grid, traversed, (x, y), heading, settings, profile)
        b = optimized.polar_profile(grid, traversed, (x, y), heading, settings, profile)
        z["cases"] += 1
        z["start_outside_grid"] += outside
        if a.dtype == b.dtype and np.array_equal(a, b) and np.array_equal(np.signbit(a), np.signbit(b)):
            z["equal"] += 1
        elif z["first_mismatch"] is None:
            z["first_mismatch"] = {"case": k, "x": x, "y": y, "heading": heading, "a": a.tolist(), "b": b.tolist()}
    report["polar_fuzz"] = z
    print("4", json.dumps(z), flush=True)

    # ---- 5. identity and isolation -------------------------------------------------------------------------
    opened = []
    watching = [False]

    def audit(event, args):
        if watching[0] and event in ("open", "os.listdir", "os.scandir", "socket.connect", "subprocess.Popen"):
            opened.append((event, str(args[0])[:120]))
    sys.addaudithook(audit)
    grid = grids[0]
    watching[0] = True
    for k in range(2000):
        optimized.polar_profile(grid, np.zeros((grid.nx, grid.ny), dtype=bool), (grid.x_min + 1.0, grid.y_min + 1.0),
                                k * 0.01, settings, profile)
    watching[0] = False
    import ast
    tree = ast.parse((common.Path(optimized.__file__)).read_text())
    imports = sorted({n.names[0].name if isinstance(n, ast.Import) else n.module for n in ast.walk(tree)
                      if isinstance(n, (ast.Import, ast.ImportFrom))})
    report["isolation"] = {"file_or_socket_events_during_2000_calls": opened, "optimized_module_imports": imports,
                           "model_sha256_after": hashlib.sha256(common.MODEL.read_bytes()).hexdigest(),
                           "frozen_policy_sha256": policy.sha256}
    report["isolation"]["checkpoint_unchanged"] = report["isolation"]["model_sha256_after"] == sha_before == common.MODEL_SHA256
    print("5", json.dumps(report["isolation"]), flush=True)
    common.RESULTS.mkdir(exist_ok=True)
    (common.RESULTS / "differential.json").write_text(json.dumps(report, indent=1, default=str))


if __name__ == "__main__":
    t = time.time()
    main()
    print(f"elapsed {time.time() - t:.0f} s")
