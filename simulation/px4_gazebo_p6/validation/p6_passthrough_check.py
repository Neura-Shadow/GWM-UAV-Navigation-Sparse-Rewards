"""P6 M2 exit check (offline, read-only): the pass-through injectors change nothing on recorded inputs.

Input: one accepted P5 flight run directory. Each block feeds the run's own records through the
accepted code twice, without and with the P6 injectors installed in pass-through (row B-0), and
requires bit-identical results:

1. **policy:** every invoked policy-ledger feature tensor, scored by the frozen model; the scores must
   also equal the recorded scores, and the masked argmax the recorded choice;
2. **decisions:** every reproduced TRANSIT tick (`p5_policy_eval.reproduced_ticks`) through
   `PolicyRanker.choose`;
3. **state:** every recorded PX4 state receipt into `StateCache.update`;
4. **map:** every recorded local-map snapshot into `P5Mission.on_snapshot`;
5. **depth:** a block of recorded raw depth frames (integrity by their recorded SHA-256) through the
   depth injector: in pass-through the very object is returned. Active F1.07 and F1.06 on the same
   frames give exactly the declared reactions of the accepted decoder and pipeline.

Run under the controller interpreter (`/usr/bin/python3`, NumPy 1.26.4, `OPENBLAS_NUM_THREADS=1`).
Writes a JSON report; exits 1 on any difference. Nothing in the run directory is written.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zlib

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control", ROOT / "simulation/px4_gazebo/ros2_ws/src/gwm_sensor_adapter",
             ROOT / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance",
             ROOT / "simulation/px4_gazebo_p5/ros2_ws/src/gwm_policy_control", ROOT / "simulation/px4_gazebo_p5/validation",
             ROOT / "simulation/px4_gazebo/validation", ROOT / "simulation/px4_gazebo_p6"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import numpy as np  # noqa: E402

from injection import controller_shim, declaration, depth_faults  # noqa: E402

P5 = ROOT / "simulation/px4_gazebo_p5"
DEPTH_FRAMES = 300


def jsonl(path):
    with Path(path).open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def twice(function):
    """(plain, shimmed) results of `function()`, the second with the pass-through injectors installed."""
    plain = function()
    shim = controller_shim.Shim(declaration.build("B-0")).install()
    try:
        shimmed = function()
    finally:
        shim.uninstall()
    return plain, shimmed


def frozen_policy():
    from gwm_policy_control import contracts
    from gwm_policy_control.policy_runtime import FrozenPolicy
    model = next((P5 / "models").glob("p5-ranker-*.npz"))
    schema = contracts.load_schema(P5 / "configs/p5_model_schema.json")
    return FrozenPolicy.load(model, model.stem[len("p5-ranker-"):], contracts.schema_sha256(schema)), schema


def policy_block(run, policy):
    from gwm_policy_control.policy_ledger import decode_array
    records = [r for r in jsonl(run / "p5/policy-ledger.jsonl") if r.get("invoked") and r.get("features")]

    def score_all():
        return [[float(s) for s in policy.scores(decode_array(r["features"]))] for r in records]
    plain, shimmed = twice(score_all)
    recorded_equal = sum(s == r["scores"] for s, r in zip(shimmed, records))
    argmax_equal = 0
    for scores, record in zip(shimmed, records):
        allowed = np.array(record["allowed"], dtype=bool)
        masked = np.where(allowed, np.array(scores), -np.inf)
        argmax_equal += int(int(np.argmax(masked)) == record["choice_slot"])
    return {"records": len(records), "shim_identical": plain == shimmed, "recorded_scores_equal": recorded_equal,
            "recorded_argmax_equal": argmax_equal}


def decision_block(run, policy, schema):
    import p5_policy_eval
    from gwm_obstacle_avoidance import planner
    from gwm_policy_control.candidate_ranker import PolicyRanker
    from gwm_policy_control.observation_adapter import progress_qualified
    ticks = list(p5_policy_eval.reproduced_ticks(run))

    def choose_all():
        out = []
        counter = iter(range(10**9))
        ranker = PolicyRanker(policy, schema, clock=lambda: next(counter) * 1e-6)  # deterministic: identical timing
        for _, event, _row, grid, traversed, ctx, config in ticks:
            settings = config["planner"]
            rows = planner.evaluate_candidates(grid, traversed, ctx, settings, config["perception"]["snapshot_max_age_sim_s"])
            progress = [r for r in rows if r["valid"] and progress_qualified(r, ctx, settings)[0]]
            if not progress:
                continue
            best, record = ranker.choose(rows, progress, ctx, grid, traversed, event["state"], settings)
            out.append({"features": record["features"].tobytes().hex() if record["features"] is not None else None,
                        "scores": None if record["scores"] is None else [float(s) for s in record["scores"]],
                        "allowed": None if record["allowed"] is None else record["allowed"].tolist(),
                        "choice": record["choice_slot"], "failure": record["failure"], "p4_slot": record["p4_slot"]})
        return out
    plain, shimmed = twice(choose_all)
    return {"ticks": len(plain), "shim_identical": plain == shimmed, "failures": sum(r["failure"] is not None for r in plain)}


def state_block(run):
    from gwm_px4_control import contracts, timing
    config = contracts.load_config(run / "p2_control.yaml")
    receipts = [r for r in jsonl(run / "ros-events.jsonl") if r.get("event") == "received"
                and r.get("topic_key") != "vehicle_command_ack"]

    def replay():
        cache, out = timing.StateCache(config), []
        for r in receipts:
            try:
                out.append(cache.update(r["topic_key"], r["fields"], r["monotonic_s"]))
            except (ValueError, KeyError) as exc:
                out.append("error:" + str(exc))
        return out, json.dumps(cache.data, sort_keys=True), json.dumps(cache.stats, sort_keys=True)
    plain, shimmed = twice(replay)
    return {"receipts": len(receipts), "shim_identical": plain == shimmed}


def map_block(run, policy, schema):
    from gwm_obstacle_avoidance.contracts import load_config as load_p4
    from gwm_policy_control.contracts import load_policy_config
    from gwm_policy_control.mission import P5Mission
    from gwm_px4_control import contracts, timing
    p2, p4 = contracts.load_config(run / "p2_control.yaml"), load_p4(run / "p4_avoidance.yaml")
    p5 = load_policy_config(run / "p5_policy.yaml")
    snapshots = jsonl(run / "p4/local_map.jsonl")

    def replay():
        mission = P5Mission(p2, p4, p5, policy, schema, timing.StateCache(p2), 0.0, True)
        out = []
        for snapshot in snapshots:
            mission.on_snapshot(snapshot)
            grid = mission.grid.cells.tobytes() if mission.grid is not None else b""
            out.append(((mission.snapshot or {}).get("acquisition_sim_s"), mission.consecutive_unusable,
                        zlib.crc32(grid), zlib.crc32(mission.traversed.tobytes()) if mission.traversed is not None else None))
        return out
    plain, shimmed = twice(replay)
    return {"snapshots": len(snapshots), "usable": sum(1 for s in snapshots if s.get("usable")), "shim_identical": plain == shimmed}


def depth_block(run):
    from gwm_sensor_adapter import contracts, depth, pipeline
    config = contracts.load_config(run / "p3_sensors.yaml")
    index = [r for r in jsonl(run / "sensors/depth-index.jsonl") if r.get("accepted")][:DEPTH_FRAMES]
    frames = []
    with (run / "sensors/depth.bin").open("rb") as stream:
        for r in index:
            stream.seek(r["offset"])
            stored = stream.read(r["stored_bytes"])
            raw = zlib.decompress(stored) if r["compression"] == "zlib-1" else stored
            if hashlib.sha256(raw).hexdigest() != r["raw_sha256"]:
                raise ValueError("recorded depth frame fails its own SHA-256: receipt %d" % r["receipt_id"])
            frames.append({"width": r["width"], "height": r["height"], "encoding": r["encoding"], "is_bigendian": r["is_bigendian"],
                           "step": r["step"], "data": raw, "stamp_ns": r["acquisition_stamp_ns"], "frame_id": r["frame_id"],
                           "sim": r["receipt_sim_s"]})
    passing = depth_faults.DepthInjector(None, {}, "pass-through")
    identical = all(passing.image(f, f["sim"])[0] is f for f in frames)
    d = declaration.build("A-5.F1.07")
    dimension = depth_faults.DepthInjector(d["fault"], d["parameters"], "active")
    dimension.activate(frames[0]["sim"])
    reasons = {}
    for f in frames[:20]:
        for out in dimension.image(f, f["sim"]):
            try:
                depth.decode(out, config)
                key = "decoded"
            except depth.FrameRejected as exc:
                key = str(exc)
            reasons[key] = reasons.get(key, 0) + 1
    d = declaration.build("A-5.F1.06")
    stamps = depth_faults.DepthInjector(d["fault"], d["parameters"], "active")
    processor = pipeline.DepthProcessor(config, "p6-passthrough-check", require_state=False)
    for i, f in enumerate(frames):
        if i == 10:
            stamps.activate(f["sim"])
        for out in stamps.image(f, f["sim"]):
            processor.on_image(out, out["stamp_ns"], out["frame_id"], 1000.0 + i * 0.033, f["sim"])
    return {"frames": len(frames), "integrity_checked": len(frames), "pass_through_same_object": identical,
            "f1_07_reasons": reasons, "f1_06_rejections": processor.accounting.rejected}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    policy, schema = frozen_policy()
    report = {"contract": "p6-passthrough-check-v1", "run": args.run.name, "numpy": np.__version__,
              "policy": policy_block(args.run, policy), "decisions": decision_block(args.run, policy, schema),
              "state": state_block(args.run), "map": map_block(args.run, policy, schema), "depth": depth_block(args.run)}
    expected_f106 = {"duplicate_acquisition_stamp": 5, "acquisition_stamp_backwards": 5, "missing_acquisition_stamp": 5}
    report["passed"] = (all(report[k]["shim_identical"] for k in ("policy", "decisions", "state", "map"))
                        and report["policy"]["recorded_scores_equal"] == report["policy"]["records"]
                        and report["depth"]["pass_through_same_object"]
                        and report["depth"]["f1_07_reasons"] == {"dimension_mismatch": 10, "decoded": 10}
                        and report["depth"]["f1_06_rejections"] == expected_f106)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
