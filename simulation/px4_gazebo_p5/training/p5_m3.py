"""P5 M3 offline training pipeline (plan §4; separately authorized by the operator on 2026-10-31).

Stages: layouts -> map scans -> DAgger rounds 0..2 (collect with the shield on, train on the
aggregate) -> held-out readiness -> export. Offline only: no simulator, ROS or PX4. Every stage is
deterministic and writes under WORKDIR; a stage whose outputs exist is reused, never silently
recomputed. `export` writes the content-addressed artifact and its reports into models/ only if the
readiness report passes contract §4.2 for exactly these weights.

    python3 p5_m3.py run WORKDIR [--workers N]
    python3 p5_m3.py export WORKDIR
"""
import argparse
import hashlib
import json
import multiprocessing
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")  # fixed BLAS threading: reproducible float64 results

import numpy as np  # noqa: E402

import p5_paths  # noqa: E402
import p5_dataset  # noqa: E402
import p5_episode  # noqa: E402
import p5_layouts  # noqa: E402
import p5_oracle  # noqa: E402
import p5_readiness  # noqa: E402
import p5_train  # noqa: E402
from gwm_policy_control import contracts, policy_runtime  # noqa: E402

ROUNDS = 3
TRAIN_BLAS_THREADS = "8"
SCHEMA = contracts.load_schema(p5_paths.P5 / "configs/p5_model_schema.json")
SCHEMA_SHA = contracts.schema_sha256(SCHEMA)
MODELS = p5_paths.P5 / "models"


def log(message):
    print(time.strftime("%H:%M:%S"), message, flush=True)


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes((json.dumps(value, sort_keys=True, indent=1, allow_nan=False) + "\n").encode())
    return sha256_file(path)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def save_arrays(directory, arrays):
    directory.mkdir(parents=True, exist_ok=True)
    for name, value in arrays.items():
        np.save(directory / (name + ".npy"), value, allow_pickle=False)


def load_arrays(directory, names):
    return {name: np.load(directory / (name + ".npy"), allow_pickle=False) for name in names}


def provenance(allow_dirty):
    def git(*args):
        return subprocess.run(["git", "-C", str(p5_paths.REPO), "-c", "core.autocrlf=true", *args],
                              capture_output=True, text=True, check=True).stdout.strip()
    dirty = git("status", "--porcelain", "--untracked-files=all", "--", "simulation/px4_gazebo_p5",
                ":(exclude)simulation/px4_gazebo_p5/models")
    if dirty and not allow_dirty:
        raise SystemExit("refusing to train from uncommitted P5 sources:\n" + dirty)
    return {"code_commit": git("rev-parse", "HEAD"), "p5_tree_clean": not dirty, "python": platform.python_version(),
            "numpy": np.__version__, "openblas_num_threads": os.environ.get("OPENBLAS_NUM_THREADS"),
            "platform": platform.platform()}


def stage_layouts(work, pool):
    path = work / "layouts.json"
    if not path.exists():
        splits = {name: p5_layouts.split(name, mapper=lambda f, it: pool.imap(f, it, chunksize=8))
                  for name in p5_layouts.SPLITS}
        write_json(path, splits)
        log("layouts: " + ", ".join("%s %d (tried %d, rejected %s)" % (n, len(s["layouts"]), s["seeds_tried"], s["rejected"])
                                    for n, s in splits.items()))
    return read_json(path)


def _scan(layout):
    return p5_episode.map_scan(layout["boxes"])


def stage_scans(work, pool, splits):
    scans = {}
    for name, split in splits.items():
        path = work / "scans" / (name + ".npy")
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            np.save(path, np.stack(list(pool.imap(_scan, split["layouts"], chunksize=4))), allow_pickle=False)
            log("scans: %s done" % name)
        scans[name] = np.load(path, allow_pickle=False)
    return scans


def stage_collect(work, pool, splits, scans, round_index, weights):
    out = {}
    for name in ("train", "validation"):
        directory = work / "data" / ("round%d" % round_index) / name
        if not (directory / "episodes.json").exists():
            jobs = [(i, layout, scans[name][i], weights, round_index, SCHEMA, p5_dataset.SAMPLE_STRIDE)
                    for i, layout in enumerate(splits[name]["layouts"])]
            parts, episodes = [], []
            for arrays, episode in pool.imap(p5_dataset.collect_episode, jobs, chunksize=1):
                parts.append(arrays)
                episodes.append(episode)
            save_arrays(directory, p5_dataset.concatenate(parts))
            write_json(directory / "episodes.json", episodes)
            reached = sum(e["outcome"] == "goal_reached" for e in episodes)
            log("round %d %s: %d episodes, %d goal, %d samples, %d policy failures" % (
                round_index, name, len(episodes), reached, sum(e["samples"] for e in episodes),
                sum(e["policy_failures"] for e in episodes)))
        out[name] = load_arrays(directory, p5_dataset.ARRAYS)
    return out


PARITY_TOLERANCE = 1e-9


def stage_train(work, round_index, train_python):
    """Training runs in `train_python` (a NumPy with an optimized BLAS, threads pinned); the weights
    are then checked here, in the runtime's NumPy, against the recorded training-side scores."""
    directory = work / "models" / ("round%d" % round_index)
    if not (directory / "history.json").exists():
        log("round %d training in %s" % (round_index, train_python))
        subprocess.run([train_python, str(Path(p5_train.__file__).resolve()), str(work), str(round_index)],
                       check=True, env=dict(os.environ, OPENBLAS_NUM_THREADS=TRAIN_BLAS_THREADS))
    weights = load_arrays(directory, contracts.LAYER_SHAPES)
    if not (directory / "parity.json").exists():
        policy = p5_dataset.intermediate_policy(weights, "parity")
        x, expected = np.load(directory / "parity_x.npy"), np.load(directory / "parity_scores.npy")
        worst = max(float(np.abs(policy.scores(x[i]) - expected[i]).max()) for i in range(x.shape[0]))
        write_json(directory / "parity.json", {"states": int(x.shape[0]), "max_abs_difference": worst,
                                               "tolerance": PARITY_TOLERANCE, "runtime_numpy": np.__version__,
                                               "training_environment": read_json(directory / "environment.json")})
        if worst > PARITY_TOLERANCE:
            raise SystemExit("training/runtime forward parity failed: %.3g" % worst)
        log("round %d parity: max |runtime - training| = %.3g over %d states" % (round_index, worst, x.shape[0]))
    return weights


def _readiness_job(args):
    return p5_readiness.evaluate_layout(args)


def stage_readiness(work, pool, splits, scans, weights):
    path = work / "readiness.json"
    if not path.exists():
        weights_sha = p5_train.weights_sha256(weights)
        layouts = splits["held_out"]["layouts"]
        jobs = [(layout, scans["held_out"][i], weights, weights_sha, SCHEMA, SCHEMA_SHA) for i, layout in enumerate(layouts)]
        episodes = list(pool.imap(_readiness_job, jobs, chunksize=1))
        summary = p5_readiness.report(episodes, p5_readiness.bar())
        write_json(path, {"contract": "p5-acceptance-v1 revision 1 §4.2", "weights_sha256": weights_sha,
                          "schema_sha256": SCHEMA_SHA, "run": read_json(work / "run.json"),
                          "held_out": {k: splits["held_out"][k] for k in ("seed_base", "seeds_tried", "rejected")},
                          "layouts": [{"id": l["id"], "seed": l["seed"], "family": l["family"], "boxes": l["boxes"]}
                                      for l in layouts],
                          "summary": summary, "episodes": episodes,
                          "note": "offline synthetic closed loop; not acceptance evidence"})
        log("readiness: ready=%s p4 %.1f%% policy %.1f%% violations %d disagreement %.3f" % (
            summary["ready"], summary["p4"]["goal_success_pct"], summary["policy"]["goal_success_pct"],
            summary["checks"]["shield_violations"]["value"], summary["policy_decisions"]["disagreement_rate"] or 0.0))
    return read_json(path)


def run(work, workers, allow_dirty, train_python):
    work.mkdir(parents=True, exist_ok=True)
    if not (work / "run.json").exists():
        write_json(work / "run.json", {**provenance(allow_dirty), "workers": workers, "rounds": ROUNDS,
                                       "train_python": train_python, "train_blas_threads": TRAIN_BLAS_THREADS,
                                       "sample_stride": p5_dataset.SAMPLE_STRIDE, "hyper": p5_train.HYPER,
                                       "splits": p5_layouts.SPLITS})
    log("run: " + json.dumps(read_json(work / "run.json")))
    with multiprocessing.get_context("fork").Pool(workers) as pool:
        splits = stage_layouts(work, pool)
        scans = stage_scans(work, pool, splits)
        weights = None
        for r in range(ROUNDS):
            stage_collect(work, pool, splits, scans, r, weights)
            weights = stage_train(work, r, train_python)
        stage_readiness(work, pool, splits, scans, weights)
    write_json(work / "dataset_manifest.json", dataset_manifest(work))


def dataset_manifest(work):
    files = sorted(p for p in work.rglob("*") if p.is_file() and (p.relative_to(work).parts[0] in ("data", "scans")
                                                                    or p.name == "layouts.json"))
    splits = read_json(work / "layouts.json")
    return {"contract": "p5-acceptance-v1 revision 1 §4.1", "files": {str(p.relative_to(work).as_posix()): sha256_file(p) for p in files},
            "splits": {n: {k: s[k] for k in ("seed_base", "seeds_tried", "rejected")} | {"count": len(s["layouts"])}
                       for n, s in splits.items()},
            "sample_stride": p5_dataset.SAMPLE_STRIDE, "rounds": ROUNDS,
            "generator": "simulation/px4_gazebo_p5/training (p5_layouts, p5_episode, p5_dataset, p5_oracle)",
            "note": "the dataset is regenerated from the generator and seeds; only this manifest is kept"}


def export(work):
    run_info, readiness = read_json(work / "run.json"), read_json(work / "readiness.json")
    weights = load_arrays(work / "models" / ("round%d" % (ROUNDS - 1)), contracts.LAYER_SHAPES)
    weights_sha = p5_train.weights_sha256(weights)
    if not readiness["summary"]["ready"] or readiness["weights_sha256"] != weights_sha:
        raise SystemExit("export refused: the readiness report does not pass for these weights")
    if not run_info["p5_tree_clean"]:
        raise SystemExit("export refused: the run was not made from committed sources")
    histories = {r: read_json(work / "models" / ("round%d" % r) / "history.json") for r in range(ROUNDS)}
    parity = {r: read_json(work / "models" / ("round%d" % r) / "parity.json") for r in range(ROUNDS)}
    collections = {r: {n: read_json(work / "data" / ("round%d" % r) / n / "episodes.json") for n in ("train", "validation")}
                   for r in range(ROUNDS)}
    training_report = {
        "contract": "p5-acceptance-v1 revision 1 §4.1", "run": run_info, "weights_sha256": weights_sha,
        "oracle": {"lattice_m": p5_oracle.LATTICE_M, "band_multiplier": p5_oracle.BAND_MULTIPLIER,
                   "clearance_target_m": p5_oracle.CLEARANCE_TARGET_M, "lambda": p5_oracle.LAMBDA, "tau_m": p5_oracle.TAU_M,
                   "unreachable_cost_m": p5_oracle.UNREACHABLE_COST_M, "turn_term": "speed / yaw rate * max(0, |turn| - 25 deg)"},
        "rounds": [{"round": r, "driver": "p4_l3" if r == 0 else "policy_round_%d" % (r - 1),
                    "collection": {n: {"episodes": len(eps), "goal_reached": sum(e["outcome"] == "goal_reached" for e in eps),
                                       "samples": sum(e["samples"] for e in eps),
                                       "policy_failures": sum(e["policy_failures"] for e in eps),
                                       "outside_mask": sum(e["outside_mask"] for e in eps)}
                                   for n, eps in collections[r].items()},
                    "history": histories[r], "forward_parity": parity[r]} for r in range(ROUNDS)]}
    MODELS.mkdir(exist_ok=True)
    staging = {"dataset_manifest": write_json(MODELS / "_staging.dataset_manifest.json", read_json(work / "dataset_manifest.json")),
               "training_report": write_json(MODELS / "_staging.training_report.json", training_report),
               "readiness_report": write_json(MODELS / "_staging.readiness_report.json", readiness)}
    selected = next(h for h in histories[ROUNDS - 1] if h.get("selected"))
    s = readiness["summary"]
    metadata = {
        "architecture": contracts.ARCHITECTURE, "schema_sha256": SCHEMA_SHA, "trained": True, "weights_sha256": weights_sha,
        "training": {"method": "D1a privileged-oracle cost-to-go supervision with DAgger", "dagger_rounds": ROUNDS,
                     "hyper": run_info["hyper"], "sample_stride": run_info["sample_stride"], "selected_epoch": selected["epoch"],
                     "code_commit": run_info["code_commit"], "python": run_info["python"], "numpy": run_info["numpy"],
                     "openblas_num_threads": run_info["openblas_num_threads"]},
        "seeds": {"splits": run_info["splits"], "init_and_order": run_info["hyper"]["seed"]},
        "training_environment": parity[ROUNDS - 1]["training_environment"],
        "dataset_manifest_sha256": staging["dataset_manifest"], "training_report_sha256": staging["training_report"],
        "readiness_report_sha256": staging["readiness_report"],
        "readiness": {"ready": True, "held_out_layouts": s["p4"]["episodes"],
                      "goal_success_pct": {"p4": s["p4"]["goal_success_pct"], "policy": s["policy"]["goal_success_pct"]},
                      "shield_violations": s["checks"]["shield_violations"]["value"]},
        "provenance": "trained offline in the synthetic closed loop (plan M3, authorized 2026-10-31); "
                      "no acceptance layout was used for training, validation or selection"}
    tmp = MODELS / "_staging.npz"
    sha = policy_runtime.save_model(tmp, weights, metadata)
    final = MODELS / ("p5-ranker-%s.npz" % sha)
    tmp.replace(final)
    for kind in staging:
        (MODELS / ("_staging.%s.json" % kind)).replace(MODELS / ("p5-ranker-%s.%s.json" % (sha, kind)))
    policy_runtime.FrozenPolicy.load(final, sha, SCHEMA_SHA, require_trained=True)  # the runtime accepts it
    log("exported %s (%d bytes)" % (final.name, final.stat().st_size))
    return sha


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("run", "export"))
    parser.add_argument("work", type=Path)
    parser.add_argument("--workers", type=int, default=20)
    parser.add_argument("--allow-dirty", action="store_true", help="smoke runs only; export refuses such runs")
    parser.add_argument("--train-python", default=sys.executable,
                        help="interpreter for the training stage (a NumPy with an optimized BLAS)")
    args = parser.parse_args(argv)
    if args.command == "run":
        run(args.work, args.workers, args.allow_dirty, args.train_python)
    else:
        print(export(args.work))


if __name__ == "__main__":
    sys.exit(main())
