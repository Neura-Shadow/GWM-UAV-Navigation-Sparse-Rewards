"""Summarize preserved real runs and cross-check successful flights from ULog."""
import argparse
import hashlib
import json
from pathlib import Path

from verify_ulog import verify


def collect(root):
    results = []
    for path in sorted((root / "runs").glob("*-p1-*/summary.json")):
        source = json.loads(path.read_text())
        result = {key: source.get(key) for key in (
            "run_id", "boot", "flight_smoke", "hover", "landed_disarmed", "failure",
            "mode", "wall_duration_s", "observation_count", "config_sha256", "lock_sha256",
            "binary_sha256", "project_commit")}
        result["ulog_files"] = [{"relative_path": str(Path(item["path"]).relative_to(root)),
                                  "sha256": item["sha256"], "bytes": item["bytes"]}
                                 for item in source.get("ulog_files", [])]
        if source["flight_smoke"] == "passed":
            try:
                result["independent_ulog"] = verify(path)
            except Exception as exc:
                result["independent_ulog"] = {"status": "failed", "reason": str(exc)}
            path.with_name("ulog-verification.json").write_text(json.dumps(result["independent_ulog"], indent=2) + "\n")
        results.append(result)
    batches = []
    by_id = {result["run_id"]: result for result in results}
    for path in sorted((root / "runs").glob("*-repeat-*/summary.json")):
        source = json.loads(path.read_text())
        ids = [trial.get("run_id") for trial in source["trials"]]
        verified = sum(by_id.get(name, {}).get("independent_ulog", {}).get("status") == "passed" for name in ids)
        batches.append({"batch_id": path.parent.name, "status": source["status"],
                        "consecutive_passes": source["passed"], "required": source["required_consecutive"],
                        "independent_ulog_passes": verified, "run_ids": ids,
                        "failure": source["failure"], "input_identity": source.get("input_identity")})
    accepted = bool(batches and batches[-1]["status"] == "passed"
                    and batches[-1]["consecutive_passes"] == 20 and batches[-1]["independent_ulog_passes"] == 20)
    repeated_status = "passed" if accepted else ("failed" if batches and batches[-1]["status"] == "failed" else "incomplete")
    return {"evidence_root": str(root), "scope": "operator-directed software-only P0/P1",
            "repeated_acceptance": repeated_status,
            "ros2_autonomous_control": "not_implemented", "obstacle_avoidance": "not_implemented",
            "verifier_sha256": hashlib.sha256(Path(__file__).with_name("verify_ulog.py").read_bytes()).hexdigest(),
            "trials": results, "batches": batches}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = collect(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"trials": len(result["trials"]), "repeated_acceptance": result["repeated_acceptance"]}))
