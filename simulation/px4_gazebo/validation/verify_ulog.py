"""Read-only, independent ULog cross-check; requires the PX4 pyulog environment."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def verify(summary_path):
    import numpy as np
    from pyulog import ULog
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    from p1_contract import evaluate_hover

    summary = json.loads(summary_path.read_text())
    assert summary["flight_smoke"] == "passed", "No passed flight to cross-check"
    assert len(summary["ulog_files"]) == 1, "Expected one continuous flight ULog"
    artifact = summary["ulog_files"][0]
    path = Path(artifact["path"])
    assert hashlib.sha256(path.read_bytes()).hexdigest() == artifact["sha256"], "ULog digest mismatch"
    log = ULog(str(path))
    assert not log.dropouts, "ULog reports dropouts"
    position = log.get_dataset("vehicle_local_position").data
    status = log.get_dataset("vehicle_status").data
    land = log.get_dataset("vehicle_land_detected").data
    start = summary["hover"]["start_sim_s"] * 1e6
    end = summary["hover"]["end_sim_s"] * 1e6
    timestamps = position["timestamp"]
    first = max(0, int(np.searchsorted(timestamps, start, side="right")) - 1)
    last = min(len(timestamps) - 1, int(np.searchsorted(timestamps, end)))
    samples = [{"t": float(timestamps[i]) / 1e6,
                **{k: float(position[k][i]) for k in ("x", "y", "z")}}
               for i in range(first, last + 1)]
    hover = evaluate_hover(samples, summary["config"], summary["initial_ground_local_z"])
    assert hover["status"] == "passed", "Independent ULog hover did not pass"
    assert all(position[key][first:last + 1].all() for key in ("xy_valid", "z_valid")), "Invalid estimate"
    for dataset, field, expected in ((status, "arming_state", 2), (status, "nav_state", 4),
                                      (land, "landed", False)):
        begin = max(0, int(np.searchsorted(dataset["timestamp"], start, side="right")) - 1)
        stop = int(np.searchsorted(dataset["timestamp"], end, side="right"))
        assert np.all(dataset[field][begin:stop] == expected), "Invalid hover state: " + field
    armed = np.flatnonzero(status["arming_state"] == 2)
    assert len(armed) and status["arming_state"][-1] == 1, "Missing armed-to-disarmed transition"
    assert not status["failsafe"][armed[0]:].any(), "Failsafe after arming"
    assert not land["landed"].all() and bool(land["landed"][-1]), "Missing airborne-to-landed transition"
    assert status["timestamp"][-1] >= end and land["timestamp"][-1] >= end, "Incomplete final-state coverage"
    return {"run_id": summary["run_id"], "status": "passed", "ulog_sha256": artifact["sha256"],
            "ulog_bytes": path.stat().st_size, "ulog_dropout_count": len(log.dropouts),
            "hover": hover, "armed_observed": True, "airborne_observed": True,
            "final_landed": True, "final_disarmed": True, "failsafe_after_arming": False}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("summary", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = verify(args.summary)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
