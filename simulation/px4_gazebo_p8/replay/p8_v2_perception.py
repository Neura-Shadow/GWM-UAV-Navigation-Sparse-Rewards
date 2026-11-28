"""P8 V2: offline perception replay on the protected raw depth (contract `p8-navigation-recovery-v1` §6 V2; report
plan H11). Read-only: nothing in any run directory is written.

The recovery layer decides from the recorded local maps. V2 checks that those maps are what the accepted perception
computes from the raw frames. For each of the 24 protected P8 inputs (the 18 P7-E failures, and the contrast
flights: slots 7 P5, 15 P5, 19 and 28):
* every usable snapshot is replayed in order through the frozen P6 `recorded.PerceptionReplay` (the accepted P4 scan
  extraction and grid integration on the recorded frame and pose), with each frame checked against its recorded
  SHA-256;
* the replayed grid must equal the recorded snapshot grid bit for bit, as the P6 offline suite requires;
* for each failure, the snapshot used at its final HOVER entry is named, and must be among the identical ones.

Usage: cd /tmp && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 p8_v2_perception.py OUTPUT.json   (exclusive create)
"""
import json
from pathlib import Path
import sys
import time

P8 = Path(__file__).resolve().parents[1]
REPO = P8.parents[1]
for _path in (REPO / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance", REPO / "simulation/px4_gazebo/ros2_ws/src/gwm_sensor_adapter",
              REPO / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control", REPO / "simulation/px4_gazebo_p6/validation",
              REPO / "simulation/px4_gazebo/validation"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import numpy as np  # noqa: E402
import recorded  # noqa: E402  (frozen P6 offline-suite reader, unchanged)
from gwm_obstacle_avoidance.local_map import Grid  # noqa: E402
from gwm_sensor_adapter import depth  # noqa: E402
from p3_provenance import digest, strict_json  # noqa: E402

CONTRACT = "p8-v2-perception-v1"
ROOT = Path.home() / "uav_autonomy"
P7E_SHA = "e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608"
RESULTS = {1: "1c8672ab5d3a10df831a78ee34ea11282983fb9c002d51ef728dcc9884faa441",
           2: "62c23933960e256d9fc0d60b3ede008f9b0872752a46bbd4377cdf0783b56064",
           3: "d4da3561bfac6158b4992731d7721de3c2082cd0e5003907586e0e937d1a84de"}
CONTRAST = {(7, "p5"), (15, "p5"), (19, "p4"), (19, "p5"), (28, "p4"), (28, "p5")}


def final_hover_snapshot(run):
    result = strict_json(run / "controller-result.json")
    if result["p4"].get("abort_reason") != "hover_timeout":
        return None
    t0 = [t for t in result["p4"]["supervisor_transitions"] if t["to"] == "HOVER"][-1]
    return t0.get("snapshot_acquisition_sim_s")


def replay_run(run):
    t0 = time.perf_counter()
    snaps = recorded.usable_snapshots(run)
    p3 = recorded.configs(run)["p3"]
    index = recorded.depth_index(run)
    replay = recorded.PerceptionReplay(run)
    identical, first_difference, missing = 0, None, []
    acquisitions = []
    present = [s for s in snaps if s["acquisition_stamp_ns"] in index]
    frames = recorded.iter_depth_frames(run, [s["acquisition_stamp_ns"] for s in present])
    for k, s in enumerate(snaps):
        if s["acquisition_stamp_ns"] not in index:
            # The recorder and perception are separate best-effort subscribers: this processed frame was not recorded.
            missing.append(s["acquisition_sim_s"])
        else:
            replay.step(s, depth.decode(next(frames), p3))
        if replay.grid is not None and np.array_equal(replay.grid.cells, Grid.decode(s["grid"]).cells):
            identical += 1
            acquisitions.append(round(s["acquisition_sim_s"], 6))
        elif first_difference is None:
            first_difference = {"index": k, "acquisition_sim_s": s["acquisition_sim_s"]}
    hover = final_hover_snapshot(run)
    return {"usable_snapshots": len(snaps), "identical": identical, "first_difference": first_difference,
            "frames_missing_from_the_recording": missing,
            "all_identical": identical == len(snaps) and len(snaps) > 0,
            "final_hover_snapshot_sim_s": hover,
            "final_hover_snapshot_identical": None if hover is None else round(hover, 6) in set(acquisitions),
            "depth_bin_bytes": (run / "sensors/depth.bin").stat().st_size,
            "depth_index_sha256": digest(run / "sensors/depth-index.jsonl"),
            "local_map_sha256": digest(run / "p4/local_map.jsonl"), "wall_s": time.perf_counter() - t0}


def main(argv):
    output = Path(argv[0])
    if output.exists():
        raise SystemExit("output exists")
    rows = []
    for block, sha in RESULTS.items():
        path = ROOT / "state" / ("p7e-engineering-30-block-%02d-%s.result.json" % (block, P7E_SHA))
        if digest(path) != sha:
            raise SystemExit("block %d result differs" % block)
        for f in strict_json(path)["flights"]:
            run = Path(f["run"])
            outcome = strict_json(run / "p7/flight-record.json")["outcome"]
            if outcome != "goal_success" or (f["slot"], f["controller"]) in CONTRAST:
                rows.append(dict(slot=f["slot"], controller=f["controller"], run=run.name, outcome=outcome,
                                 role="failure" if outcome != "goal_success" else "contrast", **replay_run(run)))
    rows.sort(key=lambda r: (r["slot"], r["controller"]))
    failures = [r for r in rows if r["role"] == "failure"]
    complete = [r for r in rows if not r["frames_missing_from_the_recording"]]
    summary = {"flights": len(rows), "failures": len(failures), "contrast": len(rows) - len(failures),
               "snapshots": sum(r["usable_snapshots"] for r in rows), "identical": sum(r["identical"] for r in rows),
               "flights_with_frames_missing_from_the_recording": [(r["slot"], r["controller"], len(r["frames_missing_from_the_recording"]))
                                                                  for r in rows if r["frames_missing_from_the_recording"]],
               "complete_recordings_all_identical": all(r["all_identical"] for r in complete),
               "all_flights_identical": all(r["all_identical"] for r in rows),
               "final_hover_snapshots_identical": all(r["final_hover_snapshot_identical"] for r in failures)}
    summary["passed"] = (summary["complete_recordings_all_identical"] and summary["final_hover_snapshots_identical"]
                         and len(rows) == 24)
    summary["rule"] = ("passed: every flight with a complete recording reproduces every snapshot bit for bit, and every "
                       "failure's final-HOVER snapshot reproduces; a frame missing from a recording is reported, not hidden")
    value = {"contract": CONTRACT, "read_only": True, "p7e_manifest_sha256": P7E_SHA,
             "code_sha256": {"p8_v2_perception.py": digest(Path(__file__)),
                             "recorded.py": digest(REPO / "simulation/px4_gazebo_p6/validation/recorded.py")},
             "summary": summary, "flights": rows}
    with open(output, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=1, sort_keys=True, default=float) + "\n")
    print(json.dumps(summary, indent=1))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
