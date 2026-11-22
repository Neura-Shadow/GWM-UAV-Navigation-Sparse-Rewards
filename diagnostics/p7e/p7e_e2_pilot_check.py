"""P7-E E2 read-only check: the P7-E report code and FC-3 on the retained P7 pilot flights (plan §5 test matrix).

Reads the 40 flights of the accepted pilot block of candidate 0799c37e. It writes nothing into any run:
* **FC-3:** the assessment, computed in memory from each flight's frozen record and summary;
* **H1-H6:** the report's hard-criterion functions on those flights;
* **H9:** the frozen classifier, re-run through its own wrapper on a symlink view of each run, in a temporary
  directory. The re-run must reproduce the recorded flight record exactly, and every file of the run must keep its
  size and mtime.

Usage (WSL, from /tmp, PYTHONDONTWRITEBYTECODE=1): /usr/bin/python3 p7e_e2_pilot_check.py OUTPUT.json (exclusive)
"""
import json
from pathlib import Path
import sys
import tempfile
import time

REPO = Path(__file__).resolve().parents[2]
P7E = REPO / "simulation/px4_gazebo_p7e"
for _path in (P7E / "analysis", P7E / "scripts", P7E / "validation", REPO / "simulation/px4_gazebo/validation"):
    sys.path.insert(0, str(_path))
import p7e_assess  # noqa: E402
import p7e_report as rep  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

ROOT = Path.home() / "uav_autonomy"
PILOT = ROOT / "state/p7-pilot-block-01-0799c37e800c06c278acbc45fdec90b84159ee9d41eb7aaf868cb65a0b950aa5.result.json"


def main(argv):
    output = Path(argv[1])
    result = strict_json(PILOT)
    flights, rows = [], []
    with tempfile.TemporaryDirectory(prefix="p7e-e2-pilot-") as work:
        for n, entry in enumerate(result["flights"]):
            run = Path(entry["run"])
            record_path = run / p7e_assess.RECORD
            record = strict_json(record_path) if digest(record_path) == entry["flight_record_sha256"] else None
            summary = strict_json(run / "summary.json")
            evaluation_path = run / rep.EVALUATIONS[entry["controller"]]
            assessment = p7e_assess.assess(run, record, summary) if record else None
            flight = {k: entry.get(k) for k in ("slot", "layout", "controller", "attempt")}
            flight.update(block=1, run=str(run), summary=summary, record=record, assessment=assessment,
                          evaluation=strict_json(evaluation_path) if evaluation_path.exists() else None)
            flights.append(flight)
            before = rep.files_state(run)
            started = time.monotonic()
            again, error = rep.reclassify(run, Path(work) / str(n))
            rows.append({"run": run.name, "controller": entry["controller"], "record_matches_ledger": record is not None,
                         "frozen_validity": (record or {}).get("validity"),
                         "fc3_validity": (assessment or {}).get("validity"), "fc3_applied": ((assessment or {}).get("fc3") or {}).get("applied"),
                         "fc3_stop_added": p7e_assess.FC3_STOP in ((assessment or {}).get("stop_reasons") or []),
                         "reclassified_identical": again is not None and again == record, "reclassify_error": error,
                         "reclassify_wall_s": round(time.monotonic() - started, 2),
                         "run_files_unchanged": rep.files_state(run) == before})
    lim = rep.limits(rep.plan())
    hard = [rep.h1(flights, lim), rep.h2(flights, lim), rep.h3(flights), rep.h4(flights), rep.h5(flights), rep.h6(flights)]
    value = {"contract": "p7e-e2-pilot-check-v1", "read_only": True, "pilot_result": str(PILOT), "pilot_result_sha256": digest(PILOT),
             "report_code_sha256": digest(Path(rep.__file__)), "assessor_sha256": digest(Path(p7e_assess.__file__)),
             "flights": len(rows), "all_records_match_ledger": all(r["record_matches_ledger"] for r in rows),
             "all_reclassified_identical": all(r["reclassified_identical"] for r in rows),
             "all_run_files_unchanged": all(r["run_files_unchanged"] for r in rows),
             "fc3_changed_validity": [r["run"] for r in rows if r["fc3_validity"] != r["frozen_validity"]],
             "reclassify_wall_s_total": round(sum(r["reclassify_wall_s"] for r in rows), 1),
             "hard_criteria_h1_h6": [{"id": h["id"], "met": h["met"], "value": h["value"]} for h in hard], "rows": rows}
    atomic_json(output, value, exclusive=True)
    print(json.dumps({k: v for k, v in value.items() if k != "rows"}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
