"""P7-E flight assessment with fail-closed rule FC-3 (contract `p7-engineering-validation-v1` §8). Offline only.

The frozen P7 classifier (`p7_classify.py` of candidate 0799c37e) classifies every flight first, unchanged, and
writes `p7/flight-record.json`. It calls a flight infrastructure-invalid when there is no armed evidence. That
includes a flight whose ULog cannot be read, so its arm state is unknown.

**FC-3** narrows that. A flight is infrastructure-invalid (and may be re-flown) only on positive evidence that it
never armed:
* **`ulog_never_armed`:** the ULog was read and never shows ARMED; or
* **`controller_never_started`:** the finalized summary lists no `ros-controller` process; or
* **`no_request_arm`:** the controller's own result (hash-matched to the finalized summary's artifact list) shows it
  never entered REQUEST_ARM.

Any other invalid flight has an unknown arm state. It is safety-unverified: validity `unknown`, and the stop reason
`safety_unverified:fc3_arm_state_unknown` stops the batch. A valid flight, and every stop reason of the frozen
record, pass through unchanged. The assessment is written once to `p7e/flight-assessment.json` (exclusive) and
binds the flight record and the summary by SHA-256.
"""
import importlib.util
import json
from pathlib import Path
import sys

P7E = Path(__file__).resolve().parents[1]
REPO = P7E.parents[1]
P7 = REPO / "simulation/px4_gazebo_p7"
if str(REPO / "simulation/px4_gazebo/validation") not in sys.path:
    sys.path.insert(0, str(REPO / "simulation/px4_gazebo/validation"))
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

CONTRACT = "p7e-flight-assessment-v1"
CLASSIFIER = P7 / "validation/p7_classify.py"
RECORD, OUTPUT = "p7/flight-record.json", "p7e/flight-assessment.json"
FC3_STOP = "safety_unverified:fc3_arm_state_unknown"
CONTROLLER_PROCESS = "ros-controller"


def frozen_classifier():
    spec = importlib.util.spec_from_file_location("p7_classify_frozen", CLASSIFIER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def controller_result(run, summary):
    """The controller's own result file, only if the finalized summary's artifact list hashes it; else None."""
    path = Path(run) / "controller-result.json"
    if not summary.get("finalized") or not path.exists():
        return None
    listed = {a["path"]: a["sha256"] for a in summary.get("artifacts") or []}
    if listed.get("controller-result.json") != digest(path):
        return None
    return strict_json(path)


def never_armed_basis(run, summary, record):
    """FC-3: the positive evidence that an invalid flight never armed, or None if its arm state is unknown."""
    if (record.get("armed_evidence") or {}).get("ulog_armed") is False:
        return "ulog_never_armed", {"ulog_armed": False}
    if summary.get("finalized") is True:
        names = [p.get("name") for p in summary.get("processes") or []]
        if CONTROLLER_PROCESS not in names:
            return "controller_never_started", {"finalized": True, "processes": names}
        result = controller_result(run, summary)
        if result is not None and isinstance(result.get("transitions"), list):
            states = [t.get("to") for t in result["transitions"]]
            if "REQUEST_ARM" not in states:
                return "no_request_arm", {"controller_result_sha256": digest(Path(run) / "controller-result.json"),
                                          "states": states}
    return None, None


def assess(run, record, summary=None):
    """The P7-E assessment of one classified flight (FC-3)."""
    run = Path(run)
    summary = strict_json(run / "summary.json") if summary is None else summary
    validity, stops = record.get("validity"), list(record.get("stop_reasons") or [])
    fc3 = {"applied": False, "basis": None, "evidence": None}
    if validity == "infrastructure_invalid":
        fc3["applied"] = True
        basis, evidence = never_armed_basis(run, summary, record)
        fc3.update(basis=basis, evidence=evidence)
        if basis is None:
            validity = "unknown"
            stops.append(FC3_STOP)
    elif validity != "valid":
        validity = "unknown"
        stops.append(FC3_STOP)
    return {"contract": CONTRACT, "run": run.name, "assessor_sha256": digest(Path(__file__)),
            "flight_record_sha256": digest(run / RECORD) if (run / RECORD).exists() else None,
            "summary_sha256": digest(run / "summary.json"),
            "frozen": {"validity": record.get("validity"), "stop_reasons": record.get("stop_reasons"),
                       "outcome": record.get("outcome")},
            "validity": validity, "outcome": record.get("outcome") if validity == "valid" else None,
            "stop_reasons": stops, "fc3": fc3}


def classify(run):
    """The sequencer's hook: the frozen classifier (in the accepted evaluators' environment), then FC-3."""
    run = Path(run)
    record = frozen_classifier().classify(run)
    output = run / OUTPUT
    if output.exists():
        raise RuntimeError("A P7-E assessment exists; the first assessment is retained")
    output.parent.mkdir(exist_ok=True)
    value = assess(run, record)
    atomic_json(output, value, exclusive=True)
    return value


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        raise SystemExit("Usage: p7e_assess.py RUN   (after the frozen classifier wrote p7/flight-record.json)")
    run = Path(argv[0])
    value = assess(run, strict_json(run / RECORD))
    print(json.dumps({"validity": value["validity"], "stop_reasons": value["stop_reasons"], "fc3": value["fc3"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
