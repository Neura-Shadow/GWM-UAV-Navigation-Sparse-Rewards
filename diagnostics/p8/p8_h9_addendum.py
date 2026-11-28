"""P8 H9 assessment addendum for finding F-R1 (docs/v3_sim_p8_report.md §5). Offline and read-only.

**Why it exists.** The P8 report (`p8_report.py`, bound by the candidate) ran once and recorded "engineering
validation not passed", with the single reason "H9 not met". H9's assessment check compares each P8 assessment with
a fresh P7-E assessment on every key except `stop_reasons`. That includes `contract`, which the P8 sequencer
(`run_p8_block.assess`) rewrites by design from `p7e-flight-assessment-v1` to `p8-flight-assessment-v1`. So the
check fails on every flight.

**What it does not do.**
* It is not part of the P8 candidate (it lives in `diagnostics/`), so the frozen candidate does not drift.
* It does not change, replace or rerun the report. The report's status stays as written.
* It sets no threshold. Whether P8's acceptance may rest on the report and this addendum is the operator's decision.

**What it does,** in order, stopping at the first failure:
1. **Checks** (refused otherwise):
   * the P8 manifest verifies, with no source drift, and this tool is not one of its bound files;
   * the report exists, has the given SHA-256 and binds this manifest;
   * the report's status is "not passed", with exactly the reason "H9 not met";
   * in the report's H9 detail, every check passed except the assessment check, which failed on every flight;
   * the output does not exist.
2. **For each flight,** loaded through the report's own `load` (each file accepted only by its hash):
   * the same fresh P7-E assessment as H9 (`p7e_assess.assess(run, record, summary)`);
   * **rule O**, the report's own comparison, which must reproduce the report's row;
   * **rule C**, the corrected comparison. It is stricter than rule O in three ways:
     * `contract` must be exactly the sequencer's relabel, not merely ignored;
     * `stop_reasons` must equal the P7-E stop reasons plus the FC-5 or FC-6 reason the sequencer derives from the
       H10 record;
     * the assessment's `h10` summary must equal the H10 record, and no other key may be present.
   * No run file may change (size and mtime).
3. **Status:**
   * "explained" only if rule O reproduces the report on every flight, rule C holds on every flight, the only key
     rule O finds different is `contract` on every flight, and no run file changed;
   * otherwise "not explained".
4. **The record is written once** (exclusive create).

Usage (WSL, from /tmp, PYTHONDONTWRITEBYTECODE=1, GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1):
  /usr/bin/python3 p8_h9_addendum.py --manifest STATE/p8-candidate-2a0ae57.json \
    --report STATE/p8-engineering-report-fe2a0c6e9052.json --report-sha256 <sha> --output STATE/p8-h9-addendum-….json
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time

REPO = Path(__file__).resolve().parents[2]
P8 = REPO / "simulation/px4_gazebo_p8"
if str(P8 / "analysis") not in sys.path:
    sys.path.insert(0, str(P8 / "analysis"))
import p8_report as rep  # noqa: E402  (the bound report code, imported unchanged; it sets the other paths)

p8_manifest, run_p8_block, rep7, p7e_assess = rep.p8_manifest, rep.run_p8_block, rep.rep7, rep.p7e_assess
from p3_provenance import atomic_json, digest  # noqa: E402

CONTRACT = "p8-h9-assessment-addendum-v1"
P7E_LABEL = p7e_assess.CONTRACT  # "p7e-flight-assessment-v1"
P8_LABEL = "p8-flight-assessment-v1"  # the label `run_p8_block.assess` writes
NOT_PASSED, H9_REASON = rep.NOT_PASSED, "H9 not met"
OTHER_H9_CHECKS = (("worlds", "verified"), ("reclassification", "reproduced"), ("h10", "reproduced"),
                   ("unchanged", "unchanged"))
STATEMENT = ("This addendum does not change the P8 report, whose status stays as written. It records whether the one "
             "failing H9 check is explained by the sequencer's contract label alone. Whether P8's acceptance may rest "
             "on the report and this addendum is the operator's decision.")


class Refused(RuntimeError):
    """A check failed; nothing is written."""


# ---- the comparisons ---------------------------------------------------------------------------------------------
def rule_o(again, assessment):
    """The report's own H9 comparison, unchanged: every key of the fresh assessment except `stop_reasons`."""
    mine = {k: (assessment or {}).get(k) for k in again if k not in ("stop_reasons",)}
    return {k: again[k] for k in mine} == mine


def differing_keys(again, assessment):
    return sorted(k for k in again if k != "stop_reasons" and again[k] != (assessment or {}).get(k))


def expected_stop_reasons(again, h10_record):
    """As `run_p8_block.assess` derives them: the P7-E stop reasons, plus FC-6 or FC-5 when H10 failed."""
    stops = list(again.get("stop_reasons") or [])
    if not (h10_record or {}).get("ok"):
        violations = (h10_record or {}).get("violations") or []
        fc6 = any(any(m in v for m in run_p8_block.FC6_MARKERS) for v in violations)
        stops.append(run_p8_block.FC6 if fc6 else run_p8_block.FC5)
    return stops


def expected_h10_summary(h10_record, h10_sha256):
    h = h10_record or {}
    return {"ok": h.get("ok"), "actions": h.get("actions"), "refusals": h.get("refusals"),
            "violations": (h.get("violations") or [])[:20], "record_sha256": h10_sha256}


def rule_c(again, assessment, h10_record, h10_sha256):
    """The corrected, stricter comparison. Returns (holds, problems)."""
    a = assessment or {}
    problems = []
    for k in again:
        if k in ("contract", "stop_reasons"):
            continue
        if a.get(k) != again[k]:
            problems.append("differs:" + k)
    if again.get("contract") != P7E_LABEL:
        problems.append("p7e_label:" + str(again.get("contract")))
    if a.get("contract") != P8_LABEL:
        problems.append("p8_label:" + str(a.get("contract")))
    if a.get("stop_reasons") != expected_stop_reasons(again, h10_record):
        problems.append("stop_reasons")
    if a.get("h10") != expected_h10_summary(h10_record, h10_sha256):
        problems.append("h10_summary")
    extra = sorted(set(a) - set(again) - {"h10"})
    if extra:
        problems.append("extra_keys:" + ",".join(extra))
    return not problems, problems


def compare_flight(f, again, report_row, before, after):
    """One flight's row. `report_row` is the report's H9 assessment row for this flight."""
    run = Path(f["run"])
    h10_sha = digest(run / run_p8_block.H10_RECORD) if (run / run_p8_block.H10_RECORD).exists() else None
    holds, problems = rule_c(again, f["assessment"], f["h10"], h10_sha)
    o = rule_o(again, f["assessment"])
    return dict(rep7.ref(f), variant=f.get("variant"), rule_o_reproduced=o,
                rule_o_matches_report=report_row is not None and report_row.get("reproduced") is o,
                rule_o_differing_keys=differing_keys(again, f["assessment"]), rule_c_holds=holds,
                rule_c_problems=problems, run_files_unchanged=before == after)


def summarize(rows, flights_expected):
    n = len(rows)
    out = {"flights": n, "flights_expected": flights_expected,
           "rule_o_reproduced": sum(r["rule_o_reproduced"] for r in rows),
           "rule_o_matches_report": sum(r["rule_o_matches_report"] for r in rows),
           "rule_o_differs_on_contract_alone": sum(r["rule_o_differing_keys"] == ["contract"] for r in rows),
           "rule_c_holds": sum(r["rule_c_holds"] for r in rows),
           "run_files_unchanged": sum(r["run_files_unchanged"] for r in rows)}
    explained = (n == flights_expected and n > 0 and out["rule_o_matches_report"] == n
                 and out["rule_o_differs_on_contract_alone"] == n and out["rule_c_holds"] == n
                 and out["run_files_unchanged"] == n)
    out["status"] = "explained" if explained else "not explained"
    return out


# ---- the checks ----------------------------------------------------------------------------------------------------
def check_report(report, sha, flights_expected):
    """The report must be the not-passed report of this manifest, failing on H9's assessment check alone."""
    if (report.get("manifest") or {}).get("sha256") != sha:
        raise Refused("the report does not bind manifest %s" % sha[:12])
    if report.get("status") != NOT_PASSED or report.get("not_passed_reasons") != [H9_REASON]:
        raise Refused("the report is not 'not passed' for exactly 'H9 not met'")
    hard = {h.get("id"): h for h in report.get("hard_criteria") or []}
    if any(not hard.get("H%d" % i, {}).get("met") for i in range(1, 12) if i != 9) or hard.get("H9", {}).get("met"):
        raise Refused("the report's hard criteria are not 'all met except H9'")
    h9 = hard["H9"]
    if not (h9.get("manifest") or {}).get("verified") or not h9.get("results_bind_manifest"):
        raise Refused("the report's H9 manifest checks did not pass")
    for key, field in OTHER_H9_CHECKS:
        rows = h9.get(key) or []
        if len(rows) != flights_expected or not all(r.get(field) for r in rows):
            raise Refused("the report's H9 %s check did not pass on every flight" % key)
    rows = h9.get("assessment") or []
    if len(rows) != flights_expected or any(r.get("reproduced") for r in rows):
        raise Refused("the report's H9 assessment check did not fail on every flight")
    return rows


def preconditions(root, manifest, report_path, report_sha, output):
    frozen, sha = p8_manifest.verify_current(root, manifest)
    drift = p8_manifest.source_drift(frozen, root)
    if drift:
        raise Refused("source drift: %s" % drift[:4])
    me = Path(__file__).resolve().relative_to(REPO).as_posix()
    if me in (frozen.get("p8_source_files") or {}):
        raise Refused("this tool must not be a bound P8 file")
    if not Path(report_path).exists() or digest(report_path) != report_sha:
        raise Refused("the report is missing or its SHA-256 is not %s" % report_sha[:12])
    if Path(output).exists():
        raise Refused("the addendum exists; it is written once")
    return frozen, sha


def build(root, manifest, report_path, report_sha, output, load=rep.load, assess=p7e_assess.assess):
    _frozen, sha = preconditions(root, manifest, report_path, report_sha, output)
    report = json.loads(Path(report_path).read_text())
    results, flights = load(root, sha)
    expected = sum(len(r.get("flights") or []) for r in results.values())
    report_rows = check_report(report, sha, expected)
    by_ref = {(r["block"], r["slot"], r["controller"], r["attempt"]): r for r in report_rows}
    rows = []
    for f in flights:
        run = Path(f["run"])
        before = rep7.files_state(run)
        again = assess(run, f["record"], f["summary"])
        after = rep7.files_state(run)
        rows.append(compare_flight(f, again, by_ref.get((f["block"], f["slot"], f["controller"], f["attempt"])),
                                   before, after))
    return {"contract": CONTRACT, "finding": "F-R1", "tool_sha256": digest(Path(__file__)),
            "tool_path": Path(__file__).resolve().relative_to(REPO).as_posix(), "bound_by_candidate": False,
            "manifest_sha256": sha, "report": {"path": str(report_path), "sha256": report_sha,
                                                "status": report["status"], "not_passed_reasons": report["not_passed_reasons"],
                                                "report_code_sha256": report.get("report_code_sha256")},
            "labels": {"p7e_assessment": P7E_LABEL, "p8_assessment": P8_LABEL},
            "rules": {"O": "the report's H9 comparison: every key of the fresh P7-E assessment except stop_reasons",
                      "C": "every key equal except contract and stop_reasons; contract exactly the sequencer's relabel; "
                           "stop_reasons as the sequencer derives them from the H10 record; the h10 summary equal to "
                           "the H10 record; no other key"},
            "summary": summarize(rows, expected), "flights": rows, "statement": STATEMENT,
            "generated_unix_ns": time.time_ns()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--report-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    try:
        value = build(root, args.manifest, args.report, args.report_sha256, args.output)
    except Refused as exc:
        print(json.dumps({"refused": str(exc)}))
        return 2
    atomic_json(args.output, value, exclusive=True)
    print(json.dumps({"output": str(args.output), "sha256": digest(args.output), "summary": value["summary"]}))
    return 0 if value["summary"]["status"] == "explained" else 1


if __name__ == "__main__":
    raise SystemExit(main())
