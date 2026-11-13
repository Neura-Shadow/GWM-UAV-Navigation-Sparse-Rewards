"""P6 integration checks every live run gets (contract C7, C11, §8.1 rows 3 and 6), from the run's own files.

* `C11`: every run-local configuration equals its frozen original, except the accepted runners' own keys
  (`p6_scenarios.yaml` `run_local_config_rule`) and the keys the row's declaration implies (the two depth
  `ros_topic_name` values for the depth relay; the F3.03 schema description). Each allowed key's before and
  after values are recorded. The P3–P5 evaluators do not make this check (M0 Finding C).
* `INJECTORS` (Gate A row 6): the P6 processes are exactly those the declaration needs (off leaves no
  process, pass-through adds only the declared relays and the witness), the controller runs under the shim
  exactly when the declaration uses it, every injector ledger is armed in the declared mode, and every P6
  process ended gracefully by the accepted P3 classification.
* `RELAY` (Gate A row 3, reported): relay latency from the witness's raw and relayed receipts (one clock),
  and relay and witness CPU from the runner's resource samples.
* `GROUND_POLICY` (Gate A row 3): the accepted P5 ground policy evaluation (revision 2, bit-exact replay
  under the controller interpreter) on the run's recorded ground snapshots.
"""
import copy
import hashlib
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SIM, P4, P5 = (ROOT / "simulation" / name for name in ("px4_gazebo", "px4_gazebo_p4", "px4_gazebo_p5"))
CONFIGS = {"p3_sensors.yaml": SIM, "p3_validation.yaml": SIM, "p3_sensor_bridge.yaml": SIM, "p3_fastdds.xml": SIM,
           "p3_native_command_policy.json": SIM, "p2_control.yaml": SIM, "p2_timing.yaml": SIM, "versions.lock.yaml": SIM,
           "p4_validation.yaml": P4, "p4_scenarios.json": P4, "p4_avoidance.yaml": P4,
           "p5_validation.yaml": P5, "p5_model_schema.json": P5, "p5_policy.yaml": P5}
SCHEMA_KEY = "candidate_features[0].description"
RELAY_PROCESSES = {"DR": "p6-depth-relay", "SR": "p6-state-relay"}


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _tokens(key):
    return [int(t[1:-1]) if t.startswith("[") else t for t in re.findall(r"\[\d+\]|[^.\[\]]+", key)]


def _pop(value, key):
    """Remove one dotted/indexed key from a parsed document; returns (found, removed value)."""
    *parents, last = _tokens(key)
    for token in parents:
        try:
            value = value[token]
        except (KeyError, IndexError, TypeError):
            return False, None
    if isinstance(value, dict) and last in value:
        return True, value.pop(last)
    return False, None


def _depth_topics():
    import sys
    sys.path.insert(0, str(HERE.parent))
    from injection import witness
    topics = dict(witness.DEPTH)
    return [(topics["relayed_image"], topics["raw_image"]), (topics["relayed_info"], topics["raw_info"])]


def allowed_keys(declaration, rule):
    keys = {name: list(values) for name, values in rule["accepted_runner_changes"].items()}
    if declaration.get("fault") == "F3.03":
        keys.setdefault("p5_model_schema.json", []).append(SCHEMA_KEY)
    return keys


def c11(run, declaration, c, rule):
    """Byte equality, or for files with allowed keys, equality of the parsed documents with those keys removed."""
    allowed = allowed_keys(declaration, rule)
    relay = "DR" in declaration["mechanisms"]
    differences, changes = [], []
    for name, folder in CONFIGS.items():
        local, frozen = run.file(name), folder / "configs" / name
        if not local.exists():
            differences.append("missing:" + name)
            continue
        if name == "p3_sensor_bridge.yaml" and relay:
            text = frozen.read_text(encoding="utf-8")
            for accepted, raw in _depth_topics():
                text = text.replace("- ros_topic_name: %s\n" % accepted, "- ros_topic_name: %s\n" % raw, 1)
                changes.append({"file": name, "key": "ros_topic_name", "before": accepted, "after": raw})
            if local.read_text(encoding="utf-8") != text:
                differences.append(name)
        elif name in allowed:
            a, b = json.loads(frozen.read_text(encoding="utf-8")), json.loads(local.read_text(encoding="utf-8"))
            a, b = copy.deepcopy(a), copy.deepcopy(b)
            for key in allowed[name]:
                found_a, before = _pop(a, key)
                found_b, after = _pop(b, key)
                if found_a or found_b:
                    changes.append({"file": name, "key": key, "before": before, "after": after})
            if a != b:
                differences.append(name)
        elif _sha(local) != _sha(frozen):
            differences.append(name)
    declared = [x for x in changes if x["file"] in ("p3_sensor_bridge.yaml", "p5_model_schema.json")]
    recorded = ((run.summary.get("p6") or {}).get("run_local_changes")) or []
    recorded_pairs = sorted((x["file"], str(x["before"]), str(x["after"])) for x in recorded)
    declared_pairs = sorted((x["file"], str(x["before"]), str(x["after"])) for x in declared)
    c.record("C11", not differences and recorded_pairs == declared_pairs, differences=differences, changes=changes,
             runner_record_matches=recorded_pairs == declared_pairs)


def injectors(run, declaration, c):
    import sys
    sys.path.insert(0, str(HERE.parent))
    from injection.controller_shim import shim_mode
    from p3_process_exit import classify
    processes = run.summary.get("processes") or []
    present = sorted(p["name"] for p in processes if p["name"].startswith("p6-"))
    expected = {RELAY_PROCESSES[m] for m in ("DR", "SR") if m in declaration["mechanisms"]}
    if expected:
        expected.add("p6-witness")
    if declaration.get("fault") == "F4.07":
        expected.add("p6-competing-publisher")
    controller = next((p for p in processes if p["name"] == "ros-controller"), None)
    shim_used = controller is not None and "controller_shim" in [str(x) for x in controller["command"]]
    shim_expected = shim_mode(declaration) != "off"
    armed = {name: next((r.get("mode") for r in records if r["kind"] == "armed"), None)
             for name, records in run.fault_ledgers().items() if name in ("depth_relay", "state_relay", "controller_shim")}
    modes_ok = all(mode == declaration["mode"] for mode in armed.values())
    cleanup = {p["name"]: classify(p) for p in processes if p["name"].startswith("p6-")}
    graceful = all(cls == "graceful" for cls, _ in cleanup.values())
    ok = (set(present) == expected and (controller is None or shim_used == shim_expected) and modes_ok and graceful
          and len(present) == len(set(present)))
    c.record("INJECTORS", ok, present=present, expected=sorted(expected), shim_used=shim_used, shim_expected=shim_expected,
             armed_modes=armed, cleanup={k: list(v) for k, v in cleanup.items()})


def _quantiles(values):
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None}
    ordered = sorted(values)
    pick = lambda q: ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1) + 0.5))]  # noqa: E731
    return {"count": len(ordered), "p50": pick(0.5), "p95": pick(0.95), "max": ordered[-1]}


def relay(run, c):
    """Reported: witness latency raw -> relayed (same process clock) and the relays' CPU share."""
    out = {}
    for raw_name, relayed_name in (("raw_image", "relayed_image"), ("true_position", "relayed_position")):
        raw = {w["stamp_ns"]: w["receipt_wall_s"] for w in run.witness(raw_name)}
        relayed = [w for w in run.witness(relayed_name) if w["stamp_ns"] in raw]
        out[relayed_name] = dict(_quantiles([w["receipt_wall_s"] - raw[w["stamp_ns"]] for w in relayed]),
                                 raw_records=len(raw), relayed_records=len(run.witness(relayed_name)))
    rows = []
    path = run.file("resources.jsonl")
    if path.exists():
        rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    cpu = {}
    for name in ("p6-depth-relay", "p6-state-relay", "p6-witness", "sensor-adapter", "p4-perception"):
        samples = [(r["monotonic_s"], r["processes"][name]["cpu_s"]) for r in rows if name in r.get("processes", {})]
        if len(samples) >= 2 and samples[-1][0] > samples[0][0]:
            cpu[name] = (samples[-1][1] - samples[0][1]) / (samples[-1][0] - samples[0][0])
    out["cpu_fraction"] = cpu
    ok = out["relayed_image"]["count"] > 0 and out["relayed_position"]["count"] > 0
    c.record("RELAY", ok, **out)


def ground_policy(run, c):
    """The accepted P5 ground policy evaluation on this run's recorded ground snapshots, unchanged."""
    import p5_evaluate as p5e
    limits = json.loads((P5 / "configs/p5_validation.yaml").read_text(encoding="utf-8"))["policy"]
    f = p5e.Failures()
    window = run.summary.get("sensor_window_sim_s")
    try:
        replayed = p5e.replay(run.path, run.summary, "fixture", window)
        p5e.replay_runtime(replayed, run.summary, f)
        ground = p5e.policy_ground(replayed, limits, f)
    except (ValueError, KeyError, FileNotFoundError, TypeError) as exc:
        ground = None
        f.check(False, "policy_ground_error:" + type(exc).__name__ + ":" + str(exc)[:300])
    c.record("GROUND_POLICY", not f.items, items=f.items, window_sim_s=window,
             ground={k: v for k, v in (ground or {}).items() if k != "actions"})
