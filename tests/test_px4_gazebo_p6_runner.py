"""P6 M5: the runner's run-local inputs, the manifest's bindings, the integration checks (C11, injectors) and
the sequencer's aggregation, offline. Nothing is launched; tests that import the runner need Linux (fcntl)."""
import copy
import json
from pathlib import Path
import shutil
import sys

import pytest

REPO = Path(__file__).resolve().parents[1]
P6 = REPO / "simulation/px4_gazebo_p6"
for path in (P6, P6 / "validation", P6 / "scripts", REPO / "simulation/px4_gazebo/validation",
             REPO / "simulation/px4_gazebo/scripts", REPO / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control",
             REPO / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance",
             REPO / "simulation/px4_gazebo_p5/ros2_ws/src/gwm_policy_control"):
    sys.path.insert(0, str(path))
import p6_integration_evaluate as integration  # noqa: E402
from injection import declaration, specs  # noqa: E402
from p6_run import Run  # noqa: E402
from p6_safety_evaluate import Checks  # noqa: E402

RULE = specs.strict_json(specs.SCENARIOS)["run_local_config_rule"]
GATE_A = specs.strict_json(specs.SCENARIOS)["gates"]["A"]


def runner():
    pytest.importorskip("fcntl")
    import p6_runner
    return p6_runner


def configs(run, decl, bridge=True):
    """Run-local copies as the accepted runners and the P6 runner write them."""
    run.mkdir(parents=True, exist_ok=True)
    for name, folder in integration.CONFIGS.items():
        shutil.copyfile(folder / "configs" / name, run / name)
    for name in ("p4_avoidance.yaml", "p5_policy.yaml"):
        value = json.loads((run / name).read_text(encoding="utf-8"))
        value["enabled"] = True
        (run / name).write_text(json.dumps(value, indent=2) + "\n")
    changes = []
    if bridge and "DR" in decl["mechanisms"]:
        text = (run / "p3_sensor_bridge.yaml").read_text(encoding="utf-8")
        for accepted, raw in integration._depth_topics():
            text = text.replace("- ros_topic_name: %s\n" % accepted, "- ros_topic_name: %s\n" % raw)
            changes.append({"file": "p3_sensor_bridge.yaml", "before": accepted, "after": raw})
        (run / "p3_sensor_bridge.yaml").write_text(text)
    (run / "summary.json").write_text(json.dumps({"p6": {"run_local_changes": changes}}))
    return Run(run)


def c11(run, decl):
    c = Checks({})
    integration.c11(run, decl, c, RULE)
    return c.out["C11"]


# ---- rule C11 -------------------------------------------------------------------------------------------
def test_c11_accepts_the_accepted_runner_keys_and_the_declared_relay_keys(tmp_path):
    for row in ("A-1", "A-3", "A-5.F1.04", "A-5.F2.09"):
        d = declaration.build(row)
        out = c11(configs(tmp_path / row, d), d)
        assert out["passed"], (row, out)
    out = c11(configs(tmp_path / "a3x", declaration.build("A-3")), declaration.build("A-3"))
    assert {(x["file"], x["key"]) for x in out["changes"]} >= {("p4_avoidance.yaml", "enabled"), ("p5_policy.yaml", "enabled")}


def test_c11_refuses_any_other_change(tmp_path):
    d = declaration.build("A-3")
    run = configs(tmp_path / "other", d)
    value = json.loads((run.file("p4_avoidance.yaml")).read_text(encoding="utf-8"))
    value["perception"]["snapshot_max_age_sim_s"] = 0.5  # an undeclared key
    run.file("p4_avoidance.yaml").write_text(json.dumps(value, indent=2) + "\n")
    assert not c11(run, d)["passed"]
    run = configs(tmp_path / "bytes", d)
    run.file("p2_timing.yaml").write_text(run.file("p2_timing.yaml").read_text(encoding="utf-8") + " ")
    assert c11(run, d)["differences"] == ["p2_timing.yaml"]
    run = configs(tmp_path / "relay-off", declaration.build("A-5.F1.01"), bridge=False)
    text = run.file("p3_sensor_bridge.yaml").read_text(encoding="utf-8").replace("/gwm/sensors/front_depth/image_raw",
                                                                                "/gwm/p6/raw/front_depth/image_raw")
    run.file("p3_sensor_bridge.yaml").write_text(text)  # a row without the depth relay may not move the topic
    assert not c11(run, declaration.build("A-5.F1.01"))["passed"]


def test_c11_needs_the_runner_record_of_the_declared_keys(tmp_path):
    d = declaration.build("A-5.F1.06")
    run = configs(tmp_path / "rec", d)
    run.file("summary.json").write_text(json.dumps({"p6": {"run_local_changes": []}}))
    out = c11(Run(run.path), d)
    assert not out["passed"] and out["runner_record_matches"] is False


def test_c11_allows_the_schema_description_only_for_f3_03(tmp_path):
    from p6_integration_evaluate import SCHEMA_KEY
    for row, ok in (("A-5.F3.03", True), ("A-5.F3.02", False)):
        d = declaration.build(row)
        run = configs(tmp_path / row, d)
        schema = json.loads(run.file("p5_model_schema.json").read_text(encoding="utf-8"))
        schema["candidate_features"][0]["description"] = " "
        run.file("p5_model_schema.json").write_text(json.dumps(schema))
        record = [{"file": "p5_model_schema.json", "before": None, "after": " "}] if row == "A-5.F3.03" else []
        run.file("summary.json").write_text(json.dumps({"p6": {"run_local_changes": record}}))
        out = c11(Run(run.path), d)
        assert out["passed"] is ok, (row, out)
        assert SCHEMA_KEY == "candidate_features[0].description"


# ---- injector processes (Gate A row 6) ------------------------------------------------------------------
def process(name, command=("x",), stop="SIGINT", code=0, forced=False):
    return {"name": name, "command": list(command), "stop_method": stop, "grace_s": 10, "graceful_codes": [0],
            "exit_code": code, "reaped": True, "forced": forced, "remaining_group_members": [],
            "stop_requested_monotonic_s": 1.0, "exit_observed_monotonic_s": 1.5}


def injectors(tmp, decl, processes, armed=None):
    tmp.mkdir(parents=True)
    (tmp / "p6").mkdir()
    for name, mode in (armed or {}).items():
        (tmp / ("p6/fault-ledger-%s.jsonl" % name)).write_text(json.dumps({"kind": "armed", "mode": mode}) + "\n")
    (tmp / "summary.json").write_text(json.dumps({"processes": processes}))
    c = Checks({})
    integration.injectors(Run(tmp), decl, c)
    return c.out["INJECTORS"]


SHIM = ["/usr/bin/python3", "-s", "-P", "-B", "p6_node.py", "controller_shim", "--p6-declaration", "d", "--", "--config"]


def test_pass_through_runs_exactly_the_declared_relays_the_witness_and_the_shim(tmp_path):
    d = declaration.build("A-3")
    relays = [process("p6-witness"), process("p6-depth-relay"), process("p6-state-relay")]
    good = relays + [process("ros-controller", SHIM, stop="completes_own_work")]
    armed = {"depth_relay": "pass-through", "state_relay": "pass-through", "controller_shim": "pass-through"}
    assert injectors(tmp_path / "ok", d, good, armed)["passed"]
    assert not injectors(tmp_path / "plain", d, relays + [process("ros-controller", ["/x/p5_control"], "completes_own_work")], armed)["passed"]
    assert not injectors(tmp_path / "extra", d, good + [process("p6-competing-publisher")], armed)["passed"]
    assert not injectors(tmp_path / "forced", d, [process("p6-witness", forced=True)] + good[1:], armed)["passed"]
    assert not injectors(tmp_path / "mode", d, good, dict(armed, depth_relay="active"))["passed"]


def test_off_rows_leave_no_injector_process_and_run_the_plain_controller(tmp_path):
    d = declaration.build("A-5.F1.01")
    plain = [process("ros-controller", ["/x/p5_control"], "completes_own_work")]
    assert injectors(tmp_path / "ok", d, plain)["passed"]
    assert not injectors(tmp_path / "relay", d, plain + [process("p6-depth-relay")])["passed"]
    assert not injectors(tmp_path / "shim", d, [process("ros-controller", SHIM, "completes_own_work")])["passed"]


# ---- the runner's run-local inputs (Linux) ------------------------------------------------------------------
def test_the_relay_bridge_configuration_changes_only_the_two_declared_keys():
    r = runner()
    frozen = (REPO / "simulation/px4_gazebo/configs/p3_sensor_bridge.yaml").read_text(encoding="utf-8")
    text, changes = r.relay_bridge_config(frozen)
    differing = [(a, b) for a, b in zip(frozen.splitlines(), text.splitlines()) if a != b]
    assert len(differing) == 2 and all(b.startswith("- ros_topic_name: /gwm/p6/raw/front_depth/") for _, b in differing)
    assert [c["key"] for c in changes] == ["[image_raw].ros_topic_name", "[camera_info].ros_topic_name"]
    with pytest.raises(ValueError):
        r.relay_bridge_config(text)  # already moved: the accepted key is no longer unique


def test_the_f3_03_schema_still_loads_and_refuses_only_on_its_hash(tmp_path):
    r = runner()
    sys.path.insert(0, str(REPO / "simulation/px4_gazebo_p5/ros2_ws/src/gwm_policy_control"))
    from gwm_policy_control.contracts import load_schema, schema_sha256
    frozen = REPO / "simulation/px4_gazebo_p5/configs/p5_model_schema.json"
    text, change, digest = r.changed_schema(frozen.read_text(encoding="utf-8"))
    path = tmp_path / "schema.json"
    path.write_text(text)
    assert schema_sha256(load_schema(path)) == digest != schema_sha256(load_schema(frozen))
    assert change["key"] == "candidate_features[0].description" and change["after"] == " "


def test_every_live_gate_a_row_maps_to_an_accepted_scope_and_a_frozen_declaration():
    r = runner()
    import run_p6_gates
    for row in GATE_A:
        if row["scope"] in run_p6_gates.LIVE:
            assert row["scope"] in r.P5_SCOPE, row["id"]
            d = declaration.build(row["id"])
            assert declaration.validate(d) == d
            if row["scope"] == "ground_fixture":
                assert d["trigger"] is not None and (d["fault"] in r.LIVE_DETECTION or d["mechanisms"])
            if row["scope"] == "startup_refusal":
                assert d["fault"] in r.REFUSALS
        else:
            assert row["scope"] in ("offline", "evaluation"), row["id"]
    live = [row for row in GATE_A if row["scope"] in run_p6_gates.LIVE]
    assert len(live) == specs.strict_json(specs.SCENARIOS)["counts"]["gate_a_live_ground_runs"] == 14


def test_observation_covers_the_declared_fault_window():
    r = runner()
    assert r.observation_end(declaration.build("A-5.F1.01"), 10.0) == 15.0
    assert r.observation_end(declaration.build("A-5.F1.04"), 10.0) == 18.0
    assert r.observation_end(declaration.build("A-5.F1.06"), 10.0) == 18.0


# ---- manifest bindings ------------------------------------------------------------------------------------
def manifest():
    pytest.importorskip("fcntl")  # the accepted P5 manifest module imports the P3 provenance chain
    import p6_manifest
    return p6_manifest


def test_the_manifest_binds_every_p6_source_and_test_and_the_owner_copy():
    m = manifest()
    files = m.p6_source_files()
    assert "simulation/px4_gazebo_p6/scripts/p6_runner.py" in files and "tests/test_px4_gazebo_p6_runner.py" in files
    assert all("__pycache__" not in path for path in files)
    assert set(m.injector_identity()) >= {"depth_relay.py", "state_relay.py", "witness.py", "controller_shim.py"}
    assert m.owner_identity()["sha256"] == integration._sha(REPO / "simulation/px4_gazebo/scripts/p3_namespace_owner.py")


def test_a_p5_policy_approval_never_transfers_and_dry_runs_need_a_development_candidate(tmp_path):
    m = manifest()
    p5 = tmp_path / "p5.json"
    p5.write_text(json.dumps({"candidate_phase": "P5 p5-acceptance-v1"}))
    with pytest.raises(PermissionError):
        m.check_policy_approval(p5, development=True)
    dev = tmp_path / "dev.json"
    dev.write_text(json.dumps({"candidate_phase": "P6 M5 development candidates", "development_only": True}))
    assert m.check_policy_approval(dev, development=True)
    with pytest.raises(PermissionError):
        m.check_policy_approval(dev, development=False)
    approval = tmp_path / "dry.json"
    approval.write_text(json.dumps(m.dry_run_template("ab" * 32, "operator", "Authorize M5")))
    assert m.require_dry_run_approval(approval, "ab" * 32, {"development": True})["live_runs_max"] == 40
    with pytest.raises(PermissionError):
        m.require_dry_run_approval(approval, "ab" * 32, {"development": False})
    with pytest.raises(PermissionError):
        m.require_dry_run_approval(approval, "cd" * 32, {"development": True})


# ---- sequencer aggregation (A-6, A-7) ---------------------------------------------------------------------
def test_a6_and_a7_aggregate_every_evaluated_live_row():
    pytest.importorskip("fcntl")
    import run_p6_gates
    good = {"row": "A-3", "evaluation": {"response": {"INJECTORS": {"passed": True}}, "p5_failures_refused": [],
                                         "p5_failures_allowed": [{"item": "depth:receipt_stall"}]}}
    bad = copy.deepcopy(good)
    bad["row"] = "A-5.F1.04"
    bad["evaluation"]["response"]["INJECTORS"]["passed"] = False
    bad["evaluation"]["p5_failures_allowed"].append({"item": "cleanup:p6-depth-relay:forced_kill_after_grace"})
    assert run_p6_gates.aggregate("A-6", [good])["failures"] == []
    assert run_p6_gates.aggregate("A-6", [good, bad])["failures"] == ["injectors:A-5.F1.04"]
    assert run_p6_gates.aggregate("A-7", [good, bad])["failures"] == ["A-5.F1.04:cleanup:p6-depth-relay:forced_kill_after_grace"]
    assert run_p6_gates.aggregate("A-7", [{"row": "A-2"}])["failures"] == ["no_evaluated_live_rows"]


# ---- live fail-closed proofs (F4.07, F4.13) ---------------------------------------------------------------
def fail_closed_run(tmp, decl, failure, abort_wall, processes=(), ledgers=None, trace=()):
    (tmp / "p6").mkdir(parents=True)
    (tmp / "p6/fault-declaration.json").write_text(json.dumps(decl))
    (tmp / "ros-events.jsonl").write_text(json.dumps({"event": "abort", "reason": failure, "sim_s": 9.0,
                                                      "monotonic_s": abort_wall}) + "\n")
    (tmp / "timing-trace.jsonl").write_text("".join(json.dumps(s) + "\n" for s in trace))
    (tmp / "summary.json").write_text(json.dumps({"processes": list(processes), "controller_result": {"failure": failure}}))
    for name, records in (ledgers or {}).items():
        (tmp / ("p6/fault-ledger-%s.jsonl" % name)).write_text("".join(json.dumps(r) + "\n" for r in records))
    return Run(tmp)


def advertiser(launched=104.0, exited=110.0, code=0):
    return dict(process("p6-competing-publisher", stop="completes_own_work", code=code), launched_monotonic_s=launched,
                exit_observed_monotonic_s=exited)


def test_the_competing_publisher_proof_reads_the_controllers_own_reason_inside_the_advertisers_lifetime(tmp_path):
    from p6_fault_evaluate import prove
    d = declaration.build("A-5.F4.07")
    reason = "competing_control_publisher:" + d["parameters"]["topic"]
    ledger = {"competing_publisher": [{"kind": "declared"}, {"kind": "activated", "advertised_wall_s": 104.8},
                                      {"kind": "finished", "messages_published": 0}]}
    trace = [{"operation": "graph_check", "duration_wall_s": 0.004}]
    assert prove(fail_closed_run(tmp_path / "ok", d, reason, 105.6, [advertiser()], ledger, trace), d)["proved"]
    late = prove(fail_closed_run(tmp_path / "late", d, reason, 106.0, [advertiser()], ledger, trace), d)
    assert "proof:detection_later_than_1_0_s_plus_graph_query" in late["failures"]
    before = prove(fail_closed_run(tmp_path / "before", d, reason, 103.0, [advertiser()], ledger, trace), d)
    assert "proof:detection_outside_the_advertiser_lifetime" in before["failures"]
    other = prove(fail_closed_run(tmp_path / "other", d, "clock_stalled", 105.6, [advertiser()], ledger, trace), d)
    assert not other["proved"]
    published = dict(ledger, competing_publisher=ledger["competing_publisher"][:2] + [{"kind": "finished", "messages_published": 1}])
    assert not prove(fail_closed_run(tmp_path / "pub", d, reason, 105.6, [advertiser()], published, trace), d)["proved"]


def test_the_clock_stall_proof_needs_the_kernel_stop_and_a_detection_while_stopped(tmp_path):
    from p6_fault_evaluate import prove
    d = declaration.build("A-5.F4.13")
    act = {"kind": "activated", "signal": "SIGSTOP", "proved": True, "request_wall_s": 100.0, "stopped_seen_wall_s": 100.01,
           "resumed_seen_wall_s": 102.02}
    ledger = {"process_signal": [{"kind": "declared"}, act]}
    assert prove(fail_closed_run(tmp_path / "ok", d, "clock_stalled", 101.03, ledgers=ledger), d)["proved"]
    late = prove(fail_closed_run(tmp_path / "late", d, "clock_stalled", 101.2, ledgers=ledger), d)
    assert "proof:clock_stall_detection_later_than_1_0_s_plus_one_tick" in late["failures"]
    unproved = {"process_signal": [{"kind": "declared"}, dict(act, proved=False)]}
    assert not prove(fail_closed_run(tmp_path / "proc", d, "clock_stalled", 101.03, ledgers=unproved), d)["proved"]
    assert not prove(fail_closed_run(tmp_path / "none", d, "clock_stalled", 101.03), d)["proved"]


# ---- ROS node QoS (M5 live findings) ----------------------------------------------------------------------
def test_every_p6_node_takes_the_clock_best_effort_and_no_relay_copies_graph_qos_verbatim():
    """A reliable `/clock` subscription never matches the best-effort clock bridge, and graph endpoint QoS has
    no history or depth, so `create_publisher` refuses it (both seen in the first live A-3 run)."""
    for name in ("depth_relay", "state_relay", "witness"):
        text = (P6 / "injection" / (name + ".py")).read_text(encoding="utf-8")
        assert 'create_subscription(Clock, "/clock", self.clock, clock_profile())' in text, name
        assert "return infos[0].qos_profile\n" not in text, name


# ---- ground proofs completed in M5 (F1.02, the F2.09 window) ------------------------------------------------
def ground_run(tmp, decl, files):
    for name, rows in files.items():
        target = tmp / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("".join(json.dumps(r) + "\n" for r in rows) if name.endswith(".jsonl") else json.dumps(rows))
    (tmp / "p6").mkdir(parents=True, exist_ok=True)
    (tmp / "p6/fault-declaration.json").write_text(json.dumps(decl))
    return Run(tmp)


def test_the_adapter_stop_proof_needs_its_own_exit_and_a_live_native_source(tmp_path):
    from p6_fault_evaluate import prove
    d = declaration.build("A-5.F1.02")
    injection = {"process": "sensor-adapter", "signal": "SIGINT", "request_monotonic_s": 100.0, "t_inj_sim_s": 30.0}
    adapter = dict(process("sensor-adapter"), stop_requested_monotonic_s=100.0, exit_observed_monotonic_s=100.4)
    files = {"p4/fault_injection.json": injection, "summary.json": {"processes": [adapter]},
             "sensors/health.jsonl": [{"wall_s": 99.9, "sim_s": 29.9, "status": "fresh"}],
             "source/depth-headers.jsonl": [{"stamp_ns": int(t * 1e9)} for t in (29.9, 30.5, 31.0)]}
    assert prove(ground_run(tmp_path / "ok", d, files), d)["proved"]
    late = dict(files, **{"sensors/health.jsonl": files["sensors/health.jsonl"] + [{"wall_s": 101.0, "sim_s": 31.0, "status": "fresh"}]})
    assert "proof:adapter_health_after_exit" in prove(ground_run(tmp_path / "late", d, late), d)["failures"]
    quiet = dict(files, **{"source/depth-headers.jsonl": [{"stamp_ns": int(29.9e9)}]})
    assert "proof:native_source_stopped_too" in prove(ground_run(tmp_path / "quiet", d, quiet), d)["failures"]


def test_the_ground_pose_proof_is_bounded_by_its_window(tmp_path):
    from p6_fault_evaluate import prove
    d = declaration.build("A-5.F2.09")  # withheld for 5 s, then the relay resumes
    true = [{"receipt_sim_s": 30.0 + 0.01 * k} for k in range(1000)]
    relayed = [w for w in true if not 30.0 < w["receipt_sim_s"] <= 35.0]
    files = {"p6/fault-ledger-state_relay.jsonl": [{"kind": "activated", "sim_s": 30.0}],
             "p6/witness/true_position.jsonl": true, "p6/witness/true_attitude.jsonl": true,
             "p6/witness/relayed_position.jsonl": relayed, "p6/witness/relayed_attitude.jsonl": relayed,
             "p4/local_map.jsonl": [{"processing_sim_s": 31.0, "usable": False, "reason": "association:position:pending_newer_state"}]}
    assert prove(ground_run(tmp_path / "ok", d, files), d)["proved"]
    startup = dict(files, **{"p4/local_map.jsonl": [{"processing_sim_s": 5.0, "usable": False,
                                                        "reason": "association:position:missing_history"}]})
    assert "proof:perception_never_unassociated_in_window" in prove(ground_run(tmp_path / "startup", d, startup), d)["failures"]
