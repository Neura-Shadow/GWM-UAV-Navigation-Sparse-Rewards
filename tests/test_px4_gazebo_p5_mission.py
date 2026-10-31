"""P5 M4: `P5Mission`, the policy ledger and the `p5_control` start-up checks, offline (no ROS, no PX4).

The mission rig is the P4 M4 rig: synthetic PX4 state, ideal tracking, production perception on
independently rendered depth at 10 Hz, mission ticks at 20 Hz. Model behaviour is exercised on a
held-out training-pipeline layout, never on an acceptance scenario (contract §4.1).
"""
import inspect
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402
from gwm_obstacle_avoidance import control_node as p4_node, local_map, mission as p4_mission, perception  # noqa: E402
from gwm_obstacle_avoidance import supervisor  # noqa: E402
from gwm_policy_control import candidate_ranker, control_node, mission, policy_ledger, safety_shield  # noqa: E402
from gwm_policy_control.policy_runtime import FrozenPolicy, PolicyError, save_model  # noqa: E402
from gwm_px4_control import contracts as p2_contracts, timing  # noqa: E402
from gwm_px4_control.contracts import strict_json  # noqa: E402
from gwm_sensor_adapter.geometry import Extrinsic  # noqa: E402

syn = p5.syn
P2 = p2_contracts.load_config(syn.SIM / "configs/p2_control.yaml")
P4C = syn.p4_config()
P5C = json.loads((p5.P5 / "configs/p5_policy.yaml").read_text(encoding="utf-8"))
SENSORS = syn.sensors()
EXTRINSIC = Extrinsic(SENSORS["optical_origin_base_flu_m"])
COLUMNS = np.arange(2, 640, 4)
YAW0_NED = math.pi / 2
MODELS = p5.P5 / "models"
ARTIFACT = next(iter(sorted(MODELS.glob("p5-ranker-*.npz"))), None)
needs_artifact = pytest.mark.skipif(ARTIFACT is None, reason="no exported P5 model artifact")


def trained_policy():
    return FrozenPolicy.load(ARTIFACT, ARTIFACT.stem[len("p5-ranker-"):], p5.SCHEMA_SHA, require_trained=True)


def held_out_layout():
    """The first held-out layout of the readiness report that both P4 and the policy completed."""
    report = json.loads(ARTIFACT.with_name(ARTIFACT.stem + ".readiness_report.json").read_text(encoding="utf-8"))
    layouts = {layout["id"]: layout for layout in report["layouts"]}
    first = next(e for e in report["episodes"] if e["p4"]["outcome"] == e["policy"]["outcome"] == "goal_reached"
                 and len(layouts[e["id"]]["boxes"]) >= 2)
    return layouts[first["id"]]


class Rig:
    """The P4 M4 rig with a mission factory."""

    def __init__(self, make_mission, boxes=(), state="MAP_SCAN"):
        self.cache = timing.StateCache(P2)
        self.m = make_mission(self.cache)
        self.scene = syn.boxes_3d(list(boxes))
        self.grid = local_map.Grid.around((0.0, 0.0), P4C["geofence"], P4C["perception"]["grid_margin_m"],
                                          P4C["perception"]["grid_resolution_m"])
        self.sim, self.wall, self.tick_count = 40.0, 40.0, 0
        m = self.m
        m.origin, m.initial_yaw, m.yaw_target = (0.0, 0.0, 0.0), YAW0_NED, YAW0_NED
        m.target = (0.0, 0.0, -2.0)
        m.handover = {"completed_sim_s": 30.0}
        m.start_sim, m.state, m.enter_sim, m.enter_wall = 1.0, state, self.sim, self.wall
        m.positions = [(0.0, 0.0)]
        m.on_adapter_health({"status": "fresh", "calibration_identity": "cal", "sim_s": 1e9})
        self.publish_state()
        self.perceive()

    def publish_state(self, landed=False, **status):
        n, e, d = self.m.target
        yaw = self.m.yaw_target
        stamp = int(self.sim * 1e6)
        self.cache.clock(self.sim, self.wall)
        messages = {
            "vehicle_local_position": {"timestamp": stamp, "x": n, "y": e, "z": d, "vx": 0.0, "vy": 0.0, "vz": 0.0,
                                       "xy_valid": True, "z_valid": True, "v_xy_valid": True, "v_z_valid": True,
                                       "heading_good_for_control": True, "xy_reset_counter": 0, "z_reset_counter": 0,
                                       "heading_reset_counter": 0, "vxy_reset_counter": 0, "vz_reset_counter": 0,
                                       "ref_timestamp": 1000000},
            "vehicle_attitude": {"timestamp": stamp, "q": [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)],
                                 "quat_reset_counter": 0},
            "vehicle_status": {"timestamp": stamp, "arming_state": 2, "nav_state": 14, "system_id": 72,
                               "component_id": 1, "failsafe": False, "pre_flight_checks_pass": True, **status},
            "vehicle_land_detected": {"timestamp": stamp, "landed": landed},
            "estimator_status_flags": {"timestamp": stamp, "cs_tilt_align": True, "cs_yaw_align": True},
            "failsafe_flags": {"timestamp": stamp, "local_position_invalid": False, "local_altitude_invalid": False,
                               "attitude_invalid": False, "angular_velocity_invalid": False}}
        for name, data in messages.items():
            self.cache.update(name, data, self.wall)

    def perceive(self):
        n, e, d = self.m.target
        vehicle = (e, n, -d)
        yaw_enu = p4_mission.yaw_enu_from_ned(self.m.yaw_target)
        position, q = syn.px4_pose(vehicle, yaw_enu)
        image = syn.render(vehicle, yaw_enu, self.scene, columns=COLUMNS)
        camera_xy, rays = perception.extract_scan(image, syn.CAL, EXTRINSIC, position, q, P4C["perception"],
                                                  SENSORS["near_m"], SENSORS["far_m"], 0.0)
        perception.integrate(self.grid, camera_xy, rays, P4C["perception"]["max_hit_range_m"], SENSORS["near_m"])
        self.m.on_snapshot({"usable": True, "grid": self.grid.encode(), "acquisition_sim_s": self.sim,
                            "calibration_identity": "cal"})

    def step(self):
        self.sim += 0.05
        self.wall += 0.05
        self.tick_count += 1
        self.publish_state()
        if self.tick_count % 2 == 0:
            self.perceive()
        return self.m.tick(self.sim, self.wall, True)

    def run_until(self, predicate, limit_s):
        end = self.sim + limit_s
        while self.sim < end:
            actions = self.step()
            if predicate(actions):
                return actions
        raise AssertionError("condition not reached in %.1f s (state %s)" % (limit_s, self.m.state))


def p5_mission(policy, **changes):
    return lambda cache: mission.P5Mission(P2, P4C, dict(P5C, **changes), policy, p5.SCHEMA, cache, 0.0, True)


def p4_mission_factory(cache):
    return p4_mission.P4Mission(P2, P4C, cache, 0.0, True)


def decisions(m):
    return [e for e in m.events if e["event"] == "p4_decision"]


# ---- structure ----------------------------------------------------------------------------------------
def test_p5_mission_is_the_p4_mission_with_only_the_transit_supervisor_replaced():
    m = p5_mission(p5.interface_policy())(timing.StateCache(P2))
    assert isinstance(m, p4_mission.P4Mission) and m.order == p4_mission.P4_ORDER
    assert m.config == p4_mission.p4_p2_config(P2, P4C)
    assert m.ranker.failure_wall_s == 0.020 and m.failures_to_abort == 3
    overridden = {name for name, value in vars(mission.P5Mission).items() if callable(value)}
    assert overridden == {"__init__", "transition", "_transit", "_record", "policy_summary"}
    assert mission.LedgerSupervisor.__mro__[1] is safety_shield.PolicySupervisor
    assert {name for name, value in vars(mission.LedgerSupervisor).items() if callable(value)} == {"step"}


def test_p5_mission_with_the_p4_ranker_equals_the_p4_mission_tick_for_tick():
    """Mission parity: with P4's own selection plugged in, P5Mission reproduces P4Mission exactly."""
    boxes = held_out_layout()["boxes"] if ARTIFACT else [{"center": [4.0, 0.6], "size": [0.4, 0.4]}]
    rig4, rig5 = Rig(p4_mission_factory, boxes), Rig(p5_mission(None), boxes)
    rig5.m.ranker = candidate_ranker.P4Ranker()
    for _ in range(int(130 / 0.05)):
        assert rig5.step() == rig4.step()
        assert rig5.m.state == rig4.m.state
        if rig4.m.state not in ("MAP_SCAN", "TRANSIT"):
            break
    assert rig4.m.state == "GOAL_HOVER"
    assert rig5.m.events == rig4.m.events and rig5.m.transitions == rig4.m.transitions
    assert isinstance(rig5.m.supervisor, mission.LedgerSupervisor)
    assert rig5.m.supervisor.transit_start == rig4.m.supervisor.transit_start
    assert rig5.m.ledger_ticks == len(decisions(rig4.m)) and not any(r["invoked"] for r in rig5.m.ledger)


# ---- the trained policy in the mission -----------------------------------------------------------------
@needs_artifact
def test_trained_policy_flies_a_held_out_layout_and_ledgers_every_transit_tick():
    policy = trained_policy()
    layout = held_out_layout()
    rig = Rig(p5_mission(policy), layout["boxes"])
    rig.run_until(lambda a: rig.m.state == "TRANSIT", 45.0)
    transit_start = rig.m.supervisor.transit_start
    rig.run_until(lambda a: rig.m.state == "GOAL_HOVER", 100.0)
    m = rig.m
    assert isinstance(m.supervisor, mission.LedgerSupervisor) and m.supervisor.transit_start == transit_start
    ledger, events = m.ledger, decisions(m)
    # Complete: one record per TRANSIT supervisor step, in order, at the decision's simulation time.
    assert len(ledger) == len(events) == m.ledger_ticks and [r["tick"] for r in ledger] == list(range(1, len(ledger) + 1))
    assert all(r["sim_s"] == e["sim_s"] and r["action"] == e["action"] and r["state"] == e["state"]
               for r, e in zip(ledger, events))
    invoked = [r for r in ledger if r["invoked"]]
    assert len(invoked) > 0.9 * len(ledger)  # D3: the policy ranks in every planning state, NORMAL included
    by_kind = {}
    for record in invoked:
        decoded = json.loads(strict_json(record))  # what the node persists
        features = policy_ledger.decode_array(decoded["features"])
        assert features.shape == (26, 46) and np.array_equal(features, policy_ledger.decode_array(record["features"]))
        scores = policy.scores(features)  # the evaluator's reproduction: bit-identical
        assert decoded["scores"] == [float(s) for s in scores]
        assert decoded["failure"] is None and 0.0 <= decoded["inference_s"] < 0.020
        assert decoded["policy_sha256"] == policy.sha256
        allowed = np.array(decoded["allowed"])
        masked = np.where(allowed, scores, -np.inf)
        assert allowed[decoded["choice_slot"]] and decoded["choice_slot"] == int(np.argmax(masked))
        best = decoded["best"]
        assert best["k"] == decoded["choice_slot"] // 2 - 6
        kind = decoded["intervention"]
        by_kind[kind] = by_kind.get(kind, 0) + 1
        if kind is None:
            assert decoded["action"] == "TRANSLATE" and decoded["end"] == best["end"]
        elif kind == "yaw_to_l4":
            assert decoded["action"] == "YAW_TO" and decoded["heading"] == best["heading"]
        else:
            assert kind.startswith("hold:yaw_not_permitted") and decoded["action"] == "HOLD"
    assert by_kind.get(None, 0) > 0
    summary = m.policy_summary()
    assert summary["invocations"] == len(invoked) and summary["failures"] == {} and summary["ledger_records"] == len(ledger)
    assert summary["disagreements"] == sum(r["choice_slot"] != r["p4_slot"] for r in invoked)


class Failing:
    sha256 = "failing-test-policy"

    def scores(self, features):
        raise RuntimeError("injected")


def test_policy_failures_in_transit_hold_then_abort_and_land_through_revision_5():
    rig = Rig(p5_mission(Failing()))
    rig.run_until(lambda a: rig.m.state == "TRANSIT", 45.0)
    held = rig.m.target, rig.m.yaw_target
    actions = rig.run_until(lambda a: a["command"] is not None, 1.0)
    m = rig.m
    assert actions["command"] == {"command": 21, "params": [0.0] * 7}
    assert m.state == "REQUEST_LAND" and m.failure is None and m.p4_abort == "p4_supervisor_abort:policy_unavailable"
    assert (m.target, m.yaw_target) == held  # HOLD on every failing tick: target and yaw frozen
    assert [r["failure"] for r in m.ledger] == ["exception:RuntimeError"] * 3
    assert [r["intervention"] for r in m.ledger] == ["policy_failure_hold"] * 3
    assert [r["state"] for r in m.ledger] == [supervisor.NORMAL, supervisor.NORMAL, supervisor.ABORT]
    assert [r["action"] for r in m.ledger] == ["HOLD"] * 3 and all(r["scores"] is None for r in m.ledger)
    assert m.policy_summary()["failures"] == {"exception:RuntimeError": 3}


def test_ledger_arrays_round_trip_exactly_and_interventions_are_classified():
    value = np.random.default_rng(1).normal(size=(26, 46)) * 1e3
    encoded = json.loads(json.dumps(policy_ledger.encode_array(value)))
    assert np.array_equal(policy_ledger.decode_array(encoded), value)
    with pytest.raises(ValueError, match="ledger_dtype"):
        policy_ledger.decode_array(dict(encoded, dtype="<f4"))
    best = {"heading": 0.2, "end": (1.0, 0.2)}
    out = {"action": "TRANSLATE", "heading": 0.2, "end": (1.0, 0.2), "reason": None, "plan": {"best": best}}
    ok = {"failure": None}
    assert policy_ledger.intervention(out, None) is None and policy_ledger.intervention(out, ok) is None
    assert policy_ledger.intervention(dict(out, action="YAW_TO", end=None), ok) == "yaw_to_l4"
    assert policy_ledger.intervention(dict(out, action="HOLD", reason="yaw_not_permitted:x"), ok) == "hold:yaw_not_permitted:x"
    assert policy_ledger.intervention(out, {"failure": "timeout"}) == "policy_failure_hold"
    assert policy_ledger.intervention(dict(out, end=(0.9, 0.2)), ok) == "overridden:TRANSLATE"


# ---- p5_control start-up and drift guards --------------------------------------------------------------
def node_args(tmp_path, model, model_sha, schema_sha=p5.SCHEMA_SHA, config_sha=None, enabled=True):
    p4 = dict(P4C, enabled=True)
    p5c = json.loads(json.dumps(P5C))
    p5c.update(enabled=enabled)
    p5c["model"].update(path=str(model), sha256=config_sha if config_sha is not None else model_sha)
    (tmp_path / "p4_avoidance.yaml").write_text(json.dumps(p4))
    (tmp_path / "p5_policy.yaml").write_text(json.dumps(p5c))
    (tmp_path / "marker").write_text("{}")
    return ["--config", str(syn.SIM / "configs/p2_control.yaml"), "--timing-config", str(syn.SIM / "configs/p2_timing.yaml"),
            "--p4-config", str(tmp_path / "p4_avoidance.yaml"), "--p5-config", str(tmp_path / "p5_policy.yaml"),
            "--schema", str(p5.P5 / "configs/p5_model_schema.json"), "--schema-sha256", schema_sha,
            "--model", str(model), "--model-sha256", model_sha, "--run-dir", str(tmp_path),
            "--authorization-marker", str(tmp_path / "marker"), "--ground-diagnostic"]


def open_gates(monkeypatch):
    for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH", "GWM_ALLOW_SITL_COMMANDS",
                "GWM_P4_OWNED_NAMESPACE"):
        monkeypatch.setenv(key, "1")
    monkeypatch.setitem(sys.modules, "rclpy", None)  # any refusal must happen before ROS is imported


def test_control_node_is_importable_without_ros_and_refuses_without_gates(tmp_path, monkeypatch):
    for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH"):
        monkeypatch.delenv(key, raising=False)
    with pytest.raises(ValueError, match="explicit gates"):
        control_node.main(node_args(tmp_path, tmp_path / "none.npz", "0" * 64))
    assert control_node.EVENT_TOPIC == p4_node.EVENT_TOPIC == "/gwm/p2/events" and control_node.CONTROLLER == "p5_control"


@pytest.mark.parametrize("case,error,match", [
    ("config_sha_differs", ValueError, "p5_checkpoint_sha256_differs_from_run_configuration"),
    ("schema_hash_differs", ValueError, "p5_model_schema_differs_from_frozen_hash"),
    ("file_differs", PolicyError, "checkpoint_sha256_mismatch"),
    ("missing", PolicyError, "checkpoint_missing"),
    ("untrained", PolicyError, "untrained_model_refused"),
    ("disabled", PermissionError, "P5 is disabled")])
def test_p5_control_refuses_to_start_on_any_checkpoint_mismatch(tmp_path, monkeypatch, case, error, match):
    open_gates(monkeypatch)
    model = tmp_path / "model.npz"
    sha = save_model(model, p5.interface_arrays(), p5.interface_metadata(trained=case != "untrained"))
    kwargs = {"config_sha_differs": {"config_sha": "f" * 64}, "schema_hash_differs": {"schema_sha": "0" * 64},
              "disabled": {"enabled": False}}.get(case, {})
    if case == "file_differs":
        model.write_bytes(model.read_bytes() + b"\0")
    if case == "missing":
        model.unlink()
    with pytest.raises(error, match=match):
        control_node.main(node_args(tmp_path, model, sha, **kwargs))
    assert not (tmp_path / "p5").exists() and not (tmp_path / "ros-events.jsonl").exists()


@needs_artifact
def test_p5_control_accepts_the_exported_checkpoint_up_to_ros_import(tmp_path, monkeypatch):
    open_gates(monkeypatch)
    with pytest.raises(ImportError):  # every start-up check passed; the next step is importing ROS
        control_node.main(node_args(tmp_path, ARTIFACT, ARTIFACT.stem[len("p5-ranker-"):]))


def test_p5_control_differs_from_p4_control_only_where_declared():
    """Source-level drift guard (decision D5): the node is the P4 node except for the declared lines."""
    def body(func):
        return [line.strip() for line in inspect.getsource(func).splitlines()[1:]
                if line.strip() and not line.strip().startswith(("#", '"""'))]
    p4, p5_lines = body(p4_node.main), body(control_node.main)
    removed = [line for line in p4 if line not in p5_lines]
    added = [line for line in p5_lines if line not in p4]
    assert removed == [
        'super().__init__("p4_control", namespace="/gwm",',
        'self.mission = P4Mission(c, p4, self.cache, self.start_wall, active,',
        'self.emit({"event": "startup", "controller": "p4_control", "flight_commands_enabled": args.flight,',
        '"controller": "p4_control", "p4_outcome": outcome(m, args.flight), "p4": m.summary(),',
        'for stream in (self.record, self.decisions):']
    assert added == [
        'parser.add_argument("--p5-config", required=True, type=Path, help="run-local p5_policy.yaml")',
        'parser.add_argument("--schema", required=True, type=Path, help="frozen p5_model_schema.json")',
        'parser.add_argument("--schema-sha256", required=True, help="schema hash bound in the P5 manifest")',
        'parser.add_argument("--model", required=True, type=Path, help="content-addressed p5-ranker-<sha256>.npz")',
        'parser.add_argument("--model-sha256", required=True, help="checkpoint SHA-256 bound in the P5 manifest")',
        'p5 = load_policy_config(args.p5_config)',
        'require_p5_enabled(p5, args.authorization_marker.is_file())',
        'policy, schema = verified_policy(p5, args)',
        '(args.run_dir / "p5").mkdir(exist_ok=True)',
        'super().__init__(CONTROLLER, namespace="/gwm",',
        'self.ledger = (self.run / "p5/policy-ledger.jsonl").open("x", buffering=1)',
        'self.mission = P5Mission(c, p4, p5, policy, schema, self.cache, self.start_wall, active,',
        'self.emit({"event": "startup", "controller": CONTROLLER, "flight_commands_enabled": args.flight,',
        'self.emit({"event": "p5_policy_verified", **self.policy_identity()})',
        'def policy_identity(self):',
        'return {"model_path": str(args.model), "model_sha256": policy.sha256,',
        '"architecture": policy.metadata["architecture"], "schema_sha256": args.schema_sha256,',
        '"trained": policy.metadata["trained"], "numpy": numpy.__version__}',
        'def write_ledger(self):',
        'if not self.mission.ledger:',
        'return',
        'with self.trace.span("policy_ledger_write", self.sim(), records=len(self.mission.ledger)):',
        'while self.mission.ledger:',
        'self.ledger.write(strict_json(self.mission.ledger.pop(0)) + "\\n")',
        'self.evidence_error = "policy_ledger_write_failed"',
        'self.write_ledger()',
        'self.write_ledger()',
        '"controller": CONTROLLER, "p4_outcome": outcome(m, args.flight), "p4": m.summary(),',
        '"p5": {**self.policy_identity(), **m.policy_summary()},',
        'for stream in (self.record, self.decisions, self.ledger):']
    assert control_node.outcome is p4_node.outcome


def test_policy_modules_add_no_publication_path():
    package = p5.P5 / "ros2_ws/src/gwm_policy_control/gwm_policy_control"
    for name in ("mission", "policy_ledger"):
        text = (package / (name + ".py")).read_text(encoding="utf-8")
        assert "rclpy" not in text and "create_publisher" not in text and "px4_msgs" not in text, name
    node, p4_text = (package / "control_node.py").read_text(encoding="utf-8"), inspect.getsource(p4_node)
    assert node.count("create_publisher") == p4_text.count("create_publisher") == 2
    assert node.count(".publish(") == p4_text.count(".publish(") == 2
