"""P6 runtime supervisor: the accepted P5 runner with the declared P6 injectors (contract §2, C7, C8, §7, §8.1).

The P5 `P5Runner` is imported from its own file and subclassed. Through it the accepted P4 and P3 runners
launch the identical PX4/Gazebo/QGC/DDS/sensor/perception/controller stack with the same isolation (private
namespace, loopback only, exclusive workspace lock), ordered shutdown and finalization. P6 overrides only:

* preconditions: the P6 manifest (which re-verifies the P3, P4 and P5 manifests), then either a P6 gate
  authorization with its consumption marker, or, for gate `DRY`, a dry-run approval of a development
  candidate (plan §7 M5: no gate result, no acceptance credit);
* the row: one frozen scenario row. Its declaration `p6/fault-declaration.json` is built from the frozen
  specification before anything starts, and every injector refuses any other declaration;
* run-local inputs (rule C11): for the depth relay (DR) only the two declared `ros_topic_name` keys of the
  bridge configuration; for the start-up refusals (RL) only the declared start-up input;
* launching: the witness and each declared relay start between the accepted source and its consumers
  (DR before the adapter; SR before perception, which gets its existing `--px4-prefix /gwm/p6/px4_71`).
  With CS the controller is the accepted `p5_control` under the controller shim;
* the trigger and the injection (process signal, relay activation by SIGUSR1, competing publisher), each
  recorded in `p6/injection.json` and in the injector's own ledger. A bridge or adapter stop also writes
  the accepted P4 record `p4/fault_injection.json`, through the accepted P4 `inject`;
* the start-up refusals (F3.01–F3.03) through the accepted P5 policy_fault path;
* shutdown order: each relay stops after its consumers, the witness after the adapter; a stopped process
  is always resumed first;
* run naming (`…-p6-<row>-…`, dry runs `…-p6dry-<row>-…`) and the summary's `p6` block.

Flight rows (Gates B, S and C, M6): the accepted P5 flight on the row's world (`--scenario`), under the accepted
flight gating (GWM_ALLOW_SITL_COMMANDS=1 and --allow-simulated-flight). The runner feeds the controller's own
events to the frozen trigger and fires its mechanisms once (relay SIGUSR1, the accepted P4 bridge SIGINT, the
load harness); CS and MR fire inside the shim. The accepted flight window (PRESTREAM to COMPLETE or RECOVERY) is
written before a fail-closed exit is reported, so the unchanged P5 evaluation runs and the row's rules decide.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid

P6 = Path(__file__).resolve().parents[1]
REPO = P6.parents[1]
P5 = REPO / "simulation/px4_gazebo_p5"
_spec = importlib.util.spec_from_file_location("p5_runner_accepted", P5 / "scripts/p5_runner.py")
p5r = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(p5r)
base, p4 = p5r.base, p5r.p4
for path in (P6 / "scripts", P6):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import p6_manifest  # noqa: E402
from injection import declaration as declaration_module  # noqa: E402
from injection import process_faults, schedule, specs  # noqa: E402
from injection.controller_shim import shim_mode  # noqa: E402
from injection.fault_ledger import FaultLedger  # noqa: E402
from p3_provenance import canonical_sha256, digest, strict_json  # noqa: E402

OWNER = P6 / "scripts/p6_namespace_owner.py"
p5r.OWNER = OWNER  # the QGC ownership receipt names the owner copy actually running as PID 1 (same bytes)
NODE = P6 / "scripts/p6_node.py"
PYTHON = "/usr/bin/python3"
# P6 row scope -> P5 scope that drives the unchanged P5/P4/P3 stack
P5_SCOPE = {"p3_readiness": "p3_readiness", "ground_readiness": "p5_readiness", "ground_fixture": "p5_readiness",
            "startup_refusal": "policy_fault", "flight": "flight"}
FLIGHT_SCOPE = "flight"  # Gates B, S and C (M6): the accepted P5 flight path on the row's world, with the P6 injectors
RUNNER_MECHANISMS = ("DR", "SR", "PS", "LH")  # fired by the runner at the trigger; CS and MR fire inside the shim
LOAD_TRIGGER = "load_start"  # the parameter naming the load harness's own trigger (C-5)
ACCEPTED_DEPTH, RAW_DEPTH = "/gwm/sensors/front_depth", "/gwm/p6/raw/front_depth"
STATE_PREFIX = "/gwm/p6/px4_71"
REFUSALS = {"F3.01": "checkpoint_missing", "F3.02": "checkpoint_sha256_mismatch",
            "F3.03": "p5_model_schema_differs_from_frozen_hash"}
# F4.13 (M5 decision, 2026-11-13): from PRESTREAM on the accepted state check aborts at once, so a stopped /clock is
# reported as clock_epoch_mismatch:<state topic> before the 1.0 s clock_stalled check; the proof decides which is valid.
LIVE_DETECTION = {"F4.07": ("competing_control_publisher",), "F4.13": ("clock_stalled", "clock_epoch_mismatch:")}
LIVE_TRIGGER = "ground_prestream"  # only if the frozen specification declares it (see live_trigger)
PRESTREAM = "PRESTREAM_SAFE_SETPOINTS"
OBSERVE_SIM_S = 5.0  # after the injection: the accepted P4 ground-fault observation, or the declared window + margin
OBSERVE_MARGIN_SIM_S = 3.0


# ---- run-local inputs (C11), pure ---------------------------------------------------------------------------
def relay_bridge_config(text):
    """DR: only the two declared keys change. Each depth `ros_topic_name` moves to the raw topic, so the relay
    alone publishes the accepted names; every other byte of the bridge configuration stays as frozen."""
    changes = []
    for leaf in ("image_raw", "camera_info"):
        before = "- ros_topic_name: %s/%s\n" % (ACCEPTED_DEPTH, leaf)
        if text.count(before) != 1:
            raise ValueError("bridge configuration key is not unique: " + leaf)
        text = text.replace(before, "- ros_topic_name: %s/%s\n" % (RAW_DEPTH, leaf))
        changes.append({"file": "p3_sensor_bridge.yaml", "key": "[%s].ros_topic_name" % leaf,
                        "before": "%s/%s" % (ACCEPTED_DEPTH, leaf), "after": "%s/%s" % (RAW_DEPTH, leaf)})
    return text, changes


def changed_schema(text):
    """F3.03: the run-local schema with one changed description (as the M3 offline test), nothing else."""
    schema = json.loads(text)
    before = schema["candidate_features"][0].get("description")
    schema["candidate_features"][0]["description"] = (before or "") + " "
    change = {"file": "p5_model_schema.json", "key": "candidate_features[0].description", "before": before,
              "after": schema["candidate_features"][0]["description"]}
    return json.dumps(schema, indent=2) + "\n", change, canonical_sha256(schema)


def observation_end(declared, t_inj):
    """Observe for the accepted P4 5 s, or the longest declared fault window plus a margin."""
    windows = [v for k, v in declared["parameters"].items() if k.endswith("window_sim_s")]
    return t_inj + max([OBSERVE_SIM_S] + [w + OBSERVE_MARGIN_SIM_S for w in windows])


def live_trigger_due(rows):
    """`ground_prestream`: the ground-diagnostic controller's transition into PRESTREAM_SAFE_SETPOINTS (its
    ros-events.jsonl). Its estimate has then been valid for the accepted P2 stability time, connectivity has passed,
    every PX4 stream is in its ledger, and it holds the state for its accepted ground duration (M5 decision)."""
    return any(r.get("event") == "transition" and r.get("to") == PRESTREAM for r in rows)


def uses(declared, mechanism):
    return declaration_module.uses(declared, mechanism)


# ---- flight rows (M6), pure --------------------------------------------------------------------------------------
def runner_mechanisms(declared):
    """The declared mechanisms the runner itself fires at the trigger (the shim fires CS and MR in-process)."""
    return tuple(m for m in declared["mechanisms"] if m in RUNNER_MECHANISMS)


def flight_triggers(declared, boxes):
    """The fault trigger (for the runner-fired mechanisms other than the load harness) and the load trigger (C-5),
    each a frozen `schedule.Trigger` fed with the controller's own events, with the P4 scenario's boxes."""
    if declared["mode"] != "active":  # pass-through rows (B-0): the injectors are present and nothing fires
        return None, None
    fired = runner_mechanisms(declared)
    fault = schedule.Trigger(declared["trigger"], boxes=boxes) if any(m != "LH" for m in fired) else None
    load = schedule.Trigger(declared["parameters"][LOAD_TRIGGER], boxes=boxes) if "LH" in fired else None
    return fault, load


def scenario_boxes(scenarios, world):
    """The frozen P4 scenario's obstacle boxes (`center`, `size`, the frame of `p4_scenarios.json`), which the
    triggers and the offline rehearsals use. The generated world declaration records them as `center_m` with a
    height, a different shape, so the declaration is never passed to a trigger."""
    return [dict(box) for box in scenarios["scenarios"][world]["boxes"]]


def flight_window(result):
    """The accepted P4/P5 flight sensor window: PRESTREAM to COMPLETE, or to RECOVERY for a fail-closed landing."""
    transitions = {t["to"]: t["sim_s"] for t in (result or {}).get("transitions", [])}
    end = transitions.get("COMPLETE") or transitions.get("RECOVERY")
    begin = transitions.get("PRESTREAM_SAFE_SETPOINTS")
    if begin is None or end is None:
        return None
    return [begin, end]


class P6Runner(p5r.P5Runner):
    def __init__(self, args):
        self.row_gate, self.row = specs.row(args.row)
        self.p6_scope = self.row["scope"]
        if self.p6_scope not in P5_SCOPE:
            raise ValueError("Row %s (%s) is not a live row of this runner" % (args.row, self.p6_scope))
        args.scope = P5_SCOPE[self.p6_scope]
        if self.p6_scope == FLIGHT_SCOPE:
            args.scenario = self.row["world"]  # the accepted P4 scenario world (S1-S4), generated by the P4 runner
        self.load_trigger = self.fault_trigger = None
        super().__init__(args)  # P5 -> P4 -> P3 scope mapping, unchanged
        self.declared = declaration_module.build(args.row)
        self.dry = args.gate == p6_manifest.DRY_RUN
        self.p6_injection = None
        self.signal_thread = None
        self.witness_started = False
        self.run_local_changes = []

    # ---- preconditions ----------------------------------------------------------------------
    def preconditions(self):
        missing = [key for key in base.GATES if os.environ.get(key) != "1"]
        flight = self.p6_scope == FLIGHT_SCOPE
        if flight != (self.row_gate in p6_manifest.FLIGHT_GATES):
            raise PermissionError("Row %s: a flight row belongs to Gate B, S or C; a ground row to Gate A" % self.args.row)
        if flight and (os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1" or not self.args.allow_simulated_flight):
            raise PermissionError("A flight row needs GWM_ALLOW_SITL_COMMANDS=1 and --allow-simulated-flight (M6)")
        if not flight and self.args.allow_simulated_flight:
            raise PermissionError("Flight authorization is only valid for a flight row")
        if missing:
            raise ValueError("Missing explicit gates: " + ", ".join(missing))
        if os.getpid() != 1:
            raise ValueError("Owned private PID/network namespace required")
        interfaces = json.loads(subprocess.check_output(["ip", "-j", "link"], text=True))
        if {item["ifname"] for item in interfaces} != {"lo"}:
            raise ValueError("Owned private PID/network namespace required")
        lock = Path("/proc/1/fd/9").resolve()
        probe = os.open(lock, os.O_RDWR)
        try:
            base.fcntl.flock(probe, base.fcntl.LOCK_EX | base.fcntl.LOCK_NB)
            raise ValueError("Exclusive P1-P6 workspace lock is not held by this owner")
        except BlockingIOError:
            pass
        finally:
            os.close(probe)
        if shutil.disk_usage(self.root).free < self.validation["disk_reserve_bytes"]:
            raise ValueError("Disk reserve below the required 20 GiB")
        frozen, sha = p6_manifest.verify_current(self.root, self.args.manifest)  # includes P3, P4 and P5 (C1)
        if self.dry:
            authorization = p6_manifest.require_dry_run_approval(self.args.authorization, sha, frozen, self.row_gate)
            self.marker = None  # a run-local marker is written in prepare_run (the P4/P5 enable checks need a file)
            approval = digest(self.args.authorization)
            done = [p for p in (self.root / "runs").glob("*-p6dry-*")
                    if (p / "summary.json").exists()
                    and (strict_json(p / "summary.json").get("p6") or {}).get("dry_run_approval_sha256") == approval]
            if len(done) >= authorization["live_runs_max"]:
                raise PermissionError("Dry-run cap reached (%d/%d)" % (len(done), authorization["live_runs_max"]))
        else:
            if frozen["development"]:
                raise PermissionError("A development candidate never runs a gate")
            if self.args.gate != self.row_gate:
                raise PermissionError("Row %s belongs to Gate %s" % (self.args.row, self.row_gate))
            authorization = p6_manifest.require_authorization(self.args.authorization, self.args.gate, sha)
            marker = p6_manifest.marker_path(self.root, self.args.gate, sha)
            if not marker.exists() or strict_json(marker)["authorization_sha256"] != digest(self.args.authorization):
                raise PermissionError("Gate authorization was not consumed by the P6 gate sequencer")
            self.marker = marker
        if not base.p3_manifest.wsl_host():
            raise PermissionError("P6 runs only on the WSL D3D12 host")
        self.p6_frozen = frozen
        return p6_manifest.runner_view(frozen), sha, authorization

    # ---- run directory and run-local inputs -------------------------------------------------------------------
    def prepare_run(self, frozen):
        label = ("-p6dry-" if self.dry else "-p6-") + self.args.row.replace("'", "p")
        self.run = self.root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + label + "-" + uuid.uuid4().hex[:8])
        self.run.mkdir(parents=True)
        for name in ("tmp", "p4", "p5", "p6"):
            (self.run / name).mkdir()
        for name in ("p3_sensors.yaml", "p3_validation.yaml", "p3_sensor_bridge.yaml", "p3_fastdds.xml",
                     "p3_native_command_policy.json"):
            shutil.copyfile(p5r.SIM / "configs" / name, self.run / name)
        for name in ("p4_validation.yaml", "p4_scenarios.json"):
            shutil.copyfile(p5r.P4 / "configs" / name, self.run / name)
        for name in ("p5_validation.yaml", "p5_model_schema.json"):
            shutil.copyfile(P5 / "configs" / name, self.run / name)
        local = strict_json(p5r.P4 / "configs/p4_avoidance.yaml")
        local["enabled"] = True  # P4 contract C4, as the accepted runners
        (self.run / "p4_avoidance.yaml").write_text(json.dumps(local, indent=2) + "\n")
        (self.run / "p5_policy.yaml").write_text(json.dumps(p5r.run_local_policy_config(frozen), indent=2) + "\n")
        declaration_module.write(self.run / "p6/fault-declaration.json", self.declared)
        if uses(self.declared, "DR") != "off":
            text, changes = relay_bridge_config((self.run / "p3_sensor_bridge.yaml").read_text())
            (self.run / "p3_sensor_bridge.yaml").write_text(text)
            self.run_local_changes += changes
        if self.p6_scope == "startup_refusal":
            self.policy_fault = self.refusal_fixture(frozen)
        if self.dry:
            self.marker = self.run / "p6/dry-run-marker.json"
            self.marker.write_text(json.dumps({"contract": "p6-dry-run-marker-v1", "approval_sha256": digest(self.args.authorization),
                                               "row": self.args.row, "acceptance_credit": False}) + "\n")

    def refusal_fixture(self, frozen):
        """F3.01 absent checkpoint, F3.02 the P5 mismatched copy, F3.03 the changed run-local schema."""
        fault, model = self.declared["fault"], frozen["model"]
        if fault == "F3.02":
            record = p5r.mismatched_checkpoint(frozen, self.run)  # the accepted P5 Gate A pattern
        elif fault == "F3.01":
            record = {"contract": "p6-startup-refusal-v1", "kind": "checkpoint_missing",
                      "model_path": str(self.run / "p6/absent-checkpoint.npz"), "expected_sha256": model["sha256"],
                      "actual_sha256": None}
        elif fault == "F3.03":
            text, change, run_local = changed_schema((self.run / "p5_model_schema.json").read_text())
            (self.run / "p5_model_schema.json").write_text(text)
            self.run_local_changes.append(change)
            record = {"contract": "p6-startup-refusal-v1", "kind": "schema_mismatch", "model_path": str(P5 / model["path"]),
                      "expected_sha256": model["sha256"], "actual_sha256": model["sha256"],
                      "schema_sha256_frozen": frozen["schema_sha256"], "schema_sha256_run_local": run_local}
        else:
            raise ValueError("no start-up refusal fixture for " + str(fault))
        return dict(record, fault=fault, expected_refusal=REFUSALS[fault])

    # ---- launching --------------------------------------------------------------------------
    def ledger(self, injector):
        return self.run / "p6" / ("fault-ledger-%s.jsonl" % injector)

    def node(self, module, *arguments):
        return [PYTHON, "-s", "-P", "-B", str(NODE), module, *arguments]

    def launch(self, name, command, env, stop_method="SIGINT", grace=None, cwd=None):
        if name == "sensor-adapter" and uses(self.declared, "DR") != "off":
            self.start_witness(env)
            self.launch("p6-depth-relay", self.node("depth_relay", "--declaration", str(self.run / "p6/fault-declaration.json"),
                                                    "--ledger", str(self.ledger("depth_relay"))), env)
        if name == "p4-perception" and uses(self.declared, "SR") != "off":
            self.start_witness(env)
            self.launch("p6-state-relay", self.node("state_relay", "--declaration", str(self.run / "p6/fault-declaration.json"),
                                                    "--ledger", str(self.ledger("state_relay"))), env)
            command = list(command) + ["--px4-prefix", STATE_PREFIX]
        return super().launch(name, command, env, stop_method, grace, cwd)

    def start_witness(self, env):
        if self.witness_started:
            return
        self.witness_started = True
        flags = (["--depth"] if uses(self.declared, "DR") != "off" else []) + (["--state"] if uses(self.declared, "SR") != "off" else [])
        self.launch("p6-witness", self.node("witness", "--run-dir", str(self.run), *flags), env)

    def p4_controller_command(self, env):
        """CS: the accepted `p5_control` arguments, run through the controller shim (diagnostic-runner pattern)."""
        command, env = super().p4_controller_command(env)
        if shim_mode(self.declared) == "off":
            return command, env
        if not command[0].endswith("/p5_control"):
            raise ValueError("unexpected controller command")
        return self.node("controller_shim", "--p6-declaration", str(self.run / "p6/fault-declaration.json"),
                         "--p6-ledger", str(self.ledger("controller_shim")), "--", *command[1:]), env

    # ---- controller phase, trigger and injection -------------------------------------------------------------
    def run_controller(self, config):
        if self.p6_scope == FLIGHT_SCOPE:
            return self.flight(config)
        if self.p6_scope != "ground_fixture":
            return super().run_controller(config)  # P5: p3_readiness, p5_readiness and policy_fault, unchanged
        trigger = self.declared["trigger"]
        if trigger == "ground_window":
            super().run_controller(config)  # the accepted P4 ground path: controller completes, ground window held
            self.summary["status"] = "incomplete"
            self.inject_fault()
            self.observe()
        elif trigger == LIVE_TRIGGER:
            self.live_trigger(config)
        else:
            raise base.Abort("trigger_not_available_on_the_ground:" + str(trigger))
        self.summary["status"] = "passed"

    def live_trigger(self, config):
        """A fault the running ground-diagnostic controller must detect: inject at its transition into
        PRESTREAM_SAFE_SETPOINTS (it is then running its graph and clock checks on a valid estimate), accept only the
        declared detection as its failure, then hold the unchanged ground window so the sensor checks still apply (the
        P5 policy_fault pattern); the window therefore begins after the estimator start-up."""
        deadline = self.start + config["wall_deadline_s"] + 90
        events = base.Tail(self.run / "ros-events.jsonl")
        while self.controller.poll() is None:
            self.pump()
            if self.p6_injection is None and live_trigger_due(events.read()):
                self.inject_fault()
            if time.monotonic() > deadline:
                raise base.Abort("deadline:controller")
            time.sleep(0.05)
        self.join_signal_thread()
        result_path = self.run / "controller-result.json"
        self.summary["controller_exit_code"] = self.controller.record["exit_code"]
        self.summary["controller_result"] = result = strict_json(result_path) if result_path.exists() else None
        if result is not None:
            self.summary["controller_publishers_only_from_controller"] = bool(result.get("graph_valid"))
        if self.p6_injection is None:
            raise base.Abort("trigger_never_fired")
        expected = LIVE_DETECTION.get(self.declared["fault"])
        failure = (result or {}).get("failure")
        self.summary["p6"]["declared_detection"] = {"expected": expected, "controller_failure": failure,
                                                    "controller_exit_code": self.summary["controller_exit_code"]}
        if expected is None or not str(failure).startswith(expected):
            raise base.Abort("declared_detection_not_observed:" + str(failure))
        window = self.p4_validation["ground_fixture"]["window_sim_s"]
        begin = self.wait_for(self.sim_now, deadline, "ground_window_clock")
        self.wait_for(lambda: (self.sim_now() or 0) >= begin + window, deadline, "ground_window")
        self.summary["sensor_window_sim_s"] = [begin, begin + window]

    def flight(self, config):
        """A Gate B, S or C row (M6): the accepted P5 flight on the row's world. The runner feeds the controller's own
        events to the frozen trigger and fires its mechanisms (relay SIGUSR1, the accepted P4 bridge SIGINT, the load
        harness) exactly once; the shim fires CS and MR in-process. The sensor window is the accepted flight window
        (PRESTREAM to COMPLETE or RECOVERY), written before a fail-closed exit is reported, so the unchanged P5
        evaluation still runs and the row's rules decide (contract §8.2-8.4)."""
        self.fault_trigger, self.load_trigger = flight_triggers(self.declared, scenario_boxes(self.scenarios, self.args.scenario))
        deadline = self.start + config["wall_deadline_s"] + 90
        events = base.Tail(self.run / "ros-events.jsonl")
        while self.controller.poll() is None:
            self.pump()
            rows = events.read()
            if rows:
                self.watch_flight_triggers(rows)
            if time.monotonic() > deadline:
                raise base.Abort("deadline:controller")
            time.sleep(0.05)
        self.join_signal_thread()
        result_path = self.run / "controller-result.json"
        self.summary["controller_exit_code"] = code = self.controller.record["exit_code"]
        self.summary["controller_result"] = result = strict_json(result_path) if result_path.exists() else None
        if result is not None:
            self.summary["controller_publishers_only_from_controller"] = bool(result.get("graph_valid"))
        window = flight_window(result)
        if window is None:
            raise base.Abort("controller_failed:" + str((result or {}).get("failure")))
        self.summary["sensor_window_sim_s"] = window
        self.summary["p6"]["flight"] = {"controller_exit_code": code, "controller_failure": (result or {}).get("failure"),
                                        "fault_trigger_fired_sim_s": self.fault_trigger and self.fault_trigger.fired_sim,
                                        "load_trigger_fired_sim_s": self.load_trigger and self.load_trigger.fired_sim}
        if code != 0:  # as the accepted P4 runner: the run fails; the row's rules may allow the designed fail-closed exit
            raise base.Abort("controller_failed:" + str(result.get("failure")))
        self.summary["status"] = "passed"

    def watch_flight_triggers(self, rows):
        for row in rows:
            if self.fault_trigger is not None and self.p6_injection is None:
                fired = self.fault_trigger.feed(row)
                if fired is not None:
                    self.inject_fault(trigger_event=row)
            if self.load_trigger is not None and self.load_trigger.fired_sim is None:
                fired = self.load_trigger.feed(row)
                if fired is not None:
                    self.launch_load(fired, row)

    def launch_load(self, fired_sim, row):
        p = self.declared["parameters"]
        item = self.launch("p6-load-harness", self.node("load_harness", "--workers", str(p["load_workers"]),
                                                        "--seconds", str(p["load_window_s"]), "--ledger", str(self.ledger("load_harness")),
                                                        "--row", self.declared["row"], "--fault", str(self.declared["fault"]),
                                                        "--sim-s", str(fired_sim)),
                           self.environment(), stop_method="completes_own_work")
        self.summary["p6"]["load"] = {"process": "p6-load-harness", "pid": item.pid, "trigger": p[LOAD_TRIGGER],
                                      "trigger_sim_s": fired_sim, "trigger_event_sim_s": row.get("sim_s"),
                                      "workers": p["load_workers"], "window_s": p["load_window_s"],
                                      "request_monotonic_s": time.monotonic()}
        self.save()

    def owned_process(self, name):
        item = next((o for o in self.owned if o.name == name), None)
        if item is None or item.poll() is not None:
            raise base.Abort("injection_target_not_running:" + name)
        return item

    def inject_fault(self, trigger_event=None):
        d = self.declared
        fault, p, mechanisms = d["fault"], d["parameters"], d["mechanisms"]
        record = {"contract": "p6-injection-v1", "row": d["row"], "fault": fault, "trigger": d["trigger"],
                  "mechanisms": mechanisms, "t_inj_sim_s": self.sim_now(), "request_monotonic_s": time.monotonic()}
        if trigger_event is not None:  # flight: the controller's own decision event that fired the frozen trigger
            record["trigger_event"] = {k: trigger_event.get(k) for k in ("event", "sim_s", "monotonic_s", "state", "vehicle_enu")}
        if "PS" in mechanisms and p.get("signal") == "SIGINT" and trigger_event is not None:
            record["vehicle_enu"] = trigger_event.get("vehicle_enu")  # the accepted P4 record keeps the position too
        if fault == "F4.07":
            item = self.launch("p6-competing-publisher", self.node("competing_publisher", "--declaration",
                                                                   str(self.run / "p6/fault-declaration.json"),
                                                                   "--ledger", str(self.ledger("competing_publisher"))),
                               self.environment(), stop_method="completes_own_work")
            record.update(process="p6-competing-publisher", pid=item.pid, action="advertise", advertise_s=p["advertise_s"])
        elif "PS" in mechanisms:
            name, kind = p["process"], p["signal"]
            item = self.owned_process(name)
            ledger = FaultLedger(self.ledger("process_signal"), d["row"], fault, "process_signal")
            ledger.record("declared", None, declaration=d)
            before = process_faults.proc_state(item.pid)
            if kind == "SIGINT":
                ned = None
                enu = record.get("vehicle_enu")
                if enu is not None:  # the accepted P4 record is NED: (north, east, down) from ENU (east, north, up)
                    ned = [float(enu[1]), float(enu[0]), -float(enu[2])] if len(enu) >= 3 else [float(enu[1]), float(enu[0]), 0.0]
                self.inject(name, vehicle_ned=ned)  # accepted P4: request_stop recorded, p4/fault_injection.json
                ledger.record("activated", record["t_inj_sim_s"], pid=item.pid, signal="SIGINT", state_before=before,
                              request_wall_s=item.record["stop_requested_monotonic_s"])
                ledger.close()
            elif kind == "SIGSTOP":
                def stop_and_resume():
                    try:
                        process_faults.inject(item.pid, "SIGSTOP", ledger, record["t_inj_sim_s"],
                                              resume_after_wall_s=p["resume_after_wall_s"])
                    finally:
                        ledger.close()
                self.signal_thread = threading.Thread(target=stop_and_resume, daemon=True)
                self.signal_thread.start()
            else:
                raise base.Abort("undeclared_process_signal:" + str(kind))
            record.update(process=name, pid=item.pid, signal=kind)
        elif "DR" in mechanisms or "SR" in mechanisms:
            name = "p6-depth-relay" if "DR" in mechanisms else "p6-state-relay"
            item = self.owned_process(name)
            os.kill(item.pid, signal.SIGUSR1)  # the relay activates on its next /clock message
            record.update(process=name, pid=item.pid, signal="SIGUSR1")
        else:
            raise base.Abort("no_ground_injection_for:" + str(fault))
        self.p6_injection = record
        base.atomic_json(self.run / "p6/injection.json", record, exclusive=True)
        self.summary["p6"]["injection"] = record
        self.save()

    def observe(self):
        begin = self.p6_injection["t_inj_sim_s"]
        until = observation_end(self.declared, begin)
        deadline = time.monotonic() + 4 * (until - begin) + 30  # bounded even if simulation runs slow
        self.wait_for(lambda: (self.sim_now() or 0) >= until, deadline, "fault_observation")
        self.join_signal_thread()
        self.summary["p6"]["observation_sim_s"] = [begin, until]

    def join_signal_thread(self):
        if self.signal_thread is not None:
            self.signal_thread.join(timeout=30)
            if self.signal_thread.is_alive():
                raise base.Abort("process_signal_not_completed")

    # ---- shutdown ---------------------------------------------------------------------------
    def stop(self, name, sig=signal.SIGINT):
        super().stop(name, sig)  # P4: perception before the bridge, truth after the clock bridge
        if name == "sensor-bridge":
            super().stop("p6-state-relay")  # its only consumer, perception, has stopped
            super().stop("p6-depth-relay")  # its source has stopped
        elif name == "sensor-adapter":
            super().stop("p6-witness")

    def shutdown(self):
        errors = []
        if self.signal_thread is not None:
            self.signal_thread.join(timeout=30)  # a stopped process is resumed before anything is stopped
            if self.signal_thread.is_alive():
                errors.append("process_signal_thread_still_running")
        for name in ("p6-competing-publisher", "p6-load-harness"):
            try:
                self.stop(name)
            except Exception as exc:
                errors.append("stop:%s:%s:%s" % (name, type(exc).__name__, exc))
        return errors + super().shutdown()

    # ---- entry ----------------------------------------------------------------------------------
    def execute(self):
        self.world_dir = self.declaration = None
        frozen, sha, authorization = self.preconditions()
        self.frozen = frozen
        self.prepare_run(frozen)
        print("P6 evidence: " + str(self.run), flush=True)
        v = self.p5_validation
        self.summary = {"schema_version": 1, "contract": "p6-runtime-summary-v1", "run_id": self.run.name,
                        "p3_kind": self.args.scope, "p4_scope": self.p4_scope, "p5_scope": self.p5_scope,
                        "gate": self.args.gate, "role": self.args.role, "frozen_manifest_sha256": sha,
                        "p3_baseline": {"manifest": v["p3_baseline_manifest"], "sha256": frozen["baselines"]["p3_sha256"]},
                        "p4_baseline": {"manifest": v["p4_baseline_manifest"], "sha256": frozen["baselines"]["p4_sha256"]},
                        "p5_model": frozen["model"], "schema_sha256": frozen["schema_sha256"],
                        "controller_runtime": dict(frozen["controller_runtime"], openblas_num_threads="1"),
                        "policy_fault": self.policy_fault,
                        "authorization_sha256": digest(self.args.authorization), "started_unix_ns": time.time_ns(),
                        "status": "incomplete", "failure": None, "identity": {}, "processes": [],
                        "startup": {"milestones": [], "origin_monotonic_ns": None},
                        "native_command_policy": frozen["runtime"]["native_command_policy"],
                        "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=P6, text=True).strip(),
                        "isolation": "private user/mount/PID/network namespace, loopback only, exclusive workspace lock",
                        "flight_commands_possible": self.p6_scope == FLIGHT_SCOPE, "finalized": False, "diagnostic_variant": None,
                        "acceptance_credit_possible": not self.dry and not self.p6_frozen["development"],
                        "p4_enabled_run_local": True, "p5_enabled_run_local": True, "p5_owner_script": str(OWNER),
                        "p6": {"contract": "p6-runtime-summary-v1", "row": self.args.row, "row_gate": self.row_gate,
                               "scope": self.p6_scope, "declaration_sha256": digest(self.run / "p6/fault-declaration.json"),
                               "p6_manifest_sha256": sha, "p5_manifest_sha256": self.p6_frozen["baselines"]["p5_sha256"],
                               "development": self.p6_frozen["development"], "dry_run": self.dry,
                               "dry_run_approval_sha256": digest(self.args.authorization) if self.dry else None,
                               "injectors": {m: uses(self.declared, m) for m in ("DR", "SR", "CS", "MR", "PS", "LH", "RL")},
                               "world": self.args.scenario, "runner_mechanisms": list(runner_mechanisms(self.declared)),
                               "controller_shim": shim_mode(self.declared), "run_local_changes": self.run_local_changes,
                               "injection": None, "observation_sim_s": None}}
        self.sampler = base.ResourceSampler(self.run, self.owned, self.root)
        self.sampler.start()
        try:
            self.save()
            self.prepare_world()
            config = self.px4_stack(frozen)
            self.run_controller(config)
        except (Exception, KeyboardInterrupt) as exc:
            self.summary["status"] = "failed"
            self.summary["failure"] = type(exc).__name__ + ":" + str(exc)
            print("P6 stopped: " + self.summary["failure"], flush=True)
        finally:
            try:
                errors = self.shutdown()
            except Exception as exc:
                errors = ["shutdown_exception:" + type(exc).__name__ + ":" + str(exc)]
            self.summary["shutdown_errors"] = errors
            if errors and self.summary["status"] == "passed":
                self.summary["status"], self.summary["failure"] = "failed", "shutdown:" + errors[0]
            self.finalize()
        print(json.dumps({"run_id": self.run.name, "status": self.summary["status"],
                          "failure": self.summary["failure"]}), flush=True)
        return 0 if self.summary["status"] == "passed" else 1


def parse(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true", required=True)
    parser.add_argument("--row", required=True, help="a frozen live row of p6_scenarios.yaml (Gate A ground, or B/S/C flight)")
    parser.add_argument("--gate", choices=sorted(p6_manifest.GATES) + [p6_manifest.DRY_RUN], required=True)
    parser.add_argument("--role", required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    a = parser.parse_args(argv)
    return argparse.Namespace(run=True, row=a.row, gate=a.gate, role=a.role, manifest=a.manifest,
                              authorization=a.authorization, allow_simulated_flight=a.allow_simulated_flight,
                              scope=None, fixture=None, scenario=None, variant=None)


def main(argv=None):
    return P6Runner(parse(argv)).execute()


if __name__ == "__main__":
    raise SystemExit(main())
