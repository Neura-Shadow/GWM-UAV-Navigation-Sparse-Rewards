"""P5 runtime supervisor: the accepted P4 runner with the `p5_control` controller (contract §2, §9).

The P4 `P4Runner` is imported from its own file and subclassed. Through it, the accepted P3 runner's
sensor stack, PX4/QGC/DDS launch, MC trace, resource sampling, ordered graceful shutdown and
finalization, and P4's scenario worlds, truth observer, P4 perception, S6 injection and shutdown
order are reused unchanged. P5 overrides only:

* preconditions: the P5 manifest (which re-verifies the P3 and P4 baselines), the P5 gate
  authorization and its consumption marker;
* the controller command: `p5_control` from the P5 install with the run-local P5 configuration, the
  frozen schema and the manifest-bound checkpoint and hashes, single-threaded BLAS (the evaluator
  replays scores bit for bit under the same setting). Perception stays the unchanged P4 node;
* the policy_fault scope (Gate A): one live start with a mismatched checkpoint, which must refuse
  before any node exists; the unchanged ground window follows so the sensor checks still apply;
* run naming, run-local configs and the summary contract; the QGC receipt names the P5 owner copy.
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
import time
import uuid

P5 = Path(__file__).resolve().parents[1]
REPO = P5.parents[1]
P4 = REPO / "simulation/px4_gazebo_p4"
SIM = REPO / "simulation/px4_gazebo"
for path in (P5 / "scripts", SIM / "scripts", SIM / "validation"):
    if str(path) not in sys.path:
        sys.path.append(str(path))

_spec = importlib.util.spec_from_file_location("p4_runner_accepted", P4 / "scripts/p4_runner.py")
p4 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(p4)
base = p4.base

import p5_manifest  # noqa: E402
from p3_provenance import digest, strict_json  # noqa: E402

OWNER = P5 / "scripts/p5_namespace_owner.py"
# P5 scope -> P4 scope that drives the unchanged P4/P3 stack
SCOPES = {"p3_readiness": "p3_readiness", "p5_readiness": "p4_readiness", "fixture": "fixture",
          "policy_fault": "p4_readiness", "flight": "flight"}
POLICY_FAULTS = ("checkpoint_mismatch",)


def run_local_policy_config(frozen):
    """Contract C6: enabled, with the manifest-bound checkpoint, only in an authorized run's local copy."""
    local = strict_json(P5 / "configs/p5_policy.yaml")
    local["enabled"] = True
    local["model"].update(path=str(P5 / frozen["model"]["path"]), sha256=frozen["model"]["sha256"])
    return local


def mismatched_checkpoint(frozen, run):
    """Gate A policy fault: the bound checkpoint plus one byte, so only the file SHA-256 check can refuse it."""
    path = run / "p5/mismatched-checkpoint.npz"
    path.write_bytes((P5 / frozen["model"]["path"]).read_bytes() + b"\0")
    return {"contract": "p5-policy-fault-v1", "kind": "checkpoint_mismatch", "model_path": str(path),
            "expected_sha256": frozen["model"]["sha256"], "actual_sha256": digest(path),
            "expected_refusal": strict_json(P5 / "configs/p5_validation.yaml")["policy_fault"]["checkpoint_mismatch"]["refusal"]}


class P5Runner(p4.P4Runner):
    def __init__(self, args):
        self.p5_scope = args.scope
        args.scope = SCOPES[args.scope]  # the P4 runner's scope drives the unchanged P4 and P3 stack
        super().__init__(args)
        self.p5_validation = strict_json(P5 / "configs/p5_validation.yaml")
        self.policy_fault = None

    # ---- preconditions ----------------------------------------------------------------------
    def preconditions(self):
        missing = [key for key in base.GATES if os.environ.get(key) != "1"]
        flight = self.p5_scope == "flight"
        if flight and (os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1" or not self.args.allow_simulated_flight):
            missing.append("GWM_ALLOW_SITL_COMMANDS/--allow-simulated-flight")
        if not flight and self.args.allow_simulated_flight:
            raise ValueError("Flight authorization is only valid for the flight scope")
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
            raise ValueError("Exclusive P1-P5 workspace lock is not held by this owner")
        except BlockingIOError:
            pass
        finally:
            os.close(probe)
        if shutil.disk_usage(self.root).free < self.validation["disk_reserve_bytes"]:
            raise ValueError("Disk reserve below the required 20 GiB")
        frozen, sha = p5_manifest.verify_current(self.root, self.args.manifest)  # includes the P3 and P4 baselines
        authorization = p5_manifest.require_authorization(self.args.authorization, self.args.gate, sha)
        marker = p5_manifest.marker_path(self.root, self.args.gate, sha)
        if not marker.exists() or strict_json(marker)["authorization_sha256"] != digest(self.args.authorization):
            raise PermissionError("Gate authorization was not consumed by the P5 gate sequencer")
        if flight != (self.args.gate in p5_manifest.FLIGHT_GATES):
            raise PermissionError("Flights belong to P5 Gates B, S and C only")
        if not base.p3_manifest.wsl_host():
            raise PermissionError("P5 gates run only on the WSL D3D12 host")
        self.marker = marker
        return frozen, sha, authorization

    # ---- launching --------------------------------------------------------------------------
    def p4_controller_command(self, env):
        """The P4 runner's controller hook: `p5_control` replaces `p4_control`; perception is unchanged."""
        receipt = strict_json(self.root / "state/p5-built.json")
        lib = Path(receipt["install"]) / "gwm_policy_control/lib/gwm_policy_control"
        model = self.frozen["model"]
        checkpoint = self.policy_fault["model_path"] if self.policy_fault else str(P5 / model["path"])
        command = [str(lib / "p5_control"), "--config", str(self.run / "p2_control.yaml"),
                   "--timing-config", str(self.run / "p2_timing.yaml"), "--p4-config", str(self.run / "p4_avoidance.yaml"),
                   "--p5-config", str(self.run / "p5_policy.yaml"), "--schema", str(self.run / "p5_model_schema.json"),
                   "--schema-sha256", self.frozen["schema_sha256"], "--model", checkpoint, "--model-sha256", model["sha256"],
                   "--run-dir", str(self.run), "--authorization-marker", str(self.marker)]
        command.append("--flight" if self.p5_scope == "flight" else "--ground-diagnostic")
        # As in P4: the P2 rule gates(env, flight or ground_diagnostic) needs the SITL gate in both modes;
        # ground diagnostic still cannot send a vehicle_command.
        env = dict(env, GWM_P4_OWNED_NAMESPACE="1", GWM_ALLOW_SITL_COMMANDS="1", OPENBLAS_NUM_THREADS="1")
        return command, env

    # ---- controller phase -------------------------------------------------------------------
    def run_controller(self, config):
        if self.p5_scope != "policy_fault":
            return super().run_controller(config)
        deadline = self.start + config["wall_deadline_s"] + 90
        self.wait_for(lambda: self.controller.poll() is not None, deadline, "policy_fault_refusal")
        code = self.controller.record["exit_code"]
        self.summary["controller_exit_code"] = code
        self.summary["controller_result"] = None
        log = (self.run / "ros-controller.log").read_text(errors="replace")
        started = [name for name in ("controller-result.json", "ros-events.jsonl", "p5/policy-ledger.jsonl")
                   if (self.run / name).exists()]
        refused = code != 0 and not started and self.policy_fault["expected_refusal"] in log
        self.policy_fault.update(exit_code=code, started_artifacts=started, refusal_observed=refused)
        self.summary["policy_fault"] = self.policy_fault
        self.save()
        if not refused:
            raise base.Abort("policy_fault:checkpoint_mismatch_not_refused")
        # The unchanged P4 ground window follows, so the sensor and perception checks still apply.
        window = self.p4_validation["ground_fixture"]["window_sim_s"]
        begin = self.wait_for(self.sim_now, deadline, "ground_window_clock")
        self.wait_for(lambda: (self.sim_now() or 0) >= begin + window, deadline, "ground_window")
        self.summary["sensor_window_sim_s"] = [begin, begin + window]
        self.summary["status"] = "passed"

    # ---- shutdown ---------------------------------------------------------------------------
    def stop_qgc(self, ground):
        from qgc_unattended import ownership_receipt, require_teardown
        self.sim_now()
        clock = {"progress_wall_s": (self.clock_tail.last or {}).get("receipt_monotonic_ns", 0) / 1e9, "error": None}
        ground = self.ground_state()
        ownership_receipt(self.run, self.qgc.popen, ground, clock, OWNER, digest(OWNER))
        self.stop("qgc-monitor", signal.SIGTERM)
        record = self.qgc.record
        cleanup = {"exit_code": record["exit_code"], "forced": record["forced"],
                   "registered_members_after": record["remaining_group_members"]}
        self.summary["qgc_normal_teardown"] = require_teardown(self.run / "qgc-monitor.log", cleanup)

    # ---- entry ----------------------------------------------------------------------------------
    def prepare_run(self, frozen):
        """Run directory and run-local configs: P3 and P4 configs copied unchanged, P4 and P5 enabled locally."""
        label = {"fixture": self.args.fixture, "policy_fault": "policy-fault-" + str(self.args.fixture),
                 "flight": "flight-" + str(self.args.scenario)}.get(self.p5_scope, self.p5_scope)
        self.run = self.root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-p5-" + label + "-"
                                         + uuid.uuid4().hex[:8])
        self.run.mkdir(parents=True)
        for name in ("tmp", "p4", "p5"):
            (self.run / name).mkdir()
        for name in ("p3_sensors.yaml", "p3_validation.yaml", "p3_sensor_bridge.yaml", "p3_fastdds.xml",
                     "p3_native_command_policy.json"):
            shutil.copyfile(SIM / "configs" / name, self.run / name)
        for name in ("p4_validation.yaml", "p4_scenarios.json"):
            shutil.copyfile(P4 / "configs" / name, self.run / name)
        for name in ("p5_validation.yaml", "p5_model_schema.json"):
            shutil.copyfile(P5 / "configs" / name, self.run / name)
        local = strict_json(P4 / "configs/p4_avoidance.yaml")
        local["enabled"] = True  # P4 contract C4: the P4 stack this run reuses is enabled only here
        (self.run / "p4_avoidance.yaml").write_text(json.dumps(local, indent=2) + "\n")
        (self.run / "p5_policy.yaml").write_text(json.dumps(run_local_policy_config(frozen), indent=2) + "\n")
        if self.p5_scope == "policy_fault":
            self.policy_fault = mismatched_checkpoint(frozen, self.run)

    def execute(self):
        self.world_dir = self.declaration = None
        frozen, sha, authorization = self.preconditions()
        self.frozen = frozen
        self.prepare_run(frozen)
        print("P5 evidence: " + str(self.run), flush=True)
        v = self.p5_validation
        self.summary = {"schema_version": 1, "contract": "p5-runtime-summary-v1", "run_id": self.run.name,
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
                        "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=P5, text=True).strip(),
                        "isolation": "private user/mount/PID/network namespace, loopback only, exclusive workspace lock",
                        "flight_commands_possible": self.p5_scope == "flight", "finalized": False,
                        "diagnostic_variant": None, "acceptance_credit_possible": True,
                        "p4_enabled_run_local": True, "p5_enabled_run_local": True, "p5_owner_script": str(OWNER)}
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
            print("P5 stopped: " + self.summary["failure"], flush=True)
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
    parser.add_argument("--scope", choices=sorted(SCOPES), required=True)
    parser.add_argument("--fixture", help="G2/G4/G6 for fixture; checkpoint_mismatch for policy_fault")
    parser.add_argument("--scenario", help="S1-S6 for flight")
    parser.add_argument("--gate", choices=sorted(p5_manifest.GATES), required=True)
    parser.add_argument("--role", required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    args = parser.parse_args(argv)
    if (args.scope in ("fixture", "policy_fault")) != bool(args.fixture):
        parser.error("--fixture is required exactly for the fixture and policy_fault scopes")
    if args.scope == "policy_fault" and args.fixture not in POLICY_FAULTS:
        parser.error("--fixture for the policy_fault scope must be one of " + ", ".join(POLICY_FAULTS))
    if (args.scope == "flight") != bool(args.scenario):
        parser.error("--scenario is required exactly for the flight scope")
    return args


def main(argv=None):
    return P5Runner(parse(argv)).execute()


if __name__ == "__main__":
    raise SystemExit(main())
