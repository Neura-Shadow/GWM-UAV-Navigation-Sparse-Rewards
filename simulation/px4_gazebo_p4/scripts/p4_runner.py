"""P4 runtime supervisor: the accepted P3 runner with the P4 stack (contract §2, §8, §9.1).

The P3 `Runner` is imported from its own file and subclassed; its sensor stack,
PX4/QGC/DDS launch, MC trace, resource sampling, ordered graceful shutdown and
finalization are reused unchanged. P4 overrides only:

* preconditions: the P4 manifest, authorization and consumption marker, and the
  accepted P3 baseline (verified inside the P4 manifest check);
* the world: PX4 boots the generated scenario world (run-local gz_env.sh points
  PX4_GZ_WORLDS at the run's world directory; the pinned tree is untouched);
* the controller launch: the truth observer and P4 perception start first, then
  `p4_control` replaces the P2 controller (scope p3_readiness keeps P2 unchanged);
* S6 only: one SIGINT to the sensor bridge at 2.0 m from origin, recorded;
* shutdown order: perception stops before the sensor bridge, truth after the clock bridge;
* the QGC ownership receipt names the P4 owner copy (byte-identical to the P3 owner).
"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

P4 = Path(__file__).resolve().parents[1]
REPO = P4.parents[1]
SIM = REPO / "simulation/px4_gazebo"
for path in (P4 / "scripts", P4 / "validation", SIM / "scripts", SIM / "validation", SIM / "ros2_ws/src/gwm_px4_control"):
    if str(path) not in sys.path:
        sys.path.append(str(path))

_spec = importlib.util.spec_from_file_location("p3_runner_accepted", SIM / "scripts/p3_runner.py")
base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(base)

import p4_manifest  # noqa: E402
from p3_provenance import digest, strict_json  # noqa: E402

OWNER = P4 / "scripts/p4_namespace_owner.py"
# P4 scope -> (P3 base scope for the unchanged PX4/sensor stack, P4 role)
SCOPES = {"p3_readiness": "readiness", "p4_readiness": "readiness", "fixture": "readiness", "fault": "readiness",
          "flight": "flight"}
FAULTS = ("adapter_stop", "bridge_stop")


class P4Runner(base.Runner):
    def __init__(self, args):
        self.p4_scope = args.scope
        args.scope = SCOPES[args.scope]  # the base runner's scope drives the unchanged P3 stack
        super().__init__(args)
        self.p4_validation = strict_json(P4 / "configs/p4_validation.yaml")
        self.scenarios = strict_json(P4 / "configs/p4_scenarios.json")
        self.p4_active = self.p4_scope != "p3_readiness"
        self.injection = None

    # ---- preconditions ----------------------------------------------------------------------
    def preconditions(self):
        missing = [key for key in base.GATES if os.environ.get(key) != "1"]
        flight = self.p4_scope == "flight"
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
            raise ValueError("Exclusive P1/P2/P3/P4 workspace lock is not held by this owner")
        except BlockingIOError:
            pass
        finally:
            os.close(probe)
        if shutil.disk_usage(self.root).free < self.validation["disk_reserve_bytes"]:
            raise ValueError("Disk reserve below the required 20 GiB")
        frozen, sha = p4_manifest.verify_current(self.root, self.args.manifest)  # includes the P3 baseline check
        authorization = p4_manifest.require_authorization(self.args.authorization, self.args.gate, sha)
        marker = p4_manifest.marker_path(self.root, self.args.gate, sha)
        if not marker.exists() or strict_json(marker)["authorization_sha256"] != digest(self.args.authorization):
            raise PermissionError("Gate authorization was not consumed by the P4 gate sequencer")
        if flight != (self.args.gate in p4_manifest.FLIGHT_GATES):
            raise PermissionError("Flights belong to P4 Gates B, S and C only")
        if not base.p3_manifest.wsl_host():
            raise PermissionError("P4 gates run only on the WSL D3D12 host")
        self.marker = marker
        return frozen, sha, authorization

    # ---- launching --------------------------------------------------------------------------
    def environment(self, extra=None):
        env = super().environment(extra)
        env["GWM_P4_OWNED_NAMESPACE"] = "1"
        return env

    def launch(self, name, command, env, stop_method="SIGINT", grace=None, cwd=None):
        if name == "qgc-monitor" and self.world_dir is not None:
            self.point_px4_at_world()
        if name == "ros-controller" and self.p4_active:
            self.launch_p4_sensing()
            command, env = self.p4_controller_command(env)
        return super().launch(name, command, env, stop_method, grace, cwd)

    def point_px4_at_world(self):
        """PX4 sources rootfs/gz_env.sh; only its PX4_GZ_WORLDS line is redirected to the run world."""
        path = self.run / "rootfs/gz_env.sh"
        lines = path.read_text().splitlines()
        hits = [i for i, line in enumerate(lines) if line.startswith("export PX4_GZ_WORLDS=")]
        if len(hits) != 1:
            raise base.Abort("gz_env_world_line_not_unique")
        lines[hits[0]] = "export PX4_GZ_WORLDS=" + str(self.world_dir)
        path.write_text("\n".join(lines) + "\n")
        self.summary["px4_world"] = {"worlds_dir": str(self.world_dir), "sha256": digest(self.world_dir / "default.sdf")}

    def launch_p4_sensing(self):
        receipt = strict_json(self.root / "state/p4-built.json")
        env = self.environment({"FASTRTPS_DEFAULT_PROFILES_FILE": str(self.run / "p3_fastdds.xml")})
        obstacles = []
        for box in (self.declaration or {}).get("boxes", []):
            obstacles += ["--obstacle", box["name"]]
        self.launch("p4-truth-observer", ["/usr/bin/python3", "-s", "-P", str(P4 / "validation/p4_truth_observer.py"),
                                          str(self.run), "default", self.sensors["model"]] + obstacles, env)
        lib = Path(receipt["install"]) / "gwm_obstacle_avoidance/lib/gwm_obstacle_avoidance"
        self.launch("p4-perception", [str(lib / "p4_perception"), "--config", str(self.run / "p4_avoidance.yaml"),
                                      "--sensors", str(self.run / "p3_sensors.yaml"), "--run-dir", str(self.run),
                                      "--authorization-marker", str(self.marker)], env)

    def p4_controller_command(self, env):
        receipt = strict_json(self.root / "state/p4-built.json")
        lib = Path(receipt["install"]) / "gwm_obstacle_avoidance/lib/gwm_obstacle_avoidance"
        command = [str(lib / "p4_control"), "--config", str(self.run / "p2_control.yaml"),
                   "--timing-config", str(self.run / "p2_timing.yaml"), "--p4-config", str(self.run / "p4_avoidance.yaml"),
                   "--run-dir", str(self.run), "--authorization-marker", str(self.marker)]
        command.append("--flight" if self.p4_scope == "flight" else "--ground-diagnostic")
        env = dict(env, GWM_P4_OWNED_NAMESPACE="1")
        if self.p4_scope != "flight":
            env.pop("GWM_ALLOW_SITL_COMMANDS", None)
        return command, env

    # ---- controller phase -------------------------------------------------------------------
    def run_controller(self, config):
        if self.p4_scope == "p3_readiness":
            return super().run_controller(config)
        deadline = self.start + config["wall_deadline_s"] + 90
        events = base.Tail(self.run / "ros-events.jsonl")
        while self.controller.poll() is None:
            self.pump()
            if (self.declaration or {}).get("fault") == "bridge_stop_at_distance" and self.injection is None:
                self.watch_injection(events.read())
            if time.monotonic() > deadline:
                raise base.Abort("deadline:controller")
            time.sleep(0.05)
        result_path = self.run / "controller-result.json"
        self.summary["controller_exit_code"] = self.controller.record["exit_code"]
        self.summary["controller_result"] = strict_json(result_path) if result_path.exists() else None
        result = self.summary["controller_result"]
        if result is not None:
            self.summary["controller_publishers_only_from_controller"] = bool(result.get("graph_valid"))
        if self.controller.record["exit_code"] != 0:
            raise base.Abort("controller_failed")
        transitions = {t["to"]: t["sim_s"] for t in result["transitions"]}
        if self.p4_scope == "flight":
            end = transitions.get("COMPLETE") or transitions.get("RECOVERY")
            self.summary["sensor_window_sim_s"] = [transitions["PRESTREAM_SAFE_SETPOINTS"], end]
        else:
            # Ground scopes: the ground-diagnostic controller has finished; perception keeps running.
            # Hold the declared ground window, then (fault scope) inject and observe.
            window = self.p4_validation["ground_fixture"]["window_sim_s"]
            begin = self.wait_for(self.sim_now, deadline, "ground_window_clock")
            self.wait_for(lambda: (self.sim_now() or 0) >= begin + window, deadline, "ground_window")
            self.summary["sensor_window_sim_s"] = [begin, begin + window]
            if self.p4_scope == "fault":
                self.inject_ground_fault()
                observe_until = time.monotonic() + 5.0
                self.wait_for(lambda: time.monotonic() >= observe_until, observe_until + 1, "fault_observation")
        self.summary["status"] = "passed"

    def watch_injection(self, rows):
        """S6: the first TRANSIT sample at >= 2.0 m from origin triggers one SIGINT to the sensor bridge."""
        limit = self.p4_validation["outcomes"]["fault_inject_distance_m"]
        for row in rows:
            if row.get("event") != "sample" or row.get("phase") != "TRANSIT":
                continue
            result_origin = self.origin_ned(rows_hint=row)
            if result_origin is None:
                continue
            n, e, _ = row["sample"]["position"]
            if math.hypot(n - result_origin[0], e - result_origin[1]) >= limit:
                self.inject("sensor-bridge", vehicle_ned=row["sample"]["position"])
                return

    def origin_ned(self, rows_hint=None):
        if getattr(self, "_origin", None) is None:
            for row in base.Tail(self.run / "ros-events.jsonl").read():
                if row.get("event") == "transition" and row.get("to") == "PRESTREAM_SAFE_SETPOINTS":
                    self._origin = row["evidence"]["origin_ned"]
                    break
        return getattr(self, "_origin", None)

    def inject_ground_fault(self):
        name = {"adapter_stop": "sensor-adapter", "bridge_stop": "sensor-bridge"}[self.args.fixture]
        self.inject(name, vehicle_ned=None)

    def inject(self, process_name, vehicle_ned):
        item = next(o for o in self.owned if o.name == process_name)
        request_sim = self.sim_now()
        item.request_stop(signal.SIGINT)
        self.injection = {"contract": "p4-fault-injection-v1", "scope": self.p4_scope,
                          "scenario": (self.declaration or {}).get("scenario") or self.args.fixture,
                          "process": process_name, "pid": item.pid, "signal": "SIGINT",
                          "request_monotonic_s": item.record["stop_requested_monotonic_s"], "t_inj_sim_s": request_sim,
                          "vehicle_ned": vehicle_ned}
        base.atomic_json(self.run / "p4/fault_injection.json", self.injection, exclusive=True)
        self.summary["fault_injection"] = self.injection
        self.save()

    # ---- shutdown ---------------------------------------------------------------------------
    def stop(self, name, sig=signal.SIGINT):
        if name == "sensor-bridge":
            super().stop("p4-perception")  # the second depth subscriber stops before its source
        super().stop(name, sig)
        if name == "clock-bridge":
            super().stop("p4-truth-observer")  # ground truth records through landing

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

    # ---- world --------------------------------------------------------------------------------
    def prepare_world(self):
        """Scenario or ground-fixture world from the pinned default world (P3-pinned asset digest)."""
        from p4_worlds import generate
        self.world_dir, self.declaration = None, None
        name = self.args.scenario if self.p4_scope == "flight" else (
            self.args.fixture if self.p4_scope == "fixture" else None)
        if name is None:
            return
        receipt = strict_json(self.root / "state/px4-mc-trace-built.json")
        env_text = (Path(receipt["build"]) / "rootfs/gz_env.sh").read_text()
        worlds = next(line.split("=", 1)[1] for line in env_text.splitlines() if line.startswith("export PX4_GZ_WORLDS="))
        pinned = self.frozen["runtime"]["assets"]["Tools/simulation/gz/worlds/default.sdf"]
        if digest(Path(worlds) / "default.sdf") != pinned:
            raise base.Abort("px4_default_world_differs_from_pinned_asset")
        text, declaration = generate((Path(worlds) / "default.sdf").read_text(), name, self.scenarios)
        self.world_dir = self.run / "world"
        self.world_dir.mkdir()
        (self.world_dir / "default.sdf").write_text(text)
        base.atomic_json(self.run / "p4/scenario.json", declaration, exclusive=True)
        self.declaration = declaration
        self.summary.update(scenario=declaration, generated_world_sha256=digest(self.world_dir / "default.sdf"))

    # ---- entry ----------------------------------------------------------------------------------
    def execute(self):
        self.world_dir = self.declaration = None
        frozen, sha, authorization = self.preconditions()
        self.frozen = frozen
        label = {"fixture": self.args.fixture, "fault": "fault-" + str(self.args.fixture),
                 "flight": "flight-" + str(self.args.scenario)}.get(self.p4_scope, self.p4_scope)
        self.run = self.root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-p4-" + label + "-"
                                         + uuid.uuid4().hex[:8])
        self.run.mkdir(parents=True)
        (self.run / "tmp").mkdir()
        (self.run / "p4").mkdir()
        for name in ("p3_sensors.yaml", "p3_validation.yaml", "p3_sensor_bridge.yaml", "p3_fastdds.xml",
                     "p3_native_command_policy.json"):
            shutil.copyfile(SIM / "configs" / name, self.run / name)
        for name in ("p4_validation.yaml", "p4_scenarios.json"):
            shutil.copyfile(P4 / "configs" / name, self.run / name)
        local = strict_json(P4 / "configs/p4_avoidance.yaml")
        local["enabled"] = True  # contract C4: enabled only in this authorized run's local copy
        (self.run / "p4_avoidance.yaml").write_text(json.dumps(local, indent=2) + "\n")
        print("P4 evidence: " + str(self.run), flush=True)
        self.summary = {"schema_version": 1, "contract": "p4-runtime-summary-v1", "run_id": self.run.name,
                        "p3_kind": self.args.scope, "p4_scope": self.p4_scope, "gate": self.args.gate,
                        "role": self.args.role, "frozen_manifest_sha256": sha, "p3_baseline": frozen["p3_baseline"],
                        "authorization_sha256": digest(self.args.authorization), "started_unix_ns": time.time_ns(),
                        "status": "incomplete", "failure": None, "identity": {}, "processes": [],
                        "startup": {"milestones": [], "origin_monotonic_ns": None},
                        "native_command_policy": frozen["runtime"]["native_command_policy"],
                        "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=P4, text=True).strip(),
                        "isolation": "private user/mount/PID/network namespace, loopback only, exclusive workspace lock",
                        "flight_commands_possible": self.p4_scope == "flight", "finalized": False,
                        "diagnostic_variant": None, "acceptance_credit_possible": True,
                        "p4_enabled_run_local": True, "p4_owner_script": str(OWNER)}
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
            print("P4 stopped: " + self.summary["failure"], flush=True)
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
    parser.add_argument("--fixture", help="G2/G4/G6 for fixture; adapter_stop/bridge_stop for fault")
    parser.add_argument("--scenario", help="S1-S6 for flight")
    parser.add_argument("--gate", choices=sorted(p4_manifest.GATES), required=True)
    parser.add_argument("--role", required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    args = parser.parse_args(argv)
    if (args.scope in ("fixture", "fault")) != bool(args.fixture):
        parser.error("--fixture is required exactly for the fixture and fault scopes")
    if args.scope == "fault" and args.fixture not in FAULTS:
        parser.error("--fixture for the fault scope must be one of " + ", ".join(FAULTS))
    if (args.scope == "flight") != bool(args.scenario):
        parser.error("--scenario is required exactly for the flight scope")
    return args


def main(argv=None):
    return P4Runner(parse(argv)).execute()


if __name__ == "__main__":
    raise SystemExit(main())
