"""Loaded-SITL scheduling diagnostic runner (operator approval 2, 2026-11-01). Never an acceptance run.

Subclasses the accepted `P5Runner` (imported from its own file, unchanged); through it the accepted P4 and P3
runners launch the identical PX4/Gazebo/QGC/DDS/sensor/perception stack with the same isolation (private
namespace, loopback only, exclusive workspace lock), ordered shutdown and finalization. Overridden here only:

* preconditions: every P5 environment, namespace, lock, disk and WSL-host check, and the frozen manifest
  `db4c4779…` must still verify (model, schema, P3/P4 baselines). Instead of a gate authorization and
  consumption marker it requires the diagnostic approval record and enforces its flight cap. No gate marker
  or gate result is read or written;
* controller command: the same `p5_control` arguments, run through `diag_control.py` (CPU-D polar profile +
  per-call scheduler logging outside the timed region);
* run naming (`…-p5diag-flight-S2-…`), a run-local marker file for the P4/P5 enable checks, and a summary with
  `acceptance_credit_possible: false`;
* a 10 Hz host sampler (`p5diag/host.jsonl`): /proc/stat (procs_running, procs_blocked, CPU jiffies incl.
  steal), /proc/loadavg and, per process in the namespace, CPU time, thread count and runnable threads.
Usage (inside the namespace, via run_diag.sh): --approval FILE --manifest FILE --scenario S2 [--variant D]
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
import uuid

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
P5 = REPO / "simulation/px4_gazebo_p5"
_spec = importlib.util.spec_from_file_location("p5_runner_accepted", P5 / "scripts/p5_runner.py")
p5r = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(p5r)
base, p4, p5_manifest = p5r.base, p5r.p4, p5r.p5_manifest
from p3_provenance import digest, strict_json  # noqa: E402

OWNER = HERE / "p5_namespace_owner.py"
p5r.OWNER = OWNER  # the QGC ownership receipt names the owner copy actually running as PID 1 (same bytes)
APPROVAL_CONTRACT = "p5-sched-diag-approval-v1"


class HostSampler(threading.Thread):
    def __init__(self, path, period=0.1):
        super().__init__(daemon=True)
        self.out, self.period, self.halt = path.open("x", buffering=1), period, threading.Event()

    def sample(self):
        row = {"monotonic_ns": time.monotonic_ns()}
        with open("/proc/stat") as f:
            for line in f:
                parts = line.split()
                if parts[0] == "cpu":
                    row["cpu_jiffies"] = [int(x) for x in parts[1:]]  # user nice system idle iowait irq softirq steal
                elif parts[0] in ("procs_running", "procs_blocked"):
                    row[parts[0]] = int(parts[1])
        with open("/proc/loadavg") as f:
            row["loadavg"] = f.read().split()[:4]
        procs = {}
        for entry in os.scandir("/proc"):
            if not entry.name.isdigit():
                continue
            try:
                with open(f"/proc/{entry.name}/stat") as f:
                    head, tail = f.read().rsplit(")", 1)
                fields = tail.split()
                comm = head.split("(", 1)[1][:24]
                runnable = 0
                for task in os.scandir(f"/proc/{entry.name}/task"):
                    try:
                        with open(f"/proc/{entry.name}/task/{task.name}/stat") as t:
                            runnable += t.read().rsplit(")", 1)[1].split()[0] == "R"
                    except OSError:
                        pass
                procs[entry.name] = [comm, int(fields[11]) + int(fields[12]), int(fields[17]), runnable]
            except (OSError, IndexError, ValueError):
                continue
        row["procs"] = procs  # pid -> [comm, utime+stime ticks, threads, runnable threads]
        return row

    def run(self):
        while not self.halt.is_set():
            start = time.monotonic()
            try:
                self.out.write(json.dumps(self.sample()) + "\n")
            except Exception as exc:  # the sampler must never disturb the run
                self.out.write(json.dumps({"sampler_error": repr(exc)}) + "\n")
            self.halt.wait(max(0.0, self.period - (time.monotonic() - start)))

    def stop(self):
        self.halt.set()
        self.join(timeout=2)
        self.out.close()


class DiagRunner(p5r.P5Runner):
    def preconditions(self):
        missing = [key for key in base.GATES if os.environ.get(key) != "1"]
        if os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1" or not self.args.allow_simulated_flight:
            missing.append("GWM_ALLOW_SITL_COMMANDS/--allow-simulated-flight")
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
        if not base.p3_manifest.wsl_host():
            raise PermissionError("The diagnostic runs only on the WSL D3D12 host")
        frozen, sha = p5_manifest.verify_current(self.root, self.args.manifest)  # model, schema, P3/P4 baselines
        approval = strict_json(self.args.authorization)
        if approval.get("contract") != APPROVAL_CONTRACT or approval.get("manifest_sha256") != sha:
            raise PermissionError("Diagnostic approval record does not match")
        if self.args.scenario not in approval["scenarios"] or self.args.variant not in approval["variants"]:
            raise PermissionError("Scenario or variant outside the diagnostic approval")
        flown = sorted(p.name for p in (self.root / "runs").glob("*-p5diag-flight-*"))
        if len(flown) >= approval["flights_max"]:
            raise PermissionError(f"Diagnostic flight cap reached ({len(flown)}/{approval['flights_max']})")
        self.diag_index = len(flown) + 1
        return frozen, sha, approval

    def p4_controller_command(self, env):
        command, env = super().p4_controller_command(env)
        if not command[0].endswith("/p5_control"):
            raise ValueError("unexpected controller command")
        return ["/usr/bin/python3", str(HERE / "diag_control.py"), "--variant", self.args.variant] + command[1:], env

    def prepare_run(self, frozen):
        self.run = self.root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-p5diag-flight-"
                                         + self.args.scenario + "-" + uuid.uuid4().hex[:8])
        self.run.mkdir(parents=True)
        for name in ("tmp", "p4", "p5", "p5diag"):
            (self.run / name).mkdir()
        for name in ("p3_sensors.yaml", "p3_validation.yaml", "p3_sensor_bridge.yaml", "p3_fastdds.xml",
                     "p3_native_command_policy.json"):
            shutil.copyfile(p5r.SIM / "configs" / name, self.run / name)
        for name in ("p4_validation.yaml", "p4_scenarios.json"):
            shutil.copyfile(p5r.P4 / "configs" / name, self.run / name)
        for name in ("p5_validation.yaml", "p5_model_schema.json"):
            shutil.copyfile(P5 / "configs" / name, self.run / name)
        local = strict_json(p5r.P4 / "configs/p4_avoidance.yaml")
        local["enabled"] = True
        (self.run / "p4_avoidance.yaml").write_text(json.dumps(local, indent=2) + "\n")
        (self.run / "p5_policy.yaml").write_text(json.dumps(p5r.run_local_policy_config(frozen), indent=2) + "\n")
        self.marker = self.run / "p5diag/diag-marker.json"  # the P4/P5 enable checks need an existing file
        self.marker.write_text(json.dumps({"contract": "p5-sched-diag-marker-v1", "approval_sha256": digest(self.args.authorization),
                                           "diag_index": self.diag_index, "acceptance_credit": False}) + "\n")

    def execute(self):
        self.world_dir = self.declaration = None
        frozen, sha, approval = self.preconditions()
        self.frozen = frozen
        self.prepare_run(frozen)
        print("P5 diagnostic evidence: " + str(self.run), flush=True)
        v = self.p5_validation
        self.summary = {"schema_version": 1, "contract": "p5-sched-diag-summary-v1", "run_id": self.run.name,
                        "p3_kind": self.args.scope, "p4_scope": self.p4_scope, "p5_scope": self.p5_scope,
                        "gate": "DIAG", "role": f"sched-diag-{self.diag_index}", "frozen_manifest_sha256": sha,
                        "p3_baseline": {"manifest": v["p3_baseline_manifest"], "sha256": frozen["baselines"]["p3_sha256"]},
                        "p4_baseline": {"manifest": v["p4_baseline_manifest"], "sha256": frozen["baselines"]["p4_sha256"]},
                        "p5_model": frozen["model"], "schema_sha256": frozen["schema_sha256"],
                        "controller_runtime": dict(frozen["controller_runtime"], openblas_num_threads="1"),
                        "controller_variant": {"variant": self.args.variant, "entry": str(HERE / "diag_control.py"),
                                               "entry_sha256": digest(HERE / "diag_control.py"),
                                               "optimized_sha256": digest(HERE.parent / "p5_inference_benchmark/optimized.py")},
                        "policy_fault": None, "authorization_sha256": digest(self.args.authorization),
                        "started_unix_ns": time.time_ns(), "status": "incomplete", "failure": None, "identity": {},
                        "processes": [], "startup": {"milestones": [], "origin_monotonic_ns": None},
                        "native_command_policy": frozen["runtime"]["native_command_policy"],
                        "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=P5, text=True).strip(),
                        "isolation": "private user/mount/PID/network namespace, loopback only, exclusive workspace lock",
                        "flight_commands_possible": True, "finalized": False,
                        "diagnostic_variant": "sched-" + self.args.variant, "acceptance_credit_possible": False,
                        "p4_enabled_run_local": True, "p5_enabled_run_local": True, "p5_owner_script": str(OWNER)}
        self.sampler = base.ResourceSampler(self.run, self.owned, self.root)
        self.sampler.start()
        host = HostSampler(self.run / "p5diag/host.jsonl")
        host.start()
        try:
            self.save()
            self.prepare_world()
            config = self.px4_stack(frozen)
            self.run_controller(config)
        except (Exception, KeyboardInterrupt) as exc:
            self.summary["status"] = "failed"
            self.summary["failure"] = type(exc).__name__ + ":" + str(exc)
            print("P5 diagnostic stopped: " + self.summary["failure"], flush=True)
        finally:
            host.stop()
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


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--approval", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--scenario", choices=("S2",), required=True)
    parser.add_argument("--variant", choices=("A", "D"), default="D")
    parser.add_argument("--allow-simulated-flight", action="store_true")
    a = parser.parse_args(argv)
    args = argparse.Namespace(run=True, scope="flight", fixture=None, scenario=a.scenario, gate="DIAG",
                              role="sched-diag", manifest=a.manifest, authorization=a.approval,
                              allow_simulated_flight=a.allow_simulated_flight, variant=a.variant)
    return DiagRunner(args).execute()


if __name__ == "__main__":
    raise SystemExit(main())
