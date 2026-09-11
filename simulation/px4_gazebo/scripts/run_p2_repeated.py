"""Explicit P2 acceptance, requiring an independently verified initial smoke."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid

from p1_contract import require_gates
from p2_build import package_hash, package_manifest


def check_smoke(smoke, offline, evaluator_hash):
    if (smoke.get("kind") != "flight" or smoke.get("status") != "passed"
            or (smoke.get("controller_result") or {}).get("flight") != "passed"
            or offline.get("run_id") != smoke.get("run_id")
            or offline.get("recording_integrity") != "passed"
            or offline.get("flight_acceptance") != "passed"
            or offline.get("evaluator_sha256") != evaluator_hash):
        raise ValueError("Initial P2 ROS flight and current independent evaluator must both pass before repeated acceptance")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def current_inputs(sim):
    return {"package_hash": package_hash(package_manifest(sim)),
            "files": {str(p.relative_to(sim)): digest(p) for p in
                      [*sorted((sim/"configs").glob("p2*.yaml")), sim/"configs/versions.lock.yaml", sim/"configs/qgc-monitor.ini",
                       *(sim/"scripts"/name for name in ("run_p2_repeated.py", "run_p2_control.sh", "p2_runner.py",
                                                        "p2_build.py", "p1_runner.py", "p1_contract.py", "common.sh", "verify_p2_evidence.sh")),
                       sim/"validation/collect_p2_evidence.py"]}}


def execute(command, output, deadline=600):
    with output.open("x") as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return process.wait(timeout=deadline)
        except BaseException:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=10)
            raise


def main(args):
    if not args.run_repeat:
        print("No runtime started. Use --run-repeat --allow-simulated-flight --smoke-run PATH with all four gates.")
        return 0
    require_gates(os.environ, flight=True)
    if not args.allow_simulated_flight or args.smoke_run is None:
        raise ValueError("Explicit flight flag and initial smoke evidence path required")
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home()/"uav_autonomy"))).resolve()
    if Path.home().resolve() not in root.parents:
        raise ValueError("Linux evidence root must remain beneath HOME")
    sim = Path(__file__).resolve().parents[1]
    batch = root/"runs"/(time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())+"-p2-repeat-"+uuid.uuid4().hex[:8])
    batch.mkdir(parents=True)
    report = {"status": "not_run", "required_consecutive": 20, "passed": 0, "trials": [],
              "failure": None, "smoke_run": str(args.smoke_run), "frozen_inputs": current_inputs(sim)}

    def save():
        (batch/"summary.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")

    print("P2 repeated evidence: "+str(batch), flush=True)
    save()
    try:
        smoke = json.loads((args.smoke_run/"summary.json").read_text())
        offline = json.loads((args.smoke_run/"p2-offline-evaluation.json").read_text())
        check_smoke(smoke, offline, digest(sim/"validation/collect_p2_evidence.py"))
        if smoke["identity"]["package_hash"] != report["frozen_inputs"]["package_hash"]:
            raise ValueError("Controller changed since initial smoke")
        for key, name in (("config_sha256", "p2_control.yaml"), ("clock_bridge_sha256", "p2_clock_bridge.yaml"),
                          ("lock_sha256", "versions.lock.yaml"), ("qgc_profile_sha256", "qgc-monitor.ini")):
            if smoke["identity"][key] != digest(sim/"configs"/name):
                raise ValueError("Configuration changed since initial smoke: "+name)
        for name, expected in smoke["identity"]["launcher_sha256"].items():
            if digest(sim/"scripts"/name) != expected:
                raise ValueError("Launcher changed since initial smoke: "+name)
        report["status"] = "incomplete"
        for index in range(1, 21):
            if current_inputs(sim) != report["frozen_inputs"]:
                raise ValueError("Code/config changed within acceptance sequence; start a new streak")
            report["active_trial"] = index
            save()
            command = ["bash", str(sim/"scripts/run_p2_control.sh"), "--run", "--allow-simulated-flight"]
            if args.headless:
                command.append("--headless")
            output = batch/f"trial-{index:02d}.log"
            code = execute(command, output)
            entry = {"index": index, "exit_code": code, "log": str(output)}
            report["trials"].append(entry)
            match = re.search(r"^P2 evidence: (.+)$", output.read_text(), re.MULTILINE)
            if match is None:
                raise ValueError("Trial launch failed without a run manifest")
            run = Path(match.group(1))
            entry["run_id"] = run.name
            trial = json.loads((run/"summary.json").read_text())
            if trial["identity"] != smoke["identity"] or trial["mode"] != smoke["mode"]:
                raise ValueError("Measured runtime inputs differ from initial smoke")
            evaluation_code = execute(["bash", str(sim/"scripts/verify_p2_evidence.sh"), str(run)], batch/f"offline-{index:02d}.log", 60)
            entry["offline_exit_code"] = evaluation_code
            evaluation = json.loads((run/"p2-offline-evaluation.json").read_text())
            if code != 0 or evaluation_code != 0:
                raise ValueError("Trial or independent evidence failed: "+run.name)
            check_smoke(trial, evaluation, digest(sim/"validation/collect_p2_evidence.py"))
            if current_inputs(sim) != report["frozen_inputs"]:
                raise ValueError("Inputs changed during trial")
            report["passed"] += 1
            save()
            print(f"Consecutive P2 passes: {report['passed']}/20 ({run.name})", flush=True)
        report["status"] = "passed"
    except (Exception, KeyboardInterrupt) as exc:
        if report["status"] != "not_run":
            report["status"] = "incomplete" if isinstance(exc, KeyboardInterrupt) else "failed"
        report["failure"] = str(exc) or "Interrupted"
    finally:
        save()
    print(json.dumps(report, indent=2, allow_nan=False), flush=True)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-repeat", action="store_true")
    parser.add_argument("--allow-simulated-flight", action="store_true")
    parser.add_argument("--smoke-run", type=Path)
    parser.add_argument("--headless", action="store_true")
    raise SystemExit(main(parser.parse_args()))
