"""Explicit operator batch: 20 sequential trials, stop at the first failure."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time
import uuid

from p1_contract import require_gates


def main(args):
    if not args.run_repeat:
        print("No runtime started. Use --run-repeat --allow-simulated-flight --qgc-monitor with all four gates.")
        return 0
    require_gates(os.environ, flight=True)
    if not args.allow_simulated_flight or not args.qgc_monitor:
        raise ValueError("Repeated flight requires explicit flight and QGC monitor flags")
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy"))).resolve()
    if Path.home().resolve() not in root.parents:
        raise ValueError("Evidence root must be beneath Linux HOME")
    batch = root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-repeat-" + uuid.uuid4().hex[:8])
    batch.mkdir(parents=True)
    report = {"status": "incomplete", "required_consecutive": 20, "passed": 0,
              "trials": [], "failure": None}

    def save():
        (batch / "summary.json").write_text(json.dumps(report, indent=2) + "\n")

    print("Repeated-trial evidence: " + str(batch), flush=True)
    save()
    command = ["bash", str(Path(__file__).with_name("run_p1_baseline.sh")), "--run",
               "--allow-simulated-flight", "--qgc-monitor"]
    if args.headless:
        command.append("--headless")
    reference = None
    try:
        for index in range(1, 21):
            output = batch / f"trial-{index:02d}.log"
            report["active_trial"] = index
            save()
            with output.open("w") as stream:
                result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=False)
            match = re.search(r"^P1 evidence: (.+)$", output.read_text(), re.MULTILINE)
            entry = {"index": index, "exit_code": result.returncode, "log": str(output)}
            report["trials"].append(entry)
            if match:
                trial_path = Path(match.group(1)) / "summary.json"
                trial = json.loads(trial_path.read_text())
                entry["summary"] = str(trial_path)
                entry["run_id"] = trial["run_id"]
                identity = {key: trial[key] for key in ("config_sha256", "lock_sha256", "binary_sha256", "operator_script_sha256", "mode")}
                reference = identity if reference is None else reference
                if identity != reference:
                    raise RuntimeError("Trial inputs changed within the acceptance batch")
                if (trial["boot"] != "passed" or trial["flight_smoke"] != "passed"
                        or trial["landed_disarmed"] is not True or trial["hover"]["status"] != "passed"
                        or not trial["ulog_files"]):
                    raise RuntimeError("Trial did not meet every acceptance gate: " + trial["run_id"])
            if result.returncode != 0 or not match:
                raise RuntimeError(f"Trial {index} failed or lacks evidence")
            report["passed"] += 1
            save()
            print(f"Consecutive passes: {report['passed']}/20 ({entry['run_id']})", flush=True)
        report["status"] = "passed"
        report["input_identity"] = reference
    except (Exception, KeyboardInterrupt) as exc:
        report["status"] = "failed" if isinstance(exc, Exception) else "incomplete"
        report["failure"] = str(exc) or "Interrupted"
    finally:
        save()
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-repeat", action="store_true")
    parser.add_argument("--allow-simulated-flight", action="store_true")
    parser.add_argument("--qgc-monitor", action="store_true")
    parser.add_argument("--headless", action="store_true")
    raise SystemExit(main(parser.parse_args()))
