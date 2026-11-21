"""P7 pre-block host check (contract §7.1 'not flown', §7.4, decision D10). Read-only: it changes no host setting (C4).

Run from WSL, outside the flight namespace, by the block sequencer before an authorization is consumed.
* **Windows,** through WSL interop (one read-only PowerShell call):
  * the active AC power overlay must be Best performance;
  * the power line must be Online;
  * the host CPU load (Win32_Processor LoadPercentage, sampled) must be at or below the limit.
* **WSL:**
  * no simulator or runner process;
  * the P1-P7 workspace lock is free;
  * free space minus the block's planned use must stay at or above the 150 GB floor.

The manifest check is the sequencer's own `p7_manifest.verify_current`.
"""
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

P7 = Path(__file__).resolve().parents[1]
REPO = P7.parents[1]
sys.path.insert(0, str(REPO / "simulation/px4_gazebo/validation"))
from p3_provenance import strict_json  # noqa: E402

CONFIG = P7 / "configs/p7_host_check.yaml"
POWERSHELL = "powershell.exe"
SCRIPT = (
    "$ErrorActionPreference='Stop';"
    "$k=Get-ItemProperty 'HKLM:\\SYSTEM\\CurrentControlSet\\Control\\Power\\User\\PowerSchemes';"
    "Add-Type -AssemblyName System.Windows.Forms;"
    "$line=[System.Windows.Forms.SystemInformation]::PowerStatus.PowerLineStatus.ToString();"
    "$cpu=@(); for($i=0;$i -lt %d;$i++){ if($i -gt 0){Start-Sleep -Seconds %d};"
    "$cpu+=[double](Get-CimInstance Win32_Processor | Measure-Object -Property LoadPercentage -Average).Average };"
    "[pscustomobject]@{overlay_ac=[string]$k.ActiveOverlayAcPowerScheme; power_line=$line; cpu_percent=$cpu} | ConvertTo-Json -Compress")


def config():
    return strict_json(CONFIG)


def windows_state(c, run=subprocess.run):
    w = c["windows"]
    done = run([POWERSHELL, "-NoProfile", "-NonInteractive", "-Command", SCRIPT % (w["cpu_samples"], w["cpu_sample_interval_s"])],
               capture_output=True, text=True, timeout=120)
    if done.returncode != 0:
        raise RuntimeError("windows_state_unreadable:" + done.stderr.strip()[-300:])
    return json.loads(done.stdout.strip().splitlines()[-1])


def windows_checks(state, c):
    w = c["windows"]
    cpu = state.get("cpu_percent")
    cpu = [cpu] if isinstance(cpu, (int, float)) else list(cpu or [])
    mean = sum(cpu) / len(cpu) if cpu else None
    return {"ac_overlay": {"passed": str(state.get("overlay_ac", "")).lower() == w["ac_overlay_guid"],
                           "value": state.get("overlay_ac"), "required": w["ac_overlay_guid"]},
            "ac_power": {"passed": state.get("power_line") == w["power_line_status"], "value": state.get("power_line")},
            "host_cpu": {"passed": mean is not None and mean <= w["cpu_load_max_percent"], "samples_percent": cpu,
                         "mean_percent": mean, "max_percent": w["cpu_load_max_percent"]}}


def simulator_processes(pattern, run=subprocess.run):
    out = run(["pgrep", "-af", pattern], capture_output=True, text=True).stdout.splitlines()
    me = str(os.getpid())
    return [line for line in out if not line.startswith(me + " ") and "pgrep" not in line]


def lock_free(path):
    """True if nobody holds the workspace lock. Opens read-only and releases at once; never creates the file."""
    if not Path(path).exists():
        return True
    fd = os.open(path, os.O_RDONLY)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(fd, fcntl.LOCK_UN)
        return True
    except BlockingIOError:
        return False
    finally:
        os.close(fd)


def disk_check(free_bytes, flights, c):
    w = c["wsl"]
    planned = flights * w["planned_bytes_per_flight"]
    return {"passed": free_bytes - planned >= w["disk_floor_bytes"], "free_bytes": free_bytes, "planned_flights": flights,
            "planned_bytes": planned, "after_bytes": free_bytes - planned, "floor_bytes": w["disk_floor_bytes"]}


def check(root, flights, windows=None, processes=None, free_bytes=None):
    """The full pre-block check. `windows`, `processes` and `free_bytes` may be supplied (tests); otherwise measured."""
    c = config()
    out = {"contract": "p7-host-check-record-v1", "definition": str(CONFIG.relative_to(REPO))}
    try:
        out.update(windows_checks(windows if windows is not None else windows_state(c), c))
    except Exception as exc:
        out["windows"] = {"passed": False, "error": type(exc).__name__ + ":" + str(exc)}
    found = processes if processes is not None else simulator_processes(c["wsl"]["simulator_process_pattern"])
    out["no_simulator_process"] = {"passed": not found, "found": found[:10]}
    out["workspace_lock_free"] = {"passed": lock_free(Path(root) / c["wsl"]["workspace_lock"])}
    out["disk_floor"] = disk_check(free_bytes if free_bytes is not None else shutil.disk_usage(root).free, flights, c)
    out["passed"] = all(v.get("passed") for v in out.values() if isinstance(v, dict))
    return out


if __name__ == "__main__":
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    print(json.dumps(check(root, int(sys.argv[1]) if len(sys.argv) > 1 else 40), indent=2))
