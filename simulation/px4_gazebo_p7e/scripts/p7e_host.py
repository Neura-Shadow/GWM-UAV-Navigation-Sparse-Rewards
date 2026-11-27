"""P7-E host checks with fail-closed rule FC-2 (contract `p7-engineering-validation-v1` §8). Read-only.

The frozen P7 host check (`p7_host.check`, definition `p7_host_check.yaml` 9f78af75…) is reused unchanged by import:
the Windows checks (Best performance overlay, AC Online, host CPU), the workspace lock and the disk floor.

**FC-2.** The frozen `p7_host.simulator_processes` reads `pgrep` output without its exit status, so a `pgrep` that
fails (exit status other than 0 = found or 1 = none) reads as "no simulator process". Here, such a failure is a
failed check:
* **before a block:** the host check fails and nothing is flown (the authorization is not consumed);
* **after a flight:** `ProcessCheckFailed` is raised, and the block sequencer stops the batch (FC-1).
"""
import json
import os
from pathlib import Path
import subprocess
import sys

P7E = Path(__file__).resolve().parents[1]
REPO = P7E.parents[1]
for _path in (REPO / "simulation/px4_gazebo_p7/scripts", REPO / "simulation/px4_gazebo/validation"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import p7_host  # noqa: E402  (frozen P7 code, used unchanged)

CONTRACT = "p7e-host-check-record-v1"


class ProcessCheckFailed(RuntimeError):
    """FC-2: the simulator-process check could not be performed."""


def simulator_processes(pattern, run=subprocess.run):
    """As the frozen check, but a `pgrep` exit status other than 0 (found) or 1 (none) raises."""
    try:
        done = run(["pgrep", "-af", pattern], capture_output=True, text=True)
    except OSError as exc:
        raise ProcessCheckFailed("pgrep_unavailable:" + type(exc).__name__ + ":" + str(exc))
    if done.returncode not in (0, 1):
        raise ProcessCheckFailed("pgrep_failed:%s:%s" % (done.returncode, (done.stderr or "").strip()[-200:]))
    if done.returncode == 1 and (done.stdout or "").strip():
        raise ProcessCheckFailed("pgrep_inconsistent:exit_1_with_output")
    me = str(os.getpid())
    return [line for line in (done.stdout or "").splitlines() if not line.startswith(me + " ") and "pgrep" not in line]


def pattern():
    return p7_host.config()["wsl"]["simulator_process_pattern"]


def residual(run=subprocess.run):
    """After a flight: the simulator processes still alive; raises ProcessCheckFailed if that cannot be known."""
    return simulator_processes(pattern(), run)


def check(root, flights, windows=None, processes=None, free_bytes=None, run=subprocess.run):
    """The frozen pre-block check; a failed process check fails it (FC-2)."""
    error = None
    if processes is None:
        try:
            processes = simulator_processes(pattern(), run)
        except ProcessCheckFailed as exc:
            error, processes = str(exc), []
    out = p7_host.check(root, flights, windows=windows, processes=processes, free_bytes=free_bytes)
    out["frozen_contract"], out["contract"] = out["contract"], CONTRACT
    if error is not None:
        out["no_simulator_process"] = {"passed": False, "found": [], "error": error}
    out["passed"] = all(v.get("passed") for v in out.values() if isinstance(v, dict))
    return out


if __name__ == "__main__":
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    print(json.dumps(check(root, int(sys.argv[1]) if len(sys.argv) > 1 else 20), indent=2))
