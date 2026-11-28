"""Evidence Retention Manager: prove that a dependent tool needs only retained records, never raw depth.

It runs one accepted read-only tool in this process under a Python audit hook (PEP 578). The hook:
* refuses every open of a `sensors/depth.bin` (PermissionError) and counts the attempts;
* logs every file the tool opens under `runs/`, by its path inside the run directory.

The tool writes to a scratch output, which is compared with the committed evidence:
* `p8_v1_replay`: the JSON must be equal once the wall-clock keys (`wall_s`, `search_wall_s_max`) are removed;
* `p8_hover_timeout_analysis`: the bytes must be identical;
* `p8_specs_verify`: the regenerated layouts, schedule and recovery parameters must equal the frozen configs (the
  tool's own `verify` mode, exit 0).

A positive control first checks that the hook really refuses a depth.bin. Nothing is written except the scratch
output and the proof record (exclusive create). One tool per process, because audit hooks cannot be removed.

Usage: cd /tmp && python3 record_only_check.py <tool> <scratch-dir> <proof.json>
"""
import hashlib
import json
import os
from pathlib import Path
import runpy
import sys

REPO = Path(__file__).resolve().parents[2]
RUNS = Path.home() / "uav_autonomy" / "runs"
CONTRACT = "evidence-record-only-check-v1"
TOOLS = {
    "p8_v1_replay": ("simulation/px4_gazebo_p8/replay/p8_v1_replay.py", "docs/evidence/v3_sim_p8_v1_replay.json"),
    "p8_hover_timeout_analysis": ("diagnostics/p8/p8_hover_timeout_analysis.py",
                                  "docs/evidence/v3_sim_p8_hover_timeout_analysis.json"),
    "p8_specs_verify": ("simulation/px4_gazebo_p8/scripts/p8_specs.py", None),
}
SPECS = ("simulation/px4_gazebo_p8/configs/p8_layouts.json", "simulation/px4_gazebo_p8/configs/p8_schedule.json",
         "simulation/px4_gazebo_p8/configs/p8_recovery.json")
WALL = {"wall_s", "search_wall_s_max"}


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def strip(value):
    if isinstance(value, dict):
        return {k: strip(v) for k, v in value.items() if k not in WALL}
    if isinstance(value, list):
        return [strip(v) for v in value]
    return value


class Hook:
    def __init__(self, runs):
        self.runs = str(runs) + os.sep
        self.read, self.refused, self.armed = {}, [], False

    def __call__(self, event, args):
        if not self.armed or event != "open" or not args or not isinstance(args[0], (str, bytes, os.PathLike)):
            return
        path = os.fsdecode(args[0])
        if path.endswith("/sensors/depth.bin"):
            self.refused.append(path)
            raise PermissionError("depth.bin withheld by the record-only check: " + path)
        if path.startswith(self.runs):
            inside = path[len(self.runs):].split(os.sep, 1)
            if len(inside) == 2:
                self.read.setdefault(inside[1], set()).add(inside[0])


def main(argv):
    name, scratch, proof = argv[0], Path(argv[1]), Path(argv[2])
    if proof.exists():
        raise SystemExit("proof exists")
    script, evidence = TOOLS[name]
    scratch.mkdir(parents=True, exist_ok=True)
    control = next(RUNS.glob("*/sensors/depth.bin"))
    hook = Hook(RUNS)
    sys.addaudithook(hook)
    hook.armed = True
    try:
        open(control, "rb").close()
        control_ok = False
    except PermissionError:
        control_ok = True
    hook.refused.clear()
    output = scratch / (name + ".json")
    if name == "p8_specs_verify":
        sys.argv = [str(REPO / script), "verify", "--layouts", str(REPO / SPECS[0]), "--schedule", str(REPO / SPECS[1]),
                    "--recovery", str(REPO / SPECS[2])]
    else:
        if output.exists():
            raise SystemExit("scratch output exists")
        sys.argv = [str(REPO / script), str(output)]
    os.chdir("/tmp")
    try:
        runpy.run_path(str(REPO / script), run_name="__main__")
        code = 0
    except SystemExit as exc:
        code = exc.code if isinstance(exc.code, int) else (0 if exc.code is None else 1)
    hook.armed = False
    result = {"tool": name, "script": script, "script_sha256": sha(REPO / script), "exit_code": code,
              "positive_control_refused": control_ok, "depth_bin_open_attempts": len(hook.refused),
              "run_files_read": {k: len(v) for k, v in sorted(hook.read.items())},
              "runs_read": len(set().union(*hook.read.values())) if hook.read else 0}
    if evidence:
        result["evidence"] = evidence
        result["evidence_sha256"] = sha(REPO / evidence)
        result["output_sha256"] = sha(output) if output.exists() else None
        if name == "p8_hover_timeout_analysis":
            result["reproduced"] = result["output_sha256"] == result["evidence_sha256"]
            result["comparison"] = "byte-identical"
        else:
            same = output.exists() and strip(json.loads(output.read_text())) == strip(json.loads((REPO / evidence).read_text()))
            result["reproduced"] = bool(same)
            result["comparison"] = "equal JSON without the wall-clock keys " + ", ".join(sorted(WALL))
    else:
        result["frozen_configs"] = {p: sha(REPO / p) for p in SPECS}
        result["reproduced"] = code == 0
        result["comparison"] = "p8_specs.py verify: each regenerated document equals its frozen config"
    result["passed"] = bool(control_ok and code == 0 and not hook.refused and result["reproduced"])
    with open(proof, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(dict(contract=CONTRACT, **result), indent=1, sort_keys=True) + "\n")
    print(json.dumps(result, indent=1, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
