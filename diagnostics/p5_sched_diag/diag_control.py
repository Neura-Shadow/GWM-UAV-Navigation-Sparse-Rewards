"""Diagnostic copy of `p5_control` for the loaded-SITL scheduling diagnostic (approval 2; never an acceptance run).

Runs the installed, unchanged `gwm_policy_control.control_node.main` with two in-process changes:

* `observation_adapter.polar_profile` is replaced by the bit-exact CPU-D `optimized.polar_profile`
  (`--variant D`, the default) or left as production (`--variant A`);
* `PolicyRanker.choose` is wrapped to record, for every policy call, the call's own `inference_s` plus
  scheduler evidence read **before and after** the call, never inside its timed region: run-queue wait and
  timeslices (/proc/thread-self/schedstat), thread CPU time, voluntary/involuntary context switches, the
  Linux CPU, and the guest's runnable tasks (/proc/stat procs_running). One JSON line per call is appended to
  `<run>/p5diag/sched.jsonl` (line-buffered, like the policy ledger).

Every other byte of behaviour — model verification, mission, shield, supervisor, PX4 ownership, timeouts and
evidence — is the installed controller's. Usage: diag_control.py [--variant A|D] <p5_control arguments>
"""
import ctypes
import json
import os
from pathlib import Path
import resource
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "p5_inference_benchmark"))

libc = ctypes.CDLL("libc.so.6", use_errno=True)


def schedstat():
    with open("/proc/thread-self/schedstat") as f:
        run, wait, slices = f.read().split()
    return int(run), int(wait), int(slices)


def procs_running():
    with open("/proc/stat") as f:
        for line in f:
            if line.startswith("procs_running"):
                return int(line.split()[1])
    return None


def main(argv):
    variant = "D"
    if argv[:1] == ["--variant"]:
        variant, argv = argv[1], argv[2:]
    if variant not in ("A", "D"):
        raise SystemExit("--variant must be A or D")
    run_dir = Path(argv[argv.index("--run-dir") + 1])
    from gwm_policy_control import candidate_ranker, control_node, observation_adapter
    if variant == "D":
        import optimized
        observation_adapter.polar_profile = optimized.polar_profile
    (run_dir / "p5diag").mkdir(exist_ok=True)
    out = (run_dir / "p5diag/sched.jsonl").open("x", buffering=1)
    out.write(json.dumps({"contract": "p5-sched-diag-v1", "variant": variant, "pid": os.getpid(),
                          "polar_profile": f"{observation_adapter.polar_profile.__module__}.{observation_adapter.polar_profile.__name__}",
                          "control_node": control_node.__file__}) + "\n")
    original = candidate_ranker.PolicyRanker.choose
    calls = [0]

    def choose(self, *args, **kwargs):
        run0, wait0, sl0 = schedstat()
        r0 = resource.getrusage(resource.RUSAGE_THREAD)
        running, cpu_a, c0, w0 = procs_running(), libc.sched_getcpu(), time.thread_time_ns(), time.monotonic_ns()
        result = original(self, *args, **kwargs)
        w1, c1, cpu_b = time.monotonic_ns(), time.thread_time_ns(), libc.sched_getcpu()
        r1 = resource.getrusage(resource.RUSAGE_THREAD)
        run1, wait1, sl1 = schedstat()
        record = result[1]
        calls[0] += 1
        out.write(json.dumps({"n": calls[0], "monotonic_ns": w0, "state": record.get("state"),
                              "inference_ms": None if record.get("inference_s") is None else record["inference_s"] * 1e3,
                              "wrapper_wall_ms": (w1 - w0) / 1e6, "cpu_ms": (c1 - c0) / 1e6,
                              "runqueue_wait_ms": (wait1 - wait0) / 1e6, "oncpu_ms": (run1 - run0) / 1e6,
                              "timeslices": sl1 - sl0, "vcsw": r1.ru_nvcsw - r0.ru_nvcsw, "ivcsw": r1.ru_nivcsw - r0.ru_nivcsw,
                              "cpu_before": cpu_a, "cpu_after": cpu_b, "procs_running": running,
                              "failure": record.get("failure"), "choice_slot": None if record.get("choice_slot") is None
                              else int(record["choice_slot"])}) + "\n")
        return result
    candidate_ranker.PolicyRanker.choose = choose
    try:
        return control_node.main(argv)
    finally:
        out.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
