"""Process-signal faults (PS) for the P6 runner: one owned process, one declared signal, recorded.

The proof that the fault happened is read from the kernel, not from the request: `/proc/<pid>/stat`
shows the stopped state (`T`) after SIGSTOP and its end after SIGCONT, and the runner's process table
records the exit after SIGINT or SIGKILL.
"""
import os
import signal
import time

SIGNALS = {"SIGINT": signal.SIGINT, "SIGKILL": getattr(signal, "SIGKILL", None), "SIGTERM": signal.SIGTERM,
           "SIGSTOP": getattr(signal, "SIGSTOP", None)}
PROOF_WALL_S = 1.0  # contract §5.4: the /proc change must follow within 1.0 s wall


def proc_state(pid):
    """The single-letter scheduler state from /proc/<pid>/stat, or None once the process is gone."""
    try:
        text = open("/proc/%d/stat" % pid, encoding="ascii").read()
    except OSError:
        return None
    return text[text.rindex(")") + 2]


def _await(pid, predicate, deadline_s=PROOF_WALL_S, clock=time.monotonic):
    end = clock() + deadline_s
    while clock() < end:
        state = proc_state(pid)
        if predicate(state):
            return state, clock()
        time.sleep(0.005)
    return proc_state(pid), None


def inject(pid, kind, ledger=None, sim_s=None, resume_after_wall_s=None, clock=time.monotonic):
    """Send the declared signal; for SIGSTOP, resume with SIGCONT after `resume_after_wall_s`.
    Returns the record (also written to the ledger) with the observed /proc evidence."""
    if kind not in SIGNALS or SIGNALS[kind] is None:
        raise ValueError("unsupported process signal: " + str(kind))
    if kind == "SIGSTOP" and not resume_after_wall_s:
        raise ValueError("SIGSTOP needs a bounded resume time")
    record = {"pid": pid, "signal": kind, "request_wall_s": clock(), "state_before": proc_state(pid)}
    os.kill(pid, SIGNALS[kind])
    if kind == "SIGSTOP":
        state, seen = _await(pid, lambda s: s in ("T", "t"), clock=clock)
        record.update(stopped_state=state, stopped_seen_wall_s=seen)
        time.sleep(resume_after_wall_s)
        os.kill(pid, signal.SIGCONT)
        state, seen = _await(pid, lambda s: s not in ("T", "t"), clock=clock)
        record.update(resumed_state=state, resumed_seen_wall_s=seen, resume_request_after_wall_s=resume_after_wall_s)
        record["proved"] = record["stopped_seen_wall_s"] is not None and record["resumed_seen_wall_s"] is not None
    else:
        record["proved"] = None  # exit is proven by the runner's process table and exit code
    if ledger is not None:
        ledger.record("activated", sim_s, **record)
    return record
