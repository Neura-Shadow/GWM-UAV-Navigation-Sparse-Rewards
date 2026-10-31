"""Strict graceful-cleanup classification for every owned P3 process.

A process is graceful only when it was asked to stop by its declared method
(or completed its own bounded work), was reaped, exited with a declared
numeric success code within its grace period, needed no escalation and left
no member of its process group behind. Reaped is not graceful; signal deaths
(SIGTERM, SIGKILL, SIGSEGV, ...) are never graceful, including for an
intentionally interrupted producer.
"""
STOP_METHODS = ("SIGINT", "SIGTERM", "console_shutdown", "completes_own_work")

# Exit codes are recorded on Linux (x86_64); name them by that table, not by the
# evaluating host's signal module, so offline re-evaluation is host-independent.
LINUX_SIGNALS = {1: "SIGHUP", 2: "SIGINT", 3: "SIGQUIT", 4: "SIGILL", 5: "SIGTRAP", 6: "SIGABRT",
                 7: "SIGBUS", 8: "SIGFPE", 9: "SIGKILL", 10: "SIGUSR1", 11: "SIGSEGV", 12: "SIGUSR2",
                 13: "SIGPIPE", 14: "SIGALRM", 15: "SIGTERM", 16: "SIGSTKFLT", 17: "SIGCHLD",
                 18: "SIGCONT", 19: "SIGSTOP", 20: "SIGTSTP", 21: "SIGTTIN", 22: "SIGTTOU", 23: "SIGURG",
                 24: "SIGXCPU", 25: "SIGXFSZ", 26: "SIGVTALRM", 27: "SIGPROF", 28: "SIGWINCH", 29: "SIGIO",
                 30: "SIGPWR", 31: "SIGSYS"}


def signal_name(code):
    return LINUX_SIGNALS.get(-code, "signal_" + str(-code))


def classify(record):
    """Return (classification, reason). classification is 'graceful' or 'failed'."""
    method = record.get("stop_method")
    if method not in STOP_METHODS:
        return "failed", "undeclared_stop_method"
    code = record.get("exit_code")
    if record.get("reaped") is not True or code is None:
        return "failed", "not_reaped"
    if record.get("forced"):
        return "failed", "forced_kill_after_grace"
    if isinstance(code, bool) or not isinstance(code, int):
        return "failed", "exit_code_unavailable"
    if code < 0:
        return "failed", "terminated_by_" + signal_name(code)
    if record.get("remaining_group_members"):
        return "failed", "process_group_members_remained"
    requested = record.get("stop_requested_monotonic_s")
    exited = record.get("exit_observed_monotonic_s")
    if method != "completes_own_work":
        if requested is None:
            return "failed", "exited_before_stop_request"
        if exited is None or exited < requested:
            return "failed", "exited_before_stop_request"
        if exited - requested > record.get("grace_s", 0):
            return "failed", "graceful_deadline_exceeded"
    if code not in record.get("graceful_codes", (0,)):
        return "failed", "nonzero_exit_" + str(code)
    return "graceful", None


def classify_all(records):
    results = []
    for record in records:
        classification, reason = classify(record)
        results.append(dict(record, classification=classification, reason=reason))
    failures = [r["name"] + ":" + r["reason"] for r in results if r["classification"] != "graceful"]
    return {"processes": results, "failures": failures, "status": "passed" if not failures and results else "failed"}
