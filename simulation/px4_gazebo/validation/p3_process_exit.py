"""Strict graceful-cleanup classification for every owned P3 process.

A process is graceful only when it was asked to stop by its declared method
(or completed its own bounded work), was reaped, exited with a declared
numeric success code within its grace period, needed no escalation and left
no member of its process group behind. Reaped is not graceful; signal deaths
(SIGTERM, SIGKILL, SIGSEGV, ...) are never graceful, including for an
intentionally interrupted producer.
"""
import signal

STOP_METHODS = ("SIGINT", "SIGTERM", "console_shutdown", "completes_own_work")


def signal_name(code):
    try:
        return signal.Signals(-code).name
    except ValueError:
        return "signal_" + str(-code)


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
