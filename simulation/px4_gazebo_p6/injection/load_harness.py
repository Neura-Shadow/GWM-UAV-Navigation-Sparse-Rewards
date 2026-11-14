"""Bounded load harness (LH, decision D8): CPU busy workers and one memory worker, inside the namespace.

No host or WSL setting is changed. The approved bounds (at most 24 workers, at most 60 s, memory at
most 25 % of the free RAM) are enforced here and again at declaration time. Workers stop themselves
at their deadline.

M6 (operator decision 2026-11-14, plan §16.10-16.11): the window can outlive the stack (a C-5 flight
ends about 55 s after its load starts, sooner after an abort), so the runner's cleanup stops the
harness with SIGINT. A stop request (SIGINT or SIGTERM) ends the workers, records `cleared` with
`stopped_early` naming the signal, and exits 0; the workers ignore SIGINT, so a stop sent to the
whole process group reaches them only through the harness.
"""
import argparse
import multiprocessing
import signal
import threading
import time

from . import specs

STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM)


class _Stop(Exception):
    """A stop request from the runner's cleanup."""


def _worker_signals():
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)


def _uninterrupted(fn, *args, **kwargs):
    """A ledger line is written whole: a stop request is delivered right after it, never inside it."""
    if not hasattr(signal, "pthread_sigmask"):
        return fn(*args, **kwargs)
    mask = signal.pthread_sigmask(signal.SIG_BLOCK, set(STOP_SIGNALS))
    try:
        return fn(*args, **kwargs)
    finally:
        signal.pthread_sigmask(signal.SIG_SETMASK, mask)


def bounds():
    return specs.codes()["LOAD"]


def check(workers, seconds, memory_fraction=0.0):
    load = bounds()
    if not 0 <= workers <= load["max_workers"]:
        raise ValueError("load workers outside the approved bound")
    if not 0 < seconds <= load["max_window_s"]:
        raise ValueError("load window outside the approved bound")
    if not 0.0 <= memory_fraction <= load["max_memory_fraction_of_free"]:
        raise ValueError("memory fraction outside the approved bound")


def _busy(deadline):
    _worker_signals()
    x = 0
    while time.monotonic() < deadline:
        x = (x * 1103515245 + 12345) & 0x7FFFFFFF  # pure CPU, no I/O


def _memory(nbytes, deadline):
    _worker_signals()
    block = bytearray(nbytes)
    for i in range(0, nbytes, 4096):  # touch every page so the memory is really resident
        block[i] = 1
    while time.monotonic() < deadline:
        time.sleep(0.05)


def free_bytes():
    with open("/proc/meminfo", encoding="ascii") as stream:
        for line in stream:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) * 1024
    raise RuntimeError("MemAvailable not found")


def run(workers, seconds, memory_fraction=0.0, ledger=None, sim_s=None):
    check(workers, seconds, memory_fraction)
    deadline = time.monotonic() + seconds
    context = multiprocessing.get_context("fork")
    procs = [context.Process(target=_busy, args=(deadline,), daemon=True) for _ in range(workers)]
    memory = int(free_bytes() * memory_fraction) if memory_fraction else 0
    if memory:
        procs.append(context.Process(target=_memory, args=(memory, deadline), daemon=True))
    record = {"workers": workers, "seconds": seconds, "memory_bytes": memory, "pids": [], "start_wall_s": None}
    stopped = []

    def stop(signum, _frame):
        stopped.append(signal.Signals(signum).name)
        raise _Stop()

    handlers = threading.current_thread() is threading.main_thread()  # signal handlers live in the main thread only
    previous = {s: signal.signal(s, stop) for s in STOP_SIGNALS} if handlers else {}
    try:
        try:
            for proc in procs:
                proc.start()
            record.update(pids=[p.pid for p in procs], start_wall_s=time.monotonic())
            if ledger is not None:
                _uninterrupted(ledger.record, "activated", sim_s, **record)
            for proc in procs:
                proc.join(timeout=max(0.0, deadline - time.monotonic()) + 5.0)
                if proc.is_alive():
                    proc.kill()
        except _Stop:
            for s in previous:  # one stop request is enough; the rest of the cleanup is not interrupted
                signal.signal(s, signal.SIG_IGN)
            started = [p for p in procs if p.pid is not None]
            for proc in started:
                if proc.is_alive():
                    proc.terminate()
            for proc in started:
                proc.join(timeout=2.0)
                if proc.is_alive():
                    proc.kill()
                    proc.join(timeout=1.0)
        try:  # the harness is finishing: a late stop request must not cut the `cleared` record
            for s in previous:
                signal.signal(s, signal.SIG_IGN)
        except _Stop:  # it arrived just as the workers ended by themselves
            for s in previous:
                signal.signal(s, signal.SIG_IGN)
        record.update(end_wall_s=time.monotonic(), exit_codes=[p.exitcode for p in procs],
                      stopped_early=stopped[0] if stopped else None)
        if ledger is not None:
            ledger.record("cleared", sim_s, **record)
    finally:
        for s, handler in previous.items():
            signal.signal(s, handler)
    return record


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, required=True)
    parser.add_argument("--seconds", type=float, required=True)
    parser.add_argument("--memory-fraction", type=float, default=0.0)
    parser.add_argument("--ledger", help="M6: the fault ledger this harness records its activation and clearing in")
    parser.add_argument("--row")
    parser.add_argument("--fault")
    parser.add_argument("--sim-s", type=float, help="the trigger's simulation time, as the runner read it")
    args = parser.parse_args(argv)
    ledger = None
    if args.ledger:
        from .fault_ledger import FaultLedger
        ledger = FaultLedger(args.ledger, args.row, args.fault, "load_harness")
    try:
        run(args.workers, args.seconds, args.memory_fraction, ledger=ledger, sim_s=args.sim_s)
    finally:
        if ledger is not None:
            ledger.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
