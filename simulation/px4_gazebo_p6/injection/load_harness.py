"""Bounded load harness (LH, decision D8): CPU busy workers and one memory worker, inside the namespace.

No host or WSL setting is changed. The approved bounds (at most 24 workers, at most 60 s, memory at
most 25 % of the free RAM) are enforced here and again at declaration time. Workers stop themselves
at their deadline, so the load always ends before the stack does.
"""
import argparse
import multiprocessing
import time

from . import specs


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
    x = 0
    while time.monotonic() < deadline:
        x = (x * 1103515245 + 12345) & 0x7FFFFFFF  # pure CPU, no I/O


def _memory(nbytes, deadline):
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
    for proc in procs:
        proc.start()
    record = {"workers": workers, "seconds": seconds, "memory_bytes": memory, "pids": [p.pid for p in procs],
              "start_wall_s": time.monotonic()}
    if ledger is not None:
        ledger.record("activated", sim_s, **record)
    for proc in procs:
        proc.join(timeout=max(0.0, deadline - time.monotonic()) + 5.0)
        if proc.is_alive():
            proc.kill()
    record.update(end_wall_s=time.monotonic(), exit_codes=[p.exitcode for p in procs])
    if ledger is not None:
        ledger.record("cleared", sim_s, **record)
    return record


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, required=True)
    parser.add_argument("--seconds", type=float, required=True)
    parser.add_argument("--memory-fraction", type=float, default=0.0)
    args = parser.parse_args(argv)
    run(args.workers, args.seconds, args.memory_fraction)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
