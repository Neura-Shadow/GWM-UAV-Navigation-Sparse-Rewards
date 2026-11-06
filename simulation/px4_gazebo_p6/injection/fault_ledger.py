"""Append-only fault ledger `p6/fault-ledger-<injector>.jsonl`: what an injector declared, armed, did and finished.

One file per injector process (the relays and the controller shim run in different processes).

The ledger is the injector's own account. The evaluator uses it to know *where* to look, and then
requires independent evidence (witness, native source probe, ULog, the accepted controller's ledgers,
the process table) that the fault actually happened (contract C8).
"""
import json
import threading
import time
from pathlib import Path

CONTRACT = "p6-fault-ledger-v1"
KINDS = ("declared", "armed", "activated", "affected", "cleared", "finished")


class FaultLedger:
    def __init__(self, path, row_id, fault_id, injector, clock=time.monotonic):
        self.stream = Path(path).open("x", buffering=1)  # exclusive: a run never appends to an old ledger
        self.row_id, self.fault_id, self.injector, self.clock = row_id, fault_id, injector, clock
        self.lock = threading.Lock()
        self.seq = 0

    def record(self, kind, sim_s=None, **fields):
        if kind not in KINDS:
            raise ValueError("unknown fault ledger kind: " + str(kind))
        with self.lock:
            self.seq += 1
            entry = {"contract": CONTRACT, "seq": self.seq, "kind": kind, "row": self.row_id, "fault": self.fault_id,
                     "injector": self.injector, "sim_s": sim_s, "wall_s": self.clock(), **fields}
            self.stream.write(json.dumps(entry, allow_nan=False, separators=(",", ":")) + "\n")
            return entry

    def close(self):
        with self.lock:
            self.stream.close()


def read(path):
    """Every record, after checking the contract, the sequence and that the ledger starts with `declared`."""
    records = [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line]
    if not records or records[0]["kind"] != "declared":
        raise ValueError("fault ledger does not start with a declaration")
    by_injector = {}
    for record in records:
        if record.get("contract") != CONTRACT or record.get("kind") not in KINDS:
            raise ValueError("invalid fault ledger record")
        expected = by_injector.get(record["injector"], 0) + 1
        if record["seq"] != expected:
            raise ValueError("fault ledger sequence gap for " + record["injector"])
        by_injector[record["injector"]] = expected
    return records
