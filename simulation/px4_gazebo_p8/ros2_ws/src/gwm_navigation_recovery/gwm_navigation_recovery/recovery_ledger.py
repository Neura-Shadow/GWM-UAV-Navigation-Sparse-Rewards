"""P8 recovery ledger and parameter loading for the P8 nodes (contract `p8-navigation-recovery-v1` §3 S10, §9).

* **`load_params`:** reads `p8_recovery.json` and refuses to start unless its canonical SHA-256 (sorted keys, compact
  JSON, as `p3_provenance.canonical_sha256` computes it) equals the approved hash the runner passes in. The check
  runs before any ROS node exists.
* **`RecoveryLedger`:**
  * writes `p8/recovery-ledger.jsonl` (exclusive create), opening with a header record of the parameter hash;
  * one JSON line per pending record, written after the tick's dispatch, line-buffered;
  * flushed and `fsync`ed when the node finishes;
  * its summary goes to `controller-result.json` under `p8`.
"""
import hashlib
import json
import os
from pathlib import Path

from .recovery import LEDGER


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def load_params(path, expected_sha256):
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if canonical_sha256(value) != expected_sha256:
        raise ValueError("p8_recovery_parameters_differ_from_the_approved_hash")
    return value


class RecoveryLedger:
    def __init__(self, path, params_sha256):
        self.path, self.params_sha256 = Path(path), params_sha256
        self.stream = self.path.open("x", buffering=1)
        self.records = 0
        self.closed = None
        self._line({"ledger": LEDGER, "kind": "header", "params_sha256": params_sha256})

    def _line(self, record):
        self.stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
        self.records += 1

    def write(self, pending):
        while pending:
            self._line(pending.pop(0))

    def close(self):
        if self.closed is None:
            self.stream.flush()
            os.fsync(self.stream.fileno())
            self.stream.close()
            self.closed = {"path": str(self.path), "records": self.records, "fsynced": True, "params_sha256": self.params_sha256}
        return self.closed
