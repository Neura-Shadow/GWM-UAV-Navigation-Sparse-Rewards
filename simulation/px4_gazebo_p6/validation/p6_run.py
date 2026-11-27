"""Read-only view of one P6 run directory (M4).

A P6 run has the accepted P5 run layout (the P6 runner subclasses the P5 runner) plus `p6/`:
* `p6/fault-declaration.json` — the frozen row the run executed;
* `p6/fault-ledger-<injector>.jsonl` — each injector's own account (never trusted alone);
* `p6/witness/<stream>.jsonl` — the passive witness;
* `p6/injection.json` — the runner's activation record (trigger time, process signal, request time).
A depth-bridge stop also writes the accepted P4 record `p4/fault_injection.json`, so the unchanged P4
§9.1 window function evaluates it.
"""
import functools
import json
from pathlib import Path


def jsonl(path):
    path = Path(path)
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def strict(path):
    def reject(value):
        raise ValueError("non-finite constant " + value)
    return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=reject)


class Run:
    def __init__(self, path):
        self.path = Path(path)

    def file(self, name):
        return self.path / name

    def exists(self, name):
        return (self.path / name).exists()

    def optional(self, name):
        return strict(self.path / name) if self.exists(name) else None

    @functools.cached_property
    def summary(self):
        return self.optional("summary.json") or {}

    @functools.cached_property
    def result(self):
        return self.summary.get("controller_result") or self.optional("controller-result.json") or {}

    @functools.cached_property
    def events(self):
        return jsonl(self.path / "ros-events.jsonl")

    @functools.cached_property
    def decisions(self):
        return [e for e in self.events if e.get("event") == "p4_decision"]

    @functools.cached_property
    def setpoints(self):
        return [e for e in self.events if e.get("event") == "setpoint"]

    @functools.cached_property
    def commands(self):
        return [e for e in self.events if e.get("event") == "command_sent"]

    @functools.cached_property
    def transitions(self):
        return (self.result.get("p4") or {}).get("supervisor_transitions") or []

    @functools.cached_property
    def ledger(self):
        return jsonl(self.path / "p5/policy-ledger.jsonl")

    @functools.cached_property
    def local_map(self):
        return jsonl(self.path / "p4/local_map.jsonl")

    @functools.cached_property
    def health(self):
        return jsonl(self.path / "sensors/health.jsonl")

    @functools.cached_property
    def p5_evaluation(self):
        return self.optional("p5-evaluation.json")

    @functools.cached_property
    def declaration(self):
        return self.optional("p6/fault-declaration.json")

    @functools.cached_property
    def injection(self):
        return self.optional("p6/injection.json") or self.optional("p4/fault_injection.json")

    def fault_ledger(self, injector):
        return jsonl(self.path / "p6" / ("fault-ledger-%s.jsonl" % injector))

    def fault_ledgers(self):
        folder = self.path / "p6"
        return {p.stem[len("fault-ledger-"):]: jsonl(p) for p in sorted(folder.glob("fault-ledger-*.jsonl"))} if folder.exists() else {}

    def witness(self, stream):
        return jsonl(self.path / "p6/witness" / (stream + ".jsonl"))

    def received(self, topic_key):
        return [e for e in self.events if e.get("event") == "received" and e.get("topic_key") == topic_key]
