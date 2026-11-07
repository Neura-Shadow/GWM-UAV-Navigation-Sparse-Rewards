"""Read-only access to the frozen P6 specifications (M1): fault catalogue, outcome codes and scenario rows."""
import json
from pathlib import Path

P6 = Path(__file__).resolve().parents[1]
FAULTS = P6 / "configs/p6_faults.yaml"
SCENARIOS = P6 / "configs/p6_scenarios.yaml"


def strict_json(path):
    def reject(value):
        raise ValueError("non-finite constant " + value)
    return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=reject)


def faults():
    return {fault["id"]: fault for fault in strict_json(FAULTS)["faults"]}


def codes():
    return strict_json(FAULTS)["codes"]


def row(row_id):
    """(gate, row) for a gate row, or ("X", row) for a characterization flight."""
    scenarios = strict_json(SCENARIOS)
    for gate, rows in scenarios["gates"].items():
        for item in rows:
            if item["id"] == row_id:
                return gate, item
    for item in scenarios["characterization"]["flights"]:
        if item["id"] == row_id:
            return "X", item
    raise KeyError("unknown P6 scenario row: " + str(row_id))
