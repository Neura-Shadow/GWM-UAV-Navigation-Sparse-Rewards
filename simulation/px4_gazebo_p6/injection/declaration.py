"""Per-run fault declaration `p6/fault-declaration.json`, built only from a frozen scenario row.

An injector refuses any declaration that does not match the frozen specification byte for byte in its
row content, so a run cannot quietly inject something other than what the operator approved.
"""
import json
from pathlib import Path

from . import specs

CONTRACT = "p6-fault-declaration-v1"
MODES = ("off", "pass-through", "active")


def _mechanisms(fault):
    mechanism = fault.get("mechanism")
    if mechanism is None or mechanism.startswith("via "):
        return []
    return mechanism.split("+")


def build(row_id, mode=None):
    """`mode=None` is the row's own mode: `active` for a fault row, `pass-through` for a row without one."""
    gate, row = specs.row(row_id)
    fault_id = row.get("fault")
    if mode is None:
        mode = "active" if fault_id is not None else "pass-through"
    if mode not in MODES:
        raise ValueError("unknown injector mode: " + str(mode))
    if fault_id is None:  # pass-through rows (A-3, B-0): every listed injector present, none faulting
        injectors = dict(row.get("injectors") or {})
        if mode == "active" or any(value != "pass-through" for value in injectors.values()):
            raise ValueError("a row without a fault can only run its injectors in pass-through")
        mechanisms = sorted(injectors)
        mode = "pass-through" if injectors else "off"
    else:
        fault = specs.faults()[fault_id]
        mechanisms = _mechanisms(fault)
    parameters = dict(row.get("parameters") or {})
    _check_bounds(parameters)
    return {"contract": CONTRACT, "gate": gate, "row": row_id, "row_sha": _row_digest(row), "fault": fault_id,
            "mechanisms": mechanisms, "mode": mode, "trigger": row.get("trigger"), "parameters": parameters}


def _row_digest(row):
    import hashlib
    return hashlib.sha256(json.dumps(row, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _check_bounds(parameters):
    """The approved load bounds (D8) are enforced at declaration time, not only by the harness."""
    load = specs.codes()["LOAD"]
    if parameters.get("load_workers", 0) > load["max_workers"]:
        raise ValueError("load workers above the approved bound")
    if parameters.get("load_window_s", 0.0) > load["max_window_s"]:
        raise ValueError("load window above the approved bound")


def validate(declaration):
    """Rebuild from the frozen row and require equality: the declaration is the frozen row, nothing else."""
    if declaration.get("contract") != CONTRACT:
        raise ValueError("not a P6 fault declaration")
    if declaration != build(declaration["row"], declaration["mode"]):
        raise ValueError("fault declaration differs from the frozen scenario row " + declaration["row"])
    return declaration


def uses(declaration, mechanism):
    """The mode this injector must run in: `off` unless the declaration names its mechanism."""
    return declaration["mode"] if mechanism in declaration["mechanisms"] else "off"


def write(path, declaration):
    with Path(path).open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(validate(declaration), indent=2, allow_nan=False) + "\n")


def load(path):
    return validate(specs.strict_json(path))
