"""Per-tick policy ledger (contract §5 "Recorded per decision tick", §8 independent evaluation).

One record per TRANSIT supervisor step, whether or not the policy was invoked. An invoked record
carries the exact feature tensor (little-endian float64, base64) and the raw scores, so the evaluator
can re-run the frozen model and require bit-identical scores, recompute the masked argmax, and check
that the executed segment or heading is the policy's choice. JSON floats round-trip exactly.
"""
import base64

import numpy as np

LEDGER_CONTRACT = "p5-policy-ledger-v1"
NO_INTERVENTION = None


def encode_array(array):
    value = np.ascontiguousarray(array, dtype="<f8")
    return {"dtype": "<f8", "shape": list(value.shape), "b64": base64.b64encode(value.tobytes()).decode("ascii")}


def decode_array(encoded):
    if encoded["dtype"] != "<f8":
        raise ValueError("ledger_dtype")
    value = np.frombuffer(base64.b64decode(encoded["b64"], validate=True), dtype="<f8")
    return value.reshape(encoded["shape"]).astype(np.float64)


def intervention(out, record):
    """What the shield did to the policy's choice: None when the chosen segment is executed as chosen."""
    if record is None:
        return NO_INTERVENTION
    if record["failure"] is not None:
        return "policy_failure_hold"
    best = (out["plan"] or {}).get("best")
    if out["action"] == "TRANSLATE" and best and out["heading"] == best["heading"] and out["end"] == best["end"]:
        return NO_INTERVENTION
    if out["action"] == "YAW_TO" and best and out["heading"] == best["heading"]:
        return "yaw_to_l4"
    if out["action"] == "HOLD":
        return "hold:" + str(out["reason"])
    return "overridden:" + str(out["action"])


def entry(tick, sim, out):
    """The ledger record for one supervisor output (`PolicySupervisor.step`)."""
    record = out["policy"]
    best = (out["plan"] or {}).get("best")
    result = {"ledger": LEDGER_CONTRACT, "tick": tick, "sim_s": sim, "state": out["state"], "action": out["action"],
              "heading": out["heading"], "end": list(out["end"]) if out["end"] is not None else None,
              "length": out["length"], "reason": out["reason"], "best": best, "invoked": record is not None,
              "intervention": intervention(out, record)}
    if record is not None:
        scores, allowed = record.get("scores"), record.get("allowed")
        result.update(
            policy_sha256=record.get("policy_sha256"), planning_state=record.get("state"),
            choice_slot=None if record.get("choice_slot") is None else int(record["choice_slot"]),
            p4_slot=None if record.get("p4_slot") is None else int(record["p4_slot"]),
            failure=record["failure"], inference_s=record.get("inference_s"),
            scores=None if scores is None else [float(s) for s in scores],
            allowed=None if allowed is None else [bool(a) for a in allowed],
            features=None if record.get("features") is None else encode_array(record["features"]))
    return result
