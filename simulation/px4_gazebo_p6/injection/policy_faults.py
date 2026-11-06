"""Policy-path fault transforms (F3, F6.03, F7.02, F8.01), pure apart from the declared `sleep`.

They act around the accepted `FrozenPolicy.scores` and `candidate_ranker.policy_choice`. A delay is a
real `sleep` inside the scored call, so the accepted `PolicyRanker` clock measures it itself; nothing
here writes the policy ledger.
"""
import numpy as np

from gwm_obstacle_avoidance import planner

from .schedule import InvocationPlan


def plan_for(fault_id, parameters):
    if fault_id in ("F3.04", "F3.08"):
        return InvocationPlan("consecutive", count=parameters["consecutive_invocations"])
    if fault_id == "F3.07":
        return InvocationPlan("offsets", offsets=parameters["invocation_offsets_sim_s"])
    if fault_id == "F7.02":
        return InvocationPlan("alternating")
    if fault_id in ("F3.09", "F3.10"):
        return InvocationPlan("consecutive", count=1)
    return InvocationPlan("all")  # F3.05, F3.06, F3.11, F3.12, F3.13, F8.01: every invocation from the trigger


def worst_allowed(scores, allowed, rows_by_slot):
    """F3.13: the minimum score among allowed slots, P4 tie order inside 1e-9 (mirrors `policy_choice`)."""
    scores = np.asarray(scores, dtype=np.float64)
    worst = scores[allowed].min()
    tied = [int(i) for i in np.flatnonzero(allowed) if scores[i] <= worst + 1e-9]
    return min(tied, key=lambda i: planner.tie_key(rows_by_slot[i]))


def outside_mask(allowed):
    """F3.06: a slot the mask forbids (the accepted ranker must classify it `selection_outside_mask`)."""
    forbidden = np.flatnonzero(~np.asarray(allowed, dtype=bool))
    return int(forbidden[0]) if forbidden.size else -1


def corrupt_scores(fault_id, scores):
    if fault_id == "F3.04":
        return np.full_like(np.asarray(scores, dtype=np.float64), np.nan)
    if fault_id == "F3.05":
        return np.asarray(scores, dtype=np.float64)[:-1]
    return scores


def corrupt_features(features):
    """F3.12: one NaN in the feature tensor; the accepted `scores` must refuse it (`input_not_finite`)."""
    out = np.array(features, dtype=np.float64, copy=True)
    out[0, 0] = np.nan
    return out
