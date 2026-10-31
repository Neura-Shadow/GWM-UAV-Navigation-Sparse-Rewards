"""Selection hooks for the P5 planner replica (contract §3.3, §5).

`PolicyRanker` scores the P4 candidate slots with the frozen policy, masks everything outside the
P4 progress set and returns the masked argmax (P4 L3 tie order inside 1e-9). Any failure returns no
choice and a classified record; it never falls back to another ranking. The P4 choice is computed
in shadow for logging only. `P4Ranker` is P4's own L3 selection, used for parity tests.
"""
import time

import numpy as np

from gwm_obstacle_avoidance import planner

from . import observation_adapter
from .contracts import SLOTS
from .policy_runtime import PolicyError

TIE_EPSILON = 1e-9


def policy_choice(scores, allowed, rows_by_slot):
    scores = np.asarray(scores, dtype=np.float64)
    if scores.shape != (SLOTS,):
        raise PolicyError("output_shape")
    if not allowed.any():
        raise PolicyError("empty_mask")
    if not np.isfinite(scores[allowed]).all():
        raise PolicyError("output_not_finite")
    best = scores[allowed].max()
    tied = [int(i) for i in np.flatnonzero(allowed) if scores[i] >= best - TIE_EPSILON]
    return min(tied, key=lambda i: planner.tie_key(rows_by_slot[i]))


class P4Ranker:
    """P4's L3 selection behind the P5 hook (parity reference; never used for policy credit)."""

    def choose(self, rows, progress, ctx, grid, traversed, state, settings):
        return planner.select(progress, settings, grid.resolution), None


class PolicyRanker:
    def __init__(self, policy, schema, failure_wall_s=0.020, clock=time.perf_counter):
        self.policy, self.schema, self.failure_wall_s, self.clock = policy, schema, failure_wall_s, clock

    def choose(self, rows, progress, ctx, grid, traversed, state, settings):
        shadow = planner.select(progress, settings, grid.resolution)
        record = {"invoked": True, "policy_sha256": getattr(self.policy, "sha256", None), "state": state,
                  "features": None, "scores": None, "allowed": None, "choice_slot": None,
                  "p4_slot": observation_adapter.slot_of(shadow, settings), "failure": None, "inference_s": None}
        start = self.clock()
        try:
            if self.policy is None:
                raise PolicyError("checkpoint_missing")
            features, allowed = observation_adapter.build(rows, ctx, settings, grid, traversed, state, self.schema)
            by_slot = {observation_adapter.slot_of(r, settings): r for r in rows}
            if {observation_adapter.slot_of(r, settings) for r in progress} != set(np.flatnonzero(allowed).tolist()):
                raise PolicyError("mask_mismatch")
            scores = self.policy.scores(features)
            choice = policy_choice(scores, allowed, by_slot)
            if not allowed[choice]:
                raise PolicyError("selection_outside_mask")
            record.update(features=features, scores=np.asarray(scores, dtype=np.float64), allowed=allowed, choice_slot=choice)
        except PolicyError as exc:
            record["failure"] = exc.reason
        except Exception as exc:  # any other fault fails closed, classified
            record["failure"] = "exception:" + type(exc).__name__
        record["inference_s"] = self.clock() - start
        if record["failure"] is None and record["inference_s"] > self.failure_wall_s:
            record["failure"] = "timeout"
        if record["failure"] is not None:
            return None, record
        return by_slot[record["choice_slot"]], record
