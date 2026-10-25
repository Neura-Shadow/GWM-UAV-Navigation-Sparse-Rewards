"""Bounded PX4 state history and deterministic acquisition-time association.

No interpolation or extrapolation. The closest strictly-recorded sample within
the declared mismatch is used; an exact equidistant tie deterministically takes
the earlier (causal) sample and is flagged. Epoch-crossing brackets, empty history
and out-of-tolerance requests are reported explicitly, never invented.
"""
from collections import deque
import math

RESET_KEYS = ("xy_reset_counter", "z_reset_counter", "heading_reset_counter", "ref_timestamp")


class StateHistory:
    def __init__(self, name, capacity, mismatch_s, epoch_keys=()):
        self.name = name
        self.samples = deque(maxlen=capacity)
        self.mismatch_us = int(round(mismatch_s * 1e6))
        self.epoch_keys = epoch_keys
        self.counts = {"received": 0, "stored": 0, "duplicate": 0, "backwards": 0, "evicted": 0}
        self.last_used = None

    def add(self, timestamp_us, value):
        self.counts["received"] += 1
        if isinstance(timestamp_us, bool) or not isinstance(timestamp_us, int) or timestamp_us <= 0:
            raise ValueError("invalid_state_timestamp:" + self.name)
        if self.samples and timestamp_us == self.samples[-1][0]:
            self.counts["duplicate"] += 1
            return False
        if self.samples and timestamp_us < self.samples[-1][0]:
            self.counts["backwards"] += 1
            return False
        if len(self.samples) == self.samples.maxlen:
            self.counts["evicted"] += 1
        self.samples.append((timestamp_us, value))
        self.counts["stored"] += 1
        return True

    def epoch(self, value):
        return tuple(value.get(key) for key in self.epoch_keys)

    def match(self, acquisition_us):
        if not self.samples:
            return {"status": "missing_history"}
        oldest, newest = self.samples[0][0], self.samples[-1][0]
        if acquisition_us < oldest - self.mismatch_us:
            return {"status": "older_than_history", "oldest_us": oldest}
        best = None
        tie = False
        for index, (stamp, value) in enumerate(self.samples):
            delta = abs(stamp - acquisition_us)
            if best is None or delta < best[0]:
                best, tie = (delta, index), False
            elif delta == best[0]:
                tie = True
        delta, index = best
        stamp, value = self.samples[index]
        if delta > self.mismatch_us:
            status = "pending_newer_state" if acquisition_us > newest else "no_state_within_tolerance"
            return {"status": status, "nearest_delta_s": delta / 1e6}
        # Samples are strictly increasing, so on a tie `best` is already the earlier sample;
        # both candidates have the same |delta|, which is within the unchanged tolerance.
        # A bracket spanning a reset epoch cannot be associated with confidence.
        if self.epoch_keys:
            before = [(s, v) for s, v in self.samples if s <= acquisition_us][-1:]
            after = [(s, v) for s, v in self.samples if s >= acquisition_us][:1]
            if before and after and self.epoch(before[0][1]) != self.epoch(after[0][1]):
                # Contract v1.4: record the bracketing samples so the evaluator can verify
                # (and independently corroborate) the single PX4 reset; the decision is unchanged.
                return {"status": "reset_epoch_bracket",
                        "before": {"timestamp_us": before[0][0], "epoch": dict(zip(self.epoch_keys, self.epoch(before[0][1])))},
                        "after": {"timestamp_us": after[0][0], "epoch": dict(zip(self.epoch_keys, self.epoch(after[0][1])))}}
        reused = self.last_used == stamp
        self.last_used = stamp
        return {"status": "matched", "timestamp_us": stamp, "delta_s": (stamp - acquisition_us) / 1e6,
                "reused": reused, "equidistant_tie_earlier": tie, "value": value}


class Associator:
    """Joint position + attitude association for one acquisition timestamp."""

    def __init__(self, capacity, mismatch_s):
        self.position = StateHistory("vehicle_local_position", capacity, mismatch_s, RESET_KEYS)
        self.attitude = StateHistory("vehicle_attitude", capacity, mismatch_s, ("quat_reset_counter",))
        self.outcomes = {}

    def add_position(self, message):
        value = {k: message[k] for k in ("x", "y", "z", "heading", "xy_valid", "z_valid") + RESET_KEYS}
        return self.position.add(int(message["timestamp_sample"] or message["timestamp"]), value)

    def add_attitude(self, message):
        q = [float(v) for v in message["q"]]
        if not all(math.isfinite(v) for v in q):
            return False
        return self.attitude.add(int(message["timestamp_sample"] or message["timestamp"]),
                                 {"q": q, "quat_reset_counter": message["quat_reset_counter"]})

    def associate(self, acquisition_us):
        position = self.position.match(acquisition_us)
        attitude = self.attitude.match(acquisition_us)
        status = "matched" if position["status"] == attitude["status"] == "matched" else (
            "position:" + position["status"] if position["status"] != "matched" else "attitude:" + attitude["status"])
        self.outcomes[status] = self.outcomes.get(status, 0) + 1
        result = {"status": status, "position": position, "attitude": attitude}
        if status == "matched":
            p, a = position["value"], attitude["value"]
            valid = p["xy_valid"] is True and p["z_valid"] is True
            result.update(position_ned=[p["x"], p["y"], p["z"]], q_wxyz=a["q"], estimate_valid=valid,
                          reset_epoch={k: p[k] for k in RESET_KEYS} | {"quat_reset_counter": a["quat_reset_counter"]},
                          reused=position["reused"] or attitude["reused"])
            if not valid:
                result["status"] = "estimate_invalid"
        for part in (result["position"], result["attitude"]):
            part.pop("value", None)
        return result
