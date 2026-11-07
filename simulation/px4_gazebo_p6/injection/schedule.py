"""Pure trigger and timing logic shared by every injector (no ROS, no clocks of its own).

Triggers are evaluated on the accepted controller's own decision events (`p4_decision`, the P2
`transition` records), exactly the stream the P4 S6 injection reads. Fault windows and invocation
plans are relative to the trigger's simulation time.
"""
import math

TRIGGERS = ("ground_window", "transit_start", "avoiding_first", "avoiding_near", "normal_2m")
AVOIDING_NEAR_M = 1.5
NORMAL_DISTANCE_M = 2.0


class Trigger:
    """Fires once. `ground_window` is fired by the runner; the others by `feed(event)`."""

    def __init__(self, name, boxes=()):
        if name not in TRIGGERS:
            raise ValueError("unknown trigger: " + str(name))
        self.name, self.boxes = name, [tuple(box["center"]) for box in boxes]
        self.origin_enu = None
        self.fired_sim = None

    def fire(self, sim):
        if self.fired_sim is None:
            self.fired_sim = sim
        return self.fired_sim

    def feed(self, event):
        """One controller event; returns the trigger time once it has fired, else None."""
        if self.fired_sim is not None:
            return self.fired_sim
        if event.get("event") == "transition" and event.get("to") == "PRESTREAM_SAFE_SETPOINTS":
            origin = (event.get("evidence") or {}).get("origin_ned")
            if origin is not None:
                self.origin_enu = (float(origin[1]), float(origin[0]))
        if event.get("event") != "p4_decision" or self.name == "ground_window":
            return None
        state, sim = event.get("state"), event["sim_s"]
        if self.name == "transit_start":
            return self.fire(sim)
        if self.name == "avoiding_first" and state == "AVOIDING":
            return self.fire(sim)
        local = self._local(event.get("vehicle_enu"))
        if local is None:
            return None
        if self.name == "avoiding_near" and state == "AVOIDING" and self.boxes:
            if min(math.dist(local, box) for box in self.boxes) <= AVOIDING_NEAR_M:
                return self.fire(sim)
        if self.name == "normal_2m" and state == "NORMAL" and math.hypot(*local) >= NORMAL_DISTANCE_M:
            return self.fire(sim)
        return None

    def _local(self, vehicle_enu):
        """Vehicle position relative to the take-off origin (the world origin, where the vehicle spawns)."""
        if vehicle_enu is None or self.origin_enu is None:
            return None
        return vehicle_enu[0] - self.origin_enu[0], vehicle_enu[1] - self.origin_enu[1]


class Windows:
    """Fault-active windows [start, start + duration) relative to the trigger; `until_end` = one open window."""

    def __init__(self, offsets=(0.0,), durations=(math.inf,)):
        if len(offsets) != len(durations) or any(d <= 0 for d in durations):
            raise ValueError("invalid fault windows")
        self.windows = sorted(zip(offsets, durations))

    def active(self, relative_sim):
        return any(start <= relative_sim < start + duration for start, duration in self.windows)

    def last_end(self):
        return max(start + duration for start, duration in self.windows)


class InvocationPlan:
    """Which policy invocations after the trigger are faulted.

    * `consecutive`: the first N invocations at or after the trigger;
    * `offsets`: the first invocation at or after each trigger + offset (isolated faults);
    * `alternating`: every second invocation (F7.02);
    * `all`: every invocation after the trigger.
    """

    def __init__(self, kind, count=None, offsets=()):
        if kind not in ("consecutive", "offsets", "alternating", "all"):
            raise ValueError("unknown invocation plan: " + str(kind))
        self.kind, self.count, self.offsets = kind, count, sorted(offsets)
        self.index = 0
        self.pending = list(self.offsets)

    def faulted(self, relative_sim):
        """Call once per policy invocation at or after the trigger; returns whether it is faulted."""
        self.index += 1
        if self.kind == "consecutive":
            return self.index <= self.count
        if self.kind == "alternating":
            return self.index % 2 == 1
        if self.kind == "all":
            return True
        if self.pending and relative_sim >= self.pending[0]:
            while self.pending and relative_sim >= self.pending[0]:
                self.pending.pop(0)
            return True
        return False
