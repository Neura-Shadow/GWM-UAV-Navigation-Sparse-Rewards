"""PX4 state, ACK and map-input fault transforms (F2, F4 map/health, F5), pure.

* `StateInjector` acts on PX4 `fmu/out` state dicts, either in the state relay to perception (SR) or
  inside the controller shim before `StateCache.update` (CS, app-side only: PX4's real estimate is
  unchanged).
* `AckInjector` rewrites or drops `vehicle_command_ack` before `Mission.ack` (app view only).
* `MapInjector` acts on the local-map snapshots and adapter-health records the controller receives.

In pass-through every call returns its input object unchanged.
"""
import math

from .schedule import Windows


def _windows(parameters, key="window_sim_s"):
    duration = parameters.get(key)
    return Windows() if duration is None else Windows((0.0,), (duration,))


class _Timed:
    def __init__(self, fault_id, parameters, mode):
        self.fault_id, self.p, self.mode = fault_id, dict(parameters), mode
        self.trigger_sim = None
        self.affected = []

    def activate(self, sim):
        if self.trigger_sim is None:
            self.trigger_sim = sim

    def active(self, now_sim, windows):
        return self.mode == "active" and self.trigger_sim is not None and windows.active(now_sim - self.trigger_sim)


class StateInjector(_Timed):
    """Returns the list of state dicts to deliver now for one incoming (topic key, dict)."""

    def __init__(self, fault_id, parameters, mode):
        super().__init__(fault_id, parameters, mode)
        duration = parameters.get("withhold_s", parameters.get("window_sim_s"))
        self.windows = Windows() if duration is None else Windows((0.0,), (duration,))
        self.withheld = set(parameters.get("withheld_topics") or
                            (["vehicle_local_position"] if fault_id == "F2.01" else []))
        self.fired = False

    def state(self, key, data, now_sim):
        if not self.active(now_sim, self.windows):
            return [data]
        f = self.fault_id
        if f in ("F2.01", "F2.09", "F5.07") and (key in self.withheld or (f == "F5.07" and key == "vehicle_status")):
            self.affected.append((key, now_sim, "withheld"))
            return []
        if key != "vehicle_local_position" and f in ("F2.02", "F2.04", "F2.05", "F2.06", "F2.07", "F2.08"):
            return [data]
        if f == "F2.02":
            out = dict(data, x=data["x"] + self.p["dx_m"], y=data["y"] + self.p["dy_m"])
        elif f == "F2.04":
            if self.fired:
                return [data]
            self.fired = True
            out = dict(data, x=data["x"] + self.p["dx_m"], y=data["y"] + self.p["dy_m"],
                       xy_reset_counter=data["xy_reset_counter"] + 1)
        elif f == "F2.05":
            if self.fired:
                return [data]
            self.fired = True
            out = dict(data, timestamp=data["timestamp"] - self.p["backwards_us"])
        elif f == "F2.06":
            out = dict(data, timestamp=data["timestamp"] + self.p["future_us"])
        elif f == "F2.07":
            out = dict(data, xy_valid=False)
        elif f == "F2.08":
            out = dict(data, x=data["y"], y=data["x"])
        elif f == "F2.03" and key == "vehicle_attitude":
            half = math.radians(self.p["yaw_deg"]) / 2
            w, x, y, z = data["q"]
            c, s = math.cos(half), math.sin(half)  # q_out = q_z(yaw) * q
            out = dict(data, q=[c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w])
        elif f == "F5.05" and key == "vehicle_status":
            out = dict(data, nav_state=self.p["nav_state"])
        elif f == "F5.10" and key == "vehicle_status":
            out = dict(data, failsafe=True)
        else:
            return [data]
        self.affected.append((key, now_sim, f))
        return [out]


class AckInjector(_Timed):
    """F5.03 (176 rewritten to DENIED), F5.04 (LAND ACK dropped), F5.06 (LAND rewritten to DENIED)."""
    DENIED = 2

    def ack(self, data, now_sim):
        if self.mode != "active":
            return [data]
        command = {"F5.03": 176, "F5.04": 21, "F5.06": 21}[self.fault_id]
        if data.get("command") != command:
            return [data]
        self.affected.append(("vehicle_command_ack", now_sim, self.fault_id))
        if self.fault_id == "F5.04":
            return []
        return [dict(data, result=self.DENIED)]


class MapInjector(_Timed):
    """Snapshot and health faults inside the controller (D1a): drop, delay, reorder, burst."""

    def __init__(self, fault_id, parameters, mode):
        super().__init__(fault_id, parameters, mode)
        key = "drop_window_sim_s" if "drop_window_sim_s" in parameters else "window_sim_s"
        self.windows = _windows(parameters, key)
        self.queue = []   # (release_sim, snapshot)
        self.held = None  # reorder: one snapshot held back
        self.burst_until = None

    def snapshot(self, snapshot, now_sim):
        f = self.fault_id
        if f not in ("F4.01", "F4.02", "F4.03", "F4.04", "F8.03", "F8.06") or not self.active(now_sim, self.windows):
            return self.due(now_sim) + [snapshot]
        if f in ("F4.01", "F8.03"):
            self.affected.append(("snapshot", now_sim, "dropped"))
            return self.due(now_sim)
        if f in ("F4.02", "F8.06"):
            self.queue.append((now_sim + self.p["delay_s"], snapshot))
            self.affected.append(("snapshot", now_sim, "delayed"))
            return self.due(now_sim)
        if f == "F4.03":
            if self.held is None:
                self.held = snapshot
                self.affected.append(("snapshot", now_sim, "held"))
                return []
            held, self.held = self.held, None
            self.affected.append(("snapshot", now_sim, "swapped"))
            return [snapshot, held]  # the newer first, then the older one: reordered
        if self.burst_until is None:
            self.burst_until = now_sim + self.p["hold_s"]
        self.queue.append((self.burst_until, snapshot))
        self.affected.append(("snapshot", now_sim, "burst_held"))
        return self.due(now_sim)

    def due(self, now_sim):
        ready = [item for release, item in self.queue if release <= now_sim]
        self.queue = [(release, item) for release, item in self.queue if release > now_sim]
        return ready

    def health(self, record, now_sim):
        if self.fault_id != "F4.06" or not self.active(now_sim, self.windows):
            return [record]
        self.affected.append(("health", now_sim, "dropped"))
        return []
