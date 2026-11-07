"""Depth-relay fault transforms (F1), pure: field dicts in, field dicts out.

A frame is the dict the accepted P3 decoder takes (`width`, `height`, `encoding`, `is_bigendian`,
`step`, `data`) plus `stamp_ns` and `frame_id`. In pass-through the very same object is returned, so
the relay can republish the original message untouched. Every faulted frame is reported through
`self.affected`, which the relay writes to its fault ledger.
"""
import numpy as np

from .schedule import Windows

PASS = "pass-through"


def _window(parameters):
    duration = parameters.get("window_sim_s")
    return Windows() if duration is None else Windows((0.0,), (duration,))


class DepthInjector:
    def __init__(self, fault_id, parameters, mode):
        self.fault_id, self.p, self.mode = fault_id, dict(parameters), mode
        self.trigger_sim = None
        self.affected = []  # (receipt index, what), drained by the relay into the ledger
        self.count = 0
        self.last_out_stamp = None
        self.frozen_data = None
        self.queue = []
        self.stamp_plan = self._stamp_plan() if fault_id == "F1.06" else None
        self.windows = self._windows()

    def _windows(self):
        if self.fault_id == "F1.03":
            return Windows(self.p["burst_start_offsets_sim_s"], self.p["burst_durations_sim_s"])
        return _window(self.p)

    def _stamp_plan(self):
        plan = ["duplicate"] * self.p["duplicate"] + ["backwards"] * self.p["backwards"] + ["missing"] * self.p["missing"]
        return plan

    def activate(self, sim):
        if self.trigger_sim is None:
            self.trigger_sim = sim

    def _active(self, now_sim):
        return self.mode == "active" and self.trigger_sim is not None and self.windows.active(now_sim - self.trigger_sim)

    def image(self, frame, now_sim):
        """Frames to publish now (in order), including delayed frames that have come due."""
        self.count += 1
        if self.mode != "active":
            return [frame]
        out = self.due(now_sim)
        produced = self._transform(frame, now_sim)
        out += produced
        for item in out:
            self.last_out_stamp = item["stamp_ns"] if item["stamp_ns"] > 0 else self.last_out_stamp
        return out

    def due(self, now_sim):
        ready = [item for release, item in self.queue if release <= now_sim]
        self.queue = [(release, item) for release, item in self.queue if release > now_sim]
        return ready

    def _transform(self, frame, now_sim):
        f = self.fault_id
        if f == "F1.06":
            return self._stamps(frame, now_sim)
        if not self._active(now_sim):
            return [frame]
        if f == "F1.03":
            self.affected.append((self.count, "dropped"))
            return []
        if f == "F1.04":
            self.queue.append((now_sim + self.p["delay_s"], frame))
            self.affected.append((self.count, "delayed"))
            return []
        if f == "F1.05":
            if self.frozen_data is None:
                self.frozen_data = frame["data"]
            self.affected.append((self.count, "frozen"))
            return [dict(frame, data=self.frozen_data)]
        if f == "F1.07":
            if self._budget("frames"):
                width = self.p["width"]
                rows = np.frombuffer(frame["data"], dtype="<f4").reshape(frame["height"], -1)[:, :width]
                self.affected.append((self.count, "dimension"))
                return [dict(frame, width=width, step=width * 4, data=np.ascontiguousarray(rows).tobytes())]
            return [frame]
        if f in ("F1.08", "F1.09"):
            value = np.nan if f == "F1.08" else np.inf
            self.affected.append((self.count, "region"))
            return [self._region(frame, value)]
        if f == "F1.10":
            image = np.frombuffer(frame["data"], dtype="<f4").copy()
            keep = np.zeros(image.size, dtype=bool)
            keep[::self.p["keep_every"]] = True
            image[~keep] = np.nan
            self.affected.append((self.count, "coverage"))
            return [dict(frame, data=image.tobytes())]
        if f == "F1.12":
            self.affected.append((self.count, "frame_id"))
            return [dict(frame, frame_id=self.p["frame_id"])]
        raise ValueError("no depth transform for " + f)

    def _budget(self, key):
        used = sum(1 for _, what in self.affected if what == "dimension")
        return used < self.p[key]

    def _region(self, frame, value):
        r0, r1 = self.p["rows"]
        c0, c1 = self.p["cols"]
        image = np.frombuffer(frame["data"], dtype="<f4").reshape(frame["height"], frame["width"]).copy()
        image[r0:r1, c0:c1] = value
        return dict(frame, data=image.tobytes())

    def _stamps(self, frame, now_sim):
        """F1.06: N duplicate, N backwards and N missing stamps, then a future-offset window; afterwards
        frames that would go backwards are dropped until real stamps pass the last future one, so the
        adapter's rejection counters equal exactly the injected counts."""
        if self.trigger_sim is None:
            return [frame]
        if self.stamp_plan:
            kind = self.stamp_plan.pop(0)
            last = self.last_out_stamp if self.last_out_stamp is not None else frame["stamp_ns"]
            stamp = {"duplicate": last, "backwards": last - 1_000_000, "missing": 0}[kind]
            self.affected.append((self.count, kind))
            if not self.stamp_plan:
                self.future_start = now_sim
            return [dict(frame, stamp_ns=stamp)]
        start = getattr(self, "future_start", None)
        offset_ns = int(round(self.p["future_offset_s"] * 1e9))
        if start is not None and now_sim - start < self.p["future_window_sim_s"]:
            self.affected.append((self.count, "future"))
            return [dict(frame, stamp_ns=frame["stamp_ns"] + offset_ns)]
        if self.last_out_stamp is not None and frame["stamp_ns"] <= self.last_out_stamp:
            self.affected.append((self.count, "dropped_to_restore_order"))
            return []
        return [frame]

    def camera_info(self, info, now_sim):
        """F1.11: scale fx (K[0] and P[0]) from the trigger on; everything else unchanged."""
        if self.fault_id != "F1.11" or not self._active(now_sim):
            return info
        scale = self.p["camera_info_fx_scale"]
        k, p = list(info["k"]), list(info["p"])
        k[0] *= scale
        p[0] *= scale
        self.affected.append((self.count, "calibration"))
        return dict(info, k=k, p=p)
