"""Injected-clock health, one newest frame, and bounded acquisition association."""
from collections import deque
from copy import deepcopy
import math


class ProcessingSchedule:
    """Simulation-time phase accumulator; missed periods never create a backlog."""
    def __init__(self, rate):
        self.period=1/rate
        self.next=None
        self.last=None

    def due(self, sim):
        if not math.isfinite(sim) or self.last is not None and sim<self.last:
            raise ValueError('backwards_processing_clock')
        self.last=sim
        if self.next is None: self.next=sim
        if sim+1e-9<self.next: return False
        self.next += (math.floor((sim-self.next+1e-9)/self.period)+1)*self.period
        return True


class Health:
    def __init__(self, age_limit, stall_limit):
        self.age_limit, self.stall_limit = age_limit, stall_limit
        self.stamp = self.wall = None
        self.fault = None

    def receive(self, stamp, wall):
        if not math.isfinite(stamp) or not math.isfinite(wall):
            self.fault = 'invalid_time'
            raise ValueError(self.fault)
        if self.stamp is not None and stamp <= self.stamp:
            if stamp < self.stamp: self.fault = 'backwards_time'
            raise ValueError(self.fault or 'repeated_timestamp')
        self.stamp, self.wall = stamp, wall

    def status(self, sim, wall):
        if self.fault: return self.fault
        if self.stamp is None or wall-self.wall > self.stall_limit: return 'unavailable'
        if sim < self.stamp-.05: return 'clock_mismatch'
        if sim-self.stamp > self.age_limit: return 'stale'
        return 'fresh'


class NewestSlot:
    def __init__(self):
        self.item, self.overwritten = None, 0

    def put(self, item):
        if self.item is not None: self.overwritten += 1
        self.item = item

    def take(self):
        item, self.item = self.item, None
        return item


class StateHistory:
    """Nearest acquisition stamp only; no interpolation or reset crossing."""
    def __init__(self, capacity=300, mismatch=.05):
        self.items = deque(maxlen=capacity)
        self.mismatch = mismatch

    def add(self, stamp, epoch, state):
        if not math.isfinite(stamp): raise ValueError('invalid_state_time')
        if self.items and stamp <= self.items[-1][0]:
            if stamp < self.items[-1][0]: self.items.clear()
            else: return
        self.items.append((stamp, tuple(epoch), deepcopy(state)))

    def match(self, stamp):
        if not self.items: return None
        before = [s for s in self.items if s[0] <= stamp]
        after = [s for s in self.items if s[0] >= stamp]
        if before and after and before[-1][1] != after[0][1]: return None
        closest = min(self.items, key=lambda s: abs(s[0]-stamp))
        delta = closest[0]-stamp
        if abs(delta) > self.mismatch: return None
        return dict(timestamp=closest[0], epoch=list(closest[1]), mismatch_s=delta,
                    state=deepcopy(closest[2]), method='nearest_no_interpolation')
