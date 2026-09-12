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
    """Lossless publication groups; nearest unique group, no reset crossing."""
    def __init__(self, capacity=300, mismatch=.05):
        self.items = deque(maxlen=capacity)
        self.mismatch = mismatch
        self.fault = None
        self.reused = 0

    def add(self, stamp, epoch, state):
        if not math.isfinite(stamp): raise ValueError('invalid_state_time')
        native='timestamp_us' in state
        if native:
            sample=state.get('timestamp_sample_us')
            if type(stamp) is not int or type(sample) is not int or not 0 < sample <= stamp:
                self.fault='invalid_state_sample_time'; raise ValueError(self.fault)
            if any(type(state.get(k)) not in (int,float) or not math.isfinite(state[k]) for k in ('x','y','z','heading')):
                self.fault='invalid_state_payload'; raise ValueError(self.fault)
        if self.fault: raise ValueError(self.fault)
        if self.items:
            previous=self.items[-1]
            if stamp < previous[0]:
                self.fault='backwards_state_time'; raise ValueError(self.fault)
            if native and state.get('run_id') != previous[2].get('run_id'):
                self.fault='cross_run_state'; raise ValueError(self.fault)
            if native and sample < previous[2]['timestamp_sample_us']:
                self.fault='backwards_state_sample_time'; raise ValueError(self.fault)
            if native and stamp>previous[0] and sample==previous[2]['timestamp_sample_us']:
                self.fault='nonadvancing_distinct_state_sample'; raise ValueError(self.fault)
            if stamp == previous[0]:
                def payload(value):
                    return {k:v for k,v in value.items() if k not in ('callback_id','receipt_monotonic_ns','source_reference')}
                if tuple(epoch)==previous[1] and payload(state)==payload(previous[2]):
                    self.reused+=1; return 'duplicate_delivery'
                if not native or sample == previous[2]['timestamp_sample_us']:
                    self.fault='ambiguous_state_identity'; raise ValueError(self.fault)
        self.items.append((stamp, tuple(epoch), deepcopy(state)))
        return 'distinct_source_observation'

    def match(self, stamp, native_ns=None):
        if self.fault or not self.items: return None
        native='timestamp_us' in self.items[0][2]
        target=(native_ns if native_ns is not None else round(stamp*1000000000)) if native else stamp
        if native and (type(target) is not int or target<0): raise ValueError('invalid_image_state_time')
        def clock(item): return item[0]*1000 if native else item[0]
        before = [s for s in self.items if clock(s) <= target]
        after = [s for s in self.items if clock(s) >= target]
        if before and after and before[-1][1] != after[0][1]: return None
        distance=min(abs(clock(s)-target) for s in self.items)
        candidates=[s for s in self.items if abs(clock(s)-target)==distance]
        selected_time=min(s[0] for s in candidates)  # equal-distance tie chooses earlier time, never by payload
        candidates=[s for s in candidates if s[0]==selected_time]
        # A nearest group with more than one distinct estimator output is
        # ambiguous for one image. Never choose first/last by tracking value.
        if len(candidates)!=1: return None
        closest=candidates[0]
        delta = (clock(closest)-target)/(1000000000 if native else 1)
        if abs(delta) > self.mismatch: return None
        return dict(timestamp=closest[0]/(1000000 if native else 1),timestamp_us=closest[0] if native else None,
                    epoch=list(closest[1]), mismatch_s=delta,state=deepcopy(closest[2]),
                    method='nearest_unique_publication_group_no_interpolation')
