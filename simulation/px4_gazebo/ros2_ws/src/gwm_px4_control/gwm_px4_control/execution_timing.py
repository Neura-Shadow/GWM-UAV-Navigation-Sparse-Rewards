"""Bounded in-memory operation spans; persistence happens after control stops."""
from contextlib import contextmanager
import json
import time
import threading
from copy import deepcopy


class TraceBuffer:
    def __init__(self, capacity, clock=time.monotonic):
        if type(capacity) is not int or capacity <= 0:
            raise ValueError('Invalid trace capacity')
        self.capacity, self.clock = capacity, clock
        self.records = []
        self.overflow = 0
        self.calls = {}

    def append(self, name, start, end, sim, **meta):
        count = self.calls.get(name, 0)+1
        self.calls[name] = count
        if len(self.records) >= self.capacity:
            self.overflow += 1
            return
        self.records.append({'seq':len(self.records)+1,'operation':name,
            'start_wall_s':start,'end_wall_s':end,'duration_wall_s':end-start,
            'sim_s':sim,'call_index':count,**deepcopy(meta)})

    @contextmanager
    def span(self, name, sim, **meta):
        start = self.clock()
        try:
            yield meta
        finally:
            self.append(name, start, self.clock(), sim, **meta)

    def persist(self, path):
        with path.open('x') as stream:
            for record in self.records:
                stream.write(json.dumps(record,allow_nan=False,separators=(',',':'))+'\n')
        return {'capacity':self.capacity,'records':len(self.records),'overflow':self.overflow,
                'calls':self.calls,'complete':self.overflow==0}


def transport_allowed(key, flight, ground):
    if key == 'vehicle_command' and (not flight or ground):
        raise ValueError('VehicleCommand forbidden at transport boundary')
    if key not in ('vehicle_command','trajectory_setpoint','offboard_control_mode'):
        raise ValueError('Unsupported control topic')


def controller_transport_env(config):
    expected = {'contract':'p2-timing-v1','evidence_schema':2,
        'profile':'udp_control_synchronous_evidence','controller_transport':'UDPv4',
        'dispatch_budget_wall_s':0.05,'graph_fresh_wall_s':1.5,'trace_capacity':500000,
        'ground_prestream_sim_s':3.0,'ground_wall_s':75.0,
        'diagnostic_starts_before_repair':6,'diagnostic_starts_after_repair':3}
    if config != expected:
        raise ValueError('Unregistered P2 timing transport profile')
    return {'FASTDDS_BUILTIN_TRANSPORTS':'UDPv4','SKIP_DEFAULT_XML':'1',
            'RMW_FASTRTPS_PUBLICATION_MODE':'SYNCHRONOUS'}


class DispatchGuard:
    """One decision owner; a pre-call guard cannot preempt blocking middleware."""
    def __init__(self, budget, graph_fresh, owner=threading.get_ident):
        self.budget,self.graph_fresh,self.owner=budget,graph_fresh,owner
        self.owner_id=owner()
        self.started=None

    def begin(self, wall):
        if self.owner()!=self.owner_id or self.started is not None:
            raise ValueError('concurrent_or_reentrant_control')
        self.started=wall

    def check(self, wall, graph_checked, graph_valid):
        if self.owner()!=self.owner_id or self.started is None:
            raise ValueError('control_owner_missing')
        if wall-self.started>self.budget:
            raise ValueError('stale_action')
        if not graph_valid or graph_checked is None or wall-graph_checked>self.graph_fresh:
            raise ValueError('stale_or_invalid_graph')

    def end(self):
        self.started=None


class PublicationCoverage:
    """Actual successful call entries, not intended Mission timer starts."""
    def __init__(self, gap_limit):
        self.gap_limit=gap_limit
        self.first=None
        self.last=None
        self.count=0

    def record(self, sim):
        if self.last is not None and (sim<=self.last or sim-self.last>self.gap_limit):
            raise ValueError('prestream_publication_gap')
        self.first=sim if self.first is None else self.first
        self.last=sim
        self.count+=1

    def require(self, duration):
        if self.count<2 or self.last-self.first<duration-1.1e-6:
            raise ValueError('actual_prestream_incomplete')
