"""One binary/index owner, bounded handoff, fail-closed completion receipt."""
import hashlib
import json
import os
from queue import Queue, Full, Empty
from threading import Thread, Event
import time


class RawRecorder:
    def __init__(self, directory, capacity=8):
        self.queue = Queue(maxsize=capacity)
        self.stop = Event()
        self.failure = None
        self.accepted = self.written = self.overflow = self.peak = 0
        self.directory = directory
        self.thread = Thread(target=self._write, name='p3-raw-recorder', daemon=True)
        self.thread.start()

    def submit(self, meta, raw):
        if self.failure: raise RuntimeError(self.failure)
        try:
            self.queue.put_nowait((dict(meta), bytes(raw)))
            self.accepted += 1
            self.peak = max(self.peak, self.queue.qsize())
        except Full as exc:
            self.overflow += 1
            self.failure = 'recorder_overflow'
            raise RuntimeError(self.failure) from exc

    def _write(self):
        try:
            with (self.directory/'depth.bin').open('xb') as data, (self.directory/'depth-index.jsonl').open('x') as index:
                while not self.stop.is_set() or not self.queue.empty():
                    try: meta, raw = self.queue.get(timeout=.05)
                    except Empty: continue
                    meta.update(offset=data.tell(), bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
                                record_entry_wall_s=time.monotonic())
                    data.write(raw)
                    meta['record_return_wall_s'] = time.monotonic()
                    index.write(json.dumps(meta, allow_nan=False)+'\n')
                    self.written += 1
                data.flush(); index.flush()
                os.fsync(data.fileno()); os.fsync(index.fileno())
        except Exception as exc:
            self.failure = 'recorder_failure:'+str(exc)

    def close(self):
        self.stop.set(); self.thread.join(timeout=10)
        if self.thread.is_alive(): self.failure = 'recorder_drain_timeout'
        if self.failure or self.accepted != self.written or self.overflow:
            raise RuntimeError(self.failure or 'recording_incomplete')
        return dict(accepted=self.accepted, written=self.written, overflow=self.overflow,
                    peak=self.peak, capacity=self.queue.maxsize, fsync_completed=True)
