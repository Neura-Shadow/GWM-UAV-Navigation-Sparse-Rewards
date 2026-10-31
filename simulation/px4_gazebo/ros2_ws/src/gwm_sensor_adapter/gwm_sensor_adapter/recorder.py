"""Single-owner bounded raw depth recorder with lossless compression and hashes.

Every received payload is offered to the recorder (independent of processing
decimation). The queue is bounded; the first overflow is sticky and makes the
recording incomplete. Close drains within a deadline, fsyncs data and index,
then publishes a separate close marker. Nothing is silently dropped.
"""
import hashlib
import json
import os
from pathlib import Path
import queue
import threading
import time
import zlib

INDEX = "depth-index.jsonl"
DATA = "depth.bin"
CLOSED = "depth-recorder-closed.json"


class RawRecorder:
    def __init__(self, folder, capacity, compression="zlib-1", clock=time.monotonic):
        if compression not in ("none", "zlib-1"):
            raise ValueError("Unsupported recorder compression")
        self.folder = Path(folder)
        self.compression = compression
        self.clock = clock
        self.queue = queue.Queue(maxsize=capacity)
        self.capacity = capacity
        self.accepted = self.written = self.overflow = 0
        self.bytes_raw = self.bytes_stored = 0
        self.max_queue = 0
        self.max_write_s = 0.0
        self.error = None
        self.closed = None
        self.data = (self.folder / DATA).open("xb")
        self.index = (self.folder / INDEX).open("x", buffering=1)
        self.thread = threading.Thread(target=self._run, name="p3-depth-recorder", daemon=True)
        self.thread.start()

    def submit(self, record, payload):
        """Non-blocking; returns False (and invalidates the recording) when full."""
        if self.closed is not None:
            raise RuntimeError("recorder_closed")
        try:
            self.queue.put_nowait((dict(record), bytes(payload)))
        except queue.Full:
            self.overflow += 1
            return False
        self.accepted += 1
        self.max_queue = max(self.max_queue, self.queue.qsize())
        return True

    def _run(self):
        offset = 0
        try:
            while True:
                item = self.queue.get()
                if item is None:
                    break
                record, payload = item
                begin = self.clock()
                stored = zlib.compress(payload, 1) if self.compression == "zlib-1" else payload
                self.data.write(stored)
                record.update(offset=offset, stored_bytes=len(stored), raw_bytes=len(payload),
                              compression=self.compression,
                              raw_sha256=hashlib.sha256(payload).hexdigest(),
                              stored_sha256=hashlib.sha256(stored).hexdigest(),
                              write_monotonic_s=self.clock())
                self.index.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
                offset += len(stored)
                self.written += 1
                self.bytes_raw += len(payload)
                self.bytes_stored += len(stored)
                self.max_write_s = max(self.max_write_s, self.clock() - begin)
        except BaseException as exc:  # recorded, never hidden
            self.error = type(exc).__name__ + ":" + str(exc)

    def close(self, deadline_s):
        if self.closed is not None:
            return self.closed
        started = self.clock()
        drained = False
        try:
            self.queue.put(None, timeout=deadline_s)
            self.thread.join(timeout=max(0.0, deadline_s - (self.clock() - started)))
            drained = not self.thread.is_alive()
        except queue.Full:
            self.error = self.error or "close_queue_blocked"
        fsynced = False
        if drained:
            try:
                for stream in (self.data, self.index):
                    stream.flush()
                    os.fsync(stream.fileno())
                    stream.close()
                fsynced = True
            except OSError as exc:
                self.error = self.error or "fsync_failed:" + str(exc)
        complete = (drained and fsynced and self.error is None and self.overflow == 0
                    and self.accepted == self.written)
        self.closed = {"contract": "p3-depth-raw-recorder-v1", "capacity": self.capacity,
                       "compression": self.compression, "accepted": self.accepted, "written": self.written,
                       "overflow": self.overflow, "max_queue": self.max_queue, "raw_bytes": self.bytes_raw,
                       "stored_bytes": self.bytes_stored, "max_write_s": self.max_write_s,
                       "drained": drained, "fsynced": fsynced, "error": self.error,
                       "close_wall_s": self.clock() - started, "complete": complete}
        with (self.folder / CLOSED).open("x") as marker:
            marker.write(json.dumps(self.closed, indent=2, allow_nan=False) + "\n")
            marker.flush()
            os.fsync(marker.fileno())
        return self.closed
