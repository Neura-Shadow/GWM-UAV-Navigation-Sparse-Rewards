"""Evidence Retention Manager: an offline study of lossless depth codecs on recorded frames. It reads only.

For each run it samples `blocks` evenly spaced blocks of `per_block` consecutive accepted frames from
`sensors/depth.bin`. Every stored frame is checked against its `stored_sha256`, decompressed (the recorder's zlib-1),
and checked against its `raw_sha256`. The raw 640x480 float32 frame is then encoded by each codec. The result must
decode to the identical bytes, or the codec is reported as failed. It measures stored bytes, single-thread encode and
decode throughput (CPU time) and a few image statistics.

Transforms are all exactly invertible:
* `shuffle`: the four byte planes of the float32 values (as in Blosc and HDF5 shuffle);
* `hdelta`: the row-wise difference of the uint32 bit patterns, modulo 2**32;
* `txor`: XOR with the previous frame's uint32 bit patterns (temporal); the first frame of each block is a key frame.

Runs are identified by the SHA-256 of their depth index, never by name, so this record is no dependency of any run.
Usage: cd /tmp && python3 depth_codec_study.py OUTPUT.json ROLE=RUN_DIR [ROLE=RUN_DIR ...]
"""
import bz2  # noqa: F401  (available; not used: too slow for 30 Hz)
import hashlib
import json
import lzma
from pathlib import Path
import sys
import time
import zlib

import numpy as np

try:
    import lz4.frame as lz4f
except ImportError:  # pragma: no cover
    lz4f = None
try:
    import brotli
except ImportError:  # pragma: no cover
    brotli = None

W, H = 640, 480


def shuffle(b):
    return np.frombuffer(b, dtype=np.uint8).reshape(-1, 4).T.tobytes()


def unshuffle(b):
    return np.frombuffer(b, dtype=np.uint8).reshape(4, -1).T.tobytes()


def hdelta(b):
    u = np.frombuffer(b, dtype="<u4").reshape(H, W)
    d = u.copy()
    d[:, 1:] = u[:, 1:] - u[:, :-1]
    return d.tobytes()


def unhdelta(b):
    return np.cumsum(np.frombuffer(b, dtype="<u4").reshape(H, W), axis=1, dtype=np.uint32).astype("<u4").tobytes()


def txor(b, prev):
    return (np.frombuffer(b, dtype="<u4") ^ np.frombuffer(prev, dtype="<u4")).tobytes()


BACKENDS = {
    "zlib-1": (lambda b: zlib.compress(b, 1), zlib.decompress),
    "zlib-6": (lambda b: zlib.compress(b, 6), zlib.decompress),
    "lzma-1": (lambda b: lzma.compress(b, preset=1), lzma.decompress),
}
if lz4f:
    BACKENDS["lz4"] = (lz4f.compress, lz4f.decompress)
if brotli:
    BACKENDS["brotli-1"] = (lambda b: brotli.compress(b, quality=1), brotli.decompress)
    BACKENDS["brotli-5"] = (lambda b: brotli.compress(b, quality=5), brotli.decompress)

PIPELINES = [  # (name, transform chain, backend, temporal)
    ("zlib-1 (current recorder)", (), "zlib-1", False), ("zlib-6", (), "zlib-6", False), ("lz4", (), "lz4", False),
    ("brotli-1", (), "brotli-1", False), ("brotli-5", (), "brotli-5", False), ("lzma-1", (), "lzma-1", False),
    ("shuffle+zlib-1", ("shuffle",), "zlib-1", False), ("shuffle+zlib-6", ("shuffle",), "zlib-6", False),
    ("shuffle+lz4", ("shuffle",), "lz4", False), ("shuffle+brotli-5", ("shuffle",), "brotli-5", False),
    ("hdelta+shuffle+zlib-1", ("hdelta", "shuffle"), "zlib-1", False),
    ("hdelta+shuffle+zlib-6", ("hdelta", "shuffle"), "zlib-6", False),
    ("hdelta+shuffle+lz4", ("hdelta", "shuffle"), "lz4", False),
    ("hdelta+shuffle+brotli-5", ("hdelta", "shuffle"), "brotli-5", False),
    ("hdelta+shuffle+lzma-1", ("hdelta", "shuffle"), "lzma-1", False),
    ("txor+shuffle+zlib-1 (key: hdelta+shuffle+zlib-1)", ("hdelta", "shuffle"), "zlib-1", True),
]
FORWARD = {"shuffle": shuffle, "hdelta": hdelta}
INVERSE = {"shuffle": unshuffle, "hdelta": unhdelta}


def encode(raw, chain, backend, prev):
    data = raw
    if prev is not None:
        data = shuffle(txor(raw, prev))
    else:
        for t in chain:
            data = FORWARD[t](data)
    return BACKENDS[backend][0](data)


def decode(enc, chain, backend, prev):
    data = BACKENDS[backend][1](enc)
    if prev is not None:
        return txor(unshuffle(data), prev)
    for t in reversed(chain):
        data = INVERSE[t](data)
    return data


def sample(run, blocks, per_block):
    index = run / "sensors/depth-index.jsonl"
    rows = [json.loads(x) for x in index.read_text().splitlines() if x.strip()]
    rows = [r for r in rows if r.get("accepted") and r.get("offset") is not None]
    starts = [int(i * (len(rows) - per_block) / max(blocks - 1, 1)) for i in range(blocks)]
    picked = [[rows[s + k] for k in range(per_block)] for s in starts]
    out = []
    with open(run / "sensors/depth.bin", "rb") as stream:
        for block in picked:
            frames = []
            for r in block:
                stream.seek(r["offset"])
                stored = stream.read(r["stored_bytes"])
                assert hashlib.sha256(stored).hexdigest() == r["stored_sha256"], "stored hash"
                raw = zlib.decompress(stored) if r["compression"] == "zlib-1" else stored
                assert hashlib.sha256(raw).hexdigest() == r["raw_sha256"], "raw hash"
                frames.append((raw, r["stored_bytes"]))
            out.append(frames)
    meta = {"depth_index_sha256": hashlib.sha256(index.read_bytes()).hexdigest(), "frames_in_run": len(rows),
            "raw_bytes_in_run": sum(r["raw_bytes"] for r in rows), "stored_bytes_in_run": sum(r["stored_bytes"] for r in rows)}
    return out, meta


def stats(frames):
    arr = np.concatenate([np.frombuffer(raw, dtype="<f4") for block in frames for raw, _ in block])
    finite = np.isfinite(arr)
    return {"pixels": int(arr.size), "non_finite_fraction": round(float(1 - finite.mean()), 4),
            "at_or_beyond_far_clip_fraction": round(float((~finite | (arr >= 19.0)).mean()), 4),
            "median_finite_m": round(float(np.median(arr[finite])), 3) if finite.any() else None}


def study(run, blocks=10, per_block=15):
    frames, meta = sample(Path(run), blocks, per_block)
    raw_total = sum(len(raw) for block in frames for raw, _ in block)
    recorded = sum(s for block in frames for _, s in block)
    results = {}
    for name, chain, backend, temporal in PIPELINES:
        if backend not in BACKENDS:
            results[name] = {"available": False}
            continue
        enc_bytes, t_enc, t_dec, ok = 0, 0.0, 0.0, 0
        for block in frames:
            prev = None
            for raw, _ in block:
                p = prev if temporal else None
                t0 = time.process_time()
                enc = encode(raw, chain, backend, p)
                t1 = time.process_time()
                dec = decode(enc, chain, backend, p)
                t2 = time.process_time()
                enc_bytes += len(enc)
                t_enc += t1 - t0
                t_dec += t2 - t1
                ok += dec == raw
                prev = raw
        n = sum(len(b) for b in frames)
        results[name] = {"available": True, "lossless_frames": ok, "frames": n, "lossless": ok == n,
                         "ratio_vs_raw": round(raw_total / enc_bytes, 3),
                         "size_vs_current_recorder": round(enc_bytes / recorded, 3),
                         "encode_MBps_one_core": round(raw_total / 1e6 / t_enc, 1) if t_enc else None,
                         "decode_MBps_one_core": round(raw_total / 1e6 / t_dec, 1) if t_dec else None,
                         "realtime_30hz_one_core": bool(t_enc and raw_total / t_enc >= 30 * W * H * 4)}
    return dict(meta, sampled_frames=sum(len(b) for b in frames), sampled_raw_bytes=raw_total,
                sampled_recorded_bytes=recorded, recorded_ratio_vs_raw=round(raw_total / recorded, 3),
                image=stats(frames), codecs=results)


def main(argv):
    output = Path(argv[0])
    if output.exists():
        raise SystemExit("output exists")
    value = {"contract": "depth-codec-study-v1", "read_only": True, "frame": {"width": W, "height": H, "encoding": "32FC1",
             "raw_bytes": W * H * 4, "sensor_hz": 30, "processing_hz": 10},
             "method": __doc__.split("Usage:")[0].strip(), "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             "runs": {}}
    for arg in argv[1:]:
        role, run = arg.split("=", 1)
        value["runs"][role] = study(run)
        print(role, json.dumps({k: v["size_vs_current_recorder"] for k, v in value["runs"][role]["codecs"].items() if v.get("available")}))
    with open(output, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=1, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
