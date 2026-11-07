"""Read-only access to a retained accepted flight for the P6 offline suite (M3).

Everything is read from the run's own records: raw depth frames (each checked against its recorded
SHA-256), perception snapshots with their pose and grid, the locked calibration, and the run-local
configurations. Nothing in the run directory is written.
"""
import hashlib
import json
from pathlib import Path
import zlib


def jsonl(path):
    with Path(path).open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def depth_index(run):
    """Recorded frames keyed by acquisition stamp. The adapter recorder and perception count receipts
    separately (each is its own BEST_EFFORT subscriber), so the stamp is the only common key."""
    return {r["acquisition_stamp_ns"]: r for r in jsonl(Path(run) / "sensors/depth-index.jsonl") if r.get("accepted")}


def depth_frames(run, stamps):
    """{acquisition_stamp_ns: decoder field dict + stamp_ns, frame_id, sim} for the requested recorded frames."""
    index = depth_index(run)
    out = {}
    with (Path(run) / "sensors/depth.bin").open("rb") as stream:
        for stamp in stamps:
            r = index[stamp]
            stream.seek(r["offset"])
            stored = stream.read(r["stored_bytes"])
            raw = zlib.decompress(stored) if r["compression"] == "zlib-1" else stored
            if hashlib.sha256(raw).hexdigest() != r["raw_sha256"]:
                raise ValueError("recorded depth frame fails its own SHA-256: stamp %d" % stamp)
            out[stamp] = {"width": r["width"], "height": r["height"], "encoding": r["encoding"],
                            "is_bigendian": r["is_bigendian"], "step": r["step"], "data": raw,
                            "stamp_ns": r["acquisition_stamp_ns"], "frame_id": r["frame_id"], "sim": r["receipt_sim_s"]}
    return out


def iter_depth_frames(run, stamps):
    """The same frames as `depth_frames`, one at a time, in the order of `stamps` (bounded memory)."""
    index = depth_index(run)
    with (Path(run) / "sensors/depth.bin").open("rb") as stream:
        for stamp in stamps:
            r = index[stamp]
            stream.seek(r["offset"])
            stored = stream.read(r["stored_bytes"])
            raw = zlib.decompress(stored) if r["compression"] == "zlib-1" else stored
            if hashlib.sha256(raw).hexdigest() != r["raw_sha256"]:
                raise ValueError("recorded depth frame fails its own SHA-256: stamp %d" % stamp)
            yield {"width": r["width"], "height": r["height"], "encoding": r["encoding"], "is_bigendian": r["is_bigendian"],
                   "step": r["step"], "data": raw, "stamp_ns": r["acquisition_stamp_ns"], "frame_id": r["frame_id"],
                   "sim": r["receipt_sim_s"]}


def usable_snapshots(run):
    return [s for s in jsonl(Path(run) / "p4/local_map.jsonl") if s.get("usable") and s.get("grid")]


def calibration(run):
    return json.loads((Path(run) / "sensors/adapter-result.json").read_text(encoding="utf-8"))["calibration"]


def configs(run):
    run = Path(run)
    return {"p3": json.loads((run / "p3_sensors.yaml").read_text(encoding="utf-8")),
            "p4": json.loads((run / "p4_avoidance.yaml").read_text(encoding="utf-8")),
            "scenario": json.loads((run / "summary.json").read_text(encoding="utf-8")).get("scenario")}


class PerceptionReplay:
    """The accepted P4 scan extraction and grid integration on recorded frames with the recorded poses.

    `step(snapshot, image)` integrates one processed frame exactly as `P4Processor.observe` does for a
    matched frame; with unmodified frames the grid equals the recorded snapshot grid bit for bit
    (checked by the suite before any corruption is measured)."""

    def __init__(self, run):
        from gwm_obstacle_avoidance.local_map import Grid
        from gwm_sensor_adapter.geometry import Extrinsic
        c = configs(run)
        self.p3, self.p4 = c["p3"], c["p4"]
        self.calibration = calibration(run)
        self.extrinsic = Extrinsic(self.p3["optical_origin_base_flu_m"], identity=self.p3["extrinsic_id"])
        self.grid_class = Grid
        self.grid = None

    def step(self, snapshot, image):
        from gwm_obstacle_avoidance import perception
        p = self.p4["perception"]
        if self.grid is None:
            self.grid = self.grid_class.around(tuple(snapshot["anchor_enu"]), self.p4["geofence"], p["grid_margin_m"],
                                               p["grid_resolution_m"])
        camera_xy, rays = perception.extract_scan(image, self.calibration, self.extrinsic, snapshot["position_ned"],
                                                  snapshot["q_wxyz"], p, self.p3["near_m"], self.p3["far_m"],
                                                  snapshot["ground_up_m"])
        perception.integrate(self.grid, camera_xy, rays, p["max_hit_range_m"], self.p3["near_m"])
        return rays
