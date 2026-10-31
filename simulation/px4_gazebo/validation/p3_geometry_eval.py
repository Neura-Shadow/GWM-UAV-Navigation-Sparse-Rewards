"""Independent analytic fixture evaluator (offline; never imports production code).

Expected depth is recomputed from the generated world SDF actually used, the
native Gazebo ground-truth poses recorded by the evaluator-only probe, the
native CameraInfo and the frozen pinned optical origin. Rays are intersected
with oriented boxes by the slab method; the expected value is optical-axis Z.
Ground truth is used here only - it never reaches the live sensor pipeline.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from p3_provenance import strict_json, strict_jsonl
from p3_stream_eval import Failures, evaluate_stream, load_sensor_artifacts, read_frames


def rotation_wxyz(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z]])


def rotation_rpy(roll, pitch, yaw):
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return rz @ ry @ rx


def world_boxes(sdf_text):
    world = ET.fromstring(sdf_text).find("world")
    boxes = []
    for model in world.findall("model"):
        size = model.find("link/collision/geometry/box/size")
        if model.findtext("static") != "true" or size is None:
            continue
        pose = [float(v) for v in model.findtext("pose").split()]
        boxes.append({"name": model.get("name"), "center": np.array(pose[:3]), "rotation": rotation_rpy(*pose[3:]),
                      "half": np.array([float(v) for v in size.text.split()]) / 2})
    return world.get("name"), boxes


def ray_box(origins, directions, box):
    """Vectorised slab intersection; returns entry distance or +inf."""
    r = box["rotation"]
    o = (origins - box["center"]) @ r
    d = directions @ r
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-box["half"] - o) / d
        t2 = (box["half"] - o) / d
    near = np.nanmax(np.minimum(t1, t2), axis=-1)
    far = np.nanmin(np.maximum(t1, t2), axis=-1)
    hit = (far >= near) & (far > 0)
    return np.where(hit & (near > 0), near, np.inf)


def expected_depth(us, vs, camera, model_position, model_rotation, optical_origin_model, boxes, ground_z=0.0):
    """Optical-axis depth per pixel: direction (1, -(u-cx)/fx, -(v-cy)/fy) in model FLU."""
    fx, fy, cx, cy = camera
    local = np.stack([np.ones_like(us, dtype=float), -(us - cx) / fx, -(vs - cy) / fy], axis=-1)
    directions = local @ model_rotation.T
    origin = model_position + model_rotation @ np.asarray(optical_origin_model, dtype=float)
    origins = np.broadcast_to(origin, directions.shape)
    best = np.full(us.shape, np.inf)
    label = np.full(us.shape, "", dtype=object)
    for box in boxes:
        t = ray_box(origins, directions, box)
        closer = t < best
        best[closer], label[closer] = t[closer], box["name"]
    with np.errstate(divide="ignore", invalid="ignore"):
        tg = (ground_z - origin[2]) / directions[..., 2]
    ground = (tg > 0) & (tg < best)
    best[ground], label[ground] = tg[ground], "ground"
    # The local optical-axis component of each direction is exactly 1, so the
    # ray parameter is the optical Z depth; the Euclidean range is t*|d_local|.
    return best, label, np.linalg.norm(local, axis=-1), origin


def pose_samples(run, name):
    rows = strict_jsonl(Path(run) / "source/poses.jsonl")
    return [(r["stamp_ns"] / 1e9, r["poses"][name]) for r in rows if name in r["poses"]]


def pose_at(samples, sim_s, tolerance=1.0):
    best = min(samples, key=lambda item: abs(item[0] - sim_s))
    if abs(best[0] - sim_s) > tolerance:
        raise ValueError("ground_truth_pose_unavailable_near_window")
    return np.array(best[1]["position"]), rotation_wxyz(best[1]["q_wxyz"])


def thresholds(limits, expected_median):
    return (max(limits["median_error_m"], limits["median_error_fraction"] * expected_median),
            max(limits["p95_error_m"], limits["p95_error_fraction"] * expected_median))


def compare_frames(run, index, window, expected, mask_fn, limits, near, far):
    begin, end = window
    per_frame, unique = [], set()
    counts = {"nan": 0, "positive_infinity": 0, "negative_infinity": 0, "finite_below_near": 0,
              "finite_at_or_above_far": 0, "zero": 0}
    for row, image in read_frames(run, index):
        if image is None or not begin * 1e9 <= row["acquisition_stamp_ns"] <= end * 1e9:
            continue
        unique.add(row["raw_sha256"])
        counts["nan"] += int(np.isnan(image).sum())
        counts["positive_infinity"] += int(np.isposinf(image).sum())
        counts["negative_infinity"] += int(np.isneginf(image).sum())
        counts["finite_below_near"] += int((np.isfinite(image) & (image < near)).sum())
        counts["finite_at_or_above_far"] += int((np.isfinite(image) & (image >= far)).sum())
        counts["zero"] += int((image == 0).sum())
        per_frame.append(mask_fn(row, image))
    return per_frame, counts, len(unique)


def evaluate_fixture(run, validation, sensors):
    run = Path(run)
    f = Failures()
    summary = strict_json(run / "summary.json")
    fixture = summary["fixture"]
    case = "plane4" if fixture == "interruption" else fixture
    window = summary["measurement_window_sim_s"]
    out = {"contract": "p3-fixture-evaluation-v1", "run_id": run.name, "fixture": fixture,
           "window_sim_s": window, "independent": "no production geometry, fixture or adapter code imported"}
    world_name, boxes = world_boxes((run / "world" / (summary["world"] + ".sdf")).read_text())
    f.check(world_name == summary["world"] and any(b["name"] == "front_plane" for b in boxes), "world_identity")
    info = strict_json(run / "source/camera-info.json")
    camera = (info["k"][0], info["k"][4], info["k"][2], info["k"][5])
    adapter, closed, index, observations, health = load_sensor_artifacts(run)
    calibration = adapter.get("calibration") or {}
    f.check([calibration.get(k) for k in ("fx", "fy", "cx", "cy")] == list(camera), "native_camera_info_mismatch")
    vehicle = pose_samples(run, summary["model"])
    start_p, start_r = pose_at(vehicle, window[0])
    end_p, end_r = pose_at(vehicle, window[1])
    drift = float(np.linalg.norm(end_p - start_p))
    rot = float(math.acos(np.clip((np.trace(start_r.T @ end_r) - 1) / 2, -1, 1)))
    out["vehicle_pose"] = {"position_m": start_p.tolist(), "translation_drift_m": drift, "rotation_drift_rad": rot}
    f.check(drift <= validation["geometry_pose_translation_drift_m"]
            and rot <= validation["geometry_pose_rotation_drift_rad"], "unsettled_vehicle_pose")
    # Fixture boxes from the world file, cross-checked against native ground truth.
    plane_truth = pose_samples(run, "front_plane")
    plane_p, _ = pose_at(plane_truth, window[0])
    declared = next(b for b in boxes if b["name"] == "front_plane")["center"]
    f.check(np.linalg.norm(plane_p - declared) < 1e-6, "fixture_pose_differs_from_world")
    u0, v0, u1, v1 = validation["plane_roi_uv"]
    vs, us = np.mgrid[v0:v1, u0:u1]
    expected, label, ray_norm, origin = expected_depth(us.astype(float), vs.astype(float), camera, start_p, start_r,
                                                       sensors["optical_origin_model_m"], boxes)
    out["optical_origin_world_m"] = origin.tolist()
    finite_expected = expected[np.isfinite(expected)]
    out["expected"] = {"roi_uv": validation["plane_roi_uv"], "median_m": float(np.median(expected)),
                       "labels": {k: int(v) for k, v in zip(*np.unique(label.astype(str), return_counts=True))}}
    near, far = sensors["near_m"], sensors["far_m"]

    def roi_stats(row, image):
        roi = image[v0:v1, u0:u1]
        valid = np.isfinite(roi) & (roi >= near) & (roi < far)
        entry = {"receipt_id": row["receipt_id"], "valid_fraction": float(valid.mean()),
                 "ros_stamp_ns": row["acquisition_stamp_ns"]}
        if valid.any():
            error = np.abs(roi[valid] - expected[valid])
            entry.update(median_error_m=float(np.median(error)), p95_error_m=float(np.percentile(error, 95)),
                         measured_median_m=float(np.median(roi[valid])),
                         range_hypothesis_median_error_m=float(np.median(np.abs(roi[valid] - (expected * ray_norm)[valid]))))
        if case == "asymmetric":
            entry["regions"] = {}
            for name, (a0, b0, a1, b1) in validation["fixtures"]["asymmetric"]["region_uv"].items():
                region = image[b0:b1, a0:a1]
                mirror_lr = image[b0:b1, 640 - a1:640 - a0]
                mirror_ud = image[480 - b1:480 - b0, a0:a1]
                vv, uu = np.mgrid[b0:b1, a0:a1]
                exp, lab, _, _ = expected_depth(uu.astype(float), vv.astype(float), camera, start_p, start_r,
                                                sensors["optical_origin_model_m"], boxes)
                ok = np.isfinite(region) & (region >= near) & (region < far)
                entry["regions"][name] = {
                    "valid_fraction": float(ok.mean()), "expected_labels": sorted(set(lab.ravel().tolist())),
                    "expected_median_m": float(np.median(exp)), "measured_median_m": float(np.median(region[ok])) if ok.any() else None,
                    "median_error_m": float(np.median(np.abs(region[ok] - exp[ok]))) if ok.any() else None,
                    "mirror_left_right_median_m": float(np.nanmedian(np.where(np.isfinite(mirror_lr), mirror_lr, np.nan))),
                    "mirror_up_down_median_m": float(np.nanmedian(np.where(np.isfinite(mirror_ud), mirror_ud, np.nan)))}
        return entry

    per_frame, counts, unique = compare_frames(run, index, window, expected, roi_stats, validation, near, far)
    out["frames_evaluated"], out["unique_raw_frames"], out["pixel_counts"] = len(per_frame), unique, counts
    f.check(len(per_frame) > 0, "no_frames_in_window")
    f.check(counts["zero"] == 0, "zero_depth_fabricated")
    median_limit, p95_limit = thresholds(validation, float(np.median(expected)))
    valid_min = min((p["valid_fraction"] for p in per_frame), default=0.0)
    if case == "out_of_range":
        f.check(np.all(np.isinf(expected)) or np.all(expected >= far), "fixture_not_out_of_range_by_geometry")
        f.check(valid_min == 0 and max((p["valid_fraction"] for p in per_frame), default=1) == 0,
                "out_of_range_reported_valid")
        f.check(counts["positive_infinity"] > 0 and counts["nan"] == 0, "no_return_semantics")
    else:
        f.check(finite_expected.size == expected.size, "roi_not_fully_on_fixture")
        f.check(valid_min >= validation["valid_fraction_min"], "plane_valid_fraction")
        medians = [p["median_error_m"] for p in per_frame if "median_error_m" in p]
        p95s = [p["p95_error_m"] for p in per_frame if "p95_error_m" in p]
        f.check(len(medians) == len(per_frame) and max(medians) <= median_limit, "median_depth_error")
        f.check(len(p95s) == len(per_frame) and max(p95s) <= p95_limit, "p95_depth_error")
        out["errors"] = {"median_error_max_m": max(medians, default=None), "p95_error_max_m": max(p95s, default=None),
                         "median_limit_m": median_limit, "p95_limit_m": p95_limit, "valid_fraction_min": valid_min}
        if case == "oblique":
            range_errors = [p["range_hypothesis_median_error_m"] for p in per_frame]
            out["errors"]["euclidean_range_hypothesis_median_error_m"] = float(np.median(range_errors))
            f.check(np.median(range_errors) > median_limit, "optical_Z_not_distinguished_from_range")
        if case == "asymmetric":
            out["regions"] = regions = {}
            for name in validation["fixtures"]["asymmetric"]["region_uv"]:
                rows = [p["regions"][name] for p in per_frame]
                regions[name] = {"expected_labels": rows[0]["expected_labels"],
                                 "expected_median_m": rows[0]["expected_median_m"],
                                 "measured_median_m": float(np.median([r["measured_median_m"] for r in rows
                                                                       if r["measured_median_m"] is not None])),
                                 "median_error_max_m": max((r["median_error_m"] for r in rows
                                                            if r["median_error_m"] is not None), default=None),
                                 "valid_fraction_min": min(r["valid_fraction"] for r in rows),
                                 "mirror_left_right_median_m": float(np.median([r["mirror_left_right_median_m"] for r in rows])),
                                 "mirror_up_down_median_m": float(np.median([r["mirror_up_down_median_m"] for r in rows]))}
                region = regions[name]
                limit = thresholds(validation, region["expected_median_m"])[0]
                f.check(region["expected_labels"] == [name], "asymmetric_region_geometry:" + name)
                f.check(region["valid_fraction_min"] >= validation["valid_fraction_min"]
                        and region["median_error_max_m"] is not None and region["median_error_max_m"] <= limit,
                        "asymmetric_region_depth:" + name)
                # Orientation: the mirrored image location must not show this box's depth.
                for key in ("mirror_left_right_median_m", "mirror_up_down_median_m"):
                    f.check(abs(region[key] - region["expected_median_m"]) > limit, "image_axis_orientation:" + name + ":" + key)
            f.check(regions["left_high"]["expected_median_m"] > regions["right_low"]["expected_median_m"],
                    "asymmetric_near_far_order")
    limits = dict(validation)
    if fixture == "interruption":
        out["interruption"] = evaluate_interruption(run, summary, observations, health, validation)
        f.items += out["interruption"]["failures"]
        f.check(window[1] <= summary["interruption"]["request_sim_s"], "interruption_inside_measurement_window")
    out["stream"] = evaluate_stream(run, window, limits, sensors)
    f.items += ["stream:" + item for item in out["stream"]["failures"]]
    if summary.get("scene_change"):
        out["scene_change"] = evaluate_scene_change(run, summary, index, camera, boxes, validation, sensors)
        f.items += out["scene_change"]["failures"]
    out["failures"] = f.items
    out["status"] = "passed" if not f.items else "failed"
    return out


def evaluate_interruption(run, summary, observations, health, validation):
    f = Failures()
    event = summary["interruption"]
    request_wall = event["request_monotonic_s"]
    stale = [h for h in health if h["wall_s"] >= request_wall and h["status"] != "fresh"]
    first = stale[0] if stale else None
    after = [o for o in observations if o["processing_wall_s"] > (first["wall_s"] if first else request_wall)]
    out = {"request_monotonic_s": request_wall, "interrupted": event["component"],
           "detected": first, "detection_latency_wall_s": first["wall_s"] - request_wall if first else None,
           "observations_after_detection": len(after), "producer_exit": event.get("exit"),
           "producer_log_retained": (Path(run) / (event["component"] + ".log")).exists()}
    f.check(first is not None, "interruption_not_detected")
    f.check(first is None or out["detection_latency_wall_s"] <= validation["interruption_detect_wall_s"],
            "interruption_detection_late")
    f.check(not after, "stale_frame_published_after_interruption")
    f.check(all(h["status"] != "fresh" for h in stale), "freshness_recovered_without_new_frames")
    f.check(event.get("exit", {}).get("classification") == "graceful", "interrupted_producer_exit_not_graceful")
    f.check(out["producer_log_retained"], "interrupted_producer_evidence_missing")
    out["failures"] = f.items
    return out


def evaluate_scene_change(run, summary, index, camera, boxes, validation, sensors):
    """Changed native pixels after moving the fixture 4 m -> 5 m (excluded from the window)."""
    f = Failures()
    change = summary["scene_change"]
    begin = change["applied_sim_s"] + validation["scene_change"]["settle_sim_s"]
    window = [begin, begin + validation["scene_change"]["window_sim_s"]]
    truth = pose_samples(run, "front_plane")
    moved, _ = pose_at(truth, window[0])
    f.check(np.linalg.norm(moved - np.array(change["requested_position_m"])) < 1e-6, "scene_change_pose_not_applied")
    moved_boxes = [dict(b, center=moved) if b["name"] == "front_plane" else b for b in boxes]
    vehicle = pose_samples(run, summary["model"])
    p, r = pose_at(vehicle, window[0])
    u0, v0, u1, v1 = validation["plane_roi_uv"]
    vs, us = np.mgrid[v0:v1, u0:u1]
    expected, _, _, _ = expected_depth(us.astype(float), vs.astype(float), camera, p, r,
                                       sensors["optical_origin_model_m"], moved_boxes)
    median_limit, p95_limit = thresholds(validation, float(np.median(expected)))
    rows = []
    for row, image in read_frames(run, index):
        if image is None or not window[0] * 1e9 <= row["acquisition_stamp_ns"] <= window[1] * 1e9:
            continue
        roi = image[v0:v1, u0:u1]
        valid = np.isfinite(roi) & (roi >= sensors["near_m"]) & (roi < sensors["far_m"])
        error = np.abs(roi[valid] - expected[valid]) if valid.any() else np.array([np.inf])
        rows.append((float(valid.mean()), float(np.median(error)), float(np.percentile(error, 95)),
                     float(np.median(roi[valid])) if valid.any() else None))
    out = {"window_sim_s": window, "frames": len(rows), "expected_median_m": float(np.median(expected)),
           "measured_median_m": float(np.median([r[3] for r in rows if r[3] is not None])) if rows else None}
    f.check(rows and min(r[0] for r in rows) >= validation["valid_fraction_min"], "scene_change_valid_fraction")
    f.check(rows and max(r[1] for r in rows) <= median_limit and max(r[2] for r in rows) <= p95_limit,
            "scene_change_depth_not_updated")
    out["failures"] = f.items
    return out
