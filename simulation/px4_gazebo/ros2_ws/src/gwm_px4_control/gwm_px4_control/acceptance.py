"""Independent expected NED fixtures; does not call controller transforms."""
import math

PHASES = ("INITIAL_HOVER", "EAST_TEST", "RETURN_AFTER_EAST", "NORTH_TEST",
          "RETURN_AFTER_NORTH", "YAW_TEST", "RESTORE_INITIAL_YAW", "FINAL_HOVER")


def expected_fixture(origin, initial_yaw, config):
    n, e, d = origin
    up = d-config["height_m"]
    yaw_change = initial_yaw-math.pi*config["yaw_enu_deg"]/180
    yaw_change = math.atan2(math.sin(yaw_change), math.cos(yaw_change))
    return {name: {"position": (n+(config["north_m"] if name == "NORTH_TEST" else 0),
                                e+(config["east_m"] if name == "EAST_TEST" else 0), up),
                   "yaw": yaw_change if name == "YAW_TEST" else initial_yaw}
            for name in PHASES}


def evaluate_window(samples, target, duration, config):
    if len(samples) < 2:
        return {"status": "unknown", "reason": "missing_samples"}
    try:
        values = [v for s in samples for v in [s["t"], *s["position"], s["yaw"]]]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            return {"status": "unknown", "reason": "nonfinite_measurement"}
        if any(s["nav_state"] != 14 or s["arming_state"] != 2 or s["landed"] is not False for s in samples):
            return {"status": "failed", "reason": "phase_mode_armed_airborne"}
        gaps = [b["t"]-a["t"] for a, b in zip(samples, samples[1:])]
        elapsed = samples[-1]["t"]-samples[0]["t"]
        horizontal = max(math.hypot(s["position"][0]-target["position"][0],
                                    s["position"][1]-target["position"][1]) for s in samples)
        height = max(abs(s["position"][2]-target["position"][2]) for s in samples)
        yaw = max(abs(math.atan2(math.sin(s["yaw"]-target["yaw"]), math.cos(s["yaw"]-target["yaw"]))) for s in samples)
        passed = (elapsed >= duration-1e-8 and min(gaps) > 0 and max(gaps) <= config["max_sample_gap_sim_s"]
                  and horizontal <= config["horizontal_tolerance_m"] and height <= config["height_tolerance_m"]
                  and yaw <= math.radians(config["yaw_tolerance_deg"]))
        return {"status": "passed" if passed else "failed", "duration_sim_s": elapsed,
                "samples": len(samples), "max_gap_sim_s": max(gaps), "max_horizontal_error_m": horizontal,
                "max_height_error_m": height, "max_yaw_error_deg": math.degrees(yaw)}
    except (KeyError, TypeError, IndexError):
        return {"status": "unknown", "reason": "missing_fields"}


def evaluate_window_v2(samples, target, duration, config):
    """All evaluations affect safety; only distinct publication times cover time.

    The original strict evaluator above remains available for historical replay.
    Input source references are reconstructed/verified by the evidence reader;
    this function also rejects contradictory position identities and ordering.
    """
    if len(samples) < 2:
        return {"status": "unknown", "reason": "missing_samples"}
    try:
        from .sample_identity import stamp_us
        identities, covered = {}, []
        last_pub = last_sample = None
        generation = samples[0].get("reference_generation")
        for row in samples:
            pub, acquisition, identity = (row["timestamp_us"], row["timestamp_sample_us"], row["source_id"])
            stamp_us(pub); stamp_us(acquisition, 'timestamp_sample')
            if (type(pub) is not int or type(acquisition) is not int or not 0 < acquisition <= pub
                    or not isinstance(identity, str) or not identity):
                raise ValueError("invalid_source_identity_or_sample_time")
            if last_pub is not None and pub < last_pub:
                raise ValueError("publication_time_regression")
            if row.get("reference_generation") != generation:
                raise ValueError("reference_generation_changed_in_window")
            payload = (pub, acquisition, tuple(row["position"]), row.get("reference_generation"))
            if identity in identities:
                if identities[identity] != payload:
                    raise ValueError("conflicting_source_identity")
                if last_sample is not None and acquisition < last_sample:
                    raise ValueError("source_selection_sample_regression")
            else:
                if last_sample is not None and acquisition <= last_sample:
                    raise ValueError("distinct_source_sample_nonprogress")
                identities[identity] = payload
                last_sample = acquisition
            if not covered or covered[-1] != pub:
                covered.append(pub)
            last_pub = pub
            values = [*row["position"], row["yaw"]]
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
                raise ValueError("nonfinite_measurement")
            if (row["nav_state"] != 14 or row["arming_state"] != 2 or row["landed"] is not False
                    or row.get("health_valid", True) is not True):
                raise ValueError("phase_mode_armed_airborne_health")
        gaps = [b-a for a, b in zip(covered, covered[1:])]
        elapsed_us = covered[-1]-covered[0]
        horizontal = max(math.hypot(s["position"][0]-target["position"][0], s["position"][1]-target["position"][1]) for s in samples)
        height = max(abs(s["position"][2]-target["position"][2]) for s in samples)
        yaw = max(abs(math.atan2(math.sin(s["yaw"]-target["yaw"]), math.cos(s["yaw"]-target["yaw"]))) for s in samples)
        checks = {"required_duration": elapsed_us >= round(duration*1e6),
                  "positive_publication_coverage": bool(gaps) and min(gaps) > 0,
                  "maximum_uncovered_interval": bool(gaps) and max(gaps) <= round(config["max_sample_gap_sim_s"]*1e6),
                  "all_record_horizontal": horizontal <= config["horizontal_tolerance_m"],
                  "all_record_height": height <= config["height_tolerance_m"],
                  "all_record_yaw": yaw <= math.radians(config["yaw_tolerance_deg"])}
        return {"status": "passed" if all(checks.values()) else "failed", "contract": "p3-sample-evidence-v2",
                "checks": checks, "raw_evaluations": len(samples), "distinct_observations": len(identities),
                "reused_observations": len(samples)-len(identities), "covered_publication_times": len(covered),
                "equal_publication_distinct_outputs": len(identities)-len(covered), "samples": len(samples),
                "window_publication_us": [covered[0], covered[-1]], "duration_sim_s": elapsed_us/1e6,
                "max_gap_sim_s": max(gaps, default=0)/1e6, "max_horizontal_error_m": horizontal,
                "max_height_error_m": height, "max_yaw_error_deg": math.degrees(yaw)}
    except ValueError as exc:
        return {"status": "failed", "reason": str(exc), "contract": "p3-sample-evidence-v2"}
    except (KeyError, TypeError, IndexError):
        return {"status": "unknown", "reason": "missing_source_or_measurement_fields", "contract": "p3-sample-evidence-v2"}


def fixed_window_us(window, historical=False):
    """Native new boundaries; explicitly mapped legacy display fields only."""
    if not historical:
        start, end = window['start_us'], window['end_us']
        from .sample_identity import stamp_us
        stamp_us(start,'window_start'); stamp_us(end,'window_end')
        if (type(start) is not int or type(end) is not int or end < start
                or window['start_sim_s'] != start/1e6 or window['end_sim_s'] != end/1e6):
            raise ValueError('invalid_native_window_boundaries')
        return start, end
    return round(window['start_sim_s']*1e6), round(window['end_sim_s']*1e6)


def evaluate_flight(result, samples, config, historical=False):
    if result.get("final_state") != "COMPLETE" or result.get("failure"):
        return {"status": "failed", "reason": "controller_not_complete"}
    if result.get("origin_ned") is None or result.get("initial_yaw_ned") is None:
        return {"status": "unknown", "reason": "missing_origin"}
    if config.get("reference_policy") in ("p2-estimator-reference-v2", "p2-estimator-reference-v3"):
        reference = result.get("reference")
        try:
            valid = (reference["policy"] == config["reference_policy"] and reference["state"] == "locked"
                     and reference["lock_sim_s"] is not None and reference["stable_since_sim_s"] is not None
                     and reference["lock_sim_s"]-reference["stable_since_sim_s"] >= config["reference_settings"]["stable_sim_s"]
                     and reference["rejected_count"] == 0 and reference["rejection"] is None
                     and reference["accepted_count"] == len(reference["accepted"]) <= 1
                     and (reference["accepted_count"] == 1 or reference["aligned_at_preparation"] is True)
                     and tuple(reference["ground_origin"]) == tuple(result["origin_ned"]))
            corrected = reference["initial_anchor"]+sum(e["delta_heading"] for e in reference["accepted"])
            error = corrected-result["initial_yaw_ned"]
            valid = valid and math.isfinite(error) and abs(math.atan2(math.sin(error),math.cos(error))) < 1e-6
            if not valid:
                return {"status": "failed", "reason": "invalid_reference_lock_evidence"}
        except (KeyError, TypeError):
            return {"status": "unknown", "reason": "missing_reference_lock_evidence"}
    if config.get("reference_policy") == "p2-estimator-reference-v3":
        try:
            handover = result["handover"]
            if (handover["start_sim_s"] < reference["lock_sim_s"]
                    or handover["completed_sim_s"] <= handover["start_sim_s"]
                    or abs(handover["corrected_anchor"]-result["initial_yaw_ned"]) > 1e-6
                    or tuple(handover["ground_origin"]) != tuple(result["origin_ned"])
                    or result["max_initialization_drift_deg"] > config["yaw_tolerance_deg"]):
                return {"status": "failed", "reason": "invalid_yaw_handover_evidence"}
        except (KeyError, TypeError):
            return {"status": "unknown", "reason": "missing_yaw_handover_evidence"}
    transactions = result.get("transactions", [])
    if [r.get("command") for r in transactions] != [176, 400, 21] or any(r.get("status") != "accepted" for r in transactions):
        return {"status": "failed", "reason": "missing_command_acceptance"}
    fixture = expected_fixture(result["origin_ned"], result["initial_yaw_ned"], config)
    windows = {}
    for phase in PHASES:
        window = result.get("windows", {}).get(phase)
        if not window:
            return {"status": "unknown", "reason": "missing_window:"+phase}
        revised = config.get("sample_evidence_contract") == "p3-sample-evidence-v2"
        if revised:
            start_us, end_us = fixed_window_us(window, historical)
            selected = [s for s in samples if start_us <= s["timestamp_us"] <= end_us]
        else:
            selected = [s for s in samples if window["start_sim_s"]-1e-8 <= s["t"] <= window["end_sim_s"]+1e-8]
        duration = config["initial_hover_sim_s"] if phase == "INITIAL_HOVER" else config["final_hover_sim_s"] if phase == "FINAL_HOVER" else config["dwell_sim_s"]
        windows[phase] = (evaluate_window_v2 if revised else evaluate_window)(selected, fixture[phase], duration, config)
    return {"status": "passed" if all(w["status"] == "passed" for w in windows.values()) else "failed",
            "windows": windows, "expected_ned_fixture": fixture}
