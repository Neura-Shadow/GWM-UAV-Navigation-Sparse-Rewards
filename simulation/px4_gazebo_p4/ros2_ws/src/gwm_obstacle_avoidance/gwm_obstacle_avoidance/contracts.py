"""Strict P4 configuration contract (p4-acceptance-v1 revisions 2 and 3, §4–§6).

Every value the contract fixes is pinned here, so an edited configuration cannot
silently drift from the approved numbers. P2 timing/freshness limits are read
from the unchanged `p2_control.yaml` through the P2 loader, never copied.
"""
import json
import math
from pathlib import Path

CONTRACT = "p4-avoidance-config-v1"
MAP_CONTRACT = "p4-local-map-v1"
DECISION_CONTRACT = "p4-decision-v1"

PINNED = {
    ("perception", "column_stride"): 4,
    ("perception", "band_below_m"): 0.38,
    ("perception", "band_above_m"): 0.22,
    ("perception", "free_cap_m"): 4.0,
    ("perception", "grid_resolution_m"): 0.1,
    ("perception", "snapshot_max_age_sim_s"): 0.25,
    ("perception", "unusable_frames_to_hover"): 3,
    ("envelope", "r_body_m"): 0.39,
    ("planner", "r_infl_m"): 1.0,
    ("planner", "d_stop_m"): 0.30,
    ("planner", "heading_step_deg"): 10.0,
    ("planner", "heading_steps"): 6,
    ("planner", "lengths_m"): [0.75, 1.5],
    ("planner", "observable_half_angle_deg"): 25.0,
    ("planner", "trigger_range_m"): 3.0,
    ("planner", "min_progress_m"): 0.05,
    ("planner", "weights"): {"progress": 1.0, "deviation": 0.6, "turn": 0.2, "clearance": 0.3},
    ("planner", "clearance_cap_m"): 1.0,
    ("planner", "yaw_disc_margin_m"): 0.10,
    ("planner", "tracking_m"): 0.30,
    ("supervisor", "resume_deviation_m"): 0.10,
    ("supervisor", "hover_timeout_sim_s"): 20.0,
    ("supervisor", "stale_abort_sim_s"): 10.0,
    ("supervisor", "transit_deadline_sim_s"): 150.0,
    ("supervisor", "min_clearance_margin_m"): 0.10,
    ("mission", "goal_enu_m"): [8.0, 0.0],
    ("mission", "map_scan_deg"): 360.0,  # revision 3: one full in-place revolution
    ("mission", "height_m"): 2.0,
    ("mission", "goal_hover_sim_s"): 5.0,
    ("mission", "goal_tolerance_m"): 0.30,
    ("mission", "settled_speed_m_s"): 0.1,
    ("geofence", "x_min_m"): -1.0,
    ("geofence", "x_max_m"): 9.5,
    ("geofence", "y_abs_max_m"): 3.0,
    ("geofence", "height_min_m"): 1.0,
    ("geofence", "height_max_m"): 3.0,
}


def load_config(path):
    config = json.loads(Path(path).read_text())
    if config.get("schema_version") != 1 or config.get("contract") != CONTRACT:
        raise ValueError("Unsupported P4 avoidance configuration")
    if not isinstance(config.get("enabled"), bool):
        raise ValueError("P4 'enabled' must be an explicit boolean")
    for (section, key), expected in PINNED.items():
        if config.get(section, {}).get(key) != expected:
            raise ValueError(f"P4 contract value changed: {section}.{key}")
    for section in ("perception", "envelope", "planner", "supervisor", "mission", "geofence"):
        for key, value in config[section].items():
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"Non-finite P4 setting: {section}.{key}")
    return config


def require_enabled(config, authorization_present):
    """Contract C4: nothing runs unless enabled by an authorized P4 run."""
    if config["enabled"] is not True or not authorization_present:
        raise PermissionError("P4 is disabled: requires enabled=true and a consumed P4 authorization")


def strict_json(value):
    return json.dumps(value, allow_nan=False, separators=(",", ":"))
