"""P4 initial-planning feasibility (evidence for proposed contract revision 3).

Frames from the independent renderer are integrated exactly as the perception
node does. The proposed L2 'traversed' rule is simulated on a grid copy only;
the production planner is unchanged until the revision is approved.
"""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p4_synthetic as syn  # noqa: E402
from gwm_obstacle_avoidance import local_map, perception, planner  # noqa: E402
from gwm_sensor_adapter.geometry import Extrinsic  # noqa: E402

SENSORS = syn.sensors()
CONFIG = syn.p4_config()
P = CONFIG["perception"]
EXTRINSIC = Extrinsic(SENSORS["optical_origin_base_flu_m"])
COLUMNS = np.arange(2, 640, 4)
R_BODY = CONFIG["envelope"]["r_body_m"]
HOVER = (0.0, 0.0, 2.0)


def integrate_frame(grid, vehicle, yaw, boxes=()):
    position, q = syn.px4_pose(vehicle, yaw)
    image = syn.render(vehicle, yaw, syn.boxes_3d(list(boxes)), columns=COLUMNS)
    camera_xy, rays = perception.extract_scan(image, syn.CAL, EXTRINSIC, position, q, P,
                                              SENSORS["near_m"], SENSORS["far_m"], 0.0)
    perception.integrate(grid, camera_xy, rays, P["max_hit_range_m"], SENSORS["near_m"])


def start_map(scan, boxes=()):
    """Hover frames at yaw 0, then the scan at 10 deg/s with 10 Hz perception (1 deg per frame)."""
    grid = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
    degrees = [0] * 20 + (list(range(0, -31, -1)) + list(range(-30, 31)) + list(range(30, -1, -1))
                          if scan == "pm30" else list(range(0, 361)))
    for d in degrees:
        integrate_frame(grid, HOVER, math.radians(d), boxes)
    return grid


def footprint_unknown_radii(grid, center=(0.0, 0.0), radius=1.0):
    window, _ = grid._window((center[0],), (center[1],), radius)
    ii, jj, cx, cy = window
    distance = np.hypot(cx - center[0], cy - center[1])
    unknown = (distance <= radius) & (grid.cells[ii, jj] == local_map.UNKNOWN)
    return distance[unknown], np.degrees(np.arctan2(cy[unknown], cx[unknown]))


def with_traversed(grid, positions):
    """Proposed L2: cells within R_body of an earlier position count as FREE (never over OCCUPIED)."""
    mask = planner.traversed_mask(grid, positions, R_BODY)
    copy = local_map.Grid(grid.x_min, grid.y_min, grid.nx, grid.ny, grid.resolution, grid.cells.copy())
    copy.cells[mask & (copy.cells == local_map.UNKNOWN)] = local_map.FREE
    return copy, mask


def context(x=0.0, yaw=0.0):
    return planner.Context((x, 0.0), (x, 0.0), yaw, (8.0, 0.0), (0.0, 0.0), 0.05, CONFIG["geofence"])


@pytest.fixture(scope="module")
def maps():
    return {"pm30": start_map("pm30"), "full360": start_map("full360")}


def test_approved_pm30_scan_leaves_the_start_footprint_unobserved(maps):
    radii, bearings = footprint_unknown_radii(maps["pm30"])
    outside_body = radii > R_BODY
    assert outside_body.sum() > 100 and np.abs(bearings[outside_body]).max() > 170  # beside and behind
    assert not planner.path_status(maps["pm30"], context(), CONFIG["planner"], 0.25)["valid"]
    traversed, mask = with_traversed(maps["pm30"], [(0.0, 0.0)])
    assert not planner.path_status(traversed, context(), CONFIG["planner"], 0.25)["valid"]
    decision = planner.plan(traversed, mask, context(), CONFIG["planner"], CONFIG["envelope"], 0.25)
    assert decision["action"] == planner.SCAN and decision["candidates_valid"] == 0  # S1 would HOVER


def test_full_scan_leaves_only_the_near_clip_blind_zone_inside_the_body_disc(maps):
    radii, _ = footprint_unknown_radii(maps["full360"])
    assert len(radii) > 0, "near-clip blind zone must remain unknown (not marked free)"
    blind = 0.13233 + SENSORS["near_m"]  # optical offset + near clip
    assert radii.max() <= blind and radii.max() < R_BODY
    assert not planner.path_status(maps["full360"], context(), CONFIG["planner"], 0.25)["valid"]  # literal L2


def test_full_scan_with_traversed_disc_starts_s1_without_hovering(maps):
    traversed, mask = with_traversed(maps["full360"], [(0.0, 0.0)])
    assert planner.path_status(traversed, context(), CONFIG["planner"], 0.25)["valid"]
    decision = planner.plan(traversed, mask, context(), CONFIG["planner"], CONFIG["envelope"], 0.25)
    assert decision["action"] == planner.TRANSLATE and decision["best"]["k"] == 0
    assert decision["best"]["length"] == max(CONFIG["planner"]["lengths_m"])


def test_traversed_never_overrides_an_observed_obstacle():
    grid = start_map("full360", boxes=[{"center": [0.0, 0.75], "size": [0.2, 0.2]}])  # beside the vehicle
    traversed, _ = with_traversed(grid, [(0.0, 0.0)])
    status = planner.path_status(traversed, context(), CONFIG["planner"], 0.25)
    assert not status["valid"] and status["reason"] == "occupied"


def test_s1_straight_transit_stays_valid_beyond_the_initial_scan_radius(maps):
    grid = local_map.Grid(maps["full360"].x_min, maps["full360"].y_min, maps["full360"].nx, maps["full360"].ny,
                          maps["full360"].resolution, maps["full360"].cells.copy())
    positions = [(0.0, 0.0)]
    x = 0.0
    while x <= 6.5:  # 0.03 m per 10 Hz frame = 0.3 m/s
        integrate_frame(grid, (x, 0.0, 2.0), 0.0)
        positions.append((x, 0.0))
        traversed, _ = with_traversed(grid, positions[-60:])
        assert planner.path_status(traversed, context(x), CONFIG["planner"], 0.25)["valid"], x
        x += 0.03
