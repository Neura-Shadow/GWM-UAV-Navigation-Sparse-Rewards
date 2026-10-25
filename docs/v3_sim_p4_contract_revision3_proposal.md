# P4 contract `p4-acceptance-v1` — revision 3 proposal (initial planning feasibility)

**Status: PROPOSAL ONLY — NOT APPROVED, NOT APPLIED.** It amends the approved
[revision 2](v3_sim_p4_contract.md) (commit `d3c06b4`). No threshold, limit, speed, scenario or gate
changes. Nothing was launched.

## Problem

Under revision 2, L2 requires every cell within `r_infl` = 1.0 m of a candidate's swept segment to
be observed `FREE`. At the start that footprint is a 1.0 m disc around the takeoff point. The camera
looks forward, so the approved `MAP_SCAN` (−30° → +30° → 0°) cannot observe that whole disc. No
first candidate is ever collision-valid, and **every scenario, including the obstacle-free S1,
would `HOVER` and `ABORT`**.

## Evidence (offline, `tests/test_px4_gazebo_p4_feasibility.py`)

The tests integrate frames rendered by an independent closed-form renderer (`tests/p4_synthetic.py`)
exactly as the perception node does. The vehicle hovers at 2.0 m, perception runs at 10 Hz and the
yaw rate is 10°/s, so each frame advances 1°.

| Initial scan | Unknown cells in the 1.0 m start footprint | Straight path (L2 as written) | With the proposed L2 |
|---|---|---|---|
| −30° → +30° → 0° (revision 2) | 168 beyond `R_body`, bearings up to ±177° (beside and behind) | invalid | still invalid; the planner can only `SCAN`, so S1 would `HOVER` |
| one 360° revolution | 24, all within **0.255 m** of the centre | invalid | **valid; `TRANSLATE` along the path, k = 0, L = 1.5 m** |

* **Near-clip blind zone.** Nothing closer than the camera's near clip (0.2 m) is observed, and a
  near-clipped object may be invisible rather than reported too close. Perception therefore marks
  free space only from the near clip outward. This is an implementation tightening allowed by
  revision 2's P3 rule, already applied in `perception.integrate`. After a full revolution, the
  cells left unknown are those within optical offset + near clip (0.13 + 0.2 m) of the centre. All
  of them are **inside the body disc** (`R_body` 0.39 m) that the vehicle itself occupies.
* **Transit.** With both changes, the S1 straight transit at 0.3 m/s (one frame per 0.03 m) keeps
  the path collision-valid at every step out to 6.5 m. That is beyond the 4.0 m free cap of the
  initial scan, so cells ahead keep being observed in time.
* **Obstacles.** An obstacle beside the vehicle is still `OCCUPIED` and blocks the path. The
  traversed allowance never overrides an observed obstacle (tested).
* **Grid and yaw.** These results use the pinned 0.10 m grid, the pinned 160-column stride and
  yaw-dependent ray integration through the PX4 attitude.

## Proposed changes (rules only)

1. **§4 mission step 2 — `MAP_SCAN`.** One full 360° in-place revolution in a fixed direction
   (counter-clockwise in ENU), ending at the initial heading, at ≤ 10°/s (about 36 s), executed as an
   L5 yaw. This replaces −30° → +30° → 0°.
2. **§5 L2 — collision validity.** A cell counts as `FREE` if it is observed `FREE` **or traversed**:
   within `R_body` of an earlier position estimate, the same definition L5 already uses. Traversed
   never overrides `OCCUPIED`, and `UNKNOWN` cells outside the traversed region still invalidate a
   candidate. The physical basis is that the vehicle's own body occupied the space, and the grid
   already assumes a static world (P3 rule).
3. **§5 P3 — perception.** Each ray marks `FREE` only from the camera's near clip (`near_m` 0.2 m)
   outward. A too-close return is `OCCUPIED` at the near clip. This text records the tightening
   already implemented.

**Unchanged:** `r_infl` 1.0 m, `R_body` 0.39 m, `d_stop`, trigger range, free cap, every §6 and §6.1
limit, speeds and yaw rate, scenarios S1–S7, gates, P2/P3 limits, and the rule that translation
never enters `UNKNOWN` space.

## Timing impact

`MAP_SCAN` grows from about 12 s to about 36 s. The worst case stays inside P2's unchanged 240 s
simulation deadline:

| Phase | Time |
|---|---|
| Initialization through `INITIAL_HOVER` (P3 flights) | ≈ 38 s |
| `MAP_SCAN` | ≈ 37 s |
| `TRANSIT` deadline | 150 s |
| `GOAL_HOVER` | 5 s |
| Landing | ≈ 7 s |
| **Total** | **≈ 237 s** |

An `ABORT` at the transit deadline skips `GOAL_HOVER`.

## Tests added with the revision

`tests/test_px4_gazebo_p4_feasibility.py` (5 tests, passing):

* the revision 2 scan cannot start S1;
* the 360° scan leaves only the near-clip zone, inside `R_body`;
* the 360° scan plus the traversed rule starts S1 with `TRANSLATE`;
* traversed never overrides an obstacle;
* the S1 transit stays valid.

On approval, the planner and supervisor tests (M3) and the mission tests (M4) will cover the
production implementation of rules 1 and 2. `contracts.py` will then pin `map_scan_deg` = 360.
