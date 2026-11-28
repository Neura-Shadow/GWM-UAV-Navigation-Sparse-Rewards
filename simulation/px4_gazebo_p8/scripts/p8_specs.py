"""P8-M1 specifications (contract `p8-navigation-recovery-v1` §4, §6, §13 item 2). Offline only.

It never starts a simulator, and it writes no P2-P7-E file. Frozen code is reached only by import: the P5 generator,
the P7 world path and random stream (`p7_specs`), and the P4 configuration as data. Inputs are checked by hash:
* the P7 layouts (canonical `d742b7af…`) and the P7-E schedule (canonical `9b3deacb…`);
* the three accepted P7-E block results (`1c8672ab…`, `62c23933…`, `d4da3561…`), and through them each flight's
  hash-bound flight record;
* the P4 avoidance and P2 control configurations, recorded by file hash.

**Outputs** (exclusive create; each is bound by its canonical SHA-256):
* **`p8_layouts.json`, three stages:**
  * V3: the 10 P7-E layouts with a mission failure, which must equal the contract's list;
  * V4: 10 held-out layouts from seed base 6,000,000, chosen by the pre-registered rule below;
  * V5: the first 10 P7-E both-success layouts in slot order, stratified by the batch's family proportions.
* **`p8_schedule.json`:**
  * 3 blocks (V3, V4, V5) of 10 pairs;
  * each pair flown back to back by P8-P4R and P8-P5R;
  * the order and first variant drawn from the frozen P7 stream, seeded by the layouts' canonical hash, with 5 of 10
    pairs P8-P4R first per block.
* **`p8_recovery.json`:**
  * the recovery parameters of contract §4, each from an accepted configuration value or derived by the contract's
    stated rule from recorded evidence;
  * every input hash.

**V4 rule (contract §6, P8-D6):**
* in seed order from 6,000,000, layouts admitted by the accepted P5 generator's filters;
* excluding matches with the 1,650 P5 split layouts (regenerated and checked as P7 did) and with the 260 P7 layouts;
* each admitted layout typed by its true geometry at origin (0, 0) (`p8_geometry`):
  * **F2-type:** clearance capacity in [1.10, 1.35] m;
  * **F1-type:** capacity above 1.35 m and the nearest box within 2.3 m of the goal.
  * Both bounds come from the P7-E evidence: the F2 failures had capacity 1.14–1.30 m, and the F1 failures had goal
    clearance 1.56–2.23 m.
* the first 5 of each type are accepted.

Usage (WSL, from /tmp, PYTHONDONTWRITEBYTECODE=1, NumPy 1.26.4):
  p8_specs.py generate --layouts OUT --schedule OUT --recovery OUT   (exclusive create)
  p8_specs.py verify --layouts PATH --schedule PATH --recovery PATH  (regenerates all three; any difference fails)
"""
import argparse
import bisect
import json
import math
import multiprocessing
import os
from pathlib import Path
import sys

P8 = Path(__file__).resolve().parents[1]
REPO = P8.parents[1]
SIM, P4, P5, P7, P7E = (REPO / "simulation" / n for n in ("px4_gazebo", "px4_gazebo_p4", "px4_gazebo_p5",
                                                         "px4_gazebo_p7", "px4_gazebo_p7e"))
for _path in (P5 / "training", P4 / "scripts", SIM / "validation", P7 / "scripts", P8 / "scripts"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import numpy  # noqa: E402
import p5_layouts  # noqa: E402  (accepted P5 generator, unchanged)
import p7_specs  # noqa: E402  (frozen P7 world path and random stream, unchanged)
import p8_geometry  # noqa: E402
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json, strict_jsonl  # noqa: E402

LAYOUTS_CONTRACT, SCHEDULE_CONTRACT, RECOVERY_CONTRACT = "p8-layouts-v1", "p8-schedule-v1", "p8-recovery-parameters-v1"
GOVERNED_BY = "p8-navigation-recovery-v1"
P7_LAYOUTS_CANONICAL = "d742b7af47c6339fb6d8478870d0cf767b6f09cf41d0ef23d6661bc0b1d2258d"
P7E_SCHEDULE_CANONICAL = "9b3deacb3ee77a0baa38c78fa9197c22e569150ff6d4c76b489c494b6aea377b"
P7E_SHA = "e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608"
P7E_RESULTS = {1: "1c8672ab5d3a10df831a78ee34ea11282983fb9c002d51ef728dcc9884faa441",
               2: "62c23933960e256d9fc0d60b3ede008f9b0872752a46bbd4377cdf0783b56064",
               3: "d4da3561bfac6158b4992731d7721de3c2082cd0e5003907586e0e937d1a84de"}
CONTRACT_V3_SLOTS = [3, 6, 7, 9, 12, 14, 15, 16, 18, 23]  # contract §6
HELD_OUT = {"prefix": "p8h", "seed_base": 6_000_000, "per_type": 5}
F2_CAPACITY_M = (1.10, 1.35)
F1_GOAL_CLEARANCE_MAX_M = 2.3
STAGES = ("V3", "V4", "V5")
STAGE_PAIRS = 10
VARIANTS = {"p4": "p8_p4r", "p5": "p8_p5r"}
ORDER_LABEL, FIRST_LABEL = "p8-block-%d-order", "p8-block-%d-first"
CHUNK = 256
CONFIGS = {"p4_avoidance": "simulation/px4_gazebo_p4/configs/p4_avoidance.yaml",
           "p2_control": "simulation/px4_gazebo/configs/p2_control.yaml"}
R4_MARGIN = 1  # the guard threshold is one more than the largest count seen in any P5 goal success (contract §4)


def sim_root():
    return Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))


# ---- frozen inputs ------------------------------------------------------------------------------------------------
def frozen_inputs(root=None):
    root = Path(root or sim_root())
    layouts = strict_json(P7 / "configs/p7_layouts.json")
    schedule = strict_json(P7E / "configs/p7e_schedule.json")
    if canonical_sha256(layouts) != P7_LAYOUTS_CANONICAL:
        raise SystemExit("the frozen P7 layouts differ from the approved d742b7af…")
    if canonical_sha256(schedule) != P7E_SCHEDULE_CANONICAL:
        raise SystemExit("the frozen P7-E schedule differs from the approved 9b3deacb…")
    flights = []
    for block, sha in P7E_RESULTS.items():
        path = root / "state" / ("p7e-engineering-30-block-%02d-%s.result.json" % (block, P7E_SHA))
        if digest(path) != sha:
            raise SystemExit("the accepted P7-E block %d result differs from %s…" % (block, sha[:8]))
        for f in strict_json(path)["flights"]:
            run = Path(f["run"])
            if digest(run / "p7/flight-record.json") != f["flight_record_sha256"]:
                raise SystemExit("a P7-E flight record differs from its ledger hash: %s" % run.name)
            flights.append(dict(f, outcome=strict_json(run / "p7/flight-record.json")["outcome"]))
    return layouts, schedule, flights


def p7e_slots(schedule, flights):
    by_slot = {s["slot"]: dict(s, outcomes={}) for s in schedule["slots"]}
    for f in flights:
        by_slot[f["slot"]]["outcomes"][f["controller"]] = f["outcome"]
    return [by_slot[k] for k in sorted(by_slot)]


def p7_record(layouts, layout_id):
    return next(r for r in layouts["sets"]["confirmatory"]["layouts"] if r["id"] == layout_id)


def reused(layouts, slot, stage):
    r = p7_record(layouts, slot["layout"])
    return {"id": r["id"], "stage": stage, "source": "p7_layouts confirmatory index %d (P7-E slot %d)" % (r["index"], slot["slot"]),
            "p7e_slot": slot["slot"], "p7e_outcomes": slot["outcomes"], "seed": r["seed"], "family": r["family"],
            "boxes": r["boxes"], "world_sha256": r["world_sha256"],
            "clearance_capacity_m": p8_geometry.clearance_capacity(r["boxes"]),
            "goal_clearance_m": round(p8_geometry.goal_clearance(r["boxes"]), 6)}


def v3_v5(layouts, schedule, flights):
    slots = p7e_slots(schedule, flights)
    failed = [s for s in slots if any(o != "goal_success" for o in s["outcomes"].values())]
    if [s["slot"] for s in failed] != CONTRACT_V3_SLOTS:
        raise SystemExit("the accepted batch's failure slots differ from the contract's V3 list")
    both = [s for s in slots if all(o == "goal_success" for o in s["outcomes"].values()) and len(s["outcomes"]) == 2]
    counts = {}
    for s in slots:
        counts[s["family"]] = counts.get(s["family"], 0) + 1
    quota = largest_remainder(counts, STAGE_PAIRS)
    v5 = []
    for family in sorted(quota):
        v5 += [s for s in both if s["family"] == family][:quota[family]]
    v5.sort(key=lambda s: s["slot"])
    return ([reused(layouts, s, "V3") for s in failed], [reused(layouts, s, "V5") for s in v5],
            {"family_counts_p7e": counts, "v5_quota": quota, "both_success_slots": [s["slot"] for s in both]})


def largest_remainder(counts, total):
    n = sum(counts.values())
    raw = {k: total * v / n for k, v in counts.items()}
    out = {k: int(math.floor(x)) for k, x in raw.items()}
    for k in sorted(raw, key=lambda k: (-(raw[k] - out[k]), k))[:total - sum(out.values())]:
        out[k] += 1
    return out


# ---- V4: the held-out layouts ---------------------------------------------------------------------------------------
def measure(seed):
    """The accepted P5 admission screen, then the P8 geometry of an admitted layout (worker function)."""
    layout, reason = p5_layouts._screen(seed)
    if reason is not None:
        return layout, reason, None
    return layout, None, {"clearance_capacity_m": p8_geometry.clearance_capacity(layout["boxes"]),
                          "goal_clearance_m": round(p8_geometry.goal_clearance(layout["boxes"]), 6)}


def layout_type(features):
    cap, goal = features["clearance_capacity_m"], features["goal_clearance_m"]
    if cap is None:
        return None
    if F2_CAPACITY_M[0] <= cap <= F2_CAPACITY_M[1]:
        return "F2"
    if cap > F2_CAPACITY_M[1] and goal <= F1_GOAL_CLEARANCE_MAX_M:
        return "F1"
    return None


def held_out(pool, p5_boxes, p7_boxes):
    accepted, rejected, not_selected, tried = [], {}, 0, 0
    taken = {"F1": 0, "F2": 0}
    base = HELD_OUT["seed_base"]
    while min(taken.values()) < HELD_OUT["per_type"]:
        if tried >= p5_layouts.SEED_RANGE:
            raise RuntimeError("seed range exhausted")
        for layout, reason, features in pool.imap(measure, range(base + tried, base + tried + CHUNK), chunksize=8):
            if min(taken.values()) >= HELD_OUT["per_type"]:
                break
            tried += 1
            if reason is None and any(p5_layouts.matches(layout["boxes"], other) for other in p5_boxes):
                reason = "p5_split_match"
            if reason is None and any(p5_layouts.matches(layout["boxes"], other) for other in p7_boxes):
                reason = "p7_layout_match"
            if reason is not None:
                rejected[reason] = rejected.get(reason, 0) + 1
                continue
            kind = layout_type(features)
            if kind is None or taken[kind] >= HELD_OUT["per_type"]:
                not_selected += 1
                continue
            taken[kind] += 1
            accepted.append({"id": "%s-%07d" % (HELD_OUT["prefix"], layout["seed"]), "stage": "V4", "type": kind,
                             "index": len(accepted), "seed": layout["seed"], "family": layout["family"],
                             "boxes": p7_specs.named(layout["boxes"]), **features})
    return {"seed_base": base, "seeds_tried": tried, "rejected": dict(sorted(rejected.items())),
            "admitted_not_selected": not_selected, "per_type": HELD_OUT["per_type"], "layouts": accepted}


def layouts_document(processes=16, root=None):
    if numpy.__version__ != p7_specs.NUMPY:
        raise SystemExit("NumPy %s required (found %s)" % (p7_specs.NUMPY, numpy.__version__))
    p7_layouts, schedule, flights = frozen_inputs(root)
    v3, v5, v5_rule = v3_v5(p7_layouts, schedule, flights)
    p7_boxes = [r["boxes"] for name in ("pilot", "confirmatory") for r in p7_layouts["sets"][name]["layouts"]]
    with multiprocessing.Pool(processes) as pool:
        p5_boxes = p7_specs.p5_exclusion(pool)
        v4 = held_out(pool, p5_boxes, [[{"center": b["center"], "size": b["size"]} for b in boxes] for boxes in p7_boxes])
    base_text, base_sha = p7_specs.base_world()
    worlds = p7_specs.world_sha256(base_text, v4["layouts"])
    for layout in v4["layouts"]:
        layout["world_sha256"] = worlds[layout["id"]]
    check = p7_specs.world_sha256(base_text, v3 + v5)  # the reused layouts' worlds must regenerate to their frozen hashes
    if any(check[r["id"]] != r["world_sha256"] for r in v3 + v5):
        raise SystemExit("a reused P7 world does not regenerate to its frozen hash")
    return {"contract": LAYOUTS_CONTRACT, "schema_version": 1, "governed_by": GOVERNED_BY + " §6",
            "runtime": {"numpy": numpy.__version__, "python": sys.version.split()[0]},
            "inputs": {"p7_layouts_canonical_sha256": P7_LAYOUTS_CANONICAL, "p7e_schedule_canonical_sha256": P7E_SCHEDULE_CANONICAL,
                       "p7e_manifest_sha256": P7E_SHA, "p7e_block_results_sha256": {str(k): v for k, v in P7E_RESULTS.items()},
                       "generator_sha256": {path: digest(REPO / path) for path in p7_specs.GENERATOR_FILES},
                       "p8_geometry_sha256": digest(P8 / "scripts/p8_geometry.py"),
                       "base_world": {"asset": p7_specs.WORLD_ASSET, "sha256": base_sha, "pinned_by": p7_specs.P3_MANIFEST}},
            "box_name_rule": "p7_box_<k> (the P7 rule, so that the reused P7 world path generates the worlds unchanged)",
            "outcome_label": p7_specs.OUTCOME_LABEL,
            "rules": {"V3": "every P7-E layout with a mission failure (any controller); must equal contract §6 %s" % CONTRACT_V3_SLOTS,
                      "V4": {"seed_base": HELD_OUT["seed_base"], "admission": "p5_layouts._screen (accepted P5 filters)",
                             "exclusions": ["P5 split match (1,650 layouts, regenerated and checked, as P7)", "P7 layout match (260)"],
                             "F2": "clearance capacity in [%.2f, %.2f] m" % F2_CAPACITY_M,
                             "F1": "clearance capacity > %.2f m and goal clearance <= %.1f m" % (F2_CAPACITY_M[1], F1_GOAL_CLEARANCE_MAX_M),
                             "evidence": "P7-E F2 failures capacity 1.14-1.30 m; F1 failures goal clearance 1.56-2.23 m (cde7fa52…)",
                             "geometry": "p8_geometry at origin (0, 0): 0.05 m grid, fence margin 0.9 m, bisection to 0.01 m",
                             "take": "the first %d of each type in seed order" % HELD_OUT["per_type"]},
                      "V5": "the first both-success P7-E layouts in slot order, per family, with quotas from the batch's "
                            "family counts by largest remainder (10 in all)"},
            "v5_rule_inputs": v5_rule,
            "stages": {"V3": {"layouts": v3}, "V4": v4, "V5": {"layouts": v5}}}


# ---- the schedule -----------------------------------------------------------------------------------------------------
def schedule_document(layouts):
    seed = canonical_sha256(layouts)
    slots = []
    for b, stage in enumerate(STAGES, 1):
        ids = [r["id"] for r in layouts["stages"][stage]["layouts"]]
        if len(ids) != STAGE_PAIRS:
            raise SystemExit("stage %s has %d layouts, not %d" % (stage, len(ids), STAGE_PAIRS))
        order = p7_specs.shuffle(ids, seed, ORDER_LABEL % b)
        firsts = p7_specs.balanced_firsts(len(order), seed, FIRST_LABEL % b)
        by_id = {r["id"]: r for r in layouts["stages"][stage]["layouts"]}
        for j, (layout, first) in enumerate(zip(order, firsts), 1):
            a = VARIANTS[first]
            other = VARIANTS["p5"] if first == "p4" else VARIANTS["p4"]
            r = by_id[layout]
            slots.append({"slot": (b - 1) * STAGE_PAIRS + j, "block": b, "stage": stage, "pair_in_block": j, "layout": layout,
                          "family": r["family"], "world_sha256": r["world_sha256"], "first": a, "order": [a, other]})
    return {"contract": SCHEDULE_CONTRACT, "schema_version": 1, "governed_by": GOVERNED_BY + " §6",
            "layouts_canonical_sha256": seed,
            "rng": "the frozen P7 stream (p7_specs.draw, shuffle, balanced_firsts): SHA-256('<layouts_canonical_sha256>:"
                   "<label>:<i>'), Fisher-Yates from the end; labels '%s' and '%s'" % (ORDER_LABEL % 0, FIRST_LABEL % 0),
            "variants": {"p8_p4r": "accepted P4 stack (c110bd30) plus the P8 recovery layer",
                         "p8_p5r": "accepted P5 stack (2becfbbb, model ee006a80) plus the P8 recovery layer and the R4 guard"},
            "pair_rule": "the two flights of a pair run back to back in the listed order",
            "blocks": {"1": "V3", "2": "V4", "3": "V5"}, "block_pairs": STAGE_PAIRS, "pairs": len(slots), "flights": 2 * len(slots),
            "slots": slots}


# ---- the recovery parameters ------------------------------------------------------------------------------------------
def r4_counts(flight, window, progress):
    """For one P5 flight: the largest number of yaw_to_l4 interventions in any `window` (starting at an intervention),
    overall and among windows whose net progress toward the goal is below `progress`."""
    run = Path(flight["run"])
    goal = strict_json(run / "controller-result.json")["p4"]["goal_enu"]
    ledger = strict_jsonl(run / "p5/policy-ledger.jsonl")
    decisions = [d for d in strict_jsonl(run / "p4/decisions.jsonl") if d.get("event") == "p4_decision" and d.get("vehicle_enu")]
    times = [d["sim_s"] for d in decisions]
    dist = [math.dist(d["vehicle_enu"], goal) for d in decisions]

    def at(t):
        return dist[min(max(bisect.bisect_left(times, t), 0), len(times) - 1)]
    yaw = [r["sim_s"] for r in ledger if r.get("intervention") == "yaw_to_l4"]
    any_max = no_progress_max = 0
    for i, t0 in enumerate(yaw):
        n = bisect.bisect_right(yaw, t0 + window) - i
        any_max = max(any_max, n)
        if at(t0) - at(t0 + window) < progress:
            no_progress_max = max(no_progress_max, n)
    return {"slot": flight["slot"], "outcome": flight["outcome"], "run": run.name, "yaw_to_l4": len(yaw),
            "max_any_window": any_max, "max_no_progress_window": no_progress_max,
            "policy_ledger_sha256": digest(run / "p5/policy-ledger.jsonl"), "decisions_sha256": digest(run / "p4/decisions.jsonl")}


def recovery_document(root=None):
    _, _, flights = frozen_inputs(root)
    avoid = strict_json(REPO / CONFIGS["p4_avoidance"])
    p2 = strict_json(REPO / CONFIGS["p2_control"])
    window = avoid["supervisor"]["hover_timeout_sim_s"]
    rate, speed = p2["yaw_ramp_deg_s"], avoid["mission"]["transit_speed_m_s"]
    progress = avoid["planner"]["min_progress_m"]
    rows = [r4_counts(f, window, progress) for f in sorted(flights, key=lambda f: f["slot"]) if f["controller"] == "p5"]
    successes = [r for r in rows if r["outcome"] == "goal_success"]
    k = max(r["max_any_window"] for r in successes) + R4_MARGIN
    triggers = [r["slot"] for r in rows if r["max_no_progress_window"] >= k]
    if any(r["max_no_progress_window"] >= k for r in successes):
        raise SystemExit("the R4 guard would trigger on a recorded P5 success")
    return {"contract": RECOVERY_CONTRACT, "schema_version": 1, "governed_by": GOVERNED_BY + " §3, §4",
            "sources": {name: {"path": path, "sha256": digest(REPO / path)} for name, path in CONFIGS.items()},
            "trigger": {"supervisor_state": "HOVER", "reason": "no_progress_candidate", "stale_reason": "none",
                        "budget": "not exhausted"},
            "episode": {"window_s": window, "window_source": "p4_avoidance supervisor.hover_timeout_sim_s (unchanged)",
                        "yaw_rate_deg_s": rate, "yaw_rate_source": "p2_control yaw_ramp_deg_s (unchanged)",
                        "max_rotation_deg": window * rate, "speed_m_s": speed,
                        "speed_source": "p4_avoidance mission.transit_speed_m_s (unchanged)", "max_translation_m": window * speed,
                        "attempts": 1, "attempts_rule": "one R1 -> R2 -> R3 pass per HOVER episode (contract S8)",
                        "time_rule": "rotation time plus translation time never exceeds the remaining hover window; the "
                                     "frozen hover_timeout and transit_deadline bound everything"},
            "r1": {"goal_radius_m": round(avoid["mission"]["goal_tolerance_m"] + 0.5, 6),
                   "rule": "goal_tolerance_m (0.30, unchanged) + 0.5 m (contract §4)",
                   "evidence": "the F1 final HOVERs were 0.15-0.48 m from the goal (cde7fa52…)",
                   "blockers": "UNKNOWN cells only; no OCCUPIED cell within r_infl of the terminal segment",
                   "sector_step_deg": avoid["planner"]["heading_step_deg"], "order": "nearest blocking sector first; each once"},
            "r2": {"sector_step_deg": avoid["planner"]["heading_step_deg"], "scope": "frontier sectors outside the +/-60 degree cone",
                   "order": "ascending goal distance of the sector's nearest frontier cell; each once"},
            "r3": {"c_space": "every cell within r_infl_m (%.2f, unchanged) FREE or traversed, inside the fence margin" % avoid["planner"]["r_infl_m"],
                   "segment_max_m": max(avoid["planner"]["lengths_m"]), "segment_source": "p4_avoidance planner.lengths_m maximum",
                   "target": "the goal if reachable, else the reachable frontier cell with the least goal distance, outside dead-end memory",
                   "check": "frozen planner.collision_check at the tick each segment starts; a failure means HOLD (FC-4)"},
            "dead_end_memory": {"radius_m": avoid["envelope"]["r_body_m"], "source": "p4_avoidance envelope.r_body_m",
                                "rule": "cells within the radius of every position where a no_progress_candidate HOVER began"},
            "r4": {"variant": "p8_p5r only", "window_s": window, "net_progress_below_m": progress,
                   "net_progress_source": "p4_avoidance planner.min_progress_m (unchanged)",
                   "yaw_to_l4_at_least": k, "guard_period_s": window,
                   "rule": "no net progress (< net_progress_below_m) over window_s with at least yaw_to_l4_at_least yaw_to_l4 "
                           "interventions; then the frozen P4 selection for guard_period_s",
                   "derivation": "one more than the largest yaw_to_l4 count in any window of any P5 goal success "
                                 "(contract §4: it never triggers on the recorded successes)",
                   "evidence": {"p5_flights": rows, "largest_success_any_window": k - R4_MARGIN,
                                "largest_success_no_progress_window": max(r["max_no_progress_window"] for r in successes),
                                "would_trigger_on_slots": triggers}}}


# ---- entry ------------------------------------------------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("generate", "verify"):
        p = sub.add_parser(name)
        p.add_argument("--layouts", type=Path, required=True)
        p.add_argument("--schedule", type=Path, required=True)
        p.add_argument("--recovery", type=Path, required=True)
    args = parser.parse_args(argv)
    layouts = layouts_document()
    schedule = schedule_document(layouts)
    recovery = recovery_document()
    outputs = ((args.layouts, layouts), (args.schedule, schedule), (args.recovery, recovery))
    if args.command == "generate":
        for path, value in outputs:
            atomic_json(path, value, exclusive=True)
    else:
        for path, value in outputs:
            if strict_json(path) != value:
                raise SystemExit("differs from its regeneration: %s" % path)
    print(json.dumps({"layouts_canonical_sha256": canonical_sha256(layouts), "schedule_canonical_sha256": canonical_sha256(schedule),
                      "recovery_canonical_sha256": canonical_sha256(recovery)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
