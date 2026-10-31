"""Procedural training, validation and held-out layouts (plan §4.2, contract §4.1).

Every layout comes from one recorded seed; the three splits use disjoint seed ranges. Layouts are
1-4 static 4 m boxes: pillars (0.2-0.6 m), partial walls, walls with a gap, and mixes. The start
and goal discs are kept free, the goal must be reachable outside the true obstacles inflated by
r_infl (the oracle's strict field), and no layout may match an acceptance obstacle set (S1-S6,
G2/G4/G6): a set with the same number of boxes where some pairing has every centre within 0.25 m
and every size within 0.10 m is a match and is rejected.
"""
import itertools
import json
import math

import numpy as np

import p5_episode
import p5_oracle
import p5_paths
import p5_render

SPLITS = {"train": (1_000_000, 1200), "validation": (2_000_000, 150), "held_out": (3_000_000, 300)}
SEED_RANGE = 1_000_000
FREE_DISC_M = 1.5
CENTRE_TOL_M, SIZE_TOL_M = 0.25, 0.10
FAMILIES = (("pillars", 0.40), ("wall_with_gap", 0.20), ("partial_wall", 0.15), ("mixed", 0.25))


def acceptance_sets():
    scenarios = json.loads((p5_paths.P4 / "configs/p4_scenarios.json").read_text(encoding="utf-8"))
    out = {name: s["boxes"] for name, s in scenarios["scenarios"].items()}
    out.update({name: f["boxes"] for name, f in scenarios["ground_fixtures"].items()})
    return out


def matches(boxes, accepted, centre_tol=CENTRE_TOL_M, size_tol=SIZE_TOL_M):
    """Contract §4.1 match: same box count and a pairing within the centre and size tolerances."""
    if len(boxes) != len(accepted):
        return False
    for order in itertools.permutations(range(len(accepted))):
        if all(math.dist(boxes[i]["center"], accepted[k]["center"]) <= centre_tol
               and all(abs(a - b) <= size_tol for a, b in zip(boxes[i]["size"], accepted[k]["size"]))
               for i, k in enumerate(order)):
            return True
    return False


def matching_acceptance(boxes):
    return sorted(name for name, accepted in acceptance_sets().items() if matches(boxes, accepted))


def _box(cx, cy, sx, sy):
    return {"center": [round(float(cx), 3), round(float(cy), 3)], "size": [round(float(sx), 3), round(float(sy), 3)]}


def _pillar(rng):
    return _box(rng.uniform(1.5, 7.5), np.clip(rng.normal(0.0, 1.2), -2.8, 2.8), rng.uniform(0.2, 0.6), rng.uniform(0.2, 0.6))


def _partial_wall(rng):
    thickness, length = rng.uniform(0.2, 0.3), rng.uniform(1.0, 3.0)
    if rng.random() < 0.7:  # across the path
        return _box(rng.uniform(2.5, 6.5), rng.uniform(-2.0, 2.0), thickness, length)
    return _box(rng.uniform(2.5, 6.0), rng.uniform(-2.2, 2.2), min(length, 2.5), thickness)  # along the path


def _wall_with_gap(rng):
    x, thickness, width = rng.uniform(2.5, 6.0), rng.uniform(0.2, 0.3), rng.uniform(2.3, 3.2)
    centre = rng.uniform(-(2.9 - width / 2), 2.9 - width / 2)
    lo, hi = centre - width / 2, centre + width / 2
    return [_box(x, (lo - 4.0) / 2, thickness, lo + 4.0), _box(x, (hi + 4.0) / 2, thickness, 4.0 - hi)]


def generate(seed):
    """One candidate layout from its seed (before the acceptance, disc and feasibility filters)."""
    rng = np.random.default_rng(seed)
    names, weights = zip(*FAMILIES)
    family = names[int(rng.choice(len(names), p=np.array(weights)))]
    if family == "pillars":
        boxes = [_pillar(rng) for _ in range(int(rng.integers(1, 5)))]
    elif family == "wall_with_gap":
        boxes = _wall_with_gap(rng)
    elif family == "partial_wall":
        boxes = [_partial_wall(rng)]
    else:
        boxes = [_partial_wall(rng)] + [_pillar(rng) for _ in range(int(rng.integers(1, 3)))]
    return {"seed": int(seed), "family": family, "boxes": boxes}


def rejection(layout, config):
    """None if the layout is admissible, else the reason."""
    boxes = layout["boxes"]
    goal = tuple(config["mission"]["goal_enu_m"])
    for point in ((0.0, 0.0), goal):
        if float(p5_render.surface_distance(point[0], point[1], boxes)) < FREE_DISC_M:
            return "start_or_goal_disc"
    if matching_acceptance(boxes):
        return "acceptance_layout"
    if not p5_oracle.CostToGo(boxes, config, strict=True).feasible_from((0.0, 0.0)):
        return "goal_unreachable_inflated"
    return None


def _screen(seed):
    _, config, _ = p5_episode.configs()
    layout = generate(seed)
    return layout, rejection(layout, config)


def split(name, mapper=map, chunk=256):
    """The first `count` admissible layouts of a split in seed order, with rejection counts.
    `mapper` may be a parallel map; the result is the same as the sequential one."""
    base, count = SPLITS[name]
    layouts, rejected = [], {}
    offset = 0
    while len(layouts) < count:
        if offset >= SEED_RANGE:
            raise RuntimeError("seed range exhausted for split " + name)
        for layout, reason in mapper(_screen, range(base + offset, base + min(offset + chunk, SEED_RANGE))):
            if len(layouts) == count:
                break
            if reason is None:
                layout["id"] = "%s-%07d" % (name, layout["seed"])
                layouts.append(layout)
            else:
                rejected[reason] = rejected.get(reason, 0) + 1
            offset += 1
    return {"split": name, "seed_base": base, "seeds_tried": offset, "rejected": rejected, "layouts": layouts}
