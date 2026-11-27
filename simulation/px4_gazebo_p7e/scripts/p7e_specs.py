"""P7-E E1 specification: the 30-pair engineering-validation schedule (contract `p7-engineering-validation-v1` §3).

Offline only. It reads the frozen P7 layout document (`p7_layouts.json`, canonical d742b7af…) and checks it against
the approved hash, and it never writes a P7 file. It reuses the frozen P7 generator's random stream unchanged, by
import (`p7_specs.draw`, `shuffle`, `balanced_firsts`).

* **Layouts:** the frozen confirmatory pool, indices 200-229 in seed order (§3.1). They are not in the 200-pair P7-A
  schedule (§3.2); the overlap with the unselected 250-pair alternative is recorded.
* **Order:** a permutation from the SHA-256 counter stream seeded by the layouts' canonical hash, with the new labels
  `p7e-30-order` and `p7e-30-block-<b>-first`. No seed is chosen by hand.
* **Blocks:** 3 blocks of 10 pairs; within each block, 5 pairs fly P4 first.

Usage (WSL, from /tmp, PYTHONDONTWRITEBYTECODE=1):
  p7e_specs.py generate --output OUT.json   (exclusive create)
  p7e_specs.py verify --schedule PATH        (regenerates; any difference fails)
"""
import argparse
import json
from pathlib import Path
import sys

P7E = Path(__file__).resolve().parents[1]
REPO = P7E.parents[1]
P7 = REPO / "simulation/px4_gazebo_p7"
for _path in (P7 / "scripts", REPO / "simulation/px4_gazebo/validation"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import p7_specs  # noqa: E402  (frozen P7 code, used unchanged)
from p3_provenance import atomic_json, canonical_sha256, strict_json  # noqa: E402

CONTRACT = "p7e-schedule-v1"
LAYOUTS_CANONICAL = "d742b7af47c6339fb6d8478870d0cf767b6f09cf41d0ef23d6661bc0b1d2258d"  # approved M1 layouts
P7A_SCHEDULE_CANONICAL = "ef74c71d4242409793c6de759454a498481564c75a3dd13e3402294277ba7c88"  # approved, unchanged
POOL_INDICES = range(200, 230)
BLOCK_PAIRS, BLOCKS = 10, 3
ORDER_LABEL, FIRST_LABEL = "p7e-30-order", "p7e-30-block-%d-first"


def frozen_inputs():
    layouts = strict_json(P7 / "configs/p7_layouts.json")
    schedule = strict_json(P7 / "configs/p7_schedule.json")
    if canonical_sha256(layouts) != LAYOUTS_CANONICAL:
        raise SystemExit("the frozen P7 layouts differ from the approved d742b7af…")
    if canonical_sha256(schedule) != P7A_SCHEDULE_CANONICAL:
        raise SystemExit("the frozen P7-A schedule differs from the approved ef74c71d…")
    return layouts, schedule


def schedule_document(layouts, p7a_schedule):
    seed = canonical_sha256(layouts)
    pool = sorted(layouts["sets"]["confirmatory"]["layouts"], key=lambda r: r["index"])
    chosen = [pool[i] for i in POOL_INDICES]
    order = p7_specs.shuffle([r["id"] for r in chosen], seed, ORDER_LABEL)
    by_id = {r["id"]: r for r in chosen}
    slots = []
    for b in range(1, BLOCKS + 1):
        block = order[(b - 1) * BLOCK_PAIRS:b * BLOCK_PAIRS]
        firsts = p7_specs.balanced_firsts(len(block), seed, FIRST_LABEL % b)
        for j, (layout, first) in enumerate(zip(block, firsts), 1):
            r = by_id[layout]
            second = "p5" if first == "p4" else "p4"
            slots.append({"slot": (b - 1) * BLOCK_PAIRS + j, "block": b, "pair_in_block": j, "layout": layout,
                          "pool_index": r["index"], "seed": r["seed"], "family": r["family"], "boxes": len(r["boxes"]),
                          "world_sha256": r["world_sha256"], "first": first, "order": [first, second]})
    ids = {r["id"] for r in chosen}
    in_200 = {s["layout"] for s in p7a_schedule["confirmatory"]["200"]["slots"]}
    in_250 = {s["layout"] for s in p7a_schedule["confirmatory"]["250"]["slots"]}
    return {"contract": CONTRACT, "schema_version": 1, "governed_by": "p7-engineering-validation-v1 §3",
            "layouts_canonical_sha256": seed, "pool": "confirmatory", "pool_indices": [POOL_INDICES[0], POOL_INDICES[-1]],
            "rng": "the frozen P7 stream: draw(label, i) = int(SHA-256('<layouts_canonical_sha256>:<label>:<i>')), "
                   "big-endian; Fisher-Yates from the end; labels '%s' and '%s'" % (ORDER_LABEL, FIRST_LABEL % 0),
            "controllers": {"p4": "accepted P4 stack, manifest c110bd30", "p5": "accepted P5 stack, manifest 2becfbbb"},
            "pair_rule": "the two flights of a pair run back to back in the listed order",
            "block_pairs": BLOCK_PAIRS, "blocks": BLOCKS, "pairs": len(slots), "flights": 2 * len(slots),
            "non_contamination": {"p7a_schedule_canonical_sha256": canonical_sha256(p7a_schedule),
                                  "overlap_with_p7a_200_pair_schedule": len(ids & in_200),
                                  "overlap_with_unselected_250_pair_alternative": len(ids & in_250),
                                  "limitation": "a future P7-A expansion beyond 200 pairs in pool order would include these "
                                                "30 layouts; it must exclude them by a pre-fixed rule or state they are no "
                                                "longer unseen (contract §3.2)"},
            "slots": slots}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    g = sub.add_parser("generate")
    g.add_argument("--output", type=Path, required=True)
    v = sub.add_parser("verify")
    v.add_argument("--schedule", type=Path, required=True)
    args = parser.parse_args(argv)
    value = schedule_document(*frozen_inputs())
    if args.command == "generate":
        atomic_json(args.output, value, exclusive=True)
    elif strict_json(args.schedule) != value:
        raise SystemExit("differs from its regeneration: %s" % args.schedule)
    print(json.dumps({"schedule_canonical_sha256": canonical_sha256(value)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
