"""P7 frozen analysis: the approved analysis plan `p7-analysis-plan-v1` (canonical c9c7a6f3…) as code (contract §6, C7).

Frozen with M3 and hash-bound in the P7 manifest. It runs once, on all pre-registered pairs, with no interim
efficacy analysis. Every difference is P5 minus P4. Pure standard library, plus NumPy for the fixed-seed bootstrap
only. Runtime: WSL /usr/bin/python3 with NumPy 1.26.4.

**Methods:**
* **Primary:** exact two-sided McNemar on the discordant pairs, `p = min(1, 2 P(X <= min(b, c)))`,
  X ~ Bin(b + c, 1/2). The difference (b - c) / n with a 95 % Tango score interval (Tango 1998).
  * **Score:** T(D) = (b - c - nD) / sqrt(n (2 q + D (1 - D))). Here q is the restricted MLE of the P4-only cell
    probability, the positive root of 2n q^2 + (-b - c + (2n - b + c) D) q - c D (1 - D) = 0.
  * **Interval:** {D : |T(D)| <= z_0.975}, found by bisection.
* **S1 (clearance non-inferiority):** the mean paired difference, with a paired bootstrap of 10,000 resamples from
  `numpy.random.default_rng(20261121)`. Non-inferior if the 2.5th percentile > -0.05 m.
* **S2/S3 (time to goal, path length; both-success pairs):**
  * **Test:** two-sided Wilcoxon signed-rank, zeros dropped. Exact if at most 25 non-zero differences and no ties,
    else normal with the tie correction and a 0.5 continuity correction.
  * **Estimate and interval:** the Hodges-Lehmann median of the Walsh averages over all differences, with the
    95 % Walsh-average interval. Its critical value comes from the same exact or normal distribution, with
    m = all differences, and is exact only if the exact test applies and no difference is zero.
* **Fixed sequence:** S1 only if the primary rejects, S2 only if S1 passes, S3 only if S2 rejects.
* **Safety:** counts per controller. Zero events in M flights gives 1 - 0.05^(1/M); otherwise Clopper-Pearson.
* **Missing pairs:** listed. The primary sensitivity counts each as P4 succeeded and P5 failed.
* **Pilot rule:** d = discordant / valid pilot pairs; the 250-pair schedule if d < 27/243, else 200.
"""
import argparse
import json
import math
from pathlib import Path
from statistics import NormalDist
import sys

P7 = Path(__file__).resolve().parents[1]
REPO = P7.parents[1]
sys.path.insert(0, str(REPO / "simulation/px4_gazebo/validation"))
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

CONTRACT = "p7-analysis-result-v1"
PLAN = P7 / "configs/p7_analysis_plan.yaml"
PLAN_CANONICAL = "c9c7a6f3781d3d90cd858de4f2ea9ef8e4bdebcd1b0a8e6ab84918a4f46d9548"
ALPHA, MARGIN_M = 0.05, 0.05
BOOTSTRAP_SAMPLES, BOOTSTRAP_SEED = 10_000, 20261121
EXACT_MAX = 25
PILOT_PLANNING = (27, 243)
NORMAL = NormalDist()


# ---- distributions ----------------------------------------------------------------------------------------------
def binom_cdf(k, n, p):
    if k < 0:
        return 0.0
    if k >= n:
        return 1.0
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k + 1))


def bisect(f, lo, hi, iterations=200):
    """Root of a monotone f on [lo, hi] (f(lo) and f(hi) of opposite sign)."""
    flo = f(lo)
    for _ in range(iterations):
        mid = (lo + hi) / 2
        fm = f(mid)
        if (fm > 0) == (flo > 0):
            lo, flo = mid, fm
        else:
            hi = mid
    return (lo + hi) / 2


def clopper_pearson(x, n, alpha=ALPHA):
    if n == 0:
        return None
    lower = 0.0 if x == 0 else bisect(lambda p: (1 - binom_cdf(x - 1, n, p)) - alpha / 2, 0.0, 1.0)
    upper = 1.0 if x == n else bisect(lambda p: binom_cdf(x, n, p) - alpha / 2, 0.0, 1.0)
    return [lower, upper]


# ---- primary -----------------------------------------------------------------------------------------------------
def mcnemar_exact(b, c):
    return 1.0 if b + c == 0 else min(1.0, 2 * binom_cdf(min(b, c), b + c, 0.5))


def tango_restricted_q(b, c, n, d):
    a, bb, cc = 2 * n, -b - c + (2 * n - b + c) * d, -c * d * (1 - d)
    return (-bb + math.sqrt(max(bb * bb - 4 * a * cc, 0.0))) / (2 * a)


def tango_score(b, c, n, d):
    q = tango_restricted_q(b, c, n, d)
    variance = n * (2 * q + d * (1 - d))
    numerator = b - c - n * d
    if variance <= 0:
        return math.copysign(math.inf, numerator) if numerator else 0.0
    return numerator / math.sqrt(variance)


def tango_interval(b, c, n, alpha=ALPHA):
    if n == 0:
        return None
    z = NORMAL.inv_cdf(1 - alpha / 2)
    estimate, eps = (b - c) / n, 1e-12
    lo_end, hi_end = -1 + eps, 1 - eps
    lower = -1.0 if tango_score(b, c, n, lo_end) <= z else bisect(lambda d: tango_score(b, c, n, d) - z, lo_end, estimate)
    upper = 1.0 if tango_score(b, c, n, hi_end) >= -z else bisect(lambda d: tango_score(b, c, n, d) + z, estimate, hi_end)
    return [lower, upper]


def primary(pairs):
    b = sum(1 for p in pairs if p["p5_success"] and not p["p4_success"])
    c = sum(1 for p in pairs if p["p4_success"] and not p["p5_success"])
    both = sum(1 for p in pairs if p["p4_success"] and p["p5_success"])
    n = len(pairs)
    p = mcnemar_exact(b, c)
    return {"pairs": n, "both_success": both, "p5_only": b, "p4_only": c, "neither": n - both - b - c,
            "p_value": p, "rejects": p < ALPHA, "difference": (b - c) / n if n else None,
            "difference_ci95_tango": tango_interval(b, c, n),
            "p4_success": {"count": both + c, "proportion": (both + c) / n if n else None, "ci95": clopper_pearson(both + c, n)},
            "p5_success": {"count": both + b, "proportion": (both + b) / n if n else None, "ci95": clopper_pearson(both + b, n)}}


# ---- S1: clearance non-inferiority ---------------------------------------------------------------------------------
def clearance_noninferiority(differences):
    import numpy as np
    values = np.asarray(differences, dtype=np.float64)
    if values.size == 0:
        return {"pairs": 0, "mean": None, "ci95": None, "non_inferior": False}
    rng = np.random.default_rng(BOOTSTRAP_SEED)
    means = values[rng.integers(0, values.size, size=(BOOTSTRAP_SAMPLES, values.size))].mean(axis=1)
    low, high = np.percentile(means, [2.5, 97.5])
    return {"pairs": int(values.size), "mean": float(values.mean()), "ci95": [float(low), float(high)],
            "margin_m": MARGIN_M, "non_inferior": bool(low > -MARGIN_M)}


# ---- S2/S3: Wilcoxon signed-rank and Hodges-Lehmann ------------------------------------------------------------
def ranks(values):
    """Average ranks of `values` (1-based) and the sizes of the tie groups."""
    order = sorted(range(len(values)), key=lambda i: values[i])
    out, ties, i = [0.0] * len(values), [], 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        for k in range(i, j + 1):
            out[order[k]] = (i + j) / 2 + 1
        if j > i:
            ties.append(j - i + 1)
        i = j + 1
    return out, ties


def signed_rank_counts(n):
    """counts[w] = number of sign assignments of ranks 1..n with positive-rank sum w."""
    counts = [1] + [0] * (n * (n + 1) // 2)
    for r in range(1, n + 1):
        for w in range(len(counts) - 1, r - 1, -1):
            counts[w] += counts[w - r]
    return counts


def signed_rank_test(differences):
    nonzero = [d for d in differences if d != 0]
    n = len(nonzero)
    if n == 0:
        return {"n_nonzero": 0, "w_plus": 0.0, "method": "none", "p_value": 1.0}
    r, ties = ranks([abs(d) for d in nonzero])
    w = sum(rk for rk, d in zip(r, nonzero) if d > 0)
    if n <= EXACT_MAX and not ties:
        counts = signed_rank_counts(n)
        total, w = 2 ** n, int(round(w))
        below, above = sum(counts[:w + 1]) / total, sum(counts[w:]) / total
        return {"n_nonzero": n, "w_plus": w, "method": "exact", "p_value": min(1.0, 2 * min(below, above))}
    mean = n * (n + 1) / 4
    var = n * (n + 1) * (2 * n + 1) / 24 - sum(t ** 3 - t for t in ties) / 48
    z = max(0.0, abs(w - mean) - 0.5) / math.sqrt(var)
    return {"n_nonzero": n, "w_plus": w, "method": "normal", "z": z, "p_value": min(1.0, 2 * (1 - NORMAL.cdf(z)))}


def hodges_lehmann(differences, alpha=ALPHA):
    m = len(differences)
    if m == 0:
        return {"estimate": None, "ci95": None}
    walsh = sorted((differences[i] + differences[j]) / 2 for i in range(m) for j in range(i, m))
    count = len(walsh)
    estimate = (walsh[(count - 1) // 2] + walsh[count // 2]) / 2
    exact = m <= EXACT_MAX and 0 not in differences and not ranks([abs(d) for d in differences])[1]
    if exact:
        counts, total = signed_rank_counts(m), 2 ** m
        c, cumulative = 0, 0.0
        while c < len(counts) and cumulative + counts[c] / total <= alpha / 2:
            cumulative += counts[c] / total
            c += 1
        critical = c  # the largest C with P(W+ <= C - 1) <= alpha/2
    else:
        critical = math.floor(m * (m + 1) / 4 - NORMAL.inv_cdf(1 - alpha / 2) * math.sqrt(m * (m + 1) * (2 * m + 1) / 24))
    ci = [walsh[critical - 1], walsh[count - critical]] if critical >= 1 else None
    return {"estimate": estimate, "ci95": ci, "walsh_critical_rank": critical, "method": "exact" if exact else "normal"}


def location_endpoint(differences):
    test = signed_rank_test(differences)
    return dict(test, pairs=len(differences), rejects=test["p_value"] < ALPHA, hodges_lehmann=hodges_lehmann(differences))


# ---- safety, pairs, sequence ------------------------------------------------------------------------------------------
def safety(records):
    out = {}
    for controller in ("p4", "p5"):
        valid = [r for r in records if r["controller"] == controller and r["validity"] == "valid"]
        m = len(valid)
        events = sum(1 for r in valid if r["classes"]["safety_events"])
        unverified = sum(1 for r in valid if r["classes"]["safety_unverified"])
        mission = sum(1 for r in valid if r["classes"]["mission_failures"])
        bound = (1 - 0.05 ** (1 / m) if m else None) if events == 0 else clopper_pearson(events, m)
        out[controller] = {"valid_flights": m, "flights_with_safety_events": events, "safety_unverified_flights": unverified,
                           "mission_failure_flights": mission, "kinds": sorted({i for r in valid for i in r["classes"]["safety_events"]}),
                           "safety_event_bound": bound, "bound_kind": "one-sided 95 % upper" if events == 0 else "Clopper-Pearson 95 %"}
    return out


def pair_view(pair):
    """A pair is analysable when both flights are valid; `pair` holds the final valid attempt of each, or None."""
    p4, p5 = pair.get("p4"), pair.get("p5")
    if not (p4 and p5 and p4["validity"] == "valid" and p5["validity"] == "valid"):
        return None
    return {"layout": pair["layout"], "family": pair.get("family"), "boxes": pair.get("boxes"), "block": pair.get("block"),
            "first": pair.get("first"), "p4_success": p4["outcome"] == "goal_success", "p5_success": p5["outcome"] == "goal_success",
            "p4": p4["metrics"], "p5": p5["metrics"]}


def analyse(pairs):
    views, missing = [], []
    for pair in pairs:
        view = pair_view(pair)
        (views.append(view) if view else missing.append(pair["layout"]))
    result = {"primary": primary(views), "missing_pairs": missing}
    sensitivity = views + [{"p4_success": True, "p5_success": False} for _ in missing]
    result["primary_sensitivity_missing_as_p4_only"] = primary(sensitivity) if missing else None
    clearance = [v["p5"]["min_truth_clearance_m"] - v["p4"]["min_truth_clearance_m"] for v in views
                 if v["p4"].get("min_truth_clearance_m") is not None and v["p5"].get("min_truth_clearance_m") is not None]
    both = [v for v in views if v["p4_success"] and v["p5_success"]]
    s1 = clearance_noninferiority(clearance)
    s2 = location_endpoint([v["p5"]["time_to_goal_sim_s"] - v["p4"]["time_to_goal_sim_s"] for v in both])
    s3 = location_endpoint([v["p5"]["path_length_m"] - v["p4"]["path_length_m"] for v in both])
    tested = {"S1": result["primary"]["rejects"]}
    tested["S2"] = tested["S1"] and s1["non_inferior"]
    tested["S3"] = tested["S2"] and s2["rejects"]
    for name, value in (("S1", s1), ("S2", s2), ("S3", s3)):
        value["tested_in_sequence"] = tested[name]
        value["claim"] = ("supported" if (value.get("non_inferior") if name == "S1" else value["rejects"]) else "not supported") \
            if tested[name] else "descriptive only (not reached in the fixed sequence)"
    result["secondaries"] = {"S1_clearance_noninferiority": s1, "S2_time_to_goal": s2, "S3_path_length": s3}
    result["exploratory"] = exploratory(views)
    return result


def exploratory(views):
    def strata(key):
        out = {}
        for v in views:
            row = out.setdefault(str(v.get(key)), {"pairs": 0, "p5_only": 0, "p4_only": 0, "both": 0})
            row["pairs"] += 1
            row["p5_only"] += v["p5_success"] and not v["p4_success"]
            row["p4_only"] += v["p4_success"] and not v["p5_success"]
            row["both"] += v["p4_success"] and v["p5_success"]
        return dict(sorted(out.items()))
    return {"label": "exploratory; no claims", "by_family": strata("family"), "by_box_count": strata("boxes"),
            "by_first_controller": strata("first"), "by_block": strata("block")}


def pilot_rule(pilot_pairs):
    views = [v for v in (pair_view(p) for p in pilot_pairs) if v]
    discordant = sum(1 for v in views if v["p4_success"] != v["p5_success"])
    d = discordant / len(views) if views else None
    pairs = 250 if d is not None and d * PILOT_PLANNING[1] < PILOT_PLANNING[0] else 200
    return {"valid_pilot_pairs": len(views), "discordant": discordant, "discordance": d,
            "planning_value": "%d/%d" % PILOT_PLANNING, "selected_pairs": pairs,
            "rule": "250 if d < 27/243, else 200 (analysis plan sample_size.pilot_rule)"}


# ---- assembling pairs from block results ------------------------------------------------------------------------------
def load_flights(result_paths):
    """Final valid (or last) attempt per (slot, controller) from the block results, each record checked by its hash."""
    chosen = {}
    for path in result_paths:
        block = strict_json(path)
        for f in block.get("flights", []):
            if not f.get("run"):
                continue
            record_path = Path(f["run"]) / "p7/flight-record.json"
            if not record_path.exists() or digest(record_path) != f.get("flight_record_sha256"):
                raise ValueError("flight record missing or changed since its block: %s" % f["run"])
            record = strict_json(record_path)
            key = (block["scope"], f["block"], f["slot"], f["controller"])
            if key not in chosen or chosen[key]["validity"] != "valid":
                chosen[key] = dict(record, _flight=f)
    return chosen


def build_pairs(chosen, schedule, layouts, scope):
    records = {(k[2], k[3]): v for k, v in chosen.items() if k[0] == scope}
    rows = schedule["pilot"]["slots"] if scope == "pilot" else schedule["confirmatory"][scope.split("-")[1]]["slots"]
    info = {r["id"]: r for s in layouts["sets"].values() for r in s["layouts"]}
    return [{"layout": r["layout"], "repeat": r.get("repeat"), "family": info[r["layout"]]["family"],
             "boxes": len(info[r["layout"]]["boxes"]), "block": r["block"], "first": r["first"],
             "p4": records.get((r["slot"], "p4")), "p5": records.get((r["slot"], "p5"))} for r in rows]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("confirmatory", "pilot-rule"))
    parser.add_argument("--scope", required=True)
    parser.add_argument("--results", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if canonical_sha256(strict_json(PLAN)) != PLAN_CANONICAL:
        raise SystemExit("The analysis plan differs from the approved one")
    schedule, layouts = strict_json(P7 / "configs/p7_schedule.json"), strict_json(P7 / "configs/p7_layouts.json")
    pairs = build_pairs(load_flights(args.results), schedule, layouts, args.scope)
    value = {"contract": CONTRACT, "scope": args.scope, "plan_canonical_sha256": PLAN_CANONICAL,
             "analysis_sha256": digest(Path(__file__)), "results": [{"path": str(p), "sha256": digest(p)} for p in args.results]}
    if args.command == "pilot-rule":
        value["pilot_rule"] = pilot_rule(pairs)
    else:
        unflown = [p["layout"] for p in pairs if p["p4"] is None or p["p5"] is None]  # a flight never flown (a stop)
        value.update(analyse([p for p in pairs if p["layout"] not in unflown]), unflown_pairs=unflown,
                     safety=safety([p[c] for p in pairs for c in ("p4", "p5") if p[c]]))
    atomic_json(args.output, value, exclusive=True)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
