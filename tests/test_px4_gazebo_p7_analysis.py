"""P7 M3: the frozen analysis code against independent references (offline; synthetic data only).

* Exact McNemar against SciPy's binomial test and P5's sign test.
* The Tango interval against a brute-force score inversion that maximizes the restricted likelihood numerically.
* Wilcoxon (exact and normal) against SciPy; Hodges-Lehmann against brute force and the Hollander-Wolfe table.
* Clopper-Pearson against SciPy's beta quantiles.
* The fixed-seed bootstrap is the P5 comparison's method and reproduces itself.
* The fixed sequence, missing-pair sensitivity, the pilot rule, pair assembly and record-hash integrity.
"""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
P7 = REPO / "simulation/px4_gazebo_p7"
for path in (P7 / "analysis", P7 / "scripts", REPO / "simulation/px4_gazebo_p5/validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p7_analysis as a  # noqa: E402

scipy_stats = pytest.importorskip("scipy.stats")


def test_the_code_implements_the_approved_plan():
    import p7_manifest
    from p3_provenance import canonical_sha256, strict_json
    assert a.PLAN_CANONICAL == p7_manifest.APPROVED_SPECS["analysis_plan"] == canonical_sha256(strict_json(a.PLAN))
    plan = strict_json(a.PLAN)
    assert (a.ALPHA, a.MARGIN_M) == (plan["primary"]["alpha"], plan["secondaries"]["S1"]["margin_m"])
    assert "20261121" in plan["secondaries"]["S1"]["interval"] and a.BOOTSTRAP_SEED == 20261121 and a.BOOTSTRAP_SAMPLES == 10_000
    assert a.PILOT_PLANNING == (27, 243) and a.EXACT_MAX == 25


@pytest.mark.parametrize("b, c", [(26, 5), (5, 26), (0, 0), (3, 0), (10, 10), (40, 25), (1, 7)])
def test_exact_mcnemar(b, c):
    import p5_comparison
    expected = 1.0 if b + c == 0 else scipy_stats.binomtest(min(b, c), b + c, 0.5).pvalue
    assert a.mcnemar_exact(b, c) == pytest.approx(expected, rel=1e-12, abs=1e-15)
    assert a.mcnemar_exact(b, c) == pytest.approx(p5_comparison.sign_test(b, c) or 1.0, rel=1e-12)


def brute_tango(b, c, n, z=1.959963984540054):
    """Score inversion with the restricted MLE found numerically (not from the closed-form root)."""
    from scipy.optimize import minimize_scalar
    m = n - b - c

    def score(d):
        lo, hi = max(0.0, -d) + 1e-12, (1 - d) / 2 - 1e-12

        def nll(x):
            p12, p21, rest = x + d, x, 1 - 2 * x - d
            return -(b * math.log(p12) if b else 0) - (c * math.log(p21) if c else 0) - (m * math.log(rest) if m else 0)
        x = minimize_scalar(nll, bounds=(lo, hi), method="bounded", options={"xatol": 1e-13}).x
        return (b - c - n * d) / math.sqrt(n * (2 * x + d * (1 - d)))
    grid = np.linspace(-0.999, 0.999, 19981)
    inside = [d for d in grid if abs(score(d)) <= z]
    return min(inside), max(inside)


@pytest.mark.parametrize("b, c, n", [(26, 5, 300), (22, 5, 200), (10, 10, 200), (3, 0, 40), (0, 4, 30), (12, 2, 60)])
def test_the_tango_interval_matches_a_brute_force_score_inversion(b, c, n):
    low, high = a.tango_interval(b, c, n)
    blow, bhigh = brute_tango(b, c, n)
    assert low == pytest.approx(blow, abs=2e-4) and high == pytest.approx(bhigh, abs=2e-4)
    assert low < (b - c) / n < high
    assert a.tango_score(b, c, n, 0.0) == pytest.approx((b - c) / math.sqrt(b + c))  # at D = 0 it is McNemar's z


def test_the_tango_restricted_mle_solves_its_score_equation():
    for b, c, n, d in ((26, 5, 300, 0.03), (22, 5, 200, -0.02), (0, 4, 30, -0.05), (7, 2, 50, 0.4)):
        x, m = a.tango_restricted_q(b, c, n, d), n - b - c
        gradient = b / (x + d) + c / x - 2 * m / (1 - 2 * x - d)
        assert abs(gradient) < 1e-6
    assert a.tango_restricted_q(3, 0, 40, 0.2) == 0.0  # c = 0 and D > 0: the MLE is on the boundary q = 0


def test_the_tango_interval_at_the_edges():
    low, high = a.tango_interval(0, 0, 200)
    assert low < 0 < high and high == pytest.approx(-low, rel=1e-6) and high == pytest.approx(3.8415 / (200 + 3.8415), rel=1e-3)
    assert a.tango_interval(5, 0, 5)[1] == 1.0
    assert a.tango_interval(0, 0, 0) is None


@pytest.mark.parametrize("seed, n", [(1, 6), (2, 12), (3, 20), (4, 25)])
def test_the_exact_wilcoxon_matches_scipy(seed, n):
    d = list(np.random.default_rng(seed).normal(0.4, 1.0, n))
    ours = a.signed_rank_test(d)
    theirs = scipy_stats.wilcoxon(d, zero_method="wilcox", method="exact")
    assert ours["method"] == "exact" and ours["p_value"] == pytest.approx(theirs.pvalue, rel=1e-10)


@pytest.mark.parametrize("seed, n, rounding", [(5, 40, None), (6, 160, None), (7, 60, 1), (8, 30, 0)])
def test_the_normal_wilcoxon_with_ties_and_zeros_matches_scipy(seed, n, rounding):
    d = np.random.default_rng(seed).normal(0.2, 1.0, n)
    d = list(np.round(d, rounding) if rounding is not None else d)
    ours = a.signed_rank_test(d)
    theirs = scipy_stats.wilcoxon(d, zero_method="wilcox", correction=True, method="approx")
    assert ours["method"] == "normal" and ours["p_value"] == pytest.approx(theirs.pvalue, rel=1e-9)


def test_the_hodges_lehmann_estimate_and_interval():
    d = [0.8, -0.3, 1.9, 0.4, 1.1, 2.2, -0.6, 0.9, 1.4, 0.2]
    walsh = sorted((d[i] + d[j]) / 2 for i in range(10) for j in range(i, 10))
    hl = a.hodges_lehmann(d)
    assert hl["estimate"] == pytest.approx(float(np.median(walsh)))
    assert hl["method"] == "exact" and hl["walsh_critical_rank"] == 9  # Hollander-Wolfe, n = 10, alpha = 0.05
    assert hl["ci95"] == [walsh[8], walsh[len(walsh) - 9]] and hl["ci95"][0] <= hl["estimate"] <= hl["ci95"][1]
    big = list(np.random.default_rng(9).normal(-0.5, 1.0, 120))
    hl = a.hodges_lehmann(big)
    assert hl["method"] == "normal" and hl["ci95"][0] < hl["estimate"] < hl["ci95"][1] < 0
    assert a.hodges_lehmann([])["estimate"] is None


@pytest.mark.parametrize("x, n", [(0, 400), (1, 400), (7, 200), (200, 200), (93, 250)])
def test_clopper_pearson_matches_scipy(x, n):
    low, high = a.clopper_pearson(x, n)
    exp_low = 0.0 if x == 0 else scipy_stats.beta.ppf(0.025, x, n - x + 1)
    exp_high = 1.0 if x == n else scipy_stats.beta.ppf(0.975, x + 1, n - x)
    assert low == pytest.approx(exp_low, abs=1e-9) and high == pytest.approx(exp_high, abs=1e-9)


def test_the_bootstrap_is_the_p5_comparison_method_and_decides_non_inferiority():
    import p5_comparison
    diffs = list(np.random.default_rng(11).normal(-0.01, 0.05, 200))
    ours = a.clearance_noninferiority(diffs)
    theirs = p5_comparison.interval(diffs, np.random.default_rng(a.BOOTSTRAP_SEED), a.BOOTSTRAP_SAMPLES)
    assert ours["ci95"] == theirs and ours == a.clearance_noninferiority(diffs)
    assert ours["non_inferior"] and ours["ci95"][0] > -0.05
    worse = a.clearance_noninferiority(list(np.random.default_rng(12).normal(-0.08, 0.05, 200)))
    assert not worse["non_inferior"]
    assert a.clearance_noninferiority([])["non_inferior"] is False


# ---- pairs, sequence, safety, pilot -----------------------------------------------------------------------------
def record(controller, success, clearance=0.4, ttg=50.0, path=10.0, validity="valid", events=(), unverified=(), mission=()):
    return {"controller": controller, "validity": validity, "outcome": ("goal_success" if success else "failure") if validity == "valid" else None,
            "classes": {"safety_events": list(events), "safety_unverified": list(unverified), "mission_failures": list(mission),
                        "integrity_events": []},
            "metrics": {"min_truth_clearance_m": clearance, "time_to_goal_sim_s": ttg if success else None,
                        "path_length_m": path if success else None}}


def pairs_with(p5_only, p4_only, both, neither, rng=np.random.default_rng(3)):
    out = []
    for kind, count in (("p5", p5_only), ("p4", p4_only), ("both", both), ("neither", neither)):
        for i in range(count):
            s4, s5 = kind in ("p4", "both"), kind in ("p5", "both")
            out.append({"layout": "%s-%d" % (kind, i), "family": "pillars" if i % 2 else "mixed", "boxes": 1 + i % 3,
                        "block": 1 + i % 2, "first": "p4" if i % 2 else "p5",
                        "p4": record("p4", s4, 0.40 + rng.normal(0, 0.02), 50 + rng.normal(0, 1), 10 + rng.normal(0, 0.2)),
                        "p5": record("p5", s5, 0.39 + rng.normal(0, 0.02), 49 + rng.normal(0, 1), 9.9 + rng.normal(0, 0.2))})
    return out


def test_the_fixed_sequence_tests_a_secondary_only_after_its_predecessor():
    strong = a.analyse(pairs_with(26, 3, 140, 31))
    assert strong["primary"]["rejects"] and strong["primary"]["p_value"] < 0.001
    s1, s2 = strong["secondaries"]["S1_clearance_noninferiority"], strong["secondaries"]["S2_time_to_goal"]
    assert s1["tested_in_sequence"] and s1["non_inferior"] and s1["claim"] == "supported"
    assert s2["tested_in_sequence"] and s2["pairs"] == 140
    weak = a.analyse(pairs_with(8, 6, 150, 36))
    assert not weak["primary"]["rejects"]
    for name in ("S1_clearance_noninferiority", "S2_time_to_goal", "S3_path_length"):
        value = weak["secondaries"][name]
        assert not value["tested_in_sequence"] and value["claim"].startswith("descriptive only")
    assert set(strong["exploratory"]["by_family"]) == {"pillars", "mixed"} and strong["exploratory"]["label"].startswith("exploratory")


def test_missing_pairs_are_listed_and_the_sensitivity_counts_them_against_p5():
    pairs = pairs_with(10, 2, 50, 10)
    pairs[0]["p5"] = record("p5", False, validity="infrastructure_invalid")
    result = a.analyse(pairs)
    assert result["missing_pairs"] == [pairs[0]["layout"]] and result["primary"]["pairs"] == 71
    sens = result["primary_sensitivity_missing_as_p4_only"]
    assert sens["pairs"] == 72 and sens["p4_only"] == result["primary"]["p4_only"] + 1


def test_safety_counts_and_bounds():
    records = [record("p4", True) for _ in range(400)] + [record("p5", True) for _ in range(399)] + \
              [record("p5", False, events=["safety:collision_contact"])] + [record("p5", False, validity="infrastructure_invalid")]
    s = a.safety(records)
    assert s["p4"]["flights_with_safety_events"] == 0 and s["p4"]["safety_event_bound"] == pytest.approx(1 - 0.05 ** (1 / 400))
    assert s["p5"]["valid_flights"] == 400 and s["p5"]["flights_with_safety_events"] == 1
    assert s["p5"]["safety_event_bound"] == a.clopper_pearson(1, 400) and s["p5"]["kinds"] == ["safety:collision_contact"]


def pilot_pairs(discordant, valid=20, invalid=0):
    out = []
    for i in range(valid):
        out.append({"layout": "p%d" % i, "p4": record("p4", True), "p5": record("p5", i >= discordant)})
    for i in range(invalid):
        out.append({"layout": "x%d" % i, "p4": record("p4", True, validity="infrastructure_invalid"), "p5": record("p5", True)})
    return out


def test_the_pilot_rule():
    assert a.pilot_rule(pilot_pairs(2))["selected_pairs"] == 250  # 0.10 < 27/243
    assert a.pilot_rule(pilot_pairs(3))["selected_pairs"] == 200  # 0.15
    assert a.pilot_rule(pilot_pairs(27, valid=243))["selected_pairs"] == 200  # equal to the planning value: not below
    rule = a.pilot_rule(pilot_pairs(2, valid=19, invalid=1))
    assert rule["valid_pilot_pairs"] == 19 and rule["discordance"] == pytest.approx(2 / 19)


def test_pairs_are_built_from_the_frozen_schedule_and_records_are_hash_checked(tmp_path):
    from p3_provenance import digest, strict_json
    schedule, layouts = strict_json(P7 / "configs/p7_schedule.json"), strict_json(P7 / "configs/p7_layouts.json")
    slots = schedule["pilot"]["slots"]
    flights = []
    for slot in slots[:2]:
        for controller in slot["order"]:
            run = tmp_path / ("%d-%s" % (slot["slot"], controller))
            (run / "p7").mkdir(parents=True)
            (run / "p7/flight-record.json").write_text(json.dumps(record(controller, True)))
            flights.append({"run": str(run), "block": 1, "slot": slot["slot"], "controller": controller,
                            "flight_record_sha256": digest(run / "p7/flight-record.json")})
    result = tmp_path / "pilot.result.json"
    result.write_text(json.dumps({"scope": "pilot", "flights": flights}))
    pairs = a.build_pairs(a.load_flights([result]), schedule, layouts, "pilot")
    assert len(pairs) == 20 and pairs[0]["layout"] == slots[0]["layout"] and pairs[0]["p4"]["outcome"] == "goal_success"
    assert pairs[2]["p4"] is None and pairs[0]["family"] in {"pillars", "partial_wall", "wall_with_gap", "mixed"}
    (Path(flights[0]["run"]) / "p7/flight-record.json").write_text(json.dumps(record("p4", False)))
    with pytest.raises(ValueError, match="changed"):
        a.load_flights([result])
