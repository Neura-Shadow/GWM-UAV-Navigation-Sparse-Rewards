# v3-SIM P7-E E1 — specifications: the 30-pair schedule and the engineering report plan

**Status: APPROVED by the operator (2026-11-22, "Approve specs"); E1 COMPLETE.**
* **Approved canonical SHA-256s:** schedule `9b3deacb3ee77a0baa38c78fa9197c22e569150ff6d4c76b489c494b6aea377b`, report
  plan `15e7f7b460277e650823342bbf33a63426c226730192f3bbc2c536ab70c31d94`.
* The P7-E candidate must bind exactly these. Any change is a new specification and needs a new approval.
* **Authorized:** by the operator ("Authorize E1"), under the approved contract
  [`p7-engineering-validation-v1`](v3_sim_p7e_contract_draft.md) §3, §5–§8 and §10.
* **What it did not do:** no frozen P7 file, candidate or evidence changed; no selection record; no freeze, SITL,
  deletion or push.
* **Where the files are:** a new tree, `simulation/px4_gazebo_p7e/`, plus one new test file. The frozen P7 candidate
  `0799c37e…` still verifies; its source binding does not pick up any P7-E file.

## 1. Files and hashes (for approval)

| File | Content | Canonical SHA-256 | File SHA-256 |
|---|---|---|---|
| [`p7e_schedule.json`](../simulation/px4_gazebo_p7e/configs/p7e_schedule.json) | the 30-pair schedule | `9b3deacb3ee77a0baa38c78fa9197c22e569150ff6d4c76b489c494b6aea377b` | `3642b14d…` |
| [`p7e_report_plan.yaml`](../simulation/px4_gazebo_p7e/configs/p7e_report_plan.yaml) | the engineering report definition | `15e7f7b460277e650823342bbf33a63426c226730192f3bbc2c536ab70c31d94` | `db9bc05e…` |
| [`p7e_specs.py`](../simulation/px4_gazebo_p7e/scripts/p7e_specs.py) | generator and verifier of the schedule | — | `0f05b513…` |
| [`test_px4_gazebo_p7e_specs.py`](../tests/test_px4_gazebo_p7e_specs.py) | 8 offline tests, all passing | — | `21e49fa8…` |

* Canonical hashes are of the parsed JSON, independent of line endings (as for M1).
* `p7e_specs.py verify` regenerates the schedule identically.

## 2. The schedule (contract §3)

**How it was made:**
* **Source:** the frozen confirmatory pool of `p7_layouts.json`, which the generator refuses unless its canonical
  hash is the approved `d742b7af…`. It takes indices **200–229** in seed order (seeds 4,000,349–4,000,398).
* **Order:** the frozen P7 stream, imported unchanged and seeded by the layouts' canonical hash, with the new labels
  `p7e-30-order` and `p7e-30-block-<b>-first`. No seed was chosen by hand.
* **Blocks:** 3 blocks of 10 pairs (20 flights each). In every block, exactly 5 pairs fly P4 first. Each pair's two
  flights run back to back.
* **Integrity:** each slot carries the layout's pool index, seed, family, box count and frozen world SHA-256, so the
  runner's C6 check applies.

**Non-contamination (contract §3.2, checked by test):**
* There is **no** overlap with the P7-A 200-pair schedule (`ef74c71d…`, unchanged), and no overlap with the pilot
  layouts.
* The unselected 250-pair alternative contains all 30. The limitation is recorded in the schedule itself.

**Block 1** (10 pairs, 20 flights, 5 P4-first):
* 1: `p7c-4000351` (201, pillars, P4 first)
* 2: `p7c-4000367` (214, wall with gap, P4 first)
* 3: `p7c-4000355` (204, wall with gap, P5 first)
* 4: `p7c-4000398` (229, partial wall, P5 first)
* 5: `p7c-4000394` (227, pillars, P5 first)
* 6: `p7c-4000396` (228, mixed, P4 first)
* 7: `p7c-4000381` (222, wall with gap, P5 first)
* 8: `p7c-4000373` (216, mixed, P4 first)
* 9: `p7c-4000377` (219, wall with gap, P5 first)
* 10: `p7c-4000349` (200, mixed, P4 first)

**Block 2** (10 pairs, 20 flights, 5 P4-first):
* 11: `p7c-4000357` (206, wall with gap, P5 first)
* 12: `p7c-4000375` (218, mixed, P5 first)
* 13: `p7c-4000374` (217, pillars, P5 first)
* 14: `p7c-4000379` (221, wall with gap, P4 first)
* 15: `p7c-4000385` (224, pillars, P4 first)
* 16: `p7c-4000363` (210, mixed, P4 first)
* 17: `p7c-4000359` (208, mixed, P4 first)
* 18: `p7c-4000360` (209, partial wall, P4 first)
* 19: `p7c-4000382` (223, mixed, P5 first)
* 20: `p7c-4000354` (203, wall with gap, P5 first)

**Block 3** (10 pairs, 20 flights, 5 P4-first):
* 21: `p7c-4000364` (211, pillars, P5 first)
* 22: `p7c-4000388` (225, partial wall, P5 first)
* 23: `p7c-4000366` (213, mixed, P4 first)
* 24: `p7c-4000368` (215, pillars, P5 first)
* 25: `p7c-4000378` (220, mixed, P4 first)
* 26: `p7c-4000356` (205, wall with gap, P5 first)
* 27: `p7c-4000352` (202, pillars, P4 first)
* 28: `p7c-4000358` (207, wall with gap, P4 first)
* 29: `p7c-4000389` (226, pillars, P5 first)
* 30: `p7c-4000365` (212, pillars, P4 first)

**Family mix** of the 30: 9 mixed, 9 pillars, 9 wall with gap, 3 partial wall. It follows from seed order, not from
any selection.

## 3. The engineering report plan (contract §5–§8, §10)

**Hard criteria H1–H9.** Each has its measure, limit, the source file of the limit, its scope, and its effect on
failure. H1–H6 stop the batch. The tests check every numerical limit against its source:
* collisions 0 and clearance ≥ 0.20 m (`p4_validation.yaml`);
* hold motion 0.01 m (`p6_faults.yaml` H35);
* Policy A `bf1ef9b1…`;
* the command sequence 176/400/21.

**Reported criteria R1–R10.** Each has its measure and the accepted limit it is reported against; the tests check
each against its source:
* P2 freshness 0.2 s (`p2_control.yaml`);
* P4 perception rate 8 Hz, latency 0.25 s / 0.15 s, compute 0.050 s / 0.090 s, receipt gap 1.0 s;
* P5 inference 0.005 s / 0.010 s, participation 1.0;
* tracking 0.30 m, response latency 0.25 s;
* host CPU ≤ 15 %, overlay `ded574b5`.

**Engineering completion:** H1–H9 met and the report complete. An R breach is an integrity event in the risk
register.

**Reporting:**
* descriptive intervals (Clopper–Pearson, Tango, Hodges–Lehmann, zero-event bounds), computed by the frozen P7
  functions (`bd036a2b…`) imported unchanged;
* **no p-values**; the sign is P5 minus P4.

**Required statement.** It is identical to contract §7, verbatim; a test compares the two texts.

**Risk register:** RR1 and RR2 stay open, with their report items.

## 4. Next

* **Operator approval** of the two specifications as hashed in §1: given on 2026-11-22 ("Approve specs").
* **E2** (the P7-E code, offline, including FC-1 to FC-3 and the report code) needs its own authorization.
* **The retention-policy approval** is still a separate decision.
