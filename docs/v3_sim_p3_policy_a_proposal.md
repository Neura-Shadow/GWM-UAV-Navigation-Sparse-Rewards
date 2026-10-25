# v3-SIM P3 native command policy A — approval proposal

**Status: PROPOSAL ONLY — NOT APPROVED.** The effective default remains `strict_zero`.
Nothing in this document selects, applies or approves policy A, and no runtime was run to
derive it. P3 remains INCOMPLETE; qualification 0/3.

Date: 2026-10-25. Prepared offline from the committed policy file, the evaluator source and
the archived decision package. The archived operator approval (receipt
`4beeaf4e…4711`) covered the archived candidate only and **does not carry over**.

## 1. Why a decision is needed

Under `strict_zero` any native `COMMAND_LONG`/`COMMAND_INT` or active-control frame from a
non-vehicle source on the GCS link fails Gate A. Stock QGC (and the unattended build, which
keeps stock command generation unchanged: `command_generation = "stock unchanged"`) emits
`MAV_CMD_RUN_PREARM_CHECKS` (401) and `MAV_CMD_REQUEST_MESSAGE` (512) during connection
start-up. With `strict_zero` the honest expected Gate A outcome is therefore a failure on
`command_not_permitted_by_policy`. The operator must choose one of:

1. keep `strict_zero` and accept that Gate A is expected to fail on stock management traffic;
2. approve policy A below for this candidate with a bound receipt; or
3. commission the separately reviewed passive-QGC build (archived option B); not built.

## 2. Exact scope of policy A (as committed)

Identity: policy file `simulation/px4_gazebo/configs/p3_native_command_policy.json`
SHA-256 `2e6ab07b2ea725a67552197c0afd347af744568d495aba6999ddd9f742d045c3`;
canonical policy-A object SHA-256
**`ea307bf557c503c2617351fb3ae9b5e51e15fd7f4053acc1c46682ecaaa61295`**
(`p3_provenance.canonical_sha256`, the value `load_policy` checks).

| Field | Allowed value |
|---|---|
| Message | MAVLink id 76 `COMMAND_LONG` only (no `COMMAND_INT`, no `COMMAND_CANCEL`) |
| Source | system 255, component 190 (QGC) |
| Target | system 72, component 1 (PX4 autopilot) |
| Startup boundary | `summary.startup.origin_monotonic_ns` + 75 s (original `startup_wall_s`); any allowed command after it is `command_outside_startup_window` |
| 401 `RUN_PREARM_CHECKS` | ≤ 1 frame; params all zero (`00…00`, 28 bytes) |
| 512 `REQUEST_MESSAGE` | ≤ 34 frames total, each payload exactly one frozen pattern within its own cap |

512 payload patterns (param1 = requested message id, param2 = index; params 3–7 zero):

| param1 (message) | param2 | Cap |
|---|---|---|
| 148 `AUTOPILOT_VERSION` | 0 | 1 |
| 280 `GIMBAL_MANAGER_INFORMATION` | 0 | 6 |
| 397 `COMPONENT_METADATA` | 0 | 1 |
| 435 `AVAILABLE_MODES` | 1 … 26 (one pattern each) | 1 each (26) |

Totals: 1 × 401 + 34 × 512 (1 + 6 + 1 + 26). Each payload is matched byte-exact on the
seven little-endian float32 parameters.

Always violations under A: any other command id or payload; exceeding a pattern or command
cap; any frame after the startup boundary; any active-control message from a non-vehicle
source; a vehicle self-command on the GCS link; an incomplete capture; no QGC heartbeat.
PX4 preflight checks, parameters, the P2 mission and all P3 thresholds are unchanged.

## 3. What the operator would be approving

* Command sources documented in the archived package (source commit `4568b585…`):
  401 from `Vehicle::setEventsMetadata`; 512 from the initial-connect state machine
  (148), standard modes (435), component metadata (397) and gimbal discovery (280).
* The patterns and caps were frozen from archived runtime observation
  (`20261024T163425Z-p3-final-3ff425cc`). They have **not** been re-observed for this
  candidate, and they must not be re-derived by running the stack under this offline scope.
  If the rebuilt QGC emits any frame outside the table, Gate A fails; it is never widened
  after the fact.

## 4. Binding requirements for an approval receipt

`p3_mavlink.load_policy` accepts a receipt only if it has
`contract = "p3-native-command-policy-approval-v1"`, `policy = "A"`,
`policy_sha256 = ea307bf5…1295`, `approved = true` and a non-empty `approved_by`.

**Gap (not changed here):** the loader binds the receipt to the policy object only, not
to a candidate. A receipt with the same policy hash from another candidate would load. The
frozen manifest does embed the effective policy and receipt, and each gate authorization
binds the manifest SHA, so the operator approval is candidate-bound only if the operator
approves **together with one specific manifest**. Recommended receipt fields (the loader
ignores the extra fields today; they are for the human record):

```json
{
  "contract": "p3-native-command-policy-approval-v1",
  "policy": "A",
  "policy_sha256": "ea307bf557c503c2617351fb3ae9b5e51e15fd7f4053acc1c46682ecaaa61295",
  "approved": true,
  "approved_by": "<operator>",
  "approved_utc": "<timestamp>",
  "candidate_repository_commit": "<commit containing this policy>",
  "qgc_unattended_elf_sha256": "<ELF_sha256 from state/qgc-unattended-built.json>",
  "qgc_source_commit": "4568b585648e8c6065cca6c650b52731c1d3baa3",
  "scope": "Gate A/B/C of this candidate only; no transfer to other candidates"
}
```

Then `p3_manifest.py freeze … --policy A --policy-approval <receipt>` so the manifest
records it, followed by a separate Gate A authorization for that manifest SHA.
