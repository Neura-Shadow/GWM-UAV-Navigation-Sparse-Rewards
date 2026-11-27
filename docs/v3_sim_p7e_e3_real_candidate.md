# v3-SIM P7-E E3 — the real P7-E candidate: production Policy A and the real freeze

**Status: DONE (2026-11-22). The real P7-E candidate is frozen:
`38e23bad4bd4f425c1fc47f7d6d50af42ea81a6f5812b56a8c2f709ec7e844be`.** Nothing had been flown on it at E3.
* **Later (release note, 2026-11-28):**
  * its block 1 stopped after 3 flights (an external termination, recovered into a recorded stop; see
    [E4](v3_sim_p7e_e4_blocks.md));
  * the candidate is retired, and its records are protected evidence;
  * the batch was flown on the second real candidate `e9561f05…`, frozen on 2026-11-27 with the same code.
* **Authorized:** preparation ("Prepare them"), then two separate answers:
  * item 1, production Policy A: "Approve (Recommended)";
  * item 2, the real freeze: "Freeze it (Recommended)".
* **Records:**

  | Item | File | SHA-256 |
  |---|---|---|
  | Production Policy A | `state/p7e-policy-A-approval-2b6ee5d.json` (`development_only: false`) | `26595bba…` |
  | Real candidate | `state/p7e-candidate-2b6ee5d.json`, frozen at git HEAD `d969f48`; it verified from `/tmp` | file `a44ef63a…`; manifest `38e23bad…` |

* **What the candidate is:** development false, engineering credit possible, scope `engineering-30` with 3 blocks.
  It binds P7 `0799c37e…`, the 16 files and the four specs.
* **From now on, no bound file may change until E5.**
* **Under:** contract [`p7-engineering-validation-v1`](v3_sim_p7e_contract_draft.md) §13 items 6 and 7.
* **What it leads to.** Once frozen, the real candidate is the only one that can fly the 3 blocks of scope
  `engineering-30`. Each block still needs its own authorization (§13 item 8). Freezing flies nothing.

## 1. The two approvals, each separate

**1. Production Policy A for the real P7-E candidate**
* **File:** `state/p7e-policy-A-approval-2b6ee5d.json` (contract `p3-native-command-policy-approval-v1`).
* **Policy:** Policy A `bf1ef9b1…`, the same native-command content as P3–P7, with QGC `4568b585…` and ELF
  `82994283…`.
* **Approval fields:** `development_only: false`; `candidate_phase` "P7-E p7-engineering-validation-v1, the real
  (non-development) candidate".
* **What it binds:** the 16 accepted P7-E source and test files of commit `2b6ee5d` (canonical digest `003b4d76…`).
* **Scope:** the 3 blocks of `engineering-30`, each separately authorized, in simulated flight only (SITL, never real
  hardware). It transfers to no other candidate, and to no development candidate.
* **Why a new approval.** The E8 approval is development-only, and the P7 approval names P7. The code refuses both
  for a real P7-E candidate, as the pre-check showed.

**2. The real freeze**
* **File:** `state/p7e-candidate-2b6ee5d.json`, through `p7e_manifest.py freeze` (without `--development`), with the
  absolute approval path. It is verified from `/tmp` right after.
* **What it binds:** the same inputs as the E8 candidate, with the production approval in place of the development
  one:
  * P7 `0799c37e…`, and P3 `1e205fec`, P4 `c110bd30`, P5 `2becfbbb` and P6 `22292775` inside it;
  * the specs `9b3deacb…`, `15e7f7b4…`, `d742b7af…` and `9f78af75…`;
  * the 16 files and the owner copy `0d1c2d27…`.
* **The rule after the freeze:** no bound file may change until E5. A change would stop every block as manifest
  drift, and would mean a new candidate.

## 2. Read-only pre-check (2026-11-22)

* **Same code as accepted.** The P7-E, P7 and test trees are byte-identical to the accepted E2 commit `2b6ee5d`. Every
  later commit touched docs only, and the working tree is clean.
* **The real candidate computes.** It was computed against a draft approval in `/tmp`, removed afterwards:
  * development false, engineering credit possible;
  * files, specs, P7 binding and baselines identical to the E8 candidate's;
  * in the runtime identity, only the Policy A approval differs.
* **The guards hold:**
  * the E8 development approval is refused for a real candidate;
  * the P7 production approval is refused, since it does not name P7-E.
* **No live lock** is held, and no real P7-E candidate exists yet.
* **The host check passes:**
  * 774 GB free, so about 696 GB would remain after 60 flights (the floor is 150 GB);
  * host CPU samples were 4, 0, 32, 7 and 20 %, a mean of 12.6 % against the 15 % limit.

**One caution for the blocks.** The host CPU mean of 12.6 % is close to the limit. The check runs again before each
block. If it fails, the block is not flown and its authorization is not consumed. Closing other Windows applications
before a block avoids that.

## 3. After the freeze

**Next, each needing its own explicit authorization:**
* **Block 1, then block 2, then block 3.** Each is 10 pairs and 20 flights, about 65 min.
* **How a block is authorized:** through `p7e_manifest.py authorize --block <n>`, with the confirmation phrase
  "authorize P7-E engineering-30 block <n> <sha>".
* **The order rule (FC-1):** a block runs only after the previous block's result is unstopped and accounts for every
  planned flight.
* **Then E5:** the engineering report.
