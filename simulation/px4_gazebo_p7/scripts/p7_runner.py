"""P7 runtime supervisor: one accepted P4 or P5 flight on a frozen P7 layout (contract §2 C3, C5, C6, §8; D11).

**Reuse.** The accepted `P5Runner` is imported from its own file, and through it the accepted `P4Runner` and P3
`Runner`. Two thin subclasses fly them unchanged: `P7P4Runner(P4Runner)` and `P7P5Runner(P5Runner)`. The P3 runner's
start-up, isolation (private namespace, loopback only, exclusive workspace lock), PX4/Gazebo/QGC/DDS launch, sensor
stack, MC trace, ordered shutdown and finalization are untouched. So are P4's world generation, truth observer,
perception and supervisor, and P5's `p5_control` with its manifest-bound checkpoint. P7 adds no safety logic,
fallback or control path (C3).

**P7 overrides only:**
* `preconditions`: the P7 manifest, which re-verifies P3-P6 (C1), then one of:
  * a block authorization, its consumption marker, and the flight's slot in the frozen schedule;
  * for a development candidate, a dry-run approval that names this flight.
* `prepare_run`: run naming (`…-p7-<controller>-<item>-…`, dry runs `…-p7dry-…`), the accepted run-local configs,
  and `p7/layout.json`.
* `scenarios`: the P7 layouts through `p7_specs.scenario_mapping`, which carries the D13 declaration label
  `p7_not_predeclared`, plus the accepted S1-S6 unchanged, for the M4 parity dry run.
* `prepare_world`: the accepted world generation, then a check that the world equals the frozen one (the layout's
  recorded SHA-256, or the accepted S1-S6 hash, C6). A mismatch stops the flight before PX4 starts.
* `execute`: the accepted sequence, with the summary fields the unchanged P4/P5 evaluators read, a `p7` block, and
  `acceptance_credit_possible: false`. P7 flights never count toward a P4 or P5 gate.

The QGC ownership receipt names the P7 owner copy that actually runs as PID 1 (the same bytes as the P3 owner).
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid

P7 = Path(__file__).resolve().parents[1]
REPO = P7.parents[1]
P5 = REPO / "simulation/px4_gazebo_p5"
_spec = importlib.util.spec_from_file_location("p5_runner_accepted", P5 / "scripts/p5_runner.py")
p5r = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(p5r)
base, p4 = p5r.base, p5r.p4
if str(P7 / "scripts") not in sys.path:
    sys.path.insert(0, str(P7 / "scripts"))

import p7_manifest  # noqa: E402
import p7_specs  # noqa: E402
from p3_provenance import digest, strict_json  # noqa: E402

OWNER = P7 / "scripts/p7_namespace_owner.py"
p4.OWNER = p5r.OWNER = OWNER  # the receipt names the owner copy running as PID 1 (byte-identical to the P3 owner)
SCOPES = p7_manifest.SCOPES + (p7_manifest.DRY_RUN,)
# C6: the accepted S1-S6 worlds, as every accepted P4 and P5 flight recorded them (M0 audit §2; tested offline).
ACCEPTED_WORLDS = {"S1": "c3e5ff97710ae7b32e704b6bfa92e8d56a80dd5501535bdef75c481783049275",
                   "S2": "65dc596c20ebb247830fa3dc1e30149dca4054b15c75384e270373c7f75393db",
                   "S3": "540f3da6405f62984c30675653696318d37c69f8e5319fc524e643512e7ab801",
                   "S4": "94daf31eb7f77b54b8b769f4460819451b52ea28d74bdb54a60bfc6c2b0593da",
                   "S5": "effd475bf127516708c3b5a345db2b78f2647986d78a9e259e42804de76d6b39",
                   "S6": "65dc596c20ebb247830fa3dc1e30149dca4054b15c75384e270373c7f75393db"}


# ---- the frozen inputs --------------------------------------------------------------------------------------
def layouts():
    return strict_json(P7 / p7_manifest.SPEC_FILES["layouts"])


def schedule():
    return strict_json(P7 / p7_manifest.SPEC_FILES["schedule"])


def layout_records(doc=None):
    doc = doc or layouts()
    return {r["id"]: r for s in doc["sets"].values() for r in s["layouts"]}


def p7_scenarios(doc=None):
    """The `p4_scenarios.json`-shaped mapping: every P7 layout (D13 label) plus the accepted S1-S6, unchanged."""
    records = layout_records(doc)
    mapping = p7_specs.scenario_mapping(list(records.values()))
    accepted = strict_json(p5r.P4 / "configs/p4_scenarios.json")
    clash = set(mapping["scenarios"]) & set(accepted["scenarios"])
    if clash:
        raise ValueError("P7 layout ids clash with accepted scenarios: " + ", ".join(sorted(clash)))
    mapping["scenarios"].update(accepted["scenarios"])
    return mapping


def expected_world(item, doc=None):
    if item in ACCEPTED_WORLDS:
        return ACCEPTED_WORLDS[item]
    return layout_records(doc)[item]["world_sha256"]


def schedule_slot(scope, block, slot, doc=None):
    """The frozen slot of a pilot or confirmatory block, or a refusal."""
    doc = doc or schedule()
    rows = doc["pilot"]["slots"] if scope == "pilot" else doc["confirmatory"][scope.split("-")[1]]["slots"]
    found = [r for r in rows if r["block"] == int(block) and r["slot"] == int(slot)]
    if len(found) != 1:
        raise PermissionError("No slot %s in %s block %s of the frozen schedule" % (slot, scope, block))
    return found[0]


# ---- the P7 behaviour shared by both controllers ------------------------------------------------------------
class P7Flight:
    controller = None  # "p4" or "p5"

    def p7_init(self):
        a = self.args
        self.p7_scope, self.block, self.slot, self.item, self.dry = a.p7_scope, a.block, a.slot, a.item, a.p7_scope == p7_manifest.DRY_RUN
        self.scenarios = p7_scenarios()
        if self.item not in self.scenarios["scenarios"]:
            raise ValueError("Unknown P7 flight item: " + str(self.item))
        self.p7_frozen = None
        self.slot_record = None

    # ---- preconditions ----------------------------------------------------------------------
    def preconditions(self):
        missing = [key for key in base.GATES if os.environ.get(key) != "1"]
        if os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1" or not self.args.allow_simulated_flight:
            raise PermissionError("A P7 flight needs GWM_ALLOW_SITL_COMMANDS=1 and --allow-simulated-flight")
        if missing:
            raise ValueError("Missing explicit gates: " + ", ".join(missing))
        if os.getpid() != 1:
            raise ValueError("Owned private PID/network namespace required")
        interfaces = json.loads(subprocess.check_output(["ip", "-j", "link"], text=True))
        if {item["ifname"] for item in interfaces} != {"lo"}:
            raise ValueError("Owned private PID/network namespace required")
        lock = Path("/proc/1/fd/9").resolve()
        probe = os.open(lock, os.O_RDWR)
        try:
            base.fcntl.flock(probe, base.fcntl.LOCK_EX | base.fcntl.LOCK_NB)
            raise ValueError("Exclusive P1-P7 workspace lock is not held by this owner")
        except BlockingIOError:
            pass
        finally:
            os.close(probe)
        if shutil.disk_usage(self.root).free < self.validation["disk_reserve_bytes"]:
            raise ValueError("Disk reserve below the required 20 GiB")
        frozen, sha = p7_manifest.verify_current(self.root, self.args.manifest)  # P3-P6 re-verified (C1)
        if self.dry:
            authorization = p7_manifest.require_dry_run_approval(self.args.authorization, sha, frozen)
            if [self.item, self.controller] not in authorization["items"]:
                raise PermissionError("The dry-run approval does not name %s with %s" % (self.item, self.controller))
            approval = digest(self.args.authorization)
            done = [p for p in (self.root / "runs").glob("*-p7dry-*") if (p / "summary.json").exists()
                    and (strict_json(p / "summary.json").get("p7") or {}).get("dry_run_approval_sha256") == approval]
            if len(done) >= authorization["live_runs_max"]:
                raise PermissionError("Dry-run cap reached (%d/%d)" % (len(done), authorization["live_runs_max"]))
            self.marker = None  # a run-local marker is written in prepare_run (the P4/P5 enable checks need a file)
        else:
            if frozen["development"]:
                raise PermissionError("A development candidate never flies a pilot or confirmatory block")
            if self.p7_scope != "pilot":
                p7_manifest.require_selection(self.root, sha, self.p7_scope)
            authorization = p7_manifest.require_authorization(self.args.authorization, self.p7_scope, self.block, sha)
            marker = p7_manifest.marker_path(self.root, self.p7_scope, self.block, sha)
            if not marker.exists() or strict_json(marker)["authorization_sha256"] != digest(self.args.authorization):
                raise PermissionError("Block authorization was not consumed by the P7 block sequencer")
            self.slot_record = schedule_slot(self.p7_scope, self.block, self.slot)
            if self.slot_record["layout"] != self.item or self.controller not in self.slot_record["order"]:
                raise PermissionError("Slot %s of %s block %s is not %s with %s" % (self.slot, self.p7_scope, self.block,
                                                                                   self.item, self.controller))
            self.marker = marker
        if not base.p3_manifest.wsl_host():
            raise PermissionError("P7 runs only on the WSL D3D12 host")
        self.p7_frozen = frozen
        return p7_manifest.runner_view(frozen, self.controller), sha, authorization

    # ---- run directory and run-local inputs -------------------------------------------------------------------
    def prepare_run(self, frozen):
        label = ("-p7dry-" if self.dry else "-p7-") + self.controller + "-" + self.item
        self.run = self.root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + label + "-" + uuid.uuid4().hex[:8])
        self.run.mkdir(parents=True)
        for name in ("tmp", "p4", "p7") + (("p5",) if self.controller == "p5" else ()):
            (self.run / name).mkdir()
        for name in ("p3_sensors.yaml", "p3_validation.yaml", "p3_sensor_bridge.yaml", "p3_fastdds.xml",
                     "p3_native_command_policy.json"):
            shutil.copyfile(p5r.SIM / "configs" / name, self.run / name)
        for name in ("p4_validation.yaml", "p4_scenarios.json"):
            shutil.copyfile(p5r.P4 / "configs" / name, self.run / name)
        local = strict_json(p5r.P4 / "configs/p4_avoidance.yaml")
        local["enabled"] = True  # P4 contract C4, as the accepted runners
        (self.run / "p4_avoidance.yaml").write_text(json.dumps(local, indent=2) + "\n")
        if self.controller == "p5":
            for name in ("p5_validation.yaml", "p5_model_schema.json"):
                shutil.copyfile(P5 / "configs" / name, self.run / name)
            (self.run / "p5_policy.yaml").write_text(json.dumps(p5r.run_local_policy_config(frozen), indent=2) + "\n")
        record = {"contract": "p7-flight-layout-v1", "item": self.item, "controller": self.controller,
                  "scope": self.p7_scope, "block": self.block, "slot": self.slot, "slot_record": self.slot_record,
                  "scenario": self.scenarios["scenarios"][self.item], "expected_world_sha256": expected_world(self.item),
                  "layout": layout_records().get(self.item), "specs": self.p7_frozen["specs"]}
        (self.run / "p7/layout.json").write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
        if self.dry:
            self.marker = self.run / "p7/dry-run-marker.json"
            self.marker.write_text(json.dumps({"contract": "p7-dry-run-marker-v1", "approval_sha256": digest(self.args.authorization),
                                               "item": self.item, "controller": self.controller, "credit": False}) + "\n")

    # ---- world (C6) ---------------------------------------------------------------------------------------------
    def prepare_world(self):
        super().prepare_world()
        got, want = self.summary.get("generated_world_sha256"), expected_world(self.item)
        self.summary["p7"]["world"] = {"generated_sha256": got, "expected_sha256": want, "identical": got == want}
        if got != want:
            raise base.Abort("p7_world_differs_from_the_frozen_world")

    # ---- entry ----------------------------------------------------------------------------------
    def controller_summary(self, frozen):
        raise NotImplementedError

    def execute(self):
        self.world_dir = self.declaration = None
        frozen, sha, authorization = self.preconditions()
        self.frozen = frozen
        self.prepare_run(frozen)
        print("P7 evidence: " + str(self.run), flush=True)
        self.summary = {"schema_version": 1, "contract": "p7-runtime-summary-v1", "run_id": self.run.name,
                        "p3_kind": self.args.scope, "p4_scope": self.p4_scope, "gate": self.args.gate, "role": self.args.role,
                        "frozen_manifest_sha256": sha, "authorization_sha256": digest(self.args.authorization),
                        "started_unix_ns": time.time_ns(), "status": "incomplete", "failure": None, "identity": {},
                        "processes": [], "startup": {"milestones": [], "origin_monotonic_ns": None},
                        "native_command_policy": frozen["runtime"]["native_command_policy"],
                        "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=P7, text=True).strip(),
                        "isolation": "private user/mount/PID/network namespace, loopback only, exclusive workspace lock",
                        "flight_commands_possible": True, "finalized": False, "diagnostic_variant": None,
                        "acceptance_credit_possible": False, "p4_enabled_run_local": True,
                        **self.controller_summary(frozen),
                        "p7": {"contract": "p7-runtime-summary-v1", "controller": self.controller, "item": self.item,
                               "scope": self.p7_scope, "block": self.block, "slot": self.slot,
                               "p7_manifest_sha256": sha, "baselines": self.p7_frozen["baselines"],
                               "development": self.p7_frozen["development"], "dry_run": self.dry,
                               "dry_run_approval_sha256": digest(self.args.authorization) if self.dry else None,
                               "layout_record_sha256": digest(self.run / "p7/layout.json"), "world": None}}
        self.sampler = base.ResourceSampler(self.run, self.owned, self.root)
        self.sampler.start()
        try:
            self.save()
            self.prepare_world()
            config = self.px4_stack(frozen)
            self.run_controller(config)
        except (Exception, KeyboardInterrupt) as exc:
            self.summary["status"] = "failed"
            self.summary["failure"] = type(exc).__name__ + ":" + str(exc)
            print("P7 stopped: " + self.summary["failure"], flush=True)
        finally:
            try:
                errors = self.shutdown()
            except Exception as exc:
                errors = ["shutdown_exception:" + type(exc).__name__ + ":" + str(exc)]
            self.summary["shutdown_errors"] = errors
            if errors and self.summary["status"] == "passed":
                self.summary["status"], self.summary["failure"] = "failed", "shutdown:" + errors[0]
            self.finalize()
        print(json.dumps({"run_id": self.run.name, "status": self.summary["status"],
                          "failure": self.summary["failure"]}), flush=True)
        return 0 if self.summary["status"] == "passed" else 1


class P7P4Runner(P7Flight, p4.P4Runner):
    controller = "p4"

    def __init__(self, args):
        super().__init__(args)  # P4 -> P3 scope mapping, unchanged ("flight")
        self.p7_init()

    def controller_summary(self, frozen):
        """What the accepted P4 evaluator reads: the P3 baseline as the accepted P4 manifest binds it."""
        return {"p3_baseline": frozen["p3_baseline"], "p4_owner_script": str(OWNER)}


class P7P5Runner(P7Flight, p5r.P5Runner):
    controller = "p5"

    def __init__(self, args):
        super().__init__(args)  # P5 -> P4 -> P3 scope mapping, unchanged ("flight")
        self.p7_init()

    def controller_summary(self, frozen):
        """What the accepted P5 evaluator reads, exactly as the accepted P5 runner writes it."""
        v = self.p5_validation
        return {"p5_scope": self.p5_scope,
                "p3_baseline": {"manifest": v["p3_baseline_manifest"], "sha256": frozen["baselines"]["p3_sha256"]},
                "p4_baseline": {"manifest": v["p4_baseline_manifest"], "sha256": frozen["baselines"]["p4_sha256"]},
                "p5_model": frozen["model"], "schema_sha256": frozen["schema_sha256"],
                "controller_runtime": dict(frozen["controller_runtime"], openblas_num_threads="1"),
                "policy_fault": None, "p5_enabled_run_local": True, "p5_owner_script": str(OWNER)}


RUNNERS = {"p4": P7P4Runner, "p5": P7P5Runner}


def parse(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true", required=True)
    parser.add_argument("--controller", choices=sorted(RUNNERS), required=True)
    parser.add_argument("--item", required=True, help="a frozen P7 layout id, or S1-S6 in a dry run")
    parser.add_argument("--scope", choices=SCOPES, required=True)
    parser.add_argument("--block", type=int, default=0)
    parser.add_argument("--slot", type=int, default=0)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    a = parser.parse_args(argv)
    if a.scope == p7_manifest.DRY_RUN and (a.block or a.slot):
        parser.error("a dry run has no block or slot")
    if a.scope != p7_manifest.DRY_RUN and (a.block < 1 or a.slot < 1 or a.item in ACCEPTED_WORLDS):
        parser.error("a pilot or confirmatory flight needs its block and slot, and flies a P7 layout")
    role = "%s-%s-b%02d-s%03d-%s" % ("p7", a.scope, a.block, a.slot, a.controller)
    return argparse.Namespace(run=True, controller=a.controller, item=a.item, p7_scope=a.scope, block=a.block, slot=a.slot,
                              gate="P7", role=role, manifest=a.manifest, authorization=a.authorization,
                              allow_simulated_flight=a.allow_simulated_flight, scope="flight", fixture=None,
                              scenario=a.item, variant=None)


def main(argv=None):
    args = parse(argv)
    return RUNNERS[args.controller](args).execute()


if __name__ == "__main__":
    raise SystemExit(main())
