"""P8 runtime supervisor: one P8-P4R or P8-P5R flight on a P8 layout (contract `p8-navigation-recovery-v1` §2, §5,
§8, §13).

**Reuse (B1).** The accepted P7-E runner is imported from its own file, unchanged, and through it the frozen P7 runner
and the accepted P5, P4 and P3 runners. These stay exactly as accepted:
* the P3 start-up and isolation, the PX4/Gazebo/QGC/DDS launch, the sensor stack, MC trace, shutdown and
  finalization;
* P4's world generation, truth observer, perception and supervisor;
* the accepted P4 or P5 controller command;
* the D15 flight window (`run_controller`).

**P8 overrides only:**
* `p7_init`: the scenario mapping adds the V4 held-out layouts, through the frozen P7 mapping, to the P7 layouts and
  the accepted S1-S6.
* `preconditions`:
  * the P8 manifest, which re-verifies P7-E `e9561f05` and, inside it, P7 and P3-P6;
  * then either a block authorization of scope `p8-validation-60`, its consumption marker and the flight's slot in
    the approved P8 schedule, or a dry-run approval naming this flight.
* `prepare_run`:
  * run naming `…-p8-<variant>-<item>-…` (dry runs `…-p8dry-…`);
  * the accepted run-local configs, plus the approved `p8_recovery.json`;
  * `p7/layout.json`, with the P8 layout and slot records.
* `prepare_world`: the accepted P4 world generation, then the generated world must equal the P8 layout's approved
  world hash (C6).
* `p4_controller_command`:
  * the accepted P4 or P5 command, with only the executable replaced by `p8_p4r_control` or `p8_p5r_control` from
    the P8 install;
  * `--recovery-params` and its approved hash appended.
* `execute`: the P7-E sequence, with a `p8` block in the summary. The `p7` block keeps the keys the frozen classifier
  reads.
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

P8 = Path(__file__).resolve().parents[1]
REPO = P8.parents[1]
P7E = REPO / "simulation/px4_gazebo_p7e"
_spec = importlib.util.spec_from_file_location("p7e_runner_accepted", P7E / "scripts/p7e_runner.py")
p7er = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(p7er)  # the accepted P7-E runner; it loads the frozen P7 runner and the accepted runners
p7r, base, p4, p5r = p7er.p7r, p7er.base, p7er.p4, p7er.p5r
if str(P8 / "scripts") not in sys.path:
    sys.path.insert(0, str(P8 / "scripts"))

import p7_specs  # noqa: E402  (frozen P7 world mapping)
import p8_manifest  # noqa: E402
from p3_provenance import digest, strict_json  # noqa: E402

OWNER = p8_manifest.OWNER
p7er.OWNER = p7r.OWNER = p4.OWNER = p5r.OWNER = OWNER  # the receipt names the owner copy running as PID 1
SCOPES = (p8_manifest.SCOPE, p8_manifest.DRY_RUN)
SUMMARY_CONTRACT = "p8-runtime-summary-v1"
EVIDENCE_LINE = "P8 evidence: "
EXECUTABLES = {"p8_p4r": "p8_p4r_control", "p8_p5r": "p8_p5r_control"}


def layouts():
    return strict_json(P8 / p8_manifest.SPEC_FILES["layouts"])


def schedule():
    return strict_json(P8 / p8_manifest.SPEC_FILES["schedule"])


def layout_records(doc=None):
    doc = doc or layouts()
    return {r["id"]: r for stage in doc["stages"].values() for r in stage["layouts"]}


def p8_scenarios(doc=None):
    """The P7 mapping (P7 layouts and S1-S6, unchanged) plus the V4 held-out layouts through the frozen P7 mapping."""
    mapping = p7r.p7_scenarios()
    held_out = (doc or layouts())["stages"]["V4"]["layouts"]
    extra = p7_specs.scenario_mapping(held_out)["scenarios"]
    clash = set(extra) & set(mapping["scenarios"])
    if clash:
        raise ValueError("P8 held-out ids clash with existing scenarios: " + ", ".join(sorted(clash)))
    mapping["scenarios"].update(extra)
    return mapping


def expected_world(item, doc=None):
    return layout_records(doc)[item]["world_sha256"]


def schedule_slot(block, slot, doc=None):
    found = [r for r in (doc or schedule())["slots"] if r["block"] == int(block) and r["slot"] == int(slot)]
    if len(found) != 1:
        raise PermissionError("No slot %s in P8 block %s of the approved schedule" % (slot, block))
    return found[0]


class P8Flight(p7er.P7EFlight):
    """Mixed in before the accepted P7-E/P7 classes; the frozen D15 controller phase is inherited."""
    variant = None

    def p7_init(self):
        a = self.args
        self.p7_scope, self.block, self.slot, self.item, self.dry = a.p7_scope, a.block, a.slot, a.item, a.p7_scope == p8_manifest.DRY_RUN
        self.scenarios = p8_scenarios()
        if self.item not in layout_records():
            raise ValueError("Not a P8 layout: " + str(self.item))
        self.p7_frozen = None
        self.slot_record = None

    # ---- preconditions ----------------------------------------------------------------------------------------
    def preconditions(self):
        missing = [key for key in base.GATES if os.environ.get(key) != "1"]
        if os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1" or not self.args.allow_simulated_flight:
            raise PermissionError("A P8 flight needs GWM_ALLOW_SITL_COMMANDS=1 and --allow-simulated-flight")
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
            raise ValueError("Exclusive P1-P8 workspace lock is not held by this owner")
        except BlockingIOError:
            pass
        finally:
            os.close(probe)
        if shutil.disk_usage(self.root).free < self.validation["disk_reserve_bytes"]:
            raise ValueError("Disk reserve below the required 20 GiB")
        frozen, sha = p8_manifest.verify_current(self.root, self.args.manifest)  # P7-E, P7 and P3-P6 inside
        if self.dry:
            authorization = p8_manifest.require_dry_run_approval(self.args.authorization, sha, frozen)
            if [self.item, self.variant] not in authorization["items"]:
                raise PermissionError("The dry-run approval does not name %s with %s" % (self.item, self.variant))
            approval = digest(self.args.authorization)
            done = [p for p in (self.root / "runs").glob("*-p8dry-*") if (p / "summary.json").exists()
                    and (strict_json(p / "summary.json").get("p8") or {}).get("dry_run_approval_sha256") == approval]
            if len(done) >= authorization["live_runs_max"]:
                raise PermissionError("Dry-run cap reached (%d/%d)" % (len(done), authorization["live_runs_max"]))
            self.marker = None
        else:
            if frozen["development"]:
                raise PermissionError("A development candidate never flies a P8 block")
            authorization = p8_manifest.require_authorization(self.args.authorization, self.block, sha)
            marker = p8_manifest.marker_path(self.root, self.block, sha)
            if not marker.exists() or strict_json(marker)["authorization_sha256"] != digest(self.args.authorization):
                raise PermissionError("Block authorization was not consumed by the P8 block sequencer")
            self.slot_record = schedule_slot(self.block, self.slot)
            if self.slot_record["layout"] != self.item or self.variant not in self.slot_record["order"]:
                raise PermissionError("Slot %s of P8 block %s is not %s with %s" % (self.slot, self.block, self.item, self.variant))
            self.marker = marker
        if not base.p3_manifest.wsl_host():
            raise PermissionError("P8 runs only on the WSL D3D12 host")
        self.p7_frozen = frozen
        self.p8_sha = sha
        return p8_manifest.runner_view(frozen, self.controller), sha, authorization

    # ---- run directory and run-local inputs -----------------------------------------------------------------
    def prepare_run(self, frozen):
        label = ("-p8dry-" if self.dry else "-p8-") + self.variant + "-" + self.item
        self.run = self.root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + label + "-" + uuid.uuid4().hex[:8])
        self.run.mkdir(parents=True)
        for name in ("tmp", "p4", "p7", "p8") + (("p5",) if self.controller == "p5" else ()):
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
                shutil.copyfile(p5r.P5 / "configs" / name, self.run / name)
            (self.run / "p5_policy.yaml").write_text(json.dumps(p5r.run_local_policy_config(frozen), indent=2) + "\n")
        shutil.copyfile(P8 / p8_manifest.SPEC_FILES["recovery"], self.run / "p8_recovery.json")
        record = {"contract": "p8-flight-layout-v1", "item": self.item, "variant": self.variant, "controller": self.controller,
                  "scope": self.p7_scope, "block": self.block, "slot": self.slot, "slot_record": self.slot_record,
                  "scenario": self.scenarios["scenarios"][self.item], "expected_world_sha256": expected_world(self.item),
                  "layout": layout_records().get(self.item), "specs": self.p7_frozen["specs"]}
        (self.run / "p7/layout.json").write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
        if self.dry:
            self.marker = self.run / "p8/dry-run-marker.json"
            self.marker.write_text(json.dumps({"contract": "p8-dry-run-marker-v1", "approval_sha256": digest(self.args.authorization),
                                               "item": self.item, "variant": self.variant, "credit": False}) + "\n")

    # ---- world (C6) ---------------------------------------------------------------------------------------------
    def prepare_world(self):
        p4.P4Runner.prepare_world(self)  # the accepted generation; the P7 check is replaced by the P8 one below
        got, want = self.summary.get("generated_world_sha256"), expected_world(self.item)
        self.summary["p7"]["world"] = {"generated_sha256": got, "expected_sha256": want, "identical": got == want}
        if got != want:
            raise base.Abort("p8_world_differs_from_the_approved_world")

    # ---- the controller command -------------------------------------------------------------------------------
    def p4_controller_command(self, env):
        command, env = super().p4_controller_command(env)  # the accepted P4 or P5 command, unchanged
        receipt = strict_json(self.root / p8_manifest.BUILD_RECEIPT)
        lib = Path(receipt["install"]) / "gwm_navigation_recovery/lib/gwm_navigation_recovery"
        command = [str(lib / EXECUTABLES[self.variant])] + command[1:] + [
            "--recovery-params", str(self.run / "p8_recovery.json"),
            "--recovery-params-sha256", p8_manifest.APPROVED_SPECS["recovery"]]
        return command, env

    # ---- entry ----------------------------------------------------------------------------------------------
    def execute(self):
        self.world_dir = self.declaration = None
        frozen, sha, authorization = self.preconditions()
        self.frozen = frozen
        self.prepare_run(frozen)
        print(EVIDENCE_LINE + str(self.run), flush=True)
        approval = digest(self.args.authorization) if self.dry else None
        self.summary = {"schema_version": 1, "contract": SUMMARY_CONTRACT, "run_id": self.run.name,
                        "p3_kind": self.args.scope, "p4_scope": self.p4_scope, "gate": self.args.gate, "role": self.args.role,
                        "frozen_manifest_sha256": sha, "authorization_sha256": digest(self.args.authorization),
                        "started_unix_ns": time.time_ns(), "status": "incomplete", "failure": None, "identity": {},
                        "processes": [], "startup": {"milestones": [], "origin_monotonic_ns": None},
                        "native_command_policy": frozen["runtime"]["native_command_policy"],
                        "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=P8, text=True).strip(),
                        "isolation": "private user/mount/PID/network namespace, loopback only, exclusive workspace lock",
                        "flight_commands_possible": True, "finalized": False, "diagnostic_variant": None,
                        "acceptance_credit_possible": False, "p4_enabled_run_local": True,
                        **self.controller_summary(frozen),
                        "p7": {"contract": "p7-runtime-summary-v1", "controller": self.controller, "item": self.item,
                               "scope": self.p7_scope, "block": self.block, "slot": self.slot,
                               "p7_manifest_sha256": p8_manifest.p7e_manifest.P7_SHA256,
                               "baselines": self.p7_frozen["baselines"], "development": self.p7_frozen["development"],
                               "dry_run": self.dry, "dry_run_approval_sha256": approval,
                               "layout_record_sha256": digest(self.run / "p7/layout.json"), "world": None},
                        "p8": {"contract": SUMMARY_CONTRACT, "p8_manifest_sha256": sha, "variant": self.variant,
                               "p7e_manifest_sha256": p8_manifest.P7E_SHA256, "scope": self.p7_scope, "block": self.block,
                               "stage": p8_manifest.STAGES.get(self.block), "slot": self.slot,
                               "pair_in_block": (self.slot_record or {}).get("pair_in_block"),
                               "order": (self.slot_record or {}).get("order"),
                               "schedule_canonical_sha256": self.p7_frozen["specs"]["schedule"],
                               "recovery_params_canonical_sha256": self.p7_frozen["specs"]["recovery"],
                               "development": self.p7_frozen["development"], "dry_run": self.dry,
                               "dry_run_approval_sha256": approval, "engineering_credit_possible": not self.dry}}
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
            print("P8 stopped: " + self.summary["failure"], flush=True)
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


class P8P4RRunner(P8Flight, p7r.P7P4Runner):
    controller, variant = "p4", "p8_p4r"


class P8P5RRunner(P8Flight, p7r.P7P5Runner):
    controller, variant = "p5", "p8_p5r"


RUNNERS = {"p8_p4r": P8P4RRunner, "p8_p5r": P8P5RRunner}


def parse(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true", required=True)
    parser.add_argument("--variant", choices=sorted(RUNNERS), required=True)
    parser.add_argument("--item", required=True, help="a P8 layout id")
    parser.add_argument("--scope", choices=SCOPES, required=True)
    parser.add_argument("--block", type=int, default=0)
    parser.add_argument("--slot", type=int, default=0)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    a = parser.parse_args(argv)
    if a.scope == p8_manifest.DRY_RUN and (a.block or a.slot):
        parser.error("a dry run has no block or slot")
    if a.scope != p8_manifest.DRY_RUN and (a.block < 1 or a.slot < 1):
        parser.error("a P8 block flight needs its block and slot")
    role = "%s-%s-b%02d-s%03d-%s" % ("p8", a.scope, a.block, a.slot, a.variant)
    # `variant` stays None: the accepted P3 runner reads args.variant as its diagnostic variant. The P8 variant
    # travels as `p8_variant` (and as the runner class's own `variant`).
    return argparse.Namespace(run=True, controller=p8_manifest.VARIANTS[a.variant], p8_variant=a.variant, item=a.item,
                              p7_scope=a.scope, block=a.block, slot=a.slot, gate="P8", role=role, manifest=a.manifest,
                              authorization=a.authorization, allow_simulated_flight=a.allow_simulated_flight,
                              scope="flight", fixture=None, scenario=a.item, variant=None)


def main(argv=None):
    args = parse(argv)
    return RUNNERS[args.p8_variant](args).execute()


if __name__ == "__main__":
    raise SystemExit(main())
