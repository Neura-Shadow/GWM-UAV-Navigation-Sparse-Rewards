"""P7-E runtime supervisor: one accepted P4 or P5 flight on a P7-E layout (contract `p7-engineering-validation-v1`
E-C1, E-C2, §8, §13).

**Reuse (E-C1).** The frozen P7 runner of candidate 0799c37e is imported from its own file, unchanged. Through it,
the accepted P5, P4 and P3 runners are imported too. These stay exactly as frozen:
* the P3 start-up and isolation, the PX4/Gazebo/QGC/DDS launch, the sensor stack, MC trace, shutdown and
  finalization;
* P4's world generation, truth observer, perception and supervisor;
* P5's `p5_control`;
* the P7 world check (C6, `prepare_world`) and the D15 flight window (`run_controller`).

P7-E adds no safety logic, fallback or control path.

**P7-E overrides only:**
* `preconditions`: the P7-E manifest, which re-verifies the frozen P7 candidate and P3-P6 inside it. Then one of:
  * a block authorization of scope `engineering-30`, its consumption marker, and the flight's slot in the approved
    P7-E schedule;
  * for a development candidate, a dry-run approval that names this flight.
* `prepare_run`: run naming `…-p7e-<controller>-<item>-…` (dry runs `…-p7edry-…`), the accepted run-local configs,
  and `p7/layout.json` (the path the frozen P7 classifier hashes), with the P7-E slot record.
* `execute`: the frozen P7 sequence, with a `p7e` block in the summary. The `p7` block keeps the keys the frozen
  classifier reads. A P7-E flight never counts toward a P4, P5 or P7-A result.

The QGC ownership receipt names the P7-E owner copy that runs as PID 1. It is byte-identical to the P3 owner.
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

P7E = Path(__file__).resolve().parents[1]
REPO = P7E.parents[1]
P7 = REPO / "simulation/px4_gazebo_p7"
_spec = importlib.util.spec_from_file_location("p7_runner_frozen", P7 / "scripts/p7_runner.py")
p7r = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(p7r)  # the frozen P7 runner; it loads the accepted P5, P4 and P3 runners by path
base, p4, p5r = p7r.base, p7r.p4, p7r.p5r
if str(P7E / "scripts") not in sys.path:
    sys.path.insert(0, str(P7E / "scripts"))

import p7e_manifest  # noqa: E402
from p3_provenance import digest, strict_json  # noqa: E402

OWNER = p7e_manifest.OWNER
p7r.OWNER = p4.OWNER = p5r.OWNER = OWNER  # the receipt names the owner copy running as PID 1
SCOPES = (p7e_manifest.SCOPE, p7e_manifest.DRY_RUN)
SUMMARY_CONTRACT = "p7e-runtime-summary-v1"
EVIDENCE_LINE = "P7-E evidence: "


def schedule():
    return strict_json(P7E / p7e_manifest.SPEC_FILES["schedule"])


def schedule_slot(block, slot, doc=None):
    """The approved slot of one P7-E block, or a refusal."""
    found = [r for r in (doc or schedule())["slots"] if r["block"] == int(block) and r["slot"] == int(slot)]
    if len(found) != 1:
        raise PermissionError("No slot %s in P7-E block %s of the approved schedule" % (slot, block))
    return found[0]


class P7EFlight:
    """Mixed in before the frozen P7 classes: P7Flight's world check and D15 controller phase are inherited."""

    # ---- preconditions ----------------------------------------------------------------------------------------
    def preconditions(self):
        missing = [key for key in base.GATES if os.environ.get(key) != "1"]
        if os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1" or not self.args.allow_simulated_flight:
            raise PermissionError("A P7-E flight needs GWM_ALLOW_SITL_COMMANDS=1 and --allow-simulated-flight")
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
        frozen, sha = p7e_manifest.verify_current(self.root, self.args.manifest)  # P7 0799c37e and P3-P6 inside
        if self.dry:
            authorization = p7e_manifest.require_dry_run_approval(self.args.authorization, sha, frozen)
            if [self.item, self.controller] not in authorization["items"]:
                raise PermissionError("The dry-run approval does not name %s with %s" % (self.item, self.controller))
            approval = digest(self.args.authorization)
            done = [p for p in (self.root / "runs").glob("*-p7edry-*") if (p / "summary.json").exists()
                    and (strict_json(p / "summary.json").get("p7e") or {}).get("dry_run_approval_sha256") == approval]
            if len(done) >= authorization["live_runs_max"]:
                raise PermissionError("Dry-run cap reached (%d/%d)" % (len(done), authorization["live_runs_max"]))
            self.marker = None  # a run-local marker is written in prepare_run (the P4/P5 enable checks need a file)
        else:
            if frozen["development"]:
                raise PermissionError("A development candidate never flies a P7-E block")
            authorization = p7e_manifest.require_authorization(self.args.authorization, self.block, sha)
            marker = p7e_manifest.marker_path(self.root, self.block, sha)
            if not marker.exists() or strict_json(marker)["authorization_sha256"] != digest(self.args.authorization):
                raise PermissionError("Block authorization was not consumed by the P7-E block sequencer")
            self.slot_record = schedule_slot(self.block, self.slot)
            if self.slot_record["layout"] != self.item or self.controller not in self.slot_record["order"]:
                raise PermissionError("Slot %s of P7-E block %s is not %s with %s" % (self.slot, self.block, self.item,
                                                                                    self.controller))
            self.marker = marker
        if not base.p3_manifest.wsl_host():
            raise PermissionError("P7-E runs only on the WSL D3D12 host")
        self.p7_frozen = frozen
        self.p7e_sha = sha
        return p7e_manifest.runner_view(frozen, self.controller), sha, authorization

    # ---- run directory and run-local inputs -----------------------------------------------------------------
    def prepare_run(self, frozen):
        label = ("-p7edry-" if self.dry else "-p7e-") + self.controller + "-" + self.item
        self.run = self.root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + label + "-" + uuid.uuid4().hex[:8])
        self.run.mkdir(parents=True)
        for name in ("tmp", "p4", "p7", "p7e") + (("p5",) if self.controller == "p5" else ()):
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
        record = {"contract": "p7e-flight-layout-v1", "item": self.item, "controller": self.controller,
                  "scope": self.p7_scope, "block": self.block, "slot": self.slot, "slot_record": self.slot_record,
                  "scenario": self.scenarios["scenarios"][self.item], "expected_world_sha256": p7r.expected_world(self.item),
                  "layout": p7r.layout_records().get(self.item), "specs": self.p7_frozen["specs"]}
        (self.run / "p7/layout.json").write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
        if self.dry:
            self.marker = self.run / "p7e/dry-run-marker.json"
            self.marker.write_text(json.dumps({"contract": "p7e-dry-run-marker-v1", "approval_sha256": digest(self.args.authorization),
                                               "item": self.item, "controller": self.controller, "credit": False}) + "\n")

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
                        "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=P7E, text=True).strip(),
                        "isolation": "private user/mount/PID/network namespace, loopback only, exclusive workspace lock",
                        "flight_commands_possible": True, "finalized": False, "diagnostic_variant": None,
                        "acceptance_credit_possible": False, "p4_enabled_run_local": True,
                        **self.controller_summary(frozen),
                        # the keys the frozen P7 classifier reads; `world` is filled by the frozen prepare_world
                        "p7": {"contract": "p7-runtime-summary-v1", "controller": self.controller, "item": self.item,
                               "scope": self.p7_scope, "block": self.block, "slot": self.slot,
                               "p7_manifest_sha256": self.p7_frozen["p7"]["sha256"], "baselines": self.p7_frozen["baselines"],
                               "development": self.p7_frozen["development"], "dry_run": self.dry,
                               "dry_run_approval_sha256": approval,
                               "layout_record_sha256": digest(self.run / "p7/layout.json"), "world": None},
                        "p7e": {"contract": SUMMARY_CONTRACT, "p7e_manifest_sha256": sha,
                                "p7_manifest_sha256": self.p7_frozen["p7"]["sha256"], "scope": self.p7_scope,
                                "block": self.block, "slot": self.slot,
                                "pair_in_block": (self.slot_record or {}).get("pair_in_block"),
                                "order": (self.slot_record or {}).get("order"),
                                "schedule_canonical_sha256": self.p7_frozen["specs"]["schedule"],
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
            print("P7-E stopped: " + self.summary["failure"], flush=True)
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


class P7EP4Runner(P7EFlight, p7r.P7P4Runner):
    controller = "p4"


class P7EP5Runner(P7EFlight, p7r.P7P5Runner):
    controller = "p5"


RUNNERS = {"p4": P7EP4Runner, "p5": P7EP5Runner}


def parse(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true", required=True)
    parser.add_argument("--controller", choices=sorted(RUNNERS), required=True)
    parser.add_argument("--item", required=True, help="a P7-E layout id, or a frozen P7 layout or S1-S6 in a dry run")
    parser.add_argument("--scope", choices=SCOPES, required=True)
    parser.add_argument("--block", type=int, default=0)
    parser.add_argument("--slot", type=int, default=0)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    a = parser.parse_args(argv)
    if a.scope == p7e_manifest.DRY_RUN and (a.block or a.slot):
        parser.error("a dry run has no block or slot")
    if a.scope != p7e_manifest.DRY_RUN and (a.block < 1 or a.slot < 1 or a.item in p7r.ACCEPTED_WORLDS):
        parser.error("a P7-E block flight needs its block and slot, and flies a P7-E layout")
    role = "%s-%s-b%02d-s%03d-%s" % ("p7e", a.scope, a.block, a.slot, a.controller)
    return argparse.Namespace(run=True, controller=a.controller, item=a.item, p7_scope=a.scope, block=a.block, slot=a.slot,
                              gate="P7-E", role=role, manifest=a.manifest, authorization=a.authorization,
                              allow_simulated_flight=a.allow_simulated_flight, scope="flight", fixture=None,
                              scenario=a.item, variant=None)


def main(argv=None):
    args = parse(argv)
    return RUNNERS[args.controller](args).execute()


if __name__ == "__main__":
    raise SystemExit(main())
