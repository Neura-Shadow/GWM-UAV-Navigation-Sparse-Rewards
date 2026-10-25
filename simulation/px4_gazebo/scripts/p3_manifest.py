"""P3 frozen candidate identity, operator authorization and gate bookkeeping.

The candidate manifest content-addresses every input that can change runtime
behaviour or acceptance: repository sources/configs/evaluators, the untracked
but required MC-trace overlay sources, build receipts, binaries, pinned assets,
ROS/DDS runtime libraries and the effective native-command policy. A runtime
scope may start only when the recomputed identity equals the frozen one and an
operator authorization binds that exact manifest and scope. Authorizations are
consumed once; a consumed scope is never retried automatically.
"""
import os
from pathlib import Path
import sys
import time

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parents[1]
sys.path.insert(0, str(SIM / "validation"))
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

CONTRACT = "p3-candidate-manifest-v1"
GATES = {"A": "ground_readiness_and_sensor_matrix", "B": "p3_nominal_flight_smoke", "C": "p3_qualification",
         # Operator-authorized renderer evaluation: Gate A's ground plan on llvmpipe, never acceptance credit.
         "V": "ground_validation_llvmpipe_no_credit"}
GROUND_GATES = ("A", "V")
SOURCE_FOLDERS = ("scripts", "validation", "configs", "ros2_ws/src/gwm_px4_control", "ros2_ws/src/gwm_sensor_adapter")
RUNTIME_LIBRARIES = ("/opt/ros/jazzy/lib/libfastrtps.so.2.14", "/opt/ros/jazzy/lib/librmw_fastrtps_shared_cpp.so",
                     "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/publisher.py",
                     "/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge")
PINNED_ASSETS = ("Tools/simulation/gz/worlds/default.sdf", "src/modules/simulation/gz_bridge/server.config")
MODEL_FOLDERS = ("x500_depth", "OakD-Lite", "x500", "x500_base")


def source_files():
    files = {}
    for folder in SOURCE_FOLDERS:
        for path in sorted((SIM / folder).rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                files[path.relative_to(SIM).as_posix()] = digest(path)
    files["../../docs/evidence/v3_sim_p2_summary.json"] = digest(REPO / "docs/evidence/v3_sim_p2_summary.json")
    return files


def required_mc_sources():
    names = ["validation/mc_trace/GwmMcTrace.cpp", "validation/mc_trace/GwmMcTrace.hpp",
             "validation/mc_trace_evidence.py", "validation/mc_trace_schema.py", "validation/mc_trace_runtime.py",
             "validation/mc_trace_benchmark.cpp", "scripts/build_mc_trace.py", "scripts/benchmark_mc_trace.py"]
    missing = [n for n in names if not (SIM / n).is_file()]
    if missing:
        raise ValueError("Required MC causal-trace sources absent: " + ", ".join(missing))
    return {n: digest(SIM / n) for n in names}


def runtime_identity(root, policy_name=None, policy_approval=None, mc_benchmark_run=None):
    """Linux-side identity. Fails closed when any build/receipt is missing."""
    root = Path(root)
    px4 = root / "upstream/PX4-Autopilot"
    receipts = {name: strict_json(root / "state" / name) for name in
                ("p0-built.json", "p2-built.json", "p3-built.json", "px4-mc-trace-built.json", "qgc-unattended-built.json",
                 "p3-dds-agent-built.json")}
    agent = receipts["p3-dds-agent-built.json"]
    if agent.get("status") != "built" or digest(agent["binary"]) != agent["binary_sha256"]:
        raise ValueError("P3 DDS agent build missing, failed or changed")
    sys.path.insert(0, str(SIM / "validation"))
    import mc_trace_runtime
    previous = os.environ.get("GWM_PX4_MC_TRACE")
    os.environ["GWM_PX4_MC_TRACE"] = mc_trace_runtime.CONTRACT  # explicit selection, never automatic
    try:
        mc = mc_trace_runtime.selected_build(root, SIM)
    finally:
        if previous is None:
            os.environ.pop("GWM_PX4_MC_TRACE")
        else:
            os.environ["GWM_PX4_MC_TRACE"] = previous
    if mc_benchmark_run is None:
        raise ValueError("A passed native MC benchmark run is required")
    benchmark = strict_json(Path(mc_benchmark_run) / "summary.json")
    if (benchmark.get("contract") != "px4-mc-overhead-v1" or benchmark.get("status") != "passed"
            or benchmark.get("build") != mc or not all(p.get("all_output_bits_equal") for p in benchmark["pairs"])):
        raise ValueError("MC instrumentation benchmark is not a passed run for this exact build")
    qgc = receipts["qgc-unattended-built.json"]
    if qgc.get("owner_script_sha256") != digest(SIM / "scripts/p3_namespace_owner.py"):
        raise ValueError("Unattended QGC build does not bind the P3 namespace owner")
    if digest(qgc["ELF"]) != qgc["ELF_sha256"]:
        raise ValueError("Unattended QGC ELF changed")
    sys.path.insert(0, str(SIM / "validation"))
    from p3_mavlink import load_policy
    policy = load_policy(SIM / "configs/p3_native_command_policy.json", policy_name, policy_approval)
    assets = {name: digest(px4 / name) for name in PINNED_ASSETS}
    for folder in MODEL_FOLDERS:
        for path in sorted((px4 / "Tools/simulation/gz/models" / folder).rglob("*")):
            if path.is_file():
                assets[path.relative_to(px4).as_posix()] = digest(path)
    return {"receipts": {k: canonical_sha256(v) for k, v in receipts.items()},
            "baseline_px4_sha256": receipts["p0-built.json"]["measured"]["px4_binary_sha256"],
            "mc_trace_build": mc, "mc_benchmark": {"run": str(mc_benchmark_run),
                                                   "sha256": digest(Path(mc_benchmark_run) / "summary.json")},
            "qgc_unattended_elf_sha256": qgc["ELF_sha256"], "assets": assets,
            "runtime_libraries": {p: digest(p) for p in RUNTIME_LIBRARIES},
            "native_command_policy": policy,
            "renderer_requirement": strict_json(SIM / "configs/p3_validation.yaml")["renderer_requirement"]}


def candidate(root, **kwargs):
    value = {"contract": CONTRACT, "source_files": source_files(), "mc_sources": required_mc_sources(),
             "runtime": runtime_identity(root, **kwargs), "gates": GATES, "execution_allowed_without_authorization": False,
             "qualification_flights": strict_json(SIM / "configs/p3_validation.yaml")["qualification_flights"]}
    return value


def freeze(root, output, **kwargs):
    value = candidate(root, **kwargs)
    value["frozen_unix_ns"] = time.time_ns()
    atomic_json(output, value, exclusive=True)
    return canonical_sha256({k: v for k, v in value.items() if k != "frozen_unix_ns"})


def manifest_sha(value):
    return canonical_sha256({k: v for k, v in value.items() if k != "frozen_unix_ns"})


def verify_current(root, manifest_path):
    """Recompute everything; any byte difference blocks the runtime."""
    frozen = strict_json(manifest_path)
    runtime = frozen["runtime"]
    approval = runtime["native_command_policy"].get("approval_path")
    current = candidate(root, policy_name=runtime["native_command_policy"]["name"], policy_approval=approval,
                        mc_benchmark_run=runtime["mc_benchmark"]["run"])
    current["frozen_unix_ns"] = frozen.get("frozen_unix_ns")
    if current != frozen:
        differing = sorted(k for k in set(current) | set(frozen) if current.get(k) != frozen.get(k))
        raise ValueError("Current inputs differ from the frozen P3 candidate: " + ",".join(differing))
    return frozen, manifest_sha(frozen)


def require_authorization(path, gate, sha):
    value = strict_json(path)
    expected = {"contract": "p3-runtime-authorization-v1", "gate": gate, "manifest_sha256": sha,
                "scope": GATES[gate], "max_attempts": 1}
    for key, item in expected.items():
        if value.get(key) != item:
            raise PermissionError("Authorization does not bind " + key)
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Authorization is not an explicit operator approval for this scope")
    return value


def consume(root, gate, sha, authorization_path):
    """Exclusive, permanent consumption marker: no automatic retry is possible."""
    marker = Path(root) / "state" / f"p3-gate-{gate}-{sha}.consumed.json"
    atomic_json(marker, {"gate": gate, "manifest_sha256": sha, "authorization_sha256": digest(authorization_path),
                         "consumed_unix_ns": time.time_ns()}, exclusive=True)
    return marker


def authorization_template(gate, sha, operator):
    return {"contract": "p3-runtime-authorization-v1", "gate": gate, "scope": GATES[gate], "manifest_sha256": sha,
            "max_attempts": 1, "authorized": True, "authorized_by": operator,
            "excludes": ["flight"] if gate in GROUND_GATES else [],
            "statement": f"I explicitly authorize exactly one P3 Gate {gate} ({GATES[gate]}) execution of manifest {sha}."}


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description="Freeze or verify the P3 candidate; never starts runtime.")
    sub = parser.add_subparsers(dest="action", required=True)
    f = sub.add_parser("freeze")
    f.add_argument("--output", type=Path, required=True)
    f.add_argument("--policy", default=None)
    f.add_argument("--policy-approval", type=Path)
    f.add_argument("--mc-benchmark-run", type=Path, required=True)
    v = sub.add_parser("verify")
    v.add_argument("manifest", type=Path)
    a = sub.add_parser("authorize")
    a.add_argument("manifest", type=Path)
    a.add_argument("--gate", choices=sorted(GATES), required=True)
    a.add_argument("--operator", required=True)
    a.add_argument("--confirm", required=True, help='Exactly: "authorize Gate <G> <manifest-sha>"')
    a.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    if args.action == "freeze":
        print(freeze(root, args.output, policy_name=args.policy, policy_approval=args.policy_approval,
                     mc_benchmark_run=args.mc_benchmark_run))
    elif args.action == "verify":
        print(verify_current(root, args.manifest)[1])
    else:
        _, sha = verify_current(root, args.manifest)
        if args.confirm != f"authorize Gate {args.gate} {sha}":
            raise SystemExit("Confirmation phrase must be exactly: authorize Gate %s %s" % (args.gate, sha))
        atomic_json(args.output, authorization_template(args.gate, sha, args.operator), exclusive=True)
        print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
