"""P5 M0: configuration contract, frozen schema, source isolation and the P3/P4 baseline guards."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402
from gwm_policy_control import contracts  # noqa: E402
import p5_manifest  # noqa: E402

REPO = p5.syn.REPO
ACCEPTED_P4_COMMIT = "50d5002e9670d70f07c43792415494ae70f4efe0"  # P4 candidate c110bd30 sources
FROZEN_PATHS = ["simulation/px4_gazebo/scripts", "simulation/px4_gazebo/validation", "simulation/px4_gazebo/configs",
                "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control", "simulation/px4_gazebo/ros2_ws/src/gwm_sensor_adapter",
                "docs/evidence/v3_sim_p2_summary.json"] + [
                "simulation/px4_gazebo_p4/" + f for f in p5_manifest.P4_SOURCE_FOLDERS]


def test_policy_config_is_disabled_has_no_fallback_and_no_bound_model():
    config = contracts.load_policy_config(p5.P5 / "configs/p5_policy.yaml")
    assert config["enabled"] is False and config["fallback"] == "none"
    assert config["model"]["path"] is None and config["model"]["sha256"] is None
    with pytest.raises(PermissionError):
        contracts.require_enabled(config, authorization_present=True)
    enabled = json.loads(json.dumps(config))
    enabled["enabled"] = True
    with pytest.raises(PermissionError):  # still no checkpoint bound
        contracts.require_enabled(enabled, authorization_present=True)


@pytest.mark.parametrize("path,value", [(("fallback",), "p4"), (("model", "require_trained"), False),
                                        (("inference", "failure_wall_s"), 0.05), (("model", "architecture"), "other")])
def test_policy_config_rejects_changed_contract_values(tmp_path, path, value):
    config = json.loads((p5.P5 / "configs/p5_policy.yaml").read_text(encoding="utf-8"))
    node = config
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value
    changed = tmp_path / "policy.json"
    changed.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="contract value changed"):
        contracts.load_policy_config(changed)


def test_model_schema_freezes_slots_features_and_layers():
    schema = p5.SCHEMA
    assert schema["slots"]["count"] == 26 and schema["features_per_slot"] == 46
    assert [f["name"] for f in schema["candidate_features"]][:3] == ["k", "nominal_length", "clipped_length"]
    assert schema["layers"] == [[46, 64], [64, 64], [64, 1]] and schema["dtype"] == "float64"
    assert len(p5.SCHEMA_SHA) == 64 and p5.SCHEMA_SHA == p5_manifest.schema_identity()
    p4 = p5.syn.p4_config()["planner"]  # the slot layout is P4's candidate set
    assert schema["slots"]["heading_steps"] == p4["heading_steps"] and schema["slots"]["lengths_m"] == p4["lengths_m"]


def test_validation_binds_the_accepted_p3_and_p4_baselines_and_the_p4_gate_structure():
    v = p5_manifest.validation()
    p4 = json.loads((REPO / "simulation/px4_gazebo_p4/configs/p4_validation.yaml").read_text(encoding="utf-8"))
    assert v["p3_baseline_manifest_sha256"] == p4["p3_baseline_manifest_sha256"] == "1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c"
    assert v["p4_baseline_manifest_sha256"] == "c110bd30be22fb43e596cf1f4b01397cb97145138f8912d6535a7dc4c15da42b"
    assert v["policy"] == {"inference_p99_s": 0.005, "inference_max_s": 0.010, "failure_wall_s": 0.020,
                           "consecutive_failures_to_abort": 3, "failure_ticks_allowed": 0, "participation_fraction": 1.0,
                           "outside_mask_allowed": 0, "fallback_decisions_allowed": 0}
    assert v["predecessor"] == p4["predecessor"] and v["gates"]["C"] == p4["gates"]["C"] and v["gates"]["S"] == p4["gates"]["S"]


def test_p5_tree_is_outside_every_p3_and_p4_manifest_glob():
    import p3_manifest
    import p4_manifest
    sources = p5_manifest.p5_source_files()
    assert "configs/p5_model_schema.json" in sources and not any("__pycache__" in k for k in sources)
    assert not any("px4_gazebo_p5" in k for k in p4_manifest.p4_source_files())
    assert not any("px4_gazebo_p5" in str(p) for p in p3_manifest.source_files())


def test_p3_and_p4_frozen_sources_are_unchanged_since_the_accepted_p4_candidate():
    # autocrlf: the Windows checkout stores CRLF; content, not line endings, is what the manifests bind.
    diff = subprocess.run(["git", "-C", str(REPO), "-c", "core.autocrlf=true", "diff", "--name-only", ACCEPTED_P4_COMMIT, "--", *FROZEN_PATHS],
                          capture_output=True, text=True, check=True).stdout.split()
    untracked = subprocess.run(["git", "-C", str(REPO), "-c", "core.autocrlf=true", "status", "--porcelain", "--untracked-files=all", "--", *FROZEN_PATHS],
                               capture_output=True, text=True, check=True).stdout.split("\n")
    assert diff == [] and [line for line in untracked if line.strip()] == []


@pytest.mark.skipif(not (Path.home() / "uav_autonomy/state/p4-candidate-50d5002.json").exists(),
                    reason="accepted baselines live in the Linux workspace")
def test_accepted_p3_and_p4_baselines_still_verify(monkeypatch):
    monkeypatch.setenv("GWM_PX4_MC_TRACE", "px4-mc-causal-trace-v1")
    assert p5_manifest.baselines(Path.home() / "uav_autonomy") == {
        "p3_sha256": "1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c",
        "p4_sha256": "c110bd30be22fb43e596cf1f4b01397cb97145138f8912d6535a7dc4c15da42b"}
