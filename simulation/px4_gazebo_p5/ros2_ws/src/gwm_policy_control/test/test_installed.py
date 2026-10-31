"""Installed-package checks run by colcon test (build time only; no runtime launched)."""
import importlib


def test_modules_import_and_contract_constants():
    for name in ("contracts", "policy_runtime", "observation_adapter", "candidate_ranker", "decision", "safety_shield",
                 "policy_ledger", "mission", "control_node"):
        importlib.import_module("gwm_policy_control." + name)
    from gwm_policy_control import contracts, control_node, policy_ledger
    assert contracts.ARCHITECTURE == "p5-ranker-mlp-v1" and contracts.FEATURES == 46 and contracts.SLOTS == 26
    assert control_node.CONTROLLER == "p5_control" and policy_ledger.LEDGER_CONTRACT == "p5-policy-ledger-v1"


def test_controller_refuses_without_runtime_gates(monkeypatch, tmp_path):
    import pytest
    from gwm_policy_control import control_node
    for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH"):
        monkeypatch.delenv(key, raising=False)
    with pytest.raises(ValueError, match="explicit gates"):
        control_node.main(["--config", "c", "--timing-config", "t", "--p4-config", "p", "--p5-config", "q",
                           "--schema", "s", "--schema-sha256", "0", "--model", "m", "--model-sha256", "0",
                           "--run-dir", str(tmp_path), "--authorization-marker", str(tmp_path / "none"), "--ground-diagnostic"])


def test_untrained_weights_are_refused_by_the_installed_runtime(tmp_path):
    import numpy as np
    import pytest
    from gwm_policy_control import contracts, policy_runtime
    arrays = {name: np.zeros(shape) for name, shape in contracts.LAYER_SHAPES.items()}
    sha = policy_runtime.save_model(tmp_path / "m.npz", arrays, {"architecture": contracts.ARCHITECTURE,
                                                                  "schema_sha256": "s", "trained": False})
    with pytest.raises(policy_runtime.PolicyError, match="untrained_model_refused"):
        policy_runtime.FrozenPolicy.load(tmp_path / "m.npz", sha, "s", require_trained=True)
