"""Installed-package checks run by colcon test (build time only; no runtime launched)."""
import importlib


def test_modules_import_and_contract_constants():
    for name in ("contracts", "local_map", "planner", "supervisor", "perception", "mission",
                 "control_node", "perception_node"):
        importlib.import_module("gwm_obstacle_avoidance." + name)
    from gwm_obstacle_avoidance import contracts, planner
    assert contracts.PINNED[("planner", "r_infl_m")] == 1.0
    assert contracts.PINNED[("mission", "map_scan_deg")] == 360.0
    assert planner.TRANSLATE == "TRANSLATE"


def test_nodes_refuse_without_runtime_gates(monkeypatch, tmp_path):
    import pytest
    from gwm_obstacle_avoidance import perception_node
    for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH"):
        monkeypatch.delenv(key, raising=False)
    with pytest.raises(ValueError):
        perception_node.require_gates({})
