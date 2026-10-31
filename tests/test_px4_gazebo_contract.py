"""Operator-entrypoint logic only: these tests never start Linux runtimes."""
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "simulation/px4_gazebo/scripts"
spec = importlib.util.spec_from_file_location("p1_contract", SCRIPTS / "p1_contract.py")
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)


@pytest.fixture
def config():
    return contract.load_config(SCRIPTS.parent / "configs/p1_smoke.yaml")


@pytest.mark.parametrize("missing", contract.GATES + ("GWM_ALLOW_SITL_COMMANDS",))
def test_flight_rejects_every_missing_gate(missing):
    env = dict.fromkeys(contract.GATES + ("GWM_ALLOW_SITL_COMMANDS",), "1")
    del env[missing]
    with pytest.raises(ValueError, match=missing):
        contract.require_gates(env, flight=True)


@pytest.mark.parametrize("value", (True, 1, "true", "yes", "0", ""))
def test_gate_requires_exact_string_one(value):
    env = dict.fromkeys(contract.GATES, "1")
    env[contract.GATES[0]] = value
    with pytest.raises(ValueError):
        contract.require_gates(env)


def test_boot_and_flight_authorization_are_separate():
    env = dict.fromkeys(contract.GATES, "1")
    contract.require_gates(env)
    with pytest.raises(ValueError, match="SITL"):
        contract.require_gates(env, flight=True)


@pytest.mark.parametrize("value", (0, -1, True, float("nan"), float("inf"), "2"))
def test_rejects_invalid_numeric_bounds(config, tmp_path, value):
    config["target_height_m"] = value
    path = tmp_path / "config.yaml"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        contract.load_config(path)


@pytest.mark.parametrize("field,value", (("model", "iris"), ("world", "windy"),
                                        ("control_owner", "mavsdk")))
def test_rejects_scope_expansion(config, tmp_path, field, value):
    config[field] = value
    path = tmp_path / "config.yaml"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        contract.load_config(path)


def observations():
    return [{"t": n / 10, "x": 0.0, "y": 0.0, "z": -2.0} for n in range(101)]


def test_measured_hover_passes_with_full_coverage(config):
    result = contract.evaluate_hover(observations(), config, ground_z=0)
    assert result["status"] == "passed"
    assert result["duration_sim_s"] == 10
    assert result["samples"] == 101


@pytest.mark.parametrize("change", ("height", "horizontal", "gap", "short", "reverse"))
def test_hover_failure_cannot_be_reported_as_success(config, change):
    samples = observations()
    if change == "height":
        samples[30]["z"] = -2.31
    elif change == "horizontal":
        samples[30]["x"] = 0.51
    elif change == "gap":
        del samples[20:30]
    elif change == "short":
        samples.pop()
    else:
        samples[30]["t"] = samples[29]["t"]
    assert contract.evaluate_hover(samples, config, 0)["status"] == "failed"


def test_missing_or_nan_measurements_are_unknown(config):
    assert contract.evaluate_hover([], config, 0)["status"] == "unknown"
    samples = observations()
    samples[20]["z"] = float("nan")
    assert contract.evaluate_hover(samples, config, 0)["status"] == "unknown"


def test_height_uses_initial_ned_ground_reference(config):
    samples = observations()
    for s in samples:
        s["z"] += 14
    assert contract.evaluate_hover(samples, config, 14)["status"] == "passed"


def test_partial_sample_and_missing_ground_are_unknown(config):
    samples = observations()
    del samples[20]["x"]
    assert contract.evaluate_hover(samples, config, 0)["status"] == "unknown"
    assert contract.evaluate_hover(observations(), config, None)["status"] == "unknown"


def test_listener_boolean_and_timestamp_parsing():
    value = contract.parse_topic("  timestamp: 1234567 (0.01 seconds ago)\n  xy_valid: True\n  z: -1.98\n  landed: False\n")
    assert value == {"timestamp": 1234567, "xy_valid": True, "z": -1.98, "landed": False}


def test_runner_import_never_launches_process(monkeypatch):
    monkeypatch.syspath_prepend(str(SCRIPTS))
    def forbidden(*args, **kwargs):
        raise AssertionError("Runtime launch during import")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    runner_spec = importlib.util.spec_from_file_location("p1_runner", SCRIPTS / "p1_runner.py")
    runner = importlib.util.module_from_spec(runner_spec)
    runner_spec.loader.exec_module(runner)
    reads = iter([{"t": 1}, {"t": 1}, {"t": 2}])
    monkeypatch.setattr(runner.time, "sleep", lambda _: None)
    assert runner.fresh_observation(lambda: next(reads), 1, runner.time.monotonic() + 1) == {"t": 2}
    with pytest.raises(RuntimeError, match="backwards"):
        runner.fresh_observation(lambda: {"t": 0}, 1, runner.time.monotonic() + 1)
    with pytest.raises(TimeoutError, match="fresh observation"):
        runner.fresh_observation(lambda: {"t": 1}, 1, runner.time.monotonic() - 1)


def test_lock_has_measured_exact_source_pins():
    lock = json.loads((SCRIPTS.parent / "configs/versions.lock.yaml").read_text())
    for source in lock["sources"].values():
        assert len(source["commit"]) == 40
        assert source["requested_ref"] != "latest"


def test_console_ignores_character_redraw_prompts():
    partial = "\x1b[2K\rpxh> l\x1b[2K\rpxh> lo\x1b[2K\rpxh> logger status"
    assert contract.completed_console_response(partial, "logger status") is None
    assert contract.completed_console_response(partial + "\r\nFile Logging: full\r\n", "logger status") is None
    result = contract.completed_console_response(partial + "\r\nFile Logging: full\r\npxh> ", "logger status")
    assert result == "File Logging: full\r\npxh> "


def test_console_accepts_boot_prompt_with_async_log():
    assert contract.completed_console_response("Startup complete\r\npxh> INFO home set\r\n") is not None


def test_console_accepts_async_log_after_complete_command_echo():
    command = "listener vehicle_local_position -n 1"
    echo = "pxh> " + command + "INFO  [commander] Landing at current position\r\n"
    assert contract.completed_console_response(echo, command) is None
    output = "\r\nTOPIC: vehicle_local_position\r\n  timestamp: 27208000\r\npxh> "
    assert contract.completed_console_response(echo + output, command) == output
