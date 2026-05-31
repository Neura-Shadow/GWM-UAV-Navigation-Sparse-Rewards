"""Tests for the Axis 2 — Asymmetric Control modules.

Covers the takeover arbiter, safety controller, and mock ROS2 adapter.
All tests run without GPU, AirSim, or ROS2.
"""

from __future__ import annotations

import numpy as np
import pytest

from src.utils.data_types import (
    ControlCommand,
    ControlMode,
    SensorObservation,
)
from src.control.ros2_adapter import MockROS2Adapter
from src.control.safety_controller import SafetyController
from src.control.takeover import TakeoverArbiter


# ── Helpers ────────────────────────────────────────────────────────────────

def _make_obs(
    pose: tuple = (0.0, 0.0, -5.0),
    velocity: tuple = (1.0, 0.0, 0.0),
) -> SensorObservation:
    return SensorObservation(timestamp=0.0, pose=pose, velocity=velocity)


def _make_cmd(
    vx: float = 1.0,
    vy: float = 0.0,
    vz: float = 0.0,
    mode: ControlMode = ControlMode.WORLD_MODEL_GUIDED,
) -> ControlCommand:
    return ControlCommand(vx=vx, vy=vy, vz=vz, mode=mode)


# ── TakeoverArbiter tests ─────────────────────────────────────────────────

class TestTakeoverArbiterDecide:
    """Test the three-zone decision logic."""

    def test_normal_conditions_world_model_guided(self) -> None:
        arb = TakeoverArbiter()
        mode = arb.decide(uncertainty=0.2, obstacle_dist=10.0)
        assert mode == ControlMode.WORLD_MODEL_GUIDED

    def test_high_uncertainty_triggers_safety_override(self) -> None:
        arb = TakeoverArbiter(uncertainty_threshold=0.7)
        mode = arb.decide(uncertainty=0.85, obstacle_dist=10.0)
        assert mode == ControlMode.SAFETY_OVERRIDE

    def test_close_obstacle_triggers_safety_override(self) -> None:
        arb = TakeoverArbiter(obstacle_threshold=4.0, emergency_obstacle_threshold=2.0)
        mode = arb.decide(uncertainty=0.1, obstacle_dist=3.5)
        assert mode == ControlMode.SAFETY_OVERRIDE

    def test_very_close_obstacle_triggers_emergency_stop(self) -> None:
        arb = TakeoverArbiter(emergency_obstacle_threshold=2.0)
        mode = arb.decide(uncertainty=0.1, obstacle_dist=1.5)
        assert mode == ControlMode.EMERGENCY_STOP

    def test_boundary_obstacle_at_threshold(self) -> None:
        arb = TakeoverArbiter(obstacle_threshold=4.0)
        # Exactly at threshold → safe (>=)
        mode = arb.decide(uncertainty=0.1, obstacle_dist=4.0)
        assert mode == ControlMode.WORLD_MODEL_GUIDED

    def test_boundary_uncertainty_at_threshold(self) -> None:
        arb = TakeoverArbiter(uncertainty_threshold=0.7)
        # Exactly at threshold → safe (> required)
        mode = arb.decide(uncertainty=0.7, obstacle_dist=10.0)
        assert mode == ControlMode.WORLD_MODEL_GUIDED


class TestTakeoverArbiterExecute:
    def test_selects_correct_command(self) -> None:
        arb = TakeoverArbiter()
        wm_cmd = _make_cmd(vx=3.0, mode=ControlMode.WORLD_MODEL_GUIDED)
        safety_cmd = _make_cmd(vx=-2.0, mode=ControlMode.SAFETY_OVERRIDE)

        # World model mode → world model command
        result = arb.execute(wm_cmd, safety_cmd, ControlMode.WORLD_MODEL_GUIDED)
        assert result.vx == pytest.approx(3.0)
        assert result.mode == ControlMode.WORLD_MODEL_GUIDED

        # Safety mode → safety command
        result = arb.execute(wm_cmd, safety_cmd, ControlMode.SAFETY_OVERRIDE)
        assert result.vx == pytest.approx(-2.0)
        assert result.mode == ControlMode.SAFETY_OVERRIDE

        # Emergency → safety command
        result = arb.execute(wm_cmd, safety_cmd, ControlMode.EMERGENCY_STOP)
        assert result.vx == pytest.approx(-2.0)
        assert result.mode == ControlMode.EMERGENCY_STOP


# ── SafetyController tests ────────────────────────────────────────────────

class TestSafetyController:
    def test_check_safety_safe(self) -> None:
        ctrl = SafetyController(min_obstacle_dist=4.0)
        assert ctrl.check_safety(_make_obs(), obstacle_dist=10.0) is True

    def test_check_safety_unsafe(self) -> None:
        ctrl = SafetyController(min_obstacle_dist=4.0)
        assert ctrl.check_safety(_make_obs(), obstacle_dist=3.0) is False

    def test_generates_retreat_command(self) -> None:
        ctrl = SafetyController(min_obstacle_dist=4.0, retreat_speed=2.0)
        cmd = ctrl.get_safe_command(_make_obs(), obstacle_dist=2.0, current_yaw=0.0)

        assert cmd.mode == ControlMode.SAFETY_OVERRIDE
        # At yaw=0, retreat should be negative vx
        assert cmd.vx < 0
        assert cmd.metadata.get("reason") == "obstacle_retreat"

    def test_hover_when_safe(self) -> None:
        ctrl = SafetyController(min_obstacle_dist=4.0)
        cmd = ctrl.get_safe_command(_make_obs(), obstacle_dist=5.0)

        assert cmd.vx == pytest.approx(0.0)
        assert cmd.vy == pytest.approx(0.0)
        assert cmd.metadata.get("reason") == "hover"

    def test_emergency_stop(self) -> None:
        ctrl = SafetyController()
        cmd = ctrl.emergency_stop()

        assert cmd.vx == pytest.approx(0.0)
        assert cmd.vy == pytest.approx(0.0)
        assert cmd.vz == pytest.approx(0.0)
        assert cmd.mode == ControlMode.EMERGENCY_STOP
        assert cmd.metadata.get("reason") == "emergency_stop"


# ── MockROS2Adapter tests ─────────────────────────────────────────────────

class TestMockROS2Adapter:
    def test_send_command(self) -> None:
        adapter = MockROS2Adapter()
        cmd = _make_cmd(vx=1.5)
        result = adapter.send_command(cmd)

        assert result is True

    def test_tracks_history(self) -> None:
        adapter = MockROS2Adapter()
        cmd1 = _make_cmd(vx=1.0)
        cmd2 = _make_cmd(vx=2.0)

        adapter.send_command(cmd1)
        adapter.send_command(cmd2)

        assert len(adapter.command_history) == 2
        assert adapter.command_history[0].vx == pytest.approx(1.0)
        assert adapter.command_history[1].vx == pytest.approx(2.0)

    def test_get_odometry(self) -> None:
        adapter = MockROS2Adapter()
        obs = adapter.get_odometry()

        assert obs is not None
        assert isinstance(obs, SensorObservation)
        assert obs.pose == (0.0, 0.0, 0.0)

    def test_disconnect_drops_commands(self) -> None:
        adapter = MockROS2Adapter()
        adapter.disconnect()

        assert adapter.send_command(_make_cmd()) is False
        assert adapter.get_odometry() is None

    def test_reconnect_restores(self) -> None:
        adapter = MockROS2Adapter()
        adapter.disconnect()
        adapter.reconnect()

        assert adapter.send_command(_make_cmd()) is True
        assert adapter.get_odometry() is not None
