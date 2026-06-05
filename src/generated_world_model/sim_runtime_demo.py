"""Phase 6-F guarded GWM / WAM closed-loop simulation demo.

The runner connects the generated-world-model planner to simulation-only
runtime seams. Normal tests inject fake Isaac/ROS2/MAVSDK objects; live runtime
construction is attempted only after explicit environment gates and safe
deployment checks pass.
"""

from __future__ import annotations

import asyncio
import copy
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import numpy as np
import torch

from src.control.barrier_certificate import ControlBarrierFunction, SafetyLimits
from src.generated_world_model.observation_buffer import ObservationBuffer
from src.generated_world_model.planner import GeneratedWorldModelPlanner
from src.generated_world_model.rollout import AutoregressiveRollout
from src.generated_world_model.training import build_baseline_components
from src.generated_world_model.trajectory_sampler import CandidateTrajectorySampler
from src.generated_world_model.trajectory_scorer import TrajectoryScorer
from src.generated_world_model.types import GWMConfig, TrajectoryCandidate
from src.utils.data_types import ControlCommand, ControlMode, SensorObservation

SCHEMA_VERSION = "gwm_phase6_simulation_demo_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/phase6_gwm_simulation_demo.json"

COMMON_ENV_GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME",)
ISAAC_ENV_GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_ISAAC_RUNTIME_TESTS")
ROS2_ENV_GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_ROS2_SENSOR_SYNC_TESTS")
MAVSDK_PX4_ENV_GATES = (
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_RUN_MAVSDK_SITL_TESTS",
    "GWM_ALLOW_SITL_COMMANDS",
)
OPTIONAL_ENV_GATES = ("GWM_ROS2_LIVE_TOPICS", "GWM_ALLOW_PX4_LAUNCH")


def _pure_sim_deployment() -> dict[str, bool]:
    return {
        "mock": False,
        "sitl_enabled": True,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }


def _default_prior_reports() -> dict[str, str]:
    return {
        "isaac_sensor_runtime": "outputs/runtime_validation/isaac_sensor_runtime.json",
        "ros2_sim_sensor_bridge": "outputs/runtime_validation/ros2_sim_sensor_bridge.json",
        "px4_sitl_command_validation": (
            "outputs/runtime_validation/px4_sitl_command_validation.json"
        ),
        "isaac_px4_bridge_design": "outputs/runtime_validation/isaac_px4_bridge_design.json",
    }


@dataclass
class Phase6RuntimeReadiness:
    """Readiness summary consumed by the Phase 6-F simulation demo."""

    required_reports: dict[str, str]
    report_readiness: dict
    env_gates: dict

    @property
    def all_reports_ready(self) -> bool:
        return bool(self.report_readiness.get("all_ready", False))

    def to_dict(self) -> dict:
        return {
            "required_reports": dict(self.required_reports),
            "report_readiness": copy.deepcopy(self.report_readiness),
            "runtime_gates": copy.deepcopy(self.env_gates),
        }


@dataclass
class Phase6GWMSimulationDemoConfig:
    """Configuration for the Phase 6-F guarded simulation demo."""

    runtime_mode: str = "guarded"
    steps: int = 5
    horizon: int = 3
    num_candidates: int = 4
    seed: int = 7
    device: str = "cpu"
    observation_path: str = "direct_isaac"
    output_path: str | None = None
    write_output: bool = True
    require_prior_reports: bool = False
    fail_on_unavailable: bool = False
    future_full_state_coupling_requested: bool = False
    px4_launch_requested: bool = False
    planner_interval_steps: int = 1
    control_dt: float = 0.2
    context_length: int = 4
    image_height: int = 32
    image_width: int = 32
    max_speed: float = 1.0
    goal: tuple[float, float, float] = (8.0, 0.0, -5.0)
    goal_reach_dist: float = 1.0
    collision_dist: float = 0.5
    stale_observation_timeout_sec: float = 0.25
    stale_command_timeout_sec: float = 0.2
    deployment: dict[str, Any] = field(default_factory=_pure_sim_deployment)
    model: dict[str, Any] = field(default_factory=dict)
    scorer_weights: dict[str, float] = field(default_factory=dict)
    safety: dict[str, Any] = field(default_factory=dict)
    mavlink: dict[str, Any] = field(default_factory=dict)
    isaac: dict[str, Any] = field(default_factory=dict)
    ros2: dict[str, Any] = field(default_factory=dict)
    prior_reports: dict[str, str] = field(default_factory=_default_prior_reports)
    frame_transform_policy: dict[str, Any] = field(default_factory=dict)


@dataclass
class Phase6GWMSimulationDemoResult:
    """Thin wrapper around a JSON-safe Phase 6-F result payload."""

    payload: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return dict(self.payload)


def run_phase6_gwm_simulation_demo(
    config: dict | Phase6GWMSimulationDemoConfig | None = None,
    *,
    isaac_env: Any | None = None,
    ros2_synchronizer: Any | None = None,
    mavlink_bridge: Any | None = None,
) -> dict:
    """Run the guarded Phase 6-F simulation demo."""
    demo_config = _normalize_config(config)
    start = time.perf_counter()
    timings: Dict[str, Any] = {"started_at_unix": time.time()}
    errors: list[dict[str, str]] = []
    injected = {
        "isaac_env": isaac_env is not None,
        "ros2_synchronizer": ros2_synchronizer is not None,
        "mavlink_bridge": mavlink_bridge is not None,
    }
    readiness = _runtime_readiness(demo_config)
    payload = _base_payload(demo_config, readiness, timings, injected)

    refusal = _configuration_refusal(demo_config)
    if refusal is not None:
        payload["status"] = "failed"
        payload["reason"] = refusal
        payload["errors"].append({"type": "RuntimeError", "message": refusal})
        return _finalize_payload(payload, demo_config, start)

    if demo_config.require_prior_reports and not readiness.all_reports_ready:
        missing = [
            name
            for name, report in readiness.report_readiness.get("reports", {}).items()
            if not bool(report.get("ready", False))
        ]
        payload["status"] = "not_ready"
        payload["reason"] = f"Missing or unready prior Phase 6 reports: {', '.join(missing)}"
        return _finalize_payload(payload, demo_config, start)

    if (
        demo_config.future_full_state_coupling_requested
        and not bool(demo_config.frame_transform_policy.get("transforms_defined", False))
    ):
        payload["status"] = "not_ready"
        payload["reason"] = "Full Isaac/PX4 state coupling requires explicit frame transforms."
        return _finalize_payload(payload, demo_config, start)

    if not _all_runtime_injected_for_path(demo_config, injected):
        missing_gates = _missing_env_gates_for_path(demo_config)
        if missing_gates:
            payload["status"] = "skipped"
            payload["reason"] = (
                "Missing required Phase 6-F runtime env gates: "
                + ", ".join(missing_gates)
            )
            return _finalize_payload(payload, demo_config, start)

        unavailable = _runtime_unavailable_reason(demo_config)
        if unavailable is not None:
            payload["status"] = "runtime_unavailable"
            payload["reason"] = unavailable
            if demo_config.fail_on_unavailable:
                payload["errors"].append({"type": "RuntimeError", "message": unavailable})
            return _finalize_payload(payload, demo_config, start)

    runtime_invocation = payload["runtime_invocation_summary"]
    observation_source = None
    command_backend = None
    try:
        torch.manual_seed(demo_config.seed)
        np.random.seed(demo_config.seed)
        observation_source = _build_observation_source(
            demo_config,
            isaac_env=isaac_env,
            ros2_synchronizer=ros2_synchronizer,
            runtime_invocation=runtime_invocation,
        )
        command_backend = _build_command_backend(
            demo_config,
            mavlink_bridge=mavlink_bridge,
            runtime_invocation=runtime_invocation,
        )
        result_payload = _run_loop(
            demo_config,
            payload,
            observation_source,
            command_backend,
        )
        payload.update(result_payload)
    except Exception as exc:
        payload["status"] = "failed"
        payload["reason"] = str(exc)
        payload["errors"].append({"type": exc.__class__.__name__, "message": str(exc)})
        if command_backend is not None and getattr(command_backend, "connected", False):
            payload["safety_summary"]["emergency_stop_attempted"] = True
            try:
                command_backend.emergency_stop(str(exc))
            except Exception as stop_exc:  # pragma: no cover
                payload["errors"].append(
                    {
                        "type": stop_exc.__class__.__name__,
                        "message": f"emergency_stop failed: {stop_exc}",
                    }
                )
    finally:
        if command_backend is not None:
            try:
                command_backend.close()
                payload["closed"]["mavsdk_bridge"] = True
            except Exception as exc:  # pragma: no cover
                payload["closed"]["mavsdk_bridge"] = False
                payload["errors"].append(
                    {"type": exc.__class__.__name__, "message": f"bridge close failed: {exc}"}
                )
        if observation_source is not None:
            try:
                observation_source.close()
                payload["closed"]["observation_source"] = True
            except Exception as exc:  # pragma: no cover
                payload["closed"]["observation_source"] = False
                payload["errors"].append(
                    {"type": exc.__class__.__name__, "message": f"source close failed: {exc}"}
                )

    return _finalize_payload(payload, demo_config, start)


def _run_loop(
    config: Phase6GWMSimulationDemoConfig,
    payload: dict,
    observation_source: "_Phase6ObservationSource",
    command_backend: "_Phase6CommandBackend",
) -> dict:
    device = torch.device(config.device)
    gwm_config = _gwm_config(config)
    encoder, conditioner, model = build_baseline_components(gwm_config)
    encoder.to(device).eval()
    conditioner.to(device).eval()
    model.to(device).eval()
    rollout = AutoregressiveRollout(model=model, encoder=encoder, conditioner=conditioner)
    sampler = _ConfiguredCandidateSampler(
        CandidateTrajectorySampler(horizon=config.horizon, dt=config.control_dt, seed=config.seed),
        max_speed=config.max_speed,
    )
    planner = GeneratedWorldModelPlanner(
        rollout=rollout,
        scorer=TrajectoryScorer(weights=config.scorer_weights),
        sampler=sampler,
    )
    cbf = _build_cbf(config)
    buffer = ObservationBuffer(
        context_length=config.context_length,
        image_size=(config.image_height, config.image_width),
    )

    observation = observation_source.reset()
    buffer.append(observation)
    while not buffer.is_ready:
        observation = observation_source.observe()
        buffer.append(observation)

    initial_zero = _zero_command(config, "initial_zero_setpoint")
    initial_safe, _ = _apply_safety_gate(cbf, config, observation, initial_zero)
    command_backend.connect()
    command_backend.wait_until_ready()
    command_backend.start_offboard(initial_safe)

    latest_safe_command = initial_zero
    latest_safe_command_step = -1
    latest_plan: dict[str, Any] | None = None
    planner_updates = 0
    stale_commands = 0
    stale_observations = 0
    steps: list[dict[str, Any]] = []

    for step_index in range(max(1, int(config.steps))):
        if _observation_stale(observation, config, step_index):
            stale_observations += 1
            raw_command = _zero_command(config, "stale_observation_hold")
            safe_command, safety_decision = _apply_safety_gate(cbf, config, observation, raw_command)
        elif step_index % max(1, int(config.planner_interval_steps)) == 0 or latest_plan is None:
            context = buffer.as_observation_batch().to(device)
            with torch.no_grad():
                latest_plan = planner.plan(
                    context=context,
                    start=observation.pose,
                    goal=config.goal,
                    safety_context=_safety_context(cbf),
                    num_candidates=config.num_candidates,
                )
            planner_updates += 1
            raw_command = _command_from_candidate(
                latest_plan["candidate"],
                duration=config.control_dt,
                step_index=step_index,
                score=latest_plan["score"],
            )
            safe_command, safety_decision = _apply_safety_gate(cbf, config, observation, raw_command)
            latest_safe_command = safe_command
            latest_safe_command_step = step_index
        else:
            stale_age_steps = step_index - latest_safe_command_step
            stale_age_sec = stale_age_steps * config.control_dt
            if stale_age_sec > config.stale_command_timeout_sec:
                stale_commands += 1
                raw_command = _zero_command(config, "stale_command_hold")
                safe_command, safety_decision = _apply_safety_gate(cbf, config, observation, raw_command)
                latest_safe_command = safe_command
                latest_safe_command_step = step_index
            else:
                raw_command = latest_safe_command
                safe_command = latest_safe_command
                safety_decision = {
                    "reused_safe_command": True,
                    "stale_age_sec": float(stale_age_sec),
                    "safe": _command_to_dict(safe_command),
                }

        backend_result = command_backend.send_command(safe_command)
        next_observation = observation_source.apply_command(safe_command)
        buffer.append(next_observation)

        score = latest_plan["score"] if latest_plan is not None else {"total_score": 0.0, "components": {}}
        step_record = _step_record(
            config=config,
            step_index=step_index,
            observation=next_observation,
            raw_command=raw_command,
            safe_command=safe_command,
            safety_decision=safety_decision,
            score=score,
            backend_result=backend_result,
            planner_updated=step_index == latest_safe_command_step,
        )
        steps.append(step_record)
        observation = next_observation

        if step_record["collision_detected"]:
            payload["status"] = "safety_stop"
            payload["reason"] = "Collision threshold reached in simulation observation."
            break
        if step_record["goal_reached"]:
            payload["status"] = "passed"
            payload["reason"] = None
            break
    else:
        payload["status"] = "passed"
        payload["reason"] = None

    metrics = _metrics_from_steps(
        steps,
        payload["status"],
        planner_updates=planner_updates,
        stale_observations=stale_observations,
        stale_commands=stale_commands,
        command_backend=command_backend,
        observation_source=observation_source,
    )
    return {
        "steps": steps,
        "metrics": metrics,
        "loop_summary": {
            "steps_requested": int(config.steps),
            "steps_completed": len(steps),
            "planner_interval_steps": int(config.planner_interval_steps),
            "planner_updates": int(planner_updates),
            "state_coupling": "command_mirror",
            "px4_telemetry_used_for_isaac_state": False,
            "observation_path": config.observation_path,
        },
        "bridge_summary": command_backend.summary(),
        "backend_summary": {
            "observation_source": observation_source.summary(),
            "command_backend": command_backend.summary(),
        },
        "safety_summary": {
            **payload["safety_summary"],
            "safety_overrides": metrics["safety_overrides"],
            "emergency_stops": metrics["emergency_stops"],
            "cbf_applied_before_every_mavsdk_write": True,
        },
    }


class _Phase6ObservationSource:
    def reset(self) -> SensorObservation:
        raise NotImplementedError

    def observe(self) -> SensorObservation:
        raise NotImplementedError

    def apply_command(self, command: ControlCommand) -> SensorObservation:
        raise NotImplementedError

    def close(self) -> None:
        return None

    def summary(self) -> dict:
        return {}


class _DirectIsaacObservationSource(_Phase6ObservationSource):
    def __init__(self, env: Any) -> None:
        self.env = env
        self.frames = 0

    def reset(self) -> SensorObservation:
        observation = self.env.reset()
        self.frames += 1
        return observation

    def observe(self) -> SensorObservation:
        getter = getattr(self.env, "get_observation", None)
        observation = getter() if callable(getter) else self.env.reset()
        self.frames += 1
        return observation

    def apply_command(self, command: ControlCommand) -> SensorObservation:
        action = np.asarray((command.vx, command.vy, command.vz), dtype=np.float32)
        observation, _, _, _ = self.env.step(action)
        self.frames += 1
        return observation

    def close(self) -> None:
        close = getattr(self.env, "close", None)
        if callable(close):
            close()

    def summary(self) -> dict:
        return {"type": "direct_isaac", "isaac_frames_stepped": int(self.frames)}


class _ROS2IsaacObservationSource(_Phase6ObservationSource):
    def __init__(self, env: Any, synchronizer: Any) -> None:
        self.env = env
        self.synchronizer = synchronizer
        self.frames = 0
        self.packets = 0

    def reset(self) -> SensorObservation:
        reset = getattr(self.env, "reset", None)
        if callable(reset):
            reset()
        return self.observe()

    def observe(self) -> SensorObservation:
        observation = self.synchronizer.latest_observation()
        if observation is None:
            raise RuntimeError("ROS2 observation path has no synchronized observation.")
        self.packets += 1
        return observation

    def apply_command(self, command: ControlCommand) -> SensorObservation:
        action = np.asarray((command.vx, command.vy, command.vz), dtype=np.float32)
        step = getattr(self.env, "step", None)
        if callable(step):
            step(action)
            self.frames += 1
        return self.observe()

    def close(self) -> None:
        shutdown = getattr(self.synchronizer, "shutdown", None)
        if callable(shutdown):
            shutdown()
        close = getattr(self.env, "close", None)
        if callable(close):
            close()

    def summary(self) -> dict:
        return {
            "type": "ros2",
            "isaac_frames_stepped": int(self.frames),
            "ros2_packets_observed": int(self.packets),
        }


class _FakePhase6IsaacEnv:
    """Small local simulation stand-in used only by explicit fake-mode CLI runs."""

    def __init__(self, config: Phase6GWMSimulationDemoConfig) -> None:
        self.config = config
        self.pose = np.array([0.0, 0.0, config.goal[2]], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.goal = np.asarray(config.goal, dtype=np.float32)
        self.step_count = 0
        self.closed = False
        self.obstacle_distance = float(config.safety.get("fake_obstacle_distance", 20.0))

    def reset(self) -> SensorObservation:
        self.pose = np.array([0.0, 0.0, self.config.goal[2]], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.step_count = 0
        return self.get_observation()

    def get_observation(self) -> SensorObservation:
        return self._observation()

    def step(self, action: Any):
        self.velocity = np.asarray(action, dtype=np.float32).reshape(-1)[:3]
        if self.velocity.size < 3:
            self.velocity = np.pad(self.velocity, (0, 3 - self.velocity.size))
        self.pose = self.pose + self.velocity * float(self.config.control_dt)
        self.step_count += 1
        observation = self._observation()
        return observation, -observation.goal_distance, False, {"step": self.step_count}

    def close(self) -> None:
        self.closed = True

    def _observation(self) -> SensorObservation:
        height = int(self.config.image_height)
        width = int(self.config.image_width)
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[..., 0] = int((self.step_count * 31) % 255)
        image[..., 1] = np.linspace(0, 255, width, dtype=np.uint8)
        depth = np.ones((height, width), dtype=np.float32) * self.obstacle_distance
        lidar = np.array(
            [
                [self.obstacle_distance, 0.0, 0.0],
                [self.obstacle_distance, 1.0, 0.0],
                [self.obstacle_distance, -1.0, 0.0],
            ],
            dtype=np.float32,
        )
        return SensorObservation(
            timestamp=float(self.step_count * self.config.control_dt),
            pose=tuple(float(value) for value in self.pose),
            velocity=tuple(float(value) for value in self.velocity),
            goal_distance=float(np.linalg.norm(self.pose - self.goal)),
            obstacle_distance=self.obstacle_distance,
            image=image,
            lidar=lidar,
            depth=depth,
            metadata={
                "source": "phase6_fake_isaac",
                "runtime_mode": "fake",
                "frame": self.step_count,
            },
        )


class _FakePhase6ROS2Synchronizer:
    """Manual synchronizer facade backed by the fake Isaac environment."""

    def __init__(self, env: _FakePhase6IsaacEnv) -> None:
        self.env = env
        self.closed = False
        self.calls = 0

    def latest_observation(self) -> SensorObservation:
        self.calls += 1
        observation = self.env.get_observation()
        observation.metadata = {
            **dict(observation.metadata),
            "source": "phase6_fake_ros2_sync",
            "sync_packet": self.calls,
        }
        return observation

    def shutdown(self) -> None:
        self.closed = True


class _FakePhase6MAVLinkBridge:
    """Async MAVSDK-like bridge for explicit fake-mode loop validation."""

    def __init__(self) -> None:
        self.command_history: list[dict] = []
        self._connected = False
        self._offboard = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_offboard(self) -> bool:
        return self._offboard

    async def connect(self) -> bool:
        self._connected = True
        self.command_history.append({"action": "connect", "runtime_mode": "fake"})
        return True

    async def wait_until_ready(self, timeout_sec: float = 10.0) -> bool:
        self.command_history.append(
            {"action": "wait_until_ready", "timeout_sec": float(timeout_sec)}
        )
        return True

    async def start_offboard(self, initial_command: ControlCommand) -> bool:
        self._offboard = True
        self.command_history.append(
            {
                "action": "send_initial_setpoint",
                "command": _command_to_dict(initial_command),
            }
        )
        self.command_history.append({"action": "start_offboard"})
        return True

    async def send_command(self, command: ControlCommand) -> bool:
        self.command_history.append(
            {"action": "send_command", "command": _command_to_dict(command)}
        )
        return True

    async def emergency_stop(self) -> bool:
        self.command_history.append({"action": "emergency_stop"})
        self._offboard = False
        return True

    async def disconnect(self) -> None:
        self._connected = False
        self._offboard = False
        self.command_history.append({"action": "disconnect"})


class _Phase6CommandBackend:
    def __init__(self, bridge: Any, config: Phase6GWMSimulationDemoConfig) -> None:
        self.bridge = bridge
        self.config = config
        self.connected = False
        self.commands_sent = 0
        self.emergency_stops = 0

    def connect(self) -> None:
        _run_async(self.bridge.connect())
        self.connected = True

    def wait_until_ready(self) -> None:
        wait_until_ready = getattr(self.bridge, "wait_until_ready")
        _run_async(wait_until_ready(float(self.config.mavlink.get("health_timeout_sec", 10.0))))

    def start_offboard(self, initial_command: ControlCommand) -> None:
        _run_async(self.bridge.start_offboard(initial_command))

    def send_command(self, command: ControlCommand) -> dict:
        _run_async(self.bridge.send_command(command))
        self.commands_sent += 1
        history = list(getattr(self.bridge, "command_history", []))
        return _json_safe(history[-1]) if history else {"action": "send_command"}

    def emergency_stop(self, reason: str) -> dict:
        del reason
        _run_async(self.bridge.emergency_stop())
        self.emergency_stops += 1
        history = list(getattr(self.bridge, "command_history", []))
        return _json_safe(history[-1]) if history else {"action": "emergency_stop"}

    def close(self) -> None:
        if self.connected:
            _run_async(self.bridge.disconnect())
            self.connected = False

    def summary(self) -> dict:
        history = list(getattr(self.bridge, "command_history", []))
        return {
            "type": "mavsdk_px4_sitl",
            "connected": bool(getattr(self.bridge, "is_connected", False)),
            "offboard": bool(getattr(self.bridge, "is_offboard", False)),
            "commands_sent": int(self.commands_sent),
            "emergency_stops": int(self.emergency_stops),
            "command_history_count": len(history),
            "state_coupling": "command_mirror",
            "px4_telemetry_used_for_isaac_state": False,
        }


class _ConfiguredCandidateSampler:
    def __init__(self, sampler: CandidateTrajectorySampler, max_speed: float) -> None:
        self.sampler = sampler
        self.max_speed = float(max_speed)

    def sample(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        num_candidates: int = 8,
    ) -> list[TrajectoryCandidate]:
        return self.sampler.sample(start, goal, num_candidates, max_speed=self.max_speed)


def _build_observation_source(
    config: Phase6GWMSimulationDemoConfig,
    *,
    isaac_env: Any | None,
    ros2_synchronizer: Any | None,
    runtime_invocation: dict,
) -> _Phase6ObservationSource:
    fake_isaac_created = False
    if config.runtime_mode == "fake" and isaac_env is None:
        isaac_env = _FakePhase6IsaacEnv(config)
        runtime_invocation["fake_isaac_env_constructed"] = True
        fake_isaac_created = True

    if isaac_env is None:
        from src.env import IsaacSimNavigationEnv

        isaac_env = IsaacSimNavigationEnv(config=config.isaac)
        runtime_invocation["isaac_env_constructed"] = True
    elif not fake_isaac_created:
        runtime_invocation["isaac_env_injected"] = True

    if config.observation_path == "direct_isaac":
        return _DirectIsaacObservationSource(isaac_env)

    fake_ros2_created = False
    if config.runtime_mode == "fake" and ros2_synchronizer is None:
        ros2_synchronizer = _FakePhase6ROS2Synchronizer(isaac_env)
        runtime_invocation["fake_ros2_synchronizer_constructed"] = True
        fake_ros2_created = True

    if ros2_synchronizer is None:
        from src.ros2_bridge import ROS2SensorSynchronizer

        ros2_synchronizer = ROS2SensorSynchronizer(config=config.ros2)
        ros2_synchronizer.start()
        runtime_invocation["ros2_synchronizer_started"] = True
    elif not fake_ros2_created:
        runtime_invocation["ros2_synchronizer_injected"] = True
    return _ROS2IsaacObservationSource(isaac_env, ros2_synchronizer)


def _build_command_backend(
    config: Phase6GWMSimulationDemoConfig,
    *,
    mavlink_bridge: Any | None,
    runtime_invocation: dict,
) -> _Phase6CommandBackend:
    fake_bridge_created = False
    if config.runtime_mode == "fake" and mavlink_bridge is None:
        mavlink_bridge = _FakePhase6MAVLinkBridge()
        runtime_invocation["fake_mavlink_bridge_constructed"] = True
        fake_bridge_created = True

    if mavlink_bridge is None:
        from src.ros2_bridge import MAVLinkBridge

        mavlink_bridge = MAVLinkBridge(
            config={"deployment": config.deployment, "mavlink": config.mavlink},
            safety_filter=_build_cbf(config),
        )
        runtime_invocation["mavlink_bridge_constructed"] = True
    elif not fake_bridge_created:
        runtime_invocation["mavlink_bridge_injected"] = True
    return _Phase6CommandBackend(mavlink_bridge, config)


def _normalize_config(
    config: dict | Phase6GWMSimulationDemoConfig | None,
) -> Phase6GWMSimulationDemoConfig:
    if isinstance(config, Phase6GWMSimulationDemoConfig):
        return copy.deepcopy(config)
    source = _demo_config_section(config or {})
    deployment = dict(source.get("deployment") or _pure_sim_deployment())
    model = dict(source.get("model") or {})
    scoring = dict(source.get("trajectory_scoring") or {})
    safety = dict(source.get("safety") or {})
    frame_policy = dict(source.get("frame_transform_policy") or {})
    return Phase6GWMSimulationDemoConfig(
        runtime_mode=str(source.get("runtime_mode", "guarded")).lower(),
        steps=int(source.get("steps", 5)),
        horizon=int(source.get("horizon", model.get("horizon", 3))),
        num_candidates=int(source.get("num_candidates", 4)),
        seed=int(source.get("seed", 7)),
        device=str(source.get("device", "cpu")),
        observation_path=str(source.get("observation_path", "direct_isaac")).lower(),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        require_prior_reports=bool(source.get("require_prior_reports", False)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        future_full_state_coupling_requested=bool(
            source.get("future_full_state_coupling_requested", False)
        ),
        px4_launch_requested=bool(source.get("px4_launch_requested", False)),
        planner_interval_steps=int(source.get("planner_interval_steps", 1)),
        control_dt=float(source.get("control_dt", 0.2)),
        context_length=int(source.get("context_length", model.get("context_length", 4))),
        image_height=int(source.get("image_height", model.get("image_height", 32))),
        image_width=int(source.get("image_width", model.get("image_width", 32))),
        max_speed=float(source.get("max_speed", 1.0)),
        goal=_tuple3(source.get("goal", (8.0, 0.0, -5.0))),
        goal_reach_dist=float(source.get("goal_reach_dist", 1.0)),
        collision_dist=float(source.get("collision_dist", 0.5)),
        stale_observation_timeout_sec=float(source.get("stale_observation_timeout_sec", 0.25)),
        stale_command_timeout_sec=float(source.get("stale_command_timeout_sec", 0.2)),
        deployment=deployment,
        model=model,
        scorer_weights=dict(scoring.get("weights") or source.get("scorer_weights") or {}),
        safety=safety,
        mavlink=dict(source.get("mavlink") or {}),
        isaac=dict(source.get("isaac") or {}),
        ros2=dict(source.get("ros2") or {}),
        prior_reports=dict(source.get("prior_reports") or _default_prior_reports()),
        frame_transform_policy=frame_policy,
    )


def _demo_config_section(config: Mapping[str, Any]) -> Dict[str, Any]:
    if "runtime_validation" in config:
        runtime_validation = config.get("runtime_validation") or {}
        if isinstance(runtime_validation, Mapping):
            return dict(runtime_validation.get("phase6_gwm_simulation_demo") or {})
    if "phase6_gwm_simulation_demo" in config:
        return dict(config.get("phase6_gwm_simulation_demo") or {})
    return dict(config)


def _runtime_readiness(config: Phase6GWMSimulationDemoConfig) -> Phase6RuntimeReadiness:
    required_reports = _required_report_paths(config)
    reports: dict[str, dict[str, Any]] = {}
    for name, path_text in required_reports.items():
        path = Path(path_text)
        entry: dict[str, Any] = {
            "path": path_text,
            "exists": path.exists(),
            "status": None,
            "ready": False,
            "reason": None,
        }
        if path.exists():
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                status = payload.get("status")
                entry["status"] = status
                entry["ready"] = status in {"passed", "ready"}
                if not entry["ready"]:
                    entry["reason"] = f"report status is {status!r}"
            except Exception as exc:
                entry["reason"] = f"could not read report: {exc}"
        else:
            entry["reason"] = "report missing"
        reports[name] = entry
    return Phase6RuntimeReadiness(
        required_reports=required_reports,
        report_readiness={
            "required": bool(config.require_prior_reports),
            "all_ready": all(bool(report["ready"]) for report in reports.values()),
            "reports": reports,
        },
        env_gates=_runtime_gates(),
    )


def _required_report_paths(config: Phase6GWMSimulationDemoConfig) -> dict[str, str]:
    names = ["isaac_sensor_runtime", "px4_sitl_command_validation", "isaac_px4_bridge_design"]
    if config.observation_path == "ros2":
        names.insert(1, "ros2_sim_sensor_bridge")
    return {name: config.prior_reports[name] for name in names if name in config.prior_reports}


def _runtime_gates() -> dict:
    return {
        "common": _env_group(COMMON_ENV_GATES),
        "isaac": _env_group(ISAAC_ENV_GATES),
        "ros2": _env_group(ROS2_ENV_GATES),
        "mavsdk_px4_sitl": _env_group(MAVSDK_PX4_ENV_GATES),
        "optional": _env_group(OPTIONAL_ENV_GATES, required=False),
    }


def _env_group(names: tuple[str, ...], required: bool = True) -> dict:
    return {
        name: {
            "present": name in os.environ,
            "enabled": os.environ.get(name) == "1",
            "required": required,
        }
        for name in names
    }


def _missing_env_gates_for_path(config: Phase6GWMSimulationDemoConfig) -> list[str]:
    names = set(ISAAC_ENV_GATES + MAVSDK_PX4_ENV_GATES)
    if config.observation_path == "ros2":
        names.update(ROS2_ENV_GATES)
    return sorted(name for name in names if os.environ.get(name) != "1")


def _all_runtime_injected_for_path(config: Phase6GWMSimulationDemoConfig, injected: Mapping[str, bool]) -> bool:
    if config.runtime_mode == "fake":
        return True
    if not injected.get("isaac_env", False) or not injected.get("mavlink_bridge", False):
        return False
    if config.observation_path == "ros2" and not injected.get("ros2_synchronizer", False):
        return False
    return True


def _runtime_unavailable_reason(config: Phase6GWMSimulationDemoConfig) -> str | None:
    from src.digital_twin import IsaacSimRuntime
    from src.ros2_bridge import ROS2SensorSynchronizer
    import src.ros2_bridge.mavlink_bridge as mavlink_module

    missing = []
    if not IsaacSimRuntime.is_available():
        missing.append("Isaac Sim / Isaac Lab Python runtime is unavailable")
    if config.observation_path == "ros2" and not ROS2SensorSynchronizer.is_available():
        missing.append("ROS2 sensor synchronization runtime is unavailable")
    if mavlink_module._load_mavsdk_system() is None:
        missing.append("MAVSDK Python runtime is unavailable")
    if missing:
        return "; ".join(missing)
    return None


def _configuration_refusal(config: Phase6GWMSimulationDemoConfig) -> str | None:
    if config.runtime_mode not in {"guarded", "fake"}:
        return "Phase 6-F runtime_mode must be 'guarded' or 'fake'."
    if config.observation_path not in {"direct_isaac", "ros2"}:
        return "Phase 6-F observation_path must be 'direct_isaac' or 'ros2'."
    if config.device != "cpu" and not torch.cuda.is_available():
        return "Phase 6-F non-CPU device requested, but CUDA is unavailable."
    deployment = config.deployment
    if bool(deployment.get("real_hardware_enabled", False)):
        return "Phase 6-F refuses real_hardware_enabled=True."
    if bool(deployment.get("autonomous_real_flight_enabled", False)):
        return "Phase 6-F refuses autonomous_real_flight_enabled=True."
    if bool(config.mavlink.get("real_hardware_enabled", False)):
        return "Phase 6-F refuses MAVLink real_hardware_enabled=True."
    if bool(config.mavlink.get("autonomous_real_flight_enabled", False)):
        return "Phase 6-F refuses MAVLink autonomous_real_flight_enabled=True."
    if bool(config.px4_launch_requested) or os.environ.get("GWM_ALLOW_PX4_LAUNCH") == "1":
        return "Phase 6-F refuses PX4 launch; PX4 SITL must be started externally."
    if bool(deployment.get("mock", False)):
        return "Phase 6-F live simulation demo requires deployment.mock=False."
    if not bool(deployment.get("sitl_enabled", False)):
        return "Phase 6-F live simulation demo requires deployment.sitl_enabled=True."
    return None


def _base_payload(
    config: Phase6GWMSimulationDemoConfig,
    readiness: Phase6RuntimeReadiness,
    timings: dict,
    injected: Mapping[str, bool],
) -> dict:
    readiness_payload = readiness.to_dict()
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "skipped",
        "reason": None,
        "runtime_gates": readiness_payload["runtime_gates"],
        "required_reports": readiness_payload["required_reports"],
        "report_readiness": readiness_payload["report_readiness"],
        "runtime_invocation_summary": {
            "isaac_env_injected": bool(injected.get("isaac_env", False)),
            "ros2_synchronizer_injected": bool(injected.get("ros2_synchronizer", False)),
            "mavlink_bridge_injected": bool(injected.get("mavlink_bridge", False)),
            "isaac_env_constructed": False,
            "ros2_synchronizer_started": False,
            "mavlink_bridge_constructed": False,
            "fake_isaac_env_constructed": False,
            "fake_ros2_synchronizer_constructed": False,
            "fake_mavlink_bridge_constructed": False,
            "px4_launch_attempted": False,
            "hardware_check_run": False,
        },
        "config_summary": {
            "runtime_mode": config.runtime_mode,
            "steps": int(config.steps),
            "horizon": int(config.horizon),
            "num_candidates": int(config.num_candidates),
            "seed": int(config.seed),
            "observation_path": config.observation_path,
            "planner_interval_steps": int(config.planner_interval_steps),
            "output_path": config.output_path,
            "write_output": bool(config.write_output),
        },
        "loop_summary": {},
        "bridge_summary": {},
        "coordinate_frame_summary": _coordinate_frame_summary(config),
        "steps": [],
        "metrics": _empty_metrics("skipped"),
        "safety_summary": {
            "simulation_only": True,
            "real_hardware_enabled": bool(config.deployment.get("real_hardware_enabled", False)),
            "autonomous_real_flight_enabled": bool(
                config.deployment.get("autonomous_real_flight_enabled", False)
            ),
            "cbf_required": True,
            "cbf_applied_before_every_mavsdk_write": False,
            "emergency_stop_attempted": False,
            "px4_launch_attempted": False,
        },
        "backend_summary": {},
        "timings": timings,
        "errors": [],
        "closed": {"observation_source": False, "mavsdk_bridge": False},
    }


def _coordinate_frame_summary(config: Phase6GWMSimulationDemoConfig) -> dict:
    policy = dict(config.frame_transform_policy)
    return {
        "project_frame": policy.get("project_frame", "project_default"),
        "isaac_world_frame": policy.get("isaac_world_frame", "isaac_z_up"),
        "px4_world_frame": policy.get("px4_world_frame", "px4_ned"),
        "mavsdk_command_frame": policy.get("mavsdk_command_frame", "px4_body_ned"),
        "coordinate_conversion_applied": bool(policy.get("coordinate_conversion_applied", False)),
        "transforms_defined": bool(policy.get("transforms_defined", False)),
        "state_coupling": "command_mirror",
        "px4_telemetry_used_for_isaac_state": False,
    }


def _gwm_config(config: Phase6GWMSimulationDemoConfig) -> GWMConfig:
    model = dict(config.model)
    model.update(
        {
            "image_height": config.image_height,
            "image_width": config.image_width,
            "context_length": config.context_length,
            "horizon": config.horizon,
        }
    )
    return GWMConfig.from_any(model)


def _build_cbf(config: Phase6GWMSimulationDemoConfig) -> ControlBarrierFunction:
    limits_config = dict(config.safety.get("limits") or config.safety)
    velocity = dict(limits_config.get("velocity_limits") or {})
    altitude = dict(limits_config.get("altitude_bounds") or {})
    geofence_config = dict(limits_config.get("geofence") or {})
    geofence = None
    if geofence_config.get("enabled", False):
        geofence = {
            axis: geofence_config[axis]
            for axis in ("x", "y", "z")
            if axis in geofence_config
        }
    return ControlBarrierFunction(
        limits=SafetyLimits(
            max_vx=float(velocity.get("max_vx", 4.0)),
            max_vy=float(velocity.get("max_vy", 4.0)),
            max_vz=float(velocity.get("max_vz", 2.0)),
            max_yaw_rate=float(velocity.get("max_yaw_rate", 1.0)),
            min_altitude=float(altitude.get("min_altitude", 0.5)),
            max_altitude=float(altitude.get("max_altitude", 120.0)),
            geofence=geofence,
        ),
        min_obstacle_distance=float(
            config.safety.get("min_obstacle_distance", config.safety.get("cbf", {}).get("min_obstacle_distance", 4.0))
        ),
        alpha=float(config.safety.get("alpha", config.safety.get("cbf", {}).get("alpha", 1.0))),
    )


def _safety_context(cbf: ControlBarrierFunction) -> dict:
    return {
        "min_safe_depth": cbf.min_obstacle_distance,
        "altitude_bounds": (cbf.limits.min_altitude, cbf.limits.max_altitude),
        "geofence": cbf.limits.geofence,
    }


def _apply_safety_gate(
    cbf: ControlBarrierFunction,
    config: Phase6GWMSimulationDemoConfig,
    observation: SensorObservation,
    command: ControlCommand,
) -> tuple[ControlCommand, dict]:
    saturated = cbf.saturate(command)
    altitude_ok = cbf.within_altitude_bounds(observation)
    geofence_ok = cbf.within_geofence(observation)
    obstacle = {
        "position": (
            float(observation.pose[0]) + float(observation.obstacle_distance),
            float(observation.pose[1]),
            float(observation.pose[2]),
        )
    }
    filtered = cbf.filter_action(observation, obstacle, saturated)
    if not altitude_ok:
        filtered = _safety_override(filtered, "phase6_altitude_refusal")
    elif not geofence_ok:
        filtered = _safety_override(filtered, "phase6_geofence_refusal")
    metadata = dict(filtered.metadata)
    metadata.update(
        {
            "phase6_safety_gate": True,
            "altitude_ok": bool(altitude_ok),
            "geofence_ok": bool(geofence_ok),
            "state_coupling": "command_mirror",
        }
    )
    filtered = ControlCommand(
        vx=filtered.vx,
        vy=filtered.vy,
        vz=filtered.vz,
        yaw_rate=filtered.yaw_rate,
        duration=filtered.duration,
        mode=filtered.mode,
        metadata=metadata,
    )
    return filtered, {
        "raw": _command_to_dict(command),
        "safe": _command_to_dict(filtered),
        "saturated": bool(saturated.metadata.get("saturated", False)),
        "altitude_ok": bool(altitude_ok),
        "geofence_ok": bool(geofence_ok),
        "obstacle_filter_applied": filtered.mode == ControlMode.SAFETY_OVERRIDE,
    }


def _command_from_candidate(
    candidate: TrajectoryCandidate,
    *,
    duration: float,
    step_index: int,
    score: Mapping[str, Any],
) -> ControlCommand:
    actions = np.asarray(candidate.actions, dtype=np.float32)
    if actions.ndim != 2 or actions.shape[1] < 3:
        raise RuntimeError("Phase 6-F candidate actions must have shape [H, 3].")
    action = actions[0, :3]
    return ControlCommand(
        vx=float(action[0]),
        vy=float(action[1]),
        vz=float(action[2]),
        yaw_rate=0.0,
        duration=float(duration),
        mode=ControlMode.WORLD_MODEL_GUIDED,
        metadata={
            "source": "phase6_gwm_simulation_demo",
            "step": int(step_index),
            "candidate_index": int(candidate.metadata.get("candidate_index", -1)),
            "selected_score": float(score["total_score"]),
        },
    )


def _zero_command(config: Phase6GWMSimulationDemoConfig, reason: str) -> ControlCommand:
    return ControlCommand(
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw_rate=0.0,
        duration=float(config.control_dt),
        mode=ControlMode.WORLD_MODEL_GUIDED,
        metadata={"source": "phase6_gwm_simulation_demo", "reason": reason},
    )


def _safety_override(command: ControlCommand, reason: str) -> ControlCommand:
    metadata = dict(command.metadata)
    metadata["reason"] = reason
    return ControlCommand(
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw_rate=0.0,
        duration=command.duration,
        mode=ControlMode.SAFETY_OVERRIDE,
        metadata=metadata,
    )


def _observation_stale(
    observation: SensorObservation,
    config: Phase6GWMSimulationDemoConfig,
    step_index: int,
) -> bool:
    expected = step_index * config.control_dt
    age = expected - float(observation.timestamp)
    return bool(age > config.stale_observation_timeout_sec)


def _step_record(
    *,
    config: Phase6GWMSimulationDemoConfig,
    step_index: int,
    observation: SensorObservation,
    raw_command: ControlCommand,
    safe_command: ControlCommand,
    safety_decision: Mapping[str, Any],
    score: Mapping[str, Any],
    backend_result: Mapping[str, Any],
    planner_updated: bool,
) -> dict:
    return {
        "step": int(step_index),
        "timestamp": float(observation.timestamp),
        "pose": [float(value) for value in observation.pose],
        "velocity": [float(value) for value in observation.velocity],
        "goal_distance": float(observation.goal_distance),
        "obstacle_distance": float(observation.obstacle_distance),
        "goal_reached": float(observation.goal_distance) <= float(config.goal_reach_dist),
        "collision_detected": float(observation.obstacle_distance) < float(config.collision_dist),
        "planner_updated": bool(planner_updated),
        "selected_score": float(score.get("total_score", 0.0)),
        "score_components": {
            str(key): float(value) for key, value in dict(score.get("components", {})).items()
        },
        "raw_command": _command_to_dict(raw_command),
        "safe_command": _command_to_dict(safe_command),
        "safety_decision": _json_safe(safety_decision),
        "backend_result": _json_safe(backend_result),
    }


def _metrics_from_steps(
    steps: list[dict],
    status: str,
    *,
    planner_updates: int,
    stale_observations: int,
    stale_commands: int,
    command_backend: _Phase6CommandBackend,
    observation_source: _Phase6ObservationSource,
) -> dict:
    total_scores = [float(step["selected_score"]) for step in steps]
    uncertainties = [
        float(step["score_components"].get("uncertainty", 0.0)) for step in steps
    ]
    obstacle_distances = [float(step["obstacle_distance"]) for step in steps]
    return {
        "steps": len(steps),
        "observations": len(steps),
        "planner_updates": int(planner_updates),
        "commands_sent": int(command_backend.commands_sent),
        "safety_overrides": sum(
            1 for step in steps if step["safe_command"]["mode"] == ControlMode.SAFETY_OVERRIDE.value
        ),
        "emergency_stops": int(command_backend.emergency_stops),
        "stale_observations": int(stale_observations),
        "stale_commands": int(stale_commands),
        "mean_score": float(np.mean(total_scores)) if total_scores else 0.0,
        "max_uncertainty": float(np.max(uncertainties)) if uncertainties else 0.0,
        "min_obstacle_distance": float(np.min(obstacle_distances)) if obstacle_distances else 0.0,
        "mavsdk_command_history_count": int(command_backend.summary()["command_history_count"]),
        "isaac_frames_stepped": int(observation_source.summary().get("isaac_frames_stepped", 0)),
        "final_status": status,
    }


def _empty_metrics(status: str) -> dict:
    return {
        "steps": 0,
        "observations": 0,
        "planner_updates": 0,
        "commands_sent": 0,
        "safety_overrides": 0,
        "emergency_stops": 0,
        "stale_observations": 0,
        "stale_commands": 0,
        "mean_score": 0.0,
        "max_uncertainty": 0.0,
        "min_obstacle_distance": 0.0,
        "mavsdk_command_history_count": 0,
        "isaac_frames_stepped": 0,
        "final_status": status,
    }


def _finalize_payload(
    payload: dict,
    config: Phase6GWMSimulationDemoConfig,
    start: float,
) -> dict:
    payload["timings"]["total_sec"] = round(time.perf_counter() - start, 6)
    if not payload.get("metrics"):
        payload["metrics"] = _empty_metrics(str(payload.get("status", "unknown")))
    result = _json_safe(payload)
    if config.write_output:
        output_path = Path(config.output_path or DEFAULT_OUTPUT_PATH)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def _run_async(value: Any) -> Any:
    if hasattr(value, "__await__"):
        return asyncio.run(value)
    return value


def _tuple3(value: Any) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=np.float32).reshape(-1)
    if array.size < 3:
        raise ValueError("Expected at least three coordinates.")
    return (float(array[0]), float(array[1]), float(array[2]))


def _command_to_dict(command: ControlCommand) -> dict:
    return {
        "vx": float(command.vx),
        "vy": float(command.vy),
        "vz": float(command.vz),
        "yaw_rate": float(command.yaw_rate),
        "duration": float(command.duration),
        "mode": command.mode.value,
        "metadata": _json_safe(command.metadata),
    }


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value
