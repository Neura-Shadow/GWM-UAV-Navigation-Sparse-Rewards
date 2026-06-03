"""End-to-end mock-first Generated World Model navigation demo.

The demo wires the Phase 4-A through 4-E interfaces together without enabling
real flight. The default path is synthetic observations, a lightweight GWM
rollout, CBF command filtering, and a mock execution backend.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import numpy as np
import torch
import yaml

from src.control.barrier_certificate import ControlBarrierFunction, SafetyLimits
from src.generated_world_model.observation_buffer import ObservationBuffer
from src.generated_world_model.planner import GeneratedWorldModelPlanner
from src.generated_world_model.rollout import AutoregressiveRollout
from src.generated_world_model.training import build_baseline_components
from src.generated_world_model.trajectory_sampler import CandidateTrajectorySampler
from src.generated_world_model.trajectory_scorer import TrajectoryScorer
from src.generated_world_model.types import GWMConfig, TrajectoryCandidate
from src.utils.data_types import ControlCommand, ControlMode, SensorObservation


_SCHEMA_VERSION = "gwm_navigation_demo_v1"
_CONFIG_SCHEMA_VERSION = "gwm_navigation_demo_config_v1"
_DEFAULT_DEPLOYMENT = {
    "mock": True,
    "sitl_enabled": False,
    "real_hardware_enabled": False,
    "autonomous_real_flight_enabled": False,
}


@dataclass
class GWMDemoConfig:
    """Configuration for the mock-first end-to-end GWM navigation demo."""

    observation_source: str = "mock"
    execution_backend: str = "mock"
    steps: int = 5
    horizon: int = 3
    num_candidates: int = 4
    seed: int = 7
    device: str = "cpu"
    checkpoint_path: Optional[str] = None
    output_path: Optional[str] = "outputs/gwm_demo/latest.json"
    write_output: bool = True
    allow_optional_runtime: bool = False
    fail_on_runtime_unavailable: bool = False
    start_pose: tuple[float, float, float] = (0.0, 0.0, -5.0)
    goal: tuple[float, float, float] = (10.0, 0.0, -5.0)
    control_dt: float = 0.4
    max_speed: float = 2.0
    context_length: int = 4
    image_height: int = 32
    image_width: int = 32
    goal_reach_dist: float = 1.0
    collision_dist: float = 0.5
    mock_obstacle_distance: float = 20.0
    min_safe_depth: float = 1.0
    min_obstacle_distance: float = 4.0
    model: Dict[str, Any] = field(default_factory=dict)
    scorer_weights: Dict[str, float] = field(default_factory=dict)
    safety: Dict[str, Any] = field(default_factory=dict)
    deployment: Dict[str, Any] = field(default_factory=lambda: dict(_DEFAULT_DEPLOYMENT))
    mavlink: Dict[str, Any] = field(default_factory=dict)
    ros2: Dict[str, Any] = field(default_factory=dict)
    isaac: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_any(cls, value: "GWMDemoConfig | Mapping[str, Any] | str | Path | None") -> "GWMDemoConfig":
        """Create a demo config from an object, dict, YAML path, or defaults."""
        if value is None:
            return cls()
        if isinstance(value, cls):
            return value
        source = load_demo_config(value) if isinstance(value, (str, Path)) else dict(value)
        demo = dict(source.get("demo") or source)

        deployment = dict(_DEFAULT_DEPLOYMENT)
        deployment.update(source.get("deployment") or demo.get("deployment") or {})

        model = dict(source.get("model") or {})
        model.update(demo.get("model") or {})

        scoring = dict(source.get("trajectory_scoring") or {})
        scorer_weights = dict(scoring.get("weights") or {})
        scorer_weights.update(demo.get("scorer_weights") or {})

        safety = dict(source.get("safety") or {})
        safety.update(demo.get("safety") or {})

        return cls(
            observation_source=str(demo.get("observation_source", "mock")).lower(),
            execution_backend=str(
                demo.get("execution_backend", demo.get("backend", "mock"))
            ).lower(),
            steps=int(demo.get("steps", 5)),
            horizon=int(demo.get("horizon", model.get("horizon", 3))),
            num_candidates=int(demo.get("num_candidates", 4)),
            seed=int(demo.get("seed", 7)),
            device=str(demo.get("device", "cpu")),
            checkpoint_path=_optional_str(demo.get("checkpoint_path")),
            output_path=_optional_str(demo.get("output_path", "outputs/gwm_demo/latest.json")),
            write_output=bool(demo.get("write_output", True)),
            allow_optional_runtime=bool(demo.get("allow_optional_runtime", False)),
            fail_on_runtime_unavailable=bool(demo.get("fail_on_runtime_unavailable", False)),
            start_pose=_tuple3(demo.get("start_pose", (0.0, 0.0, -5.0))),
            goal=_tuple3(demo.get("goal", (10.0, 0.0, -5.0))),
            control_dt=float(demo.get("control_dt", 0.4)),
            max_speed=float(demo.get("max_speed", 2.0)),
            context_length=int(demo.get("context_length", model.get("context_length", 4))),
            image_height=int(demo.get("image_height", model.get("image_height", 32))),
            image_width=int(demo.get("image_width", model.get("image_width", 32))),
            goal_reach_dist=float(demo.get("goal_reach_dist", 1.0)),
            collision_dist=float(demo.get("collision_dist", 0.5)),
            mock_obstacle_distance=float(demo.get("mock_obstacle_distance", 20.0)),
            min_safe_depth=float(demo.get("min_safe_depth", 1.0)),
            min_obstacle_distance=float(
                demo.get(
                    "min_obstacle_distance",
                    _nested(safety, ("cbf", "min_obstacle_distance"), 4.0),
                )
            ),
            model=model,
            scorer_weights=scorer_weights,
            safety=safety,
            deployment=deployment,
            mavlink=dict(source.get("mavlink") or demo.get("mavlink") or {}),
            ros2=dict(source.get("ros2") or demo.get("ros2") or {}),
            isaac=dict(source.get("isaac") or demo.get("isaac") or {}),
        )

    def to_summary(self) -> Dict[str, Any]:
        """Return a JSON-safe config summary."""
        return {
            "observation_source": self.observation_source,
            "execution_backend": self.execution_backend,
            "steps": self.steps,
            "horizon": self.horizon,
            "num_candidates": self.num_candidates,
            "seed": self.seed,
            "device": self.device,
            "output_path": self.output_path,
            "goal": list(self.goal),
            "control_dt": self.control_dt,
            "allow_optional_runtime": self.allow_optional_runtime,
            "write_output": self.write_output,
        }

    def gwm_config(self) -> GWMConfig:
        """Return the GWM model dimensions used by the demo."""
        model = dict(self.model)
        model.update(
            {
                "image_height": self.image_height,
                "image_width": self.image_width,
                "context_length": self.context_length,
                "horizon": self.horizon,
            }
        )
        return GWMConfig.from_any(model)


@dataclass
class GWMDemoResult:
    """Serializable result of one demo run."""

    payload: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        """Return the JSON-safe result payload."""
        return dict(self.payload)


class GWMDemoRunner:
    """Run the Phase 4-F mock-first GWM navigation demo."""

    def __init__(self, config: GWMDemoConfig | Mapping[str, Any] | str | Path | None = None) -> None:
        self.config = GWMDemoConfig.from_any(config)
        self._validate_config()
        self.device = torch.device(self.config.device)
        self.gwm_config = self.config.gwm_config()
        torch.manual_seed(self.config.seed)
        np.random.seed(self.config.seed)

        self.encoder, self.conditioner, self.model = build_baseline_components(self.gwm_config)
        self.encoder.to(self.device).eval()
        self.conditioner.to(self.device).eval()
        self.model.to(self.device).eval()
        self._load_checkpoint_if_requested()

        rollout = AutoregressiveRollout(
            model=self.model,
            encoder=self.encoder,
            conditioner=self.conditioner,
            mode="autoregressive",
        )
        sampler = CandidateTrajectorySampler(
            horizon=self.config.horizon,
            dt=self.config.control_dt,
            seed=self.config.seed,
        )
        scorer = TrajectoryScorer(weights=self.config.scorer_weights)
        self.planner = GeneratedWorldModelPlanner(
            rollout=rollout,
            scorer=scorer,
            sampler=_ConfiguredCandidateSampler(sampler, max_speed=self.config.max_speed),
        )
        self.buffer = ObservationBuffer(
            context_length=self.config.context_length,
            image_size=(self.config.image_height, self.config.image_width),
        )
        self.cbf = ControlBarrierFunction(
            limits=_safety_limits_from_config(self.config.safety),
            min_obstacle_distance=self.config.min_obstacle_distance,
            alpha=float(_nested(self.config.safety, ("cbf", "alpha"), 1.0)),
        )

    def run(self) -> Dict[str, Any]:
        """Execute the demo and return a JSON-safe result dictionary."""
        started_at = time.time()
        run_id = f"gwm-demo-{uuid.uuid4().hex[:12]}"
        unavailable_reason = self._runtime_unavailable_reason()
        if unavailable_reason is not None:
            payload = self._base_payload(run_id, started_at)
            payload["final_status"] = "runtime_unavailable"
            payload["backend_summary"]["runtime_unavailable_reason"] = unavailable_reason
            payload["finished_at"] = time.time()
            payload["metrics"] = _metrics_from_steps([], "runtime_unavailable")
            self._maybe_write_output(payload)
            if self.config.fail_on_runtime_unavailable:
                raise RuntimeError(unavailable_reason)
            return payload

        payload = self._base_payload(run_id, started_at)
        try:
            observation_source = self._build_observation_source()
            execution_backend = self._build_execution_backend()
        except RuntimeError as exc:
            if self.config.fail_on_runtime_unavailable:
                raise
            payload["final_status"] = "runtime_unavailable"
            payload["backend_summary"]["runtime_unavailable_reason"] = str(exc)
            payload["finished_at"] = time.time()
            payload["metrics"] = _metrics_from_steps([], "runtime_unavailable")
            self._maybe_write_output(payload)
            return payload

        payload["backend_summary"].update(execution_backend.summary())
        steps: list[Dict[str, Any]] = []
        final_status = "completed"

        try:
            current_observation = observation_source.reset()
            self.buffer.clear()
            self.buffer.append(current_observation)
            while not self.buffer.is_ready:
                current_observation = observation_source.observe()
                self.buffer.append(current_observation)

            for step_index in range(self.config.steps):
                if self._goal_reached(current_observation):
                    final_status = "completed"
                    break
                if current_observation.obstacle_distance < self.config.collision_dist:
                    final_status = "safety_stop"
                    break

                context = self.buffer.as_observation_batch().to(self.device)
                safety_context = self._safety_context()
                with torch.no_grad():
                    plan = self.planner.plan(
                        context=context,
                        start=current_observation.pose,
                        goal=self.config.goal,
                        safety_context=safety_context,
                        num_candidates=self.config.num_candidates,
                    )

                candidate = plan["candidate"]
                raw_command = _command_from_candidate(
                    candidate,
                    duration=self.config.control_dt,
                    step_index=step_index,
                    score=plan["score"],
                )
                safe_command = self._apply_safety_gate(current_observation, raw_command)
                backend_result = execution_backend.execute(safe_command)
                next_observation = observation_source.apply_command(safe_command)
                self.buffer.append(next_observation)

                step_record = self._step_record(
                    step_index=step_index,
                    observation=next_observation,
                    plan=plan,
                    raw_command=raw_command,
                    safe_command=safe_command,
                    backend_result=backend_result,
                )
                steps.append(step_record)
                current_observation = next_observation

                if step_record["goal_reached"]:
                    final_status = "completed"
                    break
                if step_record["collision_detected"]:
                    final_status = "safety_stop"
                    break
            else:
                final_status = "timeout"
        except Exception as exc:
            final_status = "safety_stop"
            payload["backend_summary"]["error"] = str(exc)
            if hasattr(execution_backend, "emergency_stop"):
                execution_backend.emergency_stop(str(exc))
        finally:
            execution_backend.close()
            observation_source.close()

        payload["steps"] = steps
        payload["final_status"] = final_status
        payload["metrics"] = _metrics_from_steps(steps, final_status)
        payload["backend_summary"].update(execution_backend.summary())
        payload["finished_at"] = time.time()
        self._maybe_write_output(payload)
        return GWMDemoResult(payload).to_dict()

    def _validate_config(self) -> None:
        if self.config.observation_source not in {"mock", "isaac", "ros2"}:
            raise ValueError("observation_source must be one of: mock, isaac, ros2.")
        if self.config.execution_backend not in {"mock", "isaac", "mavsdk_sitl"}:
            raise ValueError("execution_backend must be one of: mock, isaac, mavsdk_sitl.")
        if self.config.steps <= 0:
            raise ValueError("steps must be positive.")
        if self.config.horizon <= 0:
            raise ValueError("horizon must be positive.")
        if self.config.num_candidates <= 0:
            raise ValueError("num_candidates must be positive.")
        if self.config.device != "cpu" and not torch.cuda.is_available():
            raise RuntimeError("Non-CPU demo device requested, but CUDA is unavailable.")
        _reject_real_flight_flags(self.config.deployment)
        _reject_real_flight_flags(self.config.mavlink)
        if self.config.execution_backend == "mavsdk_sitl":
            if not self.config.allow_optional_runtime:
                raise RuntimeError("mavsdk_sitl execution requires allow_optional_runtime=True.")
            if bool(self.config.deployment.get("mock", True)):
                raise RuntimeError("mavsdk_sitl execution requires deployment.mock=False.")
            if not bool(self.config.deployment.get("sitl_enabled", False)):
                raise RuntimeError("mavsdk_sitl execution requires deployment.sitl_enabled=True.")

    def _runtime_unavailable_reason(self) -> Optional[str]:
        optional_sources = self.config.observation_source != "mock"
        optional_execution = self.config.execution_backend != "mock"
        if (optional_sources or optional_execution) and not self.config.allow_optional_runtime:
            return "Optional runtime paths require allow_optional_runtime=True."

        if self.config.observation_source == "isaac" or self.config.execution_backend == "isaac":
            from src.digital_twin import IsaacSimRuntime

            if not IsaacSimRuntime.is_available():
                return "Isaac Sim runtime is unavailable."

        if self.config.observation_source == "ros2":
            from src.ros2_bridge import ROS2SensorSynchronizer

            if not ROS2SensorSynchronizer.is_available():
                return "ROS2 sensor synchronization modules are unavailable."

        return None

    def _build_observation_source(self) -> "_ObservationSource":
        if self.config.observation_source == "mock":
            return _MockObservationSource(self.config)
        if self.config.observation_source == "isaac":
            return _IsaacObservationSource(self.config)
        return _ROS2ObservationSource(self.config)

    def _build_execution_backend(self) -> "_ExecutionBackend":
        if self.config.execution_backend == "mock":
            return _MockExecutionBackend()
        if self.config.execution_backend == "isaac":
            return _IsaacExecutionBackend(self.config)
        return _MAVSDKSITLExecutionBackend(self.config, self.cbf)

    def _load_checkpoint_if_requested(self) -> None:
        path = self.config.checkpoint_path
        if not path:
            return
        checkpoint_path = Path(path)
        if not checkpoint_path.exists():
            raise RuntimeError(f"GWM demo checkpoint does not exist: {checkpoint_path}")
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        if not isinstance(checkpoint, Mapping):
            raise RuntimeError("GWM demo checkpoint must contain a mapping of state dicts.")
        if "encoder" in checkpoint:
            self.encoder.load_state_dict(checkpoint["encoder"])
        if "conditioner" in checkpoint:
            self.conditioner.load_state_dict(checkpoint["conditioner"])
        if "model" in checkpoint:
            self.model.load_state_dict(checkpoint["model"])

    def _base_payload(self, run_id: str, started_at: float) -> Dict[str, Any]:
        return {
            "schema_version": _SCHEMA_VERSION,
            "run_id": run_id,
            "seed": self.config.seed,
            "started_at": started_at,
            "finished_at": None,
            "config_summary": self.config.to_summary(),
            "runtime_boundaries": {
                "isaac_sim_required": False,
                "ros2_required": False,
                "mavsdk_required": False,
                "px4_required": False,
                "real_hardware_enabled": bool(
                    self.config.deployment.get("real_hardware_enabled", False)
                ),
                "autonomous_real_flight_enabled": bool(
                    self.config.deployment.get("autonomous_real_flight_enabled", False)
                ),
            },
            "backend_summary": {
                "observation_source": self.config.observation_source,
                "execution_backend": self.config.execution_backend,
                "mock_default": self.config.observation_source == "mock"
                and self.config.execution_backend == "mock",
            },
            "steps": [],
            "metrics": {},
            "final_status": "unknown",
        }

    def _safety_context(self) -> Dict[str, Any]:
        return {
            "min_safe_depth": self.config.min_safe_depth,
            "altitude_bounds": (
                self.cbf.limits.min_altitude,
                self.cbf.limits.max_altitude,
            ),
            "geofence": self.cbf.limits.geofence,
        }

    def _apply_safety_gate(
        self,
        observation: SensorObservation,
        raw_command: ControlCommand,
    ) -> ControlCommand:
        saturated = self.cbf.saturate(raw_command)
        if not self.cbf.within_altitude_bounds(observation):
            return _safety_override(saturated, "altitude_bounds")
        if not self.cbf.within_geofence(observation):
            return _safety_override(saturated, "geofence")
        obstacle = {
            "position": (
                float(observation.pose[0]) + float(observation.obstacle_distance),
                float(observation.pose[1]),
                float(observation.pose[2]),
            )
        }
        return self.cbf.filter_action(observation, obstacle, saturated)

    def _step_record(
        self,
        step_index: int,
        observation: SensorObservation,
        plan: Dict[str, Any],
        raw_command: ControlCommand,
        safe_command: ControlCommand,
        backend_result: Dict[str, Any],
    ) -> Dict[str, Any]:
        candidate = plan["candidate"]
        score = plan["score"]
        return {
            "step": step_index,
            "timestamp": float(observation.timestamp),
            "pose": [float(v) for v in observation.pose],
            "goal_distance": float(observation.goal_distance),
            "obstacle_distance": float(observation.obstacle_distance),
            "goal_reached": self._goal_reached(observation),
            "collision_detected": observation.obstacle_distance < self.config.collision_dist,
            "candidate_count": self.config.num_candidates,
            "selected_candidate_index": int(candidate.metadata.get("candidate_index", -1)),
            "selected_score": float(score["total_score"]),
            "score_components": {
                key: float(value) for key, value in score.get("components", {}).items()
            },
            "raw_command": _command_to_dict(raw_command),
            "safe_command": _command_to_dict(safe_command),
            "safety_metadata": dict(safe_command.metadata),
            "backend_result": backend_result,
        }

    def _goal_reached(self, observation: SensorObservation) -> bool:
        return bool(observation.goal_distance <= self.config.goal_reach_dist)

    def _maybe_write_output(self, payload: Dict[str, Any]) -> None:
        if not self.config.write_output or not self.config.output_path:
            return
        output_path = Path(self.config.output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)


class _ObservationSource:
    def reset(self) -> SensorObservation:
        raise NotImplementedError

    def observe(self) -> SensorObservation:
        raise NotImplementedError

    def apply_command(self, command: ControlCommand) -> SensorObservation:
        raise NotImplementedError

    def close(self) -> None:
        return None


class _MockObservationSource(_ObservationSource):
    def __init__(self, config: GWMDemoConfig) -> None:
        self.config = config
        self.pose = np.asarray(config.start_pose, dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.step_count = 0

    def reset(self) -> SensorObservation:
        self.pose = np.asarray(self.config.start_pose, dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.step_count = 0
        return self.observe()

    def observe(self) -> SensorObservation:
        goal_distance = float(np.linalg.norm(self.pose - np.asarray(self.config.goal)))
        depth = np.full(
            (self.config.image_height, self.config.image_width),
            self.config.mock_obstacle_distance,
            dtype=np.float32,
        )
        image = np.zeros(
            (self.config.image_height, self.config.image_width, 3),
            dtype=np.uint8,
        )
        image[..., 0] = int((self.step_count * 17) % 255)
        image[..., 1] = np.linspace(0, 255, self.config.image_width, dtype=np.uint8)
        return SensorObservation(
            timestamp=self.step_count * self.config.control_dt,
            pose=tuple(float(v) for v in self.pose),
            velocity=tuple(float(v) for v in self.velocity),
            goal_distance=goal_distance,
            obstacle_distance=float(self.config.mock_obstacle_distance),
            image=image,
            depth=depth,
            metadata={
                "source": "mock_gwm_demo",
                "step": self.step_count,
                "source_coordinate_frame": "project_default",
            },
        )

    def apply_command(self, command: ControlCommand) -> SensorObservation:
        self.velocity = np.asarray((command.vx, command.vy, command.vz), dtype=np.float32)
        self.pose = self.pose + self.velocity * self.config.control_dt
        self.step_count += 1
        return self.observe()


class _IsaacObservationSource(_ObservationSource):
    def __init__(self, config: GWMDemoConfig) -> None:
        from src.env import IsaacSimNavigationEnv

        self.env = IsaacSimNavigationEnv(config=config.isaac)
        self.current: SensorObservation | None = None

    def reset(self) -> SensorObservation:
        self.current = self.env.reset()
        return self.current

    def observe(self) -> SensorObservation:
        self.current = self.env.get_observation()
        return self.current

    def apply_command(self, command: ControlCommand) -> SensorObservation:
        action = np.asarray((command.vx, command.vy, command.vz), dtype=np.float32)
        observation, _, _, _ = self.env.step(action)
        self.current = observation
        return observation

    def close(self) -> None:
        self.env.close()


class _ROS2ObservationSource(_ObservationSource):
    def __init__(self, config: GWMDemoConfig) -> None:
        from src.ros2_bridge import ROS2SensorSynchronizer

        self.sync = ROS2SensorSynchronizer(config={"ros2": config.ros2})
        self.sync.start()

    def reset(self) -> SensorObservation:
        observation = self.sync.latest_observation()
        if observation is None:
            raise RuntimeError("ROS2 observation source has no synchronized packet.")
        return observation

    def observe(self) -> SensorObservation:
        observation = self.sync.latest_observation()
        if observation is None:
            raise RuntimeError("ROS2 observation source has no synchronized packet.")
        return observation

    def apply_command(self, command: ControlCommand) -> SensorObservation:
        del command
        return self.observe()

    def close(self) -> None:
        self.sync.shutdown()


class _ExecutionBackend:
    def execute(self, command: ControlCommand) -> Dict[str, Any]:
        raise NotImplementedError

    def emergency_stop(self, reason: str) -> Dict[str, Any]:
        del reason
        return {"action": "emergency_stop", "supported": False}

    def summary(self) -> Dict[str, Any]:
        return {}

    def close(self) -> None:
        return None


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
        return self.sampler.sample(
            start=start,
            goal=goal,
            num_candidates=num_candidates,
            max_speed=self.max_speed,
        )


class _MockExecutionBackend(_ExecutionBackend):
    def __init__(self) -> None:
        self.history: list[Dict[str, Any]] = []

    def execute(self, command: ControlCommand) -> Dict[str, Any]:
        entry = {
            "action": "send_command",
            "backend": "mock",
            "command": _command_to_dict(command),
        }
        self.history.append(entry)
        return entry

    def emergency_stop(self, reason: str) -> Dict[str, Any]:
        command = ControlCommand(mode=ControlMode.EMERGENCY_STOP, metadata={"reason": reason})
        entry = {
            "action": "emergency_stop",
            "backend": "mock",
            "command": _command_to_dict(command),
        }
        self.history.append(entry)
        return entry

    def summary(self) -> Dict[str, Any]:
        return {"mock_command_history": len(self.history)}


class _IsaacExecutionBackend(_ExecutionBackend):
    def __init__(self, config: GWMDemoConfig) -> None:
        from src.env import IsaacSimNavigationEnv

        self.env = IsaacSimNavigationEnv(config=config.isaac)
        self.env.reset()

    def execute(self, command: ControlCommand) -> Dict[str, Any]:
        action = np.asarray((command.vx, command.vy, command.vz), dtype=np.float32)
        _, reward, done, info = self.env.step(action)
        return {"action": "step", "backend": "isaac", "reward": reward, "done": done, "info": info}

    def close(self) -> None:
        self.env.close()

    def summary(self) -> Dict[str, Any]:
        return {"isaac_runtime": "guarded_optional"}


class _MAVSDKSITLExecutionBackend(_ExecutionBackend):
    def __init__(self, config: GWMDemoConfig, safety_filter: ControlBarrierFunction) -> None:
        from src.ros2_bridge import MAVLinkBridge

        self.bridge = MAVLinkBridge(
            config={"deployment": config.deployment, "mavlink": config.mavlink},
            safety_filter=safety_filter,
        )
        asyncio.run(self.bridge.connect())
        asyncio.run(self.bridge.wait_until_ready(config.mavlink.get("health_timeout_sec", 10.0)))
        asyncio.run(self.bridge.start_offboard(ControlCommand()))

    def execute(self, command: ControlCommand) -> Dict[str, Any]:
        asyncio.run(self.bridge.send_command(command))
        return dict(self.bridge.command_history[-1])

    def emergency_stop(self, reason: str) -> Dict[str, Any]:
        del reason
        asyncio.run(self.bridge.emergency_stop())
        return dict(self.bridge.command_history[-1])

    def close(self) -> None:
        asyncio.run(self.bridge.disconnect())

    def summary(self) -> Dict[str, Any]:
        return {"mavsdk_sitl": True}


def run_demo(config: GWMDemoConfig | Mapping[str, Any] | str | Path | None = None) -> Dict[str, Any]:
    """Run the GWM navigation demo and return a JSON-safe result dictionary."""
    return GWMDemoRunner(config).run()


def load_demo_config(path: str | Path) -> Dict[str, Any]:
    """Load a YAML demo config file."""
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def build_arg_parser() -> argparse.ArgumentParser:
    """Return the CLI parser used by ``scripts/run_gwm_navigation_demo.py``."""
    parser = argparse.ArgumentParser(description="Run the mock-first GWM navigation demo.")
    parser.add_argument("--config", default="configs/gwm_navigation_demo.yaml")
    parser.add_argument("--backend", choices=["mock", "isaac", "mavsdk_sitl"], default=None)
    parser.add_argument("--observation-source", choices=["mock", "isaac", "ros2"], default=None)
    parser.add_argument("--execution-backend", choices=["mock", "isaac", "mavsdk_sitl"], default=None)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--horizon", type=int, default=None)
    parser.add_argument("--num-candidates", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--checkpoint", dest="checkpoint_path", default=None)
    parser.add_argument("--output", dest="output_path", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--allow-optional-runtime", action="store_true")
    parser.add_argument("--fail-on-runtime-unavailable", action="store_true")
    return parser


def config_from_args(args: argparse.Namespace) -> GWMDemoConfig:
    """Load config from CLI arguments and apply explicit overrides."""
    source: Dict[str, Any] = {}
    config_path = Path(args.config)
    if config_path.exists():
        source = load_demo_config(config_path)
    source.setdefault("demo", {})
    demo = dict(source.get("demo") or {})
    if args.backend is not None:
        demo["execution_backend"] = args.backend
    for key in (
        "observation_source",
        "execution_backend",
        "steps",
        "horizon",
        "num_candidates",
        "seed",
        "checkpoint_path",
        "output_path",
    ):
        value = getattr(args, key)
        if value is not None:
            demo[key] = value
    if args.no_write_output:
        demo["write_output"] = False
    if args.allow_optional_runtime:
        demo["allow_optional_runtime"] = True
    if args.fail_on_runtime_unavailable:
        demo["fail_on_runtime_unavailable"] = True
    source["demo"] = demo
    return GWMDemoConfig.from_any(source)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entrypoint."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    result = run_demo(config_from_args(args))
    metrics = result.get("metrics", {})
    print(
        "gwm_demo "
        f"status={result.get('final_status')} "
        f"steps={metrics.get('total_steps', 0)} "
        f"commands={metrics.get('commands_sent', 0)} "
        f"safety_overrides={metrics.get('safety_overrides', 0)}"
    )
    if result.get("config_summary", {}).get("write_output"):
        print(f"output={result.get('config_summary', {}).get('output_path', '')}")
    return 0


def _command_from_candidate(
    candidate: TrajectoryCandidate,
    *,
    duration: float,
    step_index: int,
    score: Dict[str, Any],
) -> ControlCommand:
    actions = np.asarray(candidate.actions, dtype=np.float32)
    if actions.ndim != 2 or actions.shape[1] < 3:
        raise ValueError("candidate actions must have shape [H, 3].")
    action = actions[0, :3]
    return ControlCommand(
        vx=float(action[0]),
        vy=float(action[1]),
        vz=float(action[2]),
        yaw_rate=0.0,
        duration=float(duration),
        mode=ControlMode.WORLD_MODEL_GUIDED,
        metadata={
            "source": "gwm_navigation_demo",
            "step": int(step_index),
            "candidate_index": int(candidate.metadata.get("candidate_index", -1)),
            "selected_score": float(score["total_score"]),
        },
    )


def _command_to_dict(command: ControlCommand) -> Dict[str, Any]:
    return {
        "vx": float(command.vx),
        "vy": float(command.vy),
        "vz": float(command.vz),
        "yaw_rate": float(command.yaw_rate),
        "duration": float(command.duration),
        "mode": command.mode.value,
        "metadata": dict(command.metadata),
    }


def _metrics_from_steps(steps: list[Dict[str, Any]], final_status: str) -> Dict[str, Any]:
    total_scores = [step["selected_score"] for step in steps]
    obstacle_distances = [step["obstacle_distance"] for step in steps]
    uncertainties = [
        float(step["score_components"].get("uncertainty", 0.0))
        for step in steps
    ]
    return {
        "total_steps": len(steps),
        "commands_sent": len(steps),
        "goal_reached": any(step["goal_reached"] for step in steps),
        "collision_detected": any(step["collision_detected"] for step in steps),
        "timeout": final_status == "timeout",
        "mean_total_score": float(np.mean(total_scores)) if total_scores else 0.0,
        "min_obstacle_distance": float(np.min(obstacle_distances)) if obstacle_distances else 0.0,
        "max_uncertainty": float(np.max(uncertainties)) if uncertainties else 0.0,
        "safety_overrides": sum(
            1
            for step in steps
            if step["safe_command"]["mode"] == ControlMode.SAFETY_OVERRIDE.value
        ),
        "emergency_stops": sum(
            1
            for step in steps
            if step["safe_command"]["mode"] == ControlMode.EMERGENCY_STOP.value
        ),
    }


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


def _safety_limits_from_config(config: Dict[str, Any]) -> SafetyLimits:
    velocity = dict(config.get("velocity_limits") or {})
    altitude = dict(config.get("altitude_bounds") or {})
    geofence_config = dict(config.get("geofence") or {})
    geofence = None
    if geofence_config.get("enabled", False):
        geofence = {
            axis: geofence_config[axis]
            for axis in ("x", "y", "z")
            if axis in geofence_config
        }
    return SafetyLimits(
        max_vx=float(velocity.get("max_vx", 4.0)),
        max_vy=float(velocity.get("max_vy", 4.0)),
        max_vz=float(velocity.get("max_vz", 2.0)),
        max_yaw_rate=float(velocity.get("max_yaw_rate", 1.0)),
        min_altitude=float(altitude.get("min_altitude", 0.5)),
        max_altitude=float(altitude.get("max_altitude", 120.0)),
        geofence=geofence,
    )


def _reject_real_flight_flags(config: Mapping[str, Any]) -> None:
    if bool(config.get("real_hardware_enabled", False)):
        raise RuntimeError("Phase 4-F demo rejects real_hardware_enabled=True.")
    if bool(config.get("autonomous_real_flight_enabled", False)):
        raise RuntimeError("Phase 4-F demo rejects autonomous_real_flight_enabled=True.")


def _tuple3(value: Any) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=np.float32).reshape(-1)
    if array.size < 3:
        raise ValueError("Expected at least three coordinates.")
    return (float(array[0]), float(array[1]), float(array[2]))


def _optional_str(value: Any) -> Optional[str]:
    if value is None:
        return None
    return str(value)


def _nested(config: Mapping[str, Any], path: Sequence[str], default: Any) -> Any:
    value: Any = config
    for key in path:
        if not isinstance(value, Mapping) or key not in value:
            return default
        value = value[key]
    return value
