"""Simulator backend registry.

The registry keeps backend selection explicit and lazy so optional simulator
packages are never imported unless their backend is requested.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Mapping

from src.env.base_env import BaseNavigationEnv

BackendFactory = Callable[[Mapping[str, Any]], BaseNavigationEnv]


@dataclass
class SimulatorBackendConfig:
    """Configuration for a navigation simulator backend."""

    backend: str = "mock"
    live_runtime_enabled: bool = False
    config: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_any(cls, value: "SimulatorBackendConfig | Mapping[str, Any] | None") -> "SimulatorBackendConfig":
        if isinstance(value, SimulatorBackendConfig):
            return cls(
                backend=value.backend,
                live_runtime_enabled=value.live_runtime_enabled,
                config=dict(value.config),
            )
        source = dict(value or {})
        nested = dict(source.get("simulator") or source.get("config") or {})
        backend = str(source.get("backend", nested.get("backend", "mock"))).lower()
        live_runtime_enabled = bool(
            source.get(
                "live_runtime_enabled",
                nested.get("live_runtime_enabled", False),
            )
        )
        env_config = dict(source.get("env_config") or source.get("runtime") or {})
        for key, item in nested.items():
            if key not in {"backend", "live_runtime_enabled"}:
                env_config.setdefault(key, item)
        return cls(
            backend=backend,
            live_runtime_enabled=live_runtime_enabled,
            config=env_config,
        )


class SimulatorBackendRegistry:
    """Registry of available simulator backend factories."""

    def __init__(self) -> None:
        self._factories: Dict[str, BackendFactory] = {}
        self.register("mock", _create_mock_env)
        self.register("isaac", _create_isaac_env)
        self.register("airsim", _create_airsim_env)

    def register(self, name: str, factory: BackendFactory) -> None:
        normalized = str(name).lower()
        if not normalized:
            raise ValueError("Simulator backend name must be non-empty.")
        self._factories[normalized] = factory

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._factories))

    def create(self, config: SimulatorBackendConfig | Mapping[str, Any] | None = None) -> BaseNavigationEnv:
        backend_config = SimulatorBackendConfig.from_any(config)
        factory = self._factories.get(backend_config.backend)
        if factory is None:
            available = ", ".join(self.names())
            raise ValueError(
                f"Unsupported simulator backend '{backend_config.backend}'. "
                f"Available backends: {available}."
            )
        return factory(backend_config.config)


def create_navigation_env(
    config: SimulatorBackendConfig | Mapping[str, Any] | None = None,
) -> BaseNavigationEnv:
    """Create a navigation env from the default backend registry."""
    return SimulatorBackendRegistry().create(config)


def _create_mock_env(config: Mapping[str, Any]) -> BaseNavigationEnv:
    from src.env import MockNavigationEnv

    return MockNavigationEnv(**dict(config))


def _create_isaac_env(config: Mapping[str, Any]) -> BaseNavigationEnv:
    from src.env import IsaacSimNavigationEnv

    return IsaacSimNavigationEnv(config=dict(config))


def _create_airsim_env(config: Mapping[str, Any]) -> BaseNavigationEnv:
    from src.env import AirSimNavigationEnv

    return AirSimNavigationEnv(config=dict(config))
