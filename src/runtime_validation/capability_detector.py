"""Read-only runtime capability detection for Phase 5-A."""

from __future__ import annotations

import importlib.util
import os
import platform as platform_module
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

from src.runtime_validation.types import CapabilityStatus, RuntimeCapabilityReport

AIRSIM_BACKEND_REGISTRY_NAME = "airsim"
AIRSIM_PRIMARY_MODULE = "cosysairsim"
AIRSIM_PRIMARY_LABEL = "Cosys-AirSim"
AIRSIM_FALLBACK_MODULE = "airsim"
AIRSIM_FALLBACK_LABEL = "legacy AirSim"
AIRSIM_IMPORT_ORDER = (AIRSIM_PRIMARY_MODULE, AIRSIM_FALLBACK_MODULE)

SCHEMA_VERSION = "gwm_runtime_capability_report_v1"

DEFAULT_PROBES: Dict[str, bool] = {
    "python": True,
    "cuda": True,
    "nvidia_smi": True,
    "isaac_sim": True,
    "airsim": True,
    "ros2": True,
    "mavsdk": True,
    "px4": True,
    "github_cli": True,
}

ENV_ALLOWLIST = (
    "GWM_RUNTIME_ARTIFACT_DIR",
    "GWM_RUN_ISAAC_RUNTIME_TESTS",
    "GWM_RUN_AIRSIM_RUNTIME_TESTS",
    "GWM_ALLOW_AIRSIM_API_CONTROL",
    "GWM_RUN_ROS2_SENSOR_SYNC_TESTS",
    "GWM_RUN_MAVSDK_SITL_TESTS",
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_ALLOW_SITL_COMMANDS",
    "GWM_SITL_CONNECTION_URL",
    "GWM_ROS2_LIVE_TOPICS",
    "GWM_ALLOW_PX4_LAUNCH",
    "ROS_DISTRO",
    "CUDA_VISIBLE_DEVICES",
)

SENSITIVE_KEY_PARTS = ("TOKEN", "SECRET", "PASSWORD", "KEY", "CREDENTIAL", "AUTH")


class RuntimeCapabilityDetector:
    """Detect optional runtime capabilities without launching runtimes."""

    def __init__(
        self,
        config: Optional[Mapping[str, Any]] = None,
        *,
        environ: Optional[Mapping[str, str]] = None,
        import_spec: Optional[Callable[[str], Any]] = None,
        command_finder: Optional[Callable[[str], Optional[str]]] = None,
        command_runner: Optional[Callable[[Sequence[str]], subprocess.CompletedProcess[str]]] = None,
    ) -> None:
        self.config = _normalize_config(config)
        self.environ = dict(os.environ if environ is None else environ)
        self.import_spec = import_spec or importlib.util.find_spec
        self.command_finder = command_finder or shutil.which
        self.command_runner = command_runner or self._run_command

    def detect(self) -> RuntimeCapabilityReport:
        """Return a read-only capability report."""
        probes = dict(DEFAULT_PROBES)
        probes.update(self.config.get("probes", {}))
        return RuntimeCapabilityReport(
            schema_version=SCHEMA_VERSION,
            generated_at=datetime.now(timezone.utc).isoformat(),
            platform=self._detect_platform(),
            python=self._detect_python() if probes.get("python", True) else {},
            cuda=self._detect_cuda() if probes.get("cuda", True) else {},
            gpu=self._detect_gpu(probes) if probes.get("cuda", True) else {},
            isaac_sim=self._detect_isaac_sim() if probes.get("isaac_sim", True) else _skipped("isaac_sim"),
            airsim=self._detect_airsim() if probes.get("airsim", True) else _skipped("airsim"),
            ros2=self._detect_ros2() if probes.get("ros2", True) else _skipped("ros2"),
            mavsdk=self._detect_mavsdk() if probes.get("mavsdk", True) else _skipped("mavsdk"),
            px4=self._detect_px4() if probes.get("px4", True) else _skipped("px4"),
            github_cli=(
                self._detect_github_cli()
                if probes.get("github_cli", True)
                else _skipped("github_cli")
            ),
            environment=self._detect_environment(),
            safety=self._safety_summary(),
        )

    @staticmethod
    def _run_command(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            list(command),
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )

    def _detect_platform(self) -> Dict[str, Any]:
        return {
            "platform": platform_module.platform(),
            "system": platform_module.system(),
            "release": platform_module.release(),
            "machine": platform_module.machine(),
            "processor": platform_module.processor(),
        }

    def _detect_python(self) -> Dict[str, Any]:
        return {
            "version": sys.version,
            "version_info": list(sys.version_info[:3]),
            "executable": sys.executable,
        }

    def _detect_cuda(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "torch_available": False,
            "torch_cuda_available": False,
            "torch_cuda_version": None,
            "torch_cuda_device_count": 0,
            "error": None,
        }
        try:
            import torch

            result["torch_available"] = True
            result["torch_cuda_available"] = bool(torch.cuda.is_available())
            result["torch_cuda_version"] = torch.version.cuda
            result["torch_cuda_device_count"] = int(torch.cuda.device_count())
        except Exception as exc:  # pragma: no cover - depends on local torch install
            result["error"] = str(exc)
        return result

    def _detect_gpu(self, probes: Mapping[str, bool]) -> Dict[str, Any]:
        nvidia_smi_path = self.command_finder("nvidia-smi")
        result: Dict[str, Any] = {
            "nvidia_smi_available": bool(nvidia_smi_path),
            "nvidia_smi_path": nvidia_smi_path,
            "gpus": [],
            "error": None,
        }
        if not nvidia_smi_path or not probes.get("nvidia_smi", True):
            return result
        try:
            completed = self.command_runner(
                (
                    nvidia_smi_path,
                    "--query-gpu=name,driver_version,memory.total",
                    "--format=csv,noheader",
                )
            )
            if completed.returncode != 0:
                result["error"] = _first_line(completed.stderr or completed.stdout)
                return result
            result["gpus"] = [_parse_nvidia_smi_line(line) for line in completed.stdout.splitlines() if line.strip()]
        except Exception as exc:
            result["error"] = str(exc)
        return result

    def _detect_isaac_sim(self) -> CapabilityStatus:
        modules = {
            "isaacsim": self._safe_find_spec("isaacsim"),
            "omni": self._safe_find_spec("omni"),
        }
        return CapabilityStatus(
            name="isaac_sim",
            available=any(status["available"] for status in modules.values()),
            details={
                "modules": modules,
                "probe_only": True,
                "simulation_app_instantiated": False,
            },
            error=_first_error(modules),
        )

    def _detect_airsim(self) -> CapabilityStatus:
        modules = {
            AIRSIM_PRIMARY_MODULE: self._safe_find_spec(AIRSIM_PRIMARY_MODULE),
            AIRSIM_FALLBACK_MODULE: self._safe_find_spec(AIRSIM_FALLBACK_MODULE),
        }
        preferred = None
        for name in AIRSIM_IMPORT_ORDER:
            if modules[name]["available"]:
                preferred = name
                break
        return CapabilityStatus(
            name=AIRSIM_BACKEND_REGISTRY_NAME,
            available=preferred is not None,
            version=preferred,
            details={
                "backend_registry_name": AIRSIM_BACKEND_REGISTRY_NAME,
                "modules": modules,
                "primary_runtime": AIRSIM_PRIMARY_MODULE,
                "primary_runtime_label": AIRSIM_PRIMARY_LABEL,
                "fallback_runtime": AIRSIM_FALLBACK_MODULE,
                "fallback_runtime_label": AIRSIM_FALLBACK_LABEL,
                "preferred_module": preferred,
                "preferred_runtime_label": _airsim_runtime_label(preferred),
                "connection_attempted": False,
                "api_control_enabled": False,
                "unreal_launch_attempted": False,
            },
            error=_first_error(modules),
        )

    def _detect_ros2(self) -> CapabilityStatus:
        modules = {
            name: self._safe_find_spec(name)
            for name in ("rclpy", "message_filters", "sensor_msgs", "nav_msgs", "geometry_msgs")
        }
        env = {
            name: self._safe_env_value(name)
            for name in ("ROS_DISTRO", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH")
        }
        return CapabilityStatus(
            name="ros2",
            available=all(modules[name]["available"] for name in ("rclpy", "message_filters")),
            version=env.get("ROS_DISTRO"),
            details={
                "modules": modules,
                "environment": env,
                "nodes_started": False,
                "live_topics_checked": False,
            },
            error=_first_error(modules),
        )

    def _detect_mavsdk(self) -> CapabilityStatus:
        module = self._safe_find_spec("mavsdk")
        return CapabilityStatus(
            name="mavsdk",
            available=module["available"],
            details={
                "module": module,
                "connection_attempted": False,
            },
            error=module.get("error"),
        )

    def _detect_px4(self) -> CapabilityStatus:
        commands = {
            name: self._command_status(name)
            for name in ("px4", "make", "git")
        }
        return CapabilityStatus(
            name="px4",
            available=commands["px4"]["available"],
            details={
                "commands": commands,
                "sitl_launched": False,
                "connection_attempted": False,
            },
        )

    def _detect_github_cli(self) -> CapabilityStatus:
        gh = self._command_status("gh")
        if not gh["available"]:
            return CapabilityStatus(
                name="github_cli",
                available=False,
                details={"command": gh},
                error="gh command not found",
            )
        version = None
        auth_status = None
        auth_available = False
        version_result = self._safe_run((gh["path"], "--version"))
        if version_result["returncode"] == 0:
            version = _first_line(version_result["stdout"])
        auth_result = self._safe_run((gh["path"], "auth", "status"))
        auth_status = _redact_text((auth_result["stdout"] + "\n" + auth_result["stderr"]).strip())
        auth_available = auth_result["returncode"] == 0
        return CapabilityStatus(
            name="github_cli",
            available=True,
            version=version,
            details={
                "command": gh,
                "auth_available": auth_available,
                "auth_status": auth_status,
            },
            error=None if auth_available else _first_line(auth_status),
        )

    def _detect_environment(self) -> Dict[str, Any]:
        include_path_hints = bool(self.config.get("include_path_hints", True))
        values = {
            name: {
                "present": name in self.environ,
                "value": self._safe_env_value(name),
            }
            for name in ENV_ALLOWLIST
        }
        result: Dict[str, Any] = {
            "allowlisted_variables": values,
            "redaction_enabled": bool(self.config.get("redact_sensitive_env", True)),
            "all_environment_dumped": False,
        }
        if include_path_hints:
            result["path_hints"] = _path_hints(self.environ.get("PATH", ""))
        return result

    def _safety_summary(self) -> Dict[str, Any]:
        safety = dict(self.config.get("safety", {}))
        return {
            "launch_runtimes": bool(safety.get("launch_runtimes", False)),
            "connect_to_sitl": bool(safety.get("connect_to_sitl", False)),
            "connect_to_hardware": bool(safety.get("connect_to_hardware", False)),
            "real_hardware_enabled": False,
            "autonomous_real_flight_enabled": False,
            "read_only_probe": True,
        }

    def _safe_find_spec(self, module_name: str) -> Dict[str, Any]:
        try:
            spec = self.import_spec(module_name)
            return {
                "available": spec is not None,
                "origin": None if spec is None else getattr(spec, "origin", None),
            }
        except Exception as exc:
            return {"available": False, "origin": None, "error": str(exc)}

    def _command_status(self, command: str) -> Dict[str, Any]:
        path = self.command_finder(command)
        return {"available": bool(path), "path": path}

    def _safe_run(self, command: Sequence[str]) -> Dict[str, Any]:
        try:
            completed = self.command_runner(command)
            return {
                "returncode": int(completed.returncode),
                "stdout": _redact_text(completed.stdout or ""),
                "stderr": _redact_text(completed.stderr or ""),
            }
        except Exception as exc:
            return {"returncode": -1, "stdout": "", "stderr": _redact_text(str(exc))}

    def _safe_env_value(self, key: str) -> Optional[str]:
        if key not in self.environ:
            return None
        value = str(self.environ[key])
        if _is_sensitive_key(key) and self.config.get("redact_sensitive_env", True):
            return "<redacted>"
        return value


def _normalize_config(config: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    source = dict(config or {})
    if "runtime_validation" in source:
        source = dict(source["runtime_validation"] or {})
    normalized = {
        "write_report": bool(source.get("write_report", True)),
        "output_dir": str(source.get("output_dir", "outputs/runtime_validation")),
        "include_path_hints": bool(source.get("include_path_hints", True)),
        "redact_sensitive_env": bool(source.get("redact_sensitive_env", True)),
        "probes": dict(DEFAULT_PROBES),
        "safety": {
            "launch_runtimes": False,
            "connect_to_sitl": False,
            "connect_to_hardware": False,
        },
    }
    normalized["probes"].update(source.get("probes") or {})
    normalized["safety"].update(source.get("safety") or {})
    return normalized


def _skipped(name: str) -> CapabilityStatus:
    return CapabilityStatus(
        name=name,
        available=False,
        details={"skipped": True},
    )


def _parse_nvidia_smi_line(line: str) -> Dict[str, str]:
    name, driver, memory = (part.strip() for part in (line.split(",", 2) + ["", ""])[:3])
    return {
        "name": name,
        "driver_version": driver,
        "memory_total": memory,
    }


def _path_hints(path_value: str) -> Dict[str, Any]:
    entries = [entry for entry in path_value.split(os.pathsep) if entry]
    interesting = [
        entry
        for entry in entries
        if any(
            marker in entry.lower()
            for marker in ("ros", "px4", "isaac", "omniverse", "cuda", "mavsdk")
        )
    ]
    return {
        "entry_count": len(entries),
        "interesting_entries": interesting[:20],
        "truncated": len(interesting) > 20,
    }


def _is_sensitive_key(key: str) -> bool:
    upper = key.upper()
    return any(part in upper for part in SENSITIVE_KEY_PARTS)


def _redact_text(value: str) -> str:
    redacted_lines = []
    for line in value.splitlines():
        if any(part in line.upper() for part in SENSITIVE_KEY_PARTS):
            redacted_lines.append("<redacted>")
        else:
            redacted_lines.append(line)
    return "\n".join(redacted_lines)


def _first_line(value: Optional[str]) -> Optional[str]:
    if not value:
        return None
    for line in value.splitlines():
        if line.strip():
            return line.strip()
    return None


def _first_error(items: Mapping[str, Mapping[str, Any]]) -> Optional[str]:
    for value in items.values():
        error = value.get("error")
        if error:
            return str(error)
    return None


def _airsim_runtime_label(module_name: Optional[str]) -> Optional[str]:
    if module_name == AIRSIM_PRIMARY_MODULE:
        return AIRSIM_PRIMARY_LABEL
    if module_name == AIRSIM_FALLBACK_MODULE:
        return AIRSIM_FALLBACK_LABEL
    return None
