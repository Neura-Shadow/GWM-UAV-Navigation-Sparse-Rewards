"""Runtime validation utilities for guarded Phase 5 readiness checks."""

from src.runtime_validation.capability_detector import (
    ENV_ALLOWLIST,
    RuntimeCapabilityDetector,
)
from src.runtime_validation.closed_loop_readiness import (
    DEFAULT_OUTPUT_PATH as DEFAULT_CLOSED_LOOP_READINESS_OUTPUT_PATH,
    ClosedLoopReadinessConfig,
    ClosedLoopReadinessResult,
    build_closed_loop_pipeline_plan,
    run_closed_loop_readiness,
)
from src.runtime_validation.isaac_runtime_smoke import (
    DEFAULT_OUTPUT_PATH as DEFAULT_ISAAC_RUNTIME_SMOKE_OUTPUT_PATH,
    IsaacRuntimeSmokeConfig,
    IsaacRuntimeSmokeResult,
    build_tiny_isaac_descriptor,
    run_isaac_runtime_smoke,
)
from src.runtime_validation.isaac_sensor_runtime import (
    DEFAULT_OUTPUT_PATH as DEFAULT_ISAAC_SENSOR_RUNTIME_OUTPUT_PATH,
    IsaacSensorRuntimeConfig,
    IsaacSensorRuntimeResult,
    run_isaac_sensor_runtime,
)
from src.runtime_validation.isaac_px4_bridge_design import (
    DEFAULT_OUTPUT_PATH as DEFAULT_ISAAC_PX4_BRIDGE_DESIGN_OUTPUT_PATH,
    FrameTransformPolicy,
    IsaacPX4BridgeDesignConfig,
    IsaacPX4BridgeDesignResult,
    build_isaac_px4_bridge_plan,
    run_isaac_px4_bridge_design,
)
from src.runtime_validation.mavsdk_sitl_smoke import (
    DEFAULT_OUTPUT_PATH as DEFAULT_MAVSDK_SITL_SMOKE_OUTPUT_PATH,
    MAVSDKSITLSmokeConfig,
    MAVSDKSITLSmokeResult,
    build_safe_sitl_command,
    run_mavsdk_sitl_smoke,
)
from src.runtime_validation.px4_sitl_command_validation import (
    DEFAULT_OUTPUT_PATH as DEFAULT_PX4_SITL_COMMAND_VALIDATION_OUTPUT_PATH,
    PX4SITLCommandValidationConfig,
    PX4SITLCommandValidationResult,
    build_phase6_sitl_command_sequence,
    run_px4_sitl_command_validation,
)
from src.runtime_validation.reporting import report_to_dict, report_to_json, write_report
from src.runtime_validation.ros2_sensor_sync_smoke import (
    DEFAULT_OUTPUT_PATH as DEFAULT_ROS2_SENSOR_SYNC_SMOKE_OUTPUT_PATH,
    ROS2SensorSyncSmokeConfig,
    ROS2SensorSyncSmokeResult,
    build_mock_sensor_messages,
    run_ros2_sensor_sync_smoke,
)
from src.runtime_validation.ros2_sim_sensor_bridge import (
    DEFAULT_OUTPUT_PATH as DEFAULT_ROS2_SIM_SENSOR_BRIDGE_OUTPUT_PATH,
    ROS2SimSensorBridgeConfig,
    ROS2SimSensorBridgeResult,
    ROS2SimulationSensorBridge,
    run_ros2_sim_sensor_bridge,
)
from src.runtime_validation.types import CapabilityStatus, RuntimeCapabilityReport

__all__ = [
    "CapabilityStatus",
    "ClosedLoopReadinessConfig",
    "ClosedLoopReadinessResult",
    "DEFAULT_CLOSED_LOOP_READINESS_OUTPUT_PATH",
    "DEFAULT_ISAAC_RUNTIME_SMOKE_OUTPUT_PATH",
    "DEFAULT_ISAAC_PX4_BRIDGE_DESIGN_OUTPUT_PATH",
    "DEFAULT_ISAAC_SENSOR_RUNTIME_OUTPUT_PATH",
    "DEFAULT_MAVSDK_SITL_SMOKE_OUTPUT_PATH",
    "DEFAULT_PX4_SITL_COMMAND_VALIDATION_OUTPUT_PATH",
    "DEFAULT_ROS2_SENSOR_SYNC_SMOKE_OUTPUT_PATH",
    "DEFAULT_ROS2_SIM_SENSOR_BRIDGE_OUTPUT_PATH",
    "ENV_ALLOWLIST",
    "IsaacRuntimeSmokeConfig",
    "IsaacRuntimeSmokeResult",
    "FrameTransformPolicy",
    "IsaacPX4BridgeDesignConfig",
    "IsaacPX4BridgeDesignResult",
    "IsaacSensorRuntimeConfig",
    "IsaacSensorRuntimeResult",
    "MAVSDKSITLSmokeConfig",
    "MAVSDKSITLSmokeResult",
    "PX4SITLCommandValidationConfig",
    "PX4SITLCommandValidationResult",
    "ROS2SensorSyncSmokeConfig",
    "ROS2SensorSyncSmokeResult",
    "ROS2SimSensorBridgeConfig",
    "ROS2SimSensorBridgeResult",
    "ROS2SimulationSensorBridge",
    "RuntimeCapabilityDetector",
    "RuntimeCapabilityReport",
    "build_closed_loop_pipeline_plan",
    "build_isaac_px4_bridge_plan",
    "build_mock_sensor_messages",
    "build_phase6_sitl_command_sequence",
    "build_safe_sitl_command",
    "build_tiny_isaac_descriptor",
    "report_to_dict",
    "report_to_json",
    "run_closed_loop_readiness",
    "run_isaac_runtime_smoke",
    "run_isaac_px4_bridge_design",
    "run_isaac_sensor_runtime",
    "run_mavsdk_sitl_smoke",
    "run_px4_sitl_command_validation",
    "run_ros2_sensor_sync_smoke",
    "run_ros2_sim_sensor_bridge",
    "write_report",
]
