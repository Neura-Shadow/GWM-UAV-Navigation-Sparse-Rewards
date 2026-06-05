"""Generated World Model core for Phase 4-A UAV navigation research."""

from src.generated_world_model.action_conditioner import ActionConditioner
from src.generated_world_model.dataset import (
    GeneratedWorldModelDataset,
    create_synthetic_batch,
    load_npz_sequence,
    save_npz_sequence,
)
from src.generated_world_model.demo import (
    GWMDemoConfig,
    GWMDemoResult,
    GWMDemoRunner,
    run_demo,
)
from src.generated_world_model.sim_runtime_demo import (
    DEFAULT_OUTPUT_PATH as DEFAULT_PHASE6_GWM_SIMULATION_DEMO_OUTPUT_PATH,
    Phase6GWMSimulationDemoConfig,
    Phase6GWMSimulationDemoResult,
    Phase6RuntimeReadiness,
    run_phase6_gwm_simulation_demo,
)
from src.generated_world_model.multisim_demo import (
    DEFAULT_OUTPUT_PATH as DEFAULT_MULTISIM_GWM_DEMO_OUTPUT_PATH,
    MultiSimGWMDemoConfig,
    run_multisim_gwm_demo,
)
from src.generated_world_model.losses import generated_world_model_loss
from src.generated_world_model.future_frame_projection import (
    CameraIntrinsics,
    FutureFrameProjection,
    ProjectionConfig,
    ProjectionResult,
)
from src.generated_world_model.losses import future_frame_projection_loss
from src.generated_world_model.observation_buffer import ObservationBuffer
from src.generated_world_model.observation_encoder import ObservationEncoder
from src.generated_world_model.planner import GeneratedWorldModelPlanner
from src.generated_world_model.rollout import AutoregressiveRollout
from src.generated_world_model.training import (
    build_baseline_components,
    make_synthetic_training_batch,
    train_synthetic_step,
)
from src.generated_world_model.trajectory_sampler import CandidateTrajectorySampler
from src.generated_world_model.trajectory_scorer import TrajectoryScorer
from src.generated_world_model.types import (
    ActionSequence,
    GeneratedObservation,
    GeneratedRollout,
    GWMConfig,
    ObservationBatch,
    TrajectoryCandidate,
    TrajectoryScore,
)
from src.generated_world_model.video_dynamics_model import VideoDynamicsModel

__all__ = [
    "ActionConditioner",
    "ActionSequence",
    "AutoregressiveRollout",
    "CameraIntrinsics",
    "CandidateTrajectorySampler",
    "FutureFrameProjection",
    "GeneratedObservation",
    "GeneratedRollout",
    "GWMDemoConfig",
    "GWMDemoResult",
    "GWMDemoRunner",
    "DEFAULT_PHASE6_GWM_SIMULATION_DEMO_OUTPUT_PATH",
    "DEFAULT_MULTISIM_GWM_DEMO_OUTPUT_PATH",
    "GeneratedWorldModelDataset",
    "GeneratedWorldModelPlanner",
    "GWMConfig",
    "MultiSimGWMDemoConfig",
    "ObservationBatch",
    "ObservationBuffer",
    "ObservationEncoder",
    "Phase6GWMSimulationDemoConfig",
    "Phase6GWMSimulationDemoResult",
    "Phase6RuntimeReadiness",
    "ProjectionConfig",
    "ProjectionResult",
    "TrajectoryCandidate",
    "TrajectoryScore",
    "TrajectoryScorer",
    "VideoDynamicsModel",
    "build_baseline_components",
    "create_synthetic_batch",
    "generated_world_model_loss",
    "future_frame_projection_loss",
    "load_npz_sequence",
    "make_synthetic_training_batch",
    "run_demo",
    "run_multisim_gwm_demo",
    "run_phase6_gwm_simulation_demo",
    "save_npz_sequence",
    "train_synthetic_step",
]
