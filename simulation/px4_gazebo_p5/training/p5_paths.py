"""Import paths for the offline P5 training tools: the P3/P4 packages (read-only) and the P5 package."""
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[3]
SIM = REPO / "simulation/px4_gazebo"
P4 = REPO / "simulation/px4_gazebo_p4"
P5 = REPO / "simulation/px4_gazebo_p5"
for _path in (P5 / "training", P5 / "ros2_ws/src/gwm_policy_control", P4 / "ros2_ws/src/gwm_obstacle_avoidance",
              SIM / "ros2_ws/src/gwm_sensor_adapter", SIM / "ros2_ws/src/gwm_px4_control"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
