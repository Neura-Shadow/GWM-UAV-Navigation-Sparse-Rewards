"""pytest plugin (diagnostics only): run the existing P5 tests with the CPU-D polar profile swapped in.

Load with `-p polar_patch_plugin` and this directory on PYTHONPATH. At configure time it imports the production
`gwm_policy_control.observation_adapter` and replaces its module attribute `polar_profile` with
`optimized.polar_profile` for the test session. No source file changes. At the end it reports how many times
the optimized function ran, so a session that never exercised it cannot pass silently.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CALLS = [0]


def pytest_configure(config):
    for path in (ROOT / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control",
                 ROOT / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance",
                 ROOT / "simulation/px4_gazebo_p5/ros2_ws/src/gwm_policy_control"):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from gwm_policy_control import observation_adapter
    import optimized

    def patched(*args, **kwargs):
        CALLS[0] += 1
        return optimized.polar_profile(*args, **kwargs)
    observation_adapter.polar_profile = patched
    print(f"\n[polar_patch_plugin] observation_adapter.polar_profile -> optimized.polar_profile ({observation_adapter.__file__})")


def pytest_terminal_summary(terminalreporter):
    terminalreporter.write_line(f"[polar_patch_plugin] optimized polar_profile calls: {CALLS[0]}")
