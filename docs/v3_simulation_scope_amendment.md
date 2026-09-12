# v3-SIM: operator-directed software simulation amendment

The operator's P0/P1 implementation request explicitly adds a development and
simulation-validation lane after completed v3-1C, ahead of v3-2. This amendment
does not renumber v3-2 through v3-7 or reopen completed v1/v2 work.

## Two separate authority boundaries

AgentOps application agents remain proposal-oriented. The default 13-tool
catalogue, permission manifests, contracts, and `invoke_mock()` are unchanged.
Agents receive no direct flight-control calls, shell/network access, physical
hardware access, self-enabling runtime gates, or human-approval authority.
This development task does not implement a human-approval service.

The operator-directed development workflow may inspect the local environment,
retrieve official pinned dependencies, install simulation/build dependencies
through normal package management, build them, explicitly launch local PX4
SITL/Gazebo, perform bounded non-AI simulated takeoff/hover/landing, collect
evidence, and terminate only its own session processes. Permission applies to
software simulation only. No USB/serial controller, radio, physical UAV,
remote vehicle endpoint, or real-hardware bridge may be accessed.

No offensive targeting, weapons, payload release, pursuit, interception, or
physical engagement behavior is authorized or implemented.

## Implementation and evidence boundary

`simulation/px4_gazebo/` is an explicit operator entrypoint, separate from
AgentOps and the existing Isaac/Cosys-AirSim integrations. No runtime starts
on import, inspection, ordinary pytest, or default script invocation. Launch
requires `--run` and three process-local gates; flight additionally requires
`--allow-simulated-flight` and `GWM_ALLOW_SITL_COMMANDS=1`.
The sequential acceptance wrapper additionally requires `--run-repeat` and
passes those explicit launch/flight controls to every bounded trial.

P0 requires installed dependencies and successful builds. P1 requires observed
simulator state and a measured takeoff/hover/land cycle. Runtime-free test
success, compilation, readiness metadata, and skipped checks cannot establish
flight success. Twenty consecutive measured successes remain a separate P1
acceptance requirement. Failures and interrupted runs must be retained.

The operator's subsequent P2 request authorizes one isolated ROS 2 control
owner through Micro XRCE-DDS, a Gazebo-to-ROS clock bridge, recording, and a
bounded position-Offboard takeoff, hover, east/north/yaw test, return and land
procedure. The existing four gates remain required for flight. A read-only
DDS connection stage precedes control. Console access is inspection only;
QGC retains monitoring/heartbeat only. P2 has its own initial flight and
20-consecutive-flight acceptance records, separate from P1. Failures remain
in the history. This is simulation-side control, not AgentOps flight authority.

P2-R1 authorizes the focused, versioned estimator-reference contract repair
described in [the pinned-source contract](v3_sim_p2_reference_contract.md),
historical offline classification, a new complete ROS smoke and a new
20-consecutive-flight sequence. One bounded, paired yaw-only initialization
event may be reconciled before reference lock. This explicitly revises
strict-v1 reset semantics; PX4/dependency versions, estimator parameters,
mission geometry and original flight limits remain fixed. The historical
aborted flight remains failed and contributes no acceptance credit.

P2-R2 authorizes initialization yaw ownership and a bounded handover under
`p2-estimator-reference-v3`. From the first prestream target, ROS supplies
finite position with yaw unspecified and zero world-z yaw feedforward.
Drift monitoring and the R1 reset classifier remain active. Stable alignment
precedes a fresh-heading-seeded handover to the original corrected anchor.
The source review, scheduling tests, labelled diagnostic, new nominal smoke
and fresh twenty-trial sequence are distinct gates. The R1 failed batch and
its one passed trial remain historical and contribute no R2 credit. All
original numeric flight limits, dependency pins and control ownership remain
unchanged; this authorization does not extend to P3 or AgentOps work.

The subsequent P3 request authorizes the pinned x500_depth camera, one-way
Image/CameraInfo bridging, a separate bounded observation/recording process,
independent ground calibration and one camera-equipped P2-profile smoke.
Three fixed consecutive coexistence flights are gated on that smoke passing
both independent control and sensor evaluation; qualification stops on its
first failure. It preserves the R2 yaw and R3 controller-only UDPv4 policies,
all original flight/timing limits and the historical x500 P2 acceptance.
This slice installs or upgrades no dependencies and adds no AgentOps authority.

[P3 implementation and ground validation](v3_sim_p3_validation.md) are recorded,
but its camera-equipped smoke failed strict timestamp-window checks and
qualification remains unrun, so P3 is incomplete. P4 non-learning avoidance,
P5 fixed-weight inference, P6 failure validation, P7 batch evaluation and
v3-2 onward remain unimplemented. Micro XRCE-DDS Agent is middleware, not an
AgentOps reasoning agent. P4 is not the recommended next execution step until
the declared P3 acceptance passes.

## Workspace and sources

The authoritative project remains `D:\GWM-UAV-Navigation-Sparse-Rewards`.
Upstream checkouts, build trees, Python environments, and raw evidence live
under `/home/joker0625/uav_autonomy` in Ubuntu-24.04 WSL2, not `/mnt/c` or
`/mnt/d`. No second edited project checkout is created.

The separately named `PX4_Gazebo_ROS2_自主決策與避障_模擬設計報告(1).md`
was unavailable in the provided attachment directory and project checkout.
The implementation uses the source-derived requirements in the operator's
pasted plan; it does not claim to have read that unavailable report.

See [validation](v3_sim_p0_p1_validation.md) and
[operator procedure](../simulation/px4_gazebo/README.md).
