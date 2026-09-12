# P3 depth observation contract

P3 implements measured depth only. The Windows project is authoritative;
Linux build mirrors and raw evidence stay beneath
`/home/joker0625/uav_autonomy`. P2's accepted x500 smoke and 20-flight history
are immutable, separately referenced evidence. No historical P2 flight is a
depth-model flight. The standalone simulation design report was unavailable;
the operator's supplied requirements define this slice.

## Pinned model and actual renderer

PX4 is `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`; its Gazebo models submodule
is `b6127f4ec20de867e215fb5f78ae88b80f371909`. The include chain is
`x500_depth -> x500 -> x500_base`, plus merged `OakD-Lite`. Every source
asset, generated world and resolved SDF is hashed in the ground manifests.
Upstream files and vehicle dynamics are unchanged. Airframe
`4002_gz_x500_depth` sources `4001_gz_x500`.

The four rotors each contribute 0.016076923076923075 kg to the 2 kg base.
OakD adds 0.061 kg, giving 2.1253076923076923 kg total. Its inertia diagonal
is `[0.0000460804, 0.0000055421, 0.0000436519]` kg m², with zero products of
inertia, at camera-link offset `[0.00358,-0.03,0.014]` m. The camera has a
fixed joint and its native collision box. No sensor-only vehicle overlay
was necessary; project worlds add only independent static fixtures.

StereoOV7251 retains 640x480, `R_FLOAT32`, 30 Hz, horizontal FOV 1.274 rad,
and near/far settings 0.2/19.1 m. Native IMX214 remains 1920x1080, 30 Hz,
FOV 1.204 rad and 0.1/100 m clipping. RGB and point clouds are not bridged.
The native RGB camera resource still exists. The installed CameraSensor
implementation skips RGB image generation without image consumers/save
requests; omitting a bridge alone is not a general guarantee about rendering
work. The native sensor model is retained in all reported profiles.

Installed versions are gz-sim 8.15.0, gz-sensors 8.2.2, gz-rendering 8.2.3,
ROS Jazzy and ros_gz_bridge 1.0.24. PX4's server configuration loads
`gz::sim::systems::Sensors` with Ogre2. Actual ground Ogre logs identify
Mesa llvmpipe (LLVM 20.1.2), OpenGL 4.5. The selected coexistence mode runs
the Gazebo server through WSLg without a Gazebo GUI, with QGC offscreen and
its normal monitor heartbeat. This is real depth rendering, not physics-only
evidence. A separate post-window 4-to-5 m fixture move verifies changed
pixel data. Static calibration windows can legitimately be byte-identical.

## Complete extrinsics

All listed SDF rotations are zero. Include poses must be composed before
choosing the body origin; a joint pose does not replace a link pose.

| Frame/origin | Translation in outer x500_depth model, m |
|---|---|
| merged x500 | `[0,0,0]` |
| merged x500_base / base_link | `[0,0,0.24]` |
| OakD-Lite / camera_link | `[0.12,0.03,0.242]` |
| StereoOV7251 sensor / optical center | `[0.13233,0,0.26078]` |

Thus the optical origin in body FLU is **`[0.13233,0,0.02078]` m**.
The raw header remains `camera_link`; it neither identifies the optical
origin nor changes the pixel axes. The adapter supplies normalized frame
`gwm_front_depth_optical`, extrinsic identity
`b6127f4-x500-depth-optical-to-base-v1`, and actually transformed points:

```text
optical [X right, Y down, Z forward]
body FLU = [Z + 0.13233, -X, -Y + 0.02078]
body FRD = [FLU.x, -FLU.y, -FLU.z]
```

There is no TF publisher and no relabelled raw image. Consumers must use the
declared optical convention and transform, rather than interpreting the raw
`camera_link` header as an authoritative optical TF.

## One-way transport and calibration

The private PID/network namespace contains only loopback; DDS domain is 71,
PX4 instance 71/system 72/component 1, ROS PX4 prefix `/px4_71`, model
`x500_depth_71`, and Gazebo partition the unique run ID. Only explicitly
registered ground and flight worlds are accepted. The existing exclusive
P1/P2 workspace lock also protects P3. No graph-wide camera discovery occurs.

| Owned Gazebo topic/type | ROS topic/type |
|---|---|
| `/depth_camera`, `gz.msgs.Image` | `/gwm/sensors/front_depth/image_raw`, `sensor_msgs/msg/Image` |
| `/camera_info`, `gz.msgs.CameraInfo` | `/gwm/sensors/front_depth/camera_info`, `sensor_msgs/msg/CameraInfo` |

Both bridges are GZ_TO_ROS, eager, SENSOR_DATA/BEST_EFFORT, VOLATILE, with
configured queues of eight. Runtime graph inspection reports one image
publisher and one adapter subscriber; its depth display is UNKNOWN, so the
configured depth is not misreported as a graph measurement. RGB CameraInfo
has a different scoped topic. One existing clock bridge maps the owned
world clock to `/clock`; P3 does not add a second clock source.

The first transport experiment used default middleware and lost frames
between Gazebo and ROS. `p3-x500-depth-native-shm64-v1` explicitly applies
`p3_fastdds.xml` only to the sensor bridge and adapter: UDPv4 discovery plus
64 MiB SHM segments and port capacity 64. Image settings and acceptance
limits do not change. P2 retains its controller-only UDPv4, skipped default
XML and synchronous publication. The large-image profile never applies to
the controller, DDS Agent, QGC or clock bridge.

Observed depth is little-endian `32FC1`, step 2560, 1,228,800 bytes/frame.
The pure decoder supports explicit endian/row padding within bounded sizes;
the registered native runtime additionally rejects deviation from its exact
layout. 16UC1 and other encodings are deliberately unsupported.

Observed depth CameraInfo has fx=fy=432.496042035043, cx=320, cy=240,
P's intrinsic part equal to K, zero translation, identity rectification,
zero plumb_bob distortion, zero ROI and binning. The actual static identity
is `9f0816e89913bd9e8785cb403ce3f13b3f216cf9cc5cee28f9aa01c5538d7e6d`.
The first raw header and calibration values are retained. Each subsequent
CameraInfo is validated against that identity; mismatch/incompatibility
invalidates the cache for the run. Static validity is explicit and does not
pretend an old CameraInfo stamp is a fresh image acquisition.

## Depth and observation semantics

For the observed undistorted calibration, P supplies
`X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`, `Z=depth_m`. Independent ray/box and
ray/plane tests establish optical-axis depth rather than Euclidean range.
The installed Ogre2 shaders use radial length for far clipping and forward
depth for near clipping; the far number is not a proof of free space along
every optical ray. The observed out-of-range scene produces +Inf. NaN is
invalid, -Inf is too-close, finite values below 0.2 are too-close, and
finite values at/above 19.1 or +Inf are no-return/above-range. No nonfinite
value is filled, interpolated or replaced with the far range. Live NaN,
-Inf and 16-bit sensor operation are not claimed from synthetic tests.

`Reason` masks distinguish valid, invalid, too-close, no-return, unobserved,
stale, calibration-unavailable and transform-unavailable. Dense masks are
pure computed arrays; observations carry counts and 35 fixed measured pixel
samples with individual reasons. Missing points and state references are
JSON null. Unobserved/out-of-view regions are unknown, never free corridors.
When a whole frame is unavailable, health/errors identify that state and no
fresh observation is republished from the old frame.

The dedicated schema-1 DepthObservation is transported as strict JSON in
`std_msgs/String` on `/gwm/sensors/front_depth/observation`; health has a
separate `/gwm/sensors/front_depth/health` topic. IDs link observations to
raw records, acquisition/receipt/processing timestamps, raw/normalized
frames, calibration/extrinsic identities, reason counts, measured optical/
FLU/FRD points, source age and PX4-state association. It contains no action,
planner output, hidden obstacle map or Gazebo truth. Existing
`SensorObservation` assumes Unix time and default obstacle-distance values;
its semantics and AgentOps `AgentObservation` are not changed or reused.

The run ID is the enclosing unique run directory even when raw records live
in its `sensors` child. The recorded pre-review coexistence package emitted
`sensors` in that field; its enclosing hashed run retains attribution. The
final correction is unit/build-verified, with no fresh runtime acceptance.
See the validation report for exact package and evaluator versions.

For coexistence, a 300-item PX4 position history chooses the closest
acquisition-time state within 0.05 s. It records position, heading, validity,
source timestamp and independent reset counters. It does not interpolate;
a bracket spanning different epochs is rejected. Ground calibration needs
no PX4 association and states that explicitly. Truth/fixture labels remain
in the evaluator only. No camera data feeds PX4 estimator fusion.

## Bounded resources and acceptance

Processing targets 10 Hz in simulation time using a phase accumulator,
polled every 20 wall ms. Missed periods select one newest frame, never a
catch-up backlog. Health is checked every 0.1 wall s. The newest-frame slot
is one item; intentional overwrites are counted. The sole raw recorder has
an eight-frame queue (9.83 MB raw payload), exclusive binary/index ownership,
per-frame hashes, contiguous offsets, write timestamps, bounded drain and
fsync completion. Recorder error/overflow invalidates evidence. Two sensor
participants each reserve a bounded 64 MiB transport segment; this is not
a bound on all Gazebo, DDS, GPU, OS or process memory.

The raw rate is about 37.2 MB per simulation second. A 20 GiB disk reserve
is checked before measurement/flight readiness. Images are never inserted
in P2's event ledger or callbacks. A separate evaluator-only Gazebo header
probe records source sequence IDs and acquisition times. Acceptance requires
zero source-to-raw-record loss within the declared window, as well as exact
received/index/binary counts. Processing decimation has no raw-record credit.

The fixed ground settings are in `p3_validation.yaml`: 30 simulation seconds,
ROI `[160,80,480,200)` (upper interior, excluding floor/edges), valid fraction
>=95%, median error <=max(0.05 m,1%), p95 <=max(0.10 m,2%), source gap <=0.12 s,
delivered rate >=25 Hz, processed rate >=8 Hz, source age <=0.25 s, and no
buffer overflow. Sky/out-of-range is evaluated for correct unknown behavior,
not the plane-valid-fraction criterion. Interruption must become stale or
unavailable within 1.2 wall s; its expected failure is not nominal depth credit.

P2 controller ownership, yaw policy v3, NaN initialization yaw, handover,
mission geometry, ACK/LAND behavior, 0.2 s consumed-gap limit and 50 ms
dispatch budget remain unchanged. The only controller-package extension is
an explicit second model/world/profile identity. Perception never creates
VehicleCommand, TrajectorySetpoint, OffboardControlMode or obstacle-distance
control publishers. No P4/P5/P6/AgentOps/hardware capability is added.

## Reviewed primary references

- [PX4 Gazebo vehicles](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)
- [Jazzy ros_gz_bridge](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)
- [REP 118](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst),
  [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
- [Jazzy Image](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Image.msg),
  [CameraInfo](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/CameraInfo.msg)
- [gz-sensors 8.2.2](https://github.com/gazebosim/gz-sensors/tree/gz-sensors8_8.2.2),
  [gz-rendering 8.2.3](https://github.com/gazebosim/gz-rendering/tree/gz-rendering8_8.2.3)
- [Fast DDS 2.14.6 SHM](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/transport/shared_memory/shared_memory.html)

Actual pinned local SDF, installed headers/shaders and recorded samples take
precedence over examples from newer documentation. No dependencies were installed
or upgraded for P3; clean rebuild remains unproven.
