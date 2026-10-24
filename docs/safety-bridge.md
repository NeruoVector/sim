# Safety runtime and PX4 integration

The runtime uses the existing Ubuntu/ROS 2 Jazzy simulation image, keeping ROS
packages out of the Debian host. Conan 2.33.0 is installed as an isolated uv tool.
From the `sim` directory:

```bash
bash scripts/setup-safety-env.sh
make safety-test
```

Setup starts Docker Desktop if its user service is installed and no daemon is
reachable, starts the simulation container, mounts the sibling `drone` source at
`/drone` read-only, and builds the ROS package. The existing image must already
be present; on a new machine run `make build` first. It does not delete images,
containers or unrelated caches. Build jobs are limited to one: the 4 GB VM used
here could not sustain simultaneous Gazebo and unrestricted ROS C++ builds.
The source package path can also be supplied with
`-DNEUROVECTOR_DRONE_SOURCE_DIR=/path/to/drone` to colcon.

Host package verification:

```bash
conan create ../drone --build=missing --no-remote -s build_type=Release -s compiler.cppstd=20
```

## Responsibility map

| File | Owns |
|---|---|
| `snapshot.cpp` | Bounded typed ROS observation to library input conversion |
| `px4_state.cpp` | Actual PX4 state, NED-to-ENU conversion and estimator freshness |
| `command_channel.cpp` | PX4 interventions, command ACKs and bounded retries |
| `safety_bridge.cpp` | 100 Hz decision loop and mission failure latch |

The node retains an immutable observation message and converts its arrays only
when its sequence changes. Timer ticks reuse that typed snapshot; freshness is
still checked each tick. Shared managed-mode classification avoids divergent
state and command policies. Confirmed mode changes stop retries; retries use
wall time, one second apart, at most three per requested mode. No latency or
CPU improvement percentage has been measured.

The setup script preserves the Docker Desktop networking override on the
`desktop-linux` context. `colcon test --packages-select nv_safety_bridge` also
runs the registered DDS integration test after the package has been installed.

## Topics and ownership

`nv_safety_bridge` subscribes to `safety/observation` (`DeliveryObservation`) and
the version-matched PX4 position, attitude, status, land detection and battery
topics. Topic suffixes come from the installed `px4_msgs::MESSAGE_VERSION`;
this image exposes status `_v4`, local position `_v1`, battery `_v1`, and command
ACK `_v1`. Remap these topics for a different aircraft namespace.

Observation arrays are bounded by the library's existing limits. Observation
sequence numbers must strictly increase. All observation times use the bridge's
ROS clock; all survey, route, camera/depth and radio positions must share the
PX4 local ENU origin. PX4 local-origin resets invalidate the old mission frame
and latch delegation. Pose accuracy supplied by a producer must be a calibrated
error bound; PX4 `eph`/`epv` alone do not prove centimetre window accuracy.

Motor/thrust health, verified mission authorization, return/reserve energy,
watchdog read-back, camera/depth coverage, calibrated wind/IMU residuals, resolved
CSI paths, recipient and payload interlocks are required producer measurements.
They default to unsafe. Do not hardcode healthy flags or infer them from an empty
detection array. Refreshing a message header must not refresh an old PX4 estimator
sample. A missing/replayed/stale observation after the first observation, a denied
command or exhausted retry budget latches delegation until a new authorized
supervisor is started. Startup waits for initial observations without commanding
an unrelated active aircraft.

The node publishes `safety/decision` (`SafetyDecision`) and interventions on
`/fmu/in/vehicle_command`. `hold` requests AUTO/LOITER. A requested slowdown is
conservatively paused in LOITER because this bridge does not own a speed-tracking
route follower. Delegation requests AUTO/RTL when the aircraft is armed in a
managed mode; manual and native failsafe control have priority. ACK rejection is
reported and latches the mission. ACK acceptance alone is not a mode confirmation.
The actual status must report the requested mode to clear `command_pending`.

This bridge does not arm, disarm or release cargo. Confirmed contact and handoff
permissions remain advisory. Healthy decisions publish the authorized navigation
result; the mission controller remains responsible for route execution and must
stop its own setpoint/heartbeat stream on delegation. LOITER does not automatically
resume the old mission. Use one supervisor and one command owner per aircraft;
the installed parameter `source_component` defaults to 191.

Configure actual PX4 loss-of-Offboard/link handling, battery failsafes and return
behavior before flight. `independent_watchdog_enabled` must be verified read-back,
not merely the bridge process being alive. A bridge cannot execute its own command
after its process dies. Its producer and mission-controller contracts still apply.

## Verification

`make safety-test` runs real ROS 2 DDS transport on domains 73, 74 and 75. Only
these isolated domains publish simulated armed PX4 telemetry and command ACKs.
The cases cover window selection, gust-to-LOITER command/ACK, observation timeout
to RTL, failure latching, manual priority, rejected commands and replayed PX4
samples with fresh headers. These are transport/integration tests, not flight.

For actual, already disarmed PX4 SITL, start the DDS agent and headless simulation
in separate terminals using `make agent` and `make headless`, then run:

```bash
make safety-sitl-check
```

That observer checks real status/position messages and blocked decisions without
publishing aircraft state or sending arming commands. It fails on an armed vehicle
or unexpected commands. `make safety-bridge` starts the operational bridge; it
requires real observation producers to permit a mission.

No physical sensors, trained window detector, CSI capture/MUSIC estimator,
recipient recognition or payload hardware are created by this runtime. Those
hardware-specific producers remain necessary for physical flight.

## Observed verification

On this Debian 13 host, the setup script successfully recreated the Jazzy
container with its read-only drone source mount and Docker Desktop bridge
network. Re-running setup and the Release package build succeeded.
`colcon test` reported one registered integration test, zero failures/skips;
all three isolated DDS scenarios passed. The real disarmed PX4 SITL probe
received 3 status messages, 74 position messages and 899 blocked decisions,
with zero commands. The SITL/agent processes were stopped after the check;
the built container remains available.

Host checks also passed: strict Release and ASan/UBSan CTest suites 7/7,
42 safety regression cases, the two previously failing API reproductions,
Conan package creation/consumer safety API, and the school runner's 16 avoidance,
2 Rust/C++ route cross-check and 3 rejection cases. No physical flight was run.
