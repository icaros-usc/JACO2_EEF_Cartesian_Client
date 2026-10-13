# Jaco VLA clean workspace

A fresh ROS 2 Humble package for the KINOVA Jaco Gen2 6DOF/two-finger setup.
The VLA client and launch orchestration are rewritten. The ADA hardware driver, calibrated robot description, IMU publisher and
camera extrinsics are bundled as source packages and built in this workspace. No original source or configuration is changed.
This workspace does not contain, install, or train LeRobot or a VLA model.
See [DEPLOYMENT.md](DEPLOYMENT.md) for a new-workstation setup and
[UPSTREAM.md](UPSTREAM.md) for the bundled source provenance.

The defaults are **mock hardware, cameras off, dry run, client disabled**.
Physical motion has not been tested. Use the commissioning steps below before
selecting real hardware or enabling motion.

## Build and test

```bash
cd ~/Desktop/jaco_vla_clean
./scripts/build.sh
./scripts/test.sh
./scripts/smoke_mock.sh
```

`build.sh` builds all six packages using only `/opt/ros/humble`; it does not
source or rebuild the original workspace. The executable scripts clear inherited
workspace paths before loading ROS. They use system Python matching ROS Humble.
Install the Kinova Gen2 SDK separately, then provision system dependencies with
`scripts/install_dependencies.sh`. The SDK is not bundled. No pip install or
old-workspace underlay is required. `scripts/check_install.sh` verifies all six
package origins.

The test runner starts local HTTP and fake ROS services on **domain 173,
localhost only**, clearing inherited CycloneDDS interface settings. The mock
smoke test launches `mock_components/GenericSystem`, checks all three active
controllers, and solves a collision-aware IK request at the mock pose. Neither
script starts a real hardware driver or the physical cameras. Keep domain 173
reserved for these tests.

## Start the stack

Use `scripts/run.sh` to launch from a clean environment. For manual ROS
commands, start a fresh terminal and source these files from the project root:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

Do not source the old workspace in that terminal. If the shell's startup files
source it automatically, launch through `scripts/run.sh` or use a shell with a
clean environment as described in DEPLOYMENT.md.

Mock bringup:

```bash
./scripts/run.sh sim:=mock instruction:='pick up the object'
```

This starts the robot state publisher, MoveIt, mock IMU, camera extrinsics,
controller manager, and one grouped controller spawner. The client starts after
the spawner succeeds. Missing images/force data prevent enabling; default mock
bringup alone is not an end-to-end policy simulator. Use the test scripts for
the complete fake observation/inference loop.

For the real system, stop the old launch processes first. Do not run two
controller managers against the same robot. After commissioning, this command
starts the known components and optional SSH tunnel in one launch:

```bash
./scripts/run.sh \
  sim:=real cameras:=true dry_run:=true \
  ssh_target:=your-vla-host-alias \
  instruction:='pick up the object' \
  view_images:=true rviz:=true
```

Use an SSH alias configured with your actual account/host and working key
credentials. The tunnel binds local **127.0.0.1:8010** and forwards to remote
**127.0.0.1:8000**. Empty `ssh_target` starts no tunnel; use that if you already
have one. It uses argument arrays, not a shell, and requires noninteractive
SSH authentication. Tunnel exit shuts down the launch.

Camera serials and exposure settings match the supplied setup document. Both
camera namespaces are explicit: `/camera/camera` and `/camera/wrist`.
Serials and the IMU device path are in `src/jaco_vla/config/hardware.yaml`.
Pass another profile as `hardware_config:=/absolute/path/hardware.yaml`.
Exposure settings remain in the bringup launch. The image display windows are optional.

A real force/torque sensor driver must already publish `/wireless_ft/ftSensor3`
with fresh timestamps and calibrated readings. If the real driver provides
the original tare service, tare it while unloaded and wait for stable readings:

```bash
ros2 service call /wireless_ft/set_bias std_srvs/srv/SetBool '{data: true}'
```

There is no dummy force sensor in real bringup and no automatic tare. The old
`dummy_ft_sensor.py` generates simulated random force/torque data; its bias
service simulates a brief reduction in publishing rate. Those commands did
not provide measurements of contact forces on the physical robot.

Enable only after observations, TF, services, stationary joint velocity
feedback and (by default) force data are available:

```bash
ros2 service call /vla_client/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /vla_client/status
```

The default dry run performs inference, target generation, IK and sampled
collision checks, but sends **no arm or hand goals**. Each proposed delta is
validated from the currently measured pose. It is a diagnostic preview, not a
simulation of the cumulative policy rollout.

Stop with either service; both disable, invalidate queued/in-flight policy
responses and request cancellation of accepted goals:

```bash
ros2 service call /vla_client/stop std_srvs/srv/Trigger '{}'
ros2 service call /vla_client/enable std_srvs/srv/SetBool '{data: false}'
```

The Trigger response confirms that cancellation was requested, not that the
physical robot has stopped. Re-enabling waits for old goal results and HTTP
work to drain. After a fault, enabling again is an explicit operator action.
Ctrl-C attempts goal cancellation while the ROS context is still alive.
Hardware E-stop and independent supervision remain necessary: process death,
DDS failure or a driver failure can defeat application-level cancellation.

To attach only the new client to an already running, correctly configured
stack, use `client.launch.py`. It does not load or switch any controllers:

```bash
ros2 launch jaco_vla client.launch.py instruction:='pick up the object'
```

## Commission the policy and workcell

Review `src/jaco_vla/config/client.yaml` and copy it to a local deployment
configuration if needed. Pass that path as `client_config:=/absolute/path.yaml`.
Parameters are read-only after startup so changing frames, task or limits
cannot silently reinterpret an in-flight chunk. Restart the client to change
them.

1. Confirm the server's checkpoint and training pipeline. The actual remote
   HTTP wrapper was not available for inspection. The adapter matches the old
   client's keys and mathematics, not a verified training specification.
2. Confirm units, action normalization, camera preprocessing, EEF frame,
   translation/rotation frames, gripper sign, proprio layout and per-row
   action-step meaning. Normalized or absolute actions need a different adapter.
3. Calibrate finger open/closed values, camera extrinsics and timestamps.
   The reused ADA model includes a fork/force sensor assembly; verify that its
   collision geometry matches the physical attachments. No broad collision
   relaxation is applied.
4. Set workcell-specific Cartesian bounds and conservative motion/contact
   limits. Publish the table, fixtures and other obstacles into MoveIt's
   planning scene. The old octomap topic depended on feeding perception that
   this launch does not run; **automatic depth obstacle mapping is absent**.
   State validity checks only see geometry present in the planning scene.
5. Verify dry-run observations and targets against the training conventions.
   Set `contract_verified: true` in the deployment YAML only after this review.
6. Restart with `dry_run:=false`, keep the hardware E-stop available, and enable
   explicitly for a supervised small-motion trial. Do not treat offline tests
   as hardware validation.

The motion duration is a lower bound, not the policy's trained time step.
Arm and hand use the same trajectory duration and common future start stamp.
Each segment comes to rest, then the next action starts after successful
results and fresh stationary feedback. This prioritizes checked execution over
maximum-rate servoing. If your model requires a fixed high control rate, retime
or adapt the policy rather than assuming this stop-at-each-step executor meets
that requirement.

## HTTP and action contract

Request: POST `/infer` JSON with `image_b64`, `wrist_image_b64`, `instruction`,
`state`, and `return_chunk: true`. Images are RGB JPEG resized to 224×224 by
default. Row padding and supported RGB/BGR/RGBA/BGRA/mono encodings are handled
explicitly; invalid/unknown encodings fail instead of producing black images.

Response: `{"actions": [[dx, dy, dz, rx, ry, rz, grip], ...]}`.
Each row contains seven finite numbers. Translation deltas are metres; rotation
deltas are rotation vectors in radians. Defaults reproduce the old client's
base-frame translation and tool-frame rotation (`q_target = q_current * dq`).
`grip=-1` is open; `grip=+1` is closed. Oversized deltas and out-of-workspace
positions are rejected, not silently clipped.

`legacy_openpi_10` state is `[x,y,z, rotvec_x,rotvec_y,rotvec_z, four_policy_finger_values]`.
For measured closed fraction `g`, the last four values are:
`[0.02,-0.83,0.02,-0.83] + (1-g)*([1.51,0.93,1.51,0.93]-[0.02,-0.83,0.02,-0.83])`.
These are policy-domain values, not Jaco finger angles. The old comments called
this 8D; the code actually returned 10D. `pose_grip_7` is also available, but
select it only for a checkpoint trained with that layout.

## How asynchronous execution works

The ROS executor owns all state and ROS calls. A single background worker
encodes immutable observation snapshots and performs HTTP inference. It cannot
send commands or mutate the action queue. Timers keep servicing sensors, stop
requests, deadlines and ROS futures during inference.

A request captures a stationary action boundary and dispatch index N. While it
runs, pending old actions can continue. When its response arrives, K actions
may have been dispatched since N. The response's first K rows are discarded;
the remaining suffix **replaces** pending predictions, capped at queue capacity.
Relative deltas are never averaged or appended across overlapping chunks.
Stopped/faulted generations, expired responses, and expired queued actions
cannot be replayed. A new request starts below the refill threshold, only with
fresh frames from both cameras. There is at most one HTTP request in flight.

This alignment assumes each chunk row predicts the same next-action step index
from the captured observation. It is not wall-clock latency compensation or
real-time chunking, and predictions can diverge from actions executed while
inference ran. Validate this assumption against the checkpoint. With excessive
latency the queue can empty; the robot waits rather than replaying old actions.

The design takes prediction/execution separation and early refill from the
[LeRobot async tutorial](https://huggingface.co/docs/lerobot/async), without
installing LeRobot. Its action aggregation is intentionally not copied for this
relative-delta protocol.

Each action is checked through IK with collision avoidance and nearest-joint
constraints, then sampled full-robot arm/hand state validity along the joint
segment. Continuous joints use the nearest equivalent representation; bounded
joints are never wrapped or clamped. Start drift, branch jumps, stale inputs,
service timeouts, contact thresholds, HTTP errors, rejected goals and failed
results disable the client and clear pending actions.

Trajectory duration accounts for peak velocity and acceleration of a
rest-to-rest quintic. Both endpoints supply zero velocity and acceleration,
as specified by the [ROS 2 trajectory controller documentation](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html).
The full arm/hand path is sampled, not continuously certified. Scene changes,
tracking error, different goal start times and collisions between samples remain
limits of this check. This is not a hard real-time or certified safety controller.

## Controller and dependency decisions

One controller manager loads only `joint_state_broadcaster`,
`jaco_arm_position_controller` and `hand_controller`. They are activated as a
group. The bundled Jaco driver shares an arm/hand command mode; using position
interfaces for both avoids the old Servo/velocity/position switching sequence.
No Servo node, Cartesian controller, force-gate velocity controller or manual
controller-switch workaround is started.

The client monitors F/T while running, but the position JTC does not include the
old force-gate controller. Its checks depend on a running client and real force
measurements; hardware-level protection is separate.

The old workspace's `pick_ik` plugin had a MoveIt ABI mismatch. This deployment
uses the ROS Humble KDL plugin and does not require `pick_ik`, the feeding stack,
custom force-gate controllers or Cartesian controllers. The bundled ADA MoveIt
configuration also selects KDL. Legacy launch/teleoperation utilities and
camera-calibration tools that require those extra dependencies are omitted.

The host MoveIt executable has also exhibited a segmentation fault during
SIGINT shutdown after successful mock operation. Controller manager and the
new client exited cleanly. This upstream dependency issue remains unresolved;
check shutdown behavior before real deployment. The launch shuts down when
MoveIt, controller manager, client or the SSH tunnel exits unexpectedly.

## Layout and verification

- `src/ada_*`: bundled hardware/model/IMU/extrinsics runtime packages.
- `src/jaco_vla/jaco_vla/core.py`: geometry, contract, queue and timing.
- `transport.py`: image decoding and HTTP worker.
- `node.py`: ROS subscriptions, readiness, motion state machine and stop controls.
- `src/jaco_vla/config/`: controller, kinematics and client parameters.
- `src/jaco_vla/launch/`: complete bringup and client-only attachment.
- `tests/`: pure logic and fake ROS/HTTP integration tests.
- `scripts/`: isolated build, test and mock startup checks.

Validation covers queue overlap/expiry/generation invalidation; malformed
responses; image padding; geometry and continuous angles; quintic peak limits;
ROS callbacks during HTTP inference; dry-run suppression of goals; late
responses after stop; stale sensors; force thresholds; collision failure; and
arm/hand result handling. The mock smoke check uses actual ROS controllers and
MoveIt, with no physical devices. See `VALIDATION.md` for recorded results.
