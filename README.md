# Jaco VLA

ROS 2 Humble client for a remote VLA model and native Cartesian control of a
Kinova Jaco Gen2 6DOF/two-finger arm. Use Ubuntu 22.04, x86-64 and system Python 3.10.

## Setup

Install [ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
and the [Kinova Gen2 SDK](https://github.com/Kinovarobotics/kinova-ros), including
USB permissions. From the project root in Bash:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake python3-colcon-common-extensions \
  python3-rosdep python3-setuptools openssh-client
sudo rosdep init  # Skip if already initialized.
rosdep update
./scripts/install_dependencies.sh
```

For a nonstandard SDK installation, set `KINOVA_SDK_ROOT` to its prefix in each
build/launch terminal. The prefix must contain `include/` and `lib/` or `lib64/`.

Edit `src/jaco_vla/config/` for your setup:

- `cameras.yaml`: set serial numbers (keep the `_` prefix and quotes) and capture
  settings separately in the `overview` and `wrist` groups.
- `control.yaml`: set driver speeds, tool calibration and policy settings.
  Set `kinova_frame_type` to `fixed` (default) or `rotating`. Fixed preserves
  orientation during translation; rotating allows automatic orientation changes.
  The driver checks the actual mode at startup and rejects a mismatch.
  Set `base_position_world` (metres) and `base_quaternion_world` (xyzw) to the
  robot base pose in the policy's world frame.
  Set `action_translation_scale` and `action_rotation_scale` to convert returned
  action deltas to metres and radians; `1.0` leaves each unchanged.
  Images retain capture resolution; the server handles image preprocessing.
  The policy server is responsible for the task workspace and action magnitudes.

Connect exactly one arm, select the matching frame type in the Kinova SDK GUI,
then close the GUI before launching.
Initialize the fingers with the native Kinova joystick before launching; opening
and closing them initializes the hand. The driver rejects uninitialized fingers.
The driver selects the arm automatically. `run.sh` loads the
required configuration; missing settings cause a startup error.

```bash
./scripts/build.sh
```

Rebuild and restart after changing configuration.

## Launch and stop

Stop other robot drivers, then launch the arm and cameras. The policy server
can be offline; the local control utilities still start:

```bash
./scripts/run.sh \
  sim:=real view_camera_images:=true ssh_target:=<SSH ALIAS> \
  instruction:='<TASK INSTRUCTION>'
```

Camera drivers start by default. Use `launch_cameras:=false` if they already run
separately, and `view_camera_images:=true` to open raw camera viewers.

Add `gripper_xyz:='[0.2, -0.25, 0.5]'` to move to an initial gripper position in
metres in the robot base frame, preserving the starting orientation and finger
positions. The rollout waits for feedback within 5 mm and 5 degrees; failure to
reach the pose within 30 seconds stops startup. Omit it to start at the current
pose. `dry_run:=true` only previews this move. Positioning is excluded from recordings.

Add `trajectory_dir:=./rollouts` to record one episode per launch; the default
`trajectory_dir:=None` disables recording. Nintendo control needs no `instruction`;
recordings preserve an omitted instruction as an empty prompt.
Ctrl-C saves an `ep_00000/` directory (then `ep_00001/`, etc.) containing
`arrays.npz`, `metadata.json`, and an overview `rollout.mp4`, plus a dataset
`manifest.jsonl`, readable with `TrajectoryDataset` in `src/jaco_vla/src/dataset_utils.py`.
Each accepted action has the latest pre-action RGB images and world-frame state;
actions retain policy units before scaling. Optional server embeddings are stored
per query. Metadata records timing and the stop reason; task success is unknown
(`success: false`, `success_known: false`). Frames are buffered in memory until
shutdown. Dry runs and rejected goals add no steps.

For inference, run OpenPI's `scripts/serve_policy.py` with your trained policy
configuration and checkpoint. Add
`ssh_target:=your-vla-host-alias` to forward local port 8010 to remote port 8000,
or set `server_url` to the server's `ws://host:port` directly.
Launch uses the project-local `.venv` created by the dependency installer.

For Nintendo control, set `server_url: ws://127.0.0.1:8011` and `replan_steps: 1`
in `src/jaco_vla/config/control.yaml`. Start the local action server:

```bash
./.venv/bin/python src/jaco_vla/scripts/nintendo_server.py
```

In another terminal, launch the same arm/camera pipeline without SSH:

```bash
./scripts/run.sh sim:=real
```

The sticks use the data-collection mapping. ZL switches translation/rotation;
ZR toggles open/close; neutral sticks stop arm commands. Minus or disconnecting
the controller stops the server and faults the client. Cameras must still run.
Use Ctrl-C in the rollout terminal to stop the robot stack.

The eight-value `observation/state` is `[x, y, z, rx, ry, rz, left_finger, right_finger]`:
world-frame position in metres, a rotation vector in radians, and measured proximal
finger angles in radians (driver fingers 1, 2). Returned translation and rotation
deltas use world axes; the client converts them to the SDK base frame.
Normalized gripper predictions are clamped to `[-1, 1]` before finger calibration.
The policy receives RGB arrays as `observation/image` and `observation/wrist_image`,
and the instruction as `prompt`, matching OpenPI's LIBERO input adapter.

Inference starts automatically once observations and robot feedback are arriving.
The client discards each camera's first five seconds of frames to let colors
stabilize; set `overview_warmup_s` and `wrist_warmup_s` in `control.yaml` to adjust
these periods (zero disables warm-up). Each period starts at that camera's first
received frame; viewers show images immediately.
The client executes the first `replan_steps` actions from each prediction, then
queries the server again using the latest observations. Responses must contain at
least that many actions; remaining predictions are discarded. This setting does
not change the server's prediction length. The driver
sends zero arm velocity while inference is pending and pursues the latest gripper target.
The client stops if a required stream stops updating
for `sensor_timeout_s`. Each delta is added to the latest measured pose. The driver
converts the pose delta to a speed-limited Cartesian velocity for `action_duration_s`
(default 0.05). It streams `SendBasicTrajectory` every 5 ms, as in Kinova's Cartesian
control example, then sends zero arm velocity. Finger position targets remain active
until replaced or control is stopped; the driver tracks them with bounded finger velocities.
Success means the interval ended; firmware tracking and speed limits may prevent
reaching the target. ROS handoff adds a small timing gap.
`motion_timeout_s` is a fault timeout and must exceed `action_duration_s`.
Control stays active between goals. Heartbeat loss stops control within
`command_watchdog_s`, including between goals.
Allow policy warm-up within `server_timeout_s` (default 30). A server error or
timeout stops inference while control utilities stay up.
**Ctrl-C in this terminal stops the stack.**
After a fault, fix its cause and restart. `dry_run:=true` previews targets without
sending motion goals; with `sim:=real`, it still connects to hardware.

The Jaco firmware performs IK. MoveIt collision checking is absent; keep the
hardware E-stop available.
