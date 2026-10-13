# Jaco VLA: deployment and inference guide

This guide covers installation, configuration, real-hardware inference and
operational logs for the Jaco Gen2 6DOF/two-finger robot. Commands assume you are
in the project root, such as `~/Desktop/jaco_vla_clean`, unless stated otherwise.
The project builds its ADA dependencies locally and does not need the old
`~/colcon_ws` workspace. ROS Humble, the Kinova SDK, device permissions and the
remote inference server are external prerequisites.

**The client starts disabled. Dry run performs inference and motion validation
without sending arm or hand trajectory goals. Starting with `sim:=real` still
initializes the real hardware driver, even in dry run.** Physical motion has not
been validated by the offline checks used to develop this project.

## 1. Install on a workstation

The supported deployment baseline is Ubuntu 22.04, ROS 2 Humble, system Python
3.10 and x86-64. Install ROS Humble using the official installation instructions,
then install the system build tools:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake python3-colcon-common-extensions \
  python3-rosdep python3-setuptools openssh-client
```

Initialize rosdep once, if it is not already initialized, and update its cache:

```bash
sudo rosdep init
rosdep update
```

From the project root, install and check the package dependencies:

```bash
./scripts/install_dependencies.sh
./scripts/install_dependencies.sh --check
```

Install the **Kinova Jaco Gen2 Linux SDK** separately using the vendor's
instructions. The required files are:

- Headers: `Kinova.API.UsbCommandLayerUbuntu.h` and `KinovaTypes.h`.
- Libraries: `Kinova.API.CommLayerUbuntu.so` and
  `Kinova.API.USBCommandLayerUbuntu.so`.

Headers/libraries in standard paths such as `/usr/include` and `/usr/lib` are
detected automatically. For a custom SDK prefix, put its headers under
`include` and libraries under `lib` or `lib64`, then set:

```bash
export KINOVA_SDK_ROOT=/absolute/path/to/sdk-prefix
```

Keep that variable set for building and launching. If its location changes
after a build, run `./scripts/build.sh --cmake-clean-cache`.
The current rig's SDK libraries are x86-64; an ARM workstation requires a
compatible SDK and separate validation.

Build all six bundled packages and confirm their installation locations:

```bash
./scripts/build.sh
./scripts/check_install.sh
```

These scripts use `/opt/ros/humble` and the current project, with inherited ROS
workspace paths cleared. Do not copy an old `build/` or `install/` directory to
the new workstation; copy source and build there. Building does not initialize
or command the robot.

Install the SDK's USB-access rules and configure serial/RealSense device access.
Run the stack as your regular user with the appropriate permissions.

## 2. Prepare configuration

Keep deployment-specific copies of the two main configuration files:

```bash
mkdir -p config
cp src/jaco_vla/config/hardware.yaml config/hardware.yaml
cp src/jaco_vla/config/client.yaml config/client.yaml
```

Edit these copies and pass their absolute paths at launch. Configuration is read
at startup; restart the client after changes. Its ROS parameters are read-only
during a run.

### Hardware and calibration

`config/hardware.yaml` contains the device identities:

```yaml
overview_serial: '147322072337'
wrist_serial: '845112070736'
imu_serial_port: /dev/ttyUSB0
```

Keep camera serials quoted as strings. The same cameras retain their serials
when connected to another workstation. The IMU device number can change; check
`/dev/serial/by-id/` and use the matching persistent path when available.

The carried-over physical calibration and model files are:

- IMU calibration: `src/ada_imu/config/imu_params.yaml`.
- Camera extrinsics: `src/ada_calibrate_camera/config/calibs/auto.yaml`.
- Robot geometry: `src/ada_description/urdf/` and `meshes/`.
- Camera exposure/profile settings: `src/jaco_vla/launch/bringup.launch.py`.

Calibration can carry over if the robot, cameras, mounts and attachments are
unchanged. Verify image orientation and TF after connecting. Changed physical
geometry requires a calibration/model review. This extracted project includes
the extrinsics publisher, not the original camera-calibration tools. If editing
bundled calibration/model files, rebuild before launching to ensure the installed
resources match them.

### Policy, limits and force sensing

`config/client.yaml` uses the following structure:

```yaml
vla_client:
  ros__parameters:
    server_url: http://127.0.0.1:8010/infer
    contract_verified: false
    proprio_layout: legacy_openpi_10
    translation_frame: base
    rotation_frame: tool
    require_force_sensor: true
    publish_debug_images: false
```

Preserve the other settings from the copied file. Review these before motion:

- **Policy contract:** default response rows are
  `[dx, dy, dz, rx, ry, rz, grip]`: translation deltas in metres, rotation-vector
  deltas in radians, and `grip` from -1 (open) to +1 (closed). Translation is in
  the base frame and rotation is in the tool frame. The default
  `legacy_openpi_10` state has ten values: position, rotation vector and four
  policy-domain finger values. Confirm these conventions, image preprocessing,
  end-effector frame and action-step semantics against the actual checkpoint.
- **Gripper:** verify `finger_open` and `finger_closed` against the physical
  Jaco finger calibration; defaults are `[0.0, 0.0]` and `[1.4, 1.4]` radians.
- **Workcell bounds:** customize `workspace_min` and `workspace_max`. The shipped
  bounds are defaults, not a surveyed safe volume for your table and fixtures.
- **Motion limits:** defaults reject translation deltas above 0.03 m, rotation
  deltas above 0.15 rad and arm joint steps above 0.35 rad. Default arm velocity
  and acceleration limits are 0.25 rad/s and 0.4 rad/s². Segments come to rest
  between actions; this is not a continuous high-rate Servo executor.
- **Freshness/latency:** `sensor_timeout_s` defaults to 0.5 s and
  `max_sensor_skew_s` to 0.15 s. `server_timeout_s` is 3 s. `action_ttl_s` is
  8 s from the observation snapshot. The action queue holds at most eight
  predictions and requests a refill below four. Do not increase these limits
  just to suppress faults without understanding the timing mismatch.
- **Force sensing:** with `require_force_sensor: true`, fresh real F/T data is
  required, including during dry run. The default force topic is
  `/wireless_ft/ftSensor3`; limits are 4 N and 1 Nm. Install/start the real sensor
  driver separately and tare it unloaded if supported. The project does not
  start a dummy sensor. Setting this option false removes the client's force
  check; it does not provide equivalent contact protection.

Only set `contract_verified: true` after checking the policy and hardware
conventions. This flag is required to enable motion with dry run disabled.
**The launch argument `dry_run:=...` overrides the YAML's `dry_run` value**;
pass it explicitly for each run.

## 3. Configure the remote model connection

Start the HTTP inference service on the model workstation. The client expects
POST `/infer` requests with `image_b64`, `wrist_image_b64`, `instruction`, `state`
and `return_chunk`, and responses containing an `actions` array. The remote
server/checkpoint is not included in this project.

Configure an SSH alias or use `user@host`, with noninteractive key authentication
working on the robot workstation. Passing `ssh_target` at launch starts a tunnel:

- Local endpoint: `http://127.0.0.1:8010/infer`.
- Forwarding: local port 8010 to the remote host's `127.0.0.1:8000`.

The tunnel binds only local loopback. If you already have the tunnel running,
omit `ssh_target`; otherwise two processes will compete for local port 8010.
If using a different endpoint, change `server_url` in your client configuration.

## 4. Launch inference in dry run

Stop the old robot launch processes first. Run only one controller manager
against the physical robot. Start the real F/T driver separately when required.

In terminal A, from the project root:

```bash
./scripts/run.sh \
  sim:=real cameras:=true dry_run:=true \
  hardware_config:="$PWD/config/hardware.yaml" \
  client_config:="$PWD/config/client.yaml" \
  ssh_target:=your-vla-host-alias \
  instruction:='pick up the object' \
  view_images:=true rviz:=true
```

Replace the SSH target and task instruction. `view_images` and `rviz` are
optional. The camera topics are `/camera/camera/color/image_raw` and
`/camera/wrist/color/image_raw`. Leave `calib` at its default `auto` to use the
carried-over camera calibration.

Bringup starts cameras when requested, robot TF/model components, MoveIt, one
controller manager and the position controllers. No manual controller switch
or Servo launch is needed. The client starts after controller activation but
**does not start inference until explicitly enabled**.

In terminal B, use a shell without the old workspace sourced, go to the project
root and prepare the ROS CLI:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

Use the same `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY` and `RMW_IMPLEMENTATION` settings
as terminal A. The launch wrapper preserves those variables, but clears old
workspace paths and custom CycloneDDS URI settings. If your shell startup files
automatically load the old workspace, first open a clean shell:

```bash
env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  bash --noprofile --norc
```

Then go to the project root, set any deliberate DDS settings, and source the two
setup files. With the robot stationary and fresh observations available:

```bash
ros2 service call /vla_client/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /vla_client/status
```

Check the service response. `success: false` means inference was not enabled;
its `message` explains the prerequisite that failed. During successful dry run,
the terminal will report `DRY RUN validated target ...; no goals sent` after
IK and collision validation. Preview actions are evaluated from the measured
pose; dry run is not a simulated cumulative policy rollout.

MoveIt solves IK and checks sampled arm/finger states along each proposed joint
segment. The client then sends validated trajectories directly to the arm and
hand controllers. It does not ask MoveIt to plan an alternative route around
an obstacle. Collision rejection disables the client.

## 5. Enable physical execution and stop

After reviewing dry-run behavior and completing supervised commissioning, stop
the client/launch, set `contract_verified: true` in your client configuration,
and repeat the launch command with **`dry_run:=false`**. Enable through the same
service. Keep the hardware E-stop available.

To stop from a prepared ROS terminal:

```bash
ros2 service call /vla_client/stop std_srvs/srv/Trigger '{}'
```

Alternatively:

```bash
ros2 service call /vla_client/enable std_srvs/srv/SetBool '{data: false}'
```

Both disable the client, clear queued actions, discard late inference responses
from the previous run, and request cancellation of accepted trajectory goals.
A successful service response confirms the cancellation request, not a verified
physical stop. Re-enabling waits for previous HTTP work and goal results to
drain. Use the hardware E-stop if software stopping is ineffective.

Ctrl-C in terminal A shuts down the launch and attempts goal cancellation.
Ctrl-C in a terminal running `topic echo` stops only that observer; the client
continues running. Stop the observer or use another prepared terminal for the
stop service. A fault requires resolving its cause and explicitly enabling again.

## 6. Status, logs and recording

### Live status

`/vla_client/status` publishes a JSON string once per second:

- `enabled` and `dry_run`: whether the client is enabled and whether goals are suppressed.
- `phase`: `idle`, `ik`, `collision`, `executing` or `settling`.
- `fault`: the most recent stop/fault reason; an operator stop also populates it.
- `queue`: the number of pending actions, excluding the currently selected action.
- `requests` and `responses`: inference counters since this client process started.
- `inference_latency_s`: last accepted response's elapsed time from its observation
  snapshot, including client processing and network/server time.
- `inference_pending`: whether an HTTP worker request is still outstanding.

The launch terminal reports startup, inference latency and queue length,
dry-run validations, controller/IK failures, timeouts and stop reasons. The
client does not automatically save full policy action arrays, request payloads,
trajectories or a CSV session history.

### Log locations

ROS launch prints its log directory near startup. With the provided wrapper,
ROS runtime logs normally go under `~/.ros/log/`; launch creates a session
directory, and ROS nodes may also create individual log files under that base.
The project's `log/` directory contains colcon build logs, not a guaranteed
inference-session recording. Remote model logs remain on the model workstation.

`run.sh` clears inherited `ROS_LOG_DIR` and `ROS_HOME` settings, so setting those
variables before invoking the wrapper will not redirect its logs. To keep a
complete copy of the launch terminal output, use:

```bash
mkdir -p logs
session_log="logs/inference_$(date +%Y%m%d_%H%M%S).log"
set -o pipefail
./scripts/run.sh \
  sim:=real cameras:=true dry_run:=true \
  hardware_config:="$PWD/config/hardware.yaml" \
  client_config:="$PWD/config/client.yaml" \
  ssh_target:=your-vla-host-alias \
  instruction:='pick up the object' \
  2>&1 | tee "$session_log"
```

This is an alternative to the terminal-A launch command, not a second launch.
It records console output; it does not record images or all ROS messages.

### Optional images and ROS bags

Set `publish_debug_images: true` in the client YAML before launch to publish
`/vla_client/sent_overview` and `/vla_client/sent_wrist`. These are original
camera-message snapshots used for requests, before worker resizing/JPEG encoding.
They are ROS topics, not files automatically written to disk.

For explicit status/joint/TF recording, run this in a prepared ROS terminal:

```bash
mkdir -p logs
ros2 bag record -o "logs/inference_bag_$(date +%Y%m%d_%H%M%S)" \
  /vla_client/status /joint_states /tf /tf_static
```

Install `ros-humble-rosbag2` if the `ros2 bag` command is unavailable. Add the
`sent_*` image topics when debug publishing is enabled if you need observation
snapshots; images substantially increase storage. Stop the recorder separately
when the run ends. A bag of these topics does not contain HTTP response bodies.

## 7. Common issues and limits

- **Enable fails:** inspect the service's `message`. Check a nonempty instruction,
  fresh images and complete joint position/velocity feedback, TF, MoveIt services,
  stationary state and real F/T data when required. The motion-enabled mode also
  requires both trajectory action servers and `contract_verified: true`.
- **Missing/stale images or clock mismatch:** verify camera serials and topics,
  timestamps and device access. One stale camera is enough to stop the client.
- **Inference HTTP error/deadline:** inspect the SSH tunnel, local port 8010,
  `/infer` endpoint, remote server logs and actual response shape/units. Tunnel
  exit shuts down the launch. A malformed response disables the client.
- **Queue expiry or starvation:** compare inference latency with execution time.
  Already-dispatched predictions are skipped; expired predictions are not
  replayed. An empty queue waits for usable predictions; an expired queued action
  faults the client. Async inference does not guarantee uninterrupted motion.
- **IK/collision/branch-jump fault:** check tool/frame conventions, target units,
  current robot configuration and collision geometry. The solver is KDL; rejected
  targets are not silently clamped or replanned around obstacles.
- **No automatic obstacle mapping:** publish the table, fixtures and other
  obstacles into MoveIt's planning scene. The launch does not provide depth-based
  obstacle updates. Collision checks are sampled and depend on the scene's accuracy.
- **Motion/contact protection:** the position trajectory controllers do not include
  the old force-gate controller. The client's F/T checks and cancellation depend
  on the client, DDS and driver remaining operational; hardware protection is separate.
- **Shutdown issue:** the installed host MoveIt executable has produced a
  segmentation fault during SIGINT shutdown. Check that all processes exited and
  the robot stopped before restarting. This upstream issue remains unresolved.
