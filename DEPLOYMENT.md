# Deploy to another workstation using the same hardware

This project builds its six ROS packages from the included source. It requires
no `~/colcon_ws` checkout or installation. External prerequisites are ROS Humble,
system/Python dependencies, the **Kinova Gen2 Linux SDK**, device access, and
your existing remote model service. An actual F/T sensor driver, if used, is
also installed/configured separately.

The tested baseline is Ubuntu 22.04, Python 3.10, ROS 2 Humble and x86-64. The
SDK libraries on the current workstation are x86-64 binaries; an ARM workstation
would need a compatible vendor SDK and separate validation. Do not mix ROS or
MoveIt distributions or copy compiled libraries from the old ROS workspace.

## 1. Install ROS and system tools

Follow the [official ROS Humble Ubuntu installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
Then install the build/dependency/test tools:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake python3-colcon-common-extensions \
  python3-rosdep python3-setuptools python3-pytest openssh-client
```

Initialize rosdep once if it has not been initialized, then update its cache:

```bash
sudo rosdep init  # only on a workstation without an existing rosdep setup
rosdep update
```

From the extracted project root:

```bash
./scripts/install_dependencies.sh
./scripts/install_dependencies.sh --check
```

The install script uses the manifests to provision build/runtime dependencies.
It does not install the Kinova SDK, create credentials, modify device rules, or
start hardware. The check mode is read-only. Test tools are installed by the
apt command above; upstream hardware tests are not run by this workflow.

## 2. Install the Kinova Gen2 SDK

Obtain/install the **Jaco Gen2 Linux SDK** using Kinova's instructions. This
project uses the legacy Gen2 USB API, not the Gen3 Kortex API. See the vendor's
[Jaco2/Gen2 documentation](https://github.com/Kinovarobotics/kinova-ros) for
hardware/SDK context; its ROS 1 driver installation is not required here.

The build requires:

- `Kinova.API.UsbCommandLayerUbuntu.h` and `KinovaTypes.h`.
- `Kinova.API.CommLayerUbuntu.so`.
- `Kinova.API.USBCommandLayerUbuntu.so`.

Standard installation paths such as `/usr/include` and `/usr/lib` are detected
automatically. If using an installation prefix of your choice, arrange headers
under `<prefix>/include` and libraries under `<prefix>/lib` or `lib64`, then:

```bash
export KINOVA_SDK_ROOT=/absolute/path/to/sdk-prefix
```

The build script preserves that variable and the CMake discovery accepts it.
The launch wrapper adds its `lib`/`lib64` directories to the runtime library
search path. Set it in each relevant shell or your deployment environment.
If changing SDK locations after a build, rebuild with `--cmake-clean-cache`.
The SDK libraries are intentionally not redistributed in the project archive.
A missing SDK produces a specific CMake error rather than a later linker error.

## 3. Build and verify the software

```bash
./scripts/build.sh
./scripts/check_install.sh
./scripts/test.sh
./scripts/smoke_mock.sh
```

These commands determine the project location from the scripts, not a username
or fixed Desktop path. Build/run/test/check wrappers reset inherited ROS
workspace paths and source only `/opt/ros/humble` and, where needed, this
workspace's `install/local_setup.bash`. They work even if your login shell has
previously sourced another ROS workspace.

The origin check requires all six packages to resolve inside this project's
installation. Tests and the mock smoke use domain 173 and localhost-only DDS;
reserve that domain for testing. The smoke test starts only GenericSystem mock
hardware, with cameras and SSH disabled. It validates controller startup and
an actual collision-aware MoveIt IK request. It does not test physical devices.

The build compiles the SDK-dependent hardware plugin, but does not initialize
or command a robot. Upstream hardware tests are disabled to avoid unreviewed
real-device interactions. The project's fake-service tests run separately.

## 4. Carry over the unchanged hardware profile

The bundled defaults preserve the current rig:

- Camera serials: `src/jaco_vla/config/hardware.yaml`.
- IMU calibration vectors: `src/ada_imu/config/imu_params.yaml`.
- Camera extrinsics: `src/ada_calibrate_camera/config/calibs/auto.yaml`.
- Robot geometry: `src/ada_description/urdf` and `meshes`.
- Finger calibration/action conventions: `src/jaco_vla/config/client.yaml`.

The camera serials and physical calibration can carry over when the exact same
robot, cameras, mounts and attachments are unchanged. Verify TF and image
orientation after connecting; recalibration is needed if the physical geometry
changes or those checks disagree with the carried-over configuration.

The workstation can assign a different IMU device number. Check:

```bash
ls -l /dev/serial/by-id/
```

Copy `hardware.yaml` into a deployment-specific profile and change
`imu_serial_port` to the corresponding `/dev/serial/by-id/...` path if one is
available. Pass `hardware_config:=/absolute/path/hardware.yaml` at launch.
A persistent udev alias is another option. The original `/dev/ttyUSB0` default
is retained for compatibility; it is not assumed stable across computers.

Install the Kinova SDK's USB-access rules and appropriate device permissions.
Kinova's [USB-access instructions](https://github.com/Kinovarobotics/kinova-ros#installation)
describe the vendor's udev rule. Configure serial-port access for the IMU and
RealSense device access as required by their installed drivers. Do not solve
permission errors by running the entire ROS stack as root.

If a real force sensor is present, install/configure its driver, verify fresh
`/wireless_ft/ftSensor3` measurements and tare it unloaded after reconnecting.
The project does not start a dummy force sensor. `require_force_sensor` remains
true by default; missing real data prevents enabling the client. The physical
force-sensor driver is an explicit external prerequisite, not a hidden ADA
workspace dependency.

## 5. Configure network access and start in dry run

Create the SSH host alias/credentials on the new workstation and verify that
it reaches the same model host. No private keys are stored in this project.
The default endpoint remains `http://127.0.0.1:8010/infer`; the SSH tunnel maps
that local port to port 8000 on the remote server. Change `server_url` in your
client YAML if the server arrangement differs.

From the project root, after checking no other stack controls the robot:

```bash
./scripts/run.sh sim:=real cameras:=true dry_run:=true \
  ssh_target:=your-vla-host-alias \
  hardware_config:=/absolute/path/hardware.yaml \
  instruction:='pick up the object'
```

This explicitly starts the real hardware driver; do not run it as a software-only
test. Omit `hardware_config` to use the bundled default device paths. Empty
`ssh_target` starts no tunnel. The wrapper preserves your SSH agent socket,
GUI display settings, ROS domain/localhost choice and optional middleware
selection, but discards old workspace/library paths and CycloneDDS URI settings.
If your deployment needs custom DDS configuration, configure it explicitly in
the wrapper rather than depending on a host-specific old-workspace XML file.

For manual ROS commands, use a fresh shell that has not sourced the old
workspace, then source `/opt/ros/humble/setup.bash` and the new
`install/local_setup.bash`. A shell that bypasses shell startup files can be
opened with:

```bash
env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  bash --noprofile --norc
```

Refer to README.md for enable/stop commands, the policy-contract review and
supervised commissioning. The client starts disabled. Dry run sends no arm or
hand trajectory goals, but starting the real driver still initializes hardware.
Only set `contract_verified: true` and launch with `dry_run:=false` after the
listed checks; a migration does not validate policy quality or physical motion.

## Known limits

No remote inference wrapper/checkpoint, credentials, vendor SDK binaries or
F/T driver is included. Automatic depth obstacle mapping is not implemented;
maintain the MoveIt planning scene for your workcell. The installed host MoveIt
has exhibited a shutdown segmentation fault even with a clean ROS environment.
Successful startup/IK checks do not establish clean upstream shutdown or
physical-motion safety. See VALIDATION.md for the actual test evidence.
