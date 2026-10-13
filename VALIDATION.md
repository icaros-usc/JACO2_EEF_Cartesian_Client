# Recorded validation — October 12, 2026

The workspace now bundles `ada_description`, `ada_hardware`, `ada_moveit`,
`ada_imu`, `ada_calibrate_camera` (runtime extrinsics only) and `jaco_vla`.

- Clean build: all **six packages** built with only `/opt/ros/humble` as the
  ROS underlay, using the installed Kinova SDK. No original-workspace binaries
  or Python packages were sourced. The system setuptools emitted an existing
  package-version deprecation warning.
- `install_dependencies.sh --check`: all manifest-based system dependencies
  satisfied, with no unresolved dependency keys. No apt installation was run.
- `check_install.sh`: all six package prefixes resolve inside the new workspace.
  Generated setup files contain no `~/colcon_ws` or temporary-workspace chain.
- `test.sh`: **22 tests passed** in the isolated fake ROS/HTTP environment.
- `smoke_mock.sh`: the three actual mock controllers activated; actual
  collision-aware MoveIt IK succeeded at the mock pose; the client stayed
  disabled in dry run. Cameras, SSH and real hardware drivers were off.
- Relocation check: exported source was unpacked into a different directory,
  all six packages rebuilt from scratch, all package origins resolved to that
  relocated install, and the actual mock controller/IK smoke check passed there.
  This checks source relocation on this host; it is not a fresh-workstation test.
- Python compilation, shell syntax and six package XML parses passed.
- SHA-256 checks of **213 pre-existing files** in the five original ADA source
  packages found **zero changes**. Original source and install trees were not
  edited. Bundled copyright/license notices and robot assets are retained.

The workspace uses the ROS Humble KDL solver; it does not depend on the old
pick_ik plugin or its incompatible MoveIt binary. It also excludes the feeding
stack, custom force-gate/Cartesian controllers and calibration-tool dependencies.

The host MoveIt process still produced a segmentation fault during SIGINT
shutdown with the clean environment. The controller manager and VLA client
exited cleanly. Startup/IK PASS does not imply clean shutdown of that upstream
executable. Its error about absent 3D octomap sensors is consistent with the
explicitly documented absence of automatic depth obstacle mapping.

Not validated: another workstation's OS/SDK installation, physical robot motion,
real camera/sensor operation, remote inference behavior, checkpoint semantics,
policy quality or timing suitability. Kinova SDK installation and an actual
F/T sensor driver remain explicit external prerequisites. No SDK binaries,
credentials or remote model are included in the source archive.
