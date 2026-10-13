# Bundled ADA sources

Source: https://github.com/personalrobotics/ada_ros2
Snapshot base commit: `563d6e0e460f081ecad4e6bbfd32f937c7998d30`.
Copied from the existing local workspace on October 12, 2026. The five copied
packages had no Git changes; unrelated VLA client modifications were not copied.
Original license/copyright notices and robot assets are retained.

Bundled packages: ada_description, ada_hardware, ada_moveit, ada_imu,
ada_calibrate_camera. No original source is modified.

Local adaptations:
- Explicit Kinova SDK discovery and actionable build errors in ada_hardware.
- Runtime dependency declarations completed for the bundled entry points.
- ada_calibrate_camera contains the runtime extrinsics publisher/configuration
  only; camera-calibration tools/datasets and their dependencies are omitted.
- Deployment-relevant MoveIt data uses KDL and the clean position controllers.
  Legacy launch/teleoperation utilities requiring feeding/Servo/custom
  controllers are omitted from the extracted runtime package.
- Build/run/test scripts use only ROS Humble and this workspace.

Vendor changes should be reviewed against this base before upstream updates.
