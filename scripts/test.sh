#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/clean_env.bash"
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="$root/src/jaco_vla:${PYTHONPATH:-}"
# Never discover or command the user's robot while testing.
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
# Do not inherit host CycloneDDS interface selections that conflict with localhost.
unset CYCLONEDDS_URI
export ROS_LOG_DIR="$root/log/tests"
mkdir -p "$ROS_LOG_DIR"
cd "$root"
/usr/bin/python3 -m pytest -q tests "$@"
