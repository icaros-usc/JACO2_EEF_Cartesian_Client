#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/clean_env.bash"
set +u
source /opt/ros/humble/setup.bash
source "$root/install/local_setup.bash"
set -u
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$root/log/mock_smoke"
unset CYCLONEDDS_URI
mkdir -p "$ROS_LOG_DIR"
/usr/bin/python3 "$root/scripts/smoke_mock.py"
