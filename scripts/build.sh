#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/clean_env.bash"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$root"
# Upstream hardware tests are not invoked on a real-device workstation.
# The project tests and isolated mock smoke are run separately.
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF "$@"
