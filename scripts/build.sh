#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/clean_env.bash"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$root"
# Build only the runtime source tree; keep diagnostics in the local dev folder.
colcon --log-base "$root/.dev/log/build" build --base-paths "$root/src" \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF "$@"
