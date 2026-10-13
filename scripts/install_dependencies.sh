#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/clean_env.bash"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Install ROS 2 Humble first; see DEPLOYMENT.md.' >&2
  exit 1
fi
if ! command -v rosdep >/dev/null; then
  echo 'Install python3-rosdep, initialize rosdep, and run rosdep update; see DEPLOYMENT.md.' >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ${1:-} == --check ]]; then
  exec rosdep check --from-paths "$root/src" --ignore-src --rosdistro humble \
    -t build -t buildtool -t build_export -t buildtool_export -t exec
elif [[ $# != 0 ]]; then
  echo 'Usage: install_dependencies.sh [--check]' >&2
  exit 2
fi
# SDK installation is a separate manual prerequisite; see DEPLOYMENT.md.
exec rosdep install --from-paths "$root/src" --ignore-src --rosdistro humble \
  -t build -t buildtool -t build_export -t buildtool_export -t exec -y
