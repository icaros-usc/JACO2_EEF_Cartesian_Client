#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/clean_env.bash"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Install ROS 2 Humble first; see README.md.' >&2
  exit 1
fi
if ! command -v rosdep >/dev/null; then
  echo 'Install python3-rosdep, initialize rosdep, and run rosdep update; see README.md.' >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ $# != 0 ]]; then
  echo 'Usage: install_dependencies.sh' >&2
  exit 2
fi
# SDK installation is a separate manual prerequisite; see README.md.
rosdep install --from-paths "$root/src" --ignore-src --rosdistro humble \
  -t build -t buildtool -t build_export -t buildtool_export -t exec -y
# Keep OpenPI's NumPy constraint local, compatible with Humble's SciPy.
/usr/bin/python3 -m venv --system-site-packages "$root/.venv"
"$root/.venv/bin/python" -m pip install \
  'openpi-client==0.1.2' 'websockets==15.0.1' 'websocket-client==1.9.0' \
  'pygame==2.6.1' \
  'imageio==2.37.0' 'imageio-ffmpeg==0.6.0' \
  'numpy>=1.23.5,<1.25'
