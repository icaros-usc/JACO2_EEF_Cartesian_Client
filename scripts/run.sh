#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/clean_env.bash"
if [[ ! -x "$root/.venv/bin/python" ]]; then
  echo 'Install dependencies first: ./scripts/install_dependencies.sh' >&2
  exit 1
fi
export PATH="$root/.venv/bin:$PATH"
if [[ ! -f "$root/install/local_setup.bash" ]]; then
  echo 'Workspace is not built. Run scripts/build.sh first.' >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "$root/install/local_setup.bash"
set -u
if [[ -n ${KINOVA_SDK_ROOT:-} ]]; then
  for sdk_libdir in "$KINOVA_SDK_ROOT/lib" "$KINOVA_SDK_ROOT/lib64"; do
    if [[ -d "$sdk_libdir" ]]; then
      export LD_LIBRARY_PATH="$sdk_libdir:${LD_LIBRARY_PATH:-}"
    fi
  done
fi
exec ros2 launch jaco_vla bringup.launch.py "$@"
