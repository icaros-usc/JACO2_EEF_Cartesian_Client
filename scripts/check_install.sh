#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/clean_env.bash"
set +u
source /opt/ros/humble/setup.bash
source "$root/install/local_setup.bash"
set -u
/usr/bin/python3 "$root/scripts/check_install.py"
