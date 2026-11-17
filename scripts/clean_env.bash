# Sourced by executable entry points before any ROS setup. Re-exec without shell
# startup files or inherited workspace paths. Preserve only explicit user settings.
if [[ ${JACO_VLA_ENV_ISOLATED:-} != 1 ]]; then
  exec env -i \
    HOME="$HOME" USER="${USER:-}" PATH=/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin \
    LANG="${LANG:-C.UTF-8}" TERM="${TERM:-dumb}" \
    DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" SSH_AUTH_SOCK="${SSH_AUTH_SOCK:-}" \
    KINOVA_SDK_ROOT="${KINOVA_SDK_ROOT:-}" \
    RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
    CYCLONEDDS_URI="${CYCLONEDDS_URI:-}" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
    JACO_VLA_ENV_ISOLATED=1 bash --noprofile --norc "$0" "$@"
fi
