#!/bin/bash

# Usage: ./bridge.sh [--debug] [launch arguments, e.g. log_level:=debug]
bridge_debug=false
if [[ ${1:-} == --debug ]]; then
  bridge_debug=true
  shift
fi
bridge_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || exit 1

source ~/catkin_ws/devel/setup.bash || exit 1
rossetmaster 133.11.216.48 || exit 1
rossetip || exit 1
source ~/colcon_ws/install/setup.bash || exit 1

if ! "$bridge_debug"; then
  exec ros2 launch "$bridge_dir/raw/ros1bridge.launch.xml" "$@"
fi

command -v gdb >/dev/null || { echo 'Debug mode requires gdb.' >&2; exit 1; }
bridge_log_root="${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}"
mkdir -p -- "$bridge_log_root" || exit 1
bridge_debug_dir="$(mktemp -d "$bridge_log_root/bridge-debug-$(date +%Y%m%d-%H%M%S)-XXXXXX")" || exit 1
export ROS_LOG_DIR="$bridge_debug_dir"
echo "Bridge debug log: $bridge_debug_dir/console.log"
# Stop at a fatal signal and record all threads before GDB exits.
# Keep normal logging by default: verbose logging can alter race timing.
set -o pipefail
ros2 launch "$bridge_dir/raw/ros1bridge.launch.xml" \
  "debug_prefix:=gdb --batch -ex 'set pagination off' -ex 'run' -ex 'thread apply all bt full' --args" \
  "$@" 2>&1 | tee "$bridge_debug_dir/console.log"
