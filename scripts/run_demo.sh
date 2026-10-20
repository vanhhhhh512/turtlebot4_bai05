#!/usr/bin/env bash
set -eo pipefail

package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "$package_dir/../.." && pwd)"
source /opt/ros/humble/setup.bash

if [[ "${BAI05_SKIP_BUILD:-0}" != 1 ]]; then
  colcon build --symlink-install \
    --base-paths "$workspace_dir/src/bai04_turtlebot4" \
                 "$workspace_dir/src/bai04_turtlebot4/upstream" \
                 "$package_dir" \
    --packages-select irobot_create_nodes irobot_create_control \
                      bai04_turtlebot4 bai05_turtlebot4 \
    --allow-overriding irobot_create_nodes irobot_create_control \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
fi
source "$workspace_dir/install/setup.bash"

export ROS_DOMAIN_ID="${BAI05_ROS_DOMAIN_ID:-$((75 + $$ % 150))}"
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${BAI05_IGN_PARTITION:-bai05_$(date +%Y%m%d_%H%M%S)_$$}"
: "${DISPLAY:?An X11 desktop DISPLAY is required}"

run_dir="${BAI05_OUTPUT_DIR:-$HOME/bai05_runs/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$run_dir"
model="${BAI05_YOLO_MODEL:-$package_dir/models/yolo_world_furniture.torchscript}"
if [[ ! -f "$model" ]]; then
  printf 'YOLO model not found: %s\n' "$model" >&2
  exit 1
fi

python3 "$package_dir/scripts/arrange_windows.py" &
arranger_pid=$!
cleanup() {
  kill "$arranger_pid" 2>/dev/null || true
  wait "$arranger_pid" 2>/dev/null || true
}
trap cleanup EXIT

printf 'Run data: %s\n' "$run_dir"
printf 'DISPLAY=%s ROS_DOMAIN_ID=%s IGN_PARTITION=%s\n' \
  "$DISPLAY" "$ROS_DOMAIN_ID" "$IGN_PARTITION"
printf 'Press Ctrl+C to close Gazebo, RViz and YOLO after the route.\n'
ros2 launch bai05_turtlebot4 demo.launch.py \
  output_dir:="$run_dir" \
  yolo_model:="$model" \
  server_display:="${BAI05_SERVER_DISPLAY:-$DISPLAY}" "$@" \
  2>&1 | tee "$run_dir/launch.log"
