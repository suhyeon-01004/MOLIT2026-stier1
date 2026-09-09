#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
export DISABLE_ROS1_EOL_WARNINGS=1

/usr/bin/python3 "$ROOT_DIR/scripts/rviz_visualizer.py" --input "$ROOT_DIR/raw_mgeo" &
VISUALIZER_PID=$!
trap 'kill "$VISUALIZER_PID" 2>/dev/null || true' EXIT INT TERM
rviz -d "$ROOT_DIR/config/kcity_hd_map.rviz"
