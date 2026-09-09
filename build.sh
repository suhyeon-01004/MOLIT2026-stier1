#!/bin/bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Keep ROS Noetic on the Ubuntu toolchain even if Homebrew is earlier in the
# interactive shell PATH.
export PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/noetic/setup.bash
cd "$WORKSPACE_DIR"

catkin_make install \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_LINKER=/usr/bin/ld \
  -DCMAKE_AR=/usr/bin/ar
