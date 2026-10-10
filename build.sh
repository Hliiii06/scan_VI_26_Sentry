#!/usr/bin/env bash
# 构建 SCAN-Planner + 实车适配层（产物在 build/ install/，不污染系统）
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
echo
echo "构建完成。使用前请 source：  source $(pwd)/install/setup.bash"
