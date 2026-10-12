#!/usr/bin/env bash
set -euo pipefail

if (($# == 0)); then
  echo "Використання: ./ros2.sh <команда ROS 2> [аргументи]" >&2
  echo "Наприклад: ./ros2.sh node list" >&2
  exit 2
fi

cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
docker compose exec -T ardupilot_ros bash -lc \
  'source /opt/ros/humble/setup.bash && source /home/ros/ros2_ws/install/setup.bash && exec ros2 "$@"' \
  ros2-container "$@"
