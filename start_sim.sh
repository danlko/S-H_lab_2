#!/bin/bash
set -euo pipefail
# Автоматизація запуску симуляційного середовища (Завдання 2 за методичкою)
echo "Запуск симуляції у фоновому контейнері..."
docker compose up -d

echo "Збирання ROS-пакета (зміни у вихідному коді зберігаються у fpv_labs)..."
docker compose exec -T ardupilot_ros bash -lc "source /opt/ros/humble/setup.bash && source /home/ros/ros2_ws/install/setup.bash && cd /home/ros/ros2_ws && colcon build --symlink-install --packages-select fpv_lab2"

echo "Виконання лаунч-файлу симуляції всередині контейнера..."
docker compose exec ardupilot_ros bash -c "export PATH=\$PATH:/home/ros/.local/bin && export GZ_SIM_RESOURCE_PATH=/home/ros/ros2_ws/install/ardupilot_gazebo/share && source /opt/ros/humble/setup.bash && source /home/ros/ros2_ws/install/setup.bash && ros2 launch ardupilot_gz_bringup iris_maze.launch.py lidar_dim:=2 rviz:=false"
