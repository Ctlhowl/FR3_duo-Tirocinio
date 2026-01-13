#!/bin/bash

# Clone Franka dependencies into the workspace
vcs import /ros2_ws/src/franka_ros2 < /ros2_ws/src/franka_ros2/franka.repos --recursive --skip-existing

exec "$@"