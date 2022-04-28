#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
#source /usr/share/gazebo/setup.sh
#source /usr/share/gazebo/setup.bash
source /app/ros2_ws/install/setup.bash
#printenv

exec "$@"