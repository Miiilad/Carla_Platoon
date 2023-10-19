#!/usr/bin/env bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /home/docker/carla-ros-bridge/install/setup.sh

exec "/usr/local/bin/fixuid" "-q" "$@"