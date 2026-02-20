#!/usr/bin/env bash
# Run Gazebo Sim with the Mid360 test world.
# Usage (standard colcon): from workspace root: source install/setup.bash && $0
# Usage (this repo Makefile): from repo root: $0   or  $0 /path/to/ros2-build

set -e
WORKSPACE="${1:-.}"

SETUP=
if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  SETUP="$WORKSPACE/install/setup.bash"
elif [[ -f "$WORKSPACE/out/$(lsb_release -cs)/$(dpkg --print-architecture)/install_sim/setup.bash" ]]; then
  SETUP="$WORKSPACE/out/$(lsb_release -cs)/$(dpkg --print-architecture)/install_sim/setup.bash"
fi

if [[ -z "$SETUP" ]]; then
  echo "No setup.bash found. Either:"
  echo "  source install/setup.bash   # then run this script from workspace root"
  echo "  $0 /path/to/workspace        # repo root with out/<codename>/<arch>/install_sim"
  exit 1
fi

source "$SETUP"

# Same domain as this repo's Makefile so RViz/other nodes can see /points
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-32}"

# Add plugin lib so gz-sim can load mid360_gz_plugin
PKG_PREFIX="$(ros2 pkg prefix mid360_gz_plugin)"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:${PKG_PREFIX}/lib:${PKG_PREFIX}/lib/mid360_gz_plugin"

WORLD="$(ros2 pkg prefix mid360_gz_plugin)/share/mid360_gz_plugin/worlds/mid360_test.world"
echo "Starting Gazebo Sim with world: $WORLD"
echo "PointCloud2: /points (frame_id: lidar_link). ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RViz: export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && rviz2  → Add PointCloud2, topic /points, Fixed Frame: lidar_link"
gz sim -s -v 4 "$WORLD"
