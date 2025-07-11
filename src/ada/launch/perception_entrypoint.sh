#!/bin/bash

source /opt/ros/humble/setup.bash
cd /perception_ws
./src/util/setup.sh
colcon build --symlink-install
source install/setup.bash
exec ros2 run perception perception_node
