#!/bin/bash

source /opt/ros/humble/setup.bash
cd /perception_ws
source install/setup.bash
exec ros2 run ada_perception perception_node