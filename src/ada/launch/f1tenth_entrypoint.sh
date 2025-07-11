#!/bin/bash

cd /f1tenth_ws
source install/setup.bash
exec ros2 launch f1tenth_stack bringup_launch.py