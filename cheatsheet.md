# System Bringup Command

## !!!Confirm joystick, camera and Lidar is connected. You can disconnect VESC for connecting wireless keyboard.
### Camera [Terminal 1]
```
sudo -i
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_rgbd:=true align_depth.enable:=true enable_sync:=true
```

### Lidar [Terminal 2]
```
sudo -i
(for ada1 only) ros2 launch sllidar_ros2 sllidar_s1_launch.py 
(for other group) ros2 launch sllidar_ros2 sllidar_s2_launch.py
```

### Lidar Transformer [Terminal 3]
```
sudo -i
ros2 run util lidar_transformer_node
```

### Control Center [Terminal 4]
```
sudo -i
ros2 run control_center control_center_node
```

### Start Visualization [Terminal 5]
```
sudo -i
ros2 run rviz2 rviz2 -d /home/adaX/NavOS/rviz/ada.rviz
```

### Localization [Terminal 6]
```
sudo -i
ros2 launch lidar_localization_ros2 lidar_localization.launch.py
```
If map is not shown in rviz2, restart the node.
After map is shown, use 2D Pose Estimate to localize the vehicle

### Pure pusit [Terminal 7]
```
sudo -i
ros2 launch pure_pursuit_controller pure_pursuit.launch.py
```

### Interactive Waypoints [Terminal 8]
```
sudo -i
ros2 run ada interactive_waypoints
```
You need to add a waypoint save function to save your waypoints to /home/adaX/NavOS/waypoints/YourTeamName.yaml.
Close this node after you finish waypoint recording.
In same terminal, then start path loader.
```
ros2 launch pure_pursuit_controller path_loader.launch.py waypoints_file:=/home/ada4/NavOS/waypoints/YourTeamName.yaml
```

### load path [Terminal 9]
After path loader is started
```
sudo -i
ros2 topic pub -1 /load_path std_msgs/msg/Bool "{data: true}"
```

### Stop Sign Stop [Terminal 9]
```
sudo -i
ros2 run util stop_sign_behavior_node
```

### Cone Slow Down (Optional if you developed)[Terminal 10]
Current node doesn't work functional, develop your own node by modifying /home/ada4/NavOS/src/util/util/cone_slow_behavior_node.py
```
sudo -i
ros2 run util core_behavior_node
```

## !!! Connecting VESC
### Lower Chassis Control and Perception [Terminal 11]
```
cd ~/NavOS/src/ada/launch
./start.sh
```
