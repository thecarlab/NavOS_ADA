# System Bringup Command
### Lower Chassis Control and Perception [Terminal 1]
```
cd ~/NavOS/src/ada/launch
./start.sh
```

### Camera [Terminal 2]
```
sudo -i
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_rgbd:=true align_depth.enable:=true enable_sync:=true
```

### Lidar [Terminal 3]
```
sudo -i
(for ada1 only) ros2 launch sllidar_ros2 sllidar_s1_launch.py 
(for other group) ros2 launch sllidar_ros2 sllidar_s2_launch.py
```

### Lidar Transformer [Terminal 4]
```
sudo -i
ros2 run util lidar_transformer_node
```

### Control Center [Terminal 5]
```
sudo -i
ros2 run control_center control_center_node
```

### Localization [Terminal 6]
```
sudo -i
ros2 launch lidar_localization_ros2 lidar_localization.launch.py
```

### Waypoint Recording (Optional if you need to record waypoint) [Terminal 7]
```
sudo -i
ros2 launch pure_pursuit_controller waypoint_recorder.launch.py
```

### Pure pusit [Terminal 8]
```
sudo -i
ros2 launch pure_pursuit_controller pure_pursuit.launch.py
```

### Path Loader [Terminal 9]
```
sudo -i
ros2 launch pure_pursuit_controller path_loader.launch.py
```

### Stop Sign Stop [Terminal 10]
```
sudo -i
ros2 run util stop_sign_behavior_node
```

### Cone Slow Down (Optional if you developed)[Terminal 11]
```
sudo -i
ros2 run util core_behavior_node
```
