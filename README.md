# Friday Cheetsheet
Check cheetsheet.md file.

# Every Day Recap
1. Day 1 summarization is in day1sum.md

## NavOS System Launch
1. Control and Perception Container Launch

[Terminal 1]
```
cd ~/NavOS/src/ada/launch
./start.sh
```

2. Whole system launch

[Terminal 2]
```
sudo -i
source /opt/ros/humble/setup.bash
source /home/YOUR_USR_NAME/NavOS/install/setup.bash
source /home/YOUR_USR_NAME/sensor_ws/install/setup.bash
ros2 launch ada ada_system.launch.py
```

## Separate Modules and Sensors Launch

1. Start F1tenth System in the container
```
cd ada_system/
./run_container.sh
```
  In the container, do:
```
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```
2. Launch LiDAR sensor (local)
```
ros2 launch sllidar_ros2 sllidar_s2_launch.py
```
3. Launch Camera (local)
```
ros2 run realsense2_camera realsense2_camera_node
```
  More launch options for camera with IMU.
```
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2
```
