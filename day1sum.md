# ROS2 Command

### Topics
```
ros2 topic list
ros2 topic info -v <topic_name>
ros2 topic pub <topic_name> <msg_type> "<data>"
ros2 topic pub -1 <topic_name> <msg_type> '<message_data>'
ros2 topic echo <topic_name>
```

### Run and Launch
```
ros2 run <package_name> <executable_name>
ros2 launch <package_name> <launch_file>
```

### Visualization
```
rviz2
rqt_graph
```

# ADA System

### Launch ada_system container
```	
cd ada_system
./run_container.sh
```

In the container
```	
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

### Launch control center
adax represent your vehicle's host name
```	
sudo -i
source /home/adax/NavOS/install/setup.bash
ros2 run control_center control_center_node
```
