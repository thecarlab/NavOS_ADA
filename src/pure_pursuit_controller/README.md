# Pure Pursuit Controller for ROS2

A ROS2 implementation of the Pure Pursuit path tracking algorithm with bidirectional movement support. This package listens to NDT localization poses and follows pre-recorded waypoints.

## Features

- **Pure Pursuit Algorithm**: Implements the classic pure pursuit path tracking algorithm
- **Bidirectional Movement**: Supports both forward and reverse movement along paths
- **Adaptive Lookahead**: Dynamic lookahead distance based on vehicle velocity
- **Waypoint Recording**: Tools to record and save waypoints from live poses
- **Path Loading**: Load and replay saved waypoints
- **ROS2 Native**: Built for ROS2 Foxy with modern C++ standards

## Package Structure

```
pure_pursuit_controller/
├── include/pure_pursuit_controller/
│   └── pure_pursuit_controller.hpp    # Main controller class
├── src/
│   ├── pure_pursuit_controller.cpp    # Controller implementation
│   ├── pure_pursuit_main.cpp          # Main executable
│   ├── waypoint_recorder.cpp          # Waypoint recording utility
│   └── path_loader.cpp                # Path loading utility
├── launch/
│   ├── pure_pursuit.launch.py         # Launch pure pursuit controller
│   ├── waypoint_recorder.launch.py    # Launch waypoint recorder
│   ├── path_loader.launch.py          # Launch path loader
│   └── pure_pursuit_system.launch.py  # Launch complete system
├── config/
│   └── pure_pursuit_params.yaml       # Configuration parameters
└── README.md
```

## Dependencies

- ROS2 Foxy
- rclcpp
- geometry_msgs
- nav_msgs
- std_msgs
- tf2_ros
- tf2_geometry_msgs
- yaml-cpp

## Building

```bash
# Navigate to your ROS2 workspace
cd ~/purepursuit_ws

# Source ROS2
source /opt/ros/foxy/setup.bash

# Build the package
colcon build --packages-select pure_pursuit_controller

# Source the workspace
source install/setup.bash
```

## Usage

### 1. Recording Waypoints

First, record a path by driving your robot manually while recording waypoints:

```bash
# Launch the waypoint recorder
ros2 launch pure_pursuit_controller waypoint_recorder.launch.py output_file:=my_waypoints.yaml

# In another terminal, start recording
ros2 topic pub /start_recording std_msgs/msg/Bool "data: true" --once

# Drive your robot manually, then stop recording
ros2 topic pub /start_recording std_msgs/msg/Bool "data: false" --once
```

### 2. Running Pure Pursuit Controller

Launch the complete system with path loading and pure pursuit control:

```bash
# Launch the complete system
ros2 launch pure_pursuit_controller pure_pursuit_system.launch.py waypoints_file:=my_waypoints.yaml
```

Or launch components separately:

```bash
# Launch path loader
ros2 launch pure_pursuit_controller path_loader.launch.py waypoints_file:=my_waypoints.yaml

# Launch pure pursuit controller
ros2 launch pure_pursuit_controller pure_pursuit.launch.py
```

### 3. Bidirectional Control

To switch between forward and reverse movement:

```bash
# Switch to reverse mode
ros2 topic pub /reverse_mode std_msgs/msg/Bool "data: true" --once

# Switch back to forward mode
ros2 topic pub /reverse_mode std_msgs/msg/Bool "data: false" --once
```

## Topics

### Subscribed Topics

- `/ndt_pose` (geometry_msgs/PoseStamped): Current robot pose from NDT localization
- `/path` (nav_msgs/Path): Path to follow (from path loader)
- `/reverse_mode` (std_msgs/Bool): Switch between forward/reverse movement

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Control commands for the robot
- `/current_path` (nav_msgs/Path): Currently loaded path for visualization

### Waypoint Recorder Topics

- `/start_recording` (std_msgs/Bool): Start/stop waypoint recording
- `/recorded_path` (nav_msgs/Path): Currently recorded path for visualization

### Path Loader Topics

- `/load_path` (std_msgs/Bool): Trigger to reload waypoints from file

## Parameters

Edit `config/pure_pursuit_params.yaml` to tune the controller:

```yaml
lookahead_distance: 2.0          # Base lookahead distance (meters)
min_lookahead_distance: 1.0      # Minimum lookahead distance (meters)
max_lookahead_distance: 5.0      # Maximum lookahead distance (meters)
lookahead_ratio: 2.0             # Ratio for adaptive lookahead
max_linear_velocity: 1.0         # Maximum linear velocity (m/s)
max_angular_velocity: 1.0        # Maximum angular velocity (rad/s)
goal_tolerance: 0.5              # Goal reached tolerance (meters)
wheel_base: 2.5                  # Vehicle wheelbase (meters)
control_frequency: 10.0          # Control loop frequency (Hz)
```

## Workflow

1. **Record Waypoints**: Use the waypoint recorder to capture a path
2. **Load Path**: Use the path loader to publish recorded waypoints
3. **Track Path**: The pure pursuit controller follows the path
4. **Bidirectional**: Switch direction as needed using the reverse topic

## Algorithm Details

The pure pursuit controller uses:

- **Closest Waypoint Finding**: Locates the nearest waypoint to current position
- **Adaptive Lookahead**: Adjusts lookahead distance based on velocity
- **Curvature Calculation**: Computes steering using pure pursuit geometry
- **Goal Detection**: Stops when reaching the final waypoint

The algorithm supports bidirectional movement by reversing the waypoint order and adjusting orientations for reverse travel.

## Troubleshooting

- **No movement**: Check if path is loaded and `/ndt_pose` is being published
- **Erratic behavior**: Tune lookahead distances and velocity limits
- **Goal not reached**: Adjust `goal_tolerance` parameter
- **Build errors**: Ensure all dependencies are installed (especially yaml-cpp)

## License

MIT License 