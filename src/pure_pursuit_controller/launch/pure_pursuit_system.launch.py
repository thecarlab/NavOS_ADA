from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('pure_pursuit_controller')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'pure_pursuit_params.yaml'),
        description='Path to the pure pursuit configuration file'
    )
    
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='/home/ada1/ros2_ws/waypoints.yaml',
        description='Path to the waypoints file to load'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the published path'
    )
    
    # Path loader node
    path_loader_node = Node(
        package='pure_pursuit_controller',
        executable='path_loader',
        name='path_loader',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_on_startup': True
        }],
        output='screen'
    )
    
    # Pure pursuit controller node
    pure_pursuit_node = Node(
        package='pure_pursuit_controller',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        remappings=[('/ackermann_cmd','/purepursuit_cmd')]
    )
    
    return LaunchDescription([
        config_file_arg,
        waypoints_file_arg,
        frame_id_arg,
        path_loader_node,
        pure_pursuit_node
    ]) 
