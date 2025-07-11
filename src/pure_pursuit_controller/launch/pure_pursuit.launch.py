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
        description='Path to the configuration file'
    )
    
    # Pure pursuit controller node
    pure_pursuit_node = Node(
        package='pure_pursuit_controller',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        remappings=[
            ('/pcl_pose', '/pcl_pose'),
            ('/cmd_vel', '/cmd_vel'),
            ('/path', '/follow_path'),
            ('/reverse_mode', '/reverse_mode')
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        pure_pursuit_node
    ]) 