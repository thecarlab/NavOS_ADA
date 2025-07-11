from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='waypoints.yaml',
        description='Path to the waypoints file to load'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the published path'
    )
    
    publish_on_startup_arg = DeclareLaunchArgument(
        'publish_on_startup',
        default_value='true',
        description='Whether to publish the path immediately on startup'
    )
    
    # Path loader node
    path_loader_node = Node(
        package='pure_pursuit_controller',
        executable='path_loader',
        name='path_loader',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_on_startup': LaunchConfiguration('publish_on_startup')
        }],
        output='screen',
        remappings=[
            ('/follow_path', '/follow_path'),
            ('/load_path', '/load_path')
        ]
    )
    
    return LaunchDescription([
        waypoints_file_arg,
        frame_id_arg,
        publish_on_startup_arg,
        path_loader_node
    ]) 