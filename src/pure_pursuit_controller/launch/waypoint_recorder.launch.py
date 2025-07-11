from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='waypoints.yaml',
        description='Output file for recorded waypoints'
    )
    
    distance_threshold_arg = DeclareLaunchArgument(
        'recording_distance_threshold',
        default_value='0.5',
        description='Minimum distance between recorded waypoints'
    )
    
    # Waypoint recorder node
    waypoint_recorder_node = Node(
        package='pure_pursuit_controller',
        executable='waypoint_recorder',
        name='waypoint_recorder',
        parameters=[{
            'output_file': LaunchConfiguration('output_file'),
            'recording_distance_threshold': LaunchConfiguration('recording_distance_threshold')
        }],
        output='screen',
        remappings=[
            ('/pcl_pose', '/pcl_pose'),
            ('/start_recording', '/start_recording'),
            ('/recorded_path', '/recorded_path')
        ]
    )
    
    return LaunchDescription([
        output_file_arg,
        distance_threshold_arg,
        waypoint_recorder_node
    ]) 
