from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='util',
            executable='imu_transformer_node',
            name='imu_transformer_node',
            output='screen'
        ),
        Node(
            package='util',
            executable='lidar_transformer_node',
            name='lidar_transformer_node',
            output='screen'
        )
    ])
