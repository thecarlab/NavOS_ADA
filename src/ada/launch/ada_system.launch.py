#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    lidar_localization_ros2_dir = get_package_share_directory('lidar_localization_ros2')
    control_center_dir = get_package_share_directory('control_center')
    pure_pursuit_dir = get_package_share_directory('pure_pursuit_controller')
    
    # Launch sllidar_s1
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_s1_launch.py'
            ])
        )
    )
    # Launch realsense node
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_rgbd': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true'
        }.items()
    )
    
    # Launch lidar transform node from util package
    lidar_transform_node = Node(
        package='util',
        executable='lidar_transformer_node',
        name='lidar_transformer_node',
        output='screen',
        parameters=[]
    )
    
    # Launch RViz2 with localization configuration
    rviz_config_path = '/home/ada1/ros2_ws/src/lidar_localization_ros2/rviz/localization.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Launch lidar localization (delayed to start after rviz2)
    lidar_localization_launch = TimerAction(
        period=3.0,  # Wait 3 seconds before starting lidar localization
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('lidar_localization_ros2'),
                        'launch',
                        'lidar_localization.launch.py'
                    ])
                )
            )
        ]
    )
    
    # Launch control center node
    control_center_node = Node(
        package='control_center',
        executable='control_center_node',
        name='control_center_node',
        output='screen'
    )

    # Launch pure pursuit controller
    pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pure_pursuit_controller'),
                'launch',
                'pure_pursuit_system.launch.py'
            ])
        ),
        launch_arguments={
            'config_file': os.path.join(pure_pursuit_dir, 'config', 'pure_pursuit_params.yaml'),
            'waypoints_file': '/home/ada1/ros2_ws/waypoints.yaml',
            'frame_id': 'map'
        }.items()
    )
    
    return LaunchDescription([
        rviz_node,
        sllidar_launch,
        realsense_launch,
        control_center_node,
        lidar_transform_node,
        lidar_localization_launch, 
        #pure_pursuit_launch,
    ]) 
