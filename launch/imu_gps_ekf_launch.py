#!/usr/bin/env python3
"""
Launch file for IMU+GPS EKF fusion system
Launches:
1. IMU and GPS publishers with realistic data
2. GPS to Cartesian converter
3. robot_localization EKF node
4. Optional: RViz for visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Launch arguments
    motion_type_arg = DeclareLaunchArgument(
        'motion_type',
        default_value='circular',
        description='Motion profile: circular, figure8, straight, stop_and_go'
    )
    
    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='50.0',
        description='Radius for circular/figure8 motion (meters)'
    )
    
    velocity_arg = DeclareLaunchArgument(
        'velocity',
        default_value='10.0',
        description='Vehicle velocity (m/s)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for all nodes'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('cone_stellation')
    
    # Configuration files
    ekf_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'ekf_config.yaml'
    ])
    
    # Nodes
    sensor_publisher_node = Node(
        package='cone_stellation',
        executable='imu_gps_publishers.py',
        name='realistic_sensor_publisher',
        output='screen',
        parameters=[{
            'motion_type': LaunchConfiguration('motion_type'),
            'radius': LaunchConfiguration('radius'),
            'velocity': LaunchConfiguration('velocity'),
        }]
    )
    
    gps_converter_node = Node(
        package='cone_stellation',
        executable='gps_to_cartesian.py',
        name='gps_to_cartesian_converter',
        output='screen',
        parameters=[{
            'reference_latitude': 37.540091,  # Konkuk University Ilgamho
            'reference_longitude': 127.076555,
            'reference_altitude': 39.5,
            'publish_tf': True,
            'world_frame': 'map',
            'child_frame': 'gps'
        }]
    )
    
    # Robot localization EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
            ('accel/filtered', 'accel/filtered'),
        ]
    )
    
    # Static transform publishers for TF tree
    # map -> odom (will be updated by SLAM later)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # base_link -> imu_link
    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        output='screen',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'os_imu']
    )
    
    # base_link -> gps_link
    base_to_gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps_tf',
        output='screen',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'gps']
    )
    
    # RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', PathJoinSubstitution([
            pkg_share,
            'rviz',
            'imu_gps_ekf.rviz'
        ])]
    )
    
    # # Path visualization node
    # path_visualization_node = Node(
    #     package='cone_stellation',
    #     executable='path_visualization',
    #     name='path_visualization',
    #     output='screen',
    #     parameters=[{
    #         'max_path_points': 2000,
    #         'error_scale': 10.0,
    #         'update_rate': 20.0
    #     }]
    # )
    
    # Group all nodes with optional namespace
    grouped_nodes = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        sensor_publisher_node,
        gps_converter_node,
        ekf_node,
        map_to_odom_tf,
        base_to_imu_tf,
        base_to_gps_tf,
        # path_visualization_node,
    ])
    
    return LaunchDescription([
        # Arguments
        motion_type_arg,
        radius_arg,
        velocity_arg,
        use_rviz_arg,
        namespace_arg,
        
        # Nodes
        grouped_nodes,
        rviz_node,  # Outside group to avoid namespace issues
    ])