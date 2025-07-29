#!/usr/bin/env python3
"""
Corrected launch file for robot_localization EKF node.
The EKF node is responsible for publishing the map->odom transform.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('cone_stellation')
    
    # Configuration files
    ekf_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'ekf_config_real.yaml'
    ])
    
    # --- 삭제 ---
    # gps_converter_node는 EKF가 직접 GPS 데이터를 처리하므로 필요 없습니다.
    # EKF와 TF 발행 역할이 중복되어 충돌을 일으킵니다.
    # gps_converter_node = Node(...)
    
    # Robot localization EKF node
    # 이 노드가 map -> odom TF를 발행하는 유일한 주체가 되어야 합니다.
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
    
    # --- 삭제 ---
    # map -> odom TF는 EKF가 동적으로 계산하여 발행해야 합니다.
    # 고정된 static transform을 발행하면 EKF의 출력과 충돌합니다.
    # map_to_odom_tf = Node(...)
    
    # Static transform publishers for a robot's physical structure (Correct Use)
    # 로봇 본체와 센서 간의 고정된 물리적 관계를 정의하므로 유지합니다.
    
    # base_link -> imu_link
    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        output='screen',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    # base_link -> gps_link
    base_to_gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps_tf',
        output='screen',
        # 아래 frame_id는 GPS 메시지의 header.frame_id와 일치해야 합니다.
        # 만약 GPS 원본 데이터의 frame_id가 "gps"라면 여기도 "gps"로 바꿔주세요.
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'gps']
    )
    
    return LaunchDescription([
        # EKF 노드
        ekf_node,
        
        # 로봇 구조에 대한 정적 TF 노드들
        base_to_imu_tf,
        base_to_gps_tf,
    ])