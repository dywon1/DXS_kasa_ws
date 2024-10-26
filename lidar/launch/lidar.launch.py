#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar',
            executable='sampling',
            name='sampling',
            parameters=['/home/wondoyeon/ros2_ws/src/lidar/config/parameters.yaml']
        ),
        Node(
            package='lidar',
            executable='roi',
            name='roi',
            parameters=['/home/wondoyeon/ros2_ws/src/lidar/config/parameters.yaml']
        ),
        Node(
            package='lidar',
            executable='ransac',
            name='ransac',
            parameters=['/home/wondoyeon/ros2_ws/src/lidar/config/parameters.yaml']
        ),
        # Node(
        #     package='lidar',
        #     executable='dbscan',
        #     name='dbscan',
        #     parameters=['/home/wondoyeon/ros2_ws/src/lidar/config/parameters.yaml']
        # ),
         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/wondoyeon/ros2_ws/src/lidar/config/lidar_rviz2_config.rviz']
        ),
    ])

