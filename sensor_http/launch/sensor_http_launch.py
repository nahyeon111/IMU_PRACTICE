import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_http',
            executable='sensor_http_server',
            name='sensor_http_node',
            output='screen',
        ),
        Node(
            package='sensor_http',
            executable='imu_processor',
            name='imu_processor_node',
            output='screen',
        ),
        Node(
            package='sensor_http',
            executable='imu_kalman_filter',
            name='imu_kalman_filter_node',
            output='screen',
        ),
    ])
