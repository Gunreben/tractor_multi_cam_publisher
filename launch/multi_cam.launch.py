import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tractor_multi_cam_publisher',
            executable='tractor_multi_cam_publisher',
            name='multi_cam_publisher',
            output='screen',
            parameters=[{
            }]
        )
    ])