#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Find the package directory
    tractor_pkg_prefix = FindPackageShare('tractor_multi_cam_publisher').find('tractor_multi_cam_publisher')
    
    #--------------------------------------------------------------------------
    # Camera Info Publisher for 7 cameras
    #--------------------------------------------------------------------------
    camera_info_publisher_node = Node(
        package='tractor_multi_cam_publisher',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        output='screen',
        parameters=[
            {'calibration_path': os.path.join(tractor_pkg_prefix, 'calibration')}
        ]
    )
    
    #--------------------------------------------------------------------------
    # ZED Camera Info Publisher
    #--------------------------------------------------------------------------
    zed_camera_info_publisher_node = Node(
        package='tractor_multi_cam_publisher',
        executable='zed_camera_info_publisher',
        name='zed_camera_info_publisher',
        output='screen'
    )
    
    #--------------------------------------------------------------------------
    # Combine everything into the LaunchDescription
    #--------------------------------------------------------------------------
    return LaunchDescription([
        camera_info_publisher_node,
        zed_camera_info_publisher_node,
    ])

if __name__ == '__main__':
    generate_launch_description()


