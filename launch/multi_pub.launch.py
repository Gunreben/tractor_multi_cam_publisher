#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Find the package directories
    tractor_pkg_prefix = FindPackageShare('tractor_multi_cam_publisher').find('tractor_multi_cam_publisher')
    blickfeld_pkg_prefix = FindPackageShare('blickfeld_qb2_ros2_driver').find('blickfeld_qb2_ros2_driver')
    ouster_pkg_prefix = FindPackageShare('ouster_ros').find('ouster_ros')
    zed_pkg_prefix = FindPackageShare('zed_wrapper').find('zed_wrapper')
    novatel_pkg_prefix = FindPackageShare('novatel_oem7_driver').find('novatel_oem7_driver')
    
    # Define the paths to the launch files
    tractor_launch_file = PythonLaunchDescriptionSource([tractor_pkg_prefix, '/launch/multi_cam.launch.py'])
    blickfeld_launch_file = PythonLaunchDescriptionSource([blickfeld_pkg_prefix, '/launch/blickfeld_qb2_ros2_driver.launch.py'])
    ouster_launch_file = PythonLaunchDescriptionSource([ouster_pkg_prefix, '/launch/driver.launch.py'])
    zed_launch_file = PythonLaunchDescriptionSource([zed_pkg_prefix, '/launch/zed_camera.launch.py'])
    novatel_launch_file = PythonLaunchDescriptionSource([novatel_pkg_prefix, '/launch/oem7_net.launch.py'])
    
    # Declare launch arguments
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='Model of the ZED camera.'
    )
    
    novatel_ip_arg = DeclareLaunchArgument(
        'oem7_ip_addr',
        default_value='192.168.26.100',
        description='IP address of the Novatel OEM7'
    )
    
    ouster_ip_arg = DeclareLaunchArgument(
        'sensor_hostname',
        default_value='192.168.26.85',
        description='IP address of the Ouster sensor.'
    )
    
    #--------------------------------------------------------------------------
    # robot_state_publisher for vario700_sensorrig URDF
    #--------------------------------------------------------------------------
    vario700_pkg_prefix = FindPackageShare('vario700_sensorrig').find('vario700_sensorrig')
    # Path to the URDF
    vario700_urdf = os.path.join(vario700_pkg_prefix, 'urdf', 'vario700_sensorrig.urdf')

    # Read the URDF file’s contents
    with open(vario700_urdf, 'r') as f:
        robot_description = f.read()

    # Create the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description}
        ]
    )
    
    #--------------------------------------------------------------------------
    # Add RViz2 node using vario700_sensorrig.rviz
    #--------------------------------------------------------------------------
    vario700_rviz_config = os.path.join(vario700_pkg_prefix, 'rviz', 'vario700_sensorrig.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', vario700_rviz_config]
    )
    
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
    # Combine everything into the LaunchDescription
    #--------------------------------------------------------------------------
    return LaunchDescription([
        camera_model_arg,
        novatel_ip_arg,
        ouster_ip_arg,
        
        # Tractor multi camera
        IncludeLaunchDescription(tractor_launch_file),
        
        # Camera Info Publisher
        camera_info_publisher_node,
        
        # Blickfeld deactivated till presentation
        IncludeLaunchDescription(blickfeld_launch_file),
        
        # Ouster LiDAR
        IncludeLaunchDescription(
            ouster_launch_file,
            launch_arguments=[('sensor_hostname', LaunchConfiguration('sensor_hostname'))]
        ),
        
        # ZED camera
        IncludeLaunchDescription(
            zed_launch_file,
            launch_arguments=[('camera_model', LaunchConfiguration('camera_model'))]
        ),
        
        # Novatel OEM7
        IncludeLaunchDescription(
            novatel_launch_file,
            launch_arguments=[('oem7_ip_addr', LaunchConfiguration('oem7_ip_addr'))]
        ),
        
        robot_state_publisher_node,
        
        # Deactivated rviz
        #rviz_node
    ])

if __name__ == '__main__':
    generate_launch_description()
