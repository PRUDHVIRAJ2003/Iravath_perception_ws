#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_dir = FindPackageShare('iravath_perception')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'perception_params.yaml']),
        description='Path to the configuration file'
    )
    
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo',
        default_value='false',
        description='Use YOLO detection instead of Mask R-CNN'
    )
    
    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='true',
        description='Enable debug image publishing'
    )
    
    # Camera node
    camera_node = Node(
        package='iravath_perception',
        executable='realsense_camera_node.py',
        name='realsense_camera_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )
    
    # Object detection nodes (conditionally launch based on use_yolo parameter)
    mask_rcnn_node = Node(
        package='iravath_perception',
        executable='object_detection_node.py',
        name='object_detection_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('use_yolo'))
    )
    
    yolo_node = Node(
        package='iravath_perception',
        executable='yolo_detection_node.py',
        name='yolo_detection_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_yolo'))
    )
    
    return LaunchDescription([
        config_file_arg,
        use_yolo_arg,
        enable_debug_arg,
        
        GroupAction([
            camera_node,
            mask_rcnn_node,
            yolo_node,
        ])
    ])