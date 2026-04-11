#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hef_path',
            default_value='',
            description='Path to Hailo HEF model file (empty = package default best_optimized.hef)'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Confidence threshold for detections (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'detection_frequency',
            default_value='10.0',
            description='Max detection frequency in Hz'
        ),
        DeclareLaunchArgument(
            'class_names',
            default_value='car,motorcycle,truck,bird,cat,dog',
            description='Comma-separated class names matching model output order'
        ),

        Node(
            package='dexi_yolo',
            executable='dexi_yolo_node_hailo.py',
            name='dexi_yolo_hailo_node',
            output='screen',
            parameters=[{
                'hef_path': LaunchConfiguration('hef_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'detection_frequency': LaunchConfiguration('detection_frequency'),
                'class_names': LaunchConfiguration('class_names'),
            }],
        )
    ])
