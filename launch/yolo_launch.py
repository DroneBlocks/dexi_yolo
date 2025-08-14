#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'model_path',
            default_value='models/yolov8n.pt',
            description='Path to YOLO model file in package'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Confidence threshold for detections (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'detection_frequency',
            default_value='1.0',
            description='Detection frequency in Hz'
        ),
        
        # YOLO detection node
        Node(
            package='dexi_yolo',
            executable='dexi_yolo_node.py',
            name='dexi_yolo_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'detection_frequency': LaunchConfiguration('detection_frequency'),
            }],
            remappings=[
                ('/cam0/image_raw/compressed', '/cam0/image_raw/compressed'),
                ('/yolo_detections', '/yolo_detections'),
            ]
        )
    ])
