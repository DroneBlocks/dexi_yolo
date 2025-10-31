#!/usr/bin/env python3
"""
Launch file for YOLO ONNX detection node

Usage:
    # Basic (uses defaults: 320x320, custom model, simple resize)
    ros2 launch dexi_yolo yolo_onnx_launch.py

    # Custom model and settings
    ros2 launch dexi_yolo yolo_onnx_launch.py \
        model_path:=models/best_optimized.onnx \
        input_size:=320 \
        use_letterbox:=true

    # Pi CM4 optimized (lower threads, lower frequency)
    ros2 launch dexi_yolo yolo_onnx_launch.py \
        num_threads:=1 \
        detection_frequency:=0.5

    # Desktop testing (higher threads, higher frequency)
    ros2 launch dexi_yolo yolo_onnx_launch.py \
        num_threads:=4 \
        detection_frequency:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='models/best_optimized.onnx',
        description='Path to ONNX model file'
    )

    input_size_arg = DeclareLaunchArgument(
        'input_size',
        default_value='320',
        description='Model input size (320x320 for custom trained model)'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Detection confidence threshold (0.0-1.0)'
    )

    detection_frequency_arg = DeclareLaunchArgument(
        'detection_frequency',
        default_value='1.0',
        description='Detection frequency in Hz (lower = less CPU usage)'
    )

    num_threads_arg = DeclareLaunchArgument(
        'num_threads',
        default_value='2',
        description='Number of CPU threads (1 for Pi CM4, 4+ for desktop)'
    )

    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.4',
        description='Non-Maximum Suppression threshold (lower = stricter filtering)'
    )

    use_letterbox_arg = DeclareLaunchArgument(
        'use_letterbox',
        default_value='true',
        description='Use letterbox preprocessing (preserves aspect ratio)'
    )

    # YOLO ONNX detection node
    yolo_onnx_node = Node(
        package='dexi_yolo',
        executable='dexi_yolo_node_onnx.py',
        name='dexi_yolo_onnx',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_size': LaunchConfiguration('input_size'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'detection_frequency': LaunchConfiguration('detection_frequency'),
            'num_threads': LaunchConfiguration('num_threads'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'use_letterbox': LaunchConfiguration('use_letterbox'),
        }],
        remappings=[
            ('/cam0/image_raw/compressed', '/cam0/image_raw/compressed'),
            ('/yolo_detections', '/yolo_detections'),
        ]
    )

    # Info message
    info_msg = LogInfo(
        msg=[
            '\n',
            '='*70, '\n',
            'YOLO ONNX Detection Node\n',
            '='*70, '\n',
            'Subscribing to: /cam0/image_raw/compressed\n',
            'Publishing to: /yolo_detections\n',
            '\n',
            'Parameters:\n',
            '  model_path: ', LaunchConfiguration('model_path'), '\n',
            '  input_size: ', LaunchConfiguration('input_size'), 'x', LaunchConfiguration('input_size'), '\n',
            '  confidence_threshold: ', LaunchConfiguration('confidence_threshold'), '\n',
            '  detection_frequency: ', LaunchConfiguration('detection_frequency'), ' Hz\n',
            '  num_threads: ', LaunchConfiguration('num_threads'), '\n',
            '  nms_threshold: ', LaunchConfiguration('nms_threshold'), '\n',
            '  use_letterbox: ', LaunchConfiguration('use_letterbox'), '\n',
            '\n',
            'Monitor detections:\n',
            '  ros2 topic echo /yolo_detections\n',
            '\n',
            'Press Ctrl+C to stop\n',
            '='*70, '\n'
        ]
    )

    return LaunchDescription([
        # Declare arguments
        model_path_arg,
        input_size_arg,
        confidence_threshold_arg,
        detection_frequency_arg,
        num_threads_arg,
        nms_threshold_arg,
        use_letterbox_arg,

        # Info message
        info_msg,

        # Node
        yolo_onnx_node,
    ])
