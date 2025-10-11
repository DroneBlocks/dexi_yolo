#!/usr/bin/env python3
"""
Launch file for local testing of YOLO detection with camera simulator

This launches:
1. Camera simulator - publishes video frames to /cam0/image_raw/compressed
2. YOLO ONNX node - subscribes to camera and publishes detections to /yolo_detections

Usage:
    ros2 launch dexi_yolo local_test_launch.py
    ros2 launch dexi_yolo local_test_launch.py video_path:=/path/to/your/video.mp4
    ros2 launch dexi_yolo local_test_launch.py input_size:=320 use_letterbox:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value='scripts/dexi_test_flight.mp4',
        description='Path to test video file (absolute or relative to package)'
    )

    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='30.0',
        description='Camera simulator playback FPS'
    )

    loop_video_arg = DeclareLaunchArgument(
        'loop_video',
        default_value='true',
        description='Loop video when finished'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='models/best_optimized.onnx',
        description='Path to ONNX model file'
    )

    input_size_arg = DeclareLaunchArgument(
        'input_size',
        default_value='640',
        description='Model input size (640 for custom model, 320 for default)'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Detection confidence threshold'
    )

    use_letterbox_arg = DeclareLaunchArgument(
        'use_letterbox',
        default_value='false',
        description='Use letterbox preprocessing (preserves aspect ratio)'
    )

    num_threads_arg = DeclareLaunchArgument(
        'num_threads',
        default_value='4',
        description='Number of CPU threads (can use more on desktop than Pi)'
    )

    # Camera simulator node
    camera_simulator = Node(
        package='dexi_yolo',
        executable='camera_simulator_node.py',
        name='camera_simulator',
        output='screen',
        parameters=[{
            'video_path': LaunchConfiguration('video_path'),
            'fps': LaunchConfiguration('camera_fps'),
            'loop': LaunchConfiguration('loop_video'),
            'start_paused': False,
            'compression_quality': 90
        }]
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
            'use_letterbox': LaunchConfiguration('use_letterbox'),
            'num_threads': LaunchConfiguration('num_threads'),
            'detection_frequency': 2.0,  # Process every other frame for desktop testing
            'nms_threshold': 0.4
        }]
    )

    # Info message
    info_msg = LogInfo(
        msg=[
            '\n',
            '='*70, '\n',
            'Local YOLO Detection Testing\n',
            '='*70, '\n',
            'Camera Simulator: Publishing video frames to /cam0/image_raw/compressed\n',
            'YOLO ONNX Node: Processing frames and publishing detections to /yolo_detections\n',
            '\n',
            'Monitor detections with:\n',
            '  ros2 topic echo /yolo_detections\n',
            '\n',
            'Parameters:\n',
            '  video_path: ', LaunchConfiguration('video_path'), '\n',
            '  model_path: ', LaunchConfiguration('model_path'), '\n',
            '  input_size: ', LaunchConfiguration('input_size'), 'x', LaunchConfiguration('input_size'), '\n',
            '  use_letterbox: ', LaunchConfiguration('use_letterbox'), '\n',
            '  confidence_threshold: ', LaunchConfiguration('confidence_threshold'), '\n',
            '\n',
            'Press Ctrl+C to stop\n',
            '='*70, '\n'
        ]
    )

    return LaunchDescription([
        # Declare arguments
        video_path_arg,
        camera_fps_arg,
        loop_video_arg,
        model_path_arg,
        input_size_arg,
        confidence_threshold_arg,
        use_letterbox_arg,
        num_threads_arg,

        # Info message
        info_msg,

        # Nodes
        camera_simulator,
        yolo_onnx_node,
    ])
