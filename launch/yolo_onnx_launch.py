#!/usr/bin/env python3
"""
Launch file for YOLO ONNX detection node

Usage:
    # Default profile (avr_2026)
    ros2 launch dexi_yolo yolo_onnx_launch.py

    # Another profile from models/models.yaml
    ros2 launch dexi_yolo yolo_onnx_launch.py model:=coco

    # A model of your own. classes is required and must be in training order.
    ros2 launch dexi_yolo yolo_onnx_launch.py \
        model:=/home/dexi/models/mine.onnx \
        classes:=apple,banana

    # Pi CM4
    ros2 launch dexi_yolo yolo_onnx_launch.py num_threads:=1 detection_frequency:=1.0
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve(context, *args, **kwargs):
    share = get_package_share_directory('dexi_yolo')
    profiles = yaml.safe_load(open(os.path.join(share, 'models', 'models.yaml')))

    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    model = cfg('model')
    classes = cfg('classes')
    input_size = cfg('input_size')
    confidence = cfg('confidence_threshold')

    if model in profiles:
        p = profiles[model]
        model_path = os.path.join(share, 'models', p['file'])
        classes = classes or ','.join(p['classes'])
        input_size = input_size or str(p['input_size'])
        confidence = confidence or str(p['confidence'])
    else:
        model_path = model
        if not classes:
            raise RuntimeError(
                "model '%s' is not a profile in models.yaml, so classes must be "
                "given as well. Profiles: %s" % (model, ', '.join(profiles))
            )
        input_size = input_size or '320'
        confidence = confidence or '0.5'

    return [
        LogInfo(msg='dexi_yolo: %s -> %s (%s classes, %sx%s, conf %s)'
                    % (model, os.path.basename(model_path),
                       len(classes.split(',')), input_size, input_size, confidence)),
        Node(
            package='dexi_yolo',
            executable='dexi_yolo_node_onnx.py',
            name='dexi_yolo_onnx',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'class_names': classes,
                'input_size': int(input_size),
                'confidence_threshold': float(confidence),
                'detection_frequency': float(cfg('detection_frequency')),
                'num_threads': int(cfg('num_threads')),
                'nms_threshold': float(cfg('nms_threshold')),
                'use_letterbox': cfg('use_letterbox').lower() in ('true', '1'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='avr_2026',
            description='Profile name from models/models.yaml, or a path to an .onnx'
        ),
        DeclareLaunchArgument(
            'classes',
            default_value='',
            description='Comma-separated class names in training order. Taken from the profile when empty.'
        ),
        DeclareLaunchArgument(
            'input_size',
            default_value='',
            description='Model input size. Taken from the profile when empty.'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='',
            description='Detection confidence threshold. Taken from the profile when empty.'
        ),
        DeclareLaunchArgument(
            'detection_frequency',
            default_value='1.0',
            description='Detection frequency in Hz (lower = less CPU usage)'
        ),
        DeclareLaunchArgument(
            'num_threads',
            default_value='2',
            description='Number of CPU threads (1 for Pi CM4, 4+ for desktop)'
        ),
        DeclareLaunchArgument(
            'nms_threshold',
            default_value='0.4',
            description='Non-Maximum Suppression threshold (lower = stricter filtering)'
        ),
        DeclareLaunchArgument(
            'use_letterbox',
            default_value='true',
            description='Use letterbox preprocessing (preserves aspect ratio)'
        ),

        OpaqueFunction(function=resolve),
    ])
