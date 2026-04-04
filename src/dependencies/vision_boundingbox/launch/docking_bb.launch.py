#!/usr/bin/env python3
"""Launch file for vision_boundingbox object detection node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for vision_boundingbox."""
    
    # Declare launch arguments
    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://192.168.2.6:2001/image_rtsp',
        description='Input RTSP URL'
    )
    
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='docking.onnx',
        description='ONNX model filename'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='CPU',
        description='Device to run inference on (CPU or GPU)'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Confidence threshold for detections'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.4',
        description='NMS threshold for detections'
    )
    
    publish_image_arg = DeclareLaunchArgument(
        'publish_image',
        default_value='true',
        description='Whether to publish visualization image'
    )
    
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='false',
        description='Whether to show visualization window'
    )
    
    # Vision bounding box node
    vision_node = Node(
        package='vision_boundingbox',
        executable='vision_boundingbox_exe',
        name='vision_boundingbox_node',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'device': LaunchConfiguration('device'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'publish_image': LaunchConfiguration('publish_image'),
            'visualize': LaunchConfiguration('visualize'),
            'rtsp_url': LaunchConfiguration('rtsp_url'),
            'webcam': False,
            'input_height': 640,
            'input_width': 640,
            'reject_reflections': True,
            'reject_threshold': 0.5
        }]
    )
    
    return LaunchDescription([
        rtsp_url_arg,
        model_name_arg,
        device_arg,
        conf_threshold_arg,
        nms_threshold_arg,
        publish_image_arg,
        visualize_arg,
        vision_node,
    ])
