#!/usr/bin/env python3
# launch/default_camera.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	# Launch arguments (same defaults as the XML)
	list_devices = LaunchConfiguration('list_devices')
	stream = LaunchConfiguration('stream')
	vendor_id = LaunchConfiguration('vendor_id')
	product_id = LaunchConfiguration('product_id')
	serial_no = LaunchConfiguration('serial_no')
	display = LaunchConfiguration('display')
	verbose = LaunchConfiguration('verbose')
	image_width = LaunchConfiguration('image_width')
	image_height = LaunchConfiguration('image_height')
	frame_format = LaunchConfiguration('frame_format')
	framerate = LaunchConfiguration('framerate')

	return LaunchDescription([
		DeclareLaunchArgument('list_devices', default_value='false'),
		DeclareLaunchArgument('stream', default_value='true'),
		DeclareLaunchArgument('vendor_id', default_value='0x0c45'),
		DeclareLaunchArgument('product_id', default_value='0x6366'),
		DeclareLaunchArgument('serial_no', default_value=''),
		DeclareLaunchArgument('display', default_value='true'),
		DeclareLaunchArgument('verbose', default_value='false'),
		DeclareLaunchArgument('image_width', default_value='640'),
		DeclareLaunchArgument('image_height', default_value='480'),
		DeclareLaunchArgument('frame_format', default_value='MJPEG'),
		DeclareLaunchArgument('framerate', default_value='30'),

		Node(
			package='camera_driver_ffmpeg',
			executable='camera_driver_ffmpeg_exe',
			name='camera_defaultcam',
			output='screen',
			parameters=[{
				'list_devices': list_devices,
				'stream': stream,
				'vendor_id': vendor_id,
				'product_id': product_id,
				'serial_no': serial_no,
				'display': display,
				'verbose': verbose,
				'image_width': image_width,
				'image_height': image_height,
				'frame_format': frame_format,
				'framerate': framerate,
			}],
		),
	])
