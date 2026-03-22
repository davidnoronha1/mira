import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo

def generate_launch_description():
    machine = os.environ.get('MACHINE_NAME', 'ORIN')
    port = "8554" 
    
    if machine == 'RPI4':
        usb_path = "usb-3610000.usb-2.1"
    else:
        usb_path = "usb-xhci-hcd.1-1"

    dev_path = f"/dev/v4l/by-path/platform-{usb_path}-video-index0"

    camera_node = Node(
        package='camera_driver',
        executable='camera_driver_exe',
        name='camera_auto',
        output='screen',
        parameters=[
            {'port': port},
            {'camera_frame_id': 'camera_auto'}
            # {'video_device': dev_path} 
        ]
    )

    gst_streamer = ExecuteProcess(
        cmd=[f"sleep 2 ; gst-launch-1.0 rtspsrc location=rtsp://127.0.0.1:{port}/image_rtsp ! fakesink"],
        cwd="/home",
        shell=True
    )

    return LaunchDescription([
        LogInfo(msg=f"--- MIRA Auto-Launch: Detected {machine}, using {dev_path} ---"),
        camera_node,
        gst_streamer
    ])
