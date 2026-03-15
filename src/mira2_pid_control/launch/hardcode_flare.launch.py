from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mira2_pid_control')

    timed_config = os.path.join(pkg_share, 'config', 'gate_flare_timed_params.yaml')
    imu_config   = os.path.join(pkg_share, 'config', 'gate_flare_imu_params.yaml')

    return LaunchDescription([

        # ── Timed version ────────────────────────────────────────────────────
        Node(
            package='mira2_pid_control',
            executable='gate_flare_timed_exe',
            name='gate_flare_timed',
            output='screen',
            parameters=[timed_config] if os.path.exists(timed_config) else [],
            emulate_tty=True,
        ),

        # ── IMU version ──────────────────────────────────────────────────────
        Node(
            package='mira2_pid_control',
            executable='gate_flare_imu_exe',
            name='gate_flare_imu',
            output='screen',
            parameters=[imu_config] if os.path.exists(imu_config) else [],
            emulate_tty=True,
        ),

        # ── Key publisher for manual arm/disarm ──────────────────────────────
        Node(
            package='mira2_pid_control',
            executable='key_pub.py',
            name='key_publisher',
            output='screen',
            emulate_tty=True,
        ),
    ])