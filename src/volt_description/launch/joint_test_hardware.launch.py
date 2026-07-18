from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("volt_description")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_share,
                    "launch",
                    "joint_test.launch.py",
                ])
            ),
            launch_arguments={
                "serial_port": serial_port,
                "baud_rate": baud_rate,
                "use_hardware": "true",
                "dry_run": "false",
                "auto_arm": "false",
            }.items(),
        )
    ])
