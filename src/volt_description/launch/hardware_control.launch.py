from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    config_file = PathJoinSubstitution([
        FindPackageShare("volt_description"),
        "config",
        "gait_controller.yaml",
    ])

    motion_controller = Node(
        package="volt_description",
        executable="volt_motion_controller.py",
        name="volt_motion_controller",
        output="screen",
        parameters=[config_file],
    )

    serial_bridge = Node(
        package="volt_description",
        executable="volt_serial_bridge.py",
        name="volt_serial_bridge",
        output="screen",
        parameters=[{
            "port": serial_port,
            "baud_rate": baud_rate,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Arduino Nano serial device",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Arduino firmware baud rate",
        ),
        motion_controller,
        serial_bridge,
    ])
