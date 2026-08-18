from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("volt_description")
    serial_port = LaunchConfiguration("serial_port")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_hardware = LaunchConfiguration("use_hardware")
    dry_run = LaunchConfiguration("dry_run")
    calibration_file = LaunchConfiguration("calibration_file")
    max_send_rate = LaunchConfiguration("max_send_rate")

    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "ignition.launch.py"])
        ),
        launch_arguments={"gui": "true"}.items(),
        condition=IfCondition(use_gazebo),
    )

    command_router = Node(
        package="volt_description",
        executable="volt_joint_command_router.py",
        name="volt_joint_command_router",
        output="screen",
    )

    serial_bridge = Node(
        package="volt_description",
        executable="volt_serial_bridge.py",
        name="volt_serial_bridge",
        output="screen",
        parameters=[{
            "port": serial_port,
            "baud_rate": "57600",
            "calibration_file": calibration_file,
            "max_send_rate": max_send_rate,
            "dry_run": dry_run,
            "hardware_enabled": use_hardware,
            "auto_arm": False,
            # Calibration is a supported, manual single-servo workflow and
            # intentionally does not launch the walking motion controller.
            "require_motion_safe_to_arm": False,
        }],
    )

    calibration_gui = Node(
        package="volt_description",
        executable="volt_servo_calibration_gui.py",
        name="volt_servo_calibration_gui",
        output="screen",
        parameters=[{"calibration_file": calibration_file}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_gazebo", default_value="true"),
        DeclareLaunchArgument("use_hardware", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("volt_description"),
                "config",
                "servo_calibration.yaml",
            ]),
        ),
        DeclareLaunchArgument("max_send_rate", default_value="30.0"),
        ignition_launch,
        TimerAction(period=10.5, actions=[command_router]),
        TimerAction(period=11.0, actions=[serial_bridge]),
        TimerAction(period=12.0, actions=[calibration_gui]),
    ])
