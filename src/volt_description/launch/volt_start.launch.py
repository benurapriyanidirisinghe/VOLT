from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("volt_description")
    gui = LaunchConfiguration("gui")
    auto_ready_pose = LaunchConfiguration("auto_ready_pose")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_hardware = LaunchConfiguration("use_hardware")
    dry_run = LaunchConfiguration("dry_run")
    auto_arm = LaunchConfiguration("auto_arm")
    start_serial_bridge = LaunchConfiguration("start_serial_bridge")
    calibration_file = LaunchConfiguration("calibration_file")

    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share,
                "launch",
                "ignition.launch.py",
            ])
        ),
        launch_arguments={
            "gui": gui,
            "auto_ready_pose": auto_ready_pose,
        }.items(),
    )

    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share,
                "launch",
                "control.launch.py",
            ])
        ),
        launch_arguments={
            "gui": gui,
            "auto_ready_pose": auto_ready_pose,
        }.items(),
    )

    serial_bridge = Node(
        package="volt_description",
        executable="volt_serial_bridge.py",
        name="volt_serial_bridge",
        output="screen",
        condition=IfCondition(start_serial_bridge),
        parameters=[{
            "port": serial_port,
            "baud_rate": baud_rate,
            "auto_arm": auto_arm,
            "dry_run": dry_run,
            "hardware_enabled": use_hardware,
            "calibration_file": calibration_file,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo and VOLT control GUIs",
        ),
        DeclareLaunchArgument(
            "auto_ready_pose",
            default_value="false",
            description="Automatically move from loaded zero pose to walk-ready pose",
        ),
        DeclareLaunchArgument(
            "start_serial_bridge",
            default_value="false",
            description="Start Arduino serial bridge with the main VOLT GUI",
        ),
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
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Allow serial bridge to open Arduino serial port",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Log outgoing FRAME packets without opening serial",
        ),
        DeclareLaunchArgument(
            "auto_arm",
            default_value="false",
            description="Send ARM to Arduino after connecting",
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("volt_description"),
                "config",
                "servo_calibration.yaml",
            ]),
            description="Servo calibration YAML file",
        ),
        ignition_launch,
        TimerAction(
            period=12.0,
            actions=[gui_launch],
        ),
        TimerAction(
            period=13.0,
            actions=[serial_bridge],
        ),
    ])
