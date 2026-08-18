from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("volt_description")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    auto_arm = LaunchConfiguration("auto_arm")
    dry_run = LaunchConfiguration("dry_run")
    use_hardware = LaunchConfiguration("use_hardware")
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
            "gui": gazebo_gui,
        }.items(),
    )

    serial_bridge = Node(
        package="volt_description",
        executable="volt_serial_bridge.py",
        name="volt_serial_bridge",
        output="screen",
        parameters=[{
            "port": serial_port,
            "baud_rate": baud_rate,
            "auto_arm": auto_arm,
            # This suspended single-joint tool has no motion-controller status.
            "require_motion_safe_to_arm": False,
            "dry_run": dry_run,
            "hardware_enabled": use_hardware,
            "calibration_file": calibration_file,
        }],
    )

    command_router = Node(
        package="volt_description",
        executable="volt_joint_command_router.py",
        name="volt_joint_command_router",
        output="screen",
    )

    joint_test_gui = Node(
        package="volt_description",
        executable="volt_joint_test_gui.py",
        name="volt_joint_test_gui",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Arduino Nano serial device",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="57600",
            description="Arduino firmware baud rate",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start Gazebo GUI",
        ),
        DeclareLaunchArgument(
            "auto_arm",
            default_value="false",
            description="Send ARM to Arduino after connecting",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Log outgoing FRAME packets without opening serial",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Allow serial bridge to open Arduino serial port",
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
        TimerAction(period=11.0, actions=[command_router]),
        TimerAction(period=12.0, actions=[serial_bridge]),
        TimerAction(period=14.0, actions=[joint_test_gui]),
    ])
