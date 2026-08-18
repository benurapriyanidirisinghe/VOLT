from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    auto_ready_pose = LaunchConfiguration("auto_ready_pose")
    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_arm = LaunchConfiguration("auto_arm")
    dry_run = LaunchConfiguration("dry_run")
    hardware_enabled = LaunchConfiguration("hardware_enabled")
    calibration_file = LaunchConfiguration("calibration_file")
    real_robot_profiles_file = LaunchConfiguration("real_robot_profiles_file")
    emote_config_file = LaunchConfiguration("emote_config_file")
    enable_physical_tests = LaunchConfiguration("enable_physical_tests")
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
        parameters=[
            config_file,
            {
                "auto_ready_pose": auto_ready_pose,
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
                "hardware_mode": True,
                "control_rate": 100.0,
                # Hobby servos provide no JointState feedback. Seed the
                # canonical WALK_POSE that exactly matches the firmware's
                # calibrated safe-start frame only in this hardware-only stack;
                # router HOLD and Arduino ARM remain separate motion gates.
                "open_loop_hardware": True,
                "gait_config_file": config_file,
                "real_robot_profiles_file": real_robot_profiles_file,
                "emote_config_file": emote_config_file,
                "enable_physical_tests": ParameterValue(
                    enable_physical_tests,
                    value_type=bool,
                ),
            },
        ],
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
            "dry_run": dry_run,
            "hardware_enabled": hardware_enabled,
            "calibration_file": calibration_file,
            "use_sim_time": False,
        }],
    )

    command_router = Node(
        package="volt_description",
        executable="volt_joint_command_router.py",
        name="volt_joint_command_router",
        output="screen",
    )

    control_gui = Node(
        package="volt_description",
        executable="volt_control_gui.py",
        name="volt_control_gui",
        output="screen",
        condition=IfCondition(gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start the guided VOLT physical-robot control GUI",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Arduino Nano serial device",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="250000",
            description="Arduino firmware baud rate",
        ),
        DeclareLaunchArgument(
            "auto_ready_pose",
            default_value="false",
            description="Automatically move from loaded zero pose to walk-ready pose",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use an external simulation clock when one is available",
        ),
        DeclareLaunchArgument(
            "auto_arm",
            default_value="false",
            description=(
                "Request ARM after handshake and a fresh certification of "
                "the stopped calibrated WALK_POSE"
            ),
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Do not open serial; log outgoing FRAME packets",
        ),
        DeclareLaunchArgument(
            "hardware_enabled",
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
        DeclareLaunchArgument(
            "real_robot_profiles_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("volt_description"),
                "config",
                "real_robot_profiles.yaml",
            ]),
            description="Validated real-robot tuning profiles",
        ),
        DeclareLaunchArgument(
            "emote_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("volt_description"),
                "config",
                "cartesian_emotes.yaml",
            ]),
            description="Validated Cartesian emote catalog",
        ),
        DeclareLaunchArgument(
            "enable_physical_tests",
            default_value="false",
            description=(
                "Enable the leased support-stand Cartesian test request topic"
            ),
        ),
        command_router,
        motion_controller,
        serial_bridge,
        control_gui,
    ])
