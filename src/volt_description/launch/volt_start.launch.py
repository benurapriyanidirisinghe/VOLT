from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("volt_description")
    gui = LaunchConfiguration("gui")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    auto_ready_pose = LaunchConfiguration("auto_ready_pose")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_hardware = LaunchConfiguration("use_hardware")
    dry_run = LaunchConfiguration("dry_run")
    auto_arm = LaunchConfiguration("auto_arm")
    start_serial_bridge = LaunchConfiguration("start_serial_bridge")
    calibration_file = LaunchConfiguration("calibration_file")
    joint_rate_diagnostic = LaunchConfiguration("joint_rate_diagnostic")
    joint_rate_diagnostic_output = LaunchConfiguration(
        "joint_rate_diagnostic_output"
    )
    real_robot_profiles_file = LaunchConfiguration("real_robot_profiles_file")
    emote_config_file = LaunchConfiguration("emote_config_file")
    enable_physical_tests = LaunchConfiguration("enable_physical_tests")
    use_sim_time = LaunchConfiguration("use_sim_time")
    actuator_profile = LaunchConfiguration("actuator_profile")
    hardware_enabled = LaunchConfiguration("hardware_enabled")
    effective_hardware_enabled = PythonExpression([
        "'true' if ('",
        use_hardware,
        "'.lower() == 'true' or '",
        hardware_enabled,
        "'.lower() == 'true') else 'false'",
    ])

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
            "use_sim_time": use_sim_time,
            "actuator_profile": actuator_profile,
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
            "use_sim_time": use_sim_time,
            "hardware_mode": effective_hardware_enabled,
            "open_loop_hardware": effective_hardware_enabled,
            "real_robot_profiles_file": real_robot_profiles_file,
            "emote_config_file": emote_config_file,
            "enable_physical_tests": enable_physical_tests,
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
            "auto_arm": ParameterValue(auto_arm, value_type=bool),
            "dry_run": ParameterValue(dry_run, value_type=bool),
            "hardware_enabled": ParameterValue(
                effective_hardware_enabled,
                value_type=bool,
            ),
            "calibration_file": calibration_file,
            "joint_rate_diagnostic": ParameterValue(
                joint_rate_diagnostic,
                value_type=bool,
            ),
            "joint_rate_diagnostic_output": joint_rate_diagnostic_output,
            "use_sim_time": False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the VOLT PyQt control GUI",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value=gui,
            description="Start Gazebo Sim GUI (defaults to the gui argument)",
        ),
        DeclareLaunchArgument(
            "auto_ready_pose",
            default_value="false",
            description="Automatically move from loaded zero pose to walk-ready pose",
        ),
        DeclareLaunchArgument(
            "start_serial_bridge",
            default_value="false",
            description="Start the Arduino bridge (disabled unless explicitly requested)",
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
            "joint_rate_diagnostic",
            default_value="false",
            description=(
                "Record per-channel commanded deg/s before the send-rate gate, "
                "for tuning gait limits against the firmware slew ceiling"
            ),
        ),
        DeclareLaunchArgument(
            "joint_rate_diagnostic_output",
            default_value="",
            description="Optional CSV path for the joint-rate diagnostic",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Allow serial bridge to open Arduino serial port",
        ),
        DeclareLaunchArgument(
            "hardware_enabled",
            default_value="false",
            description="Alias for use_hardware; explicitly allow serial hardware",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Log outgoing FRAME packets without opening serial",
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
                pkg_share,
                "config",
                "real_robot_profiles.yaml",
            ]),
            description="Validated simulation/real robot tuning profiles",
        ),
        DeclareLaunchArgument(
            "emote_config_file",
            default_value=PathJoinSubstitution([
                pkg_share,
                "config",
                "cartesian_emotes.yaml",
            ]),
            description="Validated Cartesian emote catalog",
        ),
        DeclareLaunchArgument(
            "enable_physical_tests",
            default_value="false",
            description=(
                "Enable finite support-stand physical test requests"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo /clock for simulation nodes and motion control",
        ),
        DeclareLaunchArgument(
            "actuator_profile",
            default_value="simulation",
            choices=["simulation", "td8130mg"],
            description="Gazebo actuator-limit profile",
        ),
        ignition_launch,
        # Router and motion controller are safe to start immediately: the router
        # begins in HOLD and the controller waits for canonical joint feedback.
        gui_launch,
        serial_bridge,
    ])
