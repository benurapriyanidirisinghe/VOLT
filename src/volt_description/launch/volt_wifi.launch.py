"""ESP32-S3 servo board over WiFi, with no simulator.

Everything runs on this workstation -- console, gamepad, motion controller,
router, serial bridge -- and the bridge opens a TCP socket to the ESP32
instead of a USB serial port. The ESP32 replaces the Arduino and its cable
at once, so there is no second computer in the servo path.

NO GAZEBO. This mode composes the nodes directly rather than including
volt_start.launch.py, because that file always brings Ignition up and the
simulator has nothing to contribute here: under hardware the motion
controller is open-loop and ignores /joint_states entirely, so Ignition was
only ever rendering the COMMANDED pose -- useful for a demo, and pure cost
when the real robot is in front of you. Dropping it also frees the GPU and
removes gz_ros2_control, robot_state_publisher and the clock bridge from a
stack that has a 750 ms disarm deadline to meet.

Set gazebo:=true to bring the visual shadow back for a single run.

The single-machine SIMULATION, GUI, PHYSICAL and JETSON paths are untouched;
volt_start.launch.py is not modified by any of this.

WHAT WIFI CHANGES: the firmware disarms after 750 ms without a frame, so
that timeout stops being a backstop and becomes the primary protection. The
host also drops frames it cannot send immediately rather than queueing them
-- a burst of stale servo targets arriving after a stall is worse than a
gap, and the sequence counter makes the gap visible in STATUS.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("volt_description")

    board_endpoint = LaunchConfiguration("board_endpoint")
    gui = LaunchConfiguration("gui")
    gazebo = LaunchConfiguration("gazebo")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    dry_run = LaunchConfiguration("dry_run")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_ready_pose = LaunchConfiguration("auto_ready_pose")
    calibration_file = LaunchConfiguration("calibration_file")
    real_robot_profiles_file = LaunchConfiguration("real_robot_profiles_file")
    emote_config_file = LaunchConfiguration("emote_config_file")
    enable_physical_tests = LaunchConfiguration("enable_physical_tests")
    actuator_profile = LaunchConfiguration("actuator_profile")

    # Motion controller, router and console come from the same file the
    # single-machine stack uses. Reusing it rather than restating the node
    # definitions keeps the two paths from drifting apart.
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "control.launch.py"])
        ),
        launch_arguments={
            "gui": gui,
            "auto_ready_pose": auto_ready_pose,
            # No simulator here, so there is no /clock to follow. The
            # authority is this machine's wall clock.
            "use_sim_time": "false",
            "hardware_mode": "true",
            "open_loop_hardware": "true",
            "real_robot_profiles_file": real_robot_profiles_file,
            "emote_config_file": emote_config_file,
            "enable_physical_tests": enable_physical_tests,
        }.items(),
    )

    # Off by default. Present so the shadow can be brought back for a demo
    # without editing anything.
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "ignition.launch.py"])
        ),
        condition=IfCondition(gazebo),
        launch_arguments={
            "gui": gazebo_gui,
            "use_sim_time": "false",
            "actuator_profile": actuator_profile,
        }.items(),
    )

    serial_bridge = Node(
        package="volt_description",
        executable="volt_serial_bridge.py",
        name="volt_serial_bridge",
        output="screen",
        parameters=[{
            # The bridge branches on the tcp:// prefix; nothing above it
            # needs to know the transport changed.
            "port": board_endpoint,
            "baud_rate": 250000,
            "auto_arm": ParameterValue(auto_arm, value_type=bool),
            "dry_run": ParameterValue(dry_run, value_type=bool),
            "hardware_enabled": True,
            "calibration_file": calibration_file,
            "joint_rate_diagnostic": False,
            "joint_rate_diagnostic_output": "",
            "use_sim_time": False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "board_endpoint",
            default_value="tcp://volt-esp32.local:3333",
            description=(
                "ESP32 servo board, as tcp://host[:port]. Use the board's IP "
                "if mDNS is unreliable on your network"
            ),
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the VOLT control console and its gamepad poll",
        ),
        DeclareLaunchArgument(
            "gazebo",
            default_value="false",
            description=(
                "Start the Ignition shadow. OFF here: the hardware is "
                "open-loop, so it renders the commanded pose, not a "
                "measurement -- cost without evidence when the real robot "
                "is in front of you"
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Show the Ignition window when gazebo:=true",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description=(
                "Log outgoing FRAME packets instead of sending them. Default "
                "true: a stack that comes up talking to servos by accident "
                "is not a default worth having"
            ),
        ),
        DeclareLaunchArgument(
            "auto_arm",
            default_value="false",
            description="Never enabled from here; ARM is a console decision",
        ),
        DeclareLaunchArgument(
            "auto_ready_pose",
            default_value="false",
            description="Automatically move from the loaded pose to walk-ready",
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=PathJoinSubstitution([
                pkg_share, "config", "servo_calibration.yaml",
            ]),
            description="Servo calibration YAML",
        ),
        DeclareLaunchArgument(
            "real_robot_profiles_file",
            default_value=PathJoinSubstitution([
                pkg_share, "config", "real_robot_profiles.yaml",
            ]),
            description="Validated real-robot tuning profiles",
        ),
        DeclareLaunchArgument(
            "emote_config_file",
            default_value=PathJoinSubstitution([
                pkg_share, "config", "cartesian_emotes.yaml",
            ]),
            description="Validated Cartesian emote catalog",
        ),
        DeclareLaunchArgument(
            "enable_physical_tests",
            default_value="true",
            description="Allow finite support-stand physical test requests",
        ),
        DeclareLaunchArgument(
            "actuator_profile",
            default_value="simulation",
            choices=["simulation", "td8130mg"],
            description="Gazebo actuator-limit profile (only when gazebo:=true)",
        ),
        ignition_launch,
        control_launch,
        serial_bridge,
    ])
