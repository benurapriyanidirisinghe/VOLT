"""Robot-side stack for the split Jetson / workstation setup.

Runs ON THE JETSON, which is the machine the Arduino is plugged into. It
starts exactly the three nodes that must sit next to the hardware:

    volt_motion_controller   the 100 Hz gait/IK loop
    volt_joint_command_router  ownership arbitration
    volt_serial_bridge       the 60 Hz binary FRAME link to the Arduino

and nothing else. No Gazebo, no control GUI, no gamepad -- those live on the
workstation and reach these nodes over DDS (see volt_operator.launch.py).

Why the split is drawn here: the serial link and the control loop are the
only parts with a real deadline. The firmware disarms after 750 ms without a
frame, so the loop feeding it should not be sharing a machine with a
renderer. Everything downstream of the router is either advisory (the GUI's
status view) or purely visual (the Gazebo shadow), and both tolerate the
network hop.

This file does not replace anything. volt_start.launch.py and the SIMULATION
and PHYSICAL desktop icons are untouched and still run the whole stack on one
machine.

DDS: both machines need the same ROS_DOMAIN_ID and the UDP-only Fast DDS
profile (~/.config/volt/fastdds_no_shm.xml). The profile is required on this
robot regardless -- shared memory rots across a kill -9 -- and it happens to
be exactly what a two-machine setup needs, since shared memory cannot cross
a network anyway. The launcher exports both.
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

    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    dry_run = LaunchConfiguration("dry_run")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_ready_pose = LaunchConfiguration("auto_ready_pose")
    start_serial_bridge = LaunchConfiguration("start_serial_bridge")
    calibration_file = LaunchConfiguration("calibration_file")
    joint_rate_diagnostic = LaunchConfiguration("joint_rate_diagnostic")
    joint_rate_diagnostic_output = LaunchConfiguration(
        "joint_rate_diagnostic_output"
    )
    real_robot_profiles_file = LaunchConfiguration("real_robot_profiles_file")
    emote_config_file = LaunchConfiguration("emote_config_file")
    enable_physical_tests = LaunchConfiguration("enable_physical_tests")

    # The motion controller and router come from the same file the
    # single-machine stack uses, with gui:=false. Reusing it rather than
    # restating the node definitions keeps the two paths from drifting: a
    # parameter added for the desktop stack reaches the Jetson for free.
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "control.launch.py"])
        ),
        launch_arguments={
            "gui": "false",
            "auto_ready_pose": auto_ready_pose,
            # No Gazebo on this machine, so there is no /clock to follow.
            "use_sim_time": "false",
            "hardware_mode": "true",
            "open_loop_hardware": "true",
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
            # Always true here: this launch file exists because the Arduino
            # is on this machine. dry_run is the switch that decides whether
            # frames actually reach the servos.
            "hardware_enabled": True,
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
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Arduino serial device on the Jetson",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="250000",
            description="Arduino firmware baud rate",
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
            description="Never enabled from here; ARM is a GUI decision",
        ),
        DeclareLaunchArgument(
            "auto_ready_pose",
            default_value="false",
            description="Automatically move from the loaded pose to walk-ready",
        ),
        DeclareLaunchArgument(
            "start_serial_bridge",
            default_value="true",
            description="Start the Arduino bridge (the point of this file)",
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
            "joint_rate_diagnostic",
            default_value="false",
            description="Record per-channel commanded deg/s before the send gate",
        ),
        DeclareLaunchArgument(
            "joint_rate_diagnostic_output",
            default_value="",
            description="Optional CSV path for the joint-rate diagnostic",
        ),
        control_launch,
        serial_bridge,
    ])
