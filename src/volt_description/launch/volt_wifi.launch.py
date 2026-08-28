"""Single-machine stack driving an ESP32-S3 servo board over WiFi.

Everything runs on this workstation -- console, gamepad, motion controller,
router, serial bridge and the Ignition shadow -- exactly as the PHYSICAL
mode does. The only difference is the last hop: the bridge opens a TCP
socket to the ESP32 instead of a USB serial port.

That is the whole point of the split. The ESP32 replaces the Arduino and the
USB cable at once, so there is no second computer in the servo path and no
cable to the robot. The board is reachable at ``tcp://volt-esp32.local:3333``
by default (the firmware sets that hostname), and the bridge treats a
``tcp://`` port exactly like a device node -- same PROTO 3 frames, same CRC,
same ARM handshake, same guards.

Everything else is deliberately unchanged: volt_start.launch.py is included
as-is, so this mode cannot drift away from the tested single-machine stack.
The SIMULATION, PHYSICAL and JETSON paths are untouched.

WHAT WIFI CHANGES, and what protects against it: the firmware disarms after
750 ms without a frame, and that timeout stops being a backstop and becomes
the primary protection. The host also drops frames it cannot send
immediately rather than queueing them -- a burst of stale servo targets
arriving after a stall is far worse than a gap, and the sequence counter
makes the gap visible in STATUS.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("volt_description")

    board_endpoint = LaunchConfiguration("board_endpoint")
    gui = LaunchConfiguration("gui")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    dry_run = LaunchConfiguration("dry_run")
    auto_ready_pose = LaunchConfiguration("auto_ready_pose")
    enable_physical_tests = LaunchConfiguration("enable_physical_tests")

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "volt_start.launch.py"])
        ),
        launch_arguments={
            "gui": gui,
            "gazebo_gui": gazebo_gui,
            "start_serial_bridge": "true",
            # The bridge branches on the tcp:// prefix; nothing else in the
            # stack needs to know the transport changed.
            "serial_port": board_endpoint,
            "use_hardware": "true",
            "dry_run": dry_run,
            "auto_arm": "false",
            "auto_ready_pose": auto_ready_pose,
            "enable_physical_tests": enable_physical_tests,
            "use_sim_time": "false",
        }.items(),
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
            "gazebo_gui",
            default_value="true",
            description="Show the Ignition shadow window",
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
            "auto_ready_pose",
            default_value="false",
            description="Automatically move from the loaded pose to walk-ready",
        ),
        DeclareLaunchArgument(
            "enable_physical_tests",
            default_value="true",
            description="Allow finite support-stand physical test requests",
        ),
        stack,
    ])
