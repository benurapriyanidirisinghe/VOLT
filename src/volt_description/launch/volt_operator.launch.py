"""Workstation-side console for the split Jetson / workstation setup.

Runs ON THE PC. Starts the operator-facing half and none of the robot half:

    volt_control_gui   the console, and the gamepad (pygame polls it here)
    Ignition           optional 3D shadow of what the robot is being told

The motion controller, router and serial bridge run on the Jetson
(volt_jetson.launch.py) and are reached over DDS. Nothing in this file talks
to hardware.

The Gazebo shadow is genuinely a shadow: under hardware the motion controller
is open-loop and ignores /joint_states entirely, so Ignition is showing the
COMMANDED pose, not a measurement. It is useful for seeing what was asked for
and worthless as evidence of what the legs did -- that still needs video. It
defaults on because this is the machine with the GPU, and off is one argument
away when the robot is the only thing worth watching.

This file does not replace anything. The SIMULATION and PHYSICAL icons still
run everything on one machine through volt_start.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("volt_description")

    gui = LaunchConfiguration("gui")
    gazebo = LaunchConfiguration("gazebo")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    actuator_profile = LaunchConfiguration("actuator_profile")

    # Ignition brings its own robot_state_publisher and gz_ros2_control, so
    # it is the whole visual half. use_sim_time stays FALSE everywhere in
    # this mode: the authority is the Jetson's wall clock, and a console
    # following Gazebo's /clock would timestamp its commands against a
    # simulation the robot is not running on.
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

    control_gui = Node(
        package="volt_description",
        executable="volt_control_gui.py",
        output="screen",
        condition=IfCondition(gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the VOLT control console (and its gamepad poll)",
        ),
        DeclareLaunchArgument(
            "gazebo",
            default_value="true",
            description=(
                "Start the Ignition shadow. It renders the COMMANDED pose, "
                "not a measurement -- the hardware is open-loop"
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Show the Ignition window (false = headless shadow)",
        ),
        DeclareLaunchArgument(
            "actuator_profile",
            default_value="simulation",
            choices=["simulation", "td8130mg"],
            description="Gazebo actuator-limit profile",
        ),
        ignition_launch,
        control_gui,
    ])
