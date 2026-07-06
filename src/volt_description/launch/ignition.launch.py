import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "volt_description"

    gui = LaunchConfiguration("gui")
    pkg_share = FindPackageShare(package_name)

    xacro_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "volt.urdf.xacro"
    ])

    world_file = PathJoinSubstitution([
        pkg_share,
        "worlds",
        "empty_ign.sdf"
    ])

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file
        ])
    }

    # Gazebo Sim / Ignition resource paths.
    # Needed so package://volt_description/urdf/stl/*.stl resolves.
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
            ":/home/ros2/Documents/volt_ws/src",
            ":/home/ros2/Documents/volt_ws/install"
        ]
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ":/home/ros2/Documents/volt_ws/src",
            ":/home/ros2/Documents/volt_ws/install"
        ]
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"),
            "/launch",
            "/gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": [
                PythonExpression([
                    "'-r -v 2 ' if '",
                    gui,
                    "' == 'true' else '-r -s -v 2 '"
                ]),
                world_file
            ]
        }.items()
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_volt",
        output="screen",
        arguments=[
            "-world", "default",
            "-topic", "robot_description",
            "-name", "volt",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.30"
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--switch-timeout",
            "30"
        ],
        output="screen"
    )

    joint_group_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_group_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--switch-timeout",
            "30"
        ],
        output="screen"
    )

    set_gazebo_position_gain = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "for i in {1..20}; do "
            "ros2 param set /gz_ros2_control position_proportional_gain 0.35 "
            "&& exit 0; sleep 0.5; done; exit 1",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo Sim GUI"
        ),
        set_ign_resource_path,
        set_gz_resource_path,

        gazebo_sim,
        TimerAction(
            period=1.0,
            actions=[clock_bridge]
        ),
        robot_state_publisher,

        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),

        TimerAction(
            period=7.0,
            actions=[joint_state_broadcaster_spawner]
        ),

        TimerAction(
            period=8.0,
            actions=[set_gazebo_position_gain]
        ),

        TimerAction(
            period=10.0,
            actions=[joint_group_position_controller_spawner]
        ),
    ])
