from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    package_name = "volt_description"
    package_share_path = get_package_share_directory(package_name)
    resource_root = os.path.dirname(package_share_path)

    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.pathsep.join(
            path for path in (
                resource_root,
                os.environ.get("GAZEBO_MODEL_PATH", ""),
            )
            if path
        ),
    )

    gui = LaunchConfiguration("gui")

    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "urdf",
        "volt.urdf.xacro"
    ])

    world_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "worlds",
        "empty.world"
    ])

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
            " sim_backend:=classic"
        ])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"),
            "/launch",
            "/gazebo.launch.py"
        ]),
        launch_arguments={
            "world": world_file,
            "gui": gui,
            "verbose": "true",
            "pause" : "false"
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    spawn_robot = Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    name="spawn_volt",
    output="screen",
    arguments=[
        "-topic", "robot_description",
        "-entity", "volt",
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo GUI"
        ),

        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,

        TimerAction(
            period=5.0,
            actions=[joint_state_broadcaster_spawner]
        ),

        TimerAction(
            period=8.0,
            actions=[joint_group_position_controller_spawner]
),
    ])
