from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("volt_description")
    gui = LaunchConfiguration("gui")

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
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo and VOLT control GUIs",
        ),
        ignition_launch,
        TimerAction(
            period=12.0,
            actions=[gui_launch],
        ),
    ])
