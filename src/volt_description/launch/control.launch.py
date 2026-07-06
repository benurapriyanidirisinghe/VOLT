from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("gui")
    config_file = PathJoinSubstitution([
        FindPackageShare("volt_description"),
        "config",
        "gait_controller.yaml",
    ])

    controller = Node(
        package="volt_description",
        executable="volt_motion_controller.py",
        name="volt_motion_controller",
        output="screen",
        parameters=[config_file],
    )

    gui = Node(
        package="volt_description",
        executable="volt_control_gui.py",
        output="screen",
        condition=IfCondition(use_gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the VOLT PyQt control window",
        ),
        controller,
        gui,
    ])
