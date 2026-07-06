from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "volt_description"

    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "urdf",
        "volt.urdf.xacro"
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "rviz",
        "volt.rviz"
    ])

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Start joint_state_publisher_gui"
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description]
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(use_gui)
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(use_rviz)
        ),
    ])