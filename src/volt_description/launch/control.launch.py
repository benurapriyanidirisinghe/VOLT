from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("gui")
    auto_ready_pose = LaunchConfiguration("auto_ready_pose")
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
        parameters=[
            config_file,
            {"auto_ready_pose": auto_ready_pose},
        ],
    )

    command_router = Node(
        package="volt_description",
        executable="volt_joint_command_router.py",
        name="volt_joint_command_router",
        output="screen",
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
        DeclareLaunchArgument(
            "auto_ready_pose",
            default_value="false",
            description="Automatically move from loaded zero pose to walk-ready pose",
        ),
        command_router,
        controller,
        gui,
    ])
