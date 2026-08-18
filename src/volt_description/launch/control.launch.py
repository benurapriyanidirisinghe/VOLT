from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("gui")
    auto_ready_pose = LaunchConfiguration("auto_ready_pose")
    use_sim_time = LaunchConfiguration("use_sim_time")
    hardware_mode = LaunchConfiguration("hardware_mode")
    open_loop_hardware = LaunchConfiguration("open_loop_hardware")
    real_robot_profiles_file = LaunchConfiguration("real_robot_profiles_file")
    emote_config_file = LaunchConfiguration("emote_config_file")
    enable_physical_tests = LaunchConfiguration("enable_physical_tests")
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
            {
                "auto_ready_pose": ParameterValue(
                    auto_ready_pose,
                    value_type=bool,
                ),
                "use_sim_time": ParameterValue(
                    PythonExpression([
                        "False if '",
                        hardware_mode,
                        "'.lower() == 'true' else '",
                        use_sim_time,
                        "'.lower() == 'true'",
                    ]),
                    value_type=bool,
                ),
                "hardware_mode": ParameterValue(
                    hardware_mode,
                    value_type=bool,
                ),
                "open_loop_hardware": ParameterValue(
                    open_loop_hardware,
                    value_type=bool,
                ),
                "control_rate": ParameterValue(
                    PythonExpression([
                        "100.0 if '",
                        hardware_mode,
                        "'.lower() == 'true' else 100.0",
                    ]),
                    value_type=float,
                ),
                "gait_config_file": config_file,
                "real_robot_profiles_file": real_robot_profiles_file,
                "emote_config_file": emote_config_file,
                "enable_physical_tests": ParameterValue(
                    enable_physical_tests,
                    value_type=bool,
                ),
            },
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
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo /clock for the motion controller",
        ),
        DeclareLaunchArgument(
            "hardware_mode",
            default_value="false",
            description="Apply conservative hardware gait speed scaling",
        ),
        DeclareLaunchArgument(
            "open_loop_hardware",
            default_value="false",
            description=(
                "Ignore simulator JointState as physical feedback and seed the "
                "calibrated hardware WALK_POSE"
            ),
        ),
        DeclareLaunchArgument(
            "real_robot_profiles_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("volt_description"),
                "config",
                "real_robot_profiles.yaml",
            ]),
            description="Validated simulation/real robot tuning profiles",
        ),
        DeclareLaunchArgument(
            "emote_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("volt_description"),
                "config",
                "cartesian_emotes.yaml",
            ]),
            description="Validated Cartesian emote catalog",
        ),
        DeclareLaunchArgument(
            "enable_physical_tests",
            default_value="false",
            description=(
                "Enable finite physical tests (hardware mode still required)"
            ),
        ),
        command_router,
        controller,
        gui,
    ])
