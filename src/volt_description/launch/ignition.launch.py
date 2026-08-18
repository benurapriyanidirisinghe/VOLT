import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def start_after_success(next_action, process_name):
    """Start the next stage only when the preceding process exited cleanly."""

    def on_exit(event, _context):
        if event.returncode == 0:
            return [next_action]
        reason = "%s failed with exit code %d" % (
            process_name,
            event.returncode,
        )
        return [
            LogInfo(msg=reason + "; stopping the partial simulation stack."),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    return on_exit


def generate_launch_description():
    package_name = "volt_description"

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    actuator_profile = LaunchConfiguration("actuator_profile")
    pkg_share = FindPackageShare(package_name)
    package_share_path = get_package_share_directory(package_name)
    resource_root = os.path.dirname(package_share_path)

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
            xacro_file,
            " sim_backend:=gz actuator_profile:=",
            actuator_profile,
        ])
    }

    # Package-derived Gazebo resource paths make package:// mesh URIs work from
    # both normal and --symlink-install workspaces without a user/workspace path.
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.pathsep.join(
            path for path in (
                resource_root,
                os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
            )
            if path
        ),
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(
            path for path in (
                resource_root,
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            )
            if path
        ),
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
            ],
            "on_exit_shutdown": "true",
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
        parameters=[
            robot_description,
            {
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            },
        ]
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

    spawn_then_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=start_after_success(
                joint_state_broadcaster_spawner,
                "VOLT entity creation",
            ),
        )
    )
    joint_state_then_position = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=start_after_success(
                joint_group_position_controller_spawner,
                "joint_state_broadcaster spawner",
            ),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo Sim GUI"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the bridged Gazebo /clock for simulation nodes",
        ),
        DeclareLaunchArgument(
            "actuator_profile",
            default_value="simulation",
            choices=["simulation", "td8130mg"],
            description=(
                "Gazebo actuator limits: proven simulation or firmware-rate "
                "TD-8130MG approximation"
            ),
        ),
        set_ign_resource_path,
        set_gz_resource_path,
        spawn_then_joint_state,
        joint_state_then_position,

        gazebo_sim,
        clock_bridge,
        robot_state_publisher,

        TimerAction(
            period=2.0,
            actions=[spawn_robot]
        ),
    ])
