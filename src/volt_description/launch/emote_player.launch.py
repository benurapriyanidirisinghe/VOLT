from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    emote = LaunchConfiguration("emote")
    emote_file = LaunchConfiguration("emote_file")
    speed_scale = LaunchConfiguration("speed_scale")
    repetitions = LaunchConfiguration("repetitions")
    amplitude = LaunchConfiguration("amplitude")
    depth = LaunchConfiguration("depth")

    player = Node(
        package="volt_description",
        executable="volt_emote_player.py",
        name="volt_emote_player",
        output="screen",
        parameters=[{
            "emote": emote,
            "emote_file": emote_file,
            "speed_scale": ParameterValue(speed_scale, value_type=float),
            "repetitions": ParameterValue(repetitions, value_type=int),
            "amplitude": ParameterValue(amplitude, value_type=float),
            "depth": ParameterValue(depth, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "emote",
            default_value="stand_ready",
            description=(
                "Controller action/emote name (legacy stand_ready and "
                "small_dance aliases are accepted)."
            ),
        ),
        DeclareLaunchArgument(
            "emote_file",
            default_value="",
            description=(
                "Deprecated compatibility argument; non-empty custom files "
                "are rejected."
            ),
        ),
        DeclareLaunchArgument(
            "speed_scale",
            default_value="1.0",
            description="Cartesian playback speed, clamped to [0.5, 2.0].",
        ),
        DeclareLaunchArgument(
            "repetitions",
            default_value="1",
            description="Cartesian repetitions, clamped to [1, 5].",
        ),
        DeclareLaunchArgument(
            "amplitude",
            default_value="1.0",
            description="Cartesian lateral/attitude scale, clamped to [0.5, 1.5].",
        ),
        DeclareLaunchArgument(
            "depth",
            default_value="1.0",
            description="Cartesian vertical scale, clamped to [0.5, 1.5].",
        ),
        player,
    ])
