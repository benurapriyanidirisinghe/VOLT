from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    emote = LaunchConfiguration("emote")
    emote_file = LaunchConfiguration("emote_file")
    speed_scale = LaunchConfiguration("speed_scale")

    player = Node(
        package="volt_description",
        executable="volt_emote_player.py",
        name="volt_emote_player",
        output="screen",
        parameters=[{
            "emote": emote,
            "emote_file": emote_file,
            "speed_scale": speed_scale,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "emote",
            default_value="stand_ready",
            description="Emote name from the volt_description/emotes directory.",
        ),
        DeclareLaunchArgument(
            "emote_file",
            default_value="",
            description="Optional absolute path to a custom emote YAML file.",
        ),
        DeclareLaunchArgument(
            "speed_scale",
            default_value="1.0",
            description="Playback speed multiplier; >1 is faster.",
        ),
        player,
    ])
