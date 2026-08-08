"""ROS 2 launch file for the unified dataset player."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _flatten(mapping, prefix=""):
    parameters = {}
    for key, value in mapping.items():
        name = "{}.{}".format(prefix, key) if prefix else str(key)
        if isinstance(value, dict):
            parameters.update(_flatten(value, name))
        else:
            parameters[name] = value
    return parameters


def _player(context):
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser()
    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    if not isinstance(config, dict):
        raise ValueError("playback config must contain a YAML mapping")
    return [
        Node(
            package="dataloader",
            executable="dataloader_player",
            name="dataloader_player",
            output="screen",
            parameters=[_flatten(config)],
        )
    ]


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("dataloader")) / "config" / "mulran.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            OpaqueFunction(function=_player),
        ]
    )
