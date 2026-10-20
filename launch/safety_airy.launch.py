"""Publish Safety Airy point clouds, IMU, diagnostics and raw MSOP/DIFOP."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("rslidar_sdk"), "config", "safety_airy.yaml"
            ]),
        ),
        Node(
            package="rslidar_sdk",
            executable="rslidar_sdk_node",
            output="screen",
            parameters=[{"config_path": LaunchConfiguration("config_path")}],
        ),
    ])
