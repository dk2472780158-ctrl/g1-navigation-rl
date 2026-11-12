import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_share = get_package_share_directory("nav2_bringup")
    default_config = os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value=default_config),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
