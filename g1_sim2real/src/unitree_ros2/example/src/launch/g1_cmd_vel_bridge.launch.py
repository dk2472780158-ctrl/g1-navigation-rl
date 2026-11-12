from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    input_mode = LaunchConfiguration("input_mode")
    max_vx = LaunchConfiguration("max_vx")
    max_vy = LaunchConfiguration("max_vy")
    max_omega = LaunchConfiguration("max_omega")
    deadzone_xy = LaunchConfiguration("deadzone_xy")
    command_duration = LaunchConfiguration("command_duration")
    watchdog_timeout = LaunchConfiguration("watchdog_timeout")
    stop_publish_period = LaunchConfiguration("stop_publish_period")
    send_stop_on_timeout = LaunchConfiguration("send_stop_on_timeout")
    log_commands = LaunchConfiguration("log_commands")

    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "input_mode",
                default_value="standard",
                description="Input mapping: standard or web_joystick",
            ),
            DeclareLaunchArgument("max_vx", default_value="0.5"),
            DeclareLaunchArgument("max_vy", default_value="0.3"),
            DeclareLaunchArgument("max_omega", default_value="0.8"),
            DeclareLaunchArgument("deadzone_xy", default_value="0.1"),
            DeclareLaunchArgument("command_duration", default_value="0.25"),
            DeclareLaunchArgument("watchdog_timeout", default_value="0.5"),
            DeclareLaunchArgument("stop_publish_period", default_value="0.1"),
            DeclareLaunchArgument("send_stop_on_timeout", default_value="true"),
            DeclareLaunchArgument("log_commands", default_value="false"),
            Node(
                package="unitree_ros2_example",
                executable="g1_cmd_vel_bridge",
                name="g1_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": cmd_vel_topic,
                        "input_mode": input_mode,
                        "max_vx": max_vx,
                        "max_vy": max_vy,
                        "max_omega": max_omega,
                        "deadzone_xy": deadzone_xy,
                        "command_duration": command_duration,
                        "watchdog_timeout": watchdog_timeout,
                        "stop_publish_period": stop_publish_period,
                        "send_stop_on_timeout": send_stop_on_timeout,
                        "log_commands": log_commands,
                    }
                ],
            ),
        ]
    )
