from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COMMANDS_REQUIRING_VALUE = {
    "continous_gait",
    "move",
    "set_balance_mode",
    "set_fsm_id",
    "set_speed_mode",
    "set_stand_height",
    "set_swing_height",
    "set_task_id",
    "set_velocity",
    "switch_move_mode",
}


def build_loco_node(context, *args, **kwargs):
    command = LaunchConfiguration("command").perform(context).strip()
    value = LaunchConfiguration("value").perform(context).strip()

    if command.startswith("--"):
        command = command[2:]

    if not command:
        raise RuntimeError("launch argument 'command' must not be empty")

    if not value and command in COMMANDS_REQUIRING_VALUE:
        raise RuntimeError(
            "launch argument 'value' is required for command '%s'" % command
        )

    loco_arg = "--%s" % command
    if value:
        loco_arg = "%s=%s" % (loco_arg, value)

    return [
        Node(
            package="unitree_ros2_example",
            executable="g1_loco_client_example",
            name="unitree_g1_control_node",
            output="screen",
            arguments=[loco_arg],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("command", default_value="get_fsm_id"),
            DeclareLaunchArgument("value", default_value=""),
            OpaqueFunction(function=build_loco_node),
        ]
    )
