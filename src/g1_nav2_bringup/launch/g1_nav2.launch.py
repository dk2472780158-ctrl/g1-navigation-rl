import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_share = get_package_share_directory("nav2_bringup")
    package_share = get_package_share_directory("g1_nav2_bringup")
    fastlio_bridge_share = get_package_share_directory("g1_fastlio_bridge")

    default_nav2_params = os.path.join(package_share, "config", "nav2_params.yaml")
    default_scan_params = os.path.join(
        package_share, "config", "pointcloud_to_laserscan.yaml"
    )
    default_fastlio_config_path = os.path.join(fastlio_bridge_share, "config")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    scan_params_file = LaunchConfiguration("scan_params_file")
    start_fastlio = LaunchConfiguration("start_fastlio")
    start_scan_converter = LaunchConfiguration("start_scan_converter")
    start_cmd_vel_bridge = LaunchConfiguration("start_cmd_vel_bridge")
    auto_global_localization = LaunchConfiguration("auto_global_localization")
    localization_rotation = LaunchConfiguration("localization_rotation")
    cloud_topic = LaunchConfiguration("cloud_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    invert_omega = LaunchConfiguration("invert_omega")
    fastlio_config_path = LaunchConfiguration("fast_lio_config_path")
    fastlio_config_file = LaunchConfiguration("fast_lio_config_file")
    fastlio_scratch_map = LaunchConfiguration("fast_lio_scratch_map")

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                fastlio_bridge_share,
                "launch",
                "g1_fastlio_mapping.launch.py",
            )
        ),
        condition=IfCondition(start_fastlio),
        launch_arguments={
            "fast_lio_config_path": fastlio_config_path,
            "fast_lio_config_file": fastlio_config_file,
            "map_file_path": fastlio_scratch_map,
            "rviz": "false",
        }.items(),
    )

    scan_converter = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        condition=IfCondition(start_scan_converter),
        parameters=[scan_params_file],
        remappings=[
            ("cloud_in", cloud_topic),
            ("scan", "/scan"),
        ],
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "map": map_yaml,
            "params_file": nav2_params_file,
            "use_composition": "False",
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "g1_navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": nav2_params_file,
            "cmd_vel_topic": cmd_vel_topic,
        }.items(),
    )

    cmd_vel_bridge = Node(
        package="g1_robot_control",
        executable="g1_cmd_vel_bridge",
        name="g1_cmd_vel_bridge_nav2",
        output="screen",
        condition=IfCondition(start_cmd_vel_bridge),
        parameters=[
            {
                "cmd_vel_topic": cmd_vel_topic,
                "max_vx": 0.60,
                "max_vy": 0.0,
                "max_omega": 0.80,
                "min_linear_speed": 0.30,
                "min_angular_speed": 0.30,
                "deadzone_xy": 0.0,
                "input_is_normalized": False,
                "invert_omega": invert_omega,
                "allow_reverse": False,
                "command_duration": 0.20,
                "watchdog_timeout": 0.60,
                "send_stop_on_timeout": True,
            }
        ],
    )

    global_localizer = Node(
        package="g1_nav2_bringup",
        executable="g1_auto_global_localization",
        name="g1_auto_global_localization",
        output="screen",
        condition=IfCondition(auto_global_localization),
        parameters=[
            {
                "enable_rotation": localization_rotation,
                "cmd_vel_input_topic": "/cmd_vel_nav",
                "odom_topic": "/Odometry",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "map",
                description="Nav2 occupancy map yaml generated from the FAST-LIO PCD map.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
                description="Nav2 parameter file for the G1.",
            ),
            DeclareLaunchArgument(
                "scan_params_file",
                default_value=default_scan_params,
                description="Parameters for pointcloud_to_laserscan.",
            ),
            DeclareLaunchArgument("start_fastlio", default_value="true"),
            DeclareLaunchArgument(
                "start_scan_converter",
                default_value="true",
                description="Start pointcloud_to_laserscan unless another /scan source is already running.",
            ),
            DeclareLaunchArgument("start_cmd_vel_bridge", default_value="true"),
            DeclareLaunchArgument(
                "auto_global_localization",
                default_value="false",
                description="Automatically initialize AMCL from the full map after startup.",
            ),
            DeclareLaunchArgument(
                "localization_rotation",
                default_value="true",
                description="Rotate in place during AMCL global localization.",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/g1_fastlio/cloud_livox_mid360_upright",
                description="PointCloud2 topic converted to /scan for AMCL and costmaps.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/nav2_cmd_vel",
                description="Navigation-only velocity topic consumed by the G1 bridge.",
            ),
            DeclareLaunchArgument(
                "invert_omega",
                default_value="false",
                description="Invert angular.z before sending it to the G1 locomotion API.",
            ),
            DeclareLaunchArgument(
                "fast_lio_config_path",
                default_value=default_fastlio_config_path,
            ),
            DeclareLaunchArgument(
                "fast_lio_config_file",
                default_value="g1_mid360_fastlio.yaml",
            ),
            DeclareLaunchArgument(
                "fast_lio_scratch_map",
                default_value="/tmp/g1_fastlio_nav_odom.pcd",
                description="Unused scratch path if /map_save is called during navigation.",
            ),
            fastlio_launch,
            scan_converter,
            localization_launch,
            navigation_launch,
            cmd_vel_bridge,
            global_localizer,
        ]
    )
