# G1 Nav2 Portable Workspace

This is a standalone ROS 2 Humble workspace for Unitree G1 navigation with a
Livox Mid360. It contains the minimum project packages needed for:

- G1 state query, FSM switching, and velocity bridging;
- Mid360/IMU preprocessing and FAST-LIO localization;
- PCD to Nav2 `map.pgm` / `map.yaml` conversion;
- AMCL, DWB navigation, point-cloud obstacle avoidance, and RViz.

## Included Packages

- `unitree_api`: Unitree sport API messages.
- `livox_ros_driver2`: Livox messages required by FAST-LIO.
- `fast_lio`: FAST-LIO mapping/local odometry and `pcd_to_nav2_map`.
- `g1_fastlio_bridge`: G1 Mid360 point-cloud and IMU preprocessing.
- `g1_robot_control`: G1 state/FSM client and Nav2 velocity bridge.
- `g1_nav2_bringup`: Nav2 launch files, AMCL, DWB, costmaps, and RViz launch.

The live robot must provide `/utlidar/cloud_livox_mid360`, `/utlidar/imu_livox_mid360`,
`/api/sport/request`, and `/api/sport/response`.

## Dependencies And Build

Install Nav2 and the point-cloud-to-laser converter:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan
```

Install any remaining system dependencies, then build this workspace only:

```bash
cd /path/to/nav2
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## G1 State And FSM

```bash
ros2 run g1_robot_control g1_loco_client_example --get_fsm_id
ros2 run g1_robot_control g1_loco_client_example --set_fsm_id=801
ros2 run g1_robot_control g1_loco_client_example --stop_move
```

`FSM 801` is the G1 locomotion state used before navigation. Keep an emergency
stop and an operator available before sending motion commands.

## Convert A PCD Map

The PCD z origin is the lidar height. Use the same physical slice for the PCD
map and online `/scan`.

```bash
mkdir -p maps
ros2 run fast_lio pcd_to_nav2_map \
  maps/g1_fastlio.pcd \
  maps \
  0.05 \
  -1.20 \
  0.30 \
  6
```

This writes `maps/map.pgm` and `maps/map.yaml`. The online scan filter in
`g1_nav2_bringup/config/pointcloud_to_laserscan.yaml` uses the same
`-1.20..0.30 m` slice relative to `base_link`.

## Start Navigation

```bash
ros2 launch g1_nav2_bringup g1_nav2.launch.py \
  map:=$(pwd)/src/maps/map.yaml \
  cmd_vel_topic:=/g1_nav_cmd_vel
```

Nav2 commands follow this private chain and do not subscribe to joystick
`/cmd_vel`:

```text
/cmd_vel_nav -> velocity_smoother -> /g1_nav_cmd_vel -> G1 bridge
```

## Open RViz

In a second terminal:

```bash
cd /path/to/nav2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch g1_nav2_bringup g1_nav2_rviz.launch.py
```

Set the fixed frame to `map`. Wait for AMCL localization to converge before
sending a `Nav2 Goal`.


默认参数 `auto_global_localization:=false`。启动后不会自动进行全局自定位，
也不会自动在整张地图上初始化 AMCL 粒子。请在 RViz 中点击 `2D Pose Estimate`，
手动设置机器人在地图中的位置和朝向，等待 AMCL 粒子云稳定后再发送导航目标。

如果需要启用自动定位，必须显式添加 `auto_global_localization:=true`。自动定位
节点会：

1. 请求 AMCL 全局重定位，在整张地图上初始化粒子；
2. 以约 `0.30 rad/s` 控制机器人原地旋转一圈；
3. 停止机器人并请求多次静止扫描更新；
4. 根据 `/amcl_pose` 协方差判断定位是否收敛。

旋转期间机器人附近必须保持空旷。只有看到类似下面的日志后再发送导航目标：

```text
Automatic localization converged after ... stationary scan updates.
```

需要使用自动定位时：

```bash
ros2 launch g1_nav2_bringup g1_nav2.launch.py \
  map:=src/maps/map.yaml \
  cmd_vel_topic:=/g1_nav_cmd_vel \
  auto_global_localization:=true
```

自动定位失败时，优先检查 `/scan`、FAST-LIO 里程计和 TF 是否正常，而不是直接
重复发送目标点。