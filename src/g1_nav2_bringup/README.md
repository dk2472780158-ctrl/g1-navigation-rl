# G1 Nav2 Bringup

这个包把现有 G1 + Mid360 + FAST-LIO 建图流程接到 Nav2 导航。

## 流程

1. 用已有 C++ 工具把 FAST-LIO 保存的 PCD 转成 Nav2 可加载的 `pgm/yaml`。
2. 导航时继续运行 FAST-LIO，提供局部里程计 `camera_init -> body`。
3. AMCL 读取 2D 静态地图和 `/scan`，发布 `map -> camera_init`。
4. `pointcloud_to_laserscan` 把 Mid360 点云转成 `/scan`，供 AMCL 和 costmap 使用。
5. Nav2 先发布内部话题 `/cmd_vel_nav`，平滑后发布导航专用话题，
   `g1_cmd_vel_bridge` 再转发为 Unitree 运动 API。

## 依赖

```bash
sudo apt install ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-pointcloud-to-laserscan
```

局部控制器使用 Nav2 内置的 DWB，不需要 TEB 或 `libg2o`。

```bash
colcon build
source install/setup.bash
```

## PCD 转地图

已有 C++ 转换器在 `fast_lio` 包内，参数顺序为：

```text
INPUT.pcd OUTPUT_DIR [resolution] [z_min] [z_max] [min_component_cells]
```

示例：

```bash
ros2 run fast_lio pcd_to_nav2_map \
  src/map/g1_fastlio.pcd \
  src/map \
  0.05 \
  0.05 \
  2.0 \
  4
```

输出文件：

- `src/map/map.pgm`
- `src/map/map.yaml`

如果地图被地面点堵住，增大 `z_min`；如果墙体、桌腿等障碍物缺失，增大
`z_max` 或降低 `min_component_cells`。

当前 `base_link` 原点位于雷达高度附近，在线 `/scan` 默认保留
`-1.20..0.30 m`，对应离地约 `0.12..1.62 m`，与当前 PCD 转地图参数一致。
如果生成地图时修改了 `z_min/z_max`，也要同步修改
`config/pointcloud_to_laserscan.yaml` 的 `min_height/max_height`。

## 获取 PGM/YAML 后启动导航

启动前确认机器人已经处于可接收速度指令的走跑控制状态，并保留急停和人工保护。

终端 1：启动 Nav2 + FAST-LIO + 点云转 scan + G1 速度桥。

```bash
cd /home/xu22/real_robot/g1slam
source install/setup.bash

ros2 launch g1_nav2_bringup g1_nav2.launch.py \
  map:=$(pwd)/src/map/map.yaml
```

如果启动时报 `package 'pointcloud_to_laserscan' not found`，说明还没安装
`ros-humble-pointcloud-to-laserscan`：

```bash
sudo apt install ros-humble-pointcloud-to-laserscan
```

如果你已经有其他节点发布 `/scan`，可以跳过本包内的点云转 scan：

```bash
ros2 launch g1_nav2_bringup g1_nav2.launch.py \
  map:=$(pwd)/src/map/map.yaml \
  start_scan_converter:=false
```

默认导航速度话题是 `/nav2_cmd_vel`。Nav2 的 velocity smoother 会发布到
`/nav2_cmd_vel`，G1 速度桥也只订阅 `/nav2_cmd_vel`，所以只发布 `/cmd_vel`
的摇杆不会打断导航。

默认启用 `auto_global_localization:=true`。它会让 AMCL 在全图撒粒子，然后通过
`/cmd_vel_nav` 以 `0.30 rad/s` 原地旋转一圈，最后请求静止扫描更新直到协方差收敛。
旋转期间保持周围净空并人工监护，只有终端出现 `Automatic localization converged`
后再发送导航目标。需要禁用旋转时添加 `localization_rotation:=false`。

如果要改成其他名字：

```bash
ros2 launch g1_nav2_bringup g1_nav2.launch.py \
  map:=$(pwd)/src/map/map.yaml \
  cmd_vel_topic:=/g1_nav_cmd_vel
```

终端 2：打开 Nav2 RViz。

```bash
cd /home/xu22/real_robot/g1slam
source install/setup.bash

ros2 launch nav2_bringup rviz_launch.py
```

RViz 操作顺序：

1. Fixed Frame 设为 `map`。
2. 确认能看到地图、机器人 TF、local/global costmap。
3. 点击 `2D Pose Estimate`，在地图上给机器人当前位置和朝向。
4. 等 AMCL 粒子云收敛后，点击 `Nav2 Goal`，先给 0.5-1.0 m 内的小目标。
5. 确认运动正常后再逐步增大目标距离。

## 启动后检查

```bash
ros2 topic hz /Odometry
ros2 topic echo --once /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /nav2_cmd_vel
```

期望 TF 树：

```text
map -> camera_init -> body -> base_link -> livox_frame
```

其中：

- `map -> camera_init` 由 AMCL 发布；
- `camera_init -> body` 由 FAST-LIO 发布；
- `body -> base_link` 和 `base_link -> livox_frame` 由现有预处理 launch 发布。

## 安全限速

默认 Nav2 启动时会以物理速度模式启动 `g1_cmd_vel_bridge`，不是摇杆归一化模式。
初始限制如下：

- `max_vx`: `0.40 m/s`
- `max_vy`: `0.0 m/s`（导航不使用横向移动）
- `max_omega`: `0.60 rad/s`
- G1 导航使用最小可执行速度：非零线速度至少 `0.30 m/s`，非零角速度至少
  `0.30 rad/s`；低于此阈值的 Nav2 平滑输出会被提升，零速度仍会立即停止。
- 默认 `invert_omega:=false`，因此 Nav2 的 `angular.z` 按 ROS 约定直接下发。
  若测试 `angular.z: +0.3` 时机器人不是从上方看逆时针转，启动时改为
  `invert_omega:=true`。
- 超过 `0.6 s` 没收到 `/nav2_cmd_vel` 时自动发送零速度

如需紧急停止：

```bash
ros2 run g1_robot_control g1_loco_client_example --stop_move
```
