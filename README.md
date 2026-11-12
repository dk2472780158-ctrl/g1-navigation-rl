# G1 人形机器人导航与强化学习步态部署

统一收录 G1 激光 SLAM、定位导航与 PPO / LSTM 步态训练及 Sim2Real 部署代码。

## 目录

| 模块 | 内容 | 文档 |
| --- | --- | --- |
| `g1_nav2/` | AMCL、Nav2、地图转换及官方运动接口速度桥 | [导航说明](g1_nav2/README.md) |
| `git_slam/` | MID360 / IMU 预处理与 FAST-LIO 激光惯性里程计 | [SLAM 说明](git_slam/README.md) |
| `g1_sim2real/` | Isaac Gym PPO / LSTM 训练、TorchScript 导出、MuJoCo 评估、SDK2 真机部署 | [强化学习说明](g1_sim2real/README.md) |

## 系统关系

```text
MID360 / IMU → FAST-LIO → 里程计 / 点云地图
地图 + 激光扫描 + 里程计 → AMCL / Nav2 → 速度桥 → 官方运动控制器

Isaac Gym + PPO / LSTM → TorchScript → MuJoCo 评估 → SDK2 低层策略部署
```

导航高层控制与 PPO 低层控制目前是两条独立执行链。Nav2 不会自动加载 PPO 策略，不能同时占用机器人运动控制权。本仓库的合并是工程代码归档，不代表已经打通导航驱动自训练步态。

## 构建与依赖

三个模块分别使用自己的构建和运行环境；请进入对应目录，按模块说明操作。**不要在总仓库根目录执行 `colcon build`**：导航和 SLAM 中存在同名 ROS 包，应分别构建。Isaac Gym 训练、MuJoCo 评估及机器人部署所需 Python / CUDA / SDK 环境也应分别配置。

强化学习策略配置包含 47 维 Actor 观测、12 维双腿动作及 50 Hz 策略更新；模型与来源记录位于 `g1_sim2real/g1_walk/manifest.json`。训练参数、随机化范围及部署条件以代码和配置为准。

## 来源与许可证

合并保留三个 GitHub 子项目的提交历史与第三方许可证；各模块 `SOURCE.md` 记录对应 Gitea 源码提交。第三方算法、SDK 与机器人资源遵循各自原有许可证，本仓库不为全部文件另行授予统一许可证。依赖缓存及构建产物未收录。

原独立仓库：
- https://github.com/dk2472780158-ctrl/g1_nav2
- https://github.com/dk2472780158-ctrl/git_slam
- https://github.com/dk2472780158-ctrl/g1_sim2real
