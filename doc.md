# G1 Walk 真机部署与运行手册

## 1. 部署位置与当前行为

| 项目 | 当前配置 |
| --- | --- |
| 部署目录 | `/workspace/g1_walk` |
| 文档 | `/workspace/doc.md` |
| 机器人通信网卡 | `eth0`，连接机器人内部网络 |
| Python | `/opt/miniconda3/envs/pms-py312/bin/python` |
| PyTorch | 2.4.0，机器人 ARM CPU 推理 |
| Unitree SDK | `/opt/unitree_sdk2_python` |
| 独立依赖目录 | `/workspace/g1_walk/.deps` |
| 策略周期 | 0.02 秒，目标频率 50 Hz |
| 实机配置 | `/workspace/deploy_model/walk.yaml` |
| 部署权重 | `/workspace/deploy_model/walk.pt` |

当前推荐命令包含 `--raw-policy`：直接按仓库约定发送模型目标，不添加额外的关节目标裁剪、目标跳变、跟踪误差、预测力矩或腿部速度中止门槛。自动姿态稳定检测和机身倾角中止门槛已经关闭，初始姿态由现场安全员确认。

保留的检查包括：机器人状态是否更新、数值是否有限、IMU 四元数是否有效、官方控制器是否占用、控制命令是否超时，以及遥控器停止按键。程序按指定时长结束，不自动恢复官方站立控制器。

## 2. 启动流程

### 2.1 现场准备

机器人重启后，官方运动控制器可能重新接管。保持现场吊装保护，机器人处于零力矩或阻尼状态时，按遥控器 **L2+R2** 进入调试模式。由现场安全员确认初始姿态及脚接地情况。

进入调试模式后，还需要执行下面的控制权释放与验证命令。`mode_machine=5` 是机型信息，不能用它判断是否进入调试模式。

### 2.2 进入部署目录

直接执行：

```bash
cd /workspace/g1_walk
```

使用这里提供的启动脚本，它们会设置 Python、SDK、DDS 动态库和线程数。不要直接用系统 `python3 run_fixed_trial.py`，系统 Python 与部署依赖环境不同。

### 2.3 释放并验证官方控制权

直接执行：

```bash
bash release_control.sh
```

这个命令**会改变控制权**，不是只读检查。它调用官方 `MotionSwitcherClient.ReleaseMode()`，然后连续查询 6 次，确认官方控制器名称保持为空，并统计最后 1 秒是否仍有外部 `rt/lowcmd` 命令。

正常输出示例：

```text
{"before": [0, {"form": "0", "name": "ai"}]}
{"release_return": [0, null]}
{"after": [0, {"form": "0", "name": ""}], ...}
...
"last_second_external_lowcmd_count": 0
```

确认 `after` 中的 `name` 是空字符串，且 `last_second_external_lowcmd_count` 是 `0`。如果控制器已释放，脚本会验证现状，不重复切换。

脚本会拒绝释放未知名称的控制器。若释放报错、名称仍为 `ai`、出现其他控制器名称，或最后 1 秒仍有外部电机命令，先处理控制源，不启动行走。当前脚本会记录非零外部命令计数，但不会仅因该计数非零自动报错，需操作人员查看结果。

只运行一个电机控制程序。遥控速度桥、旧 ONNX 控制器、另一份部署脚本可能同时发送命令；停止桥接服务后，也需要验证它是否被管理服务重新启动。

### 2.4 启动行走策略

例如前进速度指令 **0.2 m/s、持续 3 秒**：

```bash
bash run_trial.sh --execute --raw-policy --speed 0.2 --seconds 3
```

例如 **0.5 m/s、持续 10 秒**：

```bash
bash run_trial.sh --execute --raw-policy --speed 0.5 --seconds 10
```

当前程序自动开始，不需要再按遥控器 Start 或 A。执行顺序是：

1. 加载模型、预热推理，读取机器人低层状态。
2. 验证官方控制权已释放。
3. 用约 2 秒过渡到配置中的默认腿部、腰部和手臂姿态。
4. 输出 `DEFAULT_POSE_SENT; operator confirms posture; commanding forward ...`，立即进入模型闭环，不等待自动姿态稳定判定。
5. 执行指定速度与时长。
6. 结束或中止后，连续约 1 秒发送阻尼命令，保存结果并退出。

`--seconds` 只计算第 5 步的前进阶段。程序启动、模型预热、默认姿态过渡及最后阻尼时间均不计入，实际总耗时会更长。

## 3. 速度、时间和模式参数

| 参数 | 作用 | 默认值 |
| --- | --- | --- |
| `--execute` | 实际发送电机命令；不加时为只读预检 | 不执行 |
| `--raw-policy` | 直接使用模型目标，关闭额外的关节/力矩/速度中止条件 | 默认不开启 |
| `--speed` | 前进速度指令，单位 m/s | `0.2` |
| `--seconds` | 前进阶段持续时间，单位秒，必须为有限正数 | `3` |
| `--config` | 指定 YAML 格式的部署配置文件 | `/workspace/deploy_model/walk.yaml` |
| `--model` | 指定 TorchScript 权重，覆盖配置中的 `policy_path` | 使用配置中的 `policy_path` |

速度和时长可以直接修改，例如：

```bash
# 0.3 m/s，5 秒
bash run_trial.sh --execute --raw-policy --speed 0.3 --seconds 5

# 0.5 m/s，10 秒
bash run_trial.sh --execute --raw-policy --speed 0.5 --seconds 10

# 0.8 m/s，10 秒
bash run_trial.sh --execute --raw-policy --speed 0.8 --seconds 10
```

这些是不同参数的调用示例，按需要选择一条执行。模型训练的前进速度范围是 **0.2–0.8 m/s**，速度指令不代表已测得的实际行进速度。代码接受任意有限速度值；负值对应倒退指令，但这次训练和验证没有覆盖倒退。零速度指令也可能让策略原地踏步，并不等于电机关闭。

当前定时程序的横向速度和偏航速度固定为 0，遥控器摇杆不用于改变本次前进速度。它通过 `--speed` 输入固定的前进指令。

不加 `--raw-policy` 时，额外的关节目标限位、跳变、误差、力矩和腿部速度检查仍可能提前中止；自动初始姿态稳定等待和倾角中止检测则已从当前脚本中移除。

### 3.1 模型与配置文件

部署权重与配置统一放在 `/workspace/deploy_model`：

```text
/workspace/deploy_model/
├── walk.pt    # TorchScript 行走策略
└── walk.yaml  # YAML 部署配置
```

`walk.yaml` 是 YAML 格式的部署配置。默认推理入口读取这个配置，配置中的模型路径是：

```yaml
policy_path: "/workspace/deploy_model/walk.pt"
```

因此，日常运行无需额外指定模型或配置：

```bash
cd /workspace/g1_walk
bash run_trial.sh --execute --raw-policy --speed 0.5 --seconds 10
```

更换同接口模型时，可以更新 `/workspace/deploy_model/walk.pt`；修改关节默认角、PD 参数或其他部署设置时，编辑 `/workspace/deploy_model/walk.yaml`。修改后的文件在下一次启动时加载，不会自动影响已经运行的进程。

也可以通过参数选用其他文件，例如：

```bash
bash run_trial.sh --execute --raw-policy --speed 0.5 --seconds 10 \
  --config /workspace/deploy_model/walk.yaml \
  --model /workspace/deploy_model/walk.pt
```

`--model` 优先于配置中的 `policy_path`。更换的权重需是可由 `torch.jit.load` 加载的 TorchScript，兼容当前 47 维观测、12 维动作和 LSTM 状态重置接口；不能将原始训练 checkpoint 仅改名为 `walk.pt` 来部署。

## 4. 停止、结束和手臂行为

运行时可按终端 **Ctrl+C**，或遥控器 **Select / B** 停止。进入实际执行阶段后，程序会在退出流程中发送约 1 秒阻尼。

模型只输出双腿 12 个动作。腰和双臂共 17 个电机由配置保持固定目标，当前上肢目标全部为零。默认姿态过渡时可能看到手臂抬起；结束或报错时阻尼取消位置保持，手臂会放下。抬手本身不说明已经进入行走策略。

结束后官方控制器仍保持释放状态，程序不自动让机器人恢复官方站立。强制杀死进程、断电等情况不能依赖 Python 的正常退出流程。

## 5. 日志、结果与主要文件

运行脚本、源码及试验日志位于 `/workspace/g1_walk`；实际使用的模型与配置位于 `/workspace/deploy_model`；本文档位于 `/workspace/doc.md`。

| 文件或目录 | 用途 |
| --- | --- |
| `run_trial.sh` | 设置环境并启动只读预检或实际试验 |
| `run_fixed_trial.py` | 姿态过渡、固定速度策略闭环、时长控制和退出 |
| `release_control.sh` | 设置环境并运行控制权释放脚本 |
| `release_controller.py` | 释放并验证官方控制器 |
| `dds_runtime.py` | 为每个进程配置独立 DDS 日志路径 |
| `dds_logs/` | SDK/CycloneDDS 配置日志 |
| `check_dds_readonly.py` | 只读 DDS、状态和控制器检查 |
| `monitor_readonly.py` | 20 秒只读诊断，记录电机状态、温度、力矩及外部低层命令 |
| `check_trial_offline.py` | 使用假发布器验证目标处理，不驱动机器人 |
| `preflight.json` | 最近一次预检快照，会被覆盖 |
| `controller_release.json` | 最近一次控制权释放结果 |
| `trial_result.json` | 最近一次实际试验结果，会被覆盖 |
| `trial_result_*.json` | 按时间保存的实际试验结果 |
| `trial_forward3s_raw.log` | 助手执行的历史 3 秒原始策略试验文本日志 |
| `/workspace/deploy_model/walk.pt` | 当前实际加载的 TorchScript 行走策略 |
| `/workspace/deploy_model/walk.yaml` | 当前实际加载的 YAML 部署配置 |
| `policy_lstm_1.pt` | 原始导出存档；当前推理入口默认加载上述 `walk.pt` |
| `manifest.json` | 来源、仓库版本和原始文件哈希 |
| `verification.json` | 导出一致性与 MuJoCo 仿真结果 |
| `unitree_rl_gym/` | 训练仓库源码快照、部署配置及资源 |

查看最近一次结果的摘要，不打印大量逐帧记录：

```bash
/opt/miniconda3/envs/pms-py312/bin/python - <<'PY'
import json
from pathlib import Path
p = Path('/workspace/g1_walk/trial_result.json')
r = json.loads(p.read_text())
print(json.dumps({k: v for k, v in r.items() if k != 'records'}, indent=2, ensure_ascii=False))
PY
```

重点字段：

- `completed`：指定的策略循环是否正常完成。
- `error`：提前中止原因；正常完成时为 `null`。
- `forward_commands`：实际发送的策略命令次数；约为时长乘 50。
- `forward_span_s`：第一条至最后一条前进命令的间隔，通常比指定时长少约一个控制周期。
- `max_tilt`：记录的最大机身倾角，单位 rad；当前不据此中止。
- `max_target_error`：记录的最大关节目标与实测误差，单位 rad；原始策略模式不据此中止。
- `actual_displacement`：当前为 `not measured`，没有定位系统测量实际位移。

结果中的逐帧 `time` 是单调时钟，适合计算时间间隔，不是北京时间。带时间戳的结果文件名采用进程的本地时区；本手册在进入部署目录时设置 `TZ=Asia/Shanghai`，统一为北京时间。较早的文件名有北京时间和 UTC 两种情况，查看历史顺序时可使用 `ls -lt`。

## 6. 常见现象与故障处理
### 6.1 电机出现“卡卡卡”声音，但没有正常动作

声音本身不能证明机械关节锁住。此前一次试验中，官方 `ai` 控制器仍在 FSM4 站立状态，同时持续发送低层命令；自定义控制器也在发送目标，出现控制源冲突。释放官方控制权后，现场确认异响消失。

先停止当前试验，按第 5.3 节只读检查控制器状态；处理调试模式和控制权后再运行。电机 `motorstate=0` 仅表示该字段未报非零状态，不能单独排除机械受力或其他硬件问题。

### 6.2 只抬手，之后放下，没有进入行走

先查看 `trial_result.json` 的 `error` 和 `forward_commands`。历史两次输出为：

```text
Default posture did not settle within 3 seconds
forward_commands: 0
```

这表示卡在默认姿态准备阶段，前进策略尚未发送。当前版本已移除这项稳定检测，约 2 秒姿态过渡后直接进入策略，由现场安全员确认姿态。

当前日志中的阶段提示应为：

```text
DEFAULT_POSE_SENT; operator confirms posture; commanding forward ...
```

如果仍出现旧的 `Default posture did not settle`，运行的可能是旧脚本或另一个部署目录。抬手后进入阻尼、手臂放下也可能是程序正常结束或其他错误中止，不能仅凭手臂判断是否前进。

### 6.3 `Official controller must be released before actuation`

官方控制器仍占用。重新完成现场调试模式流程，执行 `bash release_control.sh` 并查看验证结果。如果外部服务重新启动控制器，需要先处理该服务。不要同时开两个行走程序。

### 6.4 `Stale lowstate`、`No fresh lowstate` 或 `Watchdog`

检查机器人是否开机、`eth0` 是否正常，以及 DDS/SDK 环境是否初始化成功。直接执行：

```bash
ip -brief addr
```

当前程序要求状态更新间隔不超过约 0.1 秒，实际控制期间命令间隔不超过约 0.12 秒。网络中断、线程阻塞、CPU 负载或 DDS 故障都可能中止试验。先用只读检查定位原因。

## 7. 模型来源与输入输出约定



任务为 `g1_walk`，使用 `g1_12dof.urdf`，训练时腰和双臂固定。`model_3000.pt` 是训练 checkpoint，不能直接给 `torch.jit.load` 使用。本次已导出为带内部状态的 LSTM TorchScript，实际部署文件为 `/workspace/deploy_model/walk.pt`；导出输出与原 Actor 的单步最大绝对差为 0，状态重置也已验证。

策略结构为 LSTM 输入 47、隐藏状态 64、1 层，Actor 为 `64 → 32 → 12`，ELU 激活。它不使用此前 15 动作 ONNX 策略的六帧拼接接口。

| 观测切片 | 内容 |
| --- | --- |
| `0:3` | 骨盆角速度 × 0.25 |
| `3:6` | 骨盆坐标系投影重力；四元数顺序 wxyz |
| `6:9` | `[vx, vy, yaw_rate] × [2, 2, 0.25]` |
| `9:21` | 双腿关节角减默认角度 |
| `21:33` | 双腿关节速度 × 0.05 |
| `33:45` | 上一步 12 维策略动作 |
| `45:47` | 0.8 秒周期相位的 sin/cos |

每条腿的顺序为：`hip_pitch、hip_roll、hip_yaw、knee、ankle_pitch、ankle_roll`；左腿对应电机 0–5，右腿 6–11。

目标计算公式：

```text
关节目标角度 = 默认角度 + 0.25 × 模型输出
```

每条腿的默认角度为 `[-0.1, 0, 0, 0.3, -0.2, 0]` rad；位置增益为 `[100, 100, 100, 150, 40, 40]`，阻尼增益为 `[2, 2, 2, 4, 2, 2]`。实机使用 hg 消息、骨盆 IMU 和 PR 模式。

仓库及来源信息保存在 `manifest.json`；模型在 12 自由度 MuJoCo 场景的六组 12 秒试验均未触发脚本的跌倒判定。真机曾完成 0.2 m/s 指令的 3 秒策略运行，后续日志也记录了完成的 10 秒策略循环。指令循环完成、关节发生运动与实际稳定向前行走是不同指标；当前脚本不测量位移，现场观察用于确认实际迈步和前进效果。
