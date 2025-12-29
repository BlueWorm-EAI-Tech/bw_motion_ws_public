# bw_motion_ws

面向 **Mantis** 机器人的独立远程操控项目，基于 ROS 2 工作空间。

本仓库核心目标是：把多种控制输入（GUI 滑条 / 交互式 IK / 文件关键帧回放）统一路由到同一个控制输出 `/ctrl/joint_target`，并让 TF（`robot_state_publisher`）稳定不断链。

## 快速开始

### 1. 编译工作空间

```bash
cd /path/to/bw_motion_ws
colcon build
source install/setup.zsh
```

### 2. 目标主机准备（实机端）

在目标主机（机器人端）上执行以下操作：

```bash
# 1) 启动远程桥接脚本
cd ~/bw_teleoperate_ws
./remote_bridge.sh

# 2) 可选：启动 Zenoh 桥接以稳定通信（推荐）
~/zenoh_ros2/zenoh-bridge-ros2dds -d 0
```

> ⚠️ **注意**：
>
> - 确保 **禁用** `start_real.sh`（VR 遥操启动脚本），两种模式互相冲突
> - Zenoh 相关配置请参考装机手册

### 3. 本地启动（控制端）

```bash
source install/setup.zsh
ros2 launch bw_sim2real phase1_sim2real.launch.py enable_ik:=false enable_bridge:=true enable_rviz:=true
```

启动后会打开：

- **RViz2**：显示机器人模型和TF
- **控制面板**：Qt界面，用于控制机器人和录制动作
- 根据需求是否开启IK模式

---

## 三种控制方式

系统支持三种输入模式，通过控制面板顶部按钮切换：


| 模式     | 按钮   | 输入来源           | 用途                   |
| -------- | ------ | ------------------ | ---------------------- |
| **GUI**  | `GUI`  | 控制面板滑条       | 手动拖滑条控制各关节   |
| **IK**   | `IK`   | RViz中的交互球     | 拖动末端球控制手臂位置 |
| **FILE** | `FILE` | 播放录制的JSON文件 | 回放预录制的运动序列   |

### 方式一：GUI 滑条控制

最直接的控制方式，适合逐关节调试。

1. 点击 `GUI` 按钮切换到 GUI 模式
2. 使用左/右手臂的滑条调整各关节角度
3. 机器人实时响应滑条变化

### 方式二：IK 交互球控制

拖动末端位置，自动求解关节角度，适合规划末端轨迹。

1. 启动时开启 IK 功能：
   ```bash
   ros2 launch bw_sim2real phase1_sim2real.launch.py enable_ik:=true
   ```
2. 点击 `IK` 按钮切换到 IK 模式
3. 在 RViz 中拖动交互球：
   - **拖动球体**：移动手臂末端位置
   - **旋转箭头**：调整手腕姿态

> ⚠️ **注意**：IK 节点较重，如不需要建议关闭。

### 方式三：文件播放控制

回放预录制的动作序列，适合重复执行相同动作。

#### 录制动作

1. 点击 `GUI` 按钮，用滑条摆好姿势
2. 点击 `Record Frame` 记录当前帧
3. 重复调整姿势并录制，形成动作序列
4. 点击 `Save` 保存为 JSON 文件
5. 如需重新录制，点击 `Clear` 清空缓存

> 💡 录制是增量的，可以多次录制后一起保存。

#### 播放动作

1. 点击 `Pick JSON...` 选择文件
2. 点击 `Load` 加载
3. 点击 `FILE` 按钮切换模式
4. 点击 `Play` 开始播放，`Stop` 停止

**播放选项**：

- **Speed**：播放速度倍率（0.05 ~ 5.0）
- **Loop**：勾选后循环播放

#### 录制文件格式

文件保存在 `motion_record/` 目录，格式：

```json
{
  "meta": {
    "format": "bw_sim2real_keyframes_v1",
    "joint_names": ["L_Shoulder_Pitch_Joint", ...],
    "created_unix": 1766991119.47
  },
  "keyframes": [
    [0.0, 0.0, 0.0, ...],
    [0.5, 0.1, 0.0, ...]
  ],
  "speeds": [1.0, 1.0, ...]
}
```

---

## 控制面板界面

```
┌─────────────────────────────────────────┐
│  Mode: gui                              │  ← 当前模式
├─────────────────────────────────────────┤
│  [GUI]  [IK]  [FILE]                    │  ← 模式切换
├─────────────────────────────────────────┤
│  Left Arm / Right Arm Sliders           │  ← 关节滑条
├─────────────────────────────────────────┤
│  (no file)        [Pick JSON...]        │  ← 文件选择
│  Speed [1.0]      □ Loop                │  ← 播放选项
│  [Record] [Clear] [Load] [Save]         │  ← 录制管理
│  [Play] [Stop]                          │  ← 播放控制
└─────────────────────────────────────────┘
```

---

## 架构概览

### 包职责

- `bw_sim2real`

  - **运行节点（runtime nodes）**：
    - `input_router`：输入源选择 + 关节补全 + 输出 `/ctrl/joint_target`
    - `mantis_playback_node`：关键帧录制/加载/保存/播放（无 UI，靠 service 被面板驱动）
    - `mantis_casadi_node`：交互式 IK 输入（可选启动，较重）
    - `sim_to_real_bridge`：仿真↔实机桥接（可选启动，较重）
- `bw_sim2real_view`

  - **统一 UI 面板 + 轻量 view.launch**：
    - `mantis_control_panel`：单面板，包含模式切换、GUI 滑条、回放控制
    - `view.launch.py`：启动 router/panel/playback（默认不启动 rviz/bridge/IK）
- `bw_interface`

  - **消息与服务接口定义**（msg/srv），包括回放控制服务：
    - `/playback/load` `/playback/save` `/playback/start` `/playback/stop`
    - `/playback/record_frame` `/playback/clear`

## 关键话题与服务

### Topics

- 输入（由控制源发布）：

  - `/input/gui/joint_states`（来自统一面板的滑条）
  - `/input/ik/joint_states`（来自 IK 节点）
  - `/input/file/joint_states`（来自回放节点播放）
- 模式切换：

  - `/sys/input_mode`（String：`gui` / `ik` / `file`）
  - `/sys/input_mode_state`（String：路由器发布的当前模式）
- 路由输出（最终控制指令）：

  - `/ctrl/joint_target`（`sensor_msgs/JointState`）
- TF：

  - `/tf` `/tf_static`

### Services（回放控制）

均由 `mantis_playback_node` 提供：

- `/playback/record_frame`
- `/playback/clear`
- `/playback/load`
- `/playback/save`
- `/playback/start`
- `/playback/stop`

## 启动方式

### 1）推荐：轻量启动（调试/稳定）

仅启动“控制与UI层”（router + panel + playback），不启动 RViz/IK/bridge：

```bash
source install/setup.zsh
ros2 launch bw_sim2real_view view.launch.py
```

`view.launch.py` 支持开关（便于排障/减负）：

```bash
# 只起 router + panel，不起 playback
ros2 launch bw_sim2real_view view.launch.py enable_playback:=false

# 只起 router（用于检查 /ctrl/joint_target 是否发布）
ros2 launch bw_sim2real_view view.launch.py enable_panel:=false enable_playback:=false

# 默认进入 file 模式（便于专门测回放）
ros2 launch bw_sim2real_view view.launch.py default_mode:=file
```

### 2）系统组合入口：phase1（可选启动重节点）

```bash
source install/setup.zsh
ros2 launch bw_sim2real phase1_sim2real.launch.py
```

`phase1_sim2real.launch.py` 已参数化（默认不启 IK/bridge，降低卡死风险）：

```bash
# 只起 view + robot_state_publisher + rviz（默认 enable_ik/enable_bridge=false）
ros2 launch bw_sim2real phase1_sim2real.launch.py

# 开启 IK
ros2 launch bw_sim2real phase1_sim2real.launch.py enable_ik:=true

# 开启sim2real桥接 (开启桥接之后可以直接控制Mantis Pro机器人，需注意安全)
ros2 launch bw_sim2real phase1_sim2real.launch.py enable_bridge:=true

# 完全不启 view（只做 TF/rviz 相关实验）
ros2 launch bw_sim2real phase1_sim2real.launch.py enable_view:=false
```

## 常见问题与排查

### 1）TF 断裂 / RViz 里树断得很严重

核心判断：`robot_state_publisher` 需要持续收到 JointState。

快速检查：

```bash
ros2 node list
ros2 topic info /ctrl/joint_target --verbose
ros2 topic echo /ctrl/joint_target --once
```

如果 `/ctrl/joint_target` 没有任何消息：

- 看三路输入是否有人在发：

```bash
ros2 topic info /input/gui/joint_states --verbose
ros2 topic info /input/ik/joint_states --verbose
ros2 topic info /input/file/joint_states --verbose
```

- 典型原因：
  - 你处在 `gui` 模式，但面板没启动或滑条没发布（`/input/gui` publisher=0）
  - 你处在 `file` 模式，但没有按 play（回放节点不会主动刷 `/input/file`）
  - IK/bridge 没启（publisher=0 属正常）

> 说明：`input_router` 内部带有 idle 发布兜底（可通过参数 `idle_publish_rate_hz` 调整/关闭），用于在“没有任何输入”时尽量维持 TF 不完全断链。

### 2）启动就很卡 / 甚至卡死

- 优先用 `view.launch.py` 启动轻量链路
- 在 `phase1_sim2real.launch.py` 里按需打开 `enable_ik` / `enable_bridge`

### 3）桥接节点作用

- 不启动桥接节点为单纯虚拟仿真模式
- 启动桥接节点实现sim2real能力

## 开发与构建

```bash
cd /home/lanchong/bw_motion_ws
colcon build
source install/setup.zsh
```

### 单独编译某个包

```bash
colcon build --packages-select bw_sim2real
colcon build --packages-select bw_sim2real_view
colcon build --packages-select mantis_description
```

## 工作空间结构

```
bw_motion_ws/
├── src/
│   ├── bw_interface/         # 消息与服务接口定义
│   ├── bw_motion_utils/      # 通用工具库
│   ├── bw_sim2real/          # 核心运行节点
│   ├── bw_sim2real_view/     # UI面板与视图
│   ├── mantis_description/   # 机器人URDF模型
├── motion_record/            # 录制的动作JSON文件
├── install/                  # 编译输出
├── build/                    # 构建缓存
└── log/                      # 日志文件
```

## 备注：Legacy 代码

历史上为快速调试写过一些独立脚本/节点。当前推荐只使用上面列出的核心节点与 launch。
如果你发现有 legacy 脚本仍需要保留，我们建议将其移动到 `scripts/legacy/` 并避免作为 console_script 安装，防止误用。
