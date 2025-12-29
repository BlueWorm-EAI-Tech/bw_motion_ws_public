# bw_motion_ws

面向 **Mantis** 机器人"仿真 → 实机（sim2real）"的 ROS 2 工作空间。

本仓库核心目标是：把多种控制输入（GUI 滑条 / 交互式 IK / 文件关键帧回放）统一路由到同一个控制输出 `/ctrl/joint_target`，并让 TF（`robot_state_publisher`）稳定不断链。

> 入口 launch 主要是：
> - `bw_sim2real/launch/phase1_sim2real.launch.py`（系统组合入口）
> - `bw_sim2real_view/launch/view.launch.py`（控制与UI层入口，轻量）


## 快速开始

### 1）编译工作空间

```bash
cd /home/lanchong/bw_motion_ws
colcon build
source install/setup.zsh
```

### 2）启动系统

```bash
ros2 launch bw_sim2real phase1_sim2real.launch.py
```

启动后会打开：
- **RViz2**：显示机器人模型和TF
- **控制面板**：Qt界面，用于控制机器人和录制动作


## 控制面板使用说明

启动后会弹出一个 Qt 控制面板窗口：

```
┌─────────────────────────────────────────┐
│  Mode: gui                              │  ← 当前输入模式
├─────────────────────────────────────────┤
│  Input Mode                             │
│  [GUI]  [IK]  [FILE]                    │  ← 模式切换按钮
├─────────────────────────────────────────┤
│  GUI Sliders                            │
│  Left Arm / Right Arm                   │  ← 关节滑条（GUI模式下使用）
├─────────────────────────────────────────┤
│  Keyframe Playback                      │
│  (no file)        [Pick JSON...]        │  ← 选择JSON文件
│  Speed [1.0]      □ Loop                │  ← 播放速度和循环选项
│  [Record] [Clear] [Load] [Save]         │  ← 录制/管理按钮
│  [Play] [Stop]                          │  ← 播放控制按钮
│  Playback: (idle)                       │  ← 状态显示
└─────────────────────────────────────────┘
```

### 三种输入模式

| 模式 | 按钮 | 输入来源 | 用途 |
|------|------|----------|------|
| **GUI** | `GUI` | 控制面板滑条 | 手动拖滑条控制各关节 |
| **IK** | `IK` | RViz中的交互球 | 拖动末端球控制手臂位置 |
| **FILE** | `FILE` | 播放录制的JSON文件 | 回放预录制的运动序列 |


## 关键帧录制与播放

### 录制运动

1. **切换到 GUI 模式**：点击 `GUI` 按钮
2. **摆好姿势**：用滑条调整机器人关节到目标位置
3. **录制关键帧**：点击 `Record Frame` 按钮（每点一次记录一帧）
4. **重复步骤 2-3**：录制多个关键帧形成动作序列
5. **保存文件**：点击 `Save` 按钮，选择保存路径（JSON格式）
6. **清空缓存**（可选）：点击 `Clear` 按钮清空已录制的帧

> 💡 **提示**：录制是增量的，可以多次录制后一起保存。如需重新开始，点击 `Clear` 清空。

### 播放运动

1. **选择文件**：点击 `Pick JSON...` 选择录制的JSON文件
2. **加载文件**：点击 `Load` 加载文件
3. **切换到 FILE 模式**：点击 `FILE` 按钮
4. **开始播放**：点击 `Play` 开始播放
5. **停止播放**：点击 `Stop` 停止

**播放选项**：
- **Speed**：调整播放速度倍率（0.05 ~ 5.0）
- **Loop**：勾选后循环播放

### 录制文件格式

录制的JSON文件保存在 `motion_record/` 目录下，格式如下：

```json
{
  "meta": {
    "format": "bw_sim2real_keyframes_v1",
    "joint_names": ["L_Shoulder_Pitch_Joint", ...],
    "record_source": "control",
    "default_frame_speed_rad_s": 1.0,
    "created_unix": 1766991119.47
  },
  "keyframes": [
    [0.0, 0.0, 0.0, ...],  // 第1帧：14个关节角度
    [0.5, 0.1, 0.0, ...],  // 第2帧
    ...
  ],
  "speeds": [1.0, 1.0, ...]  // 每帧的播放速度（可选）
}
```


## 架构概览

### 包职责（重构后推荐使用方式）

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

# 开启桥接
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
│   └── mantis_moveit_config/ # MoveIt配置（预留）
├── motion_record/            # 录制的动作JSON文件
├── install/                  # 编译输出
├── build/                    # 构建缓存
└── log/                      # 日志文件
```


## IK 交互球使用（可选）

启动时开启 IK 功能：

```bash
ros2 launch bw_sim2real phase1_sim2real.launch.py enable_ik:=true
```

在 RViz 中会出现两个交互球（左右手），可以：
- **拖动球体**：移动手臂末端位置
- **旋转箭头**：调整手腕姿态

> ⚠️ **注意**：IK 节点较重，如不需要建议关闭。


## 备注：Legacy 代码

历史上为快速调试写过一些独立脚本/节点。当前推荐只使用上面列出的核心节点与 launch。
如果你发现有 legacy 脚本仍需要保留，我们建议将其移动到 `scripts/legacy/` 并避免作为 console_script 安装，防止误用。
