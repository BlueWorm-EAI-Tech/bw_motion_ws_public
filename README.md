# bw_motion_ws

面向 **mantis** 机器人“仿真 → 实机（sim2real）”的 ROS 2 工作空间。

本仓库核心目标是：把多种控制输入（GUI 滑条 / 交互式 IK / 文件关键帧回放）统一路由到同一个控制输出 `/ctrl/joint_target`，并让 TF（`robot_state_publisher`）稳定不断链。

> 入口 launch 主要是：
> - `bw_sim2real/launch/phase1_sim2real.launch.py`（系统组合入口）
> - `bw_sim2real_view/launch/view.launch.py`（控制与UI层入口，轻量）


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
cd /home/menlin/bw_motion_ws
colcon build
source install/setup.zsh
```


## 备注：Legacy 代码

历史上为快速调试写过一些独立脚本/节点。当前推荐只使用上面列出的核心节点与 launch。
如果你发现有 legacy 脚本仍需要保留，我们建议将其移动到 `scripts/legacy/` 并避免作为 console_script 安装，防止误用。
# bw_motion_ws
