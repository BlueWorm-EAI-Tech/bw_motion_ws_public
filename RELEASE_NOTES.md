# Mantis Sim2Real V1.0.0 Release Notes

🎉 **首次正式发布** | 📅 2025-12-30

---

## 概述

Mantis Sim2Real V1.0 是 Mantis 机器人仿真到实机控制的首个正式版本。本版本提供了完整的远程操控解决方案，包括多种控制输入方式、动作录制回放、IK 交互控制。

## 主要特性

### 🎮 三种控制模式

| 模式     | 输入来源     | 用途                   |
| -------- | ------------ | ---------------------- |
| **GUI**  | 控制面板滑条 | 手动拖滑条控制各关节   |
| **IK**   | RViz 交互球  | 拖动末端球控制手臂位置 |
| **FILE** | JSON 文件    | 回放预录制的运动序列   |

### 🔄 输入路由系统 (input_router)

统一的输入源管理，将多种控制输入路由到同一控制输出：

- 支持 GUI / IK / FILE 三种模式无缝切换
- 自动关节补全，保证 TF 链完整
- 输出到 `/ctrl/joint_target` 统一控制话题
- 模式切换通过 `/sys/input_mode` 话题

### 📹 动作录制与回放 (mantis_playback_node)

- **关键帧录制**: 记录当前姿态为关键帧
- **JSON 保存/加载**: 标准化的动作文件格式
- **可调速播放**: 0.05x ~ 5.0x 速度倍率
- **循环播放**: 支持动作循环执行
- **服务接口**: `/playback/record_frame`、`/playback/load`、`/playback/save`、`/playback/start`、`/playback/stop`

### 🎯 交互式 IK 控制 (mantis_casadi_node)

- 基于 CasADi 的逆运动学求解
- RViz 交互球拖动控制
- 实时末端位姿规划

### 🖥️ 统一控制面板 (mantis_control_panel)

Qt 图形界面，集成所有控制功能：

- 模式切换按钮
- 左右臂关节滑条（14 个关节）
- 录制/播放控制面板
- 文件管理功能

### 🌉 仿真到实机桥接 (sim_to_real_bridge)

- 仿真环境与实机的数据同步
- 支持 Zenoh 协议通信

### 🔌 SDK 仿真桥接 (sdk_bridge_node)

为 [Mantis SDK](../mantis/) 提供仿真环境支持：

- 接收 SDK 发送的 JSON 格式关节数据
- 转发到 ROS2 `/joint_states` 话题
- 解决 Zenoh-ROS2 桥接的 QoS 不匹配问题

> **注意**: SDK 为独立项目，详见 [Mantis SDK 文档](../mantis/README.md)

### 📦 启动文件

- **phase1_sim2real.launch.py**: 完整 Sim2Real 启动（router/panel/playback/IK/bridge）
- **sdk_sim.launch.py**: SDK 仿真启动（robot_state_publisher + RViz + sdk_bridge_node）

### 🤖 机器人描述

- **mantis_description**: URDF 机器人模型
- 完整的关节定义和限位

## 包结构

| 包名                 | 说明                                             |
| -------------------- | ------------------------------------------------ |
| `bw_sim2real`        | 核心运行节点：input_router、playback、IK、bridge |
| `bw_sim2real_view`   | 统一 UI 面板 + 轻量启动文件                      |
| `bw_interface`       | 消息与服务接口定义 (msg/srv)                     |
| `bw_motion_utils`    | 通用工具函数                                     |
| `mantis_description` | URDF 机器人描述文件                              |

## 安装指南

### 构建 ROS2 工作空间

```bash
cd bw_motion_ws
colcon build
source install/setup.bash
```

### 安装 Zenoh 桥接

```bash
# 下载 zenoh-bridge-ros2dds
# https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases
```

## 快速开始

### 完整 Sim2Real 启动

```bash
# 实机端
cd ~/bw_teleoperate_ws
./remote_bridge.sh
zenoh-bridge-ros2dds -d 99

# 控制端
source install/setup.bash
ros2 launch bw_sim2real phase1_sim2real.launch.py enable_ik:=false enable_bridge:=true enable_rviz:=true
```

### SDK 仿真环境

```bash
# 启动仿真环境（为 SDK 提供 RViz 预览）
ros2 launch bw_sim2real sdk_sim.launch.py

# 启动 Zenoh 桥接
zenoh-bridge-ros2dds -d 99
```

## 关键话题

| 话题                       | 类型       | 说明         |
| -------------------------- | ---------- | ------------ |
| `/input/gui/joint_states`  | JointState | GUI 滑条输入 |
| `/input/ik/joint_states`   | JointState | IK 交互输入  |
| `/input/file/joint_states` | JointState | 文件回放输入 |
| `/sys/input_mode`          | String     | 模式切换命令 |
| `/ctrl/joint_target`       | JointState | 统一控制输出 |
| `/joint_states`            | JointState | 关节状态反馈 |

## 服务接口

| 服务                     | 说明             |
| ------------------------ | ---------------- |
| `/playback/record_frame` | 录制当前帧       |
| `/playback/clear`        | 清空录制缓存     |
| `/playback/load`         | 加载 JSON 文件   |
| `/playback/save`         | 保存为 JSON 文件 |
| `/playback/start`        | 开始播放         |
| `/playback/stop`         | 停止播放         |

## 系统要求

- **ROS2**: Humble
- **操作系统**: Ubuntu 22.04

## 已知限制

- IK 节点较重，不需要时建议关闭 (`enable_ik:=false`)
- SDK 仿真模式需要先启动 `sdk_sim.launch.py` 和 `zenoh-bridge-ros2dds`

---

## Changelog

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

### [1.0.0] - 2025-12-30

#### ✨ 新增功能

- **输入路由系统 (input_router)**
  - 统一管理 GUI / IK / FILE 三种输入源
  - 自动关节补全，保证 TF 链完整
  - 输出到 `/ctrl/joint_target` 统一控制话题
  - 模式切换通过 `/sys/input_mode` 话题

- **动作录制与回放 (mantis_playback_node)**
  - 关键帧录制功能
  - JSON 格式保存/加载
  - 可调速播放（0.05x ~ 5.0x）
  - 循环播放支持
  - 服务接口：`/playback/record_frame`、`/playback/load`、`/playback/save`、`/playback/start`、`/playback/stop`

- **交互式 IK 控制 (mantis_casadi_node)**
  - 基于 CasADi 的逆运动学求解
  - RViz 交互球拖动控制
  - 实时末端位姿规划

- **统一控制面板 (mantis_control_panel)**
  - Qt 图形界面
  - 模式切换按钮
  - 左右臂关节滑条（14 个关节）
  - 录制/播放控制面板
  - 文件管理功能

- **仿真到实机桥接 (sim_to_real_bridge)**
  - 仿真环境与实机的数据同步
  - 支持 Zenoh 协议通信

- **SDK 仿真桥接 (sdk_bridge_node)**
  - 接收 Zenoh JSON 数据，转发到 ROS2 `/joint_states`
  - 解决 Zenoh-ROS2 桥接的 QoS 不匹配问题
  - 为 Mantis SDK 提供仿真环境支持

- **启动文件**
  - `phase1_sim2real.launch.py`: 完整 Sim2Real 启动
  - `sdk_sim.launch.py`: SDK 仿真启动

- **机器人描述**
  - `mantis_description`: URDF 机器人模型
  - 完整的关节定义和限位

---

**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/mantis-sim2real/issues)

**许可证**: MIT License

© 2025 BlueWorm-EAI-Tech. All rights reserved.
