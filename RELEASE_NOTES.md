# Mantis Sim2Real Release Notes

## V1.0.3 (2026-01-04)

### 概述

V1.0.3 版本新增 Gazebo 仿真支持，用于底盘运动仿真预览。采用运动学仿真模式，关闭动力学计算以提高稳定性。

### ✨ 新增功能

- **Gazebo 仿真支持**: 新增 Gazebo 仿真模式，支持底盘移动预览
- **统一 Launch 文件**: 合并 RViz/Gazebo 仿真为统一的 `sdk_sim.launch.py`
  - `use_gazebo:=false`（默认）: RViz 快速预览
  - `use_gazebo:=true`: Gazebo 物理仿真，支持底盘移动
- **Gazebo Bridge 节点**: 新增 `gazebo_bridge_node`，转发 SDK 命令到 Gazebo

### 🦾 URDF 更新

- 新增 `mantis_gazebo.urdf`，包含 Gazebo 插件配置
- 添加 `libgazebo_ros_planar_move` 插件，支持底盘全向移动
- 关闭动力学仿真（`gravity=false`, `kinematic=true`），仅支持运动学仿真
- 所有 link 添加 Gazebo 惯性参数

### 📖 使用方式

```bash
# RViz 模式（默认，快速预览关节）
ros2 launch bw_sim2real sdk_sim.launch.py

# Gazebo 模式（底盘可移动）
ros2 launch bw_sim2real sdk_sim.launch.py use_gazebo:=true
```

---


**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/bw_motion_ws_public/issues)

**许可证**: MIT License

© 2025-2026 BlueWorm-EAI-Tech. All rights reserved.
