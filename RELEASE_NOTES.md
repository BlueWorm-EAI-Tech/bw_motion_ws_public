# Mantis Sim2Real Release Notes

## V1.0.4 (2026-01-05)

### 概述

V1.0.4 版本主要对桥接节点和 Launch 文件进行了命名规范化，使其更符合统一命名约定。

### 🏷️ 节点命名规范化

- `sdk_bridge_node` → `sdk_rviz_bridge_node`（RViz 仿真桥接）
- `gazebo_bridge_node` → `sdk_gazebo_bridge_node`（Gazebo 仿真桥接）

### 📦 配置更新

- 更新 `setup.py` 中的节点入口点配置
- 更新 `sdk_sim.launch.py` 中的节点名称引用

---

**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/bw_motion_ws_public/issues)

**许可证**: MIT License

© 2025-2026 BlueWorm-EAI-Tech. All rights reserved.
