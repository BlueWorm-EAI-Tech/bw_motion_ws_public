# Mantis Sim2Real Release Notes

## V1.0.5 (2026-01-21)

### 🛠️ Bug 修复

- 修复 `mantis` URDF 中左右臂初始位姿不一致的问题：
  - 修改文件：`src/mantis_description/urdf/mantis.urdf`
  - 主要改动：

    - 将右臂 `R_Shoulder_Pitch_Joint` 的 `origin` Y 值调整为与左臂对称（修正位置偏移）。
    - 将右臂 `R_Shoulder_Yaw_Joint` 的 `origin` X 值调整为与左臂对称（修正位置偏移）。
    - 将右臂 `R_Elbow_Pitch_Joint` 的关节姿态在 URDF 中永久增加了 +0.16 rad 的偏移（用户在 RViz 中临时调整后确认能修复视觉对称性）。
    - 保留 `R_Shoulder_Roll_Joint` 的原始旋转值（应用户要求未更改）。

---

**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/bw_motion_ws_public/issues)

**许可证**: MIT License

© 2025-2026 BlueWorm-EAI-Tech. All rights reserved.
