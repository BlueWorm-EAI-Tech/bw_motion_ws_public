# Mantis Sim2Real V1.0.2 Release Notes

## 概述

Mantis Sim2Real V1.0.2 版本主要增加了GUI模式下的夹爪控制功能，并修复了关节限位和IK模式下的穿模问题。

## 更新内容

### ✨ 新增功能

- **GUI 夹爪控制**: 在控制面板中增加了左右手夹爪的控制滑条，支持高精度调节。

### 🐛 问题修复

- **关节限位修正**: 修复了 `mantis.urdf` 中关节限位数据错误的问题，使其与控制代码保持一致。
- **IK 穿模修复**: 修正了碰撞模型路径引用，解决了 IK 模式下的大面积穿模问题。

### 🦾 URDF 更新

- 更新了 mantis.urdf 机器人描述文件
- 新增双手夹爪的 STL 模型文件
- 修复了 mesh 路径引用问题 (`package://mantis/` → `package://mantis_description/`)

---

**问题反馈**: [GitHub Issues](https://github.com/BlueWorm-EAI-Tech/mantis-sim2real/issues)

**许可证**: MIT License

© 2025 BlueWorm-EAI-Tech. All rights reserved.
