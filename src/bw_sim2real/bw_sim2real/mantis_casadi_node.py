#!/usr/bin/env python3
"""Interactive-marker IK input node.

Publishes `sensor_msgs/JointState` on `/input/ik/joint_states`.

The solver implementation is kept minimal in `bw_sim2real.ik_solver` so the repo
cleanup can delete other legacy/debug modules without breaking `enable_ik:=true`.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

import numpy as np
from scipy.spatial.transform import Rotation as R

from .ik_solver import MantisArmIK

class MantisCasadiNode(Node):
    def __init__(self):
        super().__init__('mantis_casadi_node')
        
        self.get_logger().info("Initializing CasADi IK Solver...")
        try:
            self.ik_solver = MantisArmIK()
            self.get_logger().info("Solver Ready.")
        except Exception as e:
            self.get_logger().error(f"Solver Init Failed: {e}")
            raise

        # 话题发布
        self.pub_ik = self.create_publisher(JointState, '/input/ik/joint_states', 10)

        # 交互球服务
        self.server = InteractiveMarkerServer(self, 'mantis_ik_controls')
        
        # --- 核心修复 1: 使用 FK 获取真实的初始位置 ---
        init_T_l, init_T_r = self.ik_solver.get_initial_fk()
        
        # 缓存目标位置
        self.target_T_l = init_T_l
        self.target_T_r = init_T_r

        # 创建交互球
        self.create_marker("left", self.target_T_l)
        self.create_marker("right", self.target_T_r)
        self.server.applyChanges()

        # 定时器
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("CasADi IK Node Started.")

    def create_marker(self, side, T):
        # 1. 创建 Marker 对象
        marker = InteractiveMarker()
        marker.header.frame_id = "base_footprint" 
        marker.name = f"{side}_hand_ctrl"
        marker.scale = 0.25 
        
        marker.pose.position.x = float(T[0,3])
        marker.pose.position.y = float(T[1,3])
        marker.pose.position.z = float(T[2,3])
        
        # ==========================================
        # 2. 添加中心实心球 (MOVE_3D)
        # ==========================================
        visual_marker = Marker()
        visual_marker.type = Marker.SPHERE
        # 设置球体相对于 Marker 整体比例的大小
        # 注意：这里的 scale 是绝对尺寸，不是相对于 marker.scale 的
        visual_marker.scale.x = 0.1 # 直径 10cm
        visual_marker.scale.y = 0.1
        visual_marker.scale.z = 0.1
        visual_marker.color.r = 0.0
        visual_marker.color.g = 1.0
        visual_marker.color.b = 1.0 # 青色
        visual_marker.color.a = 0.6 # 半透明，避免挡住箭头

        # 创建一个包含球体的 Control
        central_ctrl = InteractiveMarkerControl()
        central_ctrl.always_visible = True 
        central_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_3D # 允许 3D 自由拖拽
        central_ctrl.markers.append(visual_marker)
        marker.controls.append(central_ctrl)

        # ==========================================
        # 3. 添加正交轴移动控制 (MOVE_AXIS)
        # ==========================================
        for axis in [[1,0,0], [0,1,0], [0,0,1]]:
            ctrl = InteractiveMarkerControl()
            ctrl.orientation.w = 1.0
            ctrl.orientation.x = float(axis[0])
            ctrl.orientation.y = float(axis[1])
            ctrl.orientation.z = float(axis[2])
            ctrl.name = "move_" + str(axis)
            ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(ctrl)
            
        # ==========================================
        # 4. 添加正交轴旋转控制 (ROTATE_AXIS)
        # ==========================================
        for axis in [[1,0,0], [0,1,0], [0,0,1]]:
            ctrl = InteractiveMarkerControl()
            ctrl.orientation.w = 1.0
            ctrl.orientation.x = float(axis[0])
            ctrl.orientation.y = float(axis[1])
            ctrl.orientation.z = float(axis[2])
            ctrl.name = "rotate_" + str(axis)
            ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(ctrl)

        # ==========================================
        # 5. 提交 (只提交这一次！)
        # ==========================================
        self.server.insert(marker, feedback_callback=self.process_feedback)

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            p = feedback.pose.position
            o = feedback.pose.orientation
            
            T = np.eye(4)
            T[:3, 3] = [p.x, p.y, p.z]
            r = R.from_quat([o.x, o.y, o.z, o.w])
            T[:3, :3] = r.as_matrix()

            if "left" in feedback.marker_name:
                self.target_T_l = T
            else:
                self.target_T_r = T

    def timer_callback(self):
        # 求解 IK
        q_sol = self.ik_solver.solve_ik(self.target_T_l, self.target_T_r)
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        joint_names = self.ik_solver.get_joint_names()
        for i, name in enumerate(joint_names):
            if i < len(q_sol):
                msg.name.append(name)
                msg.position.append(float(q_sol[i]))
        self.pub_ik.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MantisCasadiNode()
    except Exception:
        # If solver init fails we already logged; exit cleanly.
        rclpy.shutdown()
        return
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()