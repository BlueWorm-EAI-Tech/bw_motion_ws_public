#!/usr/bin/env python3
"""
Gazebo 关节/底盘桥接节点

订阅 ROS2 话题，转发到 Gazebo：
- /joint_states → /gazebo_joint_commands (JointTrajectory)
- /Teleop/cmd_vel → /cmd_vel (Twist，Gazebo planar_move 插件)

数据流:
SDK → Zenoh → sdk_bridge_node → /joint_states → 本节点 → Gazebo
SDK → Zenoh → sdk_chassis_bridge_node → /Teleop/cmd_vel → 本节点 → Gazebo
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

try:
    import zenoh
except ImportError:
    zenoh = None


class GazeboBridgeNode(Node):
    """Gazebo 桥接节点"""
    
    # Gazebo 关节列表（必须与 URDF 一致）
    GAZEBO_JOINTS = [
        "Waist_Joint",
        # 左臂
        "L_Shoulder_Pitch_Joint",
        "L_Shoulder_Yaw_Joint",
        "L_Shoulder_Roll_Joint",
        "L_Elbow_Pitch_Joint",
        "L_Wrist_Roll_Joint",
        "L_Wrist_Pitch_Joint",
        "L_Wrist_Yaw_Joint",
        "L_Hand_R_Joint",
        "L_Hand_L_Joint",
        # 右臂
        "R_Shoulder_Pitch_Joint",
        "R_Shoulder_Yaw_Joint",
        "R_Shoulder_Roll_Joint",
        "R_Elbow_Pitch_Joint",
        "R_Wrist_Roll_Joint",
        "R_Wrist_Pitch_Joint",
        "R_Wrist_Yaw_Joint",
        "R_Hand_R_Joint",
        "R_Hand_L_Joint",
        # 头部
        "Neck_Joint",
        "Head_Joint",
        "L_Ear_Joint",
        "R_Ear_Joint",
    ]
    
    # SDK 关节名称 → Gazebo URDF 名称映射
    JOINT_NAME_MAP = {
        # 腰部
        "waist_joint": "Waist_Joint",
        "Waist_Joint": "Waist_Joint",
        # 左臂
        "left_shoulder_pitch_joint": "L_Shoulder_Pitch_Joint",
        "L_Shoulder_Pitch_Joint": "L_Shoulder_Pitch_Joint",
        "left_shoulder_yaw_joint": "L_Shoulder_Yaw_Joint",
        "L_Shoulder_Yaw_Joint": "L_Shoulder_Yaw_Joint",
        "left_shoulder_roll_joint": "L_Shoulder_Roll_Joint",
        "L_Shoulder_Roll_Joint": "L_Shoulder_Roll_Joint",
        "left_elbow_pitch_joint": "L_Elbow_Pitch_Joint",
        "L_Elbow_Pitch_Joint": "L_Elbow_Pitch_Joint",
        "left_wrist_roll_joint": "L_Wrist_Roll_Joint",
        "L_Wrist_Roll_Joint": "L_Wrist_Roll_Joint",
        "left_wrist_pitch_joint": "L_Wrist_Pitch_Joint",
        "L_Wrist_Pitch_Joint": "L_Wrist_Pitch_Joint",
        "left_wrist_yaw_joint": "L_Wrist_Yaw_Joint",
        "L_Wrist_Yaw_Joint": "L_Wrist_Yaw_Joint",
        "left_gripper_joint": "L_Hand_R_Joint",
        "L_Hand_R_Joint": "L_Hand_R_Joint",
        "L_Hand_L_Joint": "L_Hand_L_Joint",
        # 右臂
        "right_shoulder_pitch_joint": "R_Shoulder_Pitch_Joint",
        "R_Shoulder_Pitch_Joint": "R_Shoulder_Pitch_Joint",
        "right_shoulder_yaw_joint": "R_Shoulder_Yaw_Joint",
        "R_Shoulder_Yaw_Joint": "R_Shoulder_Yaw_Joint",
        "right_shoulder_roll_joint": "R_Shoulder_Roll_Joint",
        "R_Shoulder_Roll_Joint": "R_Shoulder_Roll_Joint",
        "right_elbow_pitch_joint": "R_Elbow_Pitch_Joint",
        "R_Elbow_Pitch_Joint": "R_Elbow_Pitch_Joint",
        "right_wrist_roll_joint": "R_Wrist_Roll_Joint",
        "R_Wrist_Roll_Joint": "R_Wrist_Roll_Joint",
        "right_wrist_pitch_joint": "R_Wrist_Pitch_Joint",
        "R_Wrist_Pitch_Joint": "R_Wrist_Pitch_Joint",
        "right_wrist_yaw_joint": "R_Wrist_Yaw_Joint",
        "R_Wrist_Yaw_Joint": "R_Wrist_Yaw_Joint",
        "right_gripper_joint": "R_Hand_R_Joint",
        "R_Hand_R_Joint": "R_Hand_R_Joint",
        "R_Hand_L_Joint": "R_Hand_L_Joint",
        # 头部
        "neck_joint": "Neck_Joint",
        "Neck_Joint": "Neck_Joint",
        "head_joint": "Head_Joint",
        "Head_Joint": "Head_Joint",
        "left_ear_joint": "L_Ear_Joint",
        "L_Ear_Joint": "L_Ear_Joint",
        "right_ear_joint": "R_Ear_Joint",
        "R_Ear_Joint": "R_Ear_Joint",
    }
    
    def __init__(self):
        super().__init__('gazebo_bridge_node')
        
        self.get_logger().info('🚀 启动 Gazebo 桥接节点')
        
        # 关节状态缓存
        self.joint_positions = {name: 0.0 for name in self.GAZEBO_JOINTS}
        
        # QoS 配置
        low_latency_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅 /joint_states（来自 sdk_bridge_node）
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            low_latency_qos
        )
        
        # 发布 JointTrajectory 给 Gazebo
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/gazebo_joint_commands',
            10
        )
        
        # 底盘控制：订阅 SDK 底盘速度并转发到 Gazebo
        self._setup_chassis_bridge()
        
        # 定时发布关节命令（50Hz）
        self.timer = self.create_timer(0.02, self._publish_joints)
        
        self.get_logger().info(f'📥 订阅: /joint_states')
        self.get_logger().info(f'📡 发布: /gazebo_joint_commands')
        self.get_logger().info(f'✅ 管理 {len(self.GAZEBO_JOINTS)} 个关节')
    
    def _setup_chassis_bridge(self):
        """设置底盘桥接（Zenoh → Gazebo）"""
        self._chassis_vx = 0.0
        self._chassis_vy = 0.0
        self._chassis_omega = 0.0
        
        # 发布到 Gazebo planar_move 插件的 cmd_vel
        self.chassis_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 尝试订阅 Zenoh 底盘话题
        if zenoh:
            try:
                config = zenoh.Config()
                self._zenoh_session = zenoh.open(config)
                self._zenoh_sub = self._zenoh_session.declare_subscriber(
                    "sdk/chassis",
                    self._on_chassis_zenoh
                )
                self.get_logger().info('📥 订阅 Zenoh: sdk/chassis')
            except Exception as e:
                self.get_logger().warn(f'Zenoh 底盘订阅失败: {e}')
                self._zenoh_session = None
        else:
            self._zenoh_session = None
            self.get_logger().warn('Zenoh 未安装，底盘控制使用 ROS2 话题')
            
            # 备用：订阅 ROS2 话题
            self.chassis_ros_sub = self.create_subscription(
                Twist,
                '/Teleop/cmd_vel',
                self._on_chassis_ros,
                10
            )
        
        # 底盘定时发布（20Hz）
        self.chassis_timer = self.create_timer(0.05, self._publish_chassis)
    
    def _on_chassis_zenoh(self, sample):
        """处理 Zenoh 底盘消息"""
        try:
            import json
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self._chassis_vx = data.get('vx', 0.0)
            self._chassis_vy = data.get('vy', 0.0)
            self._chassis_omega = data.get('omega', 0.0)
        except Exception as e:
            self.get_logger().warn(f'底盘消息解析失败: {e}')
    
    def _on_chassis_ros(self, msg: Twist):
        """处理 ROS2 底盘消息"""
        self._chassis_vx = msg.linear.x
        self._chassis_vy = msg.linear.y
        self._chassis_omega = msg.angular.z
    
    def _publish_chassis(self):
        """发布底盘速度到 Gazebo
        
        坐标转换：机器人面朝 Y 轴
        - SDK vx（前进）→ Gazebo linear.y
        - SDK vy（左移）→ Gazebo -linear.x
        - SDK omega（左转）→ Gazebo angular.z
        """
        msg = Twist()
        msg.linear.x = -self._chassis_vy  # 左移 → -X
        msg.linear.y = self._chassis_vx   # 前进 → +Y
        msg.angular.z = self._chassis_omega
        self.chassis_pub.publish(msg)
    
    def _on_joint_state(self, msg: JointState):
        """处理 /joint_states 消息"""
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                break
            
            # 映射关节名称
            gazebo_name = self.JOINT_NAME_MAP.get(name)
            if gazebo_name and gazebo_name in self.joint_positions:
                self.joint_positions[gazebo_name] = msg.position[i]
                
                # 夹爪同步（两指同步）
                if gazebo_name == "L_Hand_R_Joint":
                    self.joint_positions["L_Hand_L_Joint"] = msg.position[i]
                elif gazebo_name == "R_Hand_R_Joint":
                    self.joint_positions["R_Hand_L_Joint"] = msg.position[i]
    
    def _publish_joints(self):
        """发布 JointTrajectory 到 Gazebo"""
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = "world"
        msg.joint_names = self.GAZEBO_JOINTS
        
        point = JointTrajectoryPoint()
        point.positions = [self.joint_positions[name] for name in self.GAZEBO_JOINTS]
        point.time_from_start = Duration(sec=0, nanosec=20000000)  # 20ms
        
        msg.points = [point]
        self.traj_pub.publish(msg)
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, '_zenoh_session') and self._zenoh_session:
            self._zenoh_session.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
