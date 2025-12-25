import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool  # 【新增】用于发送上电指令
from bw_interface.msg import Event 
import time
import threading
import sys

# 目标关节顺序
TARGET_JOINT_ORDER = [
    "left_shoulder_pitch_joint", "left_shoulder_yaw_joint", "left_shoulder_roll_joint",
    "left_elbow_pitch_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_yaw_joint", "right_shoulder_roll_joint",
    "right_elbow_pitch_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
]

# 关节名称映射表
JOINT_NAME_MAP = {
    "left_shoulder_pitch_joint": "L_Shoulder_Pitch_Joint",
    "left_shoulder_yaw_joint":   "L_Shoulder_Yaw_Joint",
    "left_shoulder_roll_joint":  "L_Shoulder_Roll_Joint",
    "left_elbow_pitch_joint":    "L_Elbow_Pitch_Joint",
    "left_wrist_roll_joint":     "L_Wrist_Roll_Joint",
    "left_wrist_pitch_joint":    "L_Wrist_Pitch_Joint",
    "left_wrist_yaw_joint":      "L_Wrist_Yaw_Joint",
    
    "right_shoulder_pitch_joint": "R_Shoulder_Pitch_Joint",
    "right_shoulder_yaw_joint":   "R_Shoulder_Yaw_Joint",
    "right_shoulder_roll_joint":  "R_Shoulder_Roll_Joint",
    "right_elbow_pitch_joint":    "R_Elbow_Pitch_Joint",
    "right_wrist_roll_joint":     "R_Wrist_Roll_Joint",
    "right_wrist_pitch_joint":    "R_Wrist_Pitch_Joint",
    "right_wrist_yaw_joint":      "R_Wrist_Yaw_Joint"
}

class ZenohBridge(Node):
    def __init__(self):
        super().__init__('sim_to_real_bridge')

        # 1. 订阅仿真环境 (MoveIt/RViz)
        self.sim_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.sim_callback,
            10
        )

        # 2. 发布给真机的关节数据
        self.real_pub = self.create_publisher(
            JointState,
            '/Teleop/joint_angle_solution/smooth',
            10
        )

        # 3. 状态机事件发布
        self.event_pub = self.create_publisher(Event, '/event', 10)

        # 4. 【核心修改】不再直连 Service，改为发布 Topic 指令
        # 这个 Topic 会通过 Zenoh 传给 RobotPC 上的代理节点
        self.proxy_pub = self.create_publisher(Bool, '/cmd_enable_motor', 10)

        self.robot_initialized = False
        self.latest_sim_joints = {} 
        
        # 频率限制 (50Hz)
        self.timer = self.create_timer(0.02, self.publish_loop)
        
        self.get_logger().info("🔗 [RemotePC] 桥接节点启动，使用 Topic 代理模式")

    def initialize_robot(self):
        """通过 Topic 发送握手与上电指令"""
        if self.robot_initialized: return
        
        self.get_logger().info("🚀 正在激活机器人...")
        
        # 1. 发送状态机事件 (欺骗上层逻辑)
        def send_event(evt):
            msg = Event()
            msg.event_type = evt
            self.event_pub.publish(msg)
            time.sleep(0.1)

        send_event(Event.VR_READY)
        send_event(Event.LEFT_POSE_RESET)
        send_event(Event.RIGHT_POSE_RESET)
        send_event(Event.LEFT_START_TELEOP)
        send_event(Event.RIGHT_START_TELEOP)
        
        self.get_logger().info("✅ 状态机事件已发送")

        # 2. 【核心修改】发布上电指令到 Topic
        # 我们多发几次，确保 Zenoh 肯定传过去了 (UDP 可能会丢)
        enable_msg = Bool()
        enable_msg.data = True
        
        for i in range(5): 
            self.proxy_pub.publish(enable_msg)
            time.sleep(0.2)
            self.get_logger().info(f"⚡ 发送上电指令 Topic... ({i+1}/5)")

        self.get_logger().info("✅ 激活指令已发出，等待 RobotPC 代理执行...")
        self.robot_initialized = True

    def sim_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.latest_sim_joints[name] = msg.position[i]

    def publish_loop(self):
        if not self.robot_initialized: return
        if not self.latest_sim_joints: return

        real_msg = JointState()
        real_msg.header.stamp = self.get_clock().now().to_msg()
        real_msg.name = TARGET_JOINT_ORDER
        
        positions = []
        try:
            for target_name in TARGET_JOINT_ORDER:
                source_name = JOINT_NAME_MAP.get(target_name)
                val = self.latest_sim_joints.get(source_name, 0.0)
                positions.append(val)
            
            real_msg.position = positions
            self.real_pub.publish(real_msg)
            
        except Exception as e:
            self.get_logger().warn(f"数据转换异常: {e}")

def main():
    rclpy.init()
    node = ZenohBridge()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        print("\n===== 数字孪生同步桥接器 (Topic Proxy Mode) =====")
        print("请确保 RobotPC 上运行了 'service_proxy_node.py' ！")
        input("按 [回车] 键激活机器人并开始同步 >>> ")
        
        node.initialize_robot()
        
        print("\n✅ 同步中... 在 RViz 中拖动机器人，真机会跟随。")
        print("按 Ctrl+C 退出")
        
        while rclpy.ok():
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()