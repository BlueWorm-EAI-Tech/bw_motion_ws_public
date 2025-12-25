import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

# 配置区域
# ---------------------------------------------------------

# 使用有序列表来定义仿真与实体关节名称的对应关系
# 这样可以通过 zip 一次性生成映射，保证顺序一致
URDF_JOINT_NAME = [
    # 左臂
    "L_Shoulder_Pitch_Joint",
    "L_Shoulder_Yaw_Joint",
    "L_Shoulder_Roll_Joint",
    "L_Elbow_Pitch_Joint",
    "L_Wrist_Roll_Joint",
    "L_Wrist_Pitch_Joint",
    "L_Wrist_Yaw_Joint",
    # 右臂
    "R_Shoulder_Pitch_Joint",
    "R_Shoulder_Yaw_Joint",
    "R_Shoulder_Roll_Joint",
    "R_Elbow_Pitch_Joint",
    "R_Wrist_Roll_Joint",
    "R_Wrist_Pitch_Joint",
    "R_Wrist_Yaw_Joint",
]

SERIAL_JOINT_NAME = [
    # 左臂
    "left_shoulder_pitch_joint",
    "left_shoulder_yaw_joint",
    "left_shoulder_roll_joint",
    "left_elbow_pitch_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    # 右臂
    "right_shoulder_pitch_joint",
    "right_shoulder_yaw_joint",
    "right_shoulder_roll_joint",
    "right_elbow_pitch_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# 映射表: 仿真名 -> 实体名（通过 zip 自动生成）
JOINT_NAME_MAP = dict(zip(URDF_JOINT_NAME, SERIAL_JOINT_NAME))
# 方向修正表: 1 = 正向, -1 = 反向 (因为URDF里的电机模型有些是反着装的)
JOINT_DIRECTION_MAP = {
    # 左臂
    "left_shoulder_pitch_joint": -1,
    "left_shoulder_yaw_joint": 1,
    "left_shoulder_roll_joint": -1,
    "left_elbow_pitch_joint": 1,
    "left_wrist_roll_joint": 1,
    "left_wrist_pitch_joint": -1,
    "left_wrist_yaw_joint": 1,
    # 右臂
    "right_shoulder_pitch_joint": -1,
    "right_shoulder_yaw_joint": 1,
    "right_shoulder_roll_joint": 1,
    "right_elbow_pitch_joint": 1,
    "right_wrist_roll_joint": 1,
    "right_wrist_pitch_joint": 1,
    "right_wrist_yaw_joint": 1,
}
# ---------------------------------------------------------

class SimToRealBridge(Node):
    def __init__(self):
        super().__init__('sim_to_real_bridge')

        # 参数：EMA 平滑因子与输出频率（Hz）
        self.declare_parameter('alpha', 0.1)
        self.declare_parameter('output_frequency', 500.0)
        # 速度规划相关
        self.declare_parameter('enable_speed_planning', True)
        self.declare_parameter('default_frame_speed_rad_s', 1.0)

        try:
            self.alpha = float(self.get_parameter('alpha').value)
        except Exception:
            self.alpha = 0.1
        try:
            self.output_frequency = float(self.get_parameter('output_frequency').value)
        except Exception:
            self.output_frequency = 100.0

        self.enable_speed_planning = bool(self.get_parameter('enable_speed_planning').value)
        self.default_frame_speed = float(self.get_parameter('default_frame_speed_rad_s').value)

        # 订阅路由层发来的指令 (仅更新目标状态，不直接发布)
        self.sub = self.create_subscription(
            JointState,
            '/ctrl/joint_target',
            self.listener_callback,
            10)

        # 回放/文件模式输入：mantis_playback_node 发布 URDF 命名的 JointState
        self.sub_file = self.create_subscription(
            JointState,
            '/input/file/joint_states',
            self.file_joint_callback,
            10)

        # 回放速度序列：与 keyframes 等长（每帧 rad/s）。
        # 约定：上层在开始播放前发布一次即可。
        self.sub_file_speeds = self.create_subscription(
            Float32MultiArray,
            '/input/file/speeds',
            self.file_speeds_callback,
            10)

        # 发布给实机（平滑后）
        self.pub = self.create_publisher(
            JointState,
            '/Teleop/joint_angle_solution/smooth',
            10)

        # === 控制输出侧状态（以 serial 名为 key） ===
        self.target_map = {name: 0.0 for name in SERIAL_JOINT_NAME}
        self.current_map = {name: 0.0 for name in SERIAL_JOINT_NAME}
        self.initialized = False

        # === 文件/回放速度规划状态（URDF 帧） ===
        # 最近一次收到的回放帧（按 JOINT_NAMES/URDF 顺序对齐后）
        self._file_prev = None
        self._file_next = None
        # 每帧速度（与帧序列等长，但这里 bridge 只需要“当前帧速度”（i->i+1））
        self._file_speed_prev = self.default_frame_speed
        self._file_speed_next = self.default_frame_speed
        self._file_speeds = None  # full list from /input/file/speeds
        self._file_speed_idx = 0  # segment index: i -> i+1
        # 段内推进参数
        self._seg_s = 0.0
        self._seg_len_inf = 0.0
        self._seg_active = False
        self._last_pub_time = None

        # 定时器：按 output_frequency 周期发布平滑数据（秒）
        period_sec = 1.0 / max(1.0, self.output_frequency)
        self.publish_timer = self.create_timer(period_sec, self.publish_smooth_callback)

        self.get_logger().info(
            f"Bridge Initialized. output_frequency={self.output_frequency}Hz alpha={self.alpha} speed_planning={self.enable_speed_planning}")

    def listener_callback(self, msg):
        # 更新目标状态（只更新 map，不发布）
        for i, name in enumerate(msg.name):
            if name in JOINT_NAME_MAP:
                real_name = JOINT_NAME_MAP[name]
                direction = JOINT_DIRECTION_MAP.get(real_name, 1)
                if i < len(msg.position):
                    raw_pos = msg.position[i]
                else:
                    raw_pos = 0.0
                self.target_map[real_name] = raw_pos * direction

        # 第一次收到数据时，将 current_map 初始化为 target_map 避免瞬变
        if not self.initialized:
            for n in SERIAL_JOINT_NAME:
                self.current_map[n] = self.target_map.get(n, 0.0)
            self.initialized = True

    # -----------------------
    # File playback callbacks
    # -----------------------
    def file_speeds_callback(self, msg: Float32MultiArray):
        data = list(msg.data) if msg.data is not None else []
        if not data:
            self._file_speeds = None
            return
        # 防御：全部速度必须 > 0
        self._file_speeds = [max(float(s), 1e-3) for s in data]
        # 便于 fallback：同步更新默认速度为第一个元素
        self.default_frame_speed = float(self._file_speeds[0])
        # 新速度表到来时，从头开始消费
        self._file_speed_idx = 0

    def file_joint_callback(self, msg: JointState):
        # 将输入 joint_state 对齐到 URDF_JOINT_NAME（即 mantis_playback_node 的 JOINT_NAMES）
        if not msg.name or not msg.position:
            return
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        aligned = []
        for jn in URDF_JOINT_NAME:
            idx = name_to_idx.get(jn)
            if idx is None or idx >= len(msg.position):
                return
            aligned.append(float(msg.position[idx]))

        # 第一次收到：只初始化 prev/next
        if self._file_prev is None:
            self._file_prev = aligned
            self._file_next = aligned
            self._seg_active = False
            self._file_speed_idx = 0
            return

        # 新帧到来：把上一段目标切换为新的段
        self._file_prev = self._file_next
        self._file_next = aligned

        # 计算段长度（∞-范数）用于定义“关节距离”
        diffs = [abs(b - a) for a, b in zip(self._file_prev, self._file_next)]
        self._seg_len_inf = max(diffs) if diffs else 0.0
        self._seg_s = 0.0
        self._seg_active = True

        # 段速度：按段消费 speeds[i]（i -> i+1），若无则 fallback default
        # 这里保留 prev/next 两端速度用于段内 smoothstep 插值
        self._file_speed_prev = float(self._file_speed_next)

        seg_v = None
        if self._file_speeds is not None and self._file_speed_idx < len(self._file_speeds):
            seg_v = float(self._file_speeds[self._file_speed_idx])
        if seg_v is None:
            seg_v = float(self.default_frame_speed)
        self._file_speed_next = max(float(seg_v), 1e-3)

        self._file_speed_idx += 1

    def publish_smooth_callback(self):
        # 如果未初始化，则不发布
        if not self.initialized:
            return

        # 如果启用速度规划且段处于 active：先根据速度在段内推进并更新 target_map
        if self.enable_speed_planning and self._seg_active and self._file_prev is not None and self._file_next is not None:
            # dt（秒）
            now_t = self.get_clock().now().nanoseconds
            if self._last_pub_time is None:
                dt = 1.0 / max(1.0, self.output_frequency)
            else:
                dt = max((now_t - self._last_pub_time) * 1e-9, 0.0)
            self._last_pub_time = now_t

            # 一阶可导速度插值：smoothstep
            def smoothstep(x: float) -> float:
                x = min(max(x, 0.0), 1.0)
                return x * x * (3.0 - 2.0 * x)

            # 用当前推进比例 s 计算当前速度（段两端速度平滑过渡）
            x = 0.0
            if self._seg_len_inf > 1e-9:
                x = self._seg_s / self._seg_len_inf
            a = smoothstep(x)
            v = (1.0 - a) * self._file_speed_prev + a * self._file_speed_next
            v = max(float(v), 1e-3)

            # 推进距离（按 ∞-范数定义速度）
            self._seg_s += v * dt

            # 结束判定
            if self._seg_len_inf <= 1e-9:
                ratio = 1.0
            else:
                ratio = min(self._seg_s / self._seg_len_inf, 1.0)

            # 段内位置：线性插值
            interp_urdf = [a0 + (b0 - a0) * ratio for a0, b0 in zip(self._file_prev, self._file_next)]

            # 将 URDF named 目标转为 serial target_map
            for urdf_name, urdf_pos in zip(URDF_JOINT_NAME, interp_urdf):
                real_name = JOINT_NAME_MAP.get(urdf_name)
                if not real_name:
                    continue
                direction = JOINT_DIRECTION_MAP.get(real_name, 1)
                self.target_map[real_name] = float(urdf_pos) * float(direction)

            if ratio >= 1.0:
                self._seg_active = False

        # 对 target_map 做 EMA 平滑并发布
        out_msg = JointState()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'mantis'
        out_msg.name = SERIAL_JOINT_NAME.copy()
        out_positions = []

        for n in SERIAL_JOINT_NAME:
            target = self.target_map.get(n, 0.0)
            current = self.current_map.get(n, 0.0)
            filtered = self.alpha * target + (1.0 - self.alpha) * current
            self.current_map[n] = filtered
            out_positions.append(filtered)

        out_msg.position = out_positions
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    node = SimToRealBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()