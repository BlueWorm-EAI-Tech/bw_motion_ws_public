#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# 统一补全被动关节，保证 robot_state_publisher 的 TF 树不断裂。
# 控制源（GUI/IK/FILE）只需要专注于手臂输出；Router 负责补全与融合。
PASSIVE_JOINTS = [
    "Waist_Joint",
    "L_Hand_R_Joint",
    "L_Hand_L_Joint",
    "R_Hand_R_Joint",
    "R_Hand_L_Joint",
    "Neck_Joint",
    "Head_Joint",
    "L_Ear_Joint",
    "R_Ear_Joint",
    "Wheel_Left_Joint",
    "Wheel_Right_Joint",
    "Wheel_Back_Joint",
]

class InputRouterNode(Node):
    def __init__(self):
        super().__init__('input_router_node')

        self.declare_parameter('default_mode', 'gui')
        self.mode = str(self.get_parameter('default_mode').value).lower()  # 默认模式
        if self.mode not in ["gui", "ik", "file"]:
            self.get_logger().warn(f"Invalid default_mode '{self.mode}', fallback to 'gui'")
            self.mode = "gui"
        
        # 1. 订阅 GUI 输入
        self.sub_gui = self.create_subscription(
            JointState, '/input/gui/joint_states', self.gui_callback, 10)
            
        # 2. 订阅 IK 输入
        self.sub_ik = self.create_subscription(
            JointState, '/input/ik/joint_states', self.ik_callback, 10)

        # 3. [新增] 订阅 文件回放 输入
        self.sub_file = self.create_subscription(
            JointState, '/input/file/joint_states', self.file_callback, 10)
            
        # 4. 订阅模式切换指令
        self.sub_mode = self.create_subscription(
            String, '/sys/input_mode', self.mode_callback, 10)

        # 4.1 发布当前模式（latched，便于面板/调试工具显示）
        self.pub_mode_state = self.create_publisher(
            String, '/sys/input_mode_state', 10)

        # 5. 发布最终指令
        self.pub_target = self.create_publisher(
            JointState, '/ctrl/joint_target', 10)

        # 当当前模式没有任何输入源在发布时，robot_state_publisher 会一直缺 TF。
        # 这里用一个轻量的 idle 发布器维持 TF 连通（默认只补全被动关节 + 最近一次手臂状态）。
        self.declare_parameter('idle_publish_rate_hz', 30.0)
        self._idle_rate_hz = float(self.get_parameter('idle_publish_rate_hz').value)
        self._idle_enabled = self._idle_rate_hz > 0.0
        self._idle_timer = None
        if self._idle_enabled:
            self._idle_timer = self.create_timer(1.0 / self._idle_rate_hz, self._idle_timer_cb)

        # 缓存各输入源最近一帧，用于补全缺失关节
        self._last_gui: JointState | None = None
        self._last_ik: JointState | None = None
        self._last_file: JointState | None = None

        self.get_logger().info("Router Started. Modes: [GUI, IK, FILE]")
        self._publish_mode_state()

        # 诊断：周期性打印输入源是否存在（防止“没有 /ctrl/joint_target 输出但用户以为 router 坏了”）
        self._diag_timer = self.create_timer(2.0, self._diag_timer_cb)
        self._diag_last = {'gui': False, 'ik': False, 'file': False}

    def _cache_msg(self, source: str, msg: JointState):
        if source == 'gui':
            self._last_gui = msg
        elif source == 'ik':
            self._last_ik = msg
        elif source == 'file':
            self._last_file = msg

    def _get_last_full_state(self) -> JointState | None:
        # 优先使用当前模式的上一次消息作为参考，其次用任意模式的最近消息
        if self.mode == 'gui' and self._last_gui is not None:
            return self._last_gui
        if self.mode == 'ik' and self._last_ik is not None:
            return self._last_ik
        if self.mode == 'file' and self._last_file is not None:
            return self._last_file
        return self._last_ik or self._last_gui or self._last_file

    def _merge_to_full(self, src: JointState) -> JointState:
        """Merge src into a full JointState.

        Strategy:
        - start from a reference last_full_state (if any)
        - override with src names/positions
        - ensure PASSIVE_JOINTS exist (default 0.0)
        """
        ref = self._get_last_full_state()
        name_to_pos: dict[str, float] = {}

        if ref is not None and ref.name and ref.position:
            for n, p in zip(ref.name, ref.position):
                name_to_pos[n] = float(p)

        if src.name and src.position:
            for n, p in zip(src.name, src.position):
                name_to_pos[n] = float(p)

        # 确保关键被动关节存在
        for n in PASSIVE_JOINTS:
            name_to_pos.setdefault(n, 0.0)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        # 维持稳定顺序：先按 ref 顺序（若有），再补齐新增
        ordered: list[str] = []
        if ref is not None and ref.name:
            for n in ref.name:
                if n in name_to_pos and n not in ordered:
                    ordered.append(n)
        for n in list(src.name):
            if n in name_to_pos and n not in ordered:
                ordered.append(n)
        for n in PASSIVE_JOINTS:
            if n in name_to_pos and n not in ordered:
                ordered.append(n)

        out.name = ordered
        out.position = [name_to_pos[n] for n in ordered]
        return out

    def _publish_from(self, source: str, msg: JointState):
        self._cache_msg(source, msg)
        full = self._merge_to_full(msg)
        self.pub_target.publish(full)

    def _idle_timer_cb(self):
        # 如果当前模式已经有过消息，则不需要 idle 强刷。
        # 只有当"当前模式"没有任何 last msg 时，才发布一个纯补全的 JointState。
        if self.mode == 'gui' and self._last_gui is not None:
            return
        if self.mode == 'ik' and self._last_ik is not None:
            return
        if self.mode == 'file' and self._last_file is not None:
            return

        # 使用 ref（任意模式最近消息）来带上手臂姿态；若完全没有 ref，则仅维持被动关节 0。
        empty = JointState()
        empty.header.stamp = self.get_clock().now().to_msg()
        full = self._merge_to_full(empty)
        self.pub_target.publish(full)

    def _diag_timer_cb(self):
        # 这个诊断不会检查“是否有消息”，只检查“是否有发布者”，用于快速发现 GUI/IK 没集成的问题。
        try:
            has_gui = self.count_publishers('/input/gui/joint_states') > 0
            has_ik = self.count_publishers('/input/ik/joint_states') > 0
            has_file = self.count_publishers('/input/file/joint_states') > 0
        except Exception:
            return

        cur = {'gui': has_gui, 'ik': has_ik, 'file': has_file}
        if cur != self._diag_last:
            self._diag_last = cur
            self.get_logger().info(
                f"Input publishers: gui={has_gui} ik={has_ik} file={has_file} | current_mode={self.mode}"
            )

    def _publish_mode_state(self):
        msg = String()
        msg.data = self.mode
        self.pub_mode_state.publish(msg)

    def mode_callback(self, msg):
        cmd = msg.data.lower()
        if cmd in ["gui", "ik", "file"]:
            self.mode = cmd
            self.get_logger().info(f"Switched Input Mode to: [{self.mode.upper()}]")
            self._publish_mode_state()
        else:
            self.get_logger().warn(f"Unknown mode: {cmd}")

    def gui_callback(self, msg):
        if self.mode == "gui":
            self._publish_from('gui', msg)

    def ik_callback(self, msg):
        if self.mode == "ik":
            self._publish_from('ik', msg)

    def file_callback(self, msg):
        if self.mode == "file":
            self._publish_from('file', msg)

def main(args=None):
    rclpy.init(args=args)
    node = InputRouterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()