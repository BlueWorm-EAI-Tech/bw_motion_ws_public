#!/usr/bin/env python3
"""Mantis keyframe recorder & async player.

Why this exists:
- Recording + playback used to rely on terminal input() loops, which spam the
    terminal and is awkward to use alongside launch/rviz.
- This node now exposes a small PyQt5 panel to control record/save/load/play/stop.

Data contract:
- Subscribe recording source:
    - /ctrl/joint_target (default, record_source=control)
    - /joint_states_fdb (record_source=feedback)
- Publish playback to router:
    - /input/file/joint_states
- Request router mode:
    - /sys/input_mode (String: gui|ik|file)

File format:
- New format: {"meta": {...}, "keyframes": [[...], ...]}
- Backward compatible: [[...], ...] (assumed to match JOINT_NAMES)
"""

import json
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

# NOTE: UI has been moved to the separate package `bw_sim2real_view`.

from bw_interface.srv import (
    PlaybackClear,
    PlaybackLoad,
    PlaybackRecordFrame,
    PlaybackSave,
    PlaybackStart,
    PlaybackStop,
)

# 关节名称顺序 (必须与 URDF 一致)
JOINT_NAMES = [
    "L_Shoulder_Pitch_Joint", "L_Shoulder_Yaw_Joint", "L_Shoulder_Roll_Joint",
    "L_Elbow_Pitch_Joint", "L_Wrist_Roll_Joint", "L_Wrist_Pitch_Joint", "L_Wrist_Yaw_Joint",
    "R_Shoulder_Pitch_Joint", "R_Shoulder_Yaw_Joint", "R_Shoulder_Roll_Joint",
    "R_Elbow_Pitch_Joint", "R_Wrist_Roll_Joint", "R_Wrist_Pitch_Joint", "R_Wrist_Yaw_Joint"
]


DEFAULT_FRAME_SPEED_RAD_S = 1.0  # 默认每帧播放速度（用于段间速度规划）


class MantisPlaybackNode(Node):
    def __init__(self):
        super().__init__('mantis_playback_node')

        # === 参数配置 ===
        # record_source: 'control' (录制指令) 或 'feedback' (录制实机反馈)
        self.declare_parameter('record_source', 'control')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_joint_speed_rad_s', 0.5)
        self.declare_parameter('min_segment_duration_s', 0.5)
        self.declare_parameter('initial_move_duration_s', 2.0)

        self.source_type = self.get_parameter('record_source').get_parameter_value().string_value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.max_joint_speed = float(self.get_parameter('max_joint_speed_rad_s').value)
        self.min_seg_duration = float(self.get_parameter('min_segment_duration_s').value)
        self.initial_move_duration = float(self.get_parameter('initial_move_duration_s').value)

        # === 通信接口 ===
        # 1. 发布回放数据 (给 Router)
        self.pub_file = self.create_publisher(JointState, '/input/file/joint_states', 10)

        # 1.1 发布每帧速度序列（与 keyframes 等长，单位 rad/s）
        # 说明：sim_to_real_bridge 会订阅该话题；上层可在开始播放前发布一次即可。
        self.pub_speeds = self.create_publisher(Float32MultiArray, '/input/file/speeds', 10)

        # 2. 发布模式切换请求 (给 Router)
        self.pub_mode = self.create_publisher(String, '/sys/input_mode', 10)

        # 3. 订阅录制源
        topic_name = '/ctrl/joint_target' if self.source_type == 'control' else '/joint_states_fdb'
        self.sub_record = self.create_subscription(JointState, topic_name, self.record_callback, 10)

        self.get_logger().info(f"Recorder listening to: {topic_name}")
        self.get_logger().info("Player publishing to: /input/file/joint_states")
        self.get_logger().info("Player publishing to: /input/file/speeds")

        # === 内部状态 ===
        self._lock = threading.RLock()

        # 最新收到的一帧（按 JOINT_NAMES 顺序）
        self.current_joints = None
        self.keyframes = []
        # 每帧速度（与 keyframes 一一对应，单位 rad/s）
        self.frame_speeds = []

        self.is_playing = False
        self._play_thread = None
        self._stop_evt = threading.Event()

        self.loaded_filename = ""
        self.last_saved_filename = ""
        self.feedback_joint_names = []

        # 给 Qt 面板用的状态文本（避免在 ROS 回调里直接触 UI）
        self.status_text = "Ready"

        # === Services (UI/control plane) ===
        # 统一以 service 方式暴露功能，避免 UI 直接 import/调用此类。
        self.srv_record = self.create_service(
            PlaybackRecordFrame, '/playback/record_frame', self._srv_record_frame
        )
        self.srv_clear = self.create_service(
            PlaybackClear, '/playback/clear', self._srv_clear
        )
        self.srv_save = self.create_service(
            PlaybackSave, '/playback/save', self._srv_save
        )
        self.srv_load = self.create_service(
            PlaybackLoad, '/playback/load', self._srv_load
        )
        self.srv_start = self.create_service(
            PlaybackStart, '/playback/start', self._srv_start
        )
        self.srv_stop = self.create_service(
            PlaybackStop, '/playback/stop', self._srv_stop
        )

    def record_callback(self, msg):
        # 更稳健：按关节名对齐到 JOINT_NAMES
        if not msg.name or not msg.position:
            return

        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        aligned: List[float] = []
        for jn in JOINT_NAMES:
            idx = name_to_idx.get(jn)
            if idx is None or idx >= len(msg.position):
                return
            aligned.append(float(msg.position[idx]))

        with self._lock:
            self.current_joints = aligned

    def record_frame(self):
        with self._lock:
            if self.current_joints is None:
                self.status_text = "No data received yet"
                return False
            self.keyframes.append(list(self.current_joints))
            # 新录制帧默认速度
            self.frame_speeds.append(float(DEFAULT_FRAME_SPEED_RAD_S))
            self.status_text = f"Recorded frame #{len(self.keyframes)}"
            return True

    def _build_file_dict(self) -> Dict[str, Any]:
        return {
            "meta": {
                "format": "bw_sim2real_keyframes_v1",
                "joint_names": list(JOINT_NAMES),
                "record_source": self.source_type,
                "default_frame_speed_rad_s": float(DEFAULT_FRAME_SPEED_RAD_S),
                "created_unix": time.time(),
            },
            "keyframes": list(self.keyframes),
            # v2: speeds 与 keyframes 等长
            "speeds": list(self.frame_speeds),
        }

    def save_file(self, filename: str) -> Tuple[bool, str]:
        if not filename:
            return False, "Empty filename"
        if not filename.endswith('.json'):
            filename += '.json'
        try:
            with self._lock:
                payload = self._build_file_dict()
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            self.last_saved_filename = filename
            self.status_text = f"Saved {len(self.keyframes)} frames"
            return True, filename
        except Exception as e:
            self.status_text = f"Save failed: {e}"
            return False, str(e)

    def switch_router_mode(self, mode):
        msg = String()
        msg.data = mode
        self.pub_mode.publish(msg)
        # 给一点时间让 Router 反应
        time.sleep(0.1)

    def load_file(self, filename: str) -> Tuple[bool, str]:
        if not filename:
            return False, "Empty filename"
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.status_text = f"Load failed: {e}"
            return False, str(e)

        keyframes: List[List[float]]
        speeds: Optional[List[float]] = None
        if isinstance(data, list):
            # 旧格式：直接是 [[...], ...]
            keyframes = data
            joint_names = list(JOINT_NAMES)
        elif isinstance(data, dict) and 'keyframes' in data:
            keyframes = data.get('keyframes') or []
            meta = data.get('meta') or {}
            joint_names = meta.get('joint_names') or list(JOINT_NAMES)
            speeds = data.get('speeds')
        else:
            self.status_text = "Unknown file format"
            return False, "Unknown file format"

        if not keyframes:
            self.status_text = "Empty keyframes"
            return False, "Empty keyframes"

        # 校验每帧维度
        expected_dim = len(JOINT_NAMES)
        for i, fr in enumerate(keyframes):
            if not isinstance(fr, list) or len(fr) != expected_dim:
                self.status_text = f"Frame {i} dim mismatch"
                return False, f"Frame {i} dim mismatch"

        # 速度数组：可缺省（旧文件默认每帧 1.0rad/s）
        if speeds is not None:
            if (not isinstance(speeds, list)) or (len(speeds) != len(keyframes)):
                self.status_text = "Speeds length mismatch"
                return False, "Speeds length mismatch"
            # 防御性：速度必须为正
            speeds = [max(float(s), 1e-3) for s in speeds]
        else:
            speeds = [float(DEFAULT_FRAME_SPEED_RAD_S) for _ in range(len(keyframes))]

        # joint_names 不一致时：目前仅提示（可扩展为映射）
        if list(joint_names) != list(JOINT_NAMES):
            self.get_logger().warn(
                "Keyframe joint_names != current JOINT_NAMES; using current ordering."
            )

        with self._lock:
            self.keyframes = [list(fr) for fr in keyframes]
            self.frame_speeds = list(speeds)
        self.loaded_filename = filename
        self.status_text = f"Loaded {len(self.keyframes)} frames"
        return True, filename

    def start_playback(self, filename: str, speed: float = 1.0, loop: bool = False) -> Tuple[bool, str]:
        if self.is_playing:
            return False, "Already playing"
        ok, msg = self.load_file(filename)
        if not ok:
            return False, msg

        self._stop_evt.clear()
        self.is_playing = True

        t = threading.Thread(target=self._play_worker, args=(speed, loop), daemon=True)
        self._play_thread = t
        t.start()
        self.status_text = "Playback started"
        return True, "started"

    def stop_playback(self):
        if not self.is_playing:
            return
        self._stop_evt.set()
        self.is_playing = False
        self.status_text = "Stopping..."

    # ---------------------
    # Service callbacks
    # ---------------------
    def _srv_record_frame(self, request, response):
        ok = self.record_frame()
        response.success = bool(ok)
        response.message = self.status_text
        return response

    def _srv_clear(self, request, response):
        with self._lock:
            self.keyframes = []
        self.status_text = 'Cleared keyframes'
        response.success = True
        response.message = self.status_text
        return response

    def _srv_save(self, request, response):
        ok, msg = self.save_file(request.file_path)
        response.success = bool(ok)
        response.message = msg if msg else self.status_text
        return response

    def _srv_load(self, request, response):
        ok, msg = self.load_file(request.file_path)
        response.success = bool(ok)
        response.message = msg
        return response

    def _srv_start(self, request, response):
        ok, msg = self.start_playback(
            request.file_path,
            speed=float(request.speed) if request.speed > 0.0 else 1.0,
            loop=bool(request.loop),
        )
        response.success = bool(ok)
        response.message = msg
        return response

    def _srv_stop(self, request, response):
        self.stop_playback()
        response.success = True
        response.message = self.status_text
        return response

    def _play_worker(self, speed: float, loop: bool):
        # 防御
        speed = max(float(speed), 0.01)

        # 1) 自动切换 Router 到 FILE 模式
        self.switch_router_mode("file")

        with self._lock:
            keyframes = [list(fr) for fr in self.keyframes]
            frame_speeds = [float(s) for s in self.frame_speeds]
            current = list(self.current_joints) if self.current_joints is not None else None

        # 开始播放前发布一次速度表（供 bridge 做逐段速度规划）
        speeds_msg = Float32MultiArray()
        speeds_msg.data = [max(float(s), 1e-3) for s in frame_speeds] if frame_speeds else [float(DEFAULT_FRAME_SPEED_RAD_S)]
        self.pub_speeds.publish(speeds_msg)

        loop_cnt = 0
        while (not self._stop_evt.is_set()) and rclpy.ok():
            loop_cnt += 1
            # 先移动到第一帧（慢速安全）
            self.move_segment(current, keyframes[0], self.initial_move_duration, speed_scale=1.0)

            # 播放后续帧
            for i in range(len(keyframes) - 1):
                if self._stop_evt.is_set() or (not rclpy.ok()):
                    break
                start_pt = keyframes[i]
                end_pt = keyframes[i + 1]

                # 变速规划：每段持续时间由 per-frame speed 决定
                # 约定：speeds[i] 表示从 keyframes[i] -> keyframes[i+1] 的期望速度（rad/s）
                seg_speed = float(frame_speeds[i]) if i < len(frame_speeds) else float(DEFAULT_FRAME_SPEED_RAD_S)
                seg_speed = max(seg_speed, 1e-3)

                # 段距离用 ∞-范数（最大关节变化量），与 bridge 的定义一致
                max_diff = max(abs(e - s) for s, e in zip(start_pt, end_pt))

                # duration = distance / speed，再加最小段时长防止太短
                duration = max(max_diff / seg_speed, self.min_seg_duration) / speed

                self.move_segment(start_pt, end_pt, duration, speed_scale=1.0)

            if (not loop) or self._stop_evt.is_set():
                break
            time.sleep(0.5)

        self.is_playing = False
        self.status_text = "Playback finished"
        # 播放结束不强制切回 GUI，避免机器人归零

    def move_segment(self, start_j: Optional[Sequence[float]], end_j: Sequence[float], duration: float, *, speed_scale: float = 1.0):
        """执行两点间的插值运动"""
        if not start_j:
            start_j = [0.0] * len(JOINT_NAMES)  # 防御性编程

        freq = max(self.publish_rate_hz, 1.0)
        steps = max(int(max(duration, 0.02) * freq), 1)

        msg = JointState()
        # 方案A：控制源只发布手臂；由 input_router 统一补全被动关节，保证 TF 不断裂。
        msg.name = list(JOINT_NAMES)
        
        for step in range(steps):
            if self._stop_evt.is_set() or (not self.is_playing):
                return

            alpha = step / float(steps)
            # 线性插值 (Linear Interpolation)
            interp_pos = [s + (e-s)*alpha for s,e in zip(start_j, end_j)]
            
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = list(interp_pos)
            self.pub_file.publish(msg)
            time.sleep(1.0 / freq)


def main(args=None):
    rclpy.init(args=args)
    node = MantisPlaybackNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()