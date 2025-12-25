#!/usr/bin/env python3
"""Unified Qt control panel for bw_sim2real.

This panel is intentionally separated into a dedicated ROS package (bw_sim2real_view)
so UI doesn't live inside control/algorithm nodes.

Responsibilities:
- Show current input mode (/sys/input_mode_state)
- Offer quick mode switch buttons (GUI/IK/FILE)
- Provide a simple keyframe control surface by publishing commands.

Note:
- For now, to keep changes safe and incremental, the panel directly publishes mode
  commands only. In the next iteration we can add services/actions for record/play.
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from bw_interface.srv import (
    PlaybackClear,
    PlaybackLoad,
    PlaybackRecordFrame,
    PlaybackSave,
    PlaybackStart,
    PlaybackStop,
)

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QDoubleSpinBox,
    QSlider,
    QVBoxLayout,
    QWidget,
)


# ----------------------
# GUI slider definitions
# ----------------------
LEFT_ARM_JOINTS = [
    "L_Shoulder_Pitch_Joint",
    "L_Shoulder_Yaw_Joint",
    "L_Shoulder_Roll_Joint",
    "L_Elbow_Pitch_Joint",
    "L_Wrist_Roll_Joint",
    "L_Wrist_Pitch_Joint",
    "L_Wrist_Yaw_Joint",
]

RIGHT_ARM_JOINTS = [
    "R_Shoulder_Pitch_Joint",
    "R_Shoulder_Yaw_Joint",
    "R_Shoulder_Roll_Joint",
    "R_Elbow_Pitch_Joint",
    "R_Wrist_Roll_Joint",
    "R_Wrist_Pitch_Joint",
    "R_Wrist_Yaw_Joint",
]

# joint limits (radians). Keep consistent with previous mantis_gui_node.
JOINT_LIMITS = {
    "L_Shoulder_Pitch_Joint": (-1.13, 1.75),
    "L_Shoulder_Yaw_Joint": (-0.213, 2.029),
    "L_Shoulder_Roll_Joint": (-0.80, 0.82),
    "L_Elbow_Pitch_Joint": (-0.395, 1.012),
    "L_Wrist_Roll_Joint": (-1.7, 1.7),
    "L_Wrist_Pitch_Joint": (-0.562, 0.562),
    "L_Wrist_Yaw_Joint": (-1.7, 1.7),
    "R_Shoulder_Pitch_Joint": (-1.75, 1.13),
    "R_Shoulder_Yaw_Joint": (-2.029, 0.213),
    "R_Shoulder_Roll_Joint": (-0.82, 0.80),
    "R_Elbow_Pitch_Joint": (-0.395, 1.012),
    "R_Wrist_Roll_Joint": (-1.7, 1.7),
    "R_Wrist_Pitch_Joint": (-0.562, 0.562),
    "R_Wrist_Yaw_Joint": (-1.7, 1.7),
}


class ControlPanelWidget(QWidget):
    def __init__(self, ros_node: 'ControlPanelNode'):
        super().__init__()
        self.node = ros_node

        self.setWindowTitle('Mantis Control Panel (bw_sim2real_view)')
        self.setMinimumWidth(420)
        self.setStyleSheet('background-color: #2b2b2b; color: #ffffff;')

        root = QVBoxLayout()

        self.lbl_mode = QLabel('Mode: (unknown)')
        self.lbl_mode.setStyleSheet('color: #9cdcfe; font-weight: bold;')
        root.addWidget(self.lbl_mode)

        mode_group = QGroupBox('Input Mode')
        row = QHBoxLayout()
        self.btn_gui = QPushButton('GUI')
        self.btn_ik = QPushButton('IK')
        self.btn_file = QPushButton('FILE')
        row.addWidget(self.btn_gui)
        row.addWidget(self.btn_ik)
        row.addWidget(self.btn_file)
        mode_group.setLayout(row)
        root.addWidget(mode_group)

        self.btn_gui.clicked.connect(lambda: self.node.set_mode('gui'))
        self.btn_ik.clicked.connect(lambda: self.node.set_mode('ik'))
        self.btn_file.clicked.connect(lambda: self.node.set_mode('file'))

        # --- GUI slider group ---
        gui_group = QGroupBox('GUI Sliders (publish /input/gui/joint_states)')
        gui_layout = QHBoxLayout()
        gui_layout.addLayout(self._create_arm_slider_group('Left Arm', LEFT_ARM_JOINTS))
        gui_layout.addLayout(self._create_arm_slider_group('Right Arm', RIGHT_ARM_JOINTS))
        gui_group.setLayout(gui_layout)
        root.addWidget(gui_group)

        # --- playback group ---
        pb_group = QGroupBox('Keyframe Playback')
        pb_layout = QVBoxLayout()

        file_row = QHBoxLayout()
        self.lbl_file = QLabel('(no file)')
        self.btn_pick = QPushButton('Pick JSON...')
        self.btn_pick.clicked.connect(self.on_pick_file)
        file_row.addWidget(self.lbl_file, 1)
        file_row.addWidget(self.btn_pick)
        pb_layout.addLayout(file_row)

        opt_row = QHBoxLayout()
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.05, 5.0)
        self.spin_speed.setSingleStep(0.05)
        self.spin_speed.setValue(1.0)
        self.chk_loop = QCheckBox('Loop')
        opt_row.addWidget(QLabel('Speed'))
        opt_row.addWidget(self.spin_speed)
        opt_row.addWidget(self.chk_loop)
        pb_layout.addLayout(opt_row)

        btn_row = QHBoxLayout()
        self.btn_record = QPushButton('Record Frame')
        self.btn_clear = QPushButton('Clear')
        self.btn_load = QPushButton('Load')
        self.btn_save = QPushButton('Save')
        self.btn_play = QPushButton('Play')
        self.btn_stop = QPushButton('Stop')
        for b in [self.btn_record, self.btn_clear, self.btn_load, self.btn_save, self.btn_play, self.btn_stop]:
            btn_row.addWidget(b)
        pb_layout.addLayout(btn_row)

        self.lbl_playback = QLabel('Playback: (idle)')
        self.lbl_playback.setStyleSheet('color: #c586c0;')
        pb_layout.addWidget(self.lbl_playback)

        pb_group.setLayout(pb_layout)
        root.addWidget(pb_group)

        # button wiring
        self.btn_record.clicked.connect(self.on_record)
        self.btn_clear.clicked.connect(self.on_clear)
        self.btn_load.clicked.connect(self.on_load)
        self.btn_save.clicked.connect(self.on_save)
        self.btn_play.clicked.connect(self.on_play)
        self.btn_stop.clicked.connect(self.on_stop)

        self.setLayout(root)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

        # publish sliders at ~30Hz
        self.pub_timer = QTimer()
        self.pub_timer.timeout.connect(self.publish_gui_joints)
        self.pub_timer.start(33)

    def _create_arm_slider_group(self, title: str, joints: list[str]) -> QVBoxLayout:
        col = QVBoxLayout()
        col.addWidget(QLabel(f'{title}'))
        for name in joints:
            row = QHBoxLayout()
            label = QLabel(name)
            label.setFixedWidth(160)
            val_label = QLabel('0.00')
            val_label.setFixedWidth(45)

            slider = QSlider()
            slider.setOrientation(1)  # Qt.Horizontal == 1

            lo, hi = JOINT_LIMITS.get(name, (-1.0, 1.0))
            slider.setRange(int(lo * 100), int(hi * 100))
            slider.setValue(0)

            def _on_change(val: int, n=name, l=val_label):
                real_val = val / 100.0
                self.node.gui_joint_values[n] = float(real_val)
                l.setText(f'{real_val:.2f}')

            slider.valueChanged.connect(_on_change)
            self.node.gui_joint_values.setdefault(name, 0.0)

            row.addWidget(label)
            row.addWidget(slider)
            row.addWidget(val_label)
            col.addLayout(row)
        return col

    def refresh(self):
        self.lbl_mode.setText(f"Mode: {self.node.current_mode}")
        self.lbl_playback.setText(f"Playback: {self.node.playback_status}")

    def on_pick_file(self):
        fname, _ = QFileDialog.getOpenFileName(self, 'Select keyframe json', '', 'JSON files (*.json)')
        if fname:
            self.node.playback_file = fname
            self.lbl_file.setText(fname)

    def on_record(self):
        self.node.call_record()

    def on_clear(self):
        self.node.call_clear()

    def on_load(self):
        if not self.node.playback_file:
            self.on_pick_file()
        if self.node.playback_file:
            self.node.call_load(self.node.playback_file)

    def on_save(self):
        default = self.node.playback_file or 'motion.json'
        fname, _ = QFileDialog.getSaveFileName(self, 'Save keyframes', default, 'JSON files (*.json)')
        if fname:
            self.node.playback_file = fname
            self.lbl_file.setText(fname)
            self.node.call_save(fname)

    def on_play(self):
        if not self.node.playback_file:
            self.on_pick_file()
        if self.node.playback_file:
            self.node.call_start(
                self.node.playback_file,
                speed=float(self.spin_speed.value()),
                loop=bool(self.chk_loop.isChecked()),
            )

    def on_stop(self):
        self.node.call_stop()

    def publish_gui_joints(self):
        # Regardless of mode, it's harmless to publish; router will only use it when mode==gui.
        self.node.publish_gui_joint_state()


class ControlPanelNode(Node):
    def __init__(self):
        super().__init__('mantis_control_panel')

        self.current_mode = '(unknown)'
        self.playback_status = '(unknown)'
        self.playback_file = ''

        self.pub_mode = self.create_publisher(String, '/sys/input_mode', 10)
        self.sub_mode = self.create_subscription(String, '/sys/input_mode_state', self._mode_cb, 10)

        # GUI slider output (hand-only; router will complete passive joints)
        self.pub_gui = self.create_publisher(JointState, '/input/gui/joint_states', 10)
        self.gui_joint_values = {}

        # service clients
        self.cli_record = self.create_client(PlaybackRecordFrame, '/playback/record_frame')
        self.cli_clear = self.create_client(PlaybackClear, '/playback/clear')
        self.cli_load = self.create_client(PlaybackLoad, '/playback/load')
        self.cli_save = self.create_client(PlaybackSave, '/playback/save')
        self.cli_start = self.create_client(PlaybackStart, '/playback/start')
        self.cli_stop = self.create_client(PlaybackStop, '/playback/stop')

    def _mode_cb(self, msg: String):
        self.current_mode = msg.data

    def set_mode(self, mode: str):
        m = String()
        m.data = mode
        self.pub_mode.publish(m)

    def publish_gui_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # keep stable order
        names = list(self.gui_joint_values.keys())
        msg.name = names
        msg.position = [float(self.gui_joint_values[n]) for n in names]
        self.pub_gui.publish(msg)

    def _call(self, client, req, label: str):
        if not client.service_is_ready():
            # don't block UI; just update status
            self.playback_status = f'{label}: service not ready'
            return
        fut = client.call_async(req)

        def _done(_fut):
            try:
                resp = _fut.result()
                if hasattr(resp, 'message'):
                    self.playback_status = resp.message
                elif hasattr(resp, 'success'):
                    self.playback_status = f'{label}: {resp.success}'
                else:
                    self.playback_status = f'{label}: done'
            except Exception as e:
                self.playback_status = f'{label}: {e}'

        fut.add_done_callback(_done)

    def call_record(self):
        req = PlaybackRecordFrame.Request()
        self._call(self.cli_record, req, 'record')

    def call_clear(self):
        req = PlaybackClear.Request()
        self._call(self.cli_clear, req, 'clear')

    def call_load(self, file_path: str):
        req = PlaybackLoad.Request()
        req.file_path = file_path
        self._call(self.cli_load, req, 'load')

    def call_save(self, file_path: str):
        req = PlaybackSave.Request()
        req.file_path = file_path
        self._call(self.cli_save, req, 'save')

    def call_start(self, file_path: str, speed: float, loop: bool):
        req = PlaybackStart.Request()
        req.file_path = file_path
        req.speed = float(speed)
        req.loop = bool(loop)
        self._call(self.cli_start, req, 'start')

    def call_stop(self):
        req = PlaybackStop.Request()
        self._call(self.cli_stop, req, 'stop')


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanelNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    w = ControlPanelWidget(node)
    w.show()

    while rclpy.ok() and w.isVisible():
        app.processEvents()
        time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
