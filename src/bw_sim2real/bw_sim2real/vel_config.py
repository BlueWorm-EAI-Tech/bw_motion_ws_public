#!/usr/bin/env python3
"""Velocity config editor for bw_sim2real keyframe JSON.

Goal
- Edit per-frame speeds (rad/s) stored in keyframe json: top-level `speeds`.
- Keep backward compatibility: if file has no `speeds`, initialize them.

UI
- Matplotlib line plot for speeds.
- One vertical slider per frame (scrollable).
- Save writes back JSON (pretty-printed).

Usage
- Run as a normal python script in the ROS2 environment.

Notes
- This tool is intentionally standalone (not a ROS2 node).
"""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import matplotlib

# Prefer Qt backend if available.
matplotlib.use("Qt5Agg")

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from PyQt5 import QtCore, QtGui, QtWidgets

DEFAULT_SPEED_RAD_S = 1.0
MIN_SPEED = 1e-3


@dataclass
class KeyframeFile:
    path: str
    data: Dict[str, Any]
    keyframes: List[List[float]]
    speeds: List[float]


def _load_json(path: str) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_keyframe_file(path: str) -> KeyframeFile:
    raw = _load_json(path)

    if isinstance(raw, list):
        # legacy: [[...], ...]
        keyframes = raw
        data: Dict[str, Any] = {"meta": {"format": "legacy"}, "keyframes": keyframes}
        speeds = [DEFAULT_SPEED_RAD_S for _ in range(len(keyframes))]
        data["speeds"] = speeds
        return KeyframeFile(path=path, data=data, keyframes=keyframes, speeds=speeds)

    if not (isinstance(raw, dict) and "keyframes" in raw):
        raise ValueError("Unknown file format: expected list or dict with 'keyframes'")

    data = raw
    keyframes = data.get("keyframes") or []
    if not isinstance(keyframes, list) or not keyframes:
        raise ValueError("Empty or invalid 'keyframes'")

    speeds = data.get("speeds")
    if speeds is None:
        speeds = [DEFAULT_SPEED_RAD_S for _ in range(len(keyframes))]
        data["speeds"] = speeds
    else:
        if not isinstance(speeds, list) or len(speeds) != len(keyframes):
            raise ValueError("'speeds' must be a list with same length as 'keyframes'")
        speeds = [max(float(s), MIN_SPEED) for s in speeds]
        data["speeds"] = speeds

    return KeyframeFile(path=path, data=data, keyframes=keyframes, speeds=speeds)


def save_keyframe_file(kf: KeyframeFile, out_path: Optional[str] = None) -> None:
    out_path = out_path or kf.path
    tmp_path = out_path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(kf.data, f, ensure_ascii=False, indent=2)
    os.replace(tmp_path, out_path)


class VelConfigWindow(QtWidgets.QMainWindow):
    def __init__(self, kf: KeyframeFile, *, max_speed: float = 3.0):
        super().__init__()
        self.kf = kf
        self.max_speed = max(float(max_speed), 0.1)

        self.setWindowTitle(f"vel_config - {os.path.basename(kf.path)}")

        # Central layout
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # Plot
        self.fig = plt.Figure(figsize=(8, 3))
        self.canvas = FigureCanvas(self.fig)
        root.addWidget(self.canvas, stretch=2)

        # Slider area (scrollable)
        self.scroll = QtWidgets.QScrollArea(self)
        self.scroll.setWidgetResizable(True)
        root.addWidget(self.scroll, stretch=3)

        slider_host = QtWidgets.QWidget(self.scroll)
        self.scroll.setWidget(slider_host)
        self.slider_layout = QtWidgets.QHBoxLayout(slider_host)
        self.slider_layout.setContentsMargins(8, 8, 8, 8)
        self.slider_layout.setSpacing(6)

        # Buttons
        btn_row = QtWidgets.QHBoxLayout()
        root.addLayout(btn_row)

        self.btn_save = QtWidgets.QPushButton("保存 (Ctrl+S)")
        self.btn_save.clicked.connect(self.on_save)
        btn_row.addWidget(self.btn_save)

        self.btn_save_as = QtWidgets.QPushButton("另存为...")
        self.btn_save_as.clicked.connect(self.on_save_as)
        btn_row.addWidget(self.btn_save_as)

        self.btn_reload = QtWidgets.QPushButton("重载")
        self.btn_reload.clicked.connect(self.on_reload)
        btn_row.addWidget(self.btn_reload)

        btn_row.addStretch(1)

        self.btn_auto_smooth = QtWidgets.QPushButton("简单平滑")
        self.btn_auto_smooth.clicked.connect(self.on_auto_smooth)
        btn_row.addWidget(self.btn_auto_smooth)

        # Status
        self.status = QtWidgets.QLabel("")
        root.addWidget(self.status)

        # Shortcuts
        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+S"), self, activated=self.on_save)

        self.sliders = []
        self.slider_labels = []

        self._build_sliders()
        self._redraw_plot()

    def _build_sliders(self) -> None:
        # Clear old
        while self.slider_layout.count():
            item = self.slider_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

        self.sliders.clear()
        self.slider_labels.clear()

        for i, v in enumerate(self.kf.speeds):
            col = QtWidgets.QVBoxLayout()

            lab = QtWidgets.QLabel(f"{v:.2f}")
            lab.setAlignment(QtCore.Qt.AlignHCenter)
            col.addWidget(lab)

            s = QtWidgets.QSlider(QtCore.Qt.Vertical)
            s.setMinimum(0)
            s.setMaximum(1000)
            s.setValue(int(max(min(v / self.max_speed, 1.0), 0.0) * 1000))
            s.valueChanged.connect(lambda _, idx=i: self.on_slider_changed(idx))
            col.addWidget(s, stretch=1)

            idx_lab = QtWidgets.QLabel(str(i))
            idx_lab.setAlignment(QtCore.Qt.AlignHCenter)
            col.addWidget(idx_lab)

            wrap = QtWidgets.QWidget()
            wrap.setLayout(col)
            self.slider_layout.addWidget(wrap)

            self.sliders.append(s)
            self.slider_labels.append(lab)

        self.slider_layout.addStretch(1)

    def on_slider_changed(self, idx: int) -> None:
        s = self.sliders[idx]
        ratio = s.value() / 1000.0
        v = max(ratio * self.max_speed, MIN_SPEED)
        self.kf.speeds[idx] = float(v)
        self.kf.data["speeds"] = self.kf.speeds
        self.slider_labels[idx].setText(f"{v:.2f}")
        self._redraw_plot()

    def _redraw_plot(self) -> None:
        self.fig.clear()
        ax = self.fig.add_subplot(1, 1, 1)
        xs = list(range(len(self.kf.speeds)))
        ax.plot(xs, self.kf.speeds, "-o", linewidth=1.5, markersize=3)
        ax.set_title("Per-frame speed (rad/s)")
        ax.set_xlabel("frame index")
        ax.set_ylabel("speed")
        ax.set_ylim(0.0, self.max_speed)
        ax.grid(True, alpha=0.3)
        self.canvas.draw()

    def on_save(self) -> None:
        try:
            save_keyframe_file(self.kf, self.kf.path)
            self.status.setText(f"已保存：{self.kf.path}")
        except Exception as e:
            self.status.setText(f"保存失败：{e}")

    def on_save_as(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save As", self.kf.path, "JSON Files (*.json)")
        if not path:
            return
        try:
            save_keyframe_file(self.kf, path)
            self.status.setText(f"已另存为：{path}")
        except Exception as e:
            self.status.setText(f"保存失败：{e}")

    def on_reload(self) -> None:
        try:
            self.kf = load_keyframe_file(self.kf.path)
            self._build_sliders()
            self._redraw_plot()
            self.status.setText("已重载")
        except Exception as e:
            self.status.setText(f"重载失败：{e}")

    def on_auto_smooth(self) -> None:
        # Very lightweight smoothing: 1D moving average (3-tap) on speeds
        s = self.kf.speeds
        if len(s) < 3:
            return
        out = [s[0]]
        for i in range(1, len(s) - 1):
            out.append((s[i - 1] + s[i] + s[i + 1]) / 3.0)
        out.append(s[-1])
        out = [max(float(v), MIN_SPEED) for v in out]
        self.kf.speeds[:] = out
        self.kf.data["speeds"] = self.kf.speeds

        # refresh widgets
        for i, v in enumerate(self.kf.speeds):
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(int(max(min(v / self.max_speed, 1.0), 0.0) * 1000))
            self.sliders[i].blockSignals(False)
            self.slider_labels[i].setText(f"{v:.2f}")
        self._redraw_plot()
        self.status.setText("已平滑（3点均值）")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("file", help="keyframe json file")
    ap.add_argument("--max-speed", type=float, default=3.0, help="slider/plot max speed")
    args = ap.parse_args()

    kf = load_keyframe_file(args.file)

    app = QtWidgets.QApplication([])
    win = VelConfigWindow(kf, max_speed=args.max_speed)
    win.resize(1100, 650)
    win.show()
    app.exec_()


if __name__ == "__main__":
    main()
