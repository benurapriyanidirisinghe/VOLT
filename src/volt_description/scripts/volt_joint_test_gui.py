#!/usr/bin/env python3

import signal
import sys

import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSlider,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from volt_kinematics import JOINT_NAMES, WALK_POSE


JOINT_LIMITS = [
    (-0.548, 0.548),
    (-2.666, 1.548),
    (-2.600, 0.100),
    (-0.548, 0.548),
    (-2.666, 1.548),
    (-2.600, 0.100),
    (-0.548, 0.548),
    (-2.666, 1.548),
    (-2.600, 0.100),
    (-0.548, 0.548),
    (-2.666, 1.548),
    (-2.600, 0.100),
]

SLIDER_SCALE = 1000


class JointTestNode(Node):
    def __init__(self, status_callback):
        super().__init__("volt_joint_test_gui")
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            "/volt/joint_commands/calibration",
            10,
        )
        self.owner_publisher = self.create_publisher(String, "/volt/command_owner", 10)
        self.serial_command_publisher = self.create_publisher(
            String,
            "/volt/serial_command",
            10,
        )
        self.create_subscription(String, "/volt/serial_status", status_callback, 10)

    def publish_joints(self, values):
        self.publish_owner("CALIBRATION")
        message = Float64MultiArray()
        message.data = [float(value) for value in values]
        self.command_publisher.publish(message)

    def publish_serial_command(self, command):
        message = String()
        message.data = command
        self.serial_command_publisher.publish(message)

    def publish_owner(self, owner):
        message = String()
        message.data = owner
        self.owner_publisher.publish(message)


class JointRow:
    def __init__(self, index, name, limits, changed_callback):
        self.index = index
        self.name = name
        self.low, self.high = limits
        self.changed_callback = changed_callback
        self.updating = False

        self.name_label = QLabel("%02d  %s" % (index, name))
        self.name_label.setMinimumWidth(210)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(self.low * SLIDER_SCALE))
        self.slider.setMaximum(int(self.high * SLIDER_SCALE))
        self.slider.setSingleStep(5)
        self.slider.setPageStep(50)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(self.low, self.high)
        self.spin.setDecimals(3)
        self.spin.setSingleStep(0.01)
        self.spin.setSuffix(" rad")
        self.spin.setMinimumWidth(115)

        self.deg_label = QLabel("0.0 deg")
        self.deg_label.setMinimumWidth(75)
        self.limit_label = QLabel("[%.3f, %.3f]" % (self.low, self.high))
        self.limit_label.setMinimumWidth(120)

        self.slider.valueChanged.connect(self.slider_changed)
        self.spin.valueChanged.connect(self.spin_changed)
        self.set_value(0.0, emit=False)

    def value(self):
        return float(self.spin.value())

    def set_value(self, value, emit=True):
        value = max(self.low, min(self.high, float(value)))
        self.updating = True
        self.slider.setValue(int(round(value * SLIDER_SCALE)))
        self.spin.setValue(value)
        self.deg_label.setText("%.1f deg" % (value * 57.2957795))
        self.updating = False
        if emit:
            self.changed_callback()

    def slider_changed(self, raw_value):
        if self.updating:
            return
        self.set_value(raw_value / float(SLIDER_SCALE))

    def spin_changed(self, value):
        if self.updating:
            return
        self.set_value(value)


class JointTestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VOLT Joint Test")
        self.resize(980, 720)

        self.ros_node = JointTestNode(self.status_callback)
        self.ros_node.publish_owner("CALIBRATION")
        self.rows = []
        self.last_values = [0.0 for _ in JOINT_NAMES]
        self.status_text = "serial: unknown"
        self.shutting_down = False
        self.hardware_ready = False

        self.build_ui()
        self.apply_style()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin_ros)
        self.spin_timer.start(10)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.publish_if_live)
        self.publish_timer.start(50)

    def build_ui(self):
        root = QWidget()
        main_layout = QVBoxLayout(root)
        main_layout.setContentsMargins(14, 14, 14, 14)
        main_layout.setSpacing(10)

        header = QHBoxLayout()
        self.live_checkbox = QCheckBox("Live publish")
        self.live_checkbox.setChecked(True)
        self.hardware_label = QLabel("Hardware: unknown")
        self.hardware_label.setObjectName("hardwareState")
        self.status_label = QLabel(self.status_text)
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(42)

        send_button = QPushButton("Send Once")
        zero_button = QPushButton("Zero / Loaded Pose")
        walk_button = QPushButton("Walk Ready")
        arm_button = QPushButton("Arm Arduino")
        hold_button = QPushButton("Hold Arduino")
        disarm_button = QPushButton("Disarm Arduino")
        disable_button = QPushButton("Disable Pulses")
        status_button = QPushButton("Status")

        send_button.clicked.connect(self.publish_once)
        zero_button.clicked.connect(self.zero_pose)
        walk_button.clicked.connect(self.walk_ready_pose)
        arm_button.clicked.connect(lambda: self.send_serial("ARM"))
        hold_button.clicked.connect(lambda: self.send_serial("HOLD"))
        disarm_button.clicked.connect(lambda: self.send_serial("DISARM"))
        disable_button.clicked.connect(lambda: self.send_serial("DISABLE"))
        status_button.clicked.connect(lambda: self.send_serial("STATUS"))

        for button in (
            send_button,
            zero_button,
            walk_button,
            arm_button,
            hold_button,
            disarm_button,
            disable_button,
            status_button,
        ):
            header.addWidget(button)
        header.addStretch(1)
        header.addWidget(self.live_checkbox)
        header.addWidget(self.hardware_label)
        main_layout.addLayout(header)
        main_layout.addWidget(self.status_label)

        group = QGroupBox("Joint Commands")
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Command"), 0, 1)
        grid.addWidget(QLabel("Radians"), 0, 2)
        grid.addWidget(QLabel("Degrees"), 0, 3)
        grid.addWidget(QLabel("Limit"), 0, 4)

        for index, name in enumerate(JOINT_NAMES):
            row = JointRow(index, name, JOINT_LIMITS[index], self.values_changed)
            self.rows.append(row)
            grid.addWidget(row.name_label, index + 1, 0)
            grid.addWidget(row.slider, index + 1, 1)
            grid.addWidget(row.spin, index + 1, 2)
            grid.addWidget(row.deg_label, index + 1, 3)
            grid.addWidget(row.limit_label, index + 1, 4)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(group)
        scroll.setMinimumHeight(360)
        main_layout.addWidget(scroll, 1)

        footer = QLabel(
            "Publishes /volt/joint_commands/calibration through the command router. "
            "Arduino moves only when volt_serial_bridge is connected and firmware is armed."
        )
        footer.setWordWrap(True)
        main_layout.addWidget(footer)

        self.setCentralWidget(root)

    def apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #101820;
                color: #e5edf5;
                font-size: 13px;
            }
            QGroupBox {
                border: 1px solid #2d3a46;
                border-radius: 6px;
                margin-top: 10px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QPushButton {
                background: #1f6feb;
                border: 0;
                border-radius: 5px;
                color: white;
                padding: 8px 10px;
            }
            QPushButton:hover {
                background: #2f81f7;
            }
            QPushButton:pressed {
                background: #174ea6;
            }
            QDoubleSpinBox {
                background: #0b1118;
                border: 1px solid #304050;
                border-radius: 4px;
                padding: 4px;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #2d3a46;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #58a6ff;
                width: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }
            QCheckBox {
                spacing: 6px;
            }
            QLabel#hardwareState {
                color: #ffd166;
                font-weight: 700;
                padding: 2px 8px;
            }
            """
        )

    def values(self):
        return [row.value() for row in self.rows]

    def set_pose(self, values, publish=True):
        for row, value in zip(self.rows, values):
            row.set_value(value, emit=False)
        self.values_changed()
        if publish:
            self.publish_once()

    def values_changed(self):
        self.last_values = self.values()

    def publish_once(self):
        self.values_changed()
        self.ros_node.publish_joints(self.last_values)

    def publish_if_live(self):
        if self.live_checkbox.isChecked():
            self.publish_once()

    def zero_pose(self):
        self.set_pose([0.0 for _ in JOINT_NAMES])

    def walk_ready_pose(self):
        self.set_pose(WALK_POSE)

    def send_serial(self, command):
        self.ros_node.publish_serial_command(command)

    def status_callback(self, message):
        self.status_text = message.data
        self.status_label.setText(message.data)
        dry_run = "dry_run=1" in message.data
        hardware_enabled = "hardware_enabled=1" in message.data
        connected = "connected=1" in message.data
        armed = "armed=1" in message.data
        if dry_run or not hardware_enabled:
            self.hardware_label.setText("Hardware: DRY-RUN")
        elif not connected:
            self.hardware_label.setText("Hardware: not connected")
        elif armed:
            self.hardware_label.setText("Hardware: ARMED")
        else:
            self.hardware_label.setText("Hardware: connected, disarmed")

    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def closeEvent(self, event):
        self.shutting_down = True
        self.live_checkbox.setChecked(False)
        self.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rclpy.init()
    app = QApplication(sys.argv)
    window = JointTestWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
