#!/usr/bin/env python3

import math
import signal
import sys

import rclpy
import yaml
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from volt_kinematics import JOINT_NAMES
from volt_servo_calibration import ServoCalibrationTable, ordered_positions_from_joint_state


class CalibrationNode(Node):
    def __init__(self, status_callback, joint_state_callback):
        super().__init__("volt_servo_calibration_gui")
        self.declare_parameter("calibration_file", "")
        self.calibration_file = str(self.get_parameter("calibration_file").value)
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            "/volt/joint_commands/calibration",
            10,
        )
        self.owner_publisher = self.create_publisher(String, "/volt/command_owner", 10)
        self.serial_publisher = self.create_publisher(String, "/volt/serial_command", 10)
        self.create_subscription(String, "/volt/serial_status", status_callback, 10)
        self.create_subscription(JointState, "/joint_states", joint_state_callback, 20)

    def publish_owner(self, owner):
        message = String()
        message.data = owner
        self.owner_publisher.publish(message)

    def publish_serial(self, command):
        message = String()
        message.data = command
        self.serial_publisher.publish(message)

    def publish_pose(self, pose):
        self.publish_owner("CALIBRATION")
        message = Float64MultiArray()
        message.data = list(pose)
        self.command_publisher.publish(message)


class ServoCalibrationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VOLT Servo Calibration")
        self.resize(1020, 700)
        self.node = CalibrationNode(self.status_callback, self.joint_state_callback)
        self.node.publish_owner("CALIBRATION")
        self.calibration_file = self.node.calibration_file
        self.calibration_raw = None
        self.table = None
        self.current_pose = [0.0 for _ in JOINT_NAMES]
        self.status_label = QLabel("serial: unknown")

        self.load_calibration()
        self.build_ui()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin_ros)
        self.spin_timer.start(10)

    def load_calibration(self):
        with open(self.calibration_file, "r", encoding="utf-8") as handle:
            self.calibration_raw = yaml.safe_load(handle)
        self.table = ServoCalibrationTable.from_dict(self.calibration_raw)

    def save_calibration(self):
        with open(self.calibration_file, "w", encoding="utf-8") as handle:
            yaml.safe_dump(self.calibration_raw, handle, sort_keys=False)
        self.load_calibration()
        self.refresh_joint_fields()

    def build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)
        warn = QLabel("Suspend the robot. GUI startup and channel selection do not move servos.")
        warn.setStyleSheet("font-weight: bold; color: #ffd166;")
        layout.addWidget(warn)
        layout.addWidget(self.status_label)
        tabs = QTabWidget()
        tabs.addTab(self.build_channel_tab(), "Physical Channel")
        tabs.addTab(self.build_joint_tab(), "ROS Joint Calibration")
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(tabs)
        layout.addWidget(scroll, 1)
        self.setCentralWidget(root)

    def build_safety_buttons(self):
        row = QHBoxLayout()
        for label, command in (
            ("ARM", "ARM"),
            ("HOLD", "HOLD"),
            ("DISARM", "DISARM"),
            ("DISABLE", "DISABLE"),
            ("STATUS", "STATUS"),
        ):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, text=command: self.node.publish_serial(text))
            row.addWidget(button)
        row.addStretch(1)
        return row

    def build_channel_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addLayout(self.build_safety_buttons())

        form = QFormLayout()
        self.channel_spin = QSpinBox()
        self.channel_spin.setRange(0, 11)
        self.channel_angle = QDoubleSpinBox()
        self.channel_angle.setRange(0.0, 180.0)
        self.channel_angle.setDecimals(2)
        self.channel_angle.setSingleStep(1.0)
        self.channel_angle.setValue(90.0)
        self.channel_joint_assign = QComboBox()
        self.channel_joint_assign.addItems(JOINT_NAMES)
        form.addRow("PCA channel", self.channel_spin)
        form.addRow("Physical degrees", self.channel_angle)
        form.addRow("Assign selected channel to joint", self.channel_joint_assign)
        layout.addLayout(form)

        buttons = QHBoxLayout()
        for delta in (-5.0, -1.0, 1.0, 5.0):
            button = QPushButton("%+g deg" % delta)
            button.clicked.connect(lambda _checked=False, amount=delta: self.bump_channel(amount))
            buttons.addWidget(button)
        send = QPushButton("Send SERVO")
        send.clicked.connect(self.send_servo)
        save = QPushButton("Save Mapping")
        save.clicked.connect(self.save_channel_mapping)
        buttons.addWidget(send)
        buttons.addWidget(save)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        return widget

    def build_joint_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.joint_combo = QComboBox()
        self.joint_combo.addItems(JOINT_NAMES)
        self.joint_combo.currentIndexChanged.connect(self.refresh_joint_fields)
        layout.addWidget(self.joint_combo)

        group = QGroupBox("Selected Joint")
        form = QFormLayout(group)
        self.channel_field = QSpinBox()
        self.channel_field.setRange(0, 11)
        self.direction_field = QComboBox()
        self.direction_field.addItems(["1", "-1"])
        self.neutral_field = QDoubleSpinBox()
        self.trim_field = QDoubleSpinBox()
        self.min_field = QDoubleSpinBox()
        self.max_field = QDoubleSpinBox()
        for field in (self.neutral_field, self.trim_field, self.min_field, self.max_field):
            field.setRange(-360.0, 360.0)
            field.setDecimals(2)
        self.calculated_label = QLabel("")
        form.addRow("PCA channel", self.channel_field)
        form.addRow("Direction", self.direction_field)
        form.addRow("Neutral deg", self.neutral_field)
        form.addRow("Trim deg", self.trim_field)
        form.addRow("Min deg", self.min_field)
        form.addRow("Max deg", self.max_field)
        form.addRow("Calculated physical", self.calculated_label)
        layout.addWidget(group)

        buttons = QHBoxLayout()
        for label, rad in (("Test -0.05 rad", -0.05), ("Test zero", 0.0), ("Test +0.05 rad", 0.05)):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, value=rad: self.test_joint(value))
            buttons.addWidget(button)
        save = QPushButton("Save Calibration")
        save.clicked.connect(self.save_joint_calibration)
        reload_button = QPushButton("Reload")
        reload_button.clicked.connect(self.reload_clicked)
        buttons.addWidget(save)
        buttons.addWidget(reload_button)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        self.refresh_joint_fields()
        return widget

    def bump_channel(self, delta):
        self.channel_angle.setValue(self.channel_angle.value() + delta)

    def send_servo(self):
        self.node.publish_serial(
            "SERVO %d %.2f" % (self.channel_spin.value(), self.channel_angle.value())
        )

    def save_channel_mapping(self):
        joint = self.channel_joint_assign.currentText()
        self.calibration_raw["servos"][joint]["pca_channel"] = int(self.channel_spin.value())
        self.save_calibration()

    def selected_joint(self):
        return self.joint_combo.currentText()

    def refresh_joint_fields(self):
        if not hasattr(self, "joint_combo"):
            return
        servo = self.calibration_raw["servos"][self.selected_joint()]
        self.channel_field.setValue(int(servo["pca_channel"]))
        self.direction_field.setCurrentText(str(servo["direction"]))
        self.neutral_field.setValue(float(servo["neutral_deg"]))
        self.trim_field.setValue(float(servo.get("trim_deg", 0.0)))
        self.min_field.setValue(float(servo["min_deg"]))
        self.max_field.setValue(float(servo["max_deg"]))
        self.update_calculated_label(0.0)

    def update_calculated_label(self, rad):
        joint = self.selected_joint()
        try:
            deg = self.table.ros_radians_to_servo_degrees(joint, rad)
            self.calculated_label.setText("at %.3f rad -> %.2f deg" % (rad, deg))
        except Exception as exc:
            self.calculated_label.setText(str(exc))

    def save_joint_calibration(self):
        joint = self.selected_joint()
        servo = self.calibration_raw["servos"][joint]
        servo["pca_channel"] = int(self.channel_field.value())
        servo["direction"] = int(self.direction_field.currentText())
        servo["neutral_deg"] = float(self.neutral_field.value())
        servo["trim_deg"] = float(self.trim_field.value())
        servo["min_deg"] = float(self.min_field.value())
        servo["max_deg"] = float(self.max_field.value())
        self.save_calibration()

    def reload_clicked(self):
        self.load_calibration()
        self.refresh_joint_fields()

    def test_joint(self, rad):
        pose = list(self.current_pose)
        pose[self.joint_combo.currentIndex()] = rad
        self.node.publish_pose(pose)
        self.update_calculated_label(rad)

    def status_callback(self, message):
        self.status_label.setText(message.data)

    def joint_state_callback(self, message):
        try:
            self.current_pose = ordered_positions_from_joint_state(message)
        except Exception:
            return

    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def closeEvent(self, event):
        self.node.publish_owner("HOLD")
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rclpy.init()
    app = QApplication(sys.argv)
    window = ServoCalibrationWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
