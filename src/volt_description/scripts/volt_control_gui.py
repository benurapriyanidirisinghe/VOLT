#!/usr/bin/env python3

import json
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from PyQt5.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from std_msgs.msg import String


GAIT_LIMITS = {
    "walk": (0.058, 0.028, 0.32),
    "amble": (0.090, 0.035, 0.50),
    "slow_trot": (0.058, 0.025, 0.40),
    "normal_trot": (0.085, 0.035, 0.65),
    "fast_trot": (0.105, 0.045, 0.75),
    "trot": (0.085, 0.035, 0.65),
}


class Joystick(QWidget):
    changed = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(250, 250)
        self.setFocusPolicy(Qt.StrongFocus)
        self.vector = QPointF(0.0, 0.0)
        self.dragging = False

    def set_vector(self, forward, horizontal):
        magnitude = math.hypot(forward, horizontal)
        if magnitude > 1.0:
            forward /= magnitude
            horizontal /= magnitude
        self.vector = QPointF(horizontal, -forward)
        self.changed.emit(forward, horizontal)
        self.update()

    def update_from_mouse(self, position):
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = max(1.0, min(self.width(), self.height()) * 0.36)
        dx = (position.x() - center.x()) / radius
        dy = (position.y() - center.y()) / radius
        magnitude = math.hypot(dx, dy)
        if magnitude > 1.0:
            dx /= magnitude
            dy /= magnitude
        self.vector = QPointF(dx, dy)
        self.changed.emit(-dy, dx)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.update_from_mouse(event.localPos())

    def mouseMoveEvent(self, event):
        if self.dragging:
            self.update_from_mouse(event.localPos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = False
            self.set_vector(0.0, 0.0)

    def keyPressEvent(self, event):
        keys = {
            Qt.Key_W: (1.0, 0.0),
            Qt.Key_S: (-1.0, 0.0),
            Qt.Key_A: (0.0, -1.0),
            Qt.Key_D: (0.0, 1.0),
        }
        if event.key() in keys:
            self.set_vector(*keys[event.key()])
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat() and event.key() in (
            Qt.Key_W,
            Qt.Key_A,
            Qt.Key_S,
            Qt.Key_D,
        ):
            self.set_vector(0.0, 0.0)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = min(self.width(), self.height()) * 0.36

        painter.setPen(QPen(QColor("#3a4658"), 2))
        painter.setBrush(QColor("#17202d"))
        painter.drawEllipse(center, radius, radius)

        painter.setPen(QPen(QColor("#344054"), 1))
        painter.drawLine(
            QPointF(center.x() - radius, center.y()),
            QPointF(center.x() + radius, center.y()),
        )
        painter.drawLine(
            QPointF(center.x(), center.y() - radius),
            QPointF(center.x(), center.y() + radius),
        )

        knob = QPointF(
            center.x() + self.vector.x() * radius,
            center.y() + self.vector.y() * radius,
        )
        painter.setPen(QPen(QColor("#7dd3fc"), 3))
        painter.setBrush(QColor("#0ea5e9"))
        painter.drawEllipse(knob, 23, 23)


class VoltGuiNode(Node):
    def __init__(self, status_callback):
        super().__init__("volt_control_gui")
        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.action_publisher = self.create_publisher(String, "/volt/action", 10)
        self.gait_publisher = self.create_publisher(String, "/volt/gait", 10)
        self.pose_publisher = self.create_publisher(
            Twist,
            "/volt/body_pose",
            10,
        )
        self.create_subscription(String, "/volt/status", status_callback, 10)

    def publish_text(self, publisher, text):
        message = String()
        message.data = text
        publisher.publish(message)


class VoltControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VOLT Motion Control")
        self.resize(920, 620)

        self.ros_node = VoltGuiNode(self.status_callback)
        self.forward = 0.0
        self.horizontal = 0.0
        self.current_gait = "walk"

        self.build_ui()
        self.apply_style()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin_ros)
        self.spin_timer.start(10)

        self.command_timer = QTimer(self)
        self.command_timer.timeout.connect(self.publish_motion)
        self.command_timer.start(50)

        self.pose_timer = QTimer(self)
        self.pose_timer.timeout.connect(self.publish_pose)
        self.pose_timer.start(100)

        QTimer.singleShot(300, lambda: self.select_gait("walk"))

    def build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)
        outer.setContentsMargins(20, 20, 20, 20)
        outer.setSpacing(18)

        left = QVBoxLayout()
        title = QLabel("VOLT")
        title.setObjectName("title")
        subtitle = QLabel("Quadruped motion console")
        subtitle.setObjectName("subtitle")
        left.addWidget(title)
        left.addWidget(subtitle)

        state_group = QGroupBox("Robot State")
        state_layout = QGridLayout(state_group)
        self.state_label = QLabel("WAITING")
        self.state_label.setObjectName("state")
        self.status_detail = QLabel("Waiting for controller status")
        self.status_detail.setWordWrap(True)
        state_layout.addWidget(self.state_label, 0, 0, 1, 3)
        state_layout.addWidget(self.status_detail, 1, 0, 1, 3)

        stand_button = QPushButton("STAND")
        stand_button.clicked.connect(lambda: self.send_action("stand"))
        sit_button = QPushButton("SIT")
        sit_button.clicked.connect(lambda: self.send_action("sit"))
        stop_button = QPushButton("STOP")
        stop_button.setObjectName("stop")
        stop_button.clicked.connect(lambda: self.send_action("stop"))
        state_layout.addWidget(stand_button, 2, 0)
        state_layout.addWidget(sit_button, 2, 1)
        state_layout.addWidget(stop_button, 2, 2)
        left.addWidget(state_group)

        gait_group = QGroupBox("Gait")
        gait_layout = QGridLayout(gait_group)
        self.gait_buttons = QButtonGroup(self)
        self.gait_buttons.setExclusive(True)
        gait_choices = (
            ("walk", "WALK"),
            ("amble", "AMBLE"),
            ("slow_trot", "SLOW TROT"),
            ("normal_trot", "NORMAL TROT"),
            ("fast_trot", "FAST TROT"),
        )
        for index, (gait, label) in enumerate(gait_choices):
            button = QPushButton(label)
            button.setCheckable(True)
            button.clicked.connect(
                lambda checked, name=gait: self.select_gait(name)
            )
            self.gait_buttons.addButton(button)
            gait_layout.addWidget(button, index // 3, index % 3)
            if gait == "walk":
                button.setChecked(True)

        self.step_button = QPushButton("STEP IN PLACE")
        self.step_button.setCheckable(True)
        self.step_button.clicked.connect(lambda: self.send_action("step"))
        gait_layout.addWidget(self.step_button, 2, 0, 1, 3)
        left.addWidget(gait_group)

        speed_group = QGroupBox("Motion")
        speed_layout = QFormLayout(speed_group)
        self.drive_mode = QComboBox()
        self.drive_mode.addItems(["Normal steering", "Crab / omnidirectional"])
        speed_layout.addRow("Drive mode", self.drive_mode)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(60)
        speed_layout.addRow("Speed", self.speed_slider)

        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setRange(-100, 100)
        self.yaw_slider.setValue(0)
        self.yaw_slider.setTickPosition(QSlider.TicksBelow)
        self.yaw_slider.setTickInterval(50)
        speed_layout.addRow("Yaw trim", self.yaw_slider)
        left.addWidget(speed_group)
        left.addStretch(1)
        outer.addLayout(left, 4)

        center = QVBoxLayout()
        joystick_group = QGroupBox("Direction")
        joystick_layout = QVBoxLayout(joystick_group)
        self.joystick = Joystick()
        self.joystick.changed.connect(self.joystick_changed)
        joystick_layout.addWidget(self.joystick, alignment=Qt.AlignCenter)
        hint = QLabel(
            "Drag the pad or use W A S D. Release to stop.\n"
            "Normal: left/right turns. Crab: left/right translates."
        )
        hint.setAlignment(Qt.AlignCenter)
        hint.setObjectName("hint")
        joystick_layout.addWidget(hint)
        center.addWidget(joystick_group)
        outer.addLayout(center, 5)

        right = QVBoxLayout()
        pose_group = QGroupBox("Body Pose")
        pose_layout = QFormLayout(pose_group)
        self.height = self.make_spinbox(0.175, 0.220, 0.200, 0.001, 3)
        self.body_x = self.make_spinbox(-0.025, 0.025, 0.0, 0.002, 3)
        self.body_y = self.make_spinbox(-0.020, 0.020, 0.0, 0.002, 3)
        self.roll = self.make_spinbox(-9.0, 9.0, 0.0, 1.0, 1)
        self.pitch = self.make_spinbox(-9.0, 9.0, 0.0, 1.0, 1)
        self.yaw = self.make_spinbox(-10.0, 10.0, 0.0, 1.0, 1)
        pose_layout.addRow("Height (m)", self.height)
        pose_layout.addRow("Body X (m)", self.body_x)
        pose_layout.addRow("Body Y (m)", self.body_y)
        pose_layout.addRow("Roll (deg)", self.roll)
        pose_layout.addRow("Pitch (deg)", self.pitch)
        pose_layout.addRow("Yaw (deg)", self.yaw)

        reset_pose = QPushButton("RESET BODY POSE")
        reset_pose.clicked.connect(self.reset_body_pose)
        pose_layout.addRow(reset_pose)
        right.addWidget(pose_group)

        safety = QFrame()
        safety.setObjectName("safety")
        safety_layout = QVBoxLayout(safety)
        safety_title = QLabel("Command safety")
        safety_title.setObjectName("safetyTitle")
        safety_text = QLabel(
            "Velocity stops automatically if GUI messages are lost.\n"
            "Use STOP before switching between tests."
        )
        safety_text.setWordWrap(True)
        safety_layout.addWidget(safety_title)
        safety_layout.addWidget(safety_text)
        right.addWidget(safety)
        right.addStretch(1)
        outer.addLayout(right, 4)

    def make_spinbox(self, minimum, maximum, value, step, decimals):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setValue(value)
        box.setSingleStep(step)
        box.setDecimals(decimals)
        return box

    def apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #0b111b;
                color: #dce7f5;
                font-family: "DejaVu Sans";
                font-size: 13px;
            }
            QLabel#title {
                color: #7dd3fc;
                font-size: 34px;
                font-weight: 800;
                letter-spacing: 4px;
            }
            QLabel#subtitle, QLabel#hint {
                color: #8090a6;
            }
            QLabel#state {
                color: #67e8f9;
                font-size: 20px;
                font-weight: 700;
            }
            QGroupBox {
                border: 1px solid #263449;
                border-radius: 10px;
                margin-top: 12px;
                padding: 12px;
                font-weight: 700;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
                color: #94a3b8;
            }
            QPushButton {
                background: #172235;
                border: 1px solid #34445c;
                border-radius: 7px;
                padding: 10px;
                font-weight: 700;
            }
            QPushButton:hover {
                border-color: #38bdf8;
                background: #1d3047;
            }
            QPushButton:checked {
                background: #0369a1;
                border-color: #7dd3fc;
            }
            QPushButton#stop {
                background: #7f1d1d;
                border-color: #ef4444;
            }
            QComboBox, QDoubleSpinBox {
                background: #111a29;
                border: 1px solid #34445c;
                border-radius: 5px;
                padding: 6px;
            }
            QFrame#safety {
                background: #111a29;
                border: 1px solid #334155;
                border-radius: 9px;
            }
            QLabel#safetyTitle {
                color: #fbbf24;
                font-weight: 700;
            }
            """
        )

    def joystick_changed(self, forward, horizontal):
        self.forward = forward
        self.horizontal = horizontal

    def select_gait(self, gait):
        self.current_gait = gait
        self.ros_node.publish_text(self.ros_node.gait_publisher, gait)

    def send_action(self, action):
        if action in ("stop", "sit"):
            self.joystick.set_vector(0.0, 0.0)
            self.yaw_slider.setValue(0)
        self.ros_node.publish_text(self.ros_node.action_publisher, action)

    def publish_motion(self):
        speed = self.speed_slider.value() / 100.0
        max_x, max_y, max_yaw = GAIT_LIMITS[self.current_gait]
        message = Twist()
        message.linear.x = self.forward * max_x * speed

        if self.drive_mode.currentIndex() == 0:
            message.angular.z = -self.horizontal * max_yaw * speed
        else:
            message.linear.y = -self.horizontal * max_y * speed
            message.angular.z = (
                self.yaw_slider.value() / 100.0 * max_yaw * speed
            )
        self.ros_node.velocity_publisher.publish(message)

    def publish_pose(self):
        message = Twist()
        message.linear.x = self.body_x.value()
        message.linear.y = self.body_y.value()
        message.linear.z = self.height.value()
        message.angular.x = math.radians(self.roll.value())
        message.angular.y = math.radians(self.pitch.value())
        message.angular.z = math.radians(self.yaw.value())
        self.ros_node.pose_publisher.publish(message)

    def reset_body_pose(self):
        self.height.setValue(0.200)
        self.body_x.setValue(0.0)
        self.body_y.setValue(0.0)
        self.roll.setValue(0.0)
        self.pitch.setValue(0.0)
        self.yaw.setValue(0.0)

    def status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = str(status.get("state", "unknown")).replace("_", " ").upper()
        self.state_label.setText(state)
        connected = "connected" if status.get("controller_connected") else "offline"
        gait = str(status.get("gait", "unknown")).replace("_", " ").upper()
        error = status.get("joint_error")
        detail = "%s controller | %s gait" % (connected, gait)
        if error is not None:
            detail += " | joint error %.3f rad" % float(error)
        warning = status.get("warning")
        if warning:
            detail += "\nWARNING: " + str(warning)
            self.status_detail.setStyleSheet("color: #fca5a5;")
        else:
            self.status_detail.setStyleSheet("color: #94a3b8;")
        self.status_detail.setText(detail)
        self.step_button.setChecked(bool(status.get("step_in_place")))

    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def closeEvent(self, event):
        self.send_action("stop")
        self.publish_motion()
        self.ros_node.destroy_node()
        event.accept()


def main():
    rclpy.init()
    application = QApplication(sys.argv)
    application.setFont(QFont("DejaVu Sans", 10))
    window = VoltControlWindow()
    window.show()
    exit_code = application.exec_()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
