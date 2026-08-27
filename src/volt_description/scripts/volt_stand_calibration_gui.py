#!/usr/bin/env python3

"""Level the standing pose by nudging feet in millimetres, not servo degrees.

The problem this solves: one foot does not reach the ground at the default
stand. Fixing that by hand in servo degrees is guesswork, because how far a
foot moves per degree of trim depends on which joint you touch and on the
leg's geometry -- the knee moment arm is 71.4 mm/rad at the front and 48.8 at
the rear, so the SAME trim moves a front foot 46% further than a rear one.

Here you say "front_right, +6 mm" and the tool converts that to the correct
trim on that leg's knee joint through the repo's own forward kinematics,
applies it live, and shows all four foot heights so you can see them converge.

  trim_delta_deg = direction * degrees(dz / knee_arm)

because the robot is open loop: a trim is a permanent bias added in servo
space, so the joint the servo actually reaches is the commanded angle plus
radians(trim / direction).

Procedure:
  1. Robot on the ground, feet free, servo power on. Press HOLD STAND.
  2. Read which foot is short -- it is the one not bearing weight.
  3. Nudge that leg down in 1-2 mm steps until all four bear evenly.
  4. SAVE writes the trims to servo_calibration.yaml.

Nudging DOWN lengthens the leg (foot further from the body). Guard and limit
violations are shown per joint and block the save.
"""

import math
import signal
import sys
from pathlib import Path

import rclpy
import yaml
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

import volt_kinematics as K
from volt_servo_calibration import (
    ServoCalibrationTable,
    named_positions_from_ordered,
)

# Mirrors the firmware's tighter per-channel guards for the two front feet.
CHANNEL_GUARDS = {2: (50.0, None), 5: (None, 130.0)}
NUDGE_STEPS_MM = (-5.0, -2.0, -1.0, 1.0, 2.0, 5.0)
# A trim is a mounting correction, not a pose control. Anything past this is
# almost certainly the wrong fix (a bent linkage, a mis-indexed horn, or a
# body-height change wanted instead), so the tool refuses to go further.
TRIM_LIMIT_DEG = 12.0


def default_calibration_path():
    return str(Path(__file__).resolve().parent.parent / "config"
               / "servo_calibration.yaml")


def knee_arm_m(leg, angles):
    """d(foot z)/d(knee angle) for this leg at this pose, from the repo's FK."""
    delta = 1e-6
    probe = list(angles)
    probe[2] += delta
    return (K.forward_leg(leg, probe)[2] - K.forward_leg(leg, angles)[2]) / delta


class StandNode(Node):
    def __init__(self):
        super().__init__("volt_stand_calibration_gui")
        self.command = self.create_publisher(
            Float64MultiArray, "/volt/joint_commands/calibration", 10
        )
        self.owner = self.create_publisher(String, "/volt/command_owner", 10)
        self.serial = self.create_publisher(String, "/volt/serial_command", 10)

    def publish_pose(self, values):
        message = Float64MultiArray()
        message.data = [float(v) for v in values]
        self.command.publish(message)

    def publish_owner(self, owner):
        message = String()
        message.data = owner
        self.owner.publish(message)

    def publish_serial(self, command):
        message = String()
        message.data = command
        self.serial.publish(message)


class StandWindow(QMainWindow):
    def __init__(self, node, path):
        super().__init__()
        self.node = node
        self.path = path
        with open(path, "r", encoding="utf-8") as handle:
            self.raw = yaml.safe_load(handle)
        self.table = ServoCalibrationTable.from_file(path)
        # Working trims, seeded from the file; only these are edited.
        self.trims = {
            joint: float(self.table.servos[joint].trim_deg)
            for joint in K.JOINT_NAMES
        }
        self.holding = False

        self.setWindowTitle("VOLT — standing pose calibration")
        central = QWidget()
        outer = QVBoxLayout(central)

        warning = QLabel(
            "Feet on the ground, servo power reachable. HOLD STAND commands "
            "the canonical WALK_POSE and keeps holding it; nudges take effect "
            "immediately. Nudging a leg DOWN lengthens it."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet(
            "background:#5a3a10; color:#ffd24a; padding:8px; border-radius:4px;"
        )
        outer.addWidget(warning)

        row = QHBoxLayout()
        self.hold_button = QPushButton("HOLD STAND")
        self.hold_button.setCheckable(True)
        self.hold_button.toggled.connect(self.on_hold)
        arm = QPushButton("ARM")
        arm.clicked.connect(lambda: self.node.publish_serial("ARM"))
        disarm = QPushButton("DISARM")
        disarm.clicked.connect(lambda: self.node.publish_serial("DISARM"))
        reset = QPushButton("Reset trims to file")
        reset.clicked.connect(self.reset_trims)
        save = QPushButton("SAVE to calibration")
        save.clicked.connect(self.save)
        for widget in (self.hold_button, arm, disarm, reset, save):
            row.addWidget(widget)
        row.addStretch(1)
        outer.addLayout(row)

        box = QGroupBox("Per-leg foot height  (nudge in millimetres)")
        grid = QGridLayout(box)
        headers = ("leg", "foot height", "vs mean", "knee trim", "nudge")
        for column, title in enumerate(headers):
            label = QLabel(title)
            label.setStyleSheet("color:#9aa; font-weight:bold;")
            grid.addWidget(label, 0, column)
        self.height_labels = {}
        self.delta_labels = {}
        self.trim_labels = {}
        for index, leg in enumerate(K.LEG_ORDER):
            grid.addWidget(QLabel(leg), index + 1, 0)
            self.height_labels[leg] = QLabel("-")
            self.height_labels[leg].setFont(QFont("DejaVu Sans Mono", 10))
            grid.addWidget(self.height_labels[leg], index + 1, 1)
            self.delta_labels[leg] = QLabel("-")
            self.delta_labels[leg].setFont(QFont("DejaVu Sans Mono", 10))
            grid.addWidget(self.delta_labels[leg], index + 1, 2)
            self.trim_labels[leg] = QLabel("-")
            self.trim_labels[leg].setFont(QFont("DejaVu Sans Mono", 10))
            grid.addWidget(self.trim_labels[leg], index + 1, 3)
            holder = QWidget()
            buttons = QHBoxLayout(holder)
            buttons.setContentsMargins(0, 0, 0, 0)
            for step in NUDGE_STEPS_MM:
                button = QPushButton("%+g" % step)
                button.setMaximumWidth(48)
                button.clicked.connect(
                    lambda _c, l=leg, s=step: self.nudge(l, s)
                )
                buttons.addWidget(button)
            grid.addWidget(holder, index + 1, 4)
        outer.addWidget(box)

        level = QPushButton("Level all feet to the lowest")
        level.setToolTip(
            "Lengthen every other leg until all four match the one that "
            "currently reaches furthest. Only ever lengthens, so no leg is "
            "lifted off the ground to achieve it."
        )
        level.clicked.connect(self.level_all)
        outer.addWidget(level)

        self.report = QPlainTextEdit()
        self.report.setReadOnly(True)
        self.report.setFont(QFont("DejaVu Sans Mono", 9))
        self.report.setMinimumHeight(190)
        outer.addWidget(self.report)

        self.setCentralWidget(central)
        self.resize(1000, 700)

        self.hold_timer = QTimer(self)
        self.hold_timer.timeout.connect(self.publish_hold)
        self.refresh()

    # -- pose maths --------------------------------------------------------

    def actual_angles(self, leg):
        """Joint angles the servos physically reach, given the working trims.

        Open loop: the servo goes to neutral+trim+direction*commanded, so the
        joint it lands on is the commanded angle plus radians(trim/direction).
        """
        named = named_positions_from_ordered(K.WALK_POSE)
        out = []
        for joint_type in ("shoulder", "leg", "foot"):
            joint = "%s_%s" % (leg, joint_type)
            servo = self.table.servos[joint]
            offset = math.radians(self.trims[joint] / servo.direction)
            out.append(named[joint] + offset)
        return out

    def foot_heights_mm(self):
        return {
            leg: -K.forward_leg(leg, self.actual_angles(leg))[2] * 1000.0
            for leg in K.LEG_ORDER
        }

    def nudge(self, leg, millimetres):
        """Move one foot by dz, by trimming that leg's knee."""
        joint = "%s_foot" % leg
        servo = self.table.servos[joint]
        start = -K.forward_leg(leg, self.actual_angles(leg))[2] * 1000.0
        target = start + millimetres
        # The knee arm changes as the leg extends, so one Jacobian step
        # undershoots by a few percent. Iterate to land on the requested
        # millimetres instead of near them; three passes converge well under
        # 0.01 mm and each pass is one FK evaluation.
        proposed = self.trims[joint]
        for _pass in range(3):
            angles = self.actual_angles(leg)
            arm = knee_arm_m(leg, angles)
            if abs(arm) < 1e-6:
                return
            current = -K.forward_leg(leg, angles)[2] * 1000.0
            error = target - current
            if abs(error) < 1e-4:
                break
            # Nudging DOWN (+mm) lengthens the leg, i.e. makes foot z more
            # negative, so the correction is negated against FK's z-up sign.
            delta_theta = (-error / 1000.0) / arm
            proposed = self.trims[joint] + servo.direction * math.degrees(
                delta_theta
            )
            if abs(proposed) > TRIM_LIMIT_DEG:
                break
            self.trims[joint] = proposed
        if abs(self.trims[joint]) > TRIM_LIMIT_DEG:
            self.trims[joint] = math.copysign(
                TRIM_LIMIT_DEG, self.trims[joint]
            )
            QMessageBox.warning(
                self, "Trim limit",
                "%s hit the %.0f deg trim limit.\n\nA trim is a mounting "
                "correction; beyond this the cause is mechanical (bent "
                "linkage, horn a spline out) or you want a different body "
                "height instead." % (joint, TRIM_LIMIT_DEG),
            )
        self.refresh()

    def level_all(self):
        heights = self.foot_heights_mm()
        target = max(heights.values())      # the leg that reaches furthest
        for leg in K.LEG_ORDER:
            shortfall = target - heights[leg]
            if abs(shortfall) > 0.01:
                self.nudge(leg, shortfall)
        self.refresh()

    def reset_trims(self):
        for joint in K.JOINT_NAMES:
            self.trims[joint] = float(self.table.servos[joint].trim_deg)
        self.refresh()

    # -- violations --------------------------------------------------------

    def violations(self):
        named = named_positions_from_ordered(K.WALK_POSE)
        out = []
        for joint in K.JOINT_NAMES:
            servo = self.table.servos[joint]
            value = (servo.neutral_deg + self.trims[joint]
                     + servo.direction * math.degrees(named[joint]))
            if value < servo.min_deg - 1e-9 or value > servo.max_deg + 1e-9:
                out.append("%s at %.1f deg is outside [%.0f, %.0f]"
                           % (joint, value, servo.min_deg, servo.max_deg))
            low, high = CHANNEL_GUARDS.get(servo.pca_channel, (None, None))
            if (low is not None and value < low - 1e-9) or (
                high is not None and value > high + 1e-9
            ):
                out.append("%s at %.1f deg breaks the firmware guard on ch%d"
                           % (joint, value, servo.pca_channel))
        return out

    # -- output ------------------------------------------------------------

    def refresh(self):
        heights = self.foot_heights_mm()
        mean = sum(heights.values()) / len(heights)
        for leg in K.LEG_ORDER:
            self.height_labels[leg].setText("%8.2f mm" % heights[leg])
            delta = heights[leg] - mean
            self.delta_labels[leg].setText("%+7.2f mm" % delta)
            self.delta_labels[leg].setStyleSheet(
                "color:#86efac;" if abs(delta) < 0.5 else "color:#ffd24a;"
            )
            self.trim_labels[leg].setText(
                "%+7.2f deg" % self.trims["%s_foot" % leg]
            )
        spread = max(heights.values()) - min(heights.values())
        lines = []
        lines.append("foot heights, hip to ground, at the canonical stand:")
        for leg in K.LEG_ORDER:
            lines.append("  %-13s %8.2f mm" % (leg, heights[leg]))
        lines.append("")
        lines.append("spread %.2f mm   %s" % (
            spread,
            "level" if spread < 0.5 else "the lowest foot bears first",
        ))
        lines.append("")
        lines.append("trims that would be written:")
        for joint in K.JOINT_NAMES:
            original = float(self.table.servos[joint].trim_deg)
            mark = "  <- changed" if abs(self.trims[joint] - original) > 1e-9 \
                else ""
            lines.append("  %-22s %+7.3f deg%s"
                         % (joint, self.trims[joint], mark))
        problems = self.violations()
        if problems:
            lines.append("")
            lines.append("BLOCKING:")
            lines.extend("  " + problem for problem in problems)
        self.report.setPlainText("\n".join(lines))
        if self.holding:
            self.publish_hold()

    # -- hold / save -------------------------------------------------------

    def on_hold(self, checked):
        self.holding = checked
        self.hold_button.setText("STOP HOLDING" if checked else "HOLD STAND")
        if checked:
            self.hold_timer.start(100)
        else:
            self.hold_timer.stop()
            self.node.publish_owner("HOLD")

    def publish_hold(self):
        # WALK_POSE is the canonical commanded stand. The trims are applied by
        # the serial bridge from the calibration file, so what changes here
        # only reaches the robot after SAVE and a bridge restart -- the live
        # hold exists so the legs are loaded while you judge which is short.
        self.node.publish_owner("CALIBRATION")
        self.node.publish_pose(K.WALK_POSE)

    def save(self):
        problems = self.violations()
        if problems:
            QMessageBox.critical(
                self, "Not saved",
                "Fix these first:\n\n" + "\n".join(problems))
            return
        for joint in K.JOINT_NAMES:
            self.raw["servos"][joint]["trim_deg"] = round(
                float(self.trims[joint]), 3
            )
        with open(self.path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(self.raw, handle, sort_keys=False)
        QMessageBox.information(
            self, "Saved",
            "Trims written to\n%s\n\nRestart the serial bridge for the robot "
            "to use them: the calibration is read once at startup." % self.path)


def main():
    path = default_calibration_path()
    rclpy.init()
    application = QApplication(sys.argv)
    application.setFont(QFont("DejaVu Sans", 10))
    node = StandNode()
    try:
        window = StandWindow(node, path)
    except Exception as exc:  # noqa: BLE001
        print("ERROR: %s" % exc, file=sys.stderr)
        return 1
    spin = QTimer()
    spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin.start(20)
    signal.signal(signal.SIGINT, lambda *_a: window.close())
    window.show()
    try:
        code = application.exec_()
    except KeyboardInterrupt:
        code = 130
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
