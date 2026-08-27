#!/usr/bin/env python3

"""Measurement and reporting console for VOLT.

Three things live here, because on this robot they are one question:

  * LINK    -- the firmware's own counters (CRC failures, sequence gaps, loop
               and I2C timing, free SRAM). Stage-1 acceptance is CRC and gap
               both zero over 60 s of walking with the face animating.
  * GAIT    -- what the engine actually commands: per-leg stride, foot lift,
               joint amplitude, peak joint rate, and the left/right and
               diagonal symmetry checks. Computed through the repo's own gait
               engine, IK and calibration, so it is the real command path and
               not a model of it.
  * BUDGET  -- knee torque against usable servo stall as a function of body
               height, with the firmware's per-channel guards applied. This is
               the constraint that decides whether a foot can leave the ground
               at all.

The robot is OPEN LOOP: no servo reports its position, and under --physical
/joint_states is the simulator's state, not the machine's. So nothing here
claims to measure ACHIEVED motion. Everything is either a commanded quantity
(exact, computed from the command path) or a firmware-reported counter.
Achieved angles and foot clearance come from video tracking or simulation.
The report makes that distinction explicit rather than blurring it.
"""

import math
import signal
import sys
from pathlib import Path

import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from std_msgs.msg import String

import volt_gait_controller as G
import volt_kinematics as K
from volt_servo_calibration import (
    ServoCalibrationTable,
    named_positions_from_ordered,
)

# Mass from the URDF link inventory: body 2.80 + lidar 0.50 + two 0.20 base
# links + four legs at 0.40. The URDF cannot give a CoM offset (none of its
# <inertial> blocks carry an <origin>), so loads here assume it centred and
# any real fore-aft offset shifts them.
ROBOT_MASS_KG = 2.80 + 0.50 + 0.20 + 0.20 + 4 * 0.40
GRAVITY = 9.81

# TD-8130MG datasheet stall is 2.94 N.m at rated voltage on an unlimited
# supply. Twelve servos on one shared BEC realistically deliver 60-75%, so the
# pessimistic figure is what a walking decision should be made against.
STALL_DATASHEET_NM = 2.94
STALL_USABLE_NM = 1.76
SERVO_FREE_SPEED_DPS = 375.0
FIRMWARE_SLEW_CEILING_DPS = 240.0

# Guards tighter than the calibration, mirroring the firmware's per-channel
# limits for the two front foot joints. See CHANNEL_MIN_DEG/CHANNEL_MAX_DEG.
CHANNEL_GUARDS = {2: (50.0, None), 5: (None, 130.0)}

CONTROL_RATE_HZ = 100.0


def default_calibration_path():
    return str(Path(__file__).resolve().parent.parent / "config"
               / "servo_calibration.yaml")


def _ptp(values):
    return max(values) - min(values)


class GaitMeasurement:
    """Everything computable about one gait configuration, without hardware."""

    def __init__(self, table, gait_name, body_height, speed_scale=1.0,
                 yaw_fraction=0.0, samples=400):
        self.table = table
        self.gait_name = gait_name
        self.body_height = float(body_height)
        self.error = ""
        config = dict(G.load_gait_configs()[gait_name])
        self.config = config
        hardware_scale = config.get("hardware_speed_scale", 1.0)
        self.vx = config["max_x"] * hardware_scale * float(speed_scale)
        self.wz = config["max_yaw"] * hardware_scale * float(yaw_fraction)

        self.foot = {leg: {"x": [], "y": [], "z": []} for leg in K.LEG_ORDER}
        self.servo = {}
        self.projected = set()
        self.clamped = set()

        for index in range(samples):
            phase = index / float(samples)
            feet = {}
            for leg in K.LEG_ORDER:
                dx, dy, dz, _stance = G._sweep_offsets(
                    config, phase, (self.vx, 0.0, self.wz), leg
                )
                nominal = K.NOMINAL_FEET[leg]
                feet[leg] = (nominal[0] + dx, nominal[1] + dy,
                             -self.body_height + dz)
                self.foot[leg]["x"].append(dx)
                self.foot[leg]["y"].append(dy)
                self.foot[leg]["z"].append(dz)
            try:
                ordered, diagnostics = K.feet_to_joint_positions_diagnostic(
                    feet, height=self.body_height
                )
            except Exception as exc:  # noqa: BLE001 - reported, not raised
                self.error = "IK failed: %s" % exc
                return
            self.projected.update(diagnostics["projected_targets"])
            for joint, radians in named_positions_from_ordered(ordered).items():
                servo = table.servos[joint]
                raw = (servo.neutral_deg + servo.trim_deg
                       + servo.direction * math.degrees(radians))
                if raw < servo.min_deg - 1e-9 or raw > servo.max_deg + 1e-9:
                    self.clamped.add(joint)
                low, high = CHANNEL_GUARDS.get(servo.pca_channel, (None, None))
                if (low is not None and raw < low - 1e-9) or (
                    high is not None and raw > high + 1e-9
                ):
                    self.clamped.add(joint + " (guard)")
                self.servo.setdefault(joint, []).append(raw)

    # -- derived quantities ----------------------------------------------

    def stride_mm(self, leg):
        return _ptp(self.foot[leg]["x"]) * 1000.0

    def lift_mm(self, leg):
        return _ptp(self.foot[leg]["z"]) * 1000.0

    def joint_amplitude_deg(self, joint):
        return _ptp(self.servo[joint]) if joint in self.servo else float("nan")

    def peak_rate_dps(self):
        """Peak commanded servo rate at the real control rate."""
        dt = 1.0 / CONTROL_RATE_HZ
        period = self.config["cycle_period"]
        steps = max(int(round(period / dt)), 2)
        previous = None
        peak = 0.0
        for index in range(steps * 2 + 1):
            phase = ((index * dt) / period) % 1.0
            feet = {}
            for leg in K.LEG_ORDER:
                dx, dy, dz, _s = G._sweep_offsets(
                    self.config, phase, (self.vx, 0.0, self.wz), leg
                )
                nominal = K.NOMINAL_FEET[leg]
                feet[leg] = (nominal[0] + dx, nominal[1] + dy,
                             -self.body_height + dz)
            try:
                ordered, _d = K.feet_to_joint_positions_diagnostic(
                    feet, height=self.body_height
                )
            except Exception:  # noqa: BLE001
                return float("nan")
            current = {
                joint: self.table.ros_radians_to_servo_degrees(joint, radians)
                for joint, radians in
                named_positions_from_ordered(ordered).items()
            }
            if previous is not None and index > steps:
                peak = max(peak, max(
                    abs(current[j] - previous[j]) for j in current
                ) / dt)
            previous = current
        return peak

    def side_symmetry_mm(self):
        left = (self.stride_mm("front_left") + self.stride_mm("rear_left")) / 2
        right = (self.stride_mm("front_right")
                 + self.stride_mm("rear_right")) / 2
        return left, right, left - right

    def diagonal_symmetry_mm(self):
        def mean_lift(a, b):
            za = sum(self.foot[a]["z"]) / len(self.foot[a]["z"])
            zb = sum(self.foot[b]["z"]) / len(self.foot[b]["z"])
            return (za + zb) / 2.0 * 1000.0
        one = mean_lift("front_left", "rear_right")
        two = mean_lift("front_right", "rear_left")
        return one, two, one - two


def knee_torque_nm(body_height, feet_down):
    """Front-knee torque holding one foot, from the repo's own FK."""
    feet = {leg: (K.NOMINAL_FEET[leg][0], K.NOMINAL_FEET[leg][1],
                  -body_height) for leg in K.LEG_ORDER}
    ordered, _d = K.feet_to_joint_positions_diagnostic(feet, height=body_height)
    named = named_positions_from_ordered(ordered)
    angles = [named["front_left_%s" % j]
              for j in ("shoulder", "leg", "foot")]
    delta = 1e-6
    probe = list(angles)
    probe[2] += delta
    arm = abs(
        (K.forward_leg("front_left", probe)[2]
         - K.forward_leg("front_left", angles)[2]) / delta
    )
    return (ROBOT_MASS_KG * GRAVITY / float(feet_down)) * arm


class MeasureNode(Node):
    def __init__(self, on_serial, on_motion):
        super().__init__("volt_measure_gui")
        self.create_subscription(String, "/volt/serial_status", on_serial, 10)
        self.create_subscription(String, "/volt/status", on_motion, 10)


class MeasureWindow(QMainWindow):
    def __init__(self, table):
        super().__init__()
        self.table = table
        self.setWindowTitle("VOLT — measurement & report")
        self.serial_fields = {}
        self.motion_raw = ""

        tabs = QTabWidget()
        tabs.addTab(self._link_tab(), "LINK")
        tabs.addTab(self._gait_tab(), "GAIT")
        tabs.addTab(self._budget_tab(), "BUDGET")
        tabs.addTab(self._report_tab(), "REPORT")
        self.setCentralWidget(tabs)
        self.resize(1120, 800)

    # -- tabs -------------------------------------------------------------

    def _link_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        note = QLabel(
            "Firmware counters, polled by the bridge. Stage-1 acceptance: CRC "
            "failures and sequence gaps both zero over 60 s of walking with "
            "the face animating."
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        self.link_label = QLabel("Waiting for /volt/serial_status ...")
        self.link_label.setFont(QFont("DejaVu Sans Mono", 10))
        self.link_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.link_label)
        layout.addStretch(1)
        return page

    def _gait_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        controls = QGridLayout()
        self.gait_combo = QComboBox()
        for name in sorted(G.load_gait_configs()):
            self.gait_combo.addItem(name.upper(), name)
        self.height_spin = QDoubleSpinBox()
        self.height_spin.setRange(0.150, 0.235)
        self.height_spin.setSingleStep(0.005)
        self.height_spin.setDecimals(3)
        self.height_spin.setValue(0.200)
        self.height_spin.setSuffix(" m body height")
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.0, 1.0)
        self.speed_spin.setSingleStep(0.1)
        self.speed_spin.setValue(1.0)
        self.speed_spin.setSuffix(" speed scale")
        self.yaw_spin = QDoubleSpinBox()
        self.yaw_spin.setRange(-1.0, 1.0)
        self.yaw_spin.setSingleStep(0.25)
        self.yaw_spin.setValue(0.0)
        self.yaw_spin.setSuffix(" yaw trim")
        measure = QPushButton("Measure commanded gait")
        measure.clicked.connect(self.measure_gait)
        for column, widget in enumerate((
            self.gait_combo, self.height_spin, self.speed_spin,
            self.yaw_spin, measure,
        )):
            controls.addWidget(widget, 0, column)
        layout.addLayout(controls)
        self.gait_output = QPlainTextEdit()
        self.gait_output.setReadOnly(True)
        self.gait_output.setFont(QFont("DejaVu Sans Mono", 9))
        layout.addWidget(self.gait_output)
        return page

    def _budget_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        note = QLabel(
            "Knee torque against usable stall as body height varies, with the "
            "firmware's per-channel guards applied. Straighter legs shorten "
            "the knee moment arm, so standing TALLER lowers torque; crouching "
            "raises it."
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        button = QPushButton("Sweep body height")
        button.clicked.connect(self.sweep_budget)
        layout.addWidget(button)
        self.budget_output = QPlainTextEdit()
        self.budget_output.setReadOnly(True)
        self.budget_output.setFont(QFont("DejaVu Sans Mono", 9))
        layout.addWidget(self.budget_output)
        return page

    def _report_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        row = QHBoxLayout()
        build = QPushButton("Build full report")
        build.clicked.connect(self.build_report)
        save = QPushButton("Save to file")
        save.clicked.connect(self.save_report)
        row.addWidget(build)
        row.addWidget(save)
        row.addStretch(1)
        layout.addLayout(row)
        self.report_output = QPlainTextEdit()
        self.report_output.setReadOnly(True)
        self.report_output.setFont(QFont("DejaVu Sans Mono", 9))
        layout.addWidget(self.report_output)
        return page

    # -- actions ----------------------------------------------------------

    def measure_gait(self):
        self.gait_output.setPlainText("\n".join(self.gait_lines()))

    def gait_lines(self):
        gait = self.gait_combo.currentData()
        height = self.height_spin.value()
        measurement = GaitMeasurement(
            self.table, gait, height,
            speed_scale=self.speed_spin.value(),
            yaw_fraction=self.yaw_spin.value(),
        )
        lines = []
        config = measurement.config
        lines.append(
            "%s  cycle %.2fs (%.2f Hz)  duty %.2f  step_h %.3fm  height %.3fm"
            % (gait.upper(), config["cycle_period"],
               1.0 / config["cycle_period"], config["duty_factor"],
               config["step_height"], height)
        )
        lines.append("commanded vx %.4f m/s   wz %.4f rad/s (%.1f deg/s)"
                     % (measurement.vx, measurement.wz,
                        math.degrees(measurement.wz)))
        if measurement.error:
            lines.append("")
            lines.append("ERROR: " + measurement.error)
            return lines
        lines.append("")
        lines.append("%-13s %11s %11s %13s %13s"
                     % ("leg", "stride mm", "lift mm", "hip sweep", "knee sweep"))
        for leg in K.LEG_ORDER:
            lines.append("%-13s %11.3f %11.3f %12.3fd %12.3fd" % (
                leg, measurement.stride_mm(leg), measurement.lift_mm(leg),
                measurement.joint_amplitude_deg(leg + "_leg"),
                measurement.joint_amplitude_deg(leg + "_foot"),
            ))
        left, right, delta = measurement.side_symmetry_mm()
        one, two, diag = measurement.diagonal_symmetry_mm()
        lines.append("")
        lines.append("SYMMETRY (a yaw bias from the gait needs side asymmetry;")
        lines.append("          a 1x roll needs the diagonals to differ)")
        lines.append("  left %.4f mm   right %.4f mm   difference %.3e mm"
                     % (left, right, delta))
        lines.append("  diagonals %.4f / %.4f mm      difference %.3e mm"
                     % (one, two, diag))
        peak = measurement.peak_rate_dps()
        lines.append("")
        lines.append("RATE  peak commanded %.1f deg/s  = %.0f%% of the %.0f "
                     "deg/s firmware ceiling"
                     % (peak, 100.0 * peak / FIRMWARE_SLEW_CEILING_DPS,
                        FIRMWARE_SLEW_CEILING_DPS))
        torque = knee_torque_nm(height, 2)
        available = SERVO_FREE_SPEED_DPS * max(
            0.0, 1.0 - torque / STALL_USABLE_NM
        )
        lines.append("      loaded stance joints have about %.0f deg/s "
                     "available at this torque" % available)
        lines.append("")
        lines.append("LIMITS  IK-projected legs: %s"
                     % (", ".join(sorted(measurement.projected)) or "none"))
        lines.append("        clamped/guarded joints: %s"
                     % (", ".join(sorted(measurement.clamped)) or "none"))
        lines.append("")
        lines.append("These are COMMANDED quantities, exact from the command "
                     "path. Achieved motion is not measurable on this open-"
                     "loop robot; use video tracking or simulation.")
        return lines

    def sweep_budget(self):
        self.budget_output.setPlainText("\n".join(self.budget_lines()))

    def budget_lines(self):
        lines = []
        lines.append("mass %.2f kg  weight %.1f N   usable stall %.2f N.m "
                     "(%.0f%% of the %.2f N.m datasheet figure)"
                     % (ROBOT_MASS_KG, ROBOT_MASS_KG * GRAVITY,
                        STALL_USABLE_NM,
                        100.0 * STALL_USABLE_NM / STALL_DATASHEET_NM,
                        STALL_DATASHEET_NM))
        lines.append("")
        lines.append("%8s %12s %9s %12s %9s %s" % (
            "height", "trot N.m", "% stall", "amble N.m", "% stall", "IK"))
        for step in range(0, 9):
            height = 0.180 + 0.005 * step
            try:
                two = knee_torque_nm(height, 2)
                three = knee_torque_nm(height, 3)
                feet = {leg: (K.NOMINAL_FEET[leg][0], K.NOMINAL_FEET[leg][1],
                              -height) for leg in K.LEG_ORDER}
                _o, diagnostics = K.feet_to_joint_positions_diagnostic(
                    feet, height=height)
                state = ("projected: %s"
                         % ",".join(diagnostics["projected_targets"])
                         if diagnostics["projected_targets"] else "ok")
            except Exception as exc:  # noqa: BLE001
                lines.append("%7.3fm   %s" % (height, exc))
                continue
            flag = "  <-- over stall" if two > STALL_USABLE_NM else ""
            lines.append("%7.3fm %12.3f %8.0f%% %12.3f %8.0f%% %s%s" % (
                height, two, 100.0 * two / STALL_USABLE_NM,
                three, 100.0 * three / STALL_USABLE_NM, state, flag))
        lines.append("")
        lines.append("A trot carries the body on TWO diagonal feet; an amble "
                     "keeps three down. That is the whole difference between "
                     "the two columns, and it is why the amble lifts a foot "
                     "when the trot cannot.")
        return lines

    def build_report(self):
        lines = ["VOLT measurement report", "=" * 60, ""]
        lines.append("LINK")
        lines.extend("  " + line for line in self.link_text().splitlines())
        lines.append("")
        lines.append("GAIT")
        lines.extend("  " + line for line in self.gait_lines())
        lines.append("")
        lines.append("BUDGET")
        lines.extend("  " + line for line in self.budget_lines())
        self.report_output.setPlainText("\n".join(lines))

    def save_report(self):
        if not self.report_output.toPlainText():
            self.build_report()
        path, _filter = QFileDialog.getSaveFileName(
            self, "Save report", "volt_report.txt", "Text (*.txt)")
        if not path:
            return
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(self.report_output.toPlainText())

    # -- ROS ---------------------------------------------------------------

    def on_serial(self, message):
        self.serial_fields = dict(
            item.split("=", 1) for item in message.data.split() if "=" in item
        )

    def on_motion(self, message):
        self.motion_raw = message.data

    def link_text(self):
        fields = self.serial_fields
        if "fw_crc_fail" not in fields:
            if not fields:
                return "Waiting for /volt/serial_status ..."
            return ("Bridge is publishing, but the firmware has not reported "
                    "counters yet.\nThey need PROTO>=3 firmware and an armed, "
                    "streaming link.")
        crc = fields.get("fw_crc_fail", "?")
        gaps = fields.get("fw_seq_gap", "?")
        clean = crc == "0" and gaps == "0"
        return (
            "encoding        %s\n"
            "CRC failures    %s\n"
            "sequence gaps   %s\n"
            "frames received %s binary / %s ascii\n"
            "loop max        %s us   (includes the STATUS print itself)\n"
            "servo I2C max   %s us\n"
            "LED shows       %s\n"
            "free SRAM       %s bytes\n"
            "armed %s   streaming %s\n"
            "\nstage-1 acceptance: %s"
            % (
                "BINARY (PROTO>=3)"
                if fields.get("binary_frames") == "1" else "ASCII (legacy)",
                crc, gaps,
                fields.get("fw_frames_bin", "?"),
                fields.get("fw_frames_ascii", "?"),
                fields.get("fw_loop_max_us", "?"),
                fields.get("fw_i2c_max_us", "?"),
                fields.get("fw_led_shows", "?"),
                fields.get("fw_sram_free", "?"),
                fields.get("armed", "?"), fields.get("streaming", "?"),
                "PASS" if clean else "FAIL - link is corrupting frames",
            )
        )

    def refresh(self):
        text = self.link_text()
        self.link_label.setText(text)
        self.link_label.setStyleSheet(
            "color:#f87171;" if "FAIL" in text else "color:#86efac;"
        )


def main():
    try:
        table = ServoCalibrationTable.from_file(default_calibration_path())
    except Exception as exc:  # noqa: BLE001
        print("ERROR: cannot load calibration: %s" % exc, file=sys.stderr)
        return 1
    rclpy.init()
    application = QApplication(sys.argv)
    application.setFont(QFont("DejaVu Sans", 10))
    holder = {}
    node = MeasureNode(
        lambda m: holder["window"].on_serial(m),
        lambda m: holder["window"].on_motion(m),
    )
    window = MeasureWindow(table)
    holder["window"] = window

    spin = QTimer()
    spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin.start(20)
    refresh = QTimer()
    refresh.timeout.connect(window.refresh)
    refresh.start(500)

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
