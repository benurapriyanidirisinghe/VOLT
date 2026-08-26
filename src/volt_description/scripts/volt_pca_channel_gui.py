#!/usr/bin/env python3

"""Per-channel PCA9685 tester: which physical servo does each channel drive?

This tool talks to the firmware with raw ``SERVO <channel> <degrees>`` commands
on ``/volt/serial_command``. That path bypasses the joint-name -> channel
mapping in servo_calibration.yaml entirely, so what moves is decided by the
WIRING, not by the calibration. That is exactly what is needed to answer "does
channel 5 really drive the front-right foot?", which no amount of reading the
YAML can settle.

Workflow:
  1. Robot on a stand, feet clear, servo power reachable. ARM.
  2. Wiggle one channel. Watch which joint on the robot actually moves.
  3. Record it in that row's "observed" box.
  4. Repeat for all twelve, then press "Compare" for a diff against the
     calibration.

Every command is clamped to that channel's min_deg/max_deg from
servo_calibration.yaml before it is sent, and the firmware clamps again against
its own CHANNEL_MIN_DEG/CHANNEL_MAX_DEG. Angles are absolute servo degrees, the
same units the calibration and the firmware use.
"""

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
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from std_msgs.msg import String

from volt_kinematics import JOINT_NAMES, WALK_POSE
from volt_servo_calibration import ServoCalibrationTable, named_positions_from_ordered

CHANNEL_COUNT = 12

# Caps tighter than servo_calibration.yaml, for joints whose LINKAGE binds
# before the servo does. ch5 (front_right_foot) ran safely to 126.3 all session
# and jammed hard at 143.2. The firmware carries the same 130 cap, but it only
# takes effect after a flash, so the cap is repeated here: this tool must never
# be the thing that drives a joint into its stop.
# (min, max); None means "use the calibration value".
SAFE_CAP_DEG = {2: (50.0, None), 5: (None, 130.0)}
SLIDER_SCALE = 10
DEFAULT_WIGGLE_DEG = 8.0
SWEEP_STEP_MS = 900


def default_calibration_path():
    here = Path(__file__).resolve().parent
    return str(here.parent / "config" / "servo_calibration.yaml")


class ChannelTestNode(Node):
    def __init__(self, status_callback):
        super().__init__("volt_pca_channel_gui")
        self.serial_command_publisher = self.create_publisher(
            String, "/volt/serial_command", 10
        )
        self.owner_publisher = self.create_publisher(
            String, "/volt/command_owner", 10
        )
        self.create_subscription(
            String, "/volt/serial_status", status_callback, 10
        )

    def send_serial(self, command):
        message = String()
        message.data = command
        self.serial_command_publisher.publish(message)

    def send_owner(self, owner):
        message = String()
        message.data = owner
        self.owner_publisher.publish(message)


class ChannelRow:
    """One PCA channel: what the calibration claims, and what actually moves."""

    def __init__(self, channel, servo, joint_name, stand_deg, window, grid, row):
        self.channel = channel
        self.servo = servo
        self.claimed_joint = joint_name
        self.stand_deg = stand_deg
        self.window = window
        self._wiggle_back_to = None

        lo = servo.min_deg if servo is not None else 0.0
        hi = servo.max_deg if servo is not None else 180.0
        cap_lo, cap_hi = SAFE_CAP_DEG.get(channel, (None, None))
        self.capped = False
        if cap_lo is not None and cap_lo > lo:
            lo = cap_lo
            self.capped = True
        if cap_hi is not None and cap_hi < hi:
            hi = cap_hi
            self.capped = True
        self.low, self.high = lo, hi

        label = QLabel("ch%d" % channel)
        label.setFont(QFont("DejaVu Sans Mono", 12, QFont.Bold))
        label.setMinimumWidth(48)

        self.claim_label = QLabel(joint_name or "(unmapped)")
        self.claim_label.setMinimumWidth(180)
        self.claim_label.setStyleSheet("color: #888;")

        self.spin = QDoubleSpinBox()
        self.spin.setRange(lo, hi)
        self.spin.setDecimals(1)
        self.spin.setSingleStep(0.5)
        self.spin.setSuffix(" deg")
        self.spin.setValue(stand_deg)
        self.spin.setMinimumWidth(96)
        self.spin.valueChanged.connect(self.on_spin)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(lo * SLIDER_SCALE), int(hi * SLIDER_SCALE))
        self.slider.setValue(int(stand_deg * SLIDER_SCALE))
        self.slider.setMinimumWidth(200)
        self.slider.valueChanged.connect(self.on_slider)

        self.limit_label = QLabel(
            "[%.0f, %.0f]%s" % (lo, hi, " capped" if self.capped else "")
        )
        self.limit_label.setStyleSheet(
            "color:#ffd24a;" if self.capped else "color: #888;"
        )
        if self.capped:
            self.limit_label.setToolTip(
                "Tighter than servo_calibration.yaml: this linkage jams before "
                "the servo's nominal limit."
            )

        wiggle = QPushButton("Wiggle")
        wiggle.setToolTip("Move off the stand pose and back, so the joint is "
                          "easy to spot.")
        wiggle.clicked.connect(self.wiggle)

        center = QPushButton("Stand")
        center.setToolTip("Return this channel to its calibrated stand angle.")
        center.clicked.connect(self.center)

        self.observed = QComboBox()
        self.observed.addItem("- not checked -", None)
        for name in JOINT_NAMES:
            self.observed.addItem(name, name)
        self.observed.setMinimumWidth(190)

        for column, widget in enumerate((
            label, self.claim_label, self.spin, self.slider,
            self.limit_label, wiggle, center, self.observed,
        )):
            grid.addWidget(widget, row, column)

    # -- command path ----------------------------------------------------

    def clamp(self, value):
        return max(self.low, min(self.high, float(value)))

    def send(self, degrees):
        degrees = self.clamp(degrees)
        self.window.send_channel(self.channel, degrees)

    def on_spin(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(int(value * SLIDER_SCALE))
        self.slider.blockSignals(False)
        self.send(value)

    def on_slider(self, value):
        degrees = value / float(SLIDER_SCALE)
        self.spin.blockSignals(True)
        self.spin.setValue(degrees)
        self.spin.blockSignals(False)
        self.send(degrees)

    def center(self):
        self.spin.setValue(self.stand_deg)

    def wiggle(self):
        """Step away from the stand pose; the window schedules the return."""
        amount = self.window.wiggle_amount()
        target = self.clamp(self.stand_deg + amount)
        if abs(target - self.stand_deg) < 0.5:
            target = self.clamp(self.stand_deg - amount)
        self._wiggle_back_to = self.stand_deg
        self.spin.setValue(target)
        self.window.schedule_return(self)

    def finish_wiggle(self):
        if self._wiggle_back_to is None:
            return
        self.spin.setValue(self._wiggle_back_to)
        self._wiggle_back_to = None

    def observed_joint(self):
        return self.observed.currentData()

    def highlight(self, active):
        self.claim_label.setStyleSheet(
            "color: #ffd24a; font-weight: bold;" if active else "color: #888;"
        )


class ChannelWindow(QMainWindow):
    def __init__(self, node, table):
        super().__init__()
        self.node = node
        self.table = table
        self.setWindowTitle("VOLT — PCA channel tester")
        self.rows = []
        self._sweep_index = None
        self._last_status = ""

        by_channel = {s.pca_channel: (name, s) for name, s in table.servos.items()}
        stand = self._stand_angles()

        central = QWidget()
        outer = QVBoxLayout(central)

        warning = QLabel(
            "Raw SERVO commands go straight to the firmware and bypass the "
            "joint→channel map. Keep the robot on a stand with the power "
            "disconnect in reach. The firmware ignores these unless ARMed."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet(
            "background:#5a3a10; color:#ffd24a; padding:8px; border-radius:4px;"
        )
        outer.addWidget(warning)

        controls = QHBoxLayout()
        # The motion controller streams FRAME packets at 100 Hz holding the
        # stand pose; those would overwrite every SERVO command within 10 ms.
        # Claiming CALIBRATION ownership stops the motion path. It is
        # republished continuously because the control GUI heartbeats its own
        # MOTION ownership at 5 Hz and would otherwise win the topic back.
        self.take_control = QPushButton("Take control")
        self.take_control.setCheckable(True)
        self.take_control.setToolTip(
            "Claim CALIBRATION ownership so the walking controller stops "
            "sending frames. Required before SERVO commands do anything."
        )
        self.take_control.toggled.connect(self.on_take_control)
        controls.addWidget(self.take_control)
        self.arm_button = QPushButton("ARM")
        self.arm_button.clicked.connect(lambda: self.send_raw("ARM"))
        self.disarm_button = QPushButton("DISARM")
        self.disarm_button.clicked.connect(lambda: self.send_raw("DISARM"))
        self.all_stand_button = QPushButton("All to stand")
        self.all_stand_button.clicked.connect(self.all_to_stand)
        self.sweep_button = QPushButton("Sweep all channels")
        self.sweep_button.clicked.connect(self.toggle_sweep)
        self.wiggle_spin = QDoubleSpinBox()
        self.wiggle_spin.setRange(1.0, 25.0)
        self.wiggle_spin.setValue(DEFAULT_WIGGLE_DEG)
        self.wiggle_spin.setSuffix(" deg wiggle")
        for widget in (self.arm_button, self.disarm_button, self.all_stand_button,
                       self.sweep_button, self.wiggle_spin):
            controls.addWidget(widget)
        controls.addStretch(1)
        outer.addLayout(controls)

        self.banner = QLabel("idle")
        self.banner.setAlignment(Qt.AlignCenter)
        self.banner.setFont(QFont("DejaVu Sans", 16, QFont.Bold))
        self.banner.setStyleSheet(
            "background:#1d2433; color:#7fd1c8; padding:10px; border-radius:4px;"
        )
        outer.addWidget(self.banner)

        box = QGroupBox("Channels — 'claims' is what servo_calibration.yaml "
                        "says this channel drives")
        grid = QGridLayout(box)
        for column, title in enumerate((
            "", "claims", "angle", "", "limits", "", "", "observed (what moved)"
        )):
            header = QLabel(title)
            header.setStyleSheet("color:#9aa; font-weight:bold;")
            grid.addWidget(header, 0, column)
        for channel in range(CHANNEL_COUNT):
            joint_name, servo = by_channel.get(channel, (None, None))
            self.rows.append(ChannelRow(
                channel, servo, joint_name, stand.get(channel, 90.0),
                self, grid, channel + 1,
            ))
        outer.addWidget(box)

        compare = QPushButton("Compare observed against calibration")
        compare.clicked.connect(self.compare)
        outer.addWidget(compare)

        self.report = QPlainTextEdit()
        self.report.setReadOnly(True)
        self.report.setMinimumHeight(150)
        self.report.setFont(QFont("DejaVu Sans Mono", 9))
        outer.addWidget(self.report)

        self.status_label = QLabel("serial: (no status yet)")
        self.status_label.setStyleSheet("color:#888;")
        outer.addWidget(self.status_label)

        self.setCentralWidget(central)
        self.resize(1180, 760)

        self.return_timer = QTimer(self)
        self.return_timer.setSingleShot(True)
        self.return_timer.timeout.connect(self._do_return)
        self._pending_return = None

        self.sweep_timer = QTimer(self)
        self.sweep_timer.timeout.connect(self._sweep_tick)

        # 20 Hz beats the control GUI's 5 Hz MOTION heartbeat.
        self.owner_timer = QTimer(self)
        self.owner_timer.timeout.connect(
            lambda: self.node.send_owner("CALIBRATION")
        )

        # The firmware disarms after COMMAND_TIMEOUT_MS = 750 ms of silence,
        # and only a command refreshes that watchdog. When this tool is the
        # only thing talking to the robot there is no FRAME stream to do it,
        # so re-send each channel's current angle round-robin: one channel
        # every 40 ms covers all twelve in 480 ms, inside the timeout. Values
        # are unchanged, so nothing moves -- it only holds the watchdog and
        # keeps every channel at its commanded position.
        self._keepalive_index = 0
        self.keepalive_timer = QTimer(self)
        self.keepalive_timer.timeout.connect(self._keepalive_tick)

    def _keepalive_tick(self):
        if not self.rows:
            return
        row = self.rows[self._keepalive_index % len(self.rows)]
        self._keepalive_index += 1
        self.send_channel(row.channel, row.clamp(row.spin.value()))

    # -- ownership -------------------------------------------------------

    def on_take_control(self, checked):
        if checked:
            self.node.send_owner("CALIBRATION")
            self.owner_timer.start(50)
            self.keepalive_timer.start(40)
            self.take_control.setText("Release control")
            self.banner.setText(
                "CALIBRATION ownership held — walking frames stopped"
            )
        else:
            self.owner_timer.stop()
            self.keepalive_timer.stop()
            self.node.send_owner("HOLD")
            self.take_control.setText("Take control")
            self.banner.setText("control released (HOLD)")

    # -- helpers ---------------------------------------------------------

    def _stand_angles(self):
        """Calibrated servo angle per channel at WALK_POSE."""
        named = named_positions_from_ordered(WALK_POSE)
        out = {}
        for joint, radians in named.items():
            servo = self.table.servos[joint]
            out[servo.pca_channel] = self.table.ros_radians_to_servo_degrees(
                joint, radians
            )
        return out

    def wiggle_amount(self):
        return float(self.wiggle_spin.value())

    def send_channel(self, channel, degrees):
        self.node.send_serial("SERVO %d %.2f" % (channel, degrees))

    def send_raw(self, command):
        """ARM is gated on recent MOTION ownership at the bridge, so the
        CALIBRATION claim must be paused across it or the ARM is refused."""
        if command == "ARM" and self.take_control.isChecked():
            self.owner_timer.stop()
            self.node.send_owner("MOTION")
            QTimer.singleShot(600, lambda: self.node.send_serial("ARM"))
            QTimer.singleShot(1200, lambda: self.owner_timer.start(50))
            self.banner.setText("sent ARM (ownership briefly returned to MOTION)")
            return
        self.node.send_serial(command)
        self.banner.setText("sent %s" % command)

    def all_to_stand(self):
        for row in self.rows:
            row.center()
        self.banner.setText("all channels returned to the stand pose")

    def schedule_return(self, row):
        self._pending_return = row
        self.banner.setText("ch%d moving — which joint moved?" % row.channel)
        for other in self.rows:
            other.highlight(other is row)
        self.return_timer.start(SWEEP_STEP_MS)

    def _do_return(self):
        if self._pending_return is not None:
            self._pending_return.finish_wiggle()
            self._pending_return = None

    # -- sweep -----------------------------------------------------------

    def toggle_sweep(self):
        if self.sweep_timer.isActive():
            self.sweep_timer.stop()
            self._sweep_index = None
            for row in self.rows:
                row.highlight(False)
            self.all_to_stand()
            self.sweep_button.setText("Sweep all channels")
            self.banner.setText("sweep stopped")
            return
        self._sweep_index = 0
        self.sweep_button.setText("Stop sweep")
        self.sweep_timer.start(SWEEP_STEP_MS * 2)
        self._sweep_tick()

    def _sweep_tick(self):
        if self._sweep_index is None:
            return
        if self._sweep_index >= len(self.rows):
            self.toggle_sweep()
            return
        row = self.rows[self._sweep_index]
        self._sweep_index += 1
        row.wiggle()

    # -- reporting -------------------------------------------------------

    def compare(self):
        lines = []
        lines.append("%-4s %-22s %-22s %s" % (
            "ch", "calibration claims", "you observed", "verdict"))
        lines.append("-" * 78)
        mismatches = []
        unchecked = 0
        for row in self.rows:
            observed = row.observed_joint()
            if observed is None:
                unchecked += 1
                verdict = "not checked"
            elif observed == row.claimed_joint:
                verdict = "match"
            else:
                verdict = "MISMATCH"
                mismatches.append((row.channel, row.claimed_joint, observed))
            lines.append("%-4d %-22s %-22s %s" % (
                row.channel, row.claimed_joint or "(unmapped)",
                observed or "-", verdict))
        lines.append("")
        if unchecked:
            lines.append("%d channel(s) still unchecked." % unchecked)
        if not mismatches:
            if unchecked < CHANNEL_COUNT:
                lines.append("No mismatch among the checked channels: the "
                             "wiring agrees with servo_calibration.yaml.")
        else:
            lines.append("%d MISMATCH(es)." % len(mismatches))
            lines.append("")
            lines.append("Swap the WHOLE servo entry, not just pca_channel.")
            lines.append("direction/neutral/trim/limits describe the SERVO and")
            lines.append("its linkage, not the joint, so they must travel with")
            lines.append("the channel. Moving pca_channel alone leaves each")
            lines.append("joint paired with the other servo's mirror convention")
            lines.append("and steps both channels on the very first frame.")
            lines.append("")
            observed_channel = {}
            for row in self.rows:
                if row.observed_joint() is not None:
                    observed_channel[row.observed_joint()] = row.channel
            by_channel = {
                servo.pca_channel: servo
                for servo in self.table.servos.values()
            }
            for joint, channel in sorted(observed_channel.items()):
                current = self.table.servos[joint].pca_channel
                if current == channel:
                    continue
                donor = by_channel.get(channel)
                if donor is None:
                    continue
                lines.append("    %s:" % joint)
                lines.append("      pca_channel: %d      (was %d)"
                             % (channel, current))
                lines.append("      direction: %+d" % donor.direction)
                lines.append("      neutral_deg: %.2f" % donor.neutral_deg)
                lines.append("      trim_deg: %.2f" % donor.trim_deg)
                lines.append("      min_deg: %.1f" % donor.min_deg)
                lines.append("      max_deg: %.1f" % donor.max_deg)
            duplicates = len(observed_channel) != len(set(observed_channel.values()))
            if duplicates or len(observed_channel) < CHANNEL_COUNT - unchecked:
                lines.append("")
                lines.append("WARNING: the observations are not a clean "
                             "one-to-one mapping. Re-check before editing.")
        self.report.setPlainText("\n".join(lines))

    def handle_status(self, message):
        self._last_status = message.data

    def refresh_status(self):
        text = self._last_status
        if not text:
            return
        shown = text if len(text) < 150 else text[:147] + "..."
        self.status_label.setText("serial: %s" % shown)


def main():
    calibration_path = default_calibration_path()
    try:
        table = ServoCalibrationTable.from_file(calibration_path)
    except Exception as exc:  # noqa: BLE001 - surfaced to the operator
        print("ERROR: cannot load %s: %s" % (calibration_path, exc),
              file=sys.stderr)
        return 1

    rclpy.init()
    application = QApplication(sys.argv)
    application.setFont(QFont("DejaVu Sans", 10))

    holder = {}
    node = ChannelTestNode(lambda message: holder["window"].handle_status(message))
    window = ChannelWindow(node, table)
    holder["window"] = window

    spin = QTimer()
    spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin.start(20)

    status = QTimer()
    status.timeout.connect(window.refresh_status)
    status.start(500)

    signal.signal(signal.SIGINT, lambda *_args: window.close())
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
