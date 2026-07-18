#!/usr/bin/env python3

"""Bridge canonical ROS joint radians to channel-ordered Arduino FRAME packets."""

import math
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from volt_kinematics import JOINT_NAMES
from volt_servo_calibration import (
    CalibrationError,
    ServoCalibrationTable,
    named_positions_from_ordered,
)

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - handled at runtime on robot.
    serial = None
    SerialException = Exception


class VoltSerialBridge(Node):
    def __init__(self):
        super().__init__("volt_serial_bridge")

        default_calibration = (
            get_package_share_directory("volt_description")
            + "/config/servo_calibration.yaml"
        )
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("calibration_file", default_calibration)
        self.declare_parameter("command_topic", "/joint_command_router/output")
        self.declare_parameter("serial_command_topic", "/volt/serial_command")
        self.declare_parameter("status_topic", "/volt/serial_status")
        self.declare_parameter("max_send_rate", 30.0)
        self.declare_parameter("reconnect_period", 1.0)
        self.declare_parameter("serial_timeout", 0.02)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("auto_arm", False)
        self.declare_parameter("command_timeout", 0.75)

        self.port = str(self.get_parameter("port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.calibration_file = str(self.get_parameter("calibration_file").value)
        self.max_send_rate = float(self.get_parameter("max_send_rate").value)
        self.reconnect_period = float(self.get_parameter("reconnect_period").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.hardware_enabled = bool(self.get_parameter("hardware_enabled").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        command_topic = str(self.get_parameter("command_topic").value)
        serial_command_topic = str(self.get_parameter("serial_command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self.calibration = None
        self.calibration_valid = False
        self.calibration_error = ""
        self.load_calibration()
        if self.calibration_valid:
            self.log_mapping_table()

        self.serial_port = None
        self.last_send_time = 0.0
        self.last_connect_attempt = 0.0
        self.connected = False
        self.arduino_ready = False
        self.hardware_armed = False
        self.last_error = ""
        self.last_response = ""
        self.last_command_time = 0.0
        self.last_frame = []
        self.last_details = []
        self.frames_sent = 0
        self.frames_rejected = 0

        self.create_subscription(Float64MultiArray, command_topic, self.command_callback, 1)
        self.create_subscription(String, serial_command_topic, self.serial_command_callback, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.create_timer(0.05, self.timer_callback)

        mode = "dry-run" if self.dry_run or not self.hardware_enabled else "hardware"
        self.get_logger().info(
            "Serial bridge ready in %s mode: topic=%s calibration=%s"
            % (mode, command_topic, self.calibration_file)
        )

    def log_mapping_table(self):
        rows = []
        for joint_name in JOINT_NAMES:
            servo = self.calibration.servos[joint_name]
            rows.append(
                "%-22s ch=%2d center=%7.2f dir=%2d min=%7.2f max=%7.2f"
                % (
                    joint_name,
                    servo.pca_channel,
                    servo.neutral_deg,
                    servo.direction,
                    servo.min_deg,
                    servo.max_deg,
                )
            )
        self.get_logger().info("Servo mapping table:\n%s" % "\n".join(rows))

    def load_calibration(self):
        try:
            self.calibration = ServoCalibrationTable.from_file(self.calibration_file)
            self.calibration_valid = True
            self.calibration_error = ""
        except Exception as exc:
            self.calibration = None
            self.calibration_valid = False
            self.calibration_error = str(exc)
            self.get_logger().error("Invalid servo calibration: %s" % exc)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def connect(self):
        if self.dry_run or not self.hardware_enabled:
            return False
        if serial is None:
            self.last_error = "pyserial is not installed; install python3-serial."
            return False
        if not self.calibration_valid:
            self.last_error = "invalid calibration; hardware disabled"
            return False

        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_period:
            return False
        self.last_connect_attempt = now

        try:
            self.serial_port = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=self.serial_timeout,
                write_timeout=self.serial_timeout,
            )
            time.sleep(2.0)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.connected = True
            self.arduino_ready = True
            self.last_error = ""
            self.write_line("PING")
            if self.auto_arm:
                self.write_line("ARM")
                self.hardware_armed = True
            self.get_logger().info("Connected to Arduino on %s." % self.port)
            return True
        except SerialException as exc:
            self.connected = False
            self.arduino_ready = False
            self.serial_port = None
            self.last_error = str(exc)
            self.get_logger().warning(
                "Waiting for Arduino on %s: %s" % (self.port, exc),
                throttle_duration_sec=5.0,
            )
            return False

    def disconnect(self, reason):
        self.connected = False
        self.arduino_ready = False
        self.hardware_armed = False
        self.last_error = reason
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except SerialException:
                pass
        self.serial_port = None
        self.get_logger().warning("Arduino serial disconnected: %s" % reason)

    def read_available(self):
        if self.serial_port is None:
            return
        try:
            while self.serial_port.in_waiting:
                line = self.serial_port.readline().decode("ascii", errors="replace").strip()
                if line:
                    self.last_response = line
                    if "ARMED=1" in line or line == "OK ARM":
                        self.hardware_armed = True
                    if "ARMED=0" in line or line in ("OK DISARM", "OK HOLD", "OK DISABLE"):
                        self.hardware_armed = False
        except SerialException as exc:
            self.disconnect(str(exc))

    def write_line(self, line):
        if self.serial_port is None:
            raise SerialException("serial port is not open")
        self.serial_port.write((line + "\n").encode("ascii"))

    def serial_command_callback(self, message):
        command = message.data.strip().upper()
        allowed_simple = ("ARM", "DISARM", "HOLD", "DISABLE", "STATUS", "PING")
        if not (command in allowed_simple or command.startswith("SERVO ")):
            self.get_logger().warning("Ignored unsafe serial command '%s'." % command)
            return
        if self.dry_run or not self.hardware_enabled:
            self.last_response = "DRY_RUN %s" % command
            self.get_logger().info("Dry-run Arduino command: %s" % command)
            return
        if not self.connected and not self.connect():
            return
        try:
            self.write_line(command)
            if command == "ARM":
                self.hardware_armed = True
            elif command in ("DISARM", "HOLD", "DISABLE"):
                self.hardware_armed = False
            self.get_logger().info("Sent Arduino command: %s" % command)
        except SerialException as exc:
            self.disconnect(str(exc))

    def build_frame(self, message):
        named = named_positions_from_ordered(message.data)
        return self.calibration.channel_frame_from_positions(named)

    def command_callback(self, message):
        self.last_command_time = self.now_seconds()
        if not self.calibration_valid:
            self.frames_rejected += 1
            return
        if len(message.data) != len(JOINT_NAMES):
            self.frames_rejected += 1
            self.get_logger().warning(
                "Expected %d joints, got %d." % (len(JOINT_NAMES), len(message.data)),
                throttle_duration_sec=2.0,
            )
            return

        now = self.now_seconds()
        min_period = 1.0 / max(self.max_send_rate, 1.0)
        if now - self.last_send_time < min_period:
            return
        self.last_send_time = now

        try:
            frame, details = self.build_frame(message)
        except (CalibrationError, ValueError) as exc:
            self.frames_rejected += 1
            self.last_error = str(exc)
            self.get_logger().warning("Rejected joint command: %s" % exc, throttle_duration_sec=2.0)
            return

        self.last_frame = frame
        self.last_details = details
        line = "FRAME " + " ".join("%.2f" % value for value in frame)
        if self.dry_run or not self.hardware_enabled:
            self.frames_sent += 1
            self.log_frame(line, details)
            return

        if not self.connected and not self.connect():
            self.frames_rejected += 1
            return
        try:
            self.write_line(line)
            self.frames_sent += 1
        except SerialException as exc:
            self.frames_rejected += 1
            self.disconnect(str(exc))

    def log_frame(self, line, details):
        rows = [
            "%-22s %+8.3f %9.2f %5d%s"
            % (
                item["joint"],
                item["ros_rad"],
                item["servo_deg"],
                item["pca_channel"],
                " CLAMPED" if item["clamped"] else "",
            )
            for item in details
        ]
        self.get_logger().info(
            "Dry-run conversion:\n%-22s %8s %9s %5s\n%s\n%s"
            % ("joint", "ros_rad", "servo", "ch", "\n".join(rows), line),
            throttle_duration_sec=2.0,
        )

    def timer_callback(self):
        if not self.dry_run and self.hardware_enabled:
            if not self.connected:
                self.connect()
            self.read_available()
        self.publish_status()

    def publish_status(self):
        message = String()
        age = -1.0
        if self.last_command_time > 0:
            age = self.now_seconds() - self.last_command_time
        clamped = [item["joint"] for item in self.last_details if item["clamped"]]
        message.data = (
            "connected=%d ready=%d armed=%d dry_run=%d hardware_enabled=%d "
            "calibration_valid=%d age=%.3f sent=%d rejected=%d error=%s response=%s "
            "clamped=%s frame=%s"
            % (
                int(self.connected),
                int(self.arduino_ready),
                int(self.hardware_armed),
                int(self.dry_run),
                int(self.hardware_enabled),
                int(self.calibration_valid),
                age,
                self.frames_sent,
                self.frames_rejected,
                self.calibration_error or self.last_error,
                self.last_response,
                ",".join(clamped),
                " ".join("%.2f" % value for value in self.last_frame),
            )
        )
        self.status_publisher.publish(message)

    def shutdown(self):
        if self.serial_port is not None:
            try:
                self.write_line("HOLD")
                self.serial_port.close()
            except SerialException:
                pass
        self.serial_port = None


def main():
    rclpy.init()
    node = VoltSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
