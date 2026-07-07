#!/usr/bin/env python3

"""Bridge ROS joint position commands to the VOLT Arduino servo firmware."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from volt_kinematics import JOINT_NAMES

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - handled at runtime on robot.
    serial = None
    SerialException = Exception


class VoltSerialBridge(Node):
    """Send 12 ROS joint angles to Arduino as newline-terminated RAD packets."""

    def __init__(self):
        super().__init__("volt_serial_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("command_topic", "/joint_group_position_controller/commands")
        self.declare_parameter("status_topic", "/volt/serial_status")
        self.declare_parameter("max_send_rate", 100.0)
        self.declare_parameter("reconnect_period", 1.0)
        self.declare_parameter("serial_timeout", 0.02)
        self.declare_parameter("startup_home", True)

        self.port = str(self.get_parameter("port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.max_send_rate = float(self.get_parameter("max_send_rate").value)
        self.reconnect_period = float(self.get_parameter("reconnect_period").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)
        self.startup_home = bool(self.get_parameter("startup_home").value)
        command_topic = str(self.get_parameter("command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self.serial_port = None
        self.last_send_time = 0.0
        self.last_connect_attempt = 0.0
        self.connected = False
        self.last_error = ""
        self.packet_count = 0
        self.dropped_count = 0

        self.create_subscription(
            Float64MultiArray,
            command_topic,
            self.command_callback,
            10,
        )
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.create_timer(0.5, self.timer_callback)

        if serial is None:
            self.last_error = "pyserial is not installed; install python3-serial."
            self.get_logger().error(self.last_error)
        else:
            self.get_logger().info(
                "Serial bridge ready: %s @ %d baud, topic %s"
                % (self.port, self.baud_rate, command_topic)
            )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def connect(self):
        if serial is None:
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
            self.last_error = ""
            if self.startup_home:
                self.write_line("HOME")
            self.get_logger().info("Connected to Arduino on %s." % self.port)
            return True
        except SerialException as exc:
            self.connected = False
            self.serial_port = None
            self.last_error = str(exc)
            self.get_logger().warning(
                "Waiting for Arduino on %s: %s" % (self.port, exc),
                throttle_duration_sec=5.0,
            )
            return False

    def disconnect(self, reason):
        self.connected = False
        self.last_error = reason
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except SerialException:
                pass
        self.serial_port = None
        self.get_logger().warning("Arduino serial disconnected: %s" % reason)

    def write_line(self, line):
        if self.serial_port is None:
            raise SerialException("serial port is not open")
        self.serial_port.write((line + "\n").encode("ascii"))

    def command_callback(self, message):
        if len(message.data) != len(JOINT_NAMES):
            self.dropped_count += 1
            self.get_logger().warning(
                "Expected %d joints, got %d."
                % (len(JOINT_NAMES), len(message.data)),
                throttle_duration_sec=2.0,
            )
            return

        if not self.connected and not self.connect():
            self.dropped_count += 1
            return

        now = self.now_seconds()
        min_period = 1.0 / max(self.max_send_rate, 1.0)
        if now - self.last_send_time < min_period:
            return
        self.last_send_time = now

        # The Arduino firmware expects ROS joint angles in radians. It applies
        # the 90 degree neutral offset, trims, direction signs, and servo limits.
        line = "RAD " + " ".join("%.6f" % float(value) for value in message.data)
        try:
            self.write_line(line)
            self.packet_count += 1
        except SerialException as exc:
            self.dropped_count += 1
            self.disconnect(str(exc))

    def timer_callback(self):
        if not self.connected:
            self.connect()
        self.publish_status()

    def publish_status(self):
        message = String()
        state = "connected" if self.connected else "disconnected"
        message.data = (
            "state=%s port=%s packets=%d dropped=%d error=%s"
            % (
                state,
                self.port,
                self.packet_count,
                self.dropped_count,
                self.last_error,
            )
        )
        self.status_publisher.publish(message)

    def shutdown(self):
        if self.serial_port is not None:
            try:
                self.write_line("HOME")
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
