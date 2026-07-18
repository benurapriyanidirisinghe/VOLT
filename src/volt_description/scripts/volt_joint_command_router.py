#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from volt_kinematics import JOINT_NAMES


OWNERS = ("MOTION", "MANUAL", "CALIBRATION", "HOLD", "DISABLED")


class JointCommandRouter(Node):
    def __init__(self):
        super().__init__("volt_joint_command_router")
        self.declare_parameter("output_topic", "/joint_command_router/output")
        self.declare_parameter("controller_topic", "/joint_group_position_controller/commands")
        self.declare_parameter("owner_topic", "/volt/command_owner")
        self.declare_parameter("status_topic", "/volt/command_router_status")
        self.declare_parameter("stale_timeout", 1.0)

        self.owner = "HOLD"
        self.last_pose = [0.0 for _ in JOINT_NAMES]
        self.last_owner_command_time = 0.0
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)

        output_topic = str(self.get_parameter("output_topic").value)
        controller_topic = str(self.get_parameter("controller_topic").value)
        owner_topic = str(self.get_parameter("owner_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self.output_publisher = self.create_publisher(Float64MultiArray, output_topic, 10)
        self.controller_publisher = self.create_publisher(Float64MultiArray, controller_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)

        self.create_subscription(String, owner_topic, self.owner_callback, 10)
        self.create_subscription(Float64MultiArray, "/volt/joint_commands/motion", self.motion_callback, 10)
        self.create_subscription(Float64MultiArray, "/volt/joint_commands/manual", self.manual_callback, 10)
        self.create_subscription(Float64MultiArray, "/volt/joint_commands/calibration", self.calibration_callback, 10)
        self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Joint command router owns %s and %s." % (output_topic, controller_topic))

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def owner_callback(self, message):
        requested = message.data.strip().upper()
        if requested not in OWNERS:
            self.get_logger().warning("Ignoring unknown command owner '%s'." % requested)
            return
        if requested != self.owner:
            self.get_logger().info("Command owner changed %s -> %s." % (self.owner, requested))
        self.owner = requested
        self.last_owner_command_time = self.now_seconds()
        if requested in ("HOLD", "DISABLED"):
            self.publish_pose(self.last_pose)

    def validate_pose(self, message, source):
        if len(message.data) != len(JOINT_NAMES):
            self.get_logger().warning(
                "Ignoring %s command: expected %d joints, got %d."
                % (source, len(JOINT_NAMES), len(message.data)),
                throttle_duration_sec=2.0,
            )
            return None
        return [float(value) for value in message.data]

    def source_callback(self, message, source):
        if self.owner != source:
            return
        pose = self.validate_pose(message, source)
        if pose is None:
            return
        self.last_pose = pose
        self.last_owner_command_time = self.now_seconds()
        self.publish_pose(pose)

    def motion_callback(self, message):
        self.source_callback(message, "MOTION")

    def manual_callback(self, message):
        self.source_callback(message, "MANUAL")

    def calibration_callback(self, message):
        self.source_callback(message, "CALIBRATION")

    def publish_pose(self, pose):
        message = Float64MultiArray()
        message.data = list(pose)
        self.output_publisher.publish(message)
        self.controller_publisher.publish(message)

    def timer_callback(self):
        if self.owner != "DISABLED":
            age = self.now_seconds() - self.last_owner_command_time
            if self.owner != "HOLD" and age > self.stale_timeout:
                self.owner = "HOLD"
                self.get_logger().warning("Owner command stale; holding last pose.")
                self.publish_pose(self.last_pose)
        status = String()
        status.data = "owner=%s" % self.owner
        self.status_publisher.publish(status)


def main():
    rclpy.init()
    node = JointCommandRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
