#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from volt_kinematics import JOINT_NAMES


OWNERS = ("MOTION", "MANUAL", "CALIBRATION", "HOLD", "DISABLED")


def validate_joint_values(values, expected_count=len(JOINT_NAMES)):
    """Return a finite float command or raise ``ValueError`` without publishing."""
    try:
        values = list(values)
    except TypeError as exc:
        raise ValueError("joint command must be an iterable") from exc
    if len(values) != expected_count:
        raise ValueError(
            "expected %d joints, got %d" % (expected_count, len(values))
        )

    validated = []
    for index, value in enumerate(values):
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("joint %d is not numeric" % index) from exc
        if not math.isfinite(value):
            raise ValueError("joint %d is not finite" % index)
        validated.append(value)
    return validated


class JointCommandRouter(Node):
    def __init__(self):
        super().__init__("volt_joint_command_router")
        self.declare_parameter("output_topic", "/joint_command_router/output")
        self.declare_parameter("controller_topic", "/joint_group_position_controller/commands")
        self.declare_parameter("owner_topic", "/volt/command_owner")
        self.declare_parameter("status_topic", "/volt/command_router_status")
        self.declare_parameter("stale_timeout", 1.0)

        self.owner = "HOLD"
        self.last_pose = None
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
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 20)
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
        if requested == "HOLD" and self.last_pose is not None:
            self.publish_pose(self.last_pose)

    def validate_pose(self, message, source):
        try:
            return validate_joint_values(message.data)
        except ValueError as exc:
            self.get_logger().warning(
                "Ignoring %s command: %s." % (source, exc),
                throttle_duration_sec=2.0,
            )
            return None

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

    def joint_state_callback(self, message):
        """Seed HOLD from measured feedback without ever emitting startup zeros."""
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in JOINT_NAMES):
            return
        try:
            measured = validate_joint_values(
                [by_name[name] for name in JOINT_NAMES]
            )
        except ValueError:
            return
        if self.last_pose is None or self.owner in ("HOLD", "DISABLED"):
            self.last_pose = measured

    def publish_pose(self, pose):
        pose = validate_joint_values(pose)
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
            # Reassert HOLD rather than publishing only on the transition.  A
            # controller or serial consumer that reconnects while the router is
            # already holding must receive the last finite canonical pose too.
            if self.owner == "HOLD" and self.last_pose is not None:
                self.publish_pose(self.last_pose)
        status = String()
        status.data = (
            "owner=%s controller_connected=%d output_subscribers=%d pose_valid=%d"
            % (
                self.owner,
                int(self.controller_publisher.get_subscription_count() > 0),
                self.output_publisher.get_subscription_count(),
                int(self.last_pose is not None),
            )
        )
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
