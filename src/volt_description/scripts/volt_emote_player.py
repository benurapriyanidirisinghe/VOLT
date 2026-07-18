#!/usr/bin/env python3

"""One-shot YAML emote player for the VOLT position command path."""

import json
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from volt_kinematics import (
    FOOT_LIMIT,
    JOINT_NAMES,
    LEG_LIMIT,
    SHOULDER_LIMIT,
    clamp,
    smoothstep,
)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


LINEAR_MOVING_THRESHOLD = 0.002
ANGULAR_MOVING_THRESHOLD = 0.025


def joint_limit(joint_name):
    if joint_name.endswith("_shoulder"):
        return SHOULDER_LIMIT
    if joint_name.endswith("_leg"):
        return LEG_LIMIT
    if joint_name.endswith("_foot"):
        return FOOT_LIMIT
    raise ValueError("Unknown joint type for '%s'." % joint_name)


def interpolate_positions(start, end, proportion):
    blend = smoothstep(proportion)
    return [
        first + (second - first) * blend
        for first, second in zip(start, end)
    ]


class EmoteValidationError(ValueError):
    pass


class VoltEmotePlayer(Node):
    def __init__(self):
        super().__init__("volt_emote_player")
        self.declare_parameter("emote", "stand_ready")
        self.declare_parameter("emote_file", "")
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter(
            "command_topic",
            "/volt/joint_commands/manual",
        )
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/volt/status")
        self.declare_parameter("action_topic", "/volt/action")
        self.declare_parameter("owner_topic", "/volt/command_owner")
        self.declare_parameter("request_stop_before_play", True)
        self.declare_parameter("stop_settle_time", 0.6)
        self.declare_parameter("idle_timeout", 0.45)
        self.declare_parameter("wait_for_subscribers", 5.0)

        self.emote_name = str(self.get_parameter("emote").value)
        self.emote_file = str(self.get_parameter("emote_file").value)
        self.speed_scale = clamp(
            float(self.get_parameter("speed_scale").value),
            0.1,
            3.0,
        )
        self.publish_rate = clamp(
            float(self.get_parameter("publish_rate").value),
            10.0,
            200.0,
        )
        self.request_stop_before_play = bool(
            self.get_parameter("request_stop_before_play").value
        )
        self.stop_settle_time = max(
            0.0,
            float(self.get_parameter("stop_settle_time").value),
        )
        self.idle_timeout = max(
            0.05,
            float(self.get_parameter("idle_timeout").value),
        )
        self.wait_for_subscribers = max(
            0.0,
            float(self.get_parameter("wait_for_subscribers").value),
        )

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.owner_publisher = self.create_publisher(
            String,
            str(self.get_parameter("owner_topic").value),
            10,
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.action_publisher = self.create_publisher(
            String,
            str(self.get_parameter("action_topic").value),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_vel_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("status_topic").value),
            self.status_callback,
            10,
        )

        self.last_cmd_vel_time = None
        self.cmd_vel_active = False
        self.last_status_time = None
        self.status_moving = False
        self.last_positions = None

    def monotonic(self):
        return time.monotonic()

    def cmd_vel_callback(self, message):
        self.last_cmd_vel_time = self.monotonic()
        linear_speed = max(abs(message.linear.x), abs(message.linear.y))
        angular_speed = abs(message.angular.z)
        self.cmd_vel_active = (
            linear_speed > LINEAR_MOVING_THRESHOLD
            or angular_speed > ANGULAR_MOVING_THRESHOLD
        )

    def status_callback(self, message):
        self.last_status_time = self.monotonic()
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            self.status_moving = False
            return
        filtered = status.get("filtered_velocity", [0.0, 0.0, 0.0])
        moving_velocity = (
            len(filtered) >= 3
            and (
                max(abs(float(filtered[0])), abs(float(filtered[1])))
                > LINEAR_MOVING_THRESHOLD
                or abs(float(filtered[2])) > ANGULAR_MOVING_THRESHOLD
            )
        )
        self.status_moving = bool(status.get("moving", False)) or moving_velocity

    def recent_cmd_vel_active(self):
        if self.last_cmd_vel_time is None:
            return False
        if self.monotonic() - self.last_cmd_vel_time > self.idle_timeout:
            return False
        return self.cmd_vel_active

    def recent_status_moving(self):
        if self.last_status_time is None:
            return False
        if self.monotonic() - self.last_status_time > self.idle_timeout:
            return False
        return self.status_moving

    def movement_active(self):
        return self.recent_cmd_vel_active() or self.recent_status_moving()

    def resolve_emote_path(self):
        if self.emote_file:
            return os.path.abspath(os.path.expanduser(self.emote_file))
        if os.path.exists(self.emote_name):
            return os.path.abspath(os.path.expanduser(self.emote_name))

        filename = self.emote_name
        if not filename.endswith((".yaml", ".yml")):
            filename += ".yaml"

        candidates = []
        if get_package_share_directory is not None:
            try:
                candidates.append(
                    os.path.join(
                        get_package_share_directory("volt_description"),
                        "emotes",
                        filename,
                    )
                )
            except Exception:
                pass
        candidates.append(
            os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    "..",
                    "emotes",
                    filename,
                )
            )
        )

        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        return candidates[0]

    def validate_emote(self, emote):
        for key in ("name", "description", "joint_names", "points"):
            if key not in emote:
                raise EmoteValidationError("Emote YAML is missing '%s'." % key)

        joint_names = list(emote["joint_names"])
        if joint_names != JOINT_NAMES:
            raise EmoteValidationError(
                "joint_names must exactly match the VOLT 12-joint order."
            )

        points = emote["points"]
        if not isinstance(points, list) or not points:
            raise EmoteValidationError("points must be a non-empty list.")

        validated = []
        previous_time = -1.0
        for index, point in enumerate(points):
            if "time_from_start" not in point or "positions" not in point:
                raise EmoteValidationError(
                    "Point %d must contain time_from_start and positions." % index
                )
            point_time = float(point["time_from_start"])
            if point_time < previous_time:
                raise EmoteValidationError(
                    "Point %d time_from_start goes backwards." % index
                )
            positions = [float(value) for value in point["positions"]]
            if len(positions) != len(JOINT_NAMES):
                raise EmoteValidationError(
                    "Point %d has %d positions; expected %d."
                    % (index, len(positions), len(JOINT_NAMES))
                )
            for joint_name, value in zip(JOINT_NAMES, positions):
                lower, upper = joint_limit(joint_name)
                if value < lower - 1e-6 or value > upper + 1e-6:
                    raise EmoteValidationError(
                        "%s value %.3f is outside [%.3f, %.3f] at point %d."
                        % (joint_name, value, lower, upper, index)
                    )
            previous_time = point_time
            validated.append({
                "time_from_start": point_time,
                "positions": positions,
            })

        return {
            "name": str(emote["name"]),
            "description": str(emote["description"]),
            "joint_names": joint_names,
            "points": validated,
        }

    def load_emote(self):
        emote_path = self.resolve_emote_path()
        if not os.path.exists(emote_path):
            raise EmoteValidationError("Emote file not found: %s" % emote_path)
        with open(emote_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        if not isinstance(data, dict):
            raise EmoteValidationError("Emote YAML must contain a mapping.")
        emote = self.validate_emote(data)
        self.get_logger().info(
            "Loaded emote '%s' from %s." % (emote["name"], emote_path)
        )
        return emote

    def wait_for_command_subscriber(self):
        deadline = self.monotonic() + self.wait_for_subscribers
        while rclpy.ok() and self.command_publisher.get_subscription_count() == 0:
            if self.monotonic() >= deadline:
                self.get_logger().error(
                    "No subscribers on the joint command topic; emote not played."
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def other_command_publishers(self):
        publishers = self.get_publishers_info_by_topic(self.command_topic)
        return [
            "%s/%s" % (info.node_namespace.rstrip("/"), info.node_name)
            for info in publishers
            if info.node_name != self.get_name()
        ]

    def publish_stop_request(self):
        twist = Twist()
        action = String()
        action.data = "stop"
        start = self.monotonic()
        while rclpy.ok() and self.monotonic() - start < self.stop_settle_time:
            self.cmd_vel_publisher.publish(twist)
            self.action_publisher.publish(action)
            rclpy.spin_once(self, timeout_sec=0.05)

    def positions_at(self, points, emote_time):
        if emote_time <= points[0]["time_from_start"]:
            return list(points[0]["positions"])
        for start, end in zip(points[:-1], points[1:]):
            start_time = start["time_from_start"]
            end_time = end["time_from_start"]
            if emote_time <= end_time:
                duration = max(end_time - start_time, 1e-6)
                proportion = (emote_time - start_time) / duration
                return interpolate_positions(
                    start["positions"],
                    end["positions"],
                    proportion,
                )
        return list(points[-1]["positions"])

    def publish_positions(self, positions):
        owner = String()
        owner.data = "MANUAL"
        self.owner_publisher.publish(owner)
        message = Float64MultiArray()
        message.data = list(positions)
        self.command_publisher.publish(message)
        self.last_positions = list(positions)

    def play_once(self):
        try:
            emote = self.load_emote()
        except EmoteValidationError as error:
            self.get_logger().error(str(error))
            return False

        if self.request_stop_before_play:
            self.get_logger().info("Requesting stop before emote playback.")
            self.publish_stop_request()

        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.movement_active():
            self.get_logger().error(
                "Robot is still moving; rejecting emote '%s'." % emote["name"]
            )
            return False

        if not self.wait_for_command_subscriber():
            return False
        other_publishers = self.other_command_publishers()
        if other_publishers:
            self.get_logger().error(
                "Another node is publishing joint commands (%s); emote not played."
                % ", ".join(other_publishers)
            )
            return False

        points = emote["points"]
        total_time = points[-1]["time_from_start"]
        period = 1.0 / self.publish_rate
        start = self.monotonic()
        self.get_logger().info(
            "Playing emote '%s' at speed_scale %.2f."
            % (emote["name"], self.speed_scale)
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.movement_active():
                self.get_logger().error(
                    "Movement command detected; aborting emote '%s'."
                    % emote["name"]
                )
                return False

            elapsed = self.monotonic() - start
            emote_time = min(elapsed * self.speed_scale, total_time)
            self.publish_positions(self.positions_at(points, emote_time))
            if emote_time >= total_time:
                break
            time.sleep(period)

        self.publish_positions(points[-1]["positions"])
        self.get_logger().info("Finished emote '%s'." % emote["name"])
        return True


def main():
    rclpy.init()
    node = VoltEmotePlayer()
    try:
        node.play_once()
    except KeyboardInterrupt:
        node.get_logger().info("Emote playback interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
