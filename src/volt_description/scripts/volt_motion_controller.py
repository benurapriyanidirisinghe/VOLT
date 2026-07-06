#!/usr/bin/env python3

import json
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from volt_gait_controller import GAITS, VoltGaitController
from volt_kinematics import (
    JOINT_NAMES,
    JOINT_VELOCITY_LIMITS,
    LEG_FOOT_MID_POSE,
    LEG_FOOT_SIT_POSE,
    LEG_ORDER,
    NOMINAL_FEET,
    SIT_POSE,
    WALK_POSE,
    clamp,
    feet_to_joint_positions,
    interpolate,
)


class VoltMotionController(Node):
    def __init__(self):
        super().__init__("volt_motion_controller")

        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("command_timeout", 0.60)
        self.declare_parameter("joint_error_limit", 0.45)
        self.declare_parameter("stand_duration", 4.0)
        self.declare_parameter("sit_duration", 4.7)
        self.declare_parameter("auto_ready_pose", True)
        self.declare_parameter("body_height", 0.200)
        self.declare_parameter("debug_gait", False)
        self.declare_parameter("max_joint_velocity", 4.0)
        self.declare_parameter("max_joint_acceleration", 18.0)
        self.declare_parameter("joint_smoothing_factor", 0.90)

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.joint_error_limit = float(
            self.get_parameter("joint_error_limit").value
        )
        self.debug_gait = bool(self.get_parameter("debug_gait").value)
        self.max_joint_velocity = float(
            self.get_parameter("max_joint_velocity").value
        )
        self.max_joint_acceleration = float(
            self.get_parameter("max_joint_acceleration").value
        )
        self.joint_smoothing_factor = clamp(
            float(self.get_parameter("joint_smoothing_factor").value),
            0.05,
            1.0,
        )

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            "/joint_group_position_controller/commands",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/volt/status",
            10,
        )
        self.create_subscription(Twist, "/cmd_vel", self.velocity_callback, 10)
        self.create_subscription(String, "/volt/action", self.action_callback, 10)
        self.create_subscription(String, "/volt/gait", self.gait_callback, 10)
        self.create_subscription(
            Twist,
            "/volt/body_pose",
            self.body_pose_callback,
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            20,
        )

        self.state = "waiting"
        self.gait_name = "walk"
        self.step_in_place = False
        self.requested_velocity = [0.0, 0.0, 0.0]
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.last_velocity_time = self.now_seconds()
        self.last_update_time = self.now_seconds()
        self.motion_active = False
        self.pending_gait = None
        self.gait_controller = VoltGaitController()
        self.auto_ready_pose = bool(
            self.get_parameter("auto_ready_pose").value
        )
        self.auto_ready_requested = False
        self.auto_ready_pending = False

        self.body_height = clamp(
            float(self.get_parameter("body_height").value),
            0.175,
            0.220,
        )
        self.body_x = 0.0
        self.body_y = 0.0
        self.body_roll = 0.0
        self.body_pitch = 0.0
        self.body_yaw = 0.0

        self.measured_positions = None
        self.commanded_positions = None
        self.commanded_velocities = None
        self.transition = None
        self.warning = ""
        self.last_status_time = 0.0
        self.last_debug_time = 0.0

        period = 1.0 / max(self.control_rate, 1.0)
        self.timer = self.create_timer(period, self.control_callback)
        self.get_logger().info(
            "VOLT controller ready; waiting for joint states and position controller."
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def velocity_callback(self, message):
        self.requested_velocity = [
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        ]
        self.last_velocity_time = self.now_seconds()

    def gait_callback(self, message):
        requested = message.data.strip().lower()
        if requested not in GAITS:
            self.get_logger().warning("Unknown gait '%s'." % requested)
            return
        if requested != self.gait_name:
            if self.motion_active:
                self.pending_gait = requested
                self.gait_controller.request_stop()
                self.get_logger().info(
                    "Stopping %s before switching to %s."
                    % (self.gait_name, requested)
                )
            else:
                self.gait_name = requested
                self.gait_controller.set_gait(requested, self.now_seconds())
                self.get_logger().info("Selected %s gait." % requested)

    def body_pose_callback(self, message):
        self.body_x = clamp(float(message.linear.x), -0.025, 0.025)
        self.body_y = clamp(float(message.linear.y), -0.020, 0.020)
        self.body_height = clamp(float(message.linear.z), 0.175, 0.220)
        self.body_roll = clamp(float(message.angular.x), -0.16, 0.16)
        self.body_pitch = clamp(float(message.angular.y), -0.16, 0.16)
        self.body_yaw = clamp(float(message.angular.z), -0.18, 0.18)

    def action_callback(self, message):
        action = message.data.strip().lower()
        if action == "stand":
            self.start_stand_transition()
        elif action == "sit":
            self.start_sit_transition()
        elif action == "stop":
            self.stop_motion()
        elif action == "step":
            self.step_in_place = not self.step_in_place
            if not self.step_in_place:
                self.gait_controller.request_stop()
        elif action == "debug_on":
            self.debug_gait = True
            self.get_logger().info("Gait debug logging enabled.")
        elif action == "debug_off":
            self.debug_gait = False
            self.get_logger().info("Gait debug logging disabled.")
        else:
            self.get_logger().warning("Unknown action '%s'." % action)

    def joint_state_callback(self, message):
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in JOINT_NAMES):
            return
        self.measured_positions = [by_name[name] for name in JOINT_NAMES]
        if self.commanded_positions is None:
            self.commanded_positions = list(self.measured_positions)
            self.commanded_velocities = [0.0 for _ in JOINT_NAMES]
            self.state = "hold"
            self.get_logger().info("Joint feedback received; holding current pose.")
            if self.auto_ready_pose and not self.auto_ready_requested:
                self.auto_ready_requested = True
                self.auto_ready_pending = True
                self.get_logger().info("Auto-starting walk-ready pose.")

    def stop_motion(self):
        self.requested_velocity = [0.0, 0.0, 0.0]
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.step_in_place = False
        self.gait_controller.request_stop()

    def start_stand_transition(self):
        if self.commanded_positions is None:
            self.warning = "Cannot stand until joint feedback is available."
            return
        self.stop_motion()
        duration = float(self.get_parameter("stand_duration").value)
        sit_error = max(
            abs(current - sit)
            for current, sit in zip(self.commanded_positions, SIT_POSE)
        )
        if sit_error < 0.35:
            scale = duration / 4.0
            waypoints = [
                (0.45 * scale, SIT_POSE),
                (1.20 * scale, LEG_FOOT_SIT_POSE),
                (2.55 * scale, LEG_FOOT_MID_POSE),
                (4.00 * scale, WALK_POSE),
            ]
        else:
            waypoints = [(duration, WALK_POSE)]
        self.begin_transition(waypoints, "standing")
        self.state = "standing_up"
        self.get_logger().info("Standing up.")

    def start_sit_transition(self):
        if self.commanded_positions is None:
            self.warning = "Cannot sit until joint feedback is available."
            return
        self.stop_motion()
        duration = float(self.get_parameter("sit_duration").value)
        scale = duration / 4.7
        self.begin_transition(
            [
                (0.70 * scale, WALK_POSE),
                (2.20 * scale, LEG_FOOT_MID_POSE),
                (3.70 * scale, LEG_FOOT_SIT_POSE),
                (4.70 * scale, SIT_POSE),
            ],
            "sitting",
        )
        self.state = "sitting_down"
        self.get_logger().info("Sitting down.")

    def begin_transition(self, waypoints, final_state):
        start = (
            list(self.measured_positions)
            if self.measured_positions is not None
            else list(self.commanded_positions)
        )
        self.transition = {
            "start_time": self.now_seconds(),
            "start": start,
            "waypoints": [(float(t), list(pose)) for t, pose in waypoints],
            "final_state": final_state,
        }

    def transition_target(self, now):
        transition = self.transition
        elapsed = now - transition["start_time"]
        segment_start_time = 0.0
        segment_start = transition["start"]

        for segment_end_time, segment_end in transition["waypoints"]:
            if elapsed <= segment_end_time:
                duration = max(segment_end_time - segment_start_time, 1e-6)
                proportion = (elapsed - segment_start_time) / duration
                return interpolate(segment_start, segment_end, proportion)
            segment_start_time = segment_end_time
            segment_start = segment_end

        self.transition = None
        self.state = transition["final_state"]
        if self.state == "standing":
            self.gait_controller.reset(now)
            self.motion_active = False
        return list(segment_start)

    def standing_target(self):
        return feet_to_joint_positions(
            NOMINAL_FEET,
            height=self.body_height,
            body_x=self.body_x,
            body_y=self.body_y,
            roll=self.body_roll,
            pitch=self.body_pitch,
            yaw=self.body_yaw,
        )

    def gait_target(self, now, dt):
        feet, support_shift, active = self.gait_controller.step(
            now,
            dt,
            tuple(self.filtered_velocity),
            self.step_in_place,
        )
        self.motion_active = active
        joints = feet_to_joint_positions(
            feet,
            height=self.body_height,
            body_x=self.body_x + support_shift[0],
            body_y=self.body_y + support_shift[1],
            roll=self.body_roll,
            pitch=self.body_pitch,
            yaw=self.body_yaw,
        )
        self.log_gait_debug(now, feet, joints)
        return joints

    def joint_velocity_limit(self, index):
        return min(self.max_joint_velocity, JOINT_VELOCITY_LIMITS[index])

    def smooth_joint_target(self, target, dt):
        """Low-pass, velocity-limit, and acceleration-limit joint commands."""
        target = list(target)
        if self.commanded_positions is None:
            return target
        if self.commanded_velocities is None:
            self.commanded_velocities = [0.0 for _ in target]

        smoothed = []
        for index, raw_target in enumerate(target):
            current = self.commanded_positions[index]
            blended_target = current + (
                raw_target - current
            ) * self.joint_smoothing_factor

            desired_velocity = (blended_target - current) / max(dt, 1e-6)
            velocity_limit = self.joint_velocity_limit(index)
            desired_velocity = clamp(
                desired_velocity,
                -velocity_limit,
                velocity_limit,
            )

            current_velocity = self.commanded_velocities[index]
            velocity_step = self.max_joint_acceleration * dt
            next_velocity = current_velocity + clamp(
                desired_velocity - current_velocity,
                -velocity_step,
                velocity_step,
            )

            next_position = current + next_velocity * dt
            if (raw_target - current) * (raw_target - next_position) < 0.0:
                next_position = raw_target
                next_velocity = 0.0

            self.commanded_velocities[index] = next_velocity
            smoothed.append(next_position)
        return smoothed

    def log_gait_debug(self, now, feet, joints):
        if not self.debug_gait or now - self.last_debug_time < 0.50:
            return
        self.last_debug_time = now
        debug = self.gait_controller.debug_snapshot()
        feet_debug = {
            leg: tuple(round(value, 3) for value in feet[leg])
            for leg in LEG_ORDER
        }
        joint_debug = {
            name: round(angle, 3)
            for name, angle in zip(JOINT_NAMES, joints)
        }
        self.get_logger().info(
            "gait=%s phase=%.3f swing=%s stance=%s feet=%s joints=%s"
            % (
                self.gait_name,
                debug["phase"],
                debug["swing_legs"],
                debug["stance_legs"],
                feet_debug,
                joint_debug,
            )
        )

    def update_filtered_velocity(self, now, dt):
        if self.pending_gait is not None:
            desired = [0.0, 0.0, 0.0]
        elif now - self.last_velocity_time > self.command_timeout:
            desired = [0.0, 0.0, 0.0]
        else:
            desired = list(self.requested_velocity)

        gait = GAITS[self.gait_name]
        limits = [gait["max_x"], gait["max_y"], gait["max_yaw"]]
        rates = [
            max(0.18, gait["max_x"] * 3.0),
            max(0.12, gait["max_y"] * 3.0),
            max(1.20, gait["max_yaw"] * 3.0),
        ]
        smoothing = clamp(gait.get("smoothing_factor", 0.15), 0.02, 1.0)
        for index in range(3):
            desired[index] = clamp(desired[index], -limits[index], limits[index])
            difference = desired[index] - self.filtered_velocity[index]
            maximum_change = rates[index] * dt
            ramped_velocity = self.filtered_velocity[index] + clamp(
                difference,
                -maximum_change,
                maximum_change,
            )
            self.filtered_velocity[index] += (
                ramped_velocity - self.filtered_velocity[index]
            ) * smoothing

    def control_callback(self):
        now = self.now_seconds()
        dt = clamp(now - self.last_update_time, 0.001, 0.100)
        self.last_update_time = now

        if self.commanded_positions is None:
            self.publish_status(now)
            return
        if self.command_publisher.get_subscription_count() == 0:
            self.warning = "Position controller is not connected."
            self.publish_status(now)
            return
        if self.warning == "Position controller is not connected.":
            self.warning = ""

        if self.auto_ready_pending and self.state == "hold":
            self.auto_ready_pending = False
            self.start_stand_transition()

        self.update_filtered_velocity(now, dt)

        if self.transition is not None:
            target = self.transition_target(now)
            self.motion_active = False
        elif self.state == "standing":
            moving = (
                abs(self.filtered_velocity[0]) > 0.0015
                or abs(self.filtered_velocity[1]) > 0.0015
                or abs(self.filtered_velocity[2]) > 0.025
                or self.step_in_place
            )
            if moving or self.gait_controller.active:
                target = self.gait_target(now, dt)
            else:
                target = self.standing_target()
                self.motion_active = False
        else:
            target = list(self.commanded_positions)
            self.motion_active = False

        target = self.smooth_joint_target(target, dt)
        self.commanded_positions = target

        if self.pending_gait is not None and not self.gait_controller.active:
            self.gait_name = self.pending_gait
            self.pending_gait = None
            self.gait_controller.set_gait(self.gait_name, now)
            self.get_logger().info("Selected %s gait." % self.gait_name)

        message = Float64MultiArray()
        message.data = target
        self.command_publisher.publish(message)

        self.update_feedback_warning()
        self.publish_status(now)

    def update_feedback_warning(self):
        if self.measured_positions is None or self.commanded_positions is None:
            return
        maximum_error = max(
            abs(command - measured)
            for command, measured in zip(
                self.commanded_positions,
                self.measured_positions,
            )
        )
        if maximum_error > self.joint_error_limit:
            self.warning = "Large joint tracking error: %.2f rad" % maximum_error
        elif self.warning.startswith("Large joint"):
            self.warning = ""

    def publish_status(self, now):
        if now - self.last_status_time < 0.10:
            return
        self.last_status_time = now
        maximum_error = None
        if self.measured_positions is not None and self.commanded_positions is not None:
            maximum_error = max(
                abs(command - measured)
                for command, measured in zip(
                    self.commanded_positions,
                    self.measured_positions,
                )
            )
        debug = self.gait_controller.debug_snapshot()
        status = {
            "state": self.state,
            "gait": self.gait_name,
            "moving": self.motion_active,
            "step_in_place": self.step_in_place,
            "gait_phase": debug["phase"],
            "swing_legs": debug["swing_legs"],
            "stance_legs": debug["stance_legs"],
            "filtered_velocity": list(self.filtered_velocity),
            "controller_connected": self.command_publisher.get_subscription_count() > 0,
            "joint_error": maximum_error,
            "warning": self.warning,
        }
        message = String()
        message.data = json.dumps(status)
        self.status_publisher.publish(message)


def main():
    rclpy.init()
    node = VoltMotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
