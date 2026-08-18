#!/usr/bin/env python3

"""One-shot compatibility client for the authoritative VOLT controller.

This executable intentionally owns no joint-space playback logic.  Legacy
names are translated to controller actions or to correlated Cartesian emote
requests, while the motion controller remains the only joint-command source.
"""

import json
import math
import sys
import time
import uuid
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from volt_emote_engine import (
    MAX_REPETITIONS,
    MAX_SCALE,
    MAX_DEPTH_SCALE,
    MAX_SPEED,
    MIN_REPETITIONS,
    MIN_SCALE,
    MIN_SPEED,
    EmoteOptions,
)


ACTION_ALIASES = {
    "stand_ready": "stand",
    "sit": "sit",
}
CARTESIAN_ALIASES = {
    "bow": "bow",
    "small_dance": "happy_dance",
}
EMOTE_RESULTS = {
    "queued",
    "running",
    "returning",
    "settling",
    "completed",
    "cancelled",
    "rejected",
}
TERMINAL_EMOTE_RESULTS = {"completed", "cancelled", "rejected"}
EMOTE_KEEPALIVE_PERIOD = 0.20


class EmotePlayerError(ValueError):
    """Raised when a compatibility request cannot be sent safely."""


@dataclass(frozen=True)
class ControllerRequest:
    kind: str
    name: str


def _finite_number(value, label):
    if isinstance(value, bool):
        raise EmotePlayerError("%s must be numeric" % label)
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise EmotePlayerError("%s must be numeric" % label) from exc
    if not math.isfinite(number):
        raise EmotePlayerError("%s must be finite" % label)
    return number


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def clamp_emote_options(repetitions=1, speed=1.0, amplitude=1.0, depth=1.0):
    """Normalize legacy CLI values to the engine's public safety envelope."""
    if isinstance(repetitions, bool) or not isinstance(repetitions, int):
        raise EmotePlayerError("repetitions must be an integer")
    repetition_count = repetitions
    repetition_count = int(
        _clamp(repetition_count, MIN_REPETITIONS, MAX_REPETITIONS)
    )
    return EmoteOptions(
        repetitions=repetition_count,
        speed=_clamp(_finite_number(speed, "speed_scale"), MIN_SPEED, MAX_SPEED),
        amplitude=_clamp(
            _finite_number(amplitude, "amplitude"),
            MIN_SCALE,
            MAX_SCALE,
        ),
        depth=_clamp(
            _finite_number(depth, "depth"), MIN_SCALE, MAX_DEPTH_SCALE
        ),
    )


def resolve_controller_request(emote, emote_file=""):
    """Map a legacy selector without loading or interpreting joint YAML."""
    if str(emote_file).strip():
        raise EmotePlayerError(
            "custom emote_file playback is no longer supported; add the "
            "Cartesian emote to the motion-controller catalog instead"
        )
    name = str(emote).strip().lower()
    if not name:
        raise EmotePlayerError("emote must be a non-empty name")
    if (
        "/" in name
        or "\\" in name
        or name.endswith((".yaml", ".yml"))
    ):
        raise EmotePlayerError(
            "custom emote paths are no longer supported; use a named "
            "motion-controller Cartesian emote"
        )
    if name in ACTION_ALIASES:
        return ControllerRequest("action", ACTION_ALIASES[name])
    return ControllerRequest("emote", CARTESIAN_ALIASES.get(name, name))


def encode_start_request(request_id, name, options):
    """Create the controller's exact, finite start-request JSON schema."""
    payload = {
        "command": "start",
        "request_id": str(request_id),
        "name": str(name),
        "repetitions": int(options.repetitions),
        "speed": float(options.speed),
        "amplitude": float(options.amplitude),
        "depth": float(options.depth),
    }
    return json.dumps(
        payload,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    )


def encode_cancel_request(request_id):
    """Create the controller's exact correlated cancel-request JSON."""
    return json.dumps(
        {"command": "cancel", "request_id": str(request_id)},
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    )


def encode_keepalive_request(request_id):
    """Create the exact correlated lease-renewal JSON."""
    return json.dumps(
        {"command": "keepalive", "request_id": str(request_id)},
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    )


def status_gate_error(status, age, timeout, emote_name=None, require_idle=False):
    """Return a human-readable safety gate failure, or an empty string."""
    if not isinstance(status, dict):
        return "no valid /volt/status has been received"
    if age < 0.0 or age > timeout:
        return "/volt/status is stale (age %.2fs, limit %.2fs)" % (age, timeout)
    owner = str(status.get("command_owner", "")).strip().upper()
    if owner != "MOTION":
        return "command owner is %s; explicitly enable MOTION first" % (
            owner or "unknown"
        )
    if emote_name is not None:
        state = str(status.get("state", "")).strip().lower()
        if state != "standing":
            return "controller state is %s; Cartesian emotes require standing" % (
                state or "unknown"
            )
        available = status.get("emotes_available")
        advertised = {
            str(item).strip().lower()
            for item in available
        } if isinstance(available, (list, tuple)) else set()
        if emote_name not in advertised:
            return "Cartesian emote '%s' is not advertised by the controller" % (
                emote_name
            )
        if require_idle and (
            bool(status.get("motion_active", status.get("moving", False)))
            or bool(status.get("emote_active", False))
            or bool(status.get("emote_pending", False))
            or bool(status.get("physical_test_active", False))
        ):
            return "controller has not reached an idle standing state after STOP"
    return ""


def correlated_emote_result(status, request_id):
    """Return a recognized result only when status matches this request."""
    if not isinstance(status, dict):
        return None
    if str(status.get("emote_request_id", "")).strip() != str(request_id):
        return None
    result = str(status.get("emote_result", "")).strip().lower()
    return result if result in EMOTE_RESULTS else None


class VoltEmotePlayer(Node):
    """Translate one legacy invocation into authoritative controller messages."""

    def __init__(self):
        super().__init__("volt_emote_player")
        self.declare_parameter("emote", "stand_ready")
        self.declare_parameter("emote_file", "")
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("repetitions", 1)
        self.declare_parameter("amplitude", 1.0)
        self.declare_parameter("depth", 1.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/volt/status")
        self.declare_parameter("action_topic", "/volt/action")
        self.declare_parameter("emote_topic", "/volt/emote")
        self.declare_parameter("stop_settle_time", 0.6)
        self.declare_parameter("status_timeout", 3.0)
        self.declare_parameter("wait_for_subscribers", 5.0)
        self.declare_parameter("acknowledgement_timeout", 5.0)
        self.declare_parameter("completion_timeout", 620.0)
        self.declare_parameter("action_timeout", 20.0)

        self.emote_name = str(self.get_parameter("emote").value)
        self.emote_file = str(self.get_parameter("emote_file").value)
        requested_options = (
            self.get_parameter("repetitions").value,
            self.get_parameter("speed_scale").value,
            self.get_parameter("amplitude").value,
            self.get_parameter("depth").value,
        )
        self.options = clamp_emote_options(
            *requested_options,
        )
        normalized_options = (
            self.options.repetitions,
            self.options.speed,
            self.options.amplitude,
            self.options.depth,
        )
        if any(
            float(requested) != float(normalized)
            for requested, normalized in zip(
                requested_options,
                normalized_options,
            )
        ):
            self.get_logger().warning(
                "Clamped emote options to repetitions=%d speed=%.2f "
                "amplitude=%.2f depth=%.2f."
                % normalized_options
            )
        self.stop_settle_time = max(
            0.0,
            _finite_number(
                self.get_parameter("stop_settle_time").value,
                "stop_settle_time",
            ),
        )
        self.status_timeout = max(
            0.1,
            _finite_number(
                self.get_parameter("status_timeout").value,
                "status_timeout",
            ),
        )
        self.wait_for_subscribers = max(
            0.0,
            _finite_number(
                self.get_parameter("wait_for_subscribers").value,
                "wait_for_subscribers",
            ),
        )
        self.acknowledgement_timeout = max(
            0.1,
            _finite_number(
                self.get_parameter("acknowledgement_timeout").value,
                "acknowledgement_timeout",
            ),
        )
        self.completion_timeout = max(
            self.acknowledgement_timeout,
            _finite_number(
                self.get_parameter("completion_timeout").value,
                "completion_timeout",
            ),
        )
        self.action_timeout = max(
            0.1,
            _finite_number(
                self.get_parameter("action_timeout").value,
                "action_timeout",
            ),
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
        self.emote_publisher = self.create_publisher(
            String,
            str(self.get_parameter("emote_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("status_topic").value),
            self.status_callback,
            10,
        )

        self.last_status = None
        self.last_status_time = 0.0
        self.status_generation = 0
        self.request_id = ""
        self.last_reported_result = None

    def monotonic(self):
        return time.monotonic()

    def status_callback(self, message):
        try:
            decoded = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(decoded, dict):
            return
        self.last_status = decoded
        self.last_status_time = self.monotonic()
        self.status_generation += 1

    def status_age(self):
        if self.last_status_time <= 0.0:
            return float("inf")
        return self.monotonic() - self.last_status_time

    def spin_until(self, predicate, timeout):
        """Service callbacks while waiting; never sleep outside ROS spinning."""
        deadline = self.monotonic() + max(0.0, float(timeout))
        while rclpy.ok():
            if predicate():
                return True
            remaining = deadline - self.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))
        return False

    def spin_for(self, duration):
        deadline = self.monotonic() + max(0.0, float(duration))
        return self.spin_until(lambda: self.monotonic() >= deadline, duration)

    def wait_for_controller(self, include_emote):
        required = [
            ("/cmd_vel", self.cmd_vel_publisher),
            ("/volt/action", self.action_publisher),
        ]
        if include_emote:
            required.append(("/volt/emote", self.emote_publisher))

        def connected():
            return all(
                publisher.get_subscription_count() > 0
                for _topic, publisher in required
            )

        if self.spin_until(connected, self.wait_for_subscribers):
            return True
        missing = [
            topic
            for topic, publisher in required
            if publisher.get_subscription_count() == 0
        ]
        self.get_logger().error(
            "No motion-controller subscriber on %s."
            % ", ".join(missing)
        )
        return False

    def wait_for_fresh_status(self):
        return self.spin_until(
            lambda: not status_gate_error(
                self.last_status,
                self.status_age(),
                self.status_timeout,
            ),
            self.wait_for_subscribers,
        )

    def publish_zero_stop(self):
        """Latch neutral velocity and ask the controller for a planted STOP."""
        self.cmd_vel_publisher.publish(Twist())
        message = String()
        message.data = "stop"
        self.action_publisher.publish(message)

    def publish_action(self, action):
        message = String()
        message.data = action
        self.action_publisher.publish(message)

    def publish_emote_json(self, encoded):
        message = String()
        message.data = encoded
        self.emote_publisher.publish(message)

    def spin_emote_until(self, predicate, timeout):
        """Spin while renewing the active correlated controller lease."""
        deadline = self.monotonic() + max(0.0, float(timeout))
        next_keepalive = self.monotonic()
        while rclpy.ok():
            if predicate():
                return True
            now = self.monotonic()
            if now >= deadline:
                return False
            if self.request_id and now >= next_keepalive:
                self.publish_emote_json(
                    encode_keepalive_request(self.request_id)
                )
                next_keepalive = now + EMOTE_KEEPALIVE_PERIOD
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - now))
        return False

    def prepare_controller(self, emote_name=None):
        """Require a fresh owner/state, STOP, then observe a fresh idle frame."""
        if not self.wait_for_fresh_status():
            self.get_logger().error(
                status_gate_error(
                    self.last_status,
                    self.status_age(),
                    self.status_timeout,
                )
            )
            return False
        reason = status_gate_error(
            self.last_status,
            self.status_age(),
            self.status_timeout,
            emote_name=emote_name,
        )
        if reason:
            self.get_logger().error(reason)
            return False

        baseline_generation = self.status_generation
        self.publish_zero_stop()
        self.spin_for(self.stop_settle_time)

        def stopped_and_fresh():
            return (
                self.status_generation > baseline_generation
                and not status_gate_error(
                    self.last_status,
                    self.status_age(),
                    self.status_timeout,
                    emote_name=emote_name,
                    require_idle=emote_name is not None,
                )
            )

        if self.spin_until(stopped_and_fresh, self.acknowledgement_timeout):
            return True
        reason = status_gate_error(
            self.last_status,
            self.status_age(),
            self.status_timeout,
            emote_name=emote_name,
            require_idle=emote_name is not None,
        )
        self.get_logger().error(
            reason or "No fresh controller status arrived after STOP."
        )
        return False

    def run_action(self, action):
        if not self.wait_for_controller(include_emote=False):
            return False
        if not self.prepare_controller():
            return False
        baseline_generation = self.status_generation
        self.publish_action(action)
        target_state = "standing" if action == "stand" else "sitting"

        def action_complete():
            return (
                self.status_generation > baseline_generation
                and self.status_age() <= self.status_timeout
                and str(self.last_status.get("command_owner", "")).upper()
                == "MOTION"
                and str(self.last_status.get("state", "")).lower()
                == target_state
            )

        self.get_logger().info(
            "Requested controller action '%s'; waiting for %s."
            % (action, target_state)
        )
        if self.spin_until(action_complete, self.action_timeout):
            self.get_logger().info("Controller reached %s." % target_state)
            return True
        self.get_logger().error(
            "Timed out waiting for controller state %s after action '%s'."
            % (target_state, action)
        )
        return False

    def log_correlated_result(self, result):
        if result == self.last_reported_result:
            return
        self.last_reported_result = result
        message = str(self.last_status.get("emote_message", "")).strip()
        progress = self.last_status.get("emote_progress", 0.0)
        try:
            progress = _clamp(float(progress), 0.0, 1.0)
        except (TypeError, ValueError):
            progress = 0.0
        self.get_logger().info(
            "Emote %s (%.0f%%)%s"
            % (result, progress * 100.0, ": " + message if message else ".")
        )

    def run_cartesian_emote(self, name):
        if not self.wait_for_controller(include_emote=True):
            return False
        if not self.prepare_controller(emote_name=name):
            return False

        self.request_id = "legacy-emote-%s" % uuid.uuid4().hex[:16]
        encoded = encode_start_request(self.request_id, name, self.options)
        self.publish_emote_json(encoded)
        self.get_logger().info(
            "Requested Cartesian emote '%s' as %s." % (name, self.request_id)
        )

        if not self.spin_emote_until(
            lambda: correlated_emote_result(
                self.last_status,
                self.request_id,
            )
            is not None,
            self.acknowledgement_timeout,
        ):
            self.get_logger().error(
                "Motion controller did not acknowledge emote request %s."
                % self.request_id
            )
            self.cancel_and_stop()
            return False

        deadline = self.monotonic() + self.completion_timeout
        next_keepalive = self.monotonic()
        while rclpy.ok():
            result = correlated_emote_result(self.last_status, self.request_id)
            if result is not None:
                self.log_correlated_result(result)
                if result in TERMINAL_EMOTE_RESULTS:
                    if result == "completed":
                        return True
                    message = str(
                        self.last_status.get("emote_message", result)
                    ).strip()
                    self.get_logger().error(
                        "Cartesian emote %s: %s" % (result, message)
                    )
                    return False
            if self.status_age() > self.status_timeout:
                self.get_logger().error(
                    "Lost fresh /volt/status while waiting for emote %s."
                    % self.request_id
                )
                self.cancel_and_stop()
                return False
            remaining = deadline - self.monotonic()
            if remaining <= 0.0:
                self.get_logger().error(
                    "Timed out waiting for emote %s to complete."
                    % self.request_id
                )
                self.cancel_and_stop()
                return False
            now = self.monotonic()
            if now >= next_keepalive:
                self.publish_emote_json(
                    encode_keepalive_request(self.request_id)
                )
                next_keepalive = now + EMOTE_KEEPALIVE_PERIOD
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))
        return False

    def cancel_and_stop(self):
        """Issue a correlated cancel before the independent global STOP."""
        if self.request_id:
            self.publish_emote_json(encode_cancel_request(self.request_id))
            # Give the controller a chance to correlate the cancel before the
            # action-topic STOP independently enforces neutral motion.
            try:
                self.spin_for(0.10)
            except KeyboardInterrupt:
                pass
        self.publish_zero_stop()

    def play_once(self):
        try:
            request = resolve_controller_request(
                self.emote_name,
                self.emote_file,
            )
        except EmotePlayerError as exc:
            self.get_logger().error(str(exc))
            return False

        if request.kind == "action":
            return self.run_action(request.name)
        return self.run_cartesian_emote(request.name)


def main(args=None):
    # Keep the context alive when Python raises KeyboardInterrupt so cleanup
    # can publish a correlated cancel and STOP before shutting ROS down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    succeeded = False
    try:
        node = VoltEmotePlayer()
        succeeded = node.play_once()
    except (EmotePlayerError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print("volt_emote_player: %s" % exc, file=sys.stderr)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Interrupted; requesting correlated emote cancel and STOP."
            )
            try:
                node.cancel_and_stop()
                node.spin_for(0.15)
            except KeyboardInterrupt:
                pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if succeeded else 1


if __name__ == "__main__":
    raise SystemExit(main())
