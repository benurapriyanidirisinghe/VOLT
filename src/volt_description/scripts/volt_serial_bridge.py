#!/usr/bin/env python3

"""Bridge canonical ROS joint radians to channel-ordered Arduino FRAME packets."""

import json
import math
import socket
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float64MultiArray, String, UInt8, UInt32

from volt_kinematics import JOINT_NAMES
from volt_servo_calibration import (
    CalibrationError,
    deadband_feedforward,
    ServoCalibrationTable,
    named_positions_from_ordered,
)
from volt_serial_protocol import (
    ArduinoProtocolState,
    CRITICAL_STACK_TOPICS,
    SAFE_STOP_COMMANDS,
    SerialLineBuffer,
    duplicate_stack_topics,
    format_face_command,
    format_frame_command,
    format_binary_frame,
    BINARY_PROTOCOL_MIN_VERSION,
    FIRMWARE_COUNTER_FIELDS,
    firmware_guard_clips,
    format_led_color_b_command,
    format_led_brightness_command,
    format_led_color_command,
    format_led_effect_command,
    format_led_speed_command,
    guarded_pending_command,
    motion_status_allows_arm,
    status_token,
)

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - handled at runtime on robot.
    serial = None
    SerialException = Exception


COMMAND_OWNERS = ("MOTION", "MANUAL", "CALIBRATION", "HOLD", "DISABLED")
LONG_RESPONSE_COMMANDS = ("STATUS", "LED STATUS")
FACE_SETTING_ORDER = (
    "expression",
    "color",
    "alternate_color",
    "brightness",
    "speed",
    "effect",
)


def parse_command_owner_status(payload):
    """Return a validated router owner from JSON or key=value status text."""
    owner = None
    try:
        decoded = json.loads(payload)
    except (TypeError, ValueError):
        decoded = None
    if isinstance(decoded, dict):
        for key, value in decoded.items():
            if str(key).strip().lower() == "owner":
                owner = value
                break
    else:
        for field in str(payload).split():
            if "=" not in field:
                continue
            key, value = field.split("=", 1)
            if key.strip().lower() == "owner":
                owner = value
                break

    owner = str(owner or "").strip().upper()
    return owner if owner in COMMAND_OWNERS else None


def parse_motion_required_max_dps(status):
    """Return the greatest commanded joint speed in a hardware status report."""
    if not isinstance(status, dict):
        return None

    limits_key = "effective_joint_velocity_limits_deg_s"
    limits = status.get(limits_key)
    if not isinstance(limits, dict) or not all(
        joint_name in limits for joint_name in JOINT_NAMES
    ):
        return None
    values = [limits[joint_name] for joint_name in JOINT_NAMES]

    parsed = []
    for value in values:
        if isinstance(value, bool):
            return None
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value) or value <= 0.0:
            return None
        parsed.append(value)
    return max(parsed) if parsed else None


def normalized_color_to_rgb(message):
    """Convert a standard ColorRGBA's normalized RGB channels to bytes."""
    channels = []
    for name in ("r", "g", "b"):
        value = getattr(message, name, None)
        if isinstance(value, bool):
            raise ValueError("face color channels must be numeric")
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError("face color channels must be numeric")
        if not math.isfinite(value):
            raise ValueError("face color channels must be finite")
        channels.append(int(round(max(0.0, min(1.0, value)) * 255.0)))
    return tuple(channels)


# Resolved once rather than per status message: gethostname() can touch the
# resolver. Published so the console can show WHICH machine is driving the
# Arduino -- with the split Jetson stack that is not the machine the
# operator is sitting at.
try:
    BRIDGE_HOST = socket.gethostname()
except OSError:
    BRIDGE_HOST = "unknown"


class VoltSerialBridge(Node):
    def __init__(self):
        super().__init__("volt_serial_bridge")

        default_calibration = (
            get_package_share_directory("volt_description")
            + "/config/servo_calibration.yaml"
        )
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 250000)
        self.declare_parameter("calibration_file", default_calibration)
        self.declare_parameter("command_topic", "/joint_command_router/output")
        self.declare_parameter("serial_command_topic", "/volt/serial_command")
        self.declare_parameter("status_topic", "/volt/serial_status")
        self.declare_parameter("face_expression_topic", "/volt/face/expression")
        self.declare_parameter("face_color_topic", "/volt/face/color")
        self.declare_parameter(
            "face_alternate_color_topic",
            "/volt/face/alternate_color",
        )
        self.declare_parameter("face_brightness_topic", "/volt/face/brightness")
        self.declare_parameter("face_effect_topic", "/volt/face/effect")
        self.declare_parameter("face_speed_topic", "/volt/face/speed")
        self.declare_parameter("motion_status_topic", "/volt/status")
        self.declare_parameter(
            "command_router_status_topic",
            "/volt/command_router_status",
        )
        # A Nano has a 64-byte UART RX ring, which buffers 2.5 ms at 250000
        # baud.  A <=63-byte frame costs 2.2 ms on the wire, so 60 Hz is a 13%
        # duty cycle and leaves the PCA9685 I2C burst (~1.4 ms at 400 kHz) and
        # the face window ample room.  60 Hz also sits above the firmware's
        # 50 Hz interpolation tick, so every servo update has a target newer
        # than one pulse period instead of aliasing against a 30 Hz stream.
        self.declare_parameter("max_send_rate", 60.0)
        # Gait tuning needs the rate the controller actually demands, measured
        # on hardware, rather than an estimate.  This records per-channel
        # deg/s BEFORE the send-rate gate, so it reports what the gait plans
        # rather than what decimation happened to let through, and flags every
        # sample the firmware slew ceiling would clip.
        self.declare_parameter("joint_rate_diagnostic", False)
        self.declare_parameter("joint_rate_diagnostic_output", "")
        self.declare_parameter("firmware_slew_limit_dps", 240.0)
        # Face settings are state changes, not animation frames. Sending at
        # most one short command per timer interval keeps FRAME and safety
        # traffic ahead of cosmetic work on the Nano's small UART buffer.
        self.declare_parameter("max_face_command_rate", 10.0)
        self.declare_parameter("reconnect_period", 1.0)
        self.declare_parameter("serial_timeout", 0.02)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("auto_arm", False)
        # The calibration/joint-test launches explicitly set this false. That
        # specialized suspended-robot mode bypasses both normal ROS safety
        # reports; firmware readiness, ARM acknowledgement, and HOLD still apply.
        self.declare_parameter("require_motion_safe_to_arm", True)
        self.declare_parameter("motion_status_timeout", 3.0)
        self.declare_parameter("owner_status_timeout", 1.0)
        self.declare_parameter("arm_frame_timeout", 0.50)
        self.declare_parameter("arm_frame_settle_time", 0.25)
        self.declare_parameter("arm_frame_stable_tolerance_deg", 0.50)
        self.declare_parameter("arm_ack_timeout", 0.20)
        self.declare_parameter("command_timeout", 0.75)
        # Normal motion accepts only firmware that explicitly identifies this
        # protocol generation and its output slew ceiling. The specialized
        # calibration mode remains an intentional bypass through
        # require_motion_safe_to_arm=false.
        self.declare_parameter("required_protocol_version", 2)

        self.port = str(self.get_parameter("port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        # The firmware baud changed with the throughput work, and a host/firmware
        # mismatch is otherwise unrecoverable without reflashing blind.  Probe the
        # configured rate first, then the other known-good rates, and lock on to
        # whichever one produces a valid firmware handshake.  This makes the
        # order of "flash the Nano" versus "update the host" irrelevant.
        self.baud_candidates = []
        for candidate in (self.baud_rate, 250000, 57600):
            if candidate not in self.baud_candidates:
                self.baud_candidates.append(int(candidate))
        self.baud_index = 0
        self.baud_locked = False
        self.connect_opened_time = 0.0
        self.baud_probe_timeout = 4.0
        self.calibration_file = str(self.get_parameter("calibration_file").value)
        requested_send_rate = float(self.get_parameter("max_send_rate").value)
        if not math.isfinite(requested_send_rate):
            requested_send_rate = 60.0
        # 100 Hz is the point where frame traffic plus the face window stops
        # leaving reliable slack on a 64-byte RX ring; do not raise it without
        # also enlarging SERIAL_RX_BUFFER_SIZE in the firmware build.
        self.max_send_rate = min(100.0, max(1.0, requested_send_rate))
        self.joint_rate_diagnostic = bool(
            self.get_parameter("joint_rate_diagnostic").value
        )
        self.joint_rate_diagnostic_output = str(
            self.get_parameter("joint_rate_diagnostic_output").value or ""
        ).strip()
        self.firmware_slew_limit_dps = float(
            self.get_parameter("firmware_slew_limit_dps").value
        )
        self._rate_prev_deg = None
        self._rate_prev_time = 0.0
        self._rate_file = None
        self._rate_peak_dps = 0.0
        self._rate_peak_channel = -1
        self._rate_clipped_samples = 0
        self._rate_total_samples = 0
        self._rate_last_report = 0.0
        requested_face_rate = float(
            self.get_parameter("max_face_command_rate").value
        )
        if not math.isfinite(requested_face_rate):
            requested_face_rate = 10.0
        self.max_face_command_rate = min(10.0, max(1.0, requested_face_rate))
        self.reconnect_period = float(self.get_parameter("reconnect_period").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.hardware_enabled = bool(self.get_parameter("hardware_enabled").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.require_motion_safe_to_arm = bool(
            self.get_parameter("require_motion_safe_to_arm").value
        )
        self.motion_status_timeout = float(
            self.get_parameter("motion_status_timeout").value
        )
        self.owner_status_timeout = float(
            self.get_parameter("owner_status_timeout").value
        )
        self.arm_frame_timeout = min(
            1.0,
            max(
                0.05,
                float(self.get_parameter("arm_frame_timeout").value),
            ),
        )
        self.arm_frame_settle_time = min(
            self.arm_frame_timeout,
            max(
                0.0,
                float(self.get_parameter("arm_frame_settle_time").value),
            ),
        )
        self.arm_frame_stable_tolerance_deg = min(
            10.0,
            max(
                0.0,
                float(
                    self.get_parameter(
                        "arm_frame_stable_tolerance_deg"
                    ).value
                ),
            ),
        )
        self.arm_ack_timeout = min(
            0.50,
            max(
                0.05,
                float(self.get_parameter("arm_ack_timeout").value),
            ),
        )
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.required_protocol_version = max(
            1,
            int(self.get_parameter("required_protocol_version").value),
        )
        command_topic = str(self.get_parameter("command_topic").value)
        serial_command_topic = str(self.get_parameter("serial_command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        face_expression_topic = str(
            self.get_parameter("face_expression_topic").value
        )
        face_color_topic = str(self.get_parameter("face_color_topic").value)
        face_alternate_color_topic = str(
            self.get_parameter("face_alternate_color_topic").value
        )
        face_brightness_topic = str(
            self.get_parameter("face_brightness_topic").value
        )
        face_effect_topic = str(self.get_parameter("face_effect_topic").value)
        face_speed_topic = str(self.get_parameter("face_speed_topic").value)
        motion_status_topic = str(self.get_parameter("motion_status_topic").value)
        command_router_status_topic = str(
            self.get_parameter("command_router_status_topic").value
        )

        self.calibration = None
        self.calibration_valid = False
        self.calibration_error = ""
        self.load_calibration()
        if self.calibration_valid:
            self.log_mapping_table()

        self.serial_port = None
        self.last_send_time = 0.0
        self.next_send_deadline = 0.0
        self.last_rate_clock = 0.0
        self.last_connect_attempt = 0.0
        self.last_ping_time = 0.0
        self.last_protocol_command_time = 0.0
        self.serial_query_inflight = ""
        self.serial_query_since = 0.0
        # Distinct from self.frame_sequence, which is a pre-existing unbounded
        # counter the MOTION-ownership floor compares against.  Wrapping that
        # one at 8 bits would corrupt the interlock.
        self.binary_frame_seq = 0
        self.last_firmware_status_poll = 0.0
        self.connected = False
        self.protocol = ArduinoProtocolState()
        self.serial_lines = SerialLineBuffer()
        self.arm_requested = self.auto_arm
        self.last_error = ""
        self.last_command_time = 0.0
        self.last_frame = []
        self.last_details = []
        self.frame_sequence = 0
        self.last_frame_owner = "UNKNOWN"
        self.last_frame_owner_epoch = -1
        self.owner_epoch = 0
        self.motion_owner_frame_floor = 0
        self.frame_stable_reference = []
        self.frame_stable_since = 0.0
        self.frame_stable_samples = 0
        self.frames_sent = 0
        self.frames_rejected = 0
        self.frames_blocked = 0
        # The firmware clamps to CHANNEL_MIN_DEG/CHANNEL_MAX_DEG and reports
        # nothing back, so the host mirrors the guard to make a clip visible
        # instead of silently losing commanded travel.
        self.guard_clip_channels = {}
        self.guard_clip_frames = 0
        self.guard_clip_last_log = 0.0
        self.guard_clip_last_seen = 0.0
        self.frame_rate = 0.0
        self.frame_rate_window_start = self.now_seconds()
        self.frame_rate_window_sent = 0
        self.motion_state = "unknown"
        self.motion_moving = True
        self.motion_step_in_place = False
        self.motion_arm_neutral_ready = False
        self.motion_controller_connected = False
        self.motion_hardware_mode = False
        self.motion_required_max_dps = 0.0
        self.motion_required_max_dps_known = False
        self.last_motion_status_time = 0.0
        self.command_owner = "UNKNOWN"
        self.last_owner_status_time = 0.0
        self.owner_interlock_reason = ""
        self.stack_conflict_topics = ()
        self.last_topology_check_time = 0.0
        self.topology_check_period = 0.25

        # Desired face state survives transport resets. Dirty keys are
        # coalesced, so rapid GUI/emote changes use bounded memory and only the
        # newest value reaches the Arduino. One command is kept in flight until
        # its ACK, preventing LED traffic from filling the receive ring.
        self.face_desired = {}
        self.face_dirty = []
        self.face_inflight_key = ""
        self.face_inflight_value = None
        self.face_inflight_command = ""
        self.face_inflight_since = 0.0
        self.face_failed_keys = set()
        self.face_queue_settling = False
        self.face_status_pending = True
        self.face_status_request_time = 0.0
        self.host_sync_inflight = False
        self.host_sync_since = 0.0
        self.host_sync_error = ""
        self.last_face_send_time = 0.0
        self.face_commands_queued = 0
        self.face_commands_sent = 0
        self.face_commands_rejected = 0

        self.create_subscription(Float64MultiArray, command_topic, self.command_callback, 1)
        self.create_subscription(String, serial_command_topic, self.serial_command_callback, 10)
        self.create_subscription(
            String, face_expression_topic, self.face_expression_callback, 10
        )
        self.create_subscription(
            ColorRGBA, face_color_topic, self.face_color_callback, 10
        )
        self.create_subscription(
            ColorRGBA,
            face_alternate_color_topic,
            self.face_alternate_color_callback,
            10,
        )
        self.create_subscription(
            UInt8, face_brightness_topic, self.face_brightness_callback, 10
        )
        self.create_subscription(String, face_effect_topic, self.face_effect_callback, 10)
        self.create_subscription(UInt32, face_speed_topic, self.face_speed_callback, 10)
        self.create_subscription(String, motion_status_topic, self.motion_status_callback, 10)
        self.create_subscription(
            String,
            command_router_status_topic,
            self.command_router_status_callback,
            10,
        )
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

    def firmware_capability_compatible(self):
        """Report raw compatibility, independent of specialized-mode bypass."""
        if not self.motion_velocity_requirement_is_fresh():
            return False
        return self.protocol.firmware_capability_satisfies(
            self.required_protocol_version,
            minimum_max_dps=self.motion_required_max_dps,
        )

    def motion_velocity_requirement_is_fresh(self):
        """Require a recent, valid velocity contract in hardware motion mode."""
        if not self.motion_hardware_mode:
            return True
        if not self.motion_required_max_dps_known:
            return False
        age = time.monotonic() - self.last_motion_status_time
        return 0.0 <= age <= max(0.0, self.motion_status_timeout)

    def firmware_capability_failure(self, prefix):
        """Describe the identity, protocol, and velocity contract that failed."""
        if self.motion_hardware_mode and not self.motion_required_max_dps_known:
            return (
                "%s: hardware-mode motion status must report finite positive "
                "effective joint velocity limits (required_max_dps=unknown)"
                % prefix
            )
        if self.motion_hardware_mode and not self.motion_velocity_requirement_is_fresh():
            return (
                "%s: hardware-mode motion velocity requirement is stale "
                "(required_max_dps=%.1f)"
                % (prefix, self.motion_required_max_dps)
            )
        if self.motion_required_max_dps > 0.0:
            max_dps_requirement = (
                "MAX_DPS>=%.1f (required_max_dps=%.1f)"
                % (
                    self.motion_required_max_dps,
                    self.motion_required_max_dps,
                )
            )
        else:
            max_dps_requirement = "a positive MAX_DPS (required_max_dps=0.0)"
        return (
            "%s: firmware must identify FW=VOLT_PCA9685 with PROTO>=%d and "
            "%s; reported MAX_DPS=%.1f"
            % (
                prefix,
                self.required_protocol_version,
                max_dps_requirement,
                self.protocol.max_dps,
            )
        )

    def firmware_capability_allows_motion(self):
        """Require versioned firmware for normal motion, not calibration mode."""
        return (
            not self.require_motion_safe_to_arm
            or self.firmware_capability_compatible()
        )

    def hardware_stream_allowed(self):
        return (
            self.protocol.can_stream_frames
            and self.firmware_capability_allows_motion()
        )

    def frame_send_slot_available(self, now):
        """Rate-limit against an ideal deadline without accumulating bursts."""
        now = float(now)
        if not math.isfinite(now):
            return False
        period = 1.0 / self.max_send_rate
        if self.last_rate_clock > 0.0 and now + 1e-9 < self.last_rate_clock:
            # A ROS clock reset must not permit two physical frames back to
            # back. Re-anchor and wait one full safe period.
            self.last_rate_clock = now
            self.next_send_deadline = now + period
            return False
        if self.next_send_deadline <= 0.0:
            if self.last_send_time > 0.0 and now >= self.last_send_time:
                self.next_send_deadline = self.last_send_time + period
            else:
                self.next_send_deadline = now

        self.last_rate_clock = now
        if now + 1e-9 < self.next_send_deadline:
            return False

        next_deadline = self.next_send_deadline + period
        if next_deadline <= now + 1e-9:
            # Do not issue catch-up frames after a scheduler stall.
            next_deadline = now + period
        self.next_send_deadline = next_deadline
        return True

    def defer_next_frame_until_period_after(self, now):
        """Account for an out-of-band cached FRAME sent immediately after ARM."""
        now = float(now)
        self.last_send_time = now
        self.last_rate_clock = now
        self.next_send_deadline = now + 1.0 / self.max_send_rate

    def refresh_stack_topology(self, force=False):
        """Fail closed when multiple VOLT stacks share the global topics."""
        now = time.monotonic()
        if (
            not force
            and self.last_topology_check_time > 0.0
            and now - self.last_topology_check_time < self.topology_check_period
        ):
            return not self.stack_conflict_topics
        self.last_topology_check_time = now
        try:
            counts = {
                topic: len(self.get_publishers_info_by_topic(topic))
                for topic in CRITICAL_STACK_TOPICS
            }
        except Exception as exc:
            self.stack_conflict_topics = ("ROS graph inspection failed",)
            self.last_error = "ARM locked: ROS graph inspection failed: %s" % exc
            return False

        previous = self.stack_conflict_topics
        self.stack_conflict_topics = duplicate_stack_topics(counts)
        if self.stack_conflict_topics and self.stack_conflict_topics != previous:
            self.motion_state = "unknown"
            self.motion_moving = True
            self.motion_arm_neutral_ready = False
            self.last_motion_status_time = 0.0
            self.command_owner = "UNKNOWN"
            self.last_owner_status_time = 0.0
            self.owner_epoch += 1
            self.motion_owner_frame_floor = self.frame_sequence
            self.invalidate_frame_stability()
            self.get_logger().error(
                "DUPLICATE VOLT STACK: ARM locked; conflicting publishers on %s"
                % ", ".join(self.stack_conflict_topics)
            )
        elif previous and not self.stack_conflict_topics:
            self.motion_state = "unknown"
            self.motion_moving = True
            self.motion_arm_neutral_ready = False
            self.last_motion_status_time = 0.0
            self.command_owner = "UNKNOWN"
            self.last_owner_status_time = 0.0
            self.owner_epoch += 1
            self.motion_owner_frame_floor = self.frame_sequence
            self.invalidate_frame_stability()
            self.get_logger().info(
                "Duplicate VOLT publishers cleared; fresh status and frame required."
            )
        return not self.stack_conflict_topics

    def motion_status_callback(self, message):
        if not self.refresh_stack_topology():
            return
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self.motion_state = str(status.get("state", "unknown")).strip().lower()
        self.motion_moving = bool(status.get("moving"))
        self.motion_step_in_place = bool(status.get("step_in_place"))
        self.motion_arm_neutral_ready = bool(
            status.get("arm_neutral_ready", False)
        )
        self.motion_controller_connected = bool(status.get("controller_connected"))
        hardware_mode = status.get("hardware_mode")
        if hardware_mode is True:
            self.motion_hardware_mode = True
        elif hardware_mode is False:
            self.motion_hardware_mode = False
        # A missing/malformed hardware_mode field cannot downgrade a previously
        # confirmed hardware controller into the unguarded simulation case.
        if self.motion_hardware_mode:
            required_max_dps = parse_motion_required_max_dps(status)
            self.motion_required_max_dps_known = required_max_dps is not None
            self.motion_required_max_dps = (
                required_max_dps
                if required_max_dps is not None
                else math.inf
            )
        else:
            self.motion_required_max_dps_known = False
            self.motion_required_max_dps = 0.0
        self.last_motion_status_time = time.monotonic()

    def motion_safe_to_arm(self):
        if not self.require_motion_safe_to_arm:
            return True
        age = time.monotonic() - self.last_motion_status_time
        return motion_status_allows_arm(
            self.motion_state,
            self.motion_moving,
            self.motion_step_in_place,
            self.motion_controller_connected,
            age,
            self.motion_status_timeout,
            self.motion_arm_neutral_ready,
        )

    def owner_status_age(self):
        if self.last_owner_status_time <= 0.0:
            return -1.0
        return time.monotonic() - self.last_owner_status_time

    def owner_status_is_fresh(self):
        age = self.owner_status_age()
        return 0.0 <= age <= max(0.0, self.owner_status_timeout)

    def owner_allows_hardware_output(self):
        """Gate normal output; calibration launches explicitly disable this guard."""
        if not self.require_motion_safe_to_arm:
            return True
        return (
            not self.stack_conflict_topics
            and self.command_owner == "MOTION"
            and self.owner_status_is_fresh()
        )

    def frame_age(self):
        if self.last_command_time <= 0.0:
            return -1.0
        return self.now_seconds() - self.last_command_time

    def frame_has_current_motion_owner(self):
        """Return true only for a frame accepted after this MOTION lease began."""
        if not self.require_motion_safe_to_arm:
            return True
        return (
            self.owner_allows_hardware_output()
            and self.last_frame_owner == "MOTION"
            and self.last_frame_owner_epoch == self.owner_epoch
            and self.frame_sequence > self.motion_owner_frame_floor
        )

    def frame_stable_age(self):
        if (
            not self.frame_has_current_motion_owner()
            or self.frame_stable_since <= 0.0
        ):
            return -1.0
        age = self.now_seconds() - self.frame_stable_since
        return age if age >= 0.0 else -1.0

    def frame_is_stable(self):
        stable_age = self.frame_stable_age()
        return (
            self.frame_stable_samples >= 2
            and stable_age >= 0.0
            and stable_age + 1e-9 >= self.arm_frame_settle_time
        )

    def frame_ready_to_arm(self):
        """Require a recent stable frame accepted under the current MOTION lease."""
        if not self.require_motion_safe_to_arm:
            return True
        if (
            len(self.last_frame) != len(JOINT_NAMES)
            or not self.frame_has_current_motion_owner()
            or not self.frame_is_stable()
        ):
            return False
        age = self.frame_age()
        if not 0.0 <= age <= self.arm_frame_timeout:
            return False
        try:
            format_frame_command(self.last_frame)
        except (TypeError, ValueError):
            return False
        return True

    def arm_interlocks_safe(self):
        return (
            self.refresh_stack_topology()
            and self.firmware_capability_allows_motion()
            and self.motion_safe_to_arm()
            and self.owner_allows_hardware_output()
            and self.frame_ready_to_arm()
        )

    def invalidate_frame_stability(self):
        self.frame_stable_reference = []
        self.frame_stable_since = 0.0
        self.frame_stable_samples = 0

    def cache_frame(self, frame, details, now):
        """Cache one validated frame together with its ownership generation."""
        values = list(frame)
        previous_owner = self.last_frame_owner
        previous_epoch = self.last_frame_owner_epoch
        accepted_motion_owner = self.owner_allows_hardware_output()
        if accepted_motion_owner:
            frame_owner = "MOTION"
        elif self.owner_status_is_fresh():
            frame_owner = self.command_owner
        else:
            frame_owner = "UNKNOWN"

        self.frame_sequence += 1
        same_stable_candidate = (
            accepted_motion_owner
            and self.frame_sequence > self.motion_owner_frame_floor
            and previous_owner == "MOTION"
            and previous_epoch == self.owner_epoch
            and len(self.frame_stable_reference) == len(values)
            # Stability is about the command the Nano will actually receive.
            # Do not accept small float changes that cross a whole-degree wire
            # token boundary.
            and format_frame_command(values)
            == format_frame_command(self.frame_stable_reference)
            and all(
                abs(value - reference)
                <= self.arm_frame_stable_tolerance_deg
                for value, reference in zip(values, self.frame_stable_reference)
            )
        )
        if accepted_motion_owner:
            if same_stable_candidate:
                self.frame_stable_samples += 1
            else:
                self.frame_stable_reference = list(values)
                self.frame_stable_since = now
                self.frame_stable_samples = 1
        else:
            self.invalidate_frame_stability()

        self.last_command_time = now
        self.last_frame = values
        self.last_details = details
        self.last_frame_owner = frame_owner
        self.last_frame_owner_epoch = self.owner_epoch

    def enforce_owner_interlock(self, reason):
        """Cancel ARM and inhibit firmware motion as soon as ownership is unsafe."""
        self.arm_requested = False
        if reason and reason != self.owner_interlock_reason:
            self.owner_interlock_reason = reason
            self.last_error = reason
            self.get_logger().warning(reason, throttle_duration_sec=2.0)

        if self.dry_run or not self.hardware_enabled or not self.connected:
            return False
        pending = self.protocol.pending_command
        if pending in SAFE_STOP_COMMANDS:
            # A late ARM acknowledgement must not weaken DISARM or DISABLE to
            # HOLD. The protocol state already inhibits FRAME traffic while the
            # stronger stop command remains pending.
            self.protocol.motion_inhibited = True
            return False
        if self.protocol.armed or pending == "ARM":
            self.protocol.motion_inhibited = True
            return self.send_protocol_command("HOLD")
        return False

    def clear_owner_interlock(self):
        if self.stack_conflict_topics:
            return
        if self.last_error == self.owner_interlock_reason:
            self.last_error = ""
        self.owner_interlock_reason = ""

    def command_router_status_callback(self, message):
        if not self.refresh_stack_topology():
            return
        owner = parse_command_owner_status(message.data)
        if owner is None:
            return
        previous_owner = self.command_owner
        previous_owner_fresh = self.owner_status_is_fresh()
        owner_context_changed = (
            owner != previous_owner
            or (owner == "MOTION" and not previous_owner_fresh)
        )
        self.command_owner = owner
        self.last_owner_status_time = time.monotonic()
        if owner_context_changed:
            self.owner_epoch += 1
            self.invalidate_frame_stability()
            if owner == "MOTION":
                # A pre-existing HOLD/MANUAL frame cannot unlock a new MOTION
                # ownership interval. At least one newer router output is needed.
                self.motion_owner_frame_floor = self.frame_sequence
        if not self.require_motion_safe_to_arm:
            self.clear_owner_interlock()
            return
        if owner == "MOTION":
            self.clear_owner_interlock()
            return
        if previous_owner == "MOTION" or (
            self.protocol.armed or self.protocol.pending_command == "ARM"
        ):
            self.enforce_owner_interlock(
                "Hardware HOLD: command ownership left MOTION for %s" % owner
            )

    def queue_face_setting(self, key, value):
        """Coalesce a validated desired face setting without serial I/O."""
        if self.face_desired.get(key) == value:
            if key not in self.face_failed_keys:
                return False
            # An explicit republish after a firmware rejection is the bounded
            # refresh path. Ordinary duplicates remain fully deduplicated.
            self.face_failed_keys.discard(key)
            if key not in self.face_dirty:
                self.face_dirty.append(key)
            self.face_commands_queued += 1
            self.face_queue_settling = True
            if not self.face_failed_keys:
                failed_error = self.protocol.led_error
                self.protocol.led_error = ""
                if self.last_error == failed_error:
                    self.last_error = ""
            return True
        previous_led_error = self.protocol.led_error
        self.face_desired[key] = value
        self.face_failed_keys.discard(key)
        if not self.face_failed_keys:
            self.protocol.led_error = ""
            if self.last_error == previous_led_error:
                self.last_error = ""
        if key not in self.face_dirty:
            self.face_dirty.append(key)
        self.face_commands_queued += 1
        self.face_queue_settling = True
        return True

    def reject_face_setting(self, reason):
        self.face_commands_rejected += 1
        self.protocol.led_error = str(reason)
        self.get_logger().warning(
            "Rejected face command: %s" % reason,
            throttle_duration_sec=2.0,
        )

    def face_expression_callback(self, message):
        try:
            command = format_face_command(message.data)
        except (TypeError, ValueError) as exc:
            self.reject_face_setting(exc)
            return
        self.queue_face_setting("expression", command.split(maxsplit=1)[1])

    def face_color_callback(self, message):
        try:
            color = normalized_color_to_rgb(message)
            format_led_color_command(*color)
        except (TypeError, ValueError) as exc:
            self.reject_face_setting(exc)
            return
        self.queue_face_setting("color", color)

    def face_alternate_color_callback(self, message):
        try:
            color = normalized_color_to_rgb(message)
            format_led_color_b_command(*color)
        except (TypeError, ValueError) as exc:
            self.reject_face_setting(exc)
            return
        self.queue_face_setting("alternate_color", color)

    def face_brightness_callback(self, message):
        try:
            command = format_led_brightness_command(message.data)
        except (TypeError, ValueError) as exc:
            self.reject_face_setting(exc)
            return
        self.queue_face_setting("brightness", int(command.rsplit(" ", 1)[1]))

    def face_effect_callback(self, message):
        try:
            command = format_led_effect_command(message.data)
        except (TypeError, ValueError) as exc:
            self.reject_face_setting(exc)
            return
        self.queue_face_setting("effect", command.split(maxsplit=2)[2])

    def face_speed_callback(self, message):
        try:
            command = format_led_speed_command(message.data)
        except (TypeError, ValueError) as exc:
            self.reject_face_setting(exc)
            return
        self.queue_face_setting("speed", int(command.rsplit(" ", 1)[1]))

    def face_command_for(self, key, value):
        if key == "expression":
            return format_face_command(value)
        if key == "color":
            return format_led_color_command(*value)
        if key == "alternate_color":
            return format_led_color_b_command(*value)
        if key == "brightness":
            return format_led_brightness_command(value)
        if key == "effect":
            return format_led_effect_command(value)
        if key == "speed":
            return format_led_speed_command(value)
        raise ValueError("unknown face setting: %s" % key)

    def mark_face_for_resync(self):
        """Requeue desired state after every firmware/transport reset."""
        self.face_dirty = [
            key for key in FACE_SETTING_ORDER if key in self.face_desired
        ]
        self.face_inflight_key = ""
        self.face_inflight_value = None
        self.face_inflight_command = ""
        self.face_inflight_since = 0.0
        self.face_failed_keys.clear()
        self.face_queue_settling = bool(self.face_dirty)
        self.face_status_pending = True
        self.face_status_request_time = 0.0
        self.host_sync_inflight = False
        self.host_sync_since = 0.0
        self.host_sync_error = ""

    def face_is_synced(self):
        return (
            self.connected
            and self.protocol.ready
            and self.protocol.face_status_seen
            and not self.face_dirty
            and not self.face_inflight_key
            and not self.face_failed_keys
            and not self.face_status_pending
            and not self.host_sync_inflight
            and (
                not self.protocol.host_sync_required
                or self.protocol.host_synced
            )
        )

    def face_command_acknowledged(self, line, event):
        """Match the current LED/FACE ACK and advance the bounded queue."""
        if not self.face_inflight_key or event not in ("face", "led"):
            return False
        expected = "OK %s" % self.face_inflight_command
        response = str(line).strip()
        if response != expected and not response.startswith(expected + " "):
            return False
        self.face_failed_keys.discard(self.face_inflight_key)
        if self.last_error.startswith(("ERR LED", "ERR FACE")):
            self.last_error = ""
        self.face_inflight_key = ""
        self.face_inflight_value = None
        self.face_inflight_command = ""
        self.face_inflight_since = 0.0
        self.face_status_pending = True
        return True

    def send_next_face_command(self, now=None):
        """Send at most one visual setting, behind transport/safety traffic."""
        if self.dry_run or not self.hardware_enabled:
            # Retain the desired state so a later hardware launch can resync it.
            return False
        if not (self.connected and self.protocol.ready):
            return False
        if (
            self.protocol.pending_command
            or self.face_inflight_key
            or self.host_sync_inflight
        ):
            return False

        now = time.monotonic() if now is None else float(now)
        period = 1.0 / self.max_face_command_rate
        if now - self.last_face_send_time + 1e-9 < period:
            return False

        # Query status only after all desired state has been acknowledged. This
        # confirms support and gives the GUI the firmware's effective state.
        if not self.face_dirty:
            if (
                not self.face_status_pending
                and self.face_status_request_time > 0.0
                and now - self.face_status_request_time >= self.command_timeout
            ):
                self.face_status_pending = True
            if self.face_status_pending:
                if self.send_protocol_command("LED STATUS"):
                    self.face_status_pending = False
                    self.face_status_request_time = now
                    self.last_face_send_time = now
                    return True
                return False
            # New firmware keeps its nonblocking cyan loading indication until
            # the host's desired visual state has been acknowledged and then
            # confirmed by LED STATUS.  This terminal marker is deliberately
            # independent of servo readiness and ARM safety gates.
            if (
                self.protocol.host_sync_required
                and bool(self.face_desired)
                and self.protocol.host_ping_seen
                and self.protocol.host_snapshot_seen
                and self.protocol.face_status_seen
                and not self.protocol.host_synced
            ):
                if self.send_protocol_command("HOST SYNC"):
                    self.host_sync_inflight = True
                    self.host_sync_since = now
                    self.host_sync_error = ""
                    self.last_face_send_time = now
                    return True
            return False

        # ROS subscriptions are independent executors; callback arrival order
        # must not change the semantic application order of one GUI snapshot.
        # Always pick the first dirty setting in the fixed protocol order while
        # retaining one bounded entry per key.
        key = next(
            (
                candidate
                for candidate in FACE_SETTING_ORDER
                if candidate in self.face_dirty
            ),
            self.face_dirty[0],
        )
        self.face_dirty.remove(key)
        value = self.face_desired.get(key)
        try:
            command = self.face_command_for(key, value)
        except (TypeError, ValueError) as exc:
            self.reject_face_setting(exc)
            return False
        if not self.send_protocol_command(command):
            self.face_dirty.insert(0, key)
            return False
        self.face_inflight_key = key
        self.face_inflight_value = value
        self.face_inflight_command = command
        self.face_inflight_since = now
        self.last_face_send_time = now
        self.face_commands_sent += 1
        return True

    def service_face_queue(self, now=None):
        """Service visual traffic after one quiet timer cycle of coalescing."""
        if self.face_queue_settling:
            # Independent topic callbacks may straddle timer callbacks. Every
            # changed setting resets this one-cycle barrier, so an incomplete
            # snapshot can never send EFFECT before its later FACE callback.
            self.face_queue_settling = False
            return False
        return self.send_next_face_command(now)

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
            probe_baud = self.baud_candidates[self.baud_index]
            self.serial_port = serial.Serial(
                self.port,
                probe_baud,
                timeout=self.serial_timeout,
                write_timeout=self.serial_timeout,
                exclusive=True,
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            # Opening a Nano commonly resets it. Clear stale bytes first, then
            # preserve the firmware's startup identity banner during boot.
            time.sleep(2.0)
            self.connected = True
            self.connect_opened_time = time.monotonic()
            self.protocol.reset()
            self.serial_lines.reset()
            self.serial_query_inflight = ""
            self.serial_query_since = 0.0
            self.mark_face_for_resync()
            self.arm_requested = self.auto_arm
            self.last_error = ""
            if not self.send_protocol_command("PING"):
                return False
            self.get_logger().info(
                "Serial open on %s at %d baud; waiting for firmware "
                "PONG/ready banner." % (self.port, probe_baud)
            )
            return True
        except (SerialException, OSError) as exc:
            self.connected = False
            self.protocol.reset()
            self.serial_lines.reset()
            self.serial_query_inflight = ""
            self.serial_query_since = 0.0
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except (SerialException, OSError):
                    pass
            self.serial_port = None
            self.last_error = str(exc)
            self.get_logger().warning(
                "Waiting for Arduino on %s: %s" % (self.port, exc),
                throttle_duration_sec=5.0,
            )
            return False

    def disconnect(self, reason):
        self.connected = False
        self.connect_opened_time = 0.0
        self.baud_locked = False
        self.protocol.reset()
        self.serial_lines.reset()
        self.serial_query_inflight = ""
        self.serial_query_since = 0.0
        self.mark_face_for_resync()
        self.arm_requested = self.auto_arm
        self.last_error = reason
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except (SerialException, OSError):
                pass
        self.serial_port = None
        self.get_logger().warning("Arduino serial disconnected: %s" % reason)

    def read_available(self):
        if self.serial_port is None:
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting:
                data = self.serial_port.read(waiting)
                lines, overflowed = self.serial_lines.feed(data)
                if overflowed:
                    self.protocol.consume_response("ERR HOST_RX_LINE_TOO_LONG")
                    self.last_error = "Arduino response exceeded receive limit"
                    self.arm_requested = False
                    self.get_logger().error(self.last_error)
                for line in lines:
                    self.handle_serial_line(line)
            self.send_arm_if_ready()
        except (SerialException, OSError) as exc:
            self.disconnect(str(exc))

    def handle_serial_line(self, line):
        if self.serial_query_inflight and (
            line.startswith("ERR")
            or (
                self.serial_query_inflight == "STATUS"
                and line.startswith("OK STATUS")
            )
            or (
                self.serial_query_inflight == "LED STATUS"
                and line.startswith("OK LED STATUS")
            )
        ):
            self.serial_query_inflight = ""
            self.serial_query_since = 0.0
        if not self.baud_locked:
            # Any well-formed firmware line proves this rate is the right one.
            self.baud_locked = True
            self.baud_rate = self.baud_candidates[self.baud_index]
            self.get_logger().info(
                "Arduino link negotiated at %d baud." % self.baud_rate
            )
        was_ready = self.protocol.ready
        was_armed = self.protocol.armed
        pending_before = self.protocol.pending_command
        event = self.protocol.consume_response(line)
        if event == "recoverable_error":
            self.last_error = line
            host_sync_error = (
                self.host_sync_inflight
                and line.startswith("ERR HOST")
            )
            if host_sync_error:
                self.host_sync_inflight = False
                self.host_sync_since = 0.0
                self.host_sync_error = line
                if line.startswith("ERR HOST PING_REQUIRED"):
                    self.protocol.host_ping_seen = False
                elif line.startswith("ERR HOST SNAPSHOT_REQUIRED"):
                    self.protocol.host_snapshot_seen = False
                    self.mark_face_for_resync()
                    self.host_sync_error = line
                self.get_logger().warning(
                    "Arduino visual host synchronization remains pending: %s"
                    % line
                )
                return
            legacy_face_unknown = (
                line.startswith("ERR UNKNOWN_COMMAND")
                and not self.protocol.face_supported
                and bool(
                    self.face_inflight_key
                    or self.face_status_request_time > 0.0
                )
            )
            visual_error = (
                line.startswith(("ERR LED", "ERR FACE"))
                or legacy_face_unknown
            )
            if self.face_inflight_key and visual_error:
                # The desired value remains cached but a deterministic firmware
                # rejection must not retry forever or stall newer face changes.
                self.face_commands_rejected += 1
                self.face_failed_keys.add(self.face_inflight_key)
                self.face_inflight_key = ""
                self.face_inflight_value = None
                self.face_inflight_command = ""
                self.face_inflight_since = 0.0
                self.face_status_pending = True
            elif (
                self.face_status_request_time > 0.0
                and line.startswith("ERR UNKNOWN_COMMAND")
                and not self.protocol.face_supported
            ):
                # A legacy servo-only firmware does not understand LED STATUS.
                # Report unsupported once instead of polling it indefinitely.
                self.face_status_request_time = 0.0
                self.face_status_pending = False
                self.protocol.face_supported = False
                # Missing optional support is reported by face_supported=0,
                # not as a fault in the otherwise-compatible servo firmware.
                self.protocol.led_error = ""
                self.protocol.last_error = ""
                self.last_error = ""
            self.get_logger().warning(
                "Arduino rejected one serial line but remains armed: %s" % line
            )
            # Firmware parsing is atomic: a rejected FRAME cannot alter the
            # confirmed armed state or servo targets. Do not amplify a damaged
            # line into an immediate STATUS transaction; the normal timer and
            # watchdog continue to provide bounded recovery traffic.
            return
        if event in ("error", "timeout"):
            self.last_error = line
            self.arm_requested = False
            self.mark_face_for_resync()
            self.get_logger().warning("Arduino: %s" % line)
            return

        if event == "ready" and (
            not was_ready or line.startswith("OK VOLT_PCA9685_READY")
        ):
            # Opening a Nano resets both its animation state and servo state.
            # Replay only the newest setting for each face field after the
            # trusted identity handshake has completed.
            self.mark_face_for_resync()

        self.face_command_acknowledged(line, event)
        if event == "led_status":
            self.face_status_pending = False
            self.face_status_request_time = 0.0
        elif event == "host_sync":
            self.host_sync_inflight = False
            self.host_sync_since = 0.0
            self.host_sync_error = ""

        if event in (
            "ready",
            "armed",
            "held",
            "disarmed",
            "disabled",
            "status",
            "host_sync",
        ):
            self.last_error = ""

        armed_transition = not was_armed and self.protocol.armed
        if armed_transition:
            self.arm_requested = False
            safe_stop_pending = (
                self.protocol.pending_command
                if self.protocol.pending_command in SAFE_STOP_COMMANDS
                else pending_before
                if pending_before in SAFE_STOP_COMMANDS
                else ""
            )
            if safe_stop_pending:
                # This is a late ARM ACK crossing an already-issued stop. Keep
                # the stronger pending command and never replace it with HOLD.
                self.protocol.pending_command = safe_stop_pending
                self.protocol.motion_inhibited = True
                self.get_logger().warning(
                    "Ignored late ARM acknowledgement; %s remains pending."
                    % safe_stop_pending
                )
                return
            if not self.arm_interlocks_safe():
                self.enforce_owner_interlock(
                    "Hardware HOLD: ARM acknowledged after arming interlocks "
                    "became unsafe"
                )
                return
            self.get_logger().info(
                "Arduino confirmed ARM; live frame streaming is enabled."
            )
            if (
                self.require_motion_safe_to_arm
                and not self.send_cached_frame()
            ):
                self.enforce_owner_interlock(
                    "Hardware HOLD: ARM confirmed but the initial cached FRAME "
                    "could not be sent"
                )

    def send_cached_frame(self):
        if not (
            self.hardware_stream_allowed()
            and self.arm_interlocks_safe()
            and not self.serial_query_inflight
            and len(self.last_frame) == len(JOINT_NAMES)
            and self.last_command_time > 0.0
        ):
            return False
        frame_age = self.frame_age()
        if not 0.0 <= frame_age <= self.arm_frame_timeout:
            return False
        try:
            sent = self.send_frame_payload(self.last_frame)
        except (TypeError, ValueError) as exc:
            self.frames_rejected += 1
            self.last_error = "Cached FRAME rejected: %s" % exc
            self.get_logger().error(self.last_error)
            return False
        if not sent:
            return False
        self.defer_next_frame_until_period_after(self.now_seconds())
        self.frames_sent += 1
        return True

    def write_line(self, line):
        if self.serial_port is None:
            raise SerialException("serial port is not open")
        payload = (line + "\n").encode("ascii")
        written = self.serial_port.write(payload)
        if written != len(payload):
            raise SerialException(
                "short serial write: %d of %d bytes" % (written, len(payload))
            )

    def write_bytes(self, payload):
        if self.serial_port is None:
            raise SerialException("serial port is not open")
        written = self.serial_port.write(payload)
        if written != len(payload):
            raise SerialException(
                "short serial write: %d of %d bytes" % (written, len(payload))
            )

    def binary_frames_supported(self):
        return self.protocol.protocol_version >= BINARY_PROTOCOL_MIN_VERSION

    def send_frame_payload(self, frame):
        """Write one 12-channel frame, binary when the firmware supports it.

        Binary frames are fire-and-forget by design: the firmware answers
        corruption with a counter, never a reply, so there is no pending
        command to track.  note_command_sent() ignores FRAME for the ASCII
        path for the same reason.
        """
        if not self.binary_frames_supported():
            return self.send_protocol_command(format_frame_command(frame))
        payload = format_binary_frame(frame, self.binary_frame_seq)
        try:
            self.write_bytes(payload)
        except (SerialException, OSError) as exc:
            self.disconnect(str(exc))
            return False
        self.binary_frame_seq = (self.binary_frame_seq + 1) & 0xFF
        self.last_protocol_command_time = time.monotonic()
        return True

    def send_protocol_command(self, command):
        if command in LONG_RESPONSE_COMMANDS and self.serial_query_inflight:
            return False
        try:
            self.write_line(command)
            self.protocol.note_command_sent(command)
            now = time.monotonic()
            self.last_protocol_command_time = now
            if command == "PING":
                self.last_ping_time = now
            if command in LONG_RESPONSE_COMMANDS:
                self.serial_query_inflight = command
                self.serial_query_since = now
            return True
        except (SerialException, OSError) as exc:
            self.disconnect(str(exc))
            return False

    def send_arm_if_ready(self):
        if not (
            self.arm_requested
            and self.connected
            and self.protocol.ready
            and not self.protocol.armed
            and not self.protocol.pending_command
            and self.arm_interlocks_safe()
        ):
            return False
        if self.send_protocol_command("ARM"):
            self.get_logger().info("ARM requested; waiting for Arduino acknowledgement.")
            return True
        return False

    def serial_command_callback(self, message):
        command = message.data.strip().upper()
        allowed_simple = ("ARM", "DISARM", "HOLD", "DISABLE", "STATUS", "PING")
        if not (command in allowed_simple or command.startswith("SERVO ")):
            self.get_logger().warning("Ignored unsafe serial command '%s'." % command)
            return
        if self.dry_run or not self.hardware_enabled:
            self.protocol.last_response = "DRY_RUN %s" % command
            self.get_logger().info("Dry-run Arduino command: %s" % command)
            return

        verb = command.split(maxsplit=1)[0]
        if verb == "ARM":
            self.refresh_stack_topology(force=True)
            if not self.arm_interlocks_safe():
                self.arm_requested = self.auto_arm
                if self.stack_conflict_topics:
                    self.last_error = (
                        "ARM blocked: duplicate VOLT stacks publish the same "
                        "control or actuator-authority topics"
                    )
                elif not self.firmware_capability_allows_motion():
                    self.last_error = self.firmware_capability_failure(
                        "ARM blocked"
                    )
                elif not self.owner_allows_hardware_output():
                    self.last_error = (
                        "ARM blocked: command router must report recent "
                        "MOTION ownership"
                    )
                elif not self.motion_safe_to_arm():
                    self.last_error = (
                        "ARM blocked: motion controller must certify the "
                        "stopped calibrated WALK_POSE; ARM from sitting is "
                        "forbidden"
                    )
                else:
                    self.last_error = (
                        "ARM blocked: a recent stable 12-joint frame accepted "
                        "under the current MOTION ownership is required"
                    )
                self.get_logger().warning(
                    self.last_error,
                    throttle_duration_sec=2.0,
                )
                return

        if verb == "SERVO" and not self.owner_allows_hardware_output():
            self.last_error = (
                "SERVO blocked: fresh MOTION ownership is required in normal mode"
            )
            self.enforce_owner_interlock(self.last_error)
            return

        if not self.connected and not self.connect():
            return

        if verb == "ARM":
            self.last_error = ""
            self.arm_requested = True
            if not self.protocol.ready:
                if time.monotonic() - self.last_ping_time >= self.reconnect_period:
                    self.send_protocol_command("PING")
                self.get_logger().warning("ARM queued until Arduino handshake completes.")
                return
            self.send_arm_if_ready()
            return

        if verb in SAFE_STOP_COMMANDS:
            self.arm_requested = False

        if verb == "SERVO" and not self.hardware_stream_allowed():
            self.last_error = (
                "SERVO blocked: Arduino is not ready, armed, and firmware-compatible"
            )
            self.get_logger().warning(self.last_error, throttle_duration_sec=2.0)
            return

        if self.send_protocol_command(command):
            self.get_logger().info(
                "Sent Arduino command %s; waiting for acknowledgement." % verb
            )

    def build_frame(self, message):
        named = named_positions_from_ordered(message.data)
        return self.calibration.channel_frame_from_positions(named)

    def record_joint_rate(self, frame, now):
        """Log per-channel deg/s so gait limits can be tuned from measurement.

        Sampled before the send-rate gate: this is the rate the controller
        demands, which is what the firmware slew ceiling either passes or
        clips.  A ceiling that sits just above the demand does not merely slow
        that step -- the clipped joint falls behind its gait phase and stays
        behind for the rest of the swing, so the margin here is what matters.
        """
        if not self.joint_rate_diagnostic:
            return

        previous = self._rate_prev_deg
        previous_time = self._rate_prev_time
        self._rate_prev_deg = list(frame)
        self._rate_prev_time = now
        if previous is None or previous_time <= 0.0:
            return
        dt = now - previous_time
        # Reject non-monotonic or stalled samples; a tiny dt turns quantisation
        # noise into a meaningless spike.
        if not 0.002 <= dt <= 0.5:
            return

        rates = [abs(new - old) / dt for new, old in zip(frame, previous)]
        peak = max(rates)
        peak_channel = rates.index(peak)
        clipped = peak > self.firmware_slew_limit_dps

        self._rate_total_samples += 1
        if clipped:
            self._rate_clipped_samples += 1
        if peak > self._rate_peak_dps:
            self._rate_peak_dps = peak
            self._rate_peak_channel = peak_channel

        if self._rate_file is None and self.joint_rate_diagnostic_output:
            try:
                self._rate_file = open(
                    self.joint_rate_diagnostic_output, "w", buffering=1
                )
                header = ["t", "dt"]
                header += ["deg%d" % i for i in range(len(frame))]
                header += ["dps%d" % i for i in range(len(frame))]
                header += ["peak_dps", "peak_channel", "clipped"]
                self._rate_file.write(",".join(header) + "\n")
            except OSError as exc:
                self.joint_rate_diagnostic_output = ""
                self.get_logger().warning(
                    "Joint-rate diagnostic file unavailable: %s" % exc
                )

        if self._rate_file is not None:
            row = ["%.6f" % now, "%.6f" % dt]
            row += ["%.2f" % value for value in frame]
            row += ["%.1f" % value for value in rates]
            row += ["%.1f" % peak, str(peak_channel), "1" if clipped else "0"]
            try:
                self._rate_file.write(",".join(row) + "\n")
            except OSError:
                self._rate_file = None

        if now - self._rate_last_report >= 2.0 and self._rate_total_samples:
            self._rate_last_report = now
            clip_pct = (
                100.0 * self._rate_clipped_samples / self._rate_total_samples
            )
            self.get_logger().info(
                "joint-rate: peak %.0f deg/s on ch%d, ceiling %.0f deg/s, "
                "%.1f%% of %d samples would clip"
                % (
                    self._rate_peak_dps,
                    self._rate_peak_channel,
                    self.firmware_slew_limit_dps,
                    clip_pct,
                    self._rate_total_samples,
                )
            )

    def command_callback(self, message):
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

        try:
            frame, details = self.build_frame(message)
            line = format_frame_command(frame)
        except (CalibrationError, TypeError, ValueError) as exc:
            self.frames_rejected += 1
            self.last_error = str(exc)
            self.get_logger().warning("Rejected joint command: %s" % exc, throttle_duration_sec=2.0)
            return

        now = self.now_seconds()
        frame = self.apply_deadband_feedforward(frame)
        self.record_joint_rate(frame, now)
        self.note_firmware_guard_clips(frame, now)
        if not self.frame_send_slot_available(now):
            return

        self.last_send_time = now
        self.cache_frame(frame, details, now)
        if self.dry_run or not self.hardware_enabled:
            self.frames_sent += 1
            self.log_frame(line, details)
            return

        if not self.owner_allows_hardware_output():
            self.frames_blocked += 1
            self.enforce_owner_interlock(
                "FRAME blocked: command router ownership is not fresh MOTION"
            )
            return
        if not self.connected and not self.connect():
            self.frames_blocked += 1
            return
        if not self.hardware_stream_allowed():
            self.frames_blocked += 1
            self.get_logger().warning(
                "FRAME blocked until Arduino is ready, firmware-compatible, "
                "and ARM is acknowledged.",
                throttle_duration_sec=2.0,
            )
            return
        # STATUS responses exceed the Nano TX ring. At 57600 baud the firmware
        # may spend tens of milliseconds printing one, so retain only the newest
        # cached target until its complete response arrives. Safety commands use
        # a separate callback and always bypass this cosmetic/query gate.
        if self.serial_query_inflight:
            return
        if self.send_frame_payload(frame):
            self.frames_sent += 1
        else:
            self.frames_rejected += 1

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
        topology_safe = self.refresh_stack_topology()
        if not self.dry_run and self.hardware_enabled:
            if not topology_safe:
                self.enforce_owner_interlock(
                    "Hardware HOLD: duplicate VOLT stack publishers detected"
                )
                self.publish_status()
                return
            if not self.connected:
                self.connect()
            if (
                self.connected
                and not self.baud_locked
                and self.connect_opened_time > 0.0
                and time.monotonic() - self.connect_opened_time
                > self.baud_probe_timeout
                and len(self.baud_candidates) > 1
            ):
                stale = self.baud_candidates[self.baud_index]
                self.baud_index = (
                    (self.baud_index + 1) % len(self.baud_candidates)
                )
                self.disconnect(
                    "no valid firmware response at %d baud; trying %d"
                    % (stale, self.baud_candidates[self.baud_index])
                )
                self.publish_status()
                return
            if self.connected:
                if (
                    self.require_motion_safe_to_arm
                    and not self.firmware_capability_compatible()
                    and (
                        self.protocol.armed
                        or self.protocol.pending_command == "ARM"
                    )
                ):
                    self.enforce_owner_interlock(
                        self.firmware_capability_failure("Hardware HOLD")
                    )
                if (
                    self.require_motion_safe_to_arm
                    and not self.owner_allows_hardware_output()
                ):
                    self.enforce_owner_interlock(
                        "Hardware HOLD: MOTION ownership status is missing or stale"
                    )
                self.read_available()
                now = time.monotonic()
                if (
                    self.serial_query_inflight
                    and now - self.serial_query_since
                    >= min(self.command_timeout, 0.20)
                ):
                    self.serial_query_inflight = ""
                    self.serial_query_since = 0.0
                ping_sent = False
                if (
                    (
                        not self.protocol.ready
                        or (
                            self.protocol.host_sync_required
                            and not self.protocol.host_ping_seen
                        )
                    )
                    and not self.protocol.pending_command
                    and now - self.last_ping_time >= self.reconnect_period
                ):
                    ping_sent = self.send_protocol_command("PING")
                pending = self.protocol.pending_command
                guarded_pending = guarded_pending_command(
                    pending,
                    self.arm_interlocks_safe(),
                )
                retry_timeout = (
                    self.arm_ack_timeout
                    if guarded_pending == "ARM"
                    else self.command_timeout
                )
                if pending == "ARM" and guarded_pending == "HOLD":
                    # The Arduino may have received ARM even when its ACK was
                    # lost. Cancel immediately rather than retrying after the
                    # motion status has become active or stale.
                    self.arm_requested = False
                    self.last_error = (
                        "Pending ARM cancelled: arming interlocks are no longer safe"
                    )
                    self.get_logger().warning(
                        self.last_error,
                        throttle_duration_sec=2.0,
                    )
                    self.send_protocol_command("HOLD")
                elif (
                    guarded_pending
                    and now - self.last_protocol_command_time >= retry_timeout
                ):
                    # State-changing commands are idempotent. Retry without
                    # ever assuming the missing acknowledgement succeeded.
                    self.send_protocol_command(guarded_pending)
                self.send_arm_if_ready()
                # Poll firmware link-health counters so the GUI DIAGNOSTICS
                # tab can show CRC_FAIL/SEQ_GAP/loop timings.  A STATUS query
                # sets serial_query_inflight, and BOTH the live frame path and
                # send_cached_frame() refuse while that is set -- so polling
                # during the ARM handshake makes the post-ARM cached frame fail
                # and drops the bridge into Hardware HOLD.  Poll only once
                # streaming is actually established, never while arming.
                if (
                    self.protocol.ready
                    and self.protocol.armed
                    and self.protocol.can_stream_frames
                    and not self.arm_requested
                    and not self.protocol.pending_command
                    and not self.serial_query_inflight
                    and now - self.last_firmware_status_poll >= 10.0
                ):
                    if self.send_protocol_command("STATUS"):
                        self.last_firmware_status_poll = now
                if (
                    self.face_inflight_key
                    and now - self.face_inflight_since >= self.command_timeout
                ):
                    # A lost visual ACK is not safety critical. Coalesce a
                    # retry behind any newer value for the same setting.
                    key = self.face_inflight_key
                    value = self.face_inflight_value
                    self.face_inflight_key = ""
                    self.face_inflight_value = None
                    self.face_inflight_command = ""
                    self.face_inflight_since = 0.0
                    if self.face_desired.get(key) == value and key not in self.face_dirty:
                        self.face_dirty.append(key)
                        self.face_queue_settling = True
                if (
                    self.host_sync_inflight
                    and now - self.host_sync_since >= self.command_timeout
                ):
                    # HOST SYNC is idempotent. A lost ACK must neither block
                    # frames nor grow the visual queue; retry one bounded marker.
                    self.host_sync_inflight = False
                    self.host_sync_since = 0.0
                if not ping_sent:
                    self.service_face_queue(now)
        self.publish_status()

    def apply_deadband_feedforward(self, frame):
        """Bias each channel past its static-friction deadband.

        Applied at the last stage that still knows the commanded step and
        its direction, and before the guard check so what gets reported as
        clipped is what is actually sent.  Every servo's deadband_deg
        defaults to 0.0, so this is an exact no-op until a real value is
        measured on the bench.
        """
        previous = getattr(self, "_deadband_previous", None)
        self._deadband_previous = list(frame)
        by_channel = getattr(self, "_deadband_by_channel", None)
        if by_channel is None:
            by_channel = {
                servo.pca_channel: servo.deadband_deg
                for servo in self.calibration.servos.values()
            }
            self._deadband_by_channel = by_channel
        if previous is None or len(previous) != len(frame):
            return frame
        if not any(by_channel.values()):
            return frame
        biased = list(frame)
        for channel, value in enumerate(frame):
            deadband = by_channel.get(channel, 0.0)
            if deadband > 0.0:
                biased[channel] = value + deadband_feedforward(
                    value - previous[channel], deadband
                )
        return biased

    def note_firmware_guard_clips(self, frame, now):
        """Record any channel the firmware's travel guard will truncate."""
        clips = firmware_guard_clips(frame)
        if not clips:
            # Forget a clip that has stopped happening. The banner this feeds
            # has to describe the CURRENT command, not an all-time record --
            # the pre-arm idle frame alone would otherwise pin it on forever.
            if (
                getattr(self, "guard_clip_channels", None)
                and now - getattr(self, "guard_clip_last_seen", 0.0) >= 3.0
            ):
                self.guard_clip_channels = {}
            return
        self.guard_clip_last_seen = now
        if not hasattr(self, "guard_clip_channels"):
            self.guard_clip_channels = {}
            self.guard_clip_frames = 0
            self.guard_clip_last_log = 0.0
        self.guard_clip_frames += 1
        for channel, commanded, guard in clips:
            worst = self.guard_clip_channels.get(channel)
            excess = abs(commanded - guard)
            if worst is None or excess > worst[0]:
                self.guard_clip_channels[channel] = (excess, commanded, guard)
        if now - self.guard_clip_last_log >= 2.0:
            self.guard_clip_last_log = now
            self.get_logger().warning(
                "FIRMWARE TRAVEL GUARD is truncating commanded motion: %s. "
                "The gait is asking for travel this channel is not allowed "
                "to make, so that foot lands short on every stride."
                % ", ".join(
                    "ch%d commanded %.2f deg vs guard %.2f (%.2f deg lost)"
                    % (channel, commanded, guard, excess)
                    for channel, (excess, commanded, guard)
                    in sorted(self.guard_clip_channels.items())
                )
            )

    def publish_status(self):
        message = String()
        rate_now = self.now_seconds()
        rate_elapsed = rate_now - self.frame_rate_window_start
        if rate_elapsed >= 1.0:
            self.frame_rate = (
                self.frames_sent - self.frame_rate_window_sent
            ) / max(rate_elapsed, 1e-9)
            self.frame_rate_window_start = rate_now
            self.frame_rate_window_sent = self.frames_sent
        age = self.frame_age()
        frame_ready = self.frame_ready_to_arm()
        frame_stable_age = self.frame_stable_age()
        frame_stable = self.frame_is_stable()
        owner_age = self.owner_status_age()
        owner_fresh = self.owner_status_is_fresh()
        owner_allowed = self.owner_allows_hardware_output()
        velocity_requirement_fresh = self.motion_velocity_requirement_is_fresh()
        firmware_compatible = self.firmware_capability_compatible()
        face_expression = self.protocol.face_expression or self.face_desired.get(
            "expression", ""
        )
        face_color = self.protocol.led_color or self.face_desired.get("color", ())
        face_color_b = self.protocol.led_color_b or self.face_desired.get(
            "alternate_color", ()
        )
        face_brightness = self.protocol.led_brightness
        if face_brightness < 0:
            face_brightness = int(self.face_desired.get("brightness", -1))
        face_effect = self.protocol.led_effect or self.face_desired.get("effect", "")
        face_speed = self.protocol.led_speed_ms or int(
            self.face_desired.get("speed", 0)
        )
        if self.protocol.face_status_seen:
            face_enabled = self.protocol.led_enabled
        else:
            face_enabled = (
                face_effect != "off" and face_expression != "shutdown"
                and bool(face_effect or face_expression or face_color)
            )
        if not self.protocol.host_sync_required:
            host_sync_state = "legacy"
        elif self.protocol.host_synced:
            host_sync_state = "synced"
        elif self.host_sync_error:
            host_sync_state = "error"
        elif not self.protocol.host_ping_seen:
            host_sync_state = "waiting_ping"
        elif self.face_dirty or self.face_inflight_key:
            host_sync_state = "applying_snapshot"
        elif self.face_status_pending or self.face_status_request_time > 0.0:
            host_sync_state = "verifying_snapshot"
        elif self.host_sync_inflight:
            host_sync_state = "finalizing"
        else:
            # No desired visual snapshot has arrived yet. Firmware intentionally
            # retains its cyan loading animation in this state.
            host_sync_state = "loading"
        face_loading = (
            self.protocol.host_sync_required
            and not self.protocol.host_synced
        )
        clamped = [item["joint"] for item in self.last_details if item["clamped"]]
        message.data = (
            "host=%s connected=%d ready=%d armed=%d streaming=%d "
            "output_enabled=%d "
            "dry_run=%d hardware_enabled=%d calibration_valid=%d motion_safe=%d "
            "firmware_id=%s protocol_version=%d required_protocol_version=%d "
            "max_dps=%.1f required_max_dps=%.1f required_max_dps_known=%d "
            "required_max_dps_fresh=%d motion_hardware_mode=%d "
            "firmware_compatible=%d capability_required=%d "
            "stack_unique=%d stack_conflict=%s "
            "owner=%s owner_fresh=%d owner_required=%d owner_allowed=%d "
            "owner_age=%.3f owner_epoch=%d "
            "frame_ready=%d frame_age=%.3f frame_seq=%d "
            "frame_owner=%s frame_owner_epoch=%d "
            "frame_stable=%d frame_stable_age=%.3f frame_stable_samples=%d "
            "age=%.3f sent=%d frame_rate=%.2f rejected=%d blocked=%d "
            "face_connected=%d face_supported=%d face_synced=%d "
            "host_sync=%d host_sync_required=%d host_ping=%d "
            "host_snapshot=%d host_synced=%d host_sync_pending=%d "
            "host_sync_state=%s face_loading=%d host_sync_error=%s "
            "face_enabled=%d face_expression=%s face_color=%s "
            "face_color_b=%s face_brightness=%d "
            "face_effective_brightness=%d face_brightness_limit=%d "
            "face_led_count=%d face_effect=%s face_speed=%d "
            "face_queued=%d face_sent=%d face_rejected=%d led_error=%s "
            "pending=%s error=%s response=%s "
            "clamped=%s guard_clip=%s guard_clip_frames=%d frame=%s"
            % (
                BRIDGE_HOST,
                int(self.connected),
                int(self.protocol.ready),
                int(self.protocol.armed),
                int(self.hardware_stream_allowed() and owner_allowed),
                int(self.protocol.output_enabled),
                int(self.dry_run),
                int(self.hardware_enabled),
                int(self.calibration_valid),
                int(self.motion_safe_to_arm()),
                status_token(self.protocol.firmware_id),
                self.protocol.protocol_version,
                self.required_protocol_version,
                self.protocol.max_dps,
                self.motion_required_max_dps,
                int(self.motion_required_max_dps_known),
                int(velocity_requirement_fresh),
                int(self.motion_hardware_mode),
                int(firmware_compatible),
                int(self.require_motion_safe_to_arm),
                int(not self.stack_conflict_topics),
                status_token(",".join(self.stack_conflict_topics)),
                status_token(self.command_owner),
                int(owner_fresh),
                int(self.require_motion_safe_to_arm),
                int(owner_allowed),
                owner_age,
                self.owner_epoch,
                int(frame_ready),
                age,
                self.frame_sequence,
                status_token(self.last_frame_owner),
                self.last_frame_owner_epoch,
                int(frame_stable),
                frame_stable_age,
                self.frame_stable_samples,
                age,
                self.frames_sent,
                self.frame_rate,
                self.frames_rejected,
                self.frames_blocked,
                int(
                    self.connected
                    and self.protocol.ready
                    and self.protocol.face_supported
                ),
                int(self.protocol.face_supported),
                int(self.face_is_synced()),
                int(self.protocol.host_synced),
                int(self.protocol.host_sync_required),
                int(self.protocol.host_ping_seen),
                int(self.protocol.host_snapshot_seen),
                int(self.protocol.host_synced),
                int(self.host_sync_inflight),
                status_token(host_sync_state),
                int(face_loading),
                status_token(self.host_sync_error),
                int(face_enabled),
                status_token(face_expression),
                status_token(",".join(str(value) for value in face_color)),
                status_token(",".join(str(value) for value in face_color_b)),
                face_brightness,
                self.protocol.led_effective_brightness,
                self.protocol.led_brightness_limit,
                self.protocol.face_led_count,
                status_token(face_effect),
                face_speed,
                self.face_commands_queued,
                self.face_commands_sent,
                self.face_commands_rejected,
                status_token(self.protocol.led_error),
                status_token(self.protocol.pending_command),
                status_token(
                    self.calibration_error
                    or self.last_error
                    or self.protocol.last_error
                ),
                status_token(self.protocol.last_response),
                ",".join(clamped),
                ",".join(
                    "ch%d:%.2f<%.2f" % (channel, commanded, guard)
                    for channel, (_excess, commanded, guard)
                    in sorted(getattr(self, "guard_clip_channels", {}).items())
                ) or "-",
                int(getattr(self, "guard_clip_frames", 0)),
                " ".join("%.2f" % value for value in self.last_frame),
            )
        )
        if self.protocol.firmware_counters:
            message.data += " " + " ".join(
                "fw_%s=%s" % (name.lower(), status_token(value))
                for name, value in sorted(
                    self.protocol.firmware_counters.items()
                )
            )
        message.data += " binary_frames=%d frame_seq_tx=%d" % (
            int(self.binary_frames_supported()),
            self.binary_frame_seq,
        )
        self.status_publisher.publish(message)

    def shutdown(self):
        if self.serial_port is not None:
            try:
                # Servo safety is issued first. The face fade command follows
                # immediately and runs asynchronously in firmware; no ACK wait
                # delays HOLD or process shutdown.
                self.write_line("HOLD")
                self.write_line("FACE shutdown")
                self.serial_port.flush()
                self.serial_port.close()
            except (SerialException, OSError):
                pass
        self.serial_port = None
        self.connected = False
        self.protocol.reset()
        self.serial_lines.reset()
        self.serial_query_inflight = ""
        self.serial_query_since = 0.0


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
