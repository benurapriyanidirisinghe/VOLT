#!/usr/bin/env python3

"""Explicit, finite, support-stand tests for the physical VOLT robot.

This module deliberately separates pure Cartesian trajectory generation from
the ROS command-line runner:

* importing it never initializes ROS or touches hardware;
* the CLI has no default test and refuses to run without ``--execute``;
* Cartesian tests are requested from the motion controller on
  ``/volt/physical_test`` and never publish joint arrays;
* gait tests use the existing MOTION/gait/cmd_vel/action interfaces; and
* every exit path sends cancel (when applicable), zero velocity, STOP, HOLD,
  and never ARM.

The existing motion controller remains responsible for converting the
Cartesian frames below through VOLT's canonical IK and for publishing its
normal ``/volt/joint_commands/motion`` stream through the command router.
Servo calibration is intentionally outside this module.
"""

import argparse
import json
import math
import re
import sys
import time
import uuid
from dataclasses import dataclass

from volt_kinematics import (
    FOOT_LIMIT,
    JOINT_NAMES,
    LEG_LIMIT,
    LEG_ORDER,
    NOMINAL_FEET,
    SHOULDER_LIMIT,
    feet_to_joint_positions_diagnostic,
    smootherstep,
)


STAND = "stand"
SLOW_SQUAT = "slow-squat"
WEIGHT_SHIFT = "weight-shift"
SINGLE_LEG_LIFT = "single-leg-lift"
SINGLE_LEG_STEP = "single-leg-step"
DIAGONAL_PAIR_LIFT = "diagonal-pair-lift"
ZERO_STRIDE_TROT = "zero-stride-trot"
SLOW_CREEP = "slow-creep"
TROT_SPEED_TEST = "trot-speed"
EMERGENCY_STOP = "emergency-stop"

CARTESIAN_TEST_MODES = (
    STAND,
    SLOW_SQUAT,
    WEIGHT_SHIFT,
    SINGLE_LEG_LIFT,
    SINGLE_LEG_STEP,
    DIAGONAL_PAIR_LIFT,
)
GAIT_TEST_MODES = (
    ZERO_STRIDE_TROT,
    SLOW_CREEP,
    TROT_SPEED_TEST,
)
TEST_MODES = CARTESIAN_TEST_MODES + GAIT_TEST_MODES + (EMERGENCY_STOP,)

DIAGONAL_PAIRS = (
    ("front_left", "rear_right"),
    ("front_right", "rear_left"),
)

# Deliberately smaller than normal gait amplitudes. These tests establish basic
# load-bearing behavior before an operator attempts a translating gait.
WEIGHT_SHIFT_X = 0.006
WEIGHT_SHIFT_Y = 0.008
SINGLE_LEG_LIFT_HEIGHT = 0.020
SLOW_SQUAT_DEPTH = 0.018
SINGLE_LEG_STEP_HEIGHT = 0.015
SINGLE_LEG_STEP_FORWARD = 0.010
DIAGONAL_PAIR_LIFT_HEIGHT = 0.015
MAX_CARTESIAN_DISPLACEMENT = 0.020
MAX_TEST_JOINT_SPEED = math.radians(25.0)
DEFAULT_SAMPLE_RATE = 30.0

# A shell argument is used instead of an interactive prompt so the
# acknowledgement is explicit, auditable in command history, and testable.
SUPPORT_STAND_ACKNOWLEDGEMENT = "VOLT IS ON A SUPPORT STAND"

DEFAULT_DURATIONS = {
    STAND: 5.0,
    SLOW_SQUAT: 7.0,
    WEIGHT_SHIFT: 8.0,
    SINGLE_LEG_LIFT: 6.0,
    SINGLE_LEG_STEP: 8.0,
    DIAGONAL_PAIR_LIFT: 10.0,
    ZERO_STRIDE_TROT: 6.0,
    SLOW_CREEP: 8.0,
    TROT_SPEED_TEST: 6.0,
}
MIN_DURATIONS = {
    STAND: 2.0,
    SLOW_SQUAT: 5.0,
    WEIGHT_SHIFT: 6.0,
    SINGLE_LEG_LIFT: 4.0,
    SINGLE_LEG_STEP: 6.0,
    DIAGONAL_PAIR_LIFT: 8.0,
    ZERO_STRIDE_TROT: 3.0,
    SLOW_CREEP: 5.0,
    TROT_SPEED_TEST: 4.0,
}
MAX_DURATIONS = {
    mode: 20.0 for mode in DEFAULT_DURATIONS
}

# Existing controller interfaces and intentionally conservative commands.
PHYSICAL_TEST_TOPIC = "/volt/physical_test"
OWNER_TOPIC = "/volt/command_owner"
ACTION_TOPIC = "/volt/action"
GAIT_TOPIC = "/volt/gait"
CMD_VEL_TOPIC = "/cmd_vel"
SERIAL_COMMAND_TOPIC = "/volt/serial_command"
STATUS_TOPIC = "/volt/status"
SERIAL_STATUS_TOPIC = "/volt/serial_status"
ROUTER_STATUS_TOPIC = "/volt/command_router_status"
SLOW_CREEP_GAIT = "amble"
FAST_TROT_GAIT = "trot"
SLOW_CREEP_SPEED = 0.004
TROT_SPEED_TEST_SPEED = 0.030
COMMAND_RATE = 20.0
REQUEST_KEEPALIVE_PERIOD = 0.20
STEP_KEEPALIVE_PERIOD = 0.20
DISCOVERY_TIMEOUT = 4.0
PREPARE_STOP_TIME = 0.60
MOTION_OWNER_SETTLE_TIME = 0.70
GAIT_SELECTION_SETTLE_TIME = 0.80
CLEANUP_STOP_SETTLE_TIME = 1.60
CLEANUP_STOP_TIMEOUT = 6.0
STATUS_FRESHNESS_TIMEOUT = 1.0
STATUS_CONFIRMATION_TIMEOUT = 4.0

_REQUEST_ID_PATTERN = re.compile(r"^[A-Za-z0-9_-]{8,64}$")
_REQUEST_COMMANDS = ("start", "keepalive", "cancel")


class PhysicalTestError(ValueError):
    """Raised when a test request violates a local safety invariant."""


@dataclass(frozen=True)
class CartesianFrame:
    """One body-frame Cartesian target sampled at a finite relative time."""

    time_from_start: float
    feet: dict


@dataclass(frozen=True)
class ExecutionPlan:
    """Validated CLI intent; constructing it has no ROS or hardware effects."""

    mode: str
    duration: float
    leg: str
    disable_output: bool


def _finite_float(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise PhysicalTestError("%s must be numeric" % label) from exc
    if not math.isfinite(result):
        raise PhysicalTestError("%s must be finite" % label)
    return result


def parse_key_value_status(payload):
    """Return normalized fields from JSON or whitespace key=value status."""
    try:
        decoded = json.loads(str(payload))
    except (TypeError, ValueError, json.JSONDecodeError):
        decoded = None
    if isinstance(decoded, dict):
        return {
            str(key).strip().lower(): value
            for key, value in decoded.items()
        }
    fields = {}
    for token in str(payload).replace(",", " ").split():
        key, separator, value = token.partition("=")
        if separator:
            fields[key.strip().lower()] = value.strip()
    return fields


def _status_zero_vector(status, name, tolerance=1e-6):
    values = status.get(name)
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return False
    try:
        values = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return False
    return all(
        math.isfinite(value) and abs(value) <= tolerance
        for value in values
    )


def physical_stack_status_errors(status, require_stopped=True):
    """Return fail-closed reasons for a finite physical-test stack."""
    if not isinstance(status, dict):
        return ["missing valid /volt/status JSON"]
    errors = []
    exact = (
        ("hardware_mode", True),
        ("use_sim_time", False),
        ("physical_tests_enabled", True),
        ("command_owner", "MOTION"),
        ("motion_authorized", True),
    )
    for name, expected in exact:
        actual = status.get(name)
        matches = actual is expected if isinstance(expected, bool) else (
            actual == expected
        )
        if not matches:
            errors.append("%s must equal %r" % (name, expected))
    if require_stopped:
        stopped_exact = (
            ("state", "standing"),
            ("moving", False),
            ("motion_active", False),
            ("step_in_place", False),
            ("physical_test_active", False),
            ("physical_test_returning", False),
        )
        for name, expected in stopped_exact:
            actual = status.get(name)
            matches = actual is expected if isinstance(expected, bool) else (
                actual == expected
            )
            if not matches:
                errors.append("%s must equal %r" % (name, expected))
        for name in ("pending_gait", "pending_pose_action"):
            if name not in status or status[name] not in (None, ""):
                errors.append("%s must be empty" % name)
        stance_legs = status.get("stance_legs")
        if (
            not isinstance(stance_legs, (list, tuple))
            or len(stance_legs) != len(LEG_ORDER)
            or set(stance_legs) != set(LEG_ORDER)
        ):
            errors.append("all four canonical legs must be in stance")
        swing_legs = status.get("swing_legs")
        if not isinstance(swing_legs, (list, tuple)) or swing_legs:
            errors.append("swing_legs must be empty")
        for name in ("requested_velocity", "filtered_velocity"):
            if not _status_zero_vector(status, name):
                errors.append("%s must be a finite zero vector" % name)
    return errors


def physical_stack_is_settled(status):
    """Return true only after status confirms a complete grounded stop."""
    if not isinstance(status, dict):
        return False
    required = {
        "state",
        "moving",
        "motion_active",
        "step_in_place",
        "physical_test_active",
        "physical_test_returning",
        "pending_pose_action",
        "swing_legs",
        "stance_legs",
    }
    if not required.issubset(status):
        return False
    if status.get("state") not in ("standing", "hold"):
        return False
    if any(
        status.get(name) is not False
        for name in (
            "moving",
            "motion_active",
            "step_in_place",
            "physical_test_active",
            "physical_test_returning",
        )
    ):
        return False
    if status.get("pending_pose_action") not in (None, ""):
        return False
    swing_legs = status.get("swing_legs")
    stance_legs = status.get("stance_legs")
    return (
        isinstance(swing_legs, (list, tuple))
        and not swing_legs
        and isinstance(stance_legs, (list, tuple))
        and len(stance_legs) == len(LEG_ORDER)
        and set(stance_legs) == set(LEG_ORDER)
    )


def serial_stack_ready(fields):
    """Accept a deliberate dry run or a compatible acknowledged live ARM."""
    if not isinstance(fields, dict):
        return False
    if str(fields.get("dry_run", "")) == "1":
        return (
            str(fields.get("hardware_enabled", "")) == "0"
            and str(fields.get("calibration_valid", "")) == "1"
        )
    return all(
        str(fields.get(name, "")) == "1"
        for name in (
            "hardware_enabled",
            "connected",
            "ready",
            "armed",
            "firmware_compatible",
            "calibration_valid",
        )
    )


def _copy_nominal_feet():
    return {
        leg: tuple(float(value) for value in NOMINAL_FEET[leg])
        for leg in LEG_ORDER
    }


def _lift_pulse(progress):
    """Return a zero-slope ground -> peak -> ground quintic pulse."""
    progress = max(0.0, min(1.0, _finite_float(progress, "progress")))
    if progress <= 0.5:
        return smootherstep(progress * 2.0)
    return smootherstep((1.0 - progress) * 2.0)


def _piecewise_smooth(progress, waypoints):
    """Interpolate equally timed waypoints with zero velocity at every knot."""
    progress = max(0.0, min(1.0, _finite_float(progress, "progress")))
    if len(waypoints) < 2:
        raise PhysicalTestError("at least two waypoints are required")
    if progress >= 1.0:
        return tuple(waypoints[-1])
    scaled = progress * (len(waypoints) - 1)
    index = min(int(math.floor(scaled)), len(waypoints) - 2)
    blend = smootherstep(scaled - index)
    start = waypoints[index]
    end = waypoints[index + 1]
    return tuple(
        float(first) + (float(second) - float(first)) * blend
        for first, second in zip(start, end)
    )


def cartesian_frame_at(mode, elapsed, duration, leg=None):
    """Return a bounded pure Cartesian test frame.

    ``elapsed`` is clamped to the finite interval. All trajectories start and
    end at ``NOMINAL_FEET`` with quintic zero-velocity endpoints.
    """
    mode = str(mode).strip().lower()
    if mode not in CARTESIAN_TEST_MODES:
        raise PhysicalTestError(
            "mode must be one of %s" % (CARTESIAN_TEST_MODES,)
        )
    duration = _finite_float(duration, "duration")
    if duration <= 0.0:
        raise PhysicalTestError("duration must be positive")
    elapsed = _finite_float(elapsed, "elapsed")
    elapsed = max(0.0, min(duration, elapsed))
    progress = elapsed / duration

    selected_leg = str(leg or "").strip().lower()
    # Pure trajectory callers may preview the canonical front-left example;
    # the executable CLI and GUI contracts still require an explicit leg.
    if mode == SINGLE_LEG_STEP and not selected_leg:
        selected_leg = "front_left"
    if mode in (SINGLE_LEG_LIFT, SINGLE_LEG_STEP):
        if selected_leg not in LEG_ORDER:
            raise PhysicalTestError(
                "%s requires one canonical leg from %s"
                % (mode, LEG_ORDER)
            )
    elif selected_leg:
        raise PhysicalTestError("--leg is only valid for single-leg-lift")

    feet = _copy_nominal_feet()
    if mode == SLOW_SQUAT:
        squat = SLOW_SQUAT_DEPTH * _lift_pulse(progress)
        for leg_name in LEG_ORDER:
            x, y, z = feet[leg_name]
            feet[leg_name] = (x, y, z + squat)
    elif mode == WEIGHT_SHIFT:
        # Translating all body-frame foot targets together shifts the body over
        # planted toes. Z never changes, so no foot is intentionally lifted.
        shift_x, shift_y = _piecewise_smooth(
            progress,
            (
                (0.0, 0.0),
                (WEIGHT_SHIFT_X, WEIGHT_SHIFT_Y),
                (0.0, 0.0),
                (-WEIGHT_SHIFT_X, -WEIGHT_SHIFT_Y),
                (0.0, 0.0),
            ),
        )
        for leg_name in LEG_ORDER:
            x, y, z = feet[leg_name]
            feet[leg_name] = (x + shift_x, y + shift_y, z)
    elif mode == SINGLE_LEG_LIFT:
        x, y, z = feet[selected_leg]
        feet[selected_leg] = (
            x,
            y,
            z + SINGLE_LEG_LIFT_HEIGHT * _lift_pulse(progress),
        )
    elif mode == SINGLE_LEG_STEP:
        offset_x, offset_z = _piecewise_smooth(
            progress,
            (
                (0.0, 0.0),
                (0.0, SINGLE_LEG_STEP_HEIGHT),
                (SINGLE_LEG_STEP_FORWARD, SINGLE_LEG_STEP_HEIGHT),
                (SINGLE_LEG_STEP_FORWARD, 0.0),
                (SINGLE_LEG_STEP_FORWARD, SINGLE_LEG_STEP_HEIGHT),
                (0.0, SINGLE_LEG_STEP_HEIGHT),
                (0.0, 0.0),
            ),
        )
        x, y, z = feet[selected_leg]
        feet[selected_leg] = (x + offset_x, y, z + offset_z)
    elif mode == DIAGONAL_PAIR_LIFT:
        if progress <= 0.5:
            active_pair = DIAGONAL_PAIRS[0]
            pair_progress = progress * 2.0
        else:
            active_pair = DIAGONAL_PAIRS[1]
            pair_progress = (progress - 0.5) * 2.0
        lift = DIAGONAL_PAIR_LIFT_HEIGHT * _lift_pulse(pair_progress)
        for leg_name in active_pair:
            x, y, z = feet[leg_name]
            feet[leg_name] = (x, y, z + lift)

    for leg_name in LEG_ORDER:
        target = feet[leg_name]
        if len(target) != 3 or not all(math.isfinite(value) for value in target):
            raise PhysicalTestError("%s target is not a finite XYZ point" % leg_name)
        nominal = NOMINAL_FEET[leg_name]
        if math.dist(target, nominal) > MAX_CARTESIAN_DISPLACEMENT + 1e-12:
            raise PhysicalTestError(
                "%s target exceeds the conservative Cartesian displacement"
                % leg_name
            )
    return CartesianFrame(elapsed, feet)


def generate_cartesian_trajectory(
    mode,
    duration=None,
    sample_rate=DEFAULT_SAMPLE_RATE,
    leg=None,
):
    """Sample a finite Cartesian test, including both exact endpoints."""
    mode = str(mode).strip().lower()
    if duration is None:
        if mode not in DEFAULT_DURATIONS:
            raise PhysicalTestError("no default duration for mode '%s'" % mode)
        duration = DEFAULT_DURATIONS[mode]
    duration = _finite_float(duration, "duration")
    sample_rate = _finite_float(sample_rate, "sample_rate")
    if duration <= 0.0:
        raise PhysicalTestError("duration must be positive")
    if not 5.0 <= sample_rate <= 100.0:
        raise PhysicalTestError("sample_rate must be in [5, 100] Hz")
    intervals = max(1, int(math.ceil(duration * sample_rate)))
    return tuple(
        cartesian_frame_at(
            mode,
            duration * index / intervals,
            duration,
            leg=leg,
        )
        for index in range(intervals + 1)
    )


def _joint_limit(index):
    return (SHOULDER_LIMIT, LEG_LIMIT, FOOT_LIMIT)[index % 3]


def validate_cartesian_trajectory(
    frames,
    maximum_joint_speed=MAX_TEST_JOINT_SPEED,
):
    """Validate finite canonical IK output, limits, continuity, and speed.

    The returned tuple mirrors ``frames`` and contains canonical 12-joint
    tuples. A projected IK target is rejected instead of silently relying on
    workspace or joint-limit clamping.
    """
    try:
        frames = tuple(frames)
    except TypeError as exc:
        raise PhysicalTestError("frames must be iterable") from exc
    if not frames:
        raise PhysicalTestError("trajectory must contain at least one frame")
    maximum_joint_speed = _finite_float(
        maximum_joint_speed,
        "maximum_joint_speed",
    )
    if maximum_joint_speed <= 0.0:
        raise PhysicalTestError("maximum_joint_speed must be positive")

    joint_frames = []
    previous_time = None
    previous_positions = None
    for index, frame in enumerate(frames):
        if not isinstance(frame, CartesianFrame):
            raise PhysicalTestError("frame %d is not a CartesianFrame" % index)
        frame_time = _finite_float(frame.time_from_start, "frame time")
        if previous_time is not None and frame_time <= previous_time:
            raise PhysicalTestError("trajectory times must increase strictly")
        try:
            positions, diagnostics = feet_to_joint_positions_diagnostic(
                frame.feet
            )
        except Exception as exc:
            raise PhysicalTestError("IK failed at frame %d: %s" % (index, exc)) from exc
        if diagnostics.get("projected_targets"):
            raise PhysicalTestError(
                "IK projected targets at frame %d: %s"
                % (index, diagnostics["projected_targets"])
            )
        if len(positions) != len(JOINT_NAMES):
            raise PhysicalTestError("IK did not return 12 canonical joints")
        for joint_index, value in enumerate(positions):
            if not math.isfinite(value):
                raise PhysicalTestError(
                    "%s is not finite at frame %d"
                    % (JOINT_NAMES[joint_index], index)
                )
            lower, upper = _joint_limit(joint_index)
            if value < lower - 1e-12 or value > upper + 1e-12:
                raise PhysicalTestError(
                    "%s is outside canonical limits at frame %d"
                    % (JOINT_NAMES[joint_index], index)
                )
        if previous_positions is not None:
            dt = frame_time - previous_time
            peak_speed = max(
                abs(current - previous) / dt
                for current, previous in zip(positions, previous_positions)
            )
            if peak_speed > maximum_joint_speed + 1e-9:
                raise PhysicalTestError(
                    "trajectory exceeds %.1f deg/s at frame %d"
                    % (math.degrees(maximum_joint_speed), index)
                )
        joint_frames.append(tuple(positions))
        previous_time = frame_time
        previous_positions = positions
    return tuple(joint_frames)


def validate_test_parameters(mode, duration=None, leg=None):
    """Validate mode-specific duration and leg selection without side effects."""
    mode = str(mode).strip().lower()
    if mode not in TEST_MODES:
        raise PhysicalTestError("unknown physical test mode '%s'" % mode)
    selected_leg = str(leg or "").strip().lower()

    if mode == EMERGENCY_STOP:
        if duration is not None:
            raise PhysicalTestError("emergency-stop does not accept --duration")
        if selected_leg:
            raise PhysicalTestError("emergency-stop does not accept --leg")
        return 0.0, ""

    if duration is None:
        duration = DEFAULT_DURATIONS[mode]
    duration = _finite_float(duration, "duration")
    if not MIN_DURATIONS[mode] <= duration <= MAX_DURATIONS[mode]:
        raise PhysicalTestError(
            "%s duration must be in [%.1f, %.1f] seconds"
            % (mode, MIN_DURATIONS[mode], MAX_DURATIONS[mode])
        )
    if mode in (SINGLE_LEG_LIFT, SINGLE_LEG_STEP):
        if selected_leg not in LEG_ORDER:
            raise PhysicalTestError(
                "%s requires --leg from %s" % (mode, LEG_ORDER)
            )
    elif selected_leg:
        raise PhysicalTestError("--leg is only valid for single-leg-lift")
    return duration, selected_leg


def authorize_execution(mode, execute, acknowledgement):
    """Enforce the two explicit CLI gates before ROS can be initialized."""
    mode = str(mode).strip().lower()
    if mode not in TEST_MODES:
        raise PhysicalTestError("an explicit valid --mode is required")
    if not bool(execute):
        raise PhysicalTestError(
            "--execute is required; no test is run in preview/default mode"
        )
    if (
        mode != EMERGENCY_STOP
        and str(acknowledgement or "").strip()
        != SUPPORT_STAND_ACKNOWLEDGEMENT
    ):
        raise PhysicalTestError(
            "type --acknowledge-support-stand '%s'"
            % SUPPORT_STAND_ACKNOWLEDGEMENT
        )
    return True


def build_execution_plan(
    mode,
    execute,
    acknowledgement="",
    duration=None,
    leg=None,
    disable_output=False,
):
    """Return fully validated CLI intent, still without initializing ROS."""
    authorize_execution(mode, execute, acknowledgement)
    mode = str(mode).strip().lower()
    duration, leg = validate_test_parameters(mode, duration, leg)
    if disable_output and mode != EMERGENCY_STOP:
        raise PhysicalTestError(
            "--disable-output is only valid with emergency-stop"
        )
    return ExecutionPlan(mode, duration, leg, bool(disable_output))


def physical_test_request_payload(
    command,
    mode,
    duration,
    request_id,
    leg=None,
):
    """Build the strict String-topic JSON envelope used by the controller."""
    command = str(command).strip().lower()
    if command not in _REQUEST_COMMANDS:
        raise PhysicalTestError(
            "physical-test command must be one of %s" % (_REQUEST_COMMANDS,)
        )
    mode = str(mode).strip().lower()
    if mode not in CARTESIAN_TEST_MODES:
        raise PhysicalTestError(
            "physical-test requests only support %s"
            % (CARTESIAN_TEST_MODES,)
        )
    duration, selected_leg = validate_test_parameters(mode, duration, leg)
    request_id = str(request_id).strip()
    if not _REQUEST_ID_PATTERN.fullmatch(request_id):
        raise PhysicalTestError(
            "request_id must contain 8-64 letters, digits, '_' or '-'"
        )
    return {
        "command": command,
        "mode": mode,
        "leg": selected_leg,
        "duration": duration,
        "request_id": request_id,
    }


def physical_test_request_json(
    command,
    mode,
    duration,
    request_id,
    leg=None,
):
    """Serialize one deterministic finite request for ``std_msgs/String``."""
    return json.dumps(
        physical_test_request_payload(
            command,
            mode,
            duration,
            request_id,
            leg=leg,
        ),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


def create_argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Run one explicit, finite VOLT physical test. This tool never ARM(s) "
            "hardware and always returns the stack to STOP/HOLD."
        )
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=TEST_MODES,
        help="Explicit test to run; there is no default mode.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Required live-action gate. Omitting it performs no ROS action.",
    )
    parser.add_argument(
        "--acknowledge-support-stand",
        "--acknowledge",
        default="",
        metavar="TEXT",
        help=(
            "Required for every non-emergency mode; type exactly: %s"
            % SUPPORT_STAND_ACKNOWLEDGEMENT
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="Finite mode duration in seconds (bounded per mode).",
    )
    parser.add_argument(
        "--leg",
        choices=LEG_ORDER,
        help="Required only for single-leg-lift.",
    )
    parser.add_argument(
        "--disable-output",
        action="store_true",
        help=(
            "With emergency-stop only, send firmware DISABLE instead of the "
            "default DISARM."
        ),
    )
    return parser


def _run_ros(plan):
    """Execute a validated plan through existing ROS interfaces."""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from std_msgs.msg import String
        try:
            from rclpy.signals import SignalHandlerOptions
        except ImportError:  # pragma: no cover - older rclpy fallback.
            SignalHandlerOptions = None
    except ImportError as exc:  # pragma: no cover - depends on ROS installation.
        raise RuntimeError(
            "ROS 2 Python packages are unavailable; source the Humble workspace"
        ) from exc

    request_id = uuid.uuid4().hex

    class PhysicalTestCommandNode(Node):
        def __init__(self):
            super().__init__("volt_physical_test_" + request_id[:8])
            self.velocity_publisher = self.create_publisher(
                Twist,
                CMD_VEL_TOPIC,
                10,
            )
            self.action_publisher = self.create_publisher(
                String,
                ACTION_TOPIC,
                10,
            )
            self.gait_publisher = self.create_publisher(
                String,
                GAIT_TOPIC,
                10,
            )
            self.owner_publisher = self.create_publisher(
                String,
                OWNER_TOPIC,
                10,
            )
            self.physical_test_publisher = self.create_publisher(
                String,
                PHYSICAL_TEST_TOPIC,
                10,
            )
            self.serial_command_publisher = self.create_publisher(
                String,
                SERIAL_COMMAND_TOPIC,
                10,
            )
            self.create_subscription(
                String,
                STATUS_TOPIC,
                self.status_callback,
                10,
            )
            self.create_subscription(
                String,
                SERIAL_STATUS_TOPIC,
                self.serial_status_callback,
                10,
            )
            self.create_subscription(
                String,
                ROUTER_STATUS_TOPIC,
                self.router_status_callback,
                10,
            )
            self.status_sequence = 0
            self.latest_status = None
            self.latest_status_time = 0.0
            self.serial_status_fields = {}
            self.router_owner = ""

        def status_callback(self, message):
            try:
                status = json.loads(message.data)
            except (TypeError, ValueError, json.JSONDecodeError):
                return
            if not isinstance(status, dict):
                return
            self.status_sequence += 1
            self.latest_status = status
            self.latest_status_time = time.monotonic()

        def serial_status_callback(self, message):
            self.serial_status_fields = parse_key_value_status(message.data)

        def router_status_callback(self, message):
            fields = parse_key_value_status(message.data)
            self.router_owner = str(fields.get("owner", "")).strip().upper()

        def status_is_fresh(self):
            return (
                self.latest_status is not None
                and 0.0
                <= time.monotonic() - self.latest_status_time
                <= STATUS_FRESHNESS_TIMEOUT
            )

        def wait_for_status(self, predicate, timeout, keepalive=None):
            deadline = time.monotonic() + max(0.0, float(timeout))
            period = 1.0 / COMMAND_RATE
            while rclpy.ok() and time.monotonic() < deadline:
                if keepalive is not None:
                    keepalive()
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.status_is_fresh() and predicate(self.latest_status):
                    return True
                time.sleep(period)
            return False

        @staticmethod
        def _text_message(text):
            message = String()
            message.data = str(text)
            return message

        @staticmethod
        def _velocity_message(forward=0.0):
            message = Twist()
            message.linear.x = float(forward)
            return message

        def publish_text(self, publisher, text):
            publisher.publish(self._text_message(text))

        def publish_zero(self):
            self.velocity_publisher.publish(self._velocity_message())

        def publish_stop(self):
            self.publish_text(self.action_publisher, "stop")

        def publish_owner(self, owner):
            self.publish_text(self.owner_publisher, owner)

        def publish_serial(self, command):
            command = str(command).strip().upper()
            if command not in ("HOLD", "DISARM", "DISABLE"):
                raise PhysicalTestError(
                    "physical-test tool may only publish safe serial commands"
                )
            self.publish_text(self.serial_command_publisher, command)

        def publish_request(self, command):
            payload = physical_test_request_json(
                command,
                plan.mode,
                plan.duration,
                request_id,
                leg=plan.leg,
            )
            self.publish_text(self.physical_test_publisher, payload)

        def spin_for(self, duration, callback):
            deadline = time.monotonic() + max(0.0, float(duration))
            period = 1.0 / COMMAND_RATE
            while rclpy.ok() and time.monotonic() < deadline:
                callback()
                rclpy.spin_once(self, timeout_sec=0.0)
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(min(period, remaining))

        def required_publishers(self):
            common = {
                OWNER_TOPIC: self.owner_publisher,
                ACTION_TOPIC: self.action_publisher,
                CMD_VEL_TOPIC: self.velocity_publisher,
                SERIAL_COMMAND_TOPIC: self.serial_command_publisher,
            }
            if plan.mode in CARTESIAN_TEST_MODES:
                common[PHYSICAL_TEST_TOPIC] = self.physical_test_publisher
            elif plan.mode in GAIT_TEST_MODES:
                common[GAIT_TOPIC] = self.gait_publisher
            return common

        def stopped_stack_ready(self):
            if not self.status_is_fresh():
                return False
            return (
                not physical_stack_status_errors(self.latest_status)
                and serial_stack_ready(self.serial_status_fields)
            )

        def wait_for_stack(self):
            deadline = time.monotonic() + DISCOVERY_TIMEOUT
            required = self.required_publishers()
            while rclpy.ok() and time.monotonic() < deadline:
                missing = [
                    topic
                    for topic, publisher in required.items()
                    if publisher.get_subscription_count() < 1
                ]
                if not missing:
                    return True
                rclpy.spin_once(self, timeout_sec=0.05)
            missing = [
                topic
                for topic, publisher in required.items()
                if publisher.get_subscription_count() < 1
            ]
            self.get_logger().error(
                "Refusing test; no subscriber on: %s" % ", ".join(missing)
            )
            return False

        def prepare_motion_owner(self):
            self.get_logger().info(
                "Publishing zero velocity and STOP before the finite test."
            )
            self.spin_for(
                PREPARE_STOP_TIME,
                lambda: (self.publish_zero(), self.publish_stop()),
            )
            self.get_logger().info(
                "Requesting MOTION ownership. This tool will not send ARM."
            )
            self.spin_for(
                MOTION_OWNER_SETTLE_TIME,
                lambda: (self.publish_owner("MOTION"), self.publish_zero()),
            )
            ready = self.wait_for_status(
                lambda _status: self.stopped_stack_ready(),
                STATUS_CONFIRMATION_TIMEOUT,
                keepalive=lambda: (
                    self.publish_owner("MOTION"),
                    self.publish_zero(),
                    self.publish_stop(),
                ),
            )
            if not ready:
                status_errors = (
                    physical_stack_status_errors(self.latest_status)
                    if self.latest_status is not None
                    else ["no fresh /volt/status"]
                )
                if not serial_stack_ready(self.serial_status_fields):
                    status_errors.append(
                        "serial bridge is neither a valid dry run nor "
                        "compatible acknowledged live ARM"
                    )
                self.get_logger().error(
                    "Refusing test; stack did not reach finite-test ready "
                    "state: %s" % "; ".join(status_errors)
                )
            return ready

        def run_cartesian_request(self):
            self.get_logger().info(
                "Starting %s request %s for %.1f seconds."
                % (plan.mode, request_id, plan.duration)
            )
            request_start = time.monotonic()
            self.publish_request("start")
            accepted = self.wait_for_status(
                lambda status: (
                    status.get("physical_test_active") is True
                    and status.get("physical_test_mode") == plan.mode
                    and status.get("physical_test_request_id") == request_id
                ),
                STATUS_CONFIRMATION_TIMEOUT,
                keepalive=lambda: (
                    self.publish_zero(),
                    self.publish_request("keepalive"),
                ),
            )
            if not accepted:
                self.get_logger().error(
                    "Controller did not acknowledge Cartesian request %s."
                    % request_id
                )
                return False
            last_keepalive = -float("inf")
            period = 1.0 / COMMAND_RATE
            while (
                rclpy.ok()
                and time.monotonic() - request_start < plan.duration
            ):
                now = time.monotonic()
                self.publish_zero()
                if now - last_keepalive >= REQUEST_KEEPALIVE_PERIOD:
                    self.publish_request("keepalive")
                    last_keepalive = now
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period)
            return rclpy.ok() and accepted

        def select_gait(self, gait_name):
            def publish_selection():
                self.publish_zero()
                self.publish_text(self.gait_publisher, gait_name)

            self.spin_for(GAIT_SELECTION_SETTLE_TIME, publish_selection)
            selected = self.wait_for_status(
                lambda status: (
                    status.get("active_gait") == gait_name
                    and status.get("requested_gait") == gait_name
                    and status.get("pending_gait") in (None, "")
                    and not physical_stack_status_errors(status)
                ),
                STATUS_CONFIRMATION_TIMEOUT,
                keepalive=publish_selection,
            )
            if not selected:
                self.get_logger().error(
                    "Controller did not confirm stopped physical gait '%s'."
                    % gait_name
                )
            return selected

        def run_zero_stride_trot(self):
            if not self.select_gait(FAST_TROT_GAIT):
                return False
            self.publish_text(self.action_publisher, "step")
            start = time.monotonic()
            last_keepalive = -float("inf")
            motion_observed = False
            period = 1.0 / COMMAND_RATE
            while rclpy.ok() and time.monotonic() - start < plan.duration:
                now = time.monotonic()
                self.publish_zero()
                if now - last_keepalive >= STEP_KEEPALIVE_PERIOD:
                    self.publish_text(
                        self.action_publisher,
                        "step_keepalive",
                    )
                    last_keepalive = now
                rclpy.spin_once(self, timeout_sec=0.0)
                if self.status_is_fresh():
                    motion_observed = motion_observed or (
                        self.latest_status.get("active_gait")
                        == FAST_TROT_GAIT
                        and self.latest_status.get("step_in_place") is True
                        and self.latest_status.get("motion_active") is True
                        and not physical_stack_status_errors(
                            self.latest_status,
                            require_stopped=False,
                        )
                    )
                time.sleep(period)
            return rclpy.ok() and motion_observed

        def run_velocity_gait(self, gait_name, forward_speed):
            if not self.select_gait(gait_name):
                return False
            start = time.monotonic()
            motion_observed = False
            period = 1.0 / COMMAND_RATE
            while rclpy.ok() and time.monotonic() - start < plan.duration:
                self.velocity_publisher.publish(
                    self._velocity_message(forward_speed)
                )
                rclpy.spin_once(self, timeout_sec=0.0)
                if self.status_is_fresh():
                    motion_observed = motion_observed or (
                        self.latest_status.get("active_gait") == gait_name
                        and self.latest_status.get("motion_active") is True
                        and not physical_stack_status_errors(
                            self.latest_status,
                            require_stopped=False,
                        )
                    )
                time.sleep(period)
            return rclpy.ok() and motion_observed

        def safe_cleanup(self):
            # Cancel first while MOTION still owns the controller. Repetition
            # makes shutdown robust to DDS discovery without extending motion.
            settled = False
            try:
                if plan.mode in CARTESIAN_TEST_MODES:
                    for _ in range(3):
                        self.publish_request("cancel")
                        self.publish_zero()
                        self.publish_stop()
                        rclpy.spin_once(self, timeout_sec=0.0)
                        time.sleep(0.03)
            except Exception as exc:
                self.get_logger().error(
                    "Could not publish physical-test cancel: %s" % exc
                )
            finally:
                # Keep MOTION ownership until fresh controller status confirms
                # the airborne pair is down and every gait/test is inactive.
                # A minimum interval prevents one stale stopped sample from
                # authorizing HOLD immediately after the cancel request.
                settle_start = time.monotonic()
                settle_deadline = settle_start + CLEANUP_STOP_TIMEOUT
                settled_samples = 0
                observed_sequence = self.status_sequence
                while rclpy.ok() and time.monotonic() < settle_deadline:
                    self.publish_zero()
                    self.publish_stop()
                    rclpy.spin_once(self, timeout_sec=0.02)
                    if self.status_sequence != observed_sequence:
                        observed_sequence = self.status_sequence
                        if (
                            time.monotonic() - settle_start
                            >= CLEANUP_STOP_SETTLE_TIME
                            and self.status_is_fresh()
                            and physical_stack_is_settled(
                                self.latest_status
                            )
                        ):
                            settled_samples += 1
                        else:
                            settled_samples = 0
                    if settled_samples >= 3:
                        break
                    time.sleep(1.0 / COMMAND_RATE)
                settled = settled_samples >= 3
                if not settled:
                    self.get_logger().error(
                        "STOP was not confirmed grounded/inactive within "
                        "%.1f seconds; forcing HOLD."
                        % CLEANUP_STOP_TIMEOUT
                    )
                for _ in range(5):
                    try:
                        self.publish_zero()
                        self.publish_stop()
                        self.publish_owner("HOLD")
                        self.publish_serial("HOLD")
                        rclpy.spin_once(self, timeout_sec=0.0)
                    except Exception as exc:
                        self.get_logger().error(
                            "Could not publish complete STOP/HOLD: %s" % exc
                        )
                    time.sleep(0.03)
            return settled

        def emergency_stop(self):
            serial_command = "DISABLE" if plan.disable_output else "DISARM"
            self.get_logger().warning(
                "Publishing emergency zero/STOP/HOLD/%s." % serial_command
            )
            start = time.monotonic()
            deadline = start + DISCOVERY_TIMEOUT
            discovered = False
            confirmed = False
            required = self.required_publishers()
            while rclpy.ok() and time.monotonic() < deadline:
                self.publish_zero()
                self.publish_stop()
                self.publish_owner("HOLD")
                self.publish_serial(serial_command)
                rclpy.spin_once(self, timeout_sec=0.02)
                discovered = all(
                    publisher.get_subscription_count() > 0
                    for publisher in required.values()
                )
                serial_disarmed = (
                    str(self.serial_status_fields.get("armed", "")) == "0"
                )
                if plan.disable_output:
                    serial_disarmed = serial_disarmed and (
                        str(
                            self.serial_status_fields.get(
                                "output_enabled",
                                "",
                            )
                        )
                        == "0"
                    )
                confirmed = (
                    discovered
                    and self.router_owner in ("HOLD", "DISABLED")
                    and serial_disarmed
                )
                if confirmed and time.monotonic() - start >= 0.75:
                    break
                time.sleep(1.0 / COMMAND_RATE)
            if not confirmed:
                self.get_logger().error(
                    "Emergency commands were sent for %.1f seconds but "
                    "router/serial acknowledgement was not confirmed."
                    % DISCOVERY_TIMEOUT
                )
            return confirmed

        def execute(self):
            if plan.mode == EMERGENCY_STOP:
                return self.emergency_stop()
            if not self.wait_for_stack():
                return False
            if not self.prepare_motion_owner():
                return False
            if plan.mode in CARTESIAN_TEST_MODES:
                return self.run_cartesian_request()
            if plan.mode == ZERO_STRIDE_TROT:
                return self.run_zero_stride_trot()
            if plan.mode == SLOW_CREEP:
                return self.run_velocity_gait(
                    SLOW_CREEP_GAIT,
                    SLOW_CREEP_SPEED,
                )
            return self.run_velocity_gait(
                FAST_TROT_GAIT,
                TROT_SPEED_TEST_SPEED,
            )

    # Retain Python's SIGINT -> KeyboardInterrupt behavior when supported so
    # the ROS context stays alive long enough to publish the finally cleanup.
    if SignalHandlerOptions is None:
        rclpy.init(args=[])
    else:
        rclpy.init(
            args=[],
            signal_handler_options=SignalHandlerOptions.NO,
        )
    node = PhysicalTestCommandNode()
    succeeded = False
    try:
        succeeded = node.execute()
    except KeyboardInterrupt:
        node.get_logger().warning("Physical test interrupted; stopping.")
    finally:
        try:
            if plan.mode != EMERGENCY_STOP:
                succeeded = node.safe_cleanup() and succeeded
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0 if succeeded else 1


def main(argv=None):
    parser = create_argument_parser()
    args = parser.parse_args(argv)
    try:
        plan = build_execution_plan(
            mode=args.mode,
            execute=args.execute,
            acknowledgement=args.acknowledge_support_stand,
            duration=args.duration,
            leg=args.leg,
            disable_output=args.disable_output,
        )
    except PhysicalTestError as exc:
        parser.error(str(exc))

    try:
        return _run_ros(plan)
    except (PhysicalTestError, RuntimeError) as exc:
        print("volt_physical_tests: %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
