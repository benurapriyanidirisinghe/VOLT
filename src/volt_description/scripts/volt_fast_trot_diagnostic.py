#!/usr/bin/env python3

"""Passive live recorder for VOLT fast-trot diagnostics.

This node never publishes motion or serial commands.  Recording is armed with
``start`` and closed with ``stop`` on ``/volt/fast_trot_diagnostic``.  Rows are
written only while the motion status reports an active ``fast_trot``.
"""

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from volt_gait_controller import GAITS, TROT_PHASE_OFFSETS
from volt_kinematics import (
    FOOT_LIMIT,
    JOINT_NAMES,
    LEG_LIMIT,
    LEG_ORDER,
    KinematicsError,
    NOMINAL_FEET,
    SHOULDER_LIMIT,
    joint_positions_to_feet,
)
from volt_servo_calibration import (
    CalibrationError,
    ServoCalibrationTable,
    named_positions_from_ordered,
)


GAIT_NAME = "fast_trot"
COMMANDS = ("start", "stop")
FOOT_AXES = ("x", "y", "z")
BODY_TRANSFORM_KEYS = (
    "height",
    "body_x",
    "body_y",
    "roll",
    "pitch",
    "yaw",
)
DIAGONAL_PAIRS = (
    frozenset(("front_left", "rear_right")),
    frozenset(("front_right", "rear_left")),
)
LOOP_METRIC_KEYS = (
    "control_loop_rate_hz",
    "command_publish_rate_hz",
    "control_loop_dt_s",
    "control_loop_max_dt_s",
    "expected_control_rate_hz",
    "missed_deadlines",
)
LOOP_METRIC_ALIASES = {
    "control_loop_rate_hz": (
        "control_loop_rate_hz",
        "controller_loop_rate_hz",
        "loop_rate_hz",
        "actual_loop_rate_hz",
    ),
    "command_publish_rate_hz": (
        "command_publish_rate_hz",
        "controller_publish_rate_hz",
        "publish_rate_hz",
        "command_rate_hz",
    ),
    "control_loop_dt_s": (
        "control_loop_dt_s",
        "controller_loop_dt_s",
        "loop_dt_s",
        "actual_loop_period_s",
    ),
    "control_loop_max_dt_s": (
        "control_loop_max_dt_s",
        "controller_loop_max_dt_s",
        "max_loop_dt_s",
        "maximum_loop_period_s",
    ),
    "expected_control_rate_hz": (
        "expected_control_rate_hz",
        "configured_control_rate_hz",
        "target_loop_rate_hz",
        "control_rate_hz",
    ),
    "missed_deadlines": (
        "missed_deadlines",
        "control_loop_missed_deadlines",
        "missed_deadline_count",
        "deadline_misses",
    ),
}
JOINT_LIMITS = {
    joint_name: (
        SHOULDER_LIMIT
        if joint_name.endswith("_shoulder")
        else LEG_LIMIT
        if joint_name.endswith("_leg")
        else FOOT_LIMIT
    )
    for joint_name in JOINT_NAMES
}


def parse_serial_status(payload):
    """Parse whitespace-separated bridge status without trusting contents."""
    fields = {}
    for token in str(payload).split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key.strip()] = value.strip()
    return fields


def finite_number(value, default=float("nan")):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def finite_vector(raw, length=3):
    """Return a fixed-length finite vector, accepting XYZ dictionaries."""
    if isinstance(raw, dict) and length == 3:
        raw = [raw.get(axis) for axis in FOOT_AXES]
    if not isinstance(raw, (list, tuple)) or len(raw) != length:
        return [float("nan")] * length
    return [finite_number(value) for value in raw]


def status_vector(status, key):
    return finite_vector(status.get(key, []))


def status_per_leg_phase(status):
    """Read explicit per-leg phases or derive them from the global phase."""
    status = status if isinstance(status, dict) else {}
    explicit = status.get(
        "per_leg_phase",
        status.get("leg_phases", status.get("per_leg_phases", {})),
    )
    explicit = explicit if isinstance(explicit, dict) else {}
    global_phase = finite_number(
        status.get(
            "cycle_phase",
            status.get("gait_phase", status.get("phase")),
        )
    )
    phases = {}
    for leg_name in LEG_ORDER:
        value = finite_number(explicit.get(leg_name))
        if not math.isfinite(value) and math.isfinite(global_phase):
            value = global_phase + TROT_PHASE_OFFSETS[leg_name]
        phases[leg_name] = (
            value % 1.0 if math.isfinite(value) else float("nan")
        )
    return phases


def status_desired_feet(status):
    """Return desired gait XYZ targets with NaNs for an older missing field."""
    status = status if isinstance(status, dict) else {}
    raw = {}
    for key in (
        "desired_feet",
        "desired_foot_positions",
        "foot_targets",
        "feet",
    ):
        candidate = status.get(key)
        if isinstance(candidate, dict):
            raw = candidate
            break
    return {
        leg_name: finite_vector(raw.get(leg_name))
        for leg_name in LEG_ORDER
    }


def status_loop_metrics(status):
    """Normalize optional controller loop metrics when fields are available."""
    status = status if isinstance(status, dict) else {}
    containers = [status]
    for key in ("loop_metrics", "controller_metrics", "timing"):
        candidate = status.get(key)
        if isinstance(candidate, dict):
            containers.append(candidate)
    result = {}
    for normalized, aliases in LOOP_METRIC_ALIASES.items():
        value = float("nan")
        for container in containers:
            for alias in aliases:
                value = finite_number(container.get(alias))
                if math.isfinite(value):
                    break
            if math.isfinite(value):
                break
        result[normalized] = value
    return result


def effective_serial_frame_rate(status, bridge_rate=float("nan")):
    """Prefer bridge timing, then accept the controller's mirrored rate."""
    rate = finite_number(bridge_rate)
    if math.isfinite(rate):
        return rate
    status = status if isinstance(status, dict) else {}
    for key in (
        "arduino_frame_rate",
        "serial_frame_rate_hz",
        "serial_frame_rate",
    ):
        rate = finite_number(status.get(key))
        if math.isfinite(rate) and rate >= 0.0:
            return rate
    return float("nan")


def consecutive_deltas(current, previous, names):
    """Return current-minus-previous values without inventing a first delta."""
    current = current if isinstance(current, dict) else {}
    previous = previous if isinstance(previous, dict) else {}
    result = {}
    for name in names:
        current_value = finite_number(current.get(name))
        previous_value = finite_number(previous.get(name))
        result[name] = (
            current_value - previous_value
            if (
                math.isfinite(current_value)
                and math.isfinite(previous_value)
            )
            else float("nan")
        )
    return result


def circular_phase_distance(first, second):
    first = finite_number(first)
    second = finite_number(second)
    if not math.isfinite(first) or not math.isfinite(second):
        return float("nan")
    difference = abs((first - second) % 1.0)
    return min(difference, 1.0 - difference)


def duplicate_publisher_conflicts(status, serial_status=None):
    """Extract duplicate-publisher evidence from old and new status forms."""
    status = status if isinstance(status, dict) else {}
    serial_status = (
        serial_status if isinstance(serial_status, dict) else {}
    )
    conflicts = []
    for key in (
        "duplicate_command_topics",
        "stack_conflict_topics",
        "publisher_conflicts",
    ):
        raw = status.get(key)
        if isinstance(raw, str):
            raw = raw.split(",")
        if isinstance(raw, (list, tuple, set)):
            conflicts.extend(
                str(item).strip()
                for item in raw
                if str(item).strip() not in ("", "-")
            )
    publisher_counts = status.get("command_publisher_counts", {})
    if isinstance(publisher_counts, dict):
        for topic, count in publisher_counts.items():
            value = finite_number(count)
            if math.isfinite(value) and value > 1.0:
                conflicts.append("%s(%d)" % (topic, int(value)))

    serial_conflict = str(
        serial_status.get(
            "stack_conflict",
            status.get("stack_conflict", ""),
        )
    ).strip()
    if serial_conflict not in ("", "-", "0", "false", "False", "none"):
        conflicts.extend(
            item for item in serial_conflict.split(",") if item
        )
    stack_unique = finite_number(
        serial_status.get(
            "stack_unique",
            status.get("stack_unique"),
        )
    )
    if math.isfinite(stack_unique) and stack_unique == 0.0 and not conflicts:
        conflicts.append("unspecified command topic")

    warning = str(status.get("warning", ""))
    lowered = warning.lower()
    if (
        "duplicate" in lowered
        and ("publisher" in lowered or "stack" in lowered)
        and warning
    ):
        conflicts.append(warning)
    return tuple(dict.fromkeys(conflicts))


def check_diagonal_pairing(status, phases, phase_tolerance=0.08):
    """Warn when reported swing/stance or phase offsets are wrong."""
    status = status if isinstance(status, dict) else {}
    swing_raw = status.get("swing_legs", [])
    stance_raw = status.get("stance_legs", [])
    swing = (
        frozenset(str(leg) for leg in swing_raw)
        if isinstance(swing_raw, (list, tuple, set))
        else frozenset()
    )
    stance = (
        frozenset(str(leg) for leg in stance_raw)
        if isinstance(stance_raw, (list, tuple, set))
        else frozenset()
    )
    all_legs = frozenset(LEG_ORDER)
    reasons = []
    if swing and swing not in DIAGONAL_PAIRS:
        reasons.append("swing=%s" % "+".join(sorted(swing)))
    if swing & stance:
        reasons.append("stance/swing overlap")
    if swing and stance and (swing | stance) != all_legs:
        reasons.append("leg omitted from stance/swing")

    tolerance = max(0.0, finite_number(phase_tolerance, 0.08))
    if all(math.isfinite(finite_number(phases.get(leg))) for leg in LEG_ORDER):
        same_a = circular_phase_distance(
            phases["front_left"],
            phases["rear_right"],
        )
        same_b = circular_phase_distance(
            phases["front_right"],
            phases["rear_left"],
        )
        opposite = circular_phase_distance(
            phases["front_left"],
            phases["front_right"],
        )
        if same_a > tolerance or same_b > tolerance:
            reasons.append("diagonal legs are not phase matched")
        if abs(opposite - 0.5) > tolerance:
            reasons.append("diagonal pairs are not 180 degrees apart")
    if not reasons:
        return []
    return [(
        "diagonal_pairing",
        "Incorrect fast-trot diagonal pairing: %s."
        % "; ".join(reasons),
    )]


def check_command_jumps(
    joints,
    servos,
    previous_joints,
    previous_servos,
    joint_threshold_rad=0.18,
    servo_threshold_deg=12.0,
):
    """Check consecutive canonical-joint and final-servo discontinuities."""
    joint_deltas = consecutive_deltas(
        joints,
        previous_joints,
        JOINT_NAMES,
    )
    servo_deltas = consecutive_deltas(
        servos,
        previous_servos,
        JOINT_NAMES,
    )
    joint_limit = max(0.0, finite_number(joint_threshold_rad, 0.18))
    servo_limit = max(0.0, finite_number(servo_threshold_deg, 12.0))
    joint_jumps = [
        (name, value)
        for name, value in joint_deltas.items()
        if math.isfinite(value) and abs(value) > joint_limit
    ]
    servo_jumps = [
        (name, value)
        for name, value in servo_deltas.items()
        if math.isfinite(value) and abs(value) > servo_limit
    ]
    warnings = []
    if joint_jumps:
        name, value = max(joint_jumps, key=lambda item: abs(item[1]))
        warnings.append((
            "joint_jump",
            "Sudden canonical joint jump: %s changed %+.3f rad."
            % (name, value),
        ))
    if servo_jumps:
        name, value = max(servo_jumps, key=lambda item: abs(item[1]))
        warnings.append((
            "servo_jump",
            "Sudden mapped servo jump: %s changed %+.1f deg."
            % (name, value),
        ))
    return warnings


def check_knee_branch_flip(
    joints,
    previous_joints,
    sign_epsilon_rad=0.05,
    discontinuity_rad=0.35,
):
    """Detect a knee/foot IK sign change or knee-only discontinuity."""
    joints = joints if isinstance(joints, dict) else {}
    previous_joints = (
        previous_joints if isinstance(previous_joints, dict) else {}
    )
    epsilon = max(0.0, finite_number(sign_epsilon_rad, 0.05))
    discontinuity = max(
        0.0,
        finite_number(discontinuity_rad, 0.35),
    )
    flipped = []
    for joint_name in JOINT_NAMES:
        if not joint_name.endswith("_foot"):
            continue
        current = finite_number(joints.get(joint_name))
        previous = finite_number(previous_joints.get(joint_name))
        if not math.isfinite(current) or not math.isfinite(previous):
            continue
        sign_changed = (
            abs(current) >= epsilon
            and abs(previous) >= epsilon
            and current * previous < 0.0
        )
        if sign_changed or abs(current - previous) > discontinuity:
            flipped.append((joint_name, previous, current))
    if not flipped:
        return []
    name, previous, current = max(
        flipped,
        key=lambda item: abs(item[2] - item[1]),
    )
    return [(
        "knee_branch_flip",
        "Possible IK knee branch flip: %s changed %+.3f -> %+.3f rad."
        % (name, previous, current),
    )]


def check_joint_limits(joints, margin_rad=1e-6):
    """Check canonical joint commands against the shared physical limits."""
    joints = joints if isinstance(joints, dict) else {}
    margin = max(0.0, finite_number(margin_rad, 1e-6))
    outside = []
    for joint_name, (lower, upper) in JOINT_LIMITS.items():
        value = finite_number(joints.get(joint_name))
        if math.isfinite(value) and (
            value < lower - margin or value > upper + margin
        ):
            outside.append((joint_name, value, lower, upper))
    if not outside:
        return []
    name, value, lower, upper = outside[0]
    return [(
        "joint_limit",
        "Unsafe canonical joint command: %s=%+.3f rad outside "
        "[%+.3f, %+.3f]." % (name, value, lower, upper),
    )]


def check_swing_clearance(
    status,
    desired_feet,
    previous_feet,
    phases,
    liftoff_clearance_m=0.008,
    motion_threshold_m=0.001,
    minimum_lift_m=0.020,
    minimum_lift_ratio=0.60,
):
    """Check that swing feet lift before translating and clear the ground."""
    status = status if isinstance(status, dict) else {}
    desired_feet = desired_feet if isinstance(desired_feet, dict) else {}
    previous_feet = previous_feet if isinstance(previous_feet, dict) else {}
    swing_raw = status.get("swing_legs", [])
    swing = (
        [str(leg) for leg in swing_raw if str(leg) in LEG_ORDER]
        if isinstance(swing_raw, (list, tuple, set))
        else []
    )
    clearance_limit = max(
        0.0,
        finite_number(liftoff_clearance_m, 0.008),
    )
    motion_limit = max(
        0.0,
        finite_number(motion_threshold_m, 0.001),
    )
    early_motion = []
    for leg_name in swing:
        current = finite_vector(desired_feet.get(leg_name))
        previous = finite_vector(previous_feet.get(leg_name))
        if not all(math.isfinite(value) for value in current + previous):
            continue
        clearance = current[2] - NOMINAL_FEET[leg_name][2]
        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
        if (
            clearance < clearance_limit
            and (dx > motion_limit or abs(dy) > motion_limit)
        ):
            early_motion.append((leg_name, clearance, dx, dy))
    warnings = []
    if early_motion:
        leg, clearance, dx, dy = early_motion[0]
        warnings.append((
            "swing_before_clearance",
            "Swing foot %s translated before liftoff clearance "
            "(clearance=%.1f mm dx=%+.1f mm dy=%+.1f mm)."
            % (
                leg,
                clearance * 1000.0,
                dx * 1000.0,
                dy * 1000.0,
            ),
        ))

    requested_lift = finite_number(status.get("requested_step_height"))
    achieved_lift = finite_number(status.get("achieved_step_height"))
    completed_cycles = finite_number(status.get("fast_trot_completed_cycles"))
    minimum = max(0.0, finite_number(minimum_lift_m, 0.020))
    ratio = max(0.0, finite_number(minimum_lift_ratio, 0.60))
    required_lift = (
        max(minimum, requested_lift * ratio)
        if math.isfinite(requested_lift) and requested_lift > 0.0
        else minimum
    )
    insufficient = (
        math.isfinite(achieved_lift)
        and achieved_lift + 1e-9 < required_lift
        and math.isfinite(completed_cycles)
        and completed_cycles > 0.0
    )
    if not insufficient:
        swing_ratio = finite_number(
            status.get("swing_ratio"),
            GAITS[GAIT_NAME].get("swing_ratio", 0.40),
        )
        for leg_name in swing:
            leg_phase = finite_number(phases.get(leg_name))
            current = finite_vector(desired_feet.get(leg_name))
            if (
                not math.isfinite(leg_phase)
                or not all(math.isfinite(value) for value in current)
                or swing_ratio <= 0.0
            ):
                continue
            swing_progress = leg_phase / swing_ratio
            clearance = current[2] - NOMINAL_FEET[leg_name][2]
            if (
                0.35 <= swing_progress <= 0.65
                and clearance + 1e-9 < required_lift
            ):
                achieved_lift = clearance
                insufficient = True
                break
    if insufficient:
        warnings.append((
            "insufficient_lift",
            "Fast-trot swing lift is insufficient: %.1f mm observed, "
            "%.1f mm required."
            % (achieved_lift * 1000.0, required_lift * 1000.0),
        ))
    return warnings


def check_stance_behavior(
    status,
    desired_feet,
    previous_feet,
    reposition_threshold_m=0.006,
):
    """Check per-sample stance repositioning and reported ground error."""
    status = status if isinstance(status, dict) else {}
    desired_feet = desired_feet if isinstance(desired_feet, dict) else {}
    previous_feet = previous_feet if isinstance(previous_feet, dict) else {}
    stance_raw = status.get("stance_legs", [])
    stance = (
        [str(leg) for leg in stance_raw if str(leg) in LEG_ORDER]
        if isinstance(stance_raw, (list, tuple, set))
        else []
    )
    threshold = max(
        0.0,
        finite_number(reposition_threshold_m, 0.006),
    )
    repositioned = []
    for leg_name in stance:
        current = finite_vector(desired_feet.get(leg_name))
        previous = finite_vector(previous_feet.get(leg_name))
        if not all(math.isfinite(value) for value in current + previous):
            continue
        horizontal = math.hypot(
            current[0] - previous[0],
            current[1] - previous[1],
        )
        if horizontal > threshold:
            repositioned.append((leg_name, horizontal))
    warnings = []
    if repositioned:
        leg, distance = max(repositioned, key=lambda item: item[1])
        warnings.append((
            "stance_reposition",
            "Excessive stance-foot repositioning: %s moved %.1f mm "
            "between commands." % (leg, distance * 1000.0),
        ))

    ground_error = finite_number(status.get("stance_max_ground_error"))
    tolerance = finite_number(
        status.get("stance_ground_tolerance"),
        GAITS[GAIT_NAME].get("stance_ground_tolerance", 0.002),
    )
    if (
        math.isfinite(ground_error)
        and math.isfinite(tolerance)
        and ground_error > tolerance
    ):
        warnings.append((
            "stance_ground_error",
            "Stance ground-height error %.1f mm exceeds %.1f mm."
            % (ground_error * 1000.0, tolerance * 1000.0),
        ))
    return warnings


def check_serial_rate(frame_rate_hz, minimum_hz=20.0, maximum_hz=35.0):
    """Check the measured bridge FRAME rate against configurable bounds."""
    rate = finite_number(frame_rate_hz)
    minimum = max(0.0, finite_number(minimum_hz, 20.0))
    maximum = max(minimum, finite_number(maximum_hz, 35.0))
    if not math.isfinite(rate):
        return []
    if rate < minimum:
        return [(
            "serial_rate_low",
            "Serial FRAME rate %.1f Hz is below %.1f Hz."
            % (rate, minimum),
        )]
    if rate > maximum:
        return [(
            "serial_rate_high",
            "Serial FRAME rate %.1f Hz exceeds %.1f Hz."
            % (rate, maximum),
        )]
    return []


def check_clamps_and_ik(status, calibration_clamps=()):
    """Report current clamp/projection evidence, not stale totals."""
    status = status if isinstance(status, dict) else {}
    clamped = status.get("clamped_joints", [])
    if not isinstance(clamped, (list, tuple, set)):
        clamped = []
    clamped = sorted(
        set(str(item) for item in list(clamped) + list(calibration_clamps))
    )
    projected = status.get("projected_targets", [])
    if not isinstance(projected, (list, tuple, set)):
        projected = []
    warnings = []
    if clamped:
        warnings.append((
            "command_clamp",
            "Joint/servo commands are clamped: %s."
            % ", ".join(clamped),
        ))
    if projected:
        warnings.append((
            "ik_projection",
            "IK projected targets into the workspace: %s."
            % ", ".join(str(item) for item in projected),
        ))
    return warnings


def diagnostic_warning_checks(
    status,
    joints,
    servos,
    previous_joints,
    previous_servos,
    desired_feet,
    previous_feet,
    serial_frame_rate,
    calibration_clamps=(),
    thresholds=None,
    serial_status=None,
):
    """Run all passive fast-trot checks and return keyed warning messages."""
    thresholds = thresholds if isinstance(thresholds, dict) else {}
    phases = status_per_leg_phase(status)
    warnings = []
    warnings.extend(check_diagonal_pairing(
        status,
        phases,
        thresholds.get("diagonal_phase_tolerance", 0.08),
    ))
    warnings.extend(check_command_jumps(
        joints,
        servos,
        previous_joints,
        previous_servos,
        thresholds.get("joint_jump_threshold_rad", 0.18),
        thresholds.get("servo_jump_threshold_deg", 12.0),
    ))
    warnings.extend(check_knee_branch_flip(
        joints,
        previous_joints,
        thresholds.get("knee_sign_epsilon_rad", 0.05),
        thresholds.get("knee_discontinuity_rad", 0.35),
    ))
    warnings.extend(check_joint_limits(joints))
    warnings.extend(check_swing_clearance(
        status,
        desired_feet,
        previous_feet,
        phases,
        thresholds.get("liftoff_clearance_m", 0.008),
        thresholds.get("swing_motion_threshold_m", 0.001),
        thresholds.get("minimum_swing_lift_m", 0.020),
        thresholds.get("minimum_lift_ratio", 0.60),
    ))
    warnings.extend(check_stance_behavior(
        status,
        desired_feet,
        previous_feet,
        thresholds.get("stance_reposition_threshold_m", 0.006),
    ))
    warnings.extend(check_serial_rate(
        serial_frame_rate,
        thresholds.get("serial_rate_min_hz", 20.0),
        thresholds.get("serial_rate_max_hz", 35.0),
    ))
    warnings.extend(check_clamps_and_ik(status, calibration_clamps))
    conflicts = duplicate_publisher_conflicts(status, serial_status)
    if conflicts:
        warnings.append((
            "duplicate_publishers",
            "Duplicate servo-command publisher conflict: %s."
            % ", ".join(conflicts),
        ))
    return warnings


def throttled_warnings(warnings, now, previous_times, interval_s):
    """Pure keyed throttle: return due warnings and a copied timestamp map."""
    now = finite_number(now, 0.0)
    interval = max(0.0, finite_number(interval_s, 2.0))
    updated = dict(previous_times or {})
    due = []
    for code, message in warnings:
        last = finite_number(updated.get(code), float("-inf"))
        if now < last or now - last >= interval:
            due.append((code, message))
            updated[code] = now
    return due, updated


def should_emit_summary(now, previous_time, rate_hz):
    """Return true at the configured low rate; zero disables summaries."""
    rate = finite_number(rate_hz)
    now = finite_number(now)
    previous = finite_number(previous_time)
    if not math.isfinite(now) or not math.isfinite(rate) or rate <= 0.0:
        return False
    return (
        not math.isfinite(previous)
        or now - previous >= 1.0 / rate
    )


def _format_number(value, precision=3, suffix=""):
    value = finite_number(value)
    return (
        ("%.*f%s" % (precision, value, suffix))
        if math.isfinite(value)
        else "-"
    )


def format_terminal_summary(
    status,
    desired_feet,
    joints,
    servos,
    servo_deltas,
    serial_frame_rate,
    calibration_clamps=(),
):
    """Build one compact multi-line snapshot for a low-rate ROS log call."""
    status = status if isinstance(status, dict) else {}
    phases = status_per_leg_phase(status)
    metrics = status_loop_metrics(status)
    swing = status.get("swing_legs", [])
    stance = status.get("stance_legs", [])
    swing = swing if isinstance(swing, (list, tuple)) else []
    stance = stance if isinstance(stance, (list, tuple)) else []
    global_phase = finite_number(
        status.get("cycle_phase", status.get("gait_phase"))
    )
    lines = [
        "FAST_TROT phase=%s (%s) swing=%s stance=%s"
        % (
            _format_number(global_phase),
            status.get("phase_name", "unknown"),
            "+".join(str(leg) for leg in swing) or "none",
            "+".join(str(leg) for leg in stance) or "none",
        )
    ]
    lines.append(
        "  rates publish=%s loop=%s expected=%s dt=%s max_dt=%s "
        "missed=%s serial=%s"
        % (
            _format_number(
                metrics["command_publish_rate_hz"],
                1,
                "Hz",
            ),
            _format_number(metrics["control_loop_rate_hz"], 1, "Hz"),
            _format_number(
                metrics["expected_control_rate_hz"],
                1,
                "Hz",
            ),
            _format_number(metrics["control_loop_dt_s"] * 1000.0, 2, "ms"),
            _format_number(
                metrics["control_loop_max_dt_s"] * 1000.0,
                2,
                "ms",
            ),
            _format_number(metrics["missed_deadlines"], 0),
            _format_number(serial_frame_rate, 1, "Hz"),
        )
    )
    projected = status.get("projected_targets", [])
    projected_count = (
        len(projected)
        if isinstance(projected, (list, tuple, set))
        else 0
    )
    status_clamped = status.get("clamped_joints", [])
    status_clamped = (
        status_clamped
        if isinstance(status_clamped, (list, tuple, set))
        else []
    )
    current_clamps = len(set(status_clamped) | set(calibration_clamps))
    lines.append(
        "  clamps current=%d joint_limit=%s velocity=%s braking=%s "
        "acceleration=%s IK=%s projected_now=%d"
        % (
            current_clamps,
            status_clamp_count(
                status,
                "joint_limit_clamp_count",
                "joint_limit_clamp_counts",
                "joint_clamps",
            ),
            status_clamp_count(
                status,
                "joint_velocity_clamp_count",
                "joint_velocity_clamp_counts",
                "velocity_limit_clamps",
            ),
            status_clamp_count(
                status,
                "joint_braking_clamp_count",
                "joint_braking_clamp_counts",
                "braking_clamps",
            ),
            status_clamp_count(
                status,
                "joint_acceleration_clamp_count",
                "joint_acceleration_clamp_counts",
                "acceleration_limit_clamps",
            ),
            status_clamp_count(
                status,
                "ik_projection_count",
                "ik_projection_counts",
                "projected_target_count",
            ),
            projected_count,
        )
    )
    swing_set = set(str(leg) for leg in swing)
    for leg_name in LEG_ORDER:
        foot = finite_vector(desired_feet.get(leg_name))
        xyz = (
            "(%s,%s,%s)m"
            % tuple(_format_number(value, 3) for value in foot)
            if all(math.isfinite(value) for value in foot)
            else "-"
        )
        lines.append(
            "  %s phase=%s %s desired_xyz=%s"
            % (
                leg_name,
                _format_number(phases[leg_name]),
                "swing" if leg_name in swing_set else "stance",
                xyz,
            )
        )
    lines.append(
        "  canonical_rad "
        + " ".join(
            "%s=%s" % (
                joint_name,
                _format_number(joints.get(joint_name), 3),
            )
            for joint_name in JOINT_NAMES
        )
    )
    lines.append(
        "  mapped_servo_deg(delta) "
        + " ".join(
            "%s=%s(%s)"
            % (
                joint_name,
                _format_number(servos.get(joint_name), 1),
                _format_number(servo_deltas.get(joint_name), 1),
            )
            for joint_name in JOINT_NAMES
        )
    )
    return "\n".join(lines)


def status_body_transform(status):
    """Return a finite IK/FK body transform or a safe empty fallback."""
    raw = status.get("gait_body_transform", {})
    if not isinstance(raw, dict):
        return {}
    transform = {
        key: finite_number(raw.get(key))
        for key in BODY_TRANSFORM_KEYS
    }
    if not all(math.isfinite(value) for value in transform.values()):
        return {}
    return transform


def status_body_world(status):
    """Return the controller's finite gait-world body pose when available."""
    raw = status.get("body_world", {})
    if not isinstance(raw, dict):
        return {}
    pose = {
        key: finite_number(raw.get(key))
        for key in ("x", "y", "yaw")
    }
    if not all(math.isfinite(value) for value in pose.values()):
        return {}
    return pose


def status_clamp_count(status, total_key, per_joint_key, legacy_key):
    """Read current status totals while accepting older recorder producers."""
    total = finite_number(status.get(total_key))
    if math.isfinite(total) and total >= 0.0:
        return int(total)
    per_joint = status.get(per_joint_key)
    if isinstance(per_joint, dict):
        values = [finite_number(value) for value in per_joint.values()]
        if values and all(
            math.isfinite(value) and value >= 0.0 for value in values
        ):
            return int(sum(values))
    legacy = finite_number(status.get(legacy_key))
    return int(legacy) if math.isfinite(legacy) and legacy >= 0.0 else ""


def csv_columns():
    columns = [
        "timestamp_ros_s",
        "elapsed_s",
        "timestamp_utc",
        "active_gait",
        "motion_active",
        "cycle_phase",
        "phase_name",
        "swing_pair",
        "stance_legs",
        "requested_vx_mps",
        "requested_vy_mps",
        "requested_yaw_rps",
        "filtered_vx_mps",
        "filtered_vy_mps",
        "filtered_yaw_rps",
        "requested_stride_m",
        "achieved_stride_m",
        "signed_stride_m",
        "stride_metric_valid",
        "requested_step_height_m",
        "achieved_step_height_m",
        "configured_cycle_period_s",
        "cycle_period_s",
        "stance_grounded",
        "stance_max_ground_error_m",
        "phase_rate_scale",
        "phase_transition_hold",
        "body_world_x_m",
        "body_world_y_m",
        "body_world_yaw_rad",
        "body_pose_source",
        "foot_source",
    ]
    for leg_name in LEG_ORDER:
        columns.append("%s_phase" % leg_name)
    for leg_name in LEG_ORDER:
        for axis in FOOT_AXES:
            columns.append("%s_foot_%s_m" % (leg_name, axis))
    for joint_name in JOINT_NAMES:
        columns.append("%s_rad" % joint_name)
    for joint_name in JOINT_NAMES:
        columns.append("%s_servo_deg" % joint_name)
    for joint_name in JOINT_NAMES:
        columns.append("%s_servo_delta_deg" % joint_name)
    columns.extend([
        "control_loop_rate_hz",
        "command_publish_rate_hz",
        "control_loop_dt_s",
        "control_loop_max_dt_s",
        "expected_control_rate_hz",
        "missed_deadlines",
        "projected_targets",
        "ik_projection_count",
        "joint_clamps",
        "joint_limit_clamps",
        "velocity_limit_clamps",
        "braking_clamps",
        "acceleration_limit_clamps",
        "joint_tracking_error_rad",
        "workspace_margin_m",
        "serial_frame_rate_hz",
        "serial_sent",
        "serial_rejected",
        "serial_blocked",
        "serial_connected",
        "serial_armed",
        "serial_dry_run",
        "serial_hardware_enabled",
        "warning",
    ])
    return columns


class CycleStrideTracker:
    """Fallback signed/grounded stance tracker for legacy status producers."""

    def __init__(self):
        self.stance_active = {leg: False for leg in LEG_ORDER}
        self.start_x = {leg: None for leg in LEG_ORDER}
        self.last_x = {leg: None for leg in LEG_ORDER}
        self.max_ground_error = {leg: 0.0 for leg in LEG_ORDER}
        self.completed = {}
        self.completed_ground_error = {}
        self.completed_stride = float("nan")

    def reset(self):
        self.stance_active = {leg: False for leg in LEG_ORDER}
        self.start_x = {leg: None for leg in LEG_ORDER}
        self.last_x = {leg: None for leg in LEG_ORDER}
        self.max_ground_error = {leg: 0.0 for leg in LEG_ORDER}
        self.completed.clear()
        self.completed_ground_error.clear()
        self.completed_stride = float("nan")

    def update(
        self,
        phase,
        feet,
        stance_legs=None,
        direction=1.0,
        ground_tolerance=0.002,
    ):
        phase = finite_number(phase)
        if stance_legs is None:
            if not math.isfinite(phase):
                return self.completed_stride
            swing_ratio = GAITS[GAIT_NAME]["swing_ratio"]
            stance_legs = [
                leg
                for leg in LEG_ORDER
                if (
                    phase + TROT_PHASE_OFFSETS[leg]
                ) % 1.0 >= swing_ratio
            ]
        stance_legs = set(stance_legs)
        direction = -1.0 if float(direction) < 0.0 else 1.0
        for leg_name in LEG_ORDER:
            x_value = finite_number(feet[leg_name][0])
            z_value = finite_number(feet[leg_name][2])
            if not math.isfinite(x_value) or not math.isfinite(z_value):
                continue
            is_stance = leg_name in stance_legs
            was_stance = self.stance_active[leg_name]
            if is_stance and not was_stance:
                self.start_x[leg_name] = x_value
                self.max_ground_error[leg_name] = 0.0
            if is_stance:
                self.last_x[leg_name] = x_value
                self.max_ground_error[leg_name] = max(
                    self.max_ground_error[leg_name],
                    abs(z_value - NOMINAL_FEET[leg_name][2]),
                )
            elif was_stance:
                start_x = self.start_x[leg_name]
                last_x = self.last_x[leg_name]
                if start_x is not None and last_x is not None:
                    self.completed[leg_name] = (
                        direction * (start_x - last_x)
                    )
                    self.completed_ground_error[leg_name] = (
                        self.max_ground_error[leg_name]
                    )
                self.start_x[leg_name] = None
                self.last_x[leg_name] = None
            self.stance_active[leg_name] = is_stance

        if len(self.completed) == len(LEG_ORDER):
            grounded = max(self.completed_ground_error.values()) <= float(
                ground_tolerance
            )
            self.completed_stride = (
                max(0.0, min(self.completed.values()))
                if grounded
                else 0.0
            )
            self.completed.clear()
            self.completed_ground_error.clear()
        return self.completed_stride


class VoltFastTrotDiagnostic(Node):
    def __init__(self):
        super().__init__("volt_fast_trot_diagnostic")

        default_calibration = (
            Path(get_package_share_directory("volt_description"))
            / "config"
            / "servo_calibration.yaml"
        )
        self.declare_parameter("output_path", "")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("record_duration", 0.0)
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("calibration_file", str(default_calibration))
        self.declare_parameter("summary_rate_hz", 1.0)
        self.declare_parameter("warning_throttle_sec", 2.0)
        self.declare_parameter("joint_jump_threshold_rad", 0.18)
        self.declare_parameter("servo_jump_threshold_deg", 12.0)
        self.declare_parameter("knee_sign_epsilon_rad", 0.05)
        self.declare_parameter("knee_discontinuity_rad", 0.35)
        self.declare_parameter("diagonal_phase_tolerance", 0.08)
        self.declare_parameter("liftoff_clearance_m", 0.008)
        self.declare_parameter("swing_motion_threshold_m", 0.001)
        self.declare_parameter("minimum_swing_lift_m", 0.020)
        self.declare_parameter("minimum_lift_ratio", 0.60)
        self.declare_parameter("stance_reposition_threshold_m", 0.006)
        self.declare_parameter("serial_rate_min_hz", 20.0)
        self.declare_parameter("serial_rate_max_hz", 35.0)
        self.declare_parameter(
            "command_topic",
            "/volt/fast_trot_diagnostic",
        )

        if bool(self.get_parameter("hardware_enabled").value):
            raise ValueError(
                "The diagnostic recorder is passive; hardware_enabled "
                "must remain false"
            )
        self.output_path_parameter = str(
            self.get_parameter("output_path").value
        ).strip()
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.record_duration = max(
            0.0,
            float(self.get_parameter("record_duration").value),
        )
        calibration_file = str(
            self.get_parameter("calibration_file").value
        )
        self.calibration = ServoCalibrationTable.from_file(calibration_file)
        self.summary_rate_hz = min(
            10.0,
            max(
                0.0,
                float(self.get_parameter("summary_rate_hz").value),
            ),
        )
        self.warning_throttle_sec = max(
            0.0,
            float(self.get_parameter("warning_throttle_sec").value),
        )
        threshold_names = (
            "joint_jump_threshold_rad",
            "servo_jump_threshold_deg",
            "knee_sign_epsilon_rad",
            "knee_discontinuity_rad",
            "diagonal_phase_tolerance",
            "liftoff_clearance_m",
            "swing_motion_threshold_m",
            "minimum_swing_lift_m",
            "minimum_lift_ratio",
            "stance_reposition_threshold_m",
            "serial_rate_min_hz",
            "serial_rate_max_hz",
        )
        self.check_thresholds = {
            name: max(
                0.0,
                float(self.get_parameter(name).value),
            )
            for name in threshold_names
        }
        self.check_thresholds["serial_rate_max_hz"] = max(
            self.check_thresholds["serial_rate_min_hz"],
            self.check_thresholds["serial_rate_max_hz"],
        )

        self.status = {}
        self.serial_status = {}
        self.latest_requested_velocity = [0.0, 0.0, 0.0]
        self.recording_armed = self.auto_start
        self.csv_file = None
        self.writer = None
        self.recording_path = None
        self.recording_start_ros = None
        self.rows_written = 0
        self.invalid_arrays = 0

        self.body_world_x = 0.0
        self.body_world_y = 0.0
        self.body_world_yaw = 0.0
        self.body_pose_source = "integrated_filtered_velocity"
        self.last_status_ros = None
        self.last_phase = None
        self.last_phase_wrap_ros = None
        self.observed_cycle_period = float("nan")
        self.stride_tracker = CycleStrideTracker()

        self.previous_serial_sent = None
        self.previous_serial_ros = None
        self.serial_frame_rate = float("nan")
        self.previous_joints = {}
        self.previous_servos = {}
        self.previous_desired_feet = {}
        self.last_summary_ros = None
        self.warning_times = {}

        command_topic = str(
            self.get_parameter("command_topic").value
        )
        self.create_subscription(
            String,
            "/volt/status",
            self.status_callback,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            "/joint_command_router/output",
            self.joint_callback,
            20,
        )
        self.create_subscription(
            String,
            "/volt/serial_status",
            self.serial_status_callback,
            20,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.velocity_callback,
            20,
        )
        self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10,
        )
        self.get_logger().info(
            "Passive fast-trot recorder ready; command topic=%s "
            "auto_start=%s summary_rate=%.1f Hz"
            % (command_topic, self.auto_start, self.summary_rate_hz)
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def emit_throttled_warnings(self, warnings, now=None):
        if now is None:
            now = self.now_seconds()
        due, self.warning_times = throttled_warnings(
            warnings,
            now,
            self.warning_times,
            self.warning_throttle_sec,
        )
        if due:
            self.get_logger().warning(
                "Fast-trot diagnostic: "
                + " | ".join(warning for _code, warning in due)
            )

    def active_fast_trot(self):
        active_gait = self.status.get(
            "active_gait",
            self.status.get("gait", ""),
        )
        return (
            str(active_gait).strip() == GAIT_NAME
            and bool(
                self.status.get(
                    "motion_active",
                    self.status.get("moving", False),
                )
            )
        )

    def velocity_callback(self, message):
        values = (
            message.linear.x,
            message.linear.y,
            message.angular.z,
        )
        if all(math.isfinite(float(value)) for value in values):
            self.latest_requested_velocity = [
                float(value) for value in values
            ]

    def status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            self.get_logger().warning(
                "Ignoring malformed /volt/status JSON.",
                throttle_duration_sec=2.0,
            )
            return
        if not isinstance(status, dict):
            return

        now = self.now_seconds()
        filtered = status_vector(status, "filtered_velocity")
        controller_body_world = status_body_world(status)
        if controller_body_world:
            self.body_world_x = controller_body_world["x"]
            self.body_world_y = controller_body_world["y"]
            self.body_world_yaw = controller_body_world["yaw"]
            self.body_pose_source = "controller_gait_world"
        elif (
            self.last_status_ros is not None
            and all(math.isfinite(value) for value in filtered)
        ):
            dt = max(0.0, min(0.5, now - self.last_status_ros))
            yaw_mid = self.body_world_yaw + 0.5 * filtered[2] * dt
            self.body_world_x += (
                filtered[0] * math.cos(yaw_mid)
                - filtered[1] * math.sin(yaw_mid)
            ) * dt
            self.body_world_y += (
                filtered[0] * math.sin(yaw_mid)
                + filtered[1] * math.cos(yaw_mid)
            ) * dt
            self.body_world_yaw += filtered[2] * dt
            self.body_pose_source = "integrated_filtered_velocity"
        self.last_status_ros = now

        phase = finite_number(
            status.get("cycle_phase", status.get("gait_phase"))
        )
        if (
            math.isfinite(phase)
            and self.last_phase is not None
            and phase + 0.5 < self.last_phase
        ):
            if self.last_phase_wrap_ros is not None:
                period = now - self.last_phase_wrap_ros
                if period > 0.0:
                    self.observed_cycle_period = period
            self.last_phase_wrap_ros = now
        if math.isfinite(phase):
            self.last_phase = phase
        self.status = status
        conflicts = duplicate_publisher_conflicts(
            self.status,
            self.serial_status,
        )
        if conflicts:
            self.emit_throttled_warnings([(
                "duplicate_publishers",
                "Duplicate servo-command publisher conflict: %s."
                % ", ".join(conflicts),
            )], now)

    def serial_status_callback(self, message):
        fields = parse_serial_status(message.data)
        now = self.now_seconds()
        reported_frame_rate = finite_number(fields.get("frame_rate"))
        sent = finite_number(fields.get("sent"))
        if math.isfinite(reported_frame_rate) and reported_frame_rate >= 0.0:
            # The bridge owns a one-second FRAME-rate window; retain its
            # authoritative value rather than differentiating a throttled
            # status stream.
            self.serial_frame_rate = reported_frame_rate
        elif (
            math.isfinite(sent)
            and self.previous_serial_sent is not None
            and self.previous_serial_ros is not None
        ):
            dt = now - self.previous_serial_ros
            if dt > 1e-6 and sent >= self.previous_serial_sent:
                instantaneous = (sent - self.previous_serial_sent) / dt
                if math.isfinite(self.serial_frame_rate):
                    self.serial_frame_rate += 0.20 * (
                        instantaneous - self.serial_frame_rate
                    )
                else:
                    self.serial_frame_rate = instantaneous
        if math.isfinite(sent):
            self.previous_serial_sent = sent
            self.previous_serial_ros = now
        self.serial_status = fields
        conflicts = duplicate_publisher_conflicts(
            getattr(self, "status", {}),
            self.serial_status,
        )
        if conflicts:
            self.emit_throttled_warnings([(
                "duplicate_publishers",
                "Duplicate servo-command publisher conflict: %s."
                % ", ".join(conflicts),
            )], now)

    def command_callback(self, message):
        command = str(message.data).strip().lower()
        if command not in COMMANDS:
            self.get_logger().warning(
                "Unknown diagnostic command '%s'; expected start or stop."
                % command
            )
            return
        if command == "start":
            if self.recording_armed:
                self.get_logger().warning(
                    "Fast-trot recording is already armed."
                )
                return
            self.recording_armed = True
            self.get_logger().info(
                "Fast-trot recording armed; waiting for active fast_trot."
            )
            return
        self.stop_recording("stop command")
        self.recording_armed = False

    def unique_output_path(self):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        if self.output_path_parameter:
            requested = Path(
                self.output_path_parameter
            ).expanduser().resolve()
            if requested.suffix.lower() != ".csv":
                requested = requested / ("volt_fast_trot_%s.csv" % stamp)
        else:
            requested = (
                Path.home()
                / ".ros"
                / ("volt_fast_trot_%s.csv" % stamp)
            )
        requested.parent.mkdir(parents=True, exist_ok=True)
        if not requested.exists():
            return requested
        for index in range(1, 10000):
            candidate = requested.with_name(
                "%s_%d%s" % (requested.stem, index, requested.suffix)
            )
            if not candidate.exists():
                return candidate
        raise OSError("could not select a unique diagnostic output path")

    def open_recording(self, now):
        path = self.unique_output_path()
        self.csv_file = path.open("x", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=csv_columns(),
        )
        self.writer.writeheader()
        self.csv_file.flush()
        self.recording_path = path
        self.recording_start_ros = now
        self.rows_written = 0
        self.stride_tracker.reset()
        self.previous_joints = {}
        self.previous_servos = {}
        self.previous_desired_feet = {}
        self.last_summary_ros = None
        self.get_logger().info("Recording fast trot to %s" % path)

    def stop_recording(self, reason):
        if self.csv_file is None:
            return
        path = self.recording_path
        rows = self.rows_written
        self.csv_file.flush()
        self.csv_file.close()
        self.csv_file = None
        self.writer = None
        self.recording_path = None
        self.recording_start_ros = None
        self.get_logger().info(
            "Closed fast-trot recording (%s): %d rows in %s"
            % (reason, rows, path)
        )

    def configured_stride(self):
        status_stride = finite_number(
            self.status.get(
                "requested_stride",
                self.status.get("requested_stride_m"),
            )
        )
        if math.isfinite(status_stride):
            return status_stride
        config = GAITS.get(GAIT_NAME, {})
        base = finite_number(
            config.get(
                "step_length_x",
                config.get("stride_length"),
            )
        )
        gait_limits = self.status.get("gait_limits", {})
        fast_limits = (
            gait_limits.get(GAIT_NAME, {})
            if isinstance(gait_limits, dict)
            else {}
        )
        limit = finite_number(fast_limits.get("max_x"))
        if not math.isfinite(limit) or limit <= 0.0:
            limit = finite_number(config.get("max_x"))
        fraction = (
            min(1.0, abs(self.latest_requested_velocity[0]) / limit)
            if math.isfinite(limit) and limit > 0.0
            else 0.0
        )
        stride_scale = finite_number(
            self.status.get("stride_scale"),
            1.0,
        )
        backend_scale = finite_number(
            self.status.get("stride_backend_scale"),
            1.0,
        )
        if not math.isfinite(base):
            return float("nan")
        return base * fraction * stride_scale * backend_scale

    def cycle_period(self):
        for key in ("current_cycle_period", "cycle_period"):
            value = finite_number(self.status.get(key))
            if math.isfinite(value) and value > 0.0:
                return value
        if (
            math.isfinite(self.observed_cycle_period)
            and self.observed_cycle_period > 0.0
        ):
            return self.observed_cycle_period
        config = GAITS.get(GAIT_NAME, {})
        value = finite_number(
            config.get("hardware_cycle_period", config.get("period"))
        )
        return value

    def serial_value(self, key, default=""):
        return self.serial_status.get(key, default)

    def joint_callback(self, message):
        if not self.recording_armed or not self.active_fast_trot():
            return
        try:
            joints = list(message.data)
        except TypeError:
            joints = []
        if len(joints) != len(JOINT_NAMES) or not all(
            math.isfinite(finite_number(value))
            for value in joints
        ):
            self.invalid_arrays += 1
            self.get_logger().warning(
                "Rejected malformed router output; expected 12 finite "
                "radians.",
                throttle_duration_sec=2.0,
            )
            return
        joints = [float(value) for value in joints]
        try:
            feet = joint_positions_to_feet(
                joints,
                **status_body_transform(self.status),
            )
            _frame, details = self.calibration.channel_frame_from_positions(
                named_positions_from_ordered(joints)
            )
        except (
            CalibrationError,
            KinematicsError,
            TypeError,
            ValueError,
        ) as exc:
            self.invalid_arrays += 1
            self.get_logger().warning(
                "Rejected diagnostic sample: %s" % exc,
                throttle_duration_sec=2.0,
            )
            return

        now = self.now_seconds()
        if self.csv_file is None:
            self.open_recording(now)
        elapsed = now - self.recording_start_ros
        if self.record_duration > 0.0 and elapsed > self.record_duration:
            self.stop_recording("configured duration reached")
            self.recording_armed = False
            return

        phase = finite_number(
            self.status.get(
                "cycle_phase",
                self.status.get("gait_phase"),
            )
        )
        achieved_stride = finite_number(
            self.status.get(
                "achieved_stride",
                self.status.get("achieved_stride_m"),
            )
        )
        swing_legs = self.status.get("swing_legs", [])
        stance_legs = self.status.get("stance_legs", [])
        if not isinstance(swing_legs, (list, tuple)):
            swing_legs = []
        if not isinstance(stance_legs, (list, tuple)):
            stance_legs = []
        filtered = status_vector(self.status, "filtered_velocity")
        tracked_stride = self.stride_tracker.update(
            phase,
            feet,
            stance_legs=stance_legs,
            direction=filtered[0],
            ground_tolerance=finite_number(
                self.status.get("stance_ground_tolerance"),
                GAITS[GAIT_NAME]["stance_ground_tolerance"],
            ),
        )
        if not math.isfinite(achieved_stride):
            achieved_stride = tracked_stride

        calibrated = {
            detail["joint"]: detail["servo_deg"]
            for detail in details
        }
        named_joints = dict(zip(JOINT_NAMES, joints))
        desired_feet = status_desired_feet(self.status)
        phases = status_per_leg_phase(self.status)
        loop_metrics = status_loop_metrics(self.status)
        servo_deltas = consecutive_deltas(
            calibrated,
            self.previous_servos,
            JOINT_NAMES,
        )
        calibration_clamps = [
            detail["joint"] for detail in details if detail["clamped"]
        ]
        status_clamps = self.status.get("clamped_joints", [])
        if not isinstance(status_clamps, (list, tuple)):
            status_clamps = []
        joint_clamps = sorted(
            set(
                str(item)
                for item in list(status_clamps) + calibration_clamps
            )
        )
        projected = self.status.get("projected_targets", [])
        if not isinstance(projected, (list, tuple)):
            projected = []
        serial_frame_rate = effective_serial_frame_rate(
            self.status,
            self.serial_frame_rate,
        )
        generated_warnings = diagnostic_warning_checks(
            self.status,
            named_joints,
            calibrated,
            self.previous_joints,
            self.previous_servos,
            desired_feet,
            self.previous_desired_feet,
            serial_frame_rate,
            calibration_clamps=calibration_clamps,
            thresholds=self.check_thresholds,
            serial_status=self.serial_status,
        )
        self.emit_throttled_warnings(generated_warnings, now)
        warning_text = "; ".join(
            item
            for item in (
                str(self.status.get("warning", "")).strip(),
                *(warning for _code, warning in generated_warnings),
            )
            if item
        )

        row = {
            "timestamp_ros_s": now,
            "elapsed_s": elapsed,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "active_gait": self.status.get("active_gait", GAIT_NAME),
            "motion_active": int(self.active_fast_trot()),
            "cycle_phase": phase,
            "phase_name": self.status.get("phase_name", ""),
            "swing_pair": "+".join(str(leg) for leg in swing_legs),
            "stance_legs": "+".join(str(leg) for leg in stance_legs),
            "requested_vx_mps": self.latest_requested_velocity[0],
            "requested_vy_mps": self.latest_requested_velocity[1],
            "requested_yaw_rps": self.latest_requested_velocity[2],
            "filtered_vx_mps": filtered[0],
            "filtered_vy_mps": filtered[1],
            "filtered_yaw_rps": filtered[2],
            "requested_stride_m": self.configured_stride(),
            "achieved_stride_m": achieved_stride,
            "signed_stride_m": self.status.get("signed_stride", ""),
            "stride_metric_valid": self.status.get(
                "stride_metric_valid",
                True,
            ),
            "requested_step_height_m": self.status.get(
                "requested_step_height",
                "",
            ),
            "achieved_step_height_m": self.status.get(
                "achieved_step_height",
                "",
            ),
            "configured_cycle_period_s": self.status.get(
                "configured_cycle_period",
                "",
            ),
            "cycle_period_s": self.cycle_period(),
            "stance_grounded": self.status.get("stance_grounded", ""),
            "stance_max_ground_error_m": self.status.get(
                "stance_max_ground_error",
                "",
            ),
            "phase_rate_scale": self.status.get("phase_rate_scale", ""),
            "phase_transition_hold": self.status.get(
                "phase_transition_hold",
                "",
            ),
            "body_world_x_m": self.body_world_x,
            "body_world_y_m": self.body_world_y,
            "body_world_yaw_rad": self.body_world_yaw,
            "body_pose_source": self.body_pose_source,
            "foot_source": "forward_kinematics_from_router_output",
            "control_loop_rate_hz": loop_metrics[
                "control_loop_rate_hz"
            ],
            "command_publish_rate_hz": loop_metrics[
                "command_publish_rate_hz"
            ],
            "control_loop_dt_s": loop_metrics["control_loop_dt_s"],
            "control_loop_max_dt_s": loop_metrics[
                "control_loop_max_dt_s"
            ],
            "expected_control_rate_hz": loop_metrics[
                "expected_control_rate_hz"
            ],
            "missed_deadlines": loop_metrics["missed_deadlines"],
            "projected_targets": "+".join(str(item) for item in projected),
            "ik_projection_count": status_clamp_count(
                self.status,
                "ik_projection_count",
                "ik_projection_counts",
                "projected_target_count",
            ),
            "joint_clamps": "+".join(joint_clamps),
            "joint_limit_clamps": status_clamp_count(
                self.status,
                "joint_limit_clamp_count",
                "joint_limit_clamp_counts",
                "joint_clamps",
            ),
            "velocity_limit_clamps": status_clamp_count(
                self.status,
                "joint_velocity_clamp_count",
                "joint_velocity_clamp_counts",
                "velocity_limit_clamps",
            ),
            "braking_clamps": status_clamp_count(
                self.status,
                "joint_braking_clamp_count",
                "joint_braking_clamp_counts",
                "braking_clamps",
            ),
            "acceleration_limit_clamps": status_clamp_count(
                self.status,
                "joint_acceleration_clamp_count",
                "joint_acceleration_clamp_counts",
                "acceleration_limit_clamps",
            ),
            "joint_tracking_error_rad": self.status.get("joint_error", ""),
            "workspace_margin_m": self.status.get("workspace_margin", ""),
            "serial_frame_rate_hz": serial_frame_rate,
            "serial_sent": self.serial_value("sent"),
            "serial_rejected": self.serial_value("rejected"),
            "serial_blocked": self.serial_value("blocked"),
            "serial_connected": self.serial_value("connected"),
            "serial_armed": self.serial_value("armed"),
            "serial_dry_run": self.serial_value("dry_run"),
            "serial_hardware_enabled": self.serial_value(
                "hardware_enabled"
            ),
            "warning": warning_text,
        }
        for leg_name in LEG_ORDER:
            row["%s_phase" % leg_name] = phases[leg_name]
            for axis, value in zip(FOOT_AXES, feet[leg_name]):
                row["%s_foot_%s_m" % (leg_name, axis)] = value
        for joint_name, value in zip(JOINT_NAMES, joints):
            row["%s_rad" % joint_name] = value
            row["%s_servo_deg" % joint_name] = calibrated[joint_name]
            delta = servo_deltas[joint_name]
            row["%s_servo_delta_deg" % joint_name] = (
                delta if math.isfinite(delta) else ""
            )

        self.writer.writerow(row)
        self.rows_written += 1
        if should_emit_summary(
            now,
            self.last_summary_ros,
            self.summary_rate_hz,
        ):
            self.get_logger().info(format_terminal_summary(
                self.status,
                desired_feet,
                named_joints,
                calibrated,
                servo_deltas,
                serial_frame_rate,
                calibration_clamps=calibration_clamps,
            ))
            self.last_summary_ros = now
        self.previous_joints = dict(named_joints)
        self.previous_servos = dict(calibrated)
        self.previous_desired_feet = {
            leg_name: list(values)
            for leg_name, values in desired_feet.items()
        }
        if self.rows_written % 25 == 0:
            self.csv_file.flush()

    def destroy_node(self):
        self.stop_recording("node shutdown")
        return super().destroy_node()


def main():
    rclpy.init()
    node = VoltFastTrotDiagnostic()
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
