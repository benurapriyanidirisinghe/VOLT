#!/usr/bin/env python3

"""Closed-form kinematics and shared poses for the VOLT quadruped."""

import math


class KinematicsError(ValueError):
    """Raised when a kinematic request cannot be represented safely."""


JOINT_NAMES = [
    "front_left_shoulder",
    "front_left_leg",
    "front_left_foot",
    "front_right_shoulder",
    "front_right_leg",
    "front_right_foot",
    "rear_left_shoulder",
    "rear_left_leg",
    "rear_left_foot",
    "rear_right_shoulder",
    "rear_right_leg",
    "rear_right_foot",
]

# Body frame: +x is forward, +y is robot-left, +z is upward. Foot targets
# below the body therefore use negative z values.
LEG_ORDER = ("front_left", "front_right", "rear_left", "rear_right")

# JOINT_NAMES and LEG_ORDER are two independently written literals, but the
# whole stack silently assumes they describe the same sequence: IK builds the
# 12-value command by iterating LEG_ORDER and appending (shoulder, leg, foot),
# and the serial calibration re-labels that array with zip(JOINT_NAMES, ...).
# Editing one without the other would permute legs across the entire robot --
# turning a trot into a pace or a bound -- while every downstream check still
# passed, because a permutation is well-formed: the array is still 12 finite
# values. Nothing else in the stack can detect that, so bind them here.
_DERIVED_JOINT_NAMES = tuple(
    "%s_%s" % (leg, joint)
    for leg in LEG_ORDER
    for joint in ("shoulder", "leg", "foot")
)
if _DERIVED_JOINT_NAMES != tuple(JOINT_NAMES):
    raise ImportError(
        "LEG_ORDER and JOINT_NAMES disagree: iterating LEG_ORDER x "
        "(shoulder, leg, foot) yields %s but JOINT_NAMES is %s. One was "
        "edited without the other; this would permute legs robot-wide."
        % (list(_DERIVED_JOINT_NAMES), list(JOINT_NAMES))
    )

# Left/right mirroring is handled with LEG_SIDE and HIP_ORIGINS. The leg and
# foot joints use the same sagittal sign convention on front and rear legs;
# only the shoulder/hip roll direction mirrors across the body centerline.
LEG_SIDE = {
    "front_left": 1.0,
    "front_right": -1.0,
    "rear_left": 1.0,
    "rear_right": -1.0,
}
HIP_ORIGINS = {
    "front_left": (0.093, 0.039, 0.0),
    "front_right": (0.093, -0.039, 0.0),
    "rear_left": (-0.093, 0.039, 0.0),
    "rear_right": (-0.093, -0.039, 0.0),
}

UPPER_LEG_LENGTH = 0.1075
LOWER_LEG_LENGTH = 0.1300
HIP_OFFSET = 0.0550
NOMINAL_HEIGHT = 0.2000

SHOULDER_LIMIT = (-0.548, 0.548)
LEG_LIMIT = (-2.666, 1.548)
FOOT_LIMIT = (-2.600, 0.100)

# URDF velocity limits in rad/s, ordered to match JOINT_NAMES. These keep the
# software command filter from asking Gazebo/servos to do impossible jumps.
JOINT_VELOCITY_LIMITS = [
    2.0, 3.0, 4.5,
    2.0, 3.0, 4.5,
    2.0, 3.0, 4.5,
    2.0, 3.0, 4.5,
]

NOMINAL_FEET = {
    "front_left": (0.113, 0.104, -NOMINAL_HEIGHT),
    "front_right": (0.113, -0.104, -NOMINAL_HEIGHT),
    "rear_left": (-0.113, 0.104, -NOMINAL_HEIGHT),
    "rear_right": (-0.113, -0.104, -NOMINAL_HEIGHT),
}

# These are canonical URDF joint radians. Real-robot mounting and leveling
# corrections belong in config/servo_calibration.yaml and are applied only by
# volt_serial_bridge.py; putting hardware offsets here also tilts Gazebo.
WALK_POSE = [
    0.049620338014839456, 0.49919486687672665, -1.081211652844041,
    -0.04962033801483968, 0.49919486687672665, -1.081211652844041,
    0.049620338014839456, 0.695626658430574, -1.081211652844041,
    -0.04962033801483968, 0.695626658430574, -1.081211652844041,
]

SIT_POSE = [
    0.548, 1.548, -2.490,
    -0.548, 1.548, -2.490,
    0.548, 1.548, -2.490,
    -0.548, 1.548, -2.490,
]

LEG_FOOT_MID_POSE = [
    0.0, 0.800, -2.490,
    0.0, 0.800, -2.490,
    0.0, 0.800, -2.490,
    0.0, 0.800, -2.490,
]

LEG_FOOT_SIT_POSE = [
    0.0, 1.548, -2.490,
    0.0, 1.548, -2.490,
    0.0, 1.548, -2.490,
    0.0, 1.548, -2.490,
]


def _finite_float(value, label="value"):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise KinematicsError("%s is not numeric" % label) from exc
    if not math.isfinite(result):
        raise KinematicsError("%s is not finite" % label)
    return result


def _finite_vector(values, count, label):
    try:
        values = tuple(values)
    except TypeError as exc:
        raise KinematicsError("%s must be an iterable" % label) from exc
    if len(values) != count:
        raise KinematicsError(
            "%s must contain exactly %d values" % (label, count)
        )
    return tuple(
        _finite_float(value, "%s[%d]" % (label, index))
        for index, value in enumerate(values)
    )


def clamp(value, lower, upper):
    """Clamp one finite scalar, rejecting non-finite values instead of failing open."""
    value = _finite_float(value)
    lower = _finite_float(lower, "lower limit")
    upper = _finite_float(upper, "upper limit")
    if lower > upper:
        raise KinematicsError("lower limit is greater than upper limit")
    return max(lower, min(upper, value))


def smoothstep(value):
    value = clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def smootherstep(value):
    """Quintic blend with zero velocity and acceleration at both ends."""
    value = clamp(value, 0.0, 1.0)
    return value ** 3 * (value * (value * 6.0 - 15.0) + 10.0)


def interpolate(start, target, proportion):
    blend = smootherstep(proportion)
    return [
        first + (second - first) * blend
        for first, second in zip(start, target)
    ]


def forward_leg(leg_name, angles):
    """Return a toe position in the shoulder-joint frame."""
    if leg_name not in LEG_SIDE:
        raise KinematicsError("unknown leg '%s'" % leg_name)
    shoulder, leg, foot = _finite_vector(angles, 3, "%s angles" % leg_name)
    side = LEG_SIDE[leg_name]

    x_planar = -(
        UPPER_LEG_LENGTH * math.sin(leg)
        + LOWER_LEG_LENGTH * math.sin(leg + foot)
    )
    z_planar = -(
        UPPER_LEG_LENGTH * math.cos(leg)
        + LOWER_LEG_LENGTH * math.cos(leg + foot)
    )
    y_planar = side * HIP_OFFSET

    cos_roll = math.cos(shoulder)
    sin_roll = math.sin(shoulder)
    y = cos_roll * y_planar - sin_roll * z_planar
    z = sin_roll * y_planar + cos_roll * z_planar
    return x_planar, y, z


class LegIK:
    """Closed-form IK with workspace projection and joint limit protection."""

    def __init__(self, leg_name):
        if leg_name not in LEG_SIDE:
            raise KinematicsError("unknown leg '%s'" % leg_name)
        self.leg_name = leg_name
        self.side = LEG_SIDE[leg_name]

    def project_target_with_diagnostics(self, target):
        """Project one target into the VOLT workspace and describe every change."""
        x, y, z = _finite_vector(target, 3, "%s target" % self.leg_name)
        input_target = (x, y, z)
        hip_y = self.side * HIP_OFFSET
        reasons = []

        yz_radius = math.hypot(y, z)
        minimum_yz = HIP_OFFSET + 1e-5
        if yz_radius < minimum_yz:
            reasons.extend(("foot_target_too_close_to_hip", "workspace_projection"))
            if yz_radius < 1e-9:
                # A zero y/z direction is undefined. Choose the mechanically
                # neutral hip side and a tiny downward component deterministically.
                y = hip_y
                z = -math.sqrt(max(0.0, minimum_yz ** 2 - HIP_OFFSET ** 2))
            else:
                scale = minimum_yz / yz_radius
                y *= scale
                z *= scale
            yz_radius = minimum_yz

        z_planar = -math.sqrt(
            max(1e-10, yz_radius * yz_radius - HIP_OFFSET ** 2)
        )
        if abs(z_planar) < 0.004:
            reasons.append("near_singular_target")

        denominator = HIP_OFFSET ** 2 + z_planar ** 2
        cos_roll = (hip_y * y + z_planar * z) / denominator
        sin_roll = (-z_planar * y + hip_y * z) / denominator
        shoulder = math.atan2(sin_roll, cos_roll)

        down = -z_planar
        forward = -x
        reach = math.hypot(down, forward)
        minimum_reach = abs(LOWER_LEG_LENGTH - UPPER_LEG_LENGTH) + 0.003
        maximum_reach = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH - 0.002
        projected_reach = clamp(reach, minimum_reach, maximum_reach)
        if abs(projected_reach - reach) > 1e-9:
            reasons.append("workspace_projection")
            if reach < minimum_reach:
                reasons.append("inside_minimum_reach")
            if reach > maximum_reach:
                reasons.append("outside_maximum_reach")
            if reach < 1e-9:
                reasons.append("near_singular_target")
            scale = projected_reach / max(reach, 1e-12)
            down *= scale
            forward *= scale

        # Reconstruct a target that exactly matches the projected planar reach.
        # This keeps the diagnostic target consistent with forward kinematics.
        projected_z_planar = -down
        cos_roll = math.cos(shoulder)
        sin_roll = math.sin(shoulder)
        projected_y = cos_roll * hip_y - sin_roll * projected_z_planar
        projected_z = sin_roll * hip_y + cos_roll * projected_z_planar
        projected_target = (-forward, projected_y, projected_z)

        reasons = list(dict.fromkeys(reasons))
        return {
            "leg": self.leg_name,
            "input_target": input_target,
            "projected_target": projected_target,
            "projected": bool(reasons),
            "reasons": reasons,
            "near_singular": "near_singular_target" in reasons,
            "minimum_reach": minimum_reach,
            "maximum_reach": maximum_reach,
            "input_reach": reach,
            "projected_reach": projected_reach,
            "_shoulder": shoulder,
            "_down": down,
            "_forward": forward,
        }

    def project_target(self, target):
        """Return the legacy projection tuple used by existing callers."""
        diagnostic = self.project_target_with_diagnostics(target)
        _x, y, z = diagnostic["projected_target"]
        hip_y = self.side * HIP_OFFSET
        down = diagnostic["_down"]
        forward = diagnostic["_forward"]
        return y, z, hip_y, -down, down, forward

    def solve_with_diagnostics(self, target):
        """Solve one leg and return finite angles plus structured diagnostics."""
        diagnostic = self.project_target_with_diagnostics(target)
        shoulder = diagnostic.pop("_shoulder")
        down = diagnostic.pop("_down")
        forward = diagnostic.pop("_forward")

        cosine_foot = (
            down * down
            + forward * forward
            - UPPER_LEG_LENGTH ** 2
            - LOWER_LEG_LENGTH ** 2
        ) / (2.0 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH)
        foot = -math.acos(clamp(cosine_foot, -1.0, 1.0))
        leg = math.atan2(forward, down) - math.atan2(
            LOWER_LEG_LENGTH * math.sin(foot),
            UPPER_LEG_LENGTH + LOWER_LEG_LENGTH * math.cos(foot),
        )

        raw_angles = (shoulder, leg, foot)
        angles = (
            clamp(raw_angles[0], *SHOULDER_LIMIT),
            clamp(raw_angles[1], *LEG_LIMIT),
            clamp(raw_angles[2], *FOOT_LIMIT),
        )
        if not all(math.isfinite(value) for value in angles):
            raise KinematicsError("%s IK produced a non-finite output" % self.leg_name)

        joint_labels = ("shoulder", "leg", "foot")
        clamped_joints = [
            "%s_%s" % (self.leg_name, label)
            for label, raw, safe in zip(joint_labels, raw_angles, angles)
            if abs(raw - safe) > 1e-12
        ]
        if clamped_joints:
            diagnostic["projected"] = True
            diagnostic["reasons"] = list(diagnostic["reasons"]) + [
                "joint_limit_clamping"
            ]
        diagnostic["raw_angles"] = raw_angles
        diagnostic["angles"] = angles
        diagnostic["clamped_joints"] = clamped_joints
        return angles, diagnostic

    def solve(self, target):
        """Solve one leg and clamp hip, knee, and shoulder to safe limits."""
        return self.solve_with_diagnostics(target)[0]


def inverse_leg(leg_name, target):
    """Solve one leg, projecting out-of-workspace commands safely."""
    return LegIK(leg_name).solve(target)


def inverse_leg_diagnostic(leg_name, target):
    """Return ``(angles, diagnostic)`` without changing the legacy IK API."""
    return LegIK(leg_name).solve_with_diagnostics(target)


def rotation_matrix(roll, pitch, yaw):
    """Return a Z-Y-X body rotation matrix."""
    roll = _finite_float(roll, "body roll")
    pitch = _finite_float(pitch, "body pitch")
    yaw = _finite_float(yaw, "body yaw")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def transpose_multiply(matrix, vector):
    return tuple(
        sum(matrix[row][column] * vector[row] for row in range(3))
        for column in range(3)
    )


def matrix_multiply(matrix, vector):
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def joint_positions_to_feet(
    positions,
    height=NOMINAL_HEIGHT,
    body_x=0.0,
    body_y=0.0,
    roll=0.0,
    pitch=0.0,
    yaw=0.0,
):
    """Forward-kinematics inverse of ``feet_to_joint_positions``.

    This lets gait startup inherit the controller's current commanded/measured
    pose instead of jumping to a hard-coded nominal foot dictionary.
    """
    positions = _finite_vector(
        positions,
        len(JOINT_NAMES),
        "joint positions",
    )
    height = _finite_float(height, "body height")
    body_x = _finite_float(body_x, "body x")
    body_y = _finite_float(body_y, "body y")
    rotation = rotation_matrix(roll, pitch, yaw)
    body_translation = (body_x, body_y, height - NOMINAL_HEIGHT)
    feet = {}
    for index, leg_name in enumerate(LEG_ORDER):
        shoulder_relative = forward_leg(
            leg_name,
            positions[index * 3:index * 3 + 3],
        )
        hip = HIP_ORIGINS[leg_name]
        body_relative = tuple(
            shoulder_relative[axis] + hip[axis]
            for axis in range(3)
        )
        shifted = matrix_multiply(rotation, body_relative)
        feet[leg_name] = tuple(
            shifted[axis] + body_translation[axis]
            for axis in range(3)
        )
    return feet


def feet_to_joint_positions_diagnostic(
    feet,
    height=NOMINAL_HEIGHT,
    body_x=0.0,
    body_y=0.0,
    roll=0.0,
    pitch=0.0,
    yaw=0.0,
):
    """Convert foot targets into finite joint commands and IK diagnostics.

    Gait code may lock feet in world coordinates during stance. Before IK, those
    world footholds must be transformed back into this body/local frame so the
    solver sees the target relative to the moving body and shoulder origin.
    """
    if not isinstance(feet, dict):
        raise KinematicsError("feet must be a leg-name mapping")
    missing = [leg for leg in LEG_ORDER if leg not in feet]
    if missing:
        raise KinematicsError("feet mapping is missing: %s" % missing)
    height = _finite_float(height, "body height")
    body_x = _finite_float(body_x, "body x")
    body_y = _finite_float(body_y, "body y")
    roll = _finite_float(roll, "body roll")
    pitch = _finite_float(pitch, "body pitch")
    yaw = _finite_float(yaw, "body yaw")

    rotation = rotation_matrix(roll, pitch, yaw)
    body_translation = (body_x, body_y, height - NOMINAL_HEIGHT)
    positions = []
    leg_diagnostics = {}
    had_non_finite_input = False

    for leg_name in LEG_ORDER:
        try:
            raw_foot = tuple(feet[leg_name])
        except TypeError as exc:
            raise KinematicsError("%s foot must be an iterable" % leg_name) from exc
        if len(raw_foot) != 3:
            raise KinematicsError("%s foot must contain exactly 3 values" % leg_name)
        safe_foot = []
        invalid_input = False
        for index, value in enumerate(raw_foot):
            try:
                value = float(value)
            except (TypeError, ValueError):
                value = float("nan")
            if not math.isfinite(value):
                invalid_input = True
                value = NOMINAL_FEET[leg_name][index]
            safe_foot.append(value)
        foot = tuple(safe_foot)
        had_non_finite_input = had_non_finite_input or invalid_input
        shifted = tuple(
            foot[index] - body_translation[index]
            for index in range(3)
        )
        body_relative = transpose_multiply(rotation, shifted)
        hip = HIP_ORIGINS[leg_name]
        shoulder_relative = tuple(
            body_relative[index] - hip[index]
            for index in range(3)
        )
        angles, diagnostic = inverse_leg_diagnostic(leg_name, shoulder_relative)
        if invalid_input:
            diagnostic["projected"] = True
            diagnostic["reasons"] = list(dict.fromkeys(
                ["non_finite_input", "workspace_projection"]
                + list(diagnostic["reasons"])
            ))
            diagnostic["input_was_non_finite"] = True
        positions.extend(angles)
        leg_diagnostics[leg_name] = diagnostic

    if len(positions) != len(JOINT_NAMES) or not all(
        math.isfinite(value) for value in positions
    ):
        raise KinematicsError("whole-body IK did not produce 12 finite joints")

    projected_targets = [
        leg for leg in LEG_ORDER if leg_diagnostics[leg]["projected"]
    ]
    diagnostics = {
        "projected_targets": projected_targets,
        "legs": leg_diagnostics,
        "non_finite_input": had_non_finite_input,
        "non_finite_output": False,
    }
    return positions, diagnostics


def feet_to_joint_positions(
    feet,
    height=NOMINAL_HEIGHT,
    body_x=0.0,
    body_y=0.0,
    roll=0.0,
    pitch=0.0,
    yaw=0.0,
):
    """Compatibility wrapper returning only the canonical 12 joint radians."""
    positions, _diagnostics = feet_to_joint_positions_diagnostic(
        feet,
        height=height,
        body_x=body_x,
        body_y=body_y,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )
    return positions
