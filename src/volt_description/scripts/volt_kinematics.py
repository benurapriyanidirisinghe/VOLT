#!/usr/bin/env python3

"""Closed-form kinematics and shared poses for the VOLT quadruped."""

import math


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

WALK_POSE = [
    0.049620338014839456, 0.49919486687672665, -1.081211652844041,
    -0.04962033801483968, 0.49919486687672665, -1.081211652844041,
    0.049620338014839456, 0.695626658430574, -1.081211652844041,
    -0.04962033801483968, 0.695626658430574, -1.081211652844041,
]

SIT_POSE = [
    0.548, 1.548, -2.600,
    -0.548, 1.548, -2.600,
    0.548, 1.548, -2.600,
    -0.548, 1.548, -2.600,
]

LEG_FOOT_MID_POSE = [
    0.0, 0.800, -2.600,
    0.0, 0.800, -2.600,
    0.0, 0.800, -2.600,
    0.0, 0.800, -2.600,
]

LEG_FOOT_SIT_POSE = [
    0.0, 1.548, -2.600,
    0.0, 1.548, -2.600,
    0.0, 1.548, -2.600,
    0.0, 1.548, -2.600,
]


def clamp(value, lower, upper):
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
    shoulder, leg, foot = angles
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


def inverse_leg(leg_name, target):
    """Solve one leg, projecting small out-of-workspace commands safely."""
    x, y, z = target
    side = LEG_SIDE[leg_name]
    hip_y = side * HIP_OFFSET

    yz_radius = math.hypot(y, z)
    minimum_yz = HIP_OFFSET + 1e-5
    if yz_radius < minimum_yz:
        scale = minimum_yz / max(yz_radius, 1e-9)
        y *= scale
        z *= scale
        yz_radius = minimum_yz

    z_planar = -math.sqrt(max(1e-10, yz_radius * yz_radius - HIP_OFFSET ** 2))
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
        scale = projected_reach / max(reach, 1e-9)
        down *= scale
        forward *= scale

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

    return (
        clamp(shoulder, *SHOULDER_LIMIT),
        clamp(leg, *LEG_LIMIT),
        clamp(foot, *FOOT_LIMIT),
    )


def rotation_matrix(roll, pitch, yaw):
    """Return a Z-Y-X body rotation matrix."""
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


def feet_to_joint_positions(
    feet,
    height=NOMINAL_HEIGHT,
    body_x=0.0,
    body_y=0.0,
    roll=0.0,
    pitch=0.0,
    yaw=0.0,
):
    """Convert body-frame foot positions into all 12 joint commands."""
    rotation = rotation_matrix(roll, pitch, yaw)
    body_translation = (body_x, body_y, height - NOMINAL_HEIGHT)
    positions = []

    for leg_name in LEG_ORDER:
        foot = feet[leg_name]
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
        positions.extend(inverse_leg(leg_name, shoulder_relative))

    return positions
