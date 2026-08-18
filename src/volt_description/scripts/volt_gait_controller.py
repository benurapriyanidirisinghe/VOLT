#!/usr/bin/env python3

"""Stateful stance and swing gait generation for the VOLT quadruped.

Coordinate convention used by the gait planner:
- Body frame x points forward, y points left, z points up.
- Foot targets are stored in the body frame; z is negative because feet are
  below the body.
- Left and right legs are mirrored only through the y coordinate. The sagittal
  leg/foot IK signs stay the same for front and rear legs, so the knees fold in
  the same mechanical direction while the hip origins mirror across the body.
"""

import math
from pathlib import Path

import yaml

from volt_kinematics import LEG_ORDER, NOMINAL_FEET, clamp, smootherstep
from volt_real_profiles import load_profiles, smoothing_alpha, validate_tuning


# Upstream-derived crawl order from mike4192/spotMicro (MIT), commit 2a34f5d:
# RR -> FR -> RL -> FL. The implementation below is a clean VOLT adaptation
# with support-triangle shifts, world-locked stance feet, smooth endpoints, and
# safe stopping rather than a source-code copy.
SPOT_WALK_LEG_ORDER = (
    "rear_right",
    "front_right",
    "rear_left",
    "front_left",
)

SPOTMICRO_VIDEO_WALK = "spotmicro_video_walk"
DIAGNOSTIC_CRAWL = "diagnostic_crawl"
REAL_SAFE_TROT = "real_safe_trot"
VIDEO_WALK_SUPPORT_BY_LEG = {
    "rear_right": "front_left",
    "front_right": "back_left",
    "rear_left": "front_right",
    "front_left": "back_right",
}

GAIT_ALIASES = {
    "walk": SPOTMICRO_VIDEO_WALK,
    "trot": "normal_trot",
    "slow_crawl": DIAGNOSTIC_CRAWL,
    "real_trot": REAL_SAFE_TROT,
    "load_safe_trot": REAL_SAFE_TROT,
}

SPOT_WALK_PARAMETER_NAMES = (
    "cycle_period",
    "support_shift_duration",
    "swing_duration",
    "step_height",
    "maximum_step_x",
    "maximum_step_y",
    "maximum_yaw_rate",
    "body_shift_x",
    "body_shift_y",
    "support_margin",
    "touchdown_lead",
    "settle_duration",
    "velocity_deadband",
    "command_acceleration",
    "velocity_filter_alpha",
    "joint_smoothing_alpha",
    "maximum_body_roll",
    "maximum_body_pitch",
    "simulation_speed_scale",
    "hardware_speed_scale",
)

VIDEO_WALK_PARAMETER_NAMES = (
    "full_cycle_duration",
    "shift_duration",
    "support_verify_duration",
    "swing_duration",
    "settle_duration",
    "step_height",
    "max_step_x",
    "max_step_y",
    "max_yaw_step",
    "forward_body_shift",
    "rearward_body_shift",
    "lateral_body_shift",
    "support_margin",
    "shift_completion_threshold",
    "gait_body_height_offset",
    "body_bob_amplitude",
    "body_roll_amplitude",
    "body_pitch_amplitude",
    "touchdown_lead",
    "command_deadband_linear",
    "command_deadband_yaw",
    "velocity_filter_alpha",
    "joint_smoothing_alpha",
    "simulation_time_scale",
    "hardware_time_scale",
    "hardware_speed_scale",
)

FAST_TROT_PARAMETER_NAMES = (
    "cycle_period",
    "simulation_cycle_period",
    "hardware_cycle_period",
    "step_length_x",
    "step_height",
    "touchdown_lead_x",
    "liftoff_trail_x",
    "max_step_length_x",
    "min_step_length_x",
    "lateral_step_y",
    "max_yaw_step",
    "stance_ratio",
    "swing_ratio",
    "body_height_offset",
    "forward_pitch_bias",
    "maximum_body_roll",
    "maximum_body_pitch",
    "body_bob",
    "body_roll",
    "velocity_filter_alpha",
    "joint_smoothing_alpha",
    "command_acceleration",
    "joint_acceleration_limit",
    "phase_tracking_error_soft_limit",
    "phase_transition_error_limit",
    "minimum_phase_rate_scale",
    "stance_ground_tolerance",
    "startup_ramp_time",
    "shutdown_ramp_time",
    "startup_stride_fraction",
    "startup_cycle_speed_fraction",
    "stride_scale",
    "simulation_stride_scale",
    "hardware_stride_scale",
    "maximum_safe_stride_scale",
    "simulation_speed_scale",
    "hardware_speed_scale",
    "simulation_max_x",
    "hardware_max_x",
    "max_y",
    "hardware_joint_velocity_limit_deg_s",
    # Explicit physical-profile names.  The legacy names above remain part of
    # the public configuration contract so existing simulation profiles and
    # tuning clients continue to load unchanged.
    "gait_frequency",
    "stride_length",
    "duty_factor",
    "body_height",
    "stance_width",
    "front_foot_offset",
    "rear_foot_offset",
    "touchdown_blend",
    "liftoff_blend",
    "trot_ready_time",
    "joint_velocity_limit",
    "shoulder_joint_velocity_limit",
    "upper_leg_joint_velocity_limit",
    "knee_joint_velocity_limit",
    "max_joint_command_delta_deg",
    "command_smoothing",
    "physical_motion_scale",
    "contact_settle_time",
)

FAST_TROT_PRESETS = ("bench", "floor_test", "wide")
FAST_TROT_PRESET_PARAMETER_NAMES = (
    "stride_scale",
    "hardware_speed_scale",
    "hardware_cycle_period",
    "step_height",
)

FAST_TROT_TUNING_BOUNDS = {
    "stride_scale": (0.50, 1.25),
    "step_height": (0.020, 0.050),
    "hardware_cycle_period": (0.50, 0.90),
    "hardware_speed_scale": (0.20, 0.75),
}


def canonical_gait_name(gait_name):
    """Resolve documented compatibility aliases to one status/config name."""
    name = str(gait_name).strip().lower()
    return GAIT_ALIASES.get(name, name)


def default_gait_config_path():
    """Find the shared installed/source YAML without assuming a workspace path."""
    source_candidate = Path(__file__).resolve().parents[1] / "config" / "gait_controller.yaml"
    if source_candidate.is_file():
        return source_candidate
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("volt_description"))
            / "config"
            / "gait_controller.yaml"
        )
    except (ImportError, LookupError):
        return source_candidate


def nominal_support_margin_limit():
    """Return the inset reachable by every nominal stance-triangle centroid."""
    clearances = []
    for swing_leg in LEG_ORDER:
        triangle = [
            NOMINAL_FEET[leg][:2]
            for leg in LEG_ORDER
            if leg != swing_leg
        ]
        centroid = (
            sum(point[0] for point in triangle) / 3.0,
            sum(point[1] for point in triangle) / 3.0,
        )
        edge_clearances = []
        for start, end in zip(triangle, triangle[1:] + triangle[:1]):
            edge_x = end[0] - start[0]
            edge_y = end[1] - start[1]
            length = math.hypot(edge_x, edge_y)
            if length <= 1e-9:
                return 0.0
            cross = edge_x * (centroid[1] - start[1]) - edge_y * (
                centroid[0] - start[0]
            )
            edge_clearances.append(abs(cross) / length)
        clearances.append(min(edge_clearances))
    return min(clearances)


def load_spot_walk_config(path=None):
    """Load and validate the authoritative stable-walk tuning from YAML."""
    config_path = Path(path or default_gait_config_path()).expanduser()
    try:
        with config_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError("cannot load gait configuration %s: %s" % (config_path, exc)) from exc

    try:
        values = raw["volt_motion_controller"]["ros__parameters"]["spot_walk"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "gait configuration is missing volt_motion_controller.ros__parameters.spot_walk"
        ) from exc
    if not isinstance(values, dict):
        raise ValueError("spot_walk configuration must be a mapping")

    missing = [name for name in SPOT_WALK_PARAMETER_NAMES if name not in values]
    if missing:
        raise ValueError("spot_walk configuration is missing: %s" % missing)
    config = {
        name: float(values[name])
        for name in SPOT_WALK_PARAMETER_NAMES
    }
    if not all(math.isfinite(value) for value in config.values()):
        raise ValueError("spot_walk configuration values must be finite")
    positive = (
        "cycle_period",
        "support_shift_duration",
        "swing_duration",
        "step_height",
        "maximum_step_x",
        "maximum_step_y",
        "maximum_yaw_rate",
        "body_shift_x",
        "body_shift_y",
        "settle_duration",
        "command_acceleration",
        "maximum_body_roll",
        "maximum_body_pitch",
    )
    if any(config[name] <= 0.0 for name in positive):
        raise ValueError("spot_walk durations, limits, and amplitudes must be positive")
    if config["support_margin"] < 0.0:
        raise ValueError("spot_walk support_margin cannot be negative")
    if not 0.0 <= config["velocity_deadband"] < 1.0:
        raise ValueError(
            "spot_walk velocity_deadband must be normalized to [0, 1)"
        )
    support_margin_limit = nominal_support_margin_limit()
    if config["support_margin"] > support_margin_limit:
        raise ValueError(
            "spot_walk support_margin %.6f exceeds the conservative nominal "
            "stance-triangle limit %.6f"
            % (config["support_margin"], support_margin_limit)
        )
    if not 0.0 <= config["touchdown_lead"] <= 1.0:
        raise ValueError("spot_walk touchdown_lead must be in [0, 1]")
    for name in (
        "velocity_filter_alpha",
        "joint_smoothing_alpha",
        "simulation_speed_scale",
        "hardware_speed_scale",
    ):
        if not 0.0 < config[name] <= 1.0:
            raise ValueError("%s must be in (0, 1]" % name)
    expected_period = len(SPOT_WALK_LEG_ORDER) * (
        config["support_shift_duration"] + config["swing_duration"]
    )
    if abs(config["cycle_period"] - expected_period) > 1e-6:
        raise ValueError(
            "spot_walk cycle_period must equal four support-shift/swing pairs"
        )

    # Compatibility keys let the existing velocity/GUI/controller machinery use
    # one validated config without duplicating limits elsewhere.
    stance_duration = config["cycle_period"] - config["swing_duration"]
    config.update({
        "type": "spot_walk",
        "period": config["cycle_period"],
        "max_x": config["maximum_step_x"] / stance_duration,
        "max_y": config["maximum_step_y"] / stance_duration,
        "max_yaw": config["maximum_yaw_rate"],
        "max_step_x": config["maximum_step_x"],
        "max_step_y": config["maximum_step_y"],
        "step_height": config["step_height"],
        "acceleration": config["command_acceleration"],
        "smoothing_factor": config["velocity_filter_alpha"],
        "joint_tracking_alpha": config["joint_smoothing_alpha"],
        "settle_time": config["settle_duration"],
        "step_in_place_activity": 0.35,
    })
    return config


def load_spotmicro_video_walk_config(path=None):
    """Load and validate the deliberate reference-video stable crawl."""
    config_path = Path(path or default_gait_config_path()).expanduser()
    try:
        with config_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(
            "cannot load gait configuration %s: %s" % (config_path, exc)
        ) from exc

    try:
        values = raw["volt_motion_controller"]["ros__parameters"][
            SPOTMICRO_VIDEO_WALK
        ]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "gait configuration is missing "
            "volt_motion_controller.ros__parameters.%s"
            % SPOTMICRO_VIDEO_WALK
        ) from exc
    if not isinstance(values, dict):
        raise ValueError("%s configuration must be a mapping" % SPOTMICRO_VIDEO_WALK)
    if values.get("type") != "stable_crawl":
        raise ValueError("%s type must be stable_crawl" % SPOTMICRO_VIDEO_WALK)

    missing = [name for name in VIDEO_WALK_PARAMETER_NAMES if name not in values]
    if missing:
        raise ValueError("%s configuration is missing: %s" % (
            SPOTMICRO_VIDEO_WALK,
            missing,
        ))
    try:
        config = {
            name: float(values[name])
            for name in VIDEO_WALK_PARAMETER_NAMES
        }
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "%s scalar parameters must be numeric" % SPOTMICRO_VIDEO_WALK
        ) from exc
    if not all(math.isfinite(value) for value in config.values()):
        raise ValueError("%s configuration values must be finite" % SPOTMICRO_VIDEO_WALK)

    try:
        leg_sequence = tuple(str(leg) for leg in values["leg_sequence"])
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "%s leg_sequence must list all four VOLT legs"
            % SPOTMICRO_VIDEO_WALK
        ) from exc
    if (
        len(leg_sequence) != len(LEG_ORDER)
        or len(set(leg_sequence)) != len(LEG_ORDER)
        or set(leg_sequence) != set(LEG_ORDER)
    ):
        raise ValueError(
            "%s leg_sequence must be a permutation of %s"
            % (SPOTMICRO_VIDEO_WALK, LEG_ORDER)
        )

    positive = (
        "full_cycle_duration",
        "shift_duration",
        "support_verify_duration",
        "swing_duration",
        "settle_duration",
        "step_height",
        "max_step_x",
        "max_step_y",
        "max_yaw_step",
        "forward_body_shift",
        "rearward_body_shift",
        "lateral_body_shift",
        "simulation_time_scale",
        "hardware_time_scale",
        "hardware_speed_scale",
    )
    if any(config[name] <= 0.0 for name in positive):
        raise ValueError(
            "%s durations, limits, shifts, and scales must be positive"
            % SPOTMICRO_VIDEO_WALK
        )
    if not 0.015 <= config["step_height"] <= 0.045:
        raise ValueError("%s step_height must be in [0.015, 0.045]" % SPOTMICRO_VIDEO_WALK)
    if not 0.0 <= config["support_margin"] <= nominal_support_margin_limit():
        raise ValueError(
            "%s support_margin does not fit the nominal stance triangles"
            % SPOTMICRO_VIDEO_WALK
        )
    if not 0.0 < config["shift_completion_threshold"] <= 1.0:
        raise ValueError(
            "%s shift_completion_threshold must be in (0, 1]"
            % SPOTMICRO_VIDEO_WALK
        )
    if not 0.0 <= config["touchdown_lead"] <= 1.0:
        raise ValueError("%s touchdown_lead must be in [0, 1]" % SPOTMICRO_VIDEO_WALK)
    if config["command_deadband_linear"] < 0.0 or config["command_deadband_yaw"] < 0.0:
        raise ValueError("%s command deadbands cannot be negative" % SPOTMICRO_VIDEO_WALK)
    for name in (
        "body_bob_amplitude",
        "body_roll_amplitude",
        "body_pitch_amplitude",
    ):
        if config[name] < 0.0:
            raise ValueError("%s cannot be negative" % name)
    for name in (
        "velocity_filter_alpha",
        "joint_smoothing_alpha",
        "hardware_speed_scale",
    ):
        if not 0.0 < config[name] <= 1.0:
            raise ValueError("%s must be in (0, 1]" % name)
    if config["hardware_speed_scale"] > 0.20 + 1e-9:
        raise ValueError(
            "%s hardware_speed_scale exceeds the 20%% first-test cap"
            % SPOTMICRO_VIDEO_WALK
        )

    per_leg_duration = (
        config["shift_duration"]
        + config["support_verify_duration"]
        + config["swing_duration"]
        + config["settle_duration"]
    )
    expected_cycle = len(leg_sequence) * per_leg_duration
    if abs(config["full_cycle_duration"] - expected_cycle) > 1e-6:
        raise ValueError(
            "%s full_cycle_duration must equal four "
            "shift/verify/swing/settle operations"
            % SPOTMICRO_VIDEO_WALK
        )

    # Express stride-sized limits as conservative Twist rates. At full command
    # one leg-operation interval produces at most one configured step. The
    # touchdown planner independently clamps every frozen foothold to the exact
    # Cartesian/yaw step limits.
    config.update({
        "type": "stable_crawl",
        "leg_sequence": leg_sequence,
        "period": config["full_cycle_duration"],
        "max_x": config["max_step_x"] / per_leg_duration,
        "max_y": config["max_step_y"] / per_leg_duration,
        "max_yaw": config["max_yaw_step"] / per_leg_duration,
        "maximum_step_x": config["max_step_x"],
        "maximum_step_y": config["max_step_y"],
        "maximum_yaw_rate": config["max_yaw_step"] / per_leg_duration,
        "velocity_deadband": 0.0,
        "smoothing_factor": config["velocity_filter_alpha"],
        "joint_tracking_alpha": config["joint_smoothing_alpha"],
        "simulation_speed_scale": 1.0,
        "step_in_place_activity": 0.35,
        "touchdown_settle_duration": config["settle_duration"],
        "stop_center_duration": config["shift_duration"],
        "maximum_body_roll": 0.08,
        "maximum_body_pitch": 0.08,
    })
    return config


def load_fast_trot_config(path=None):
    """Load the physical fast-trot trajectory and its bounded safe presets."""
    config_path = Path(path or default_gait_config_path()).expanduser()
    try:
        with config_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(
            "cannot load gait configuration %s: %s" % (config_path, exc)
        ) from exc

    try:
        values = raw["volt_motion_controller"]["ros__parameters"]["fast_trot"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "gait configuration is missing "
            "volt_motion_controller.ros__parameters.fast_trot"
        ) from exc
    if not isinstance(values, dict):
        raise ValueError("fast_trot configuration must be a mapping")
    if values.get("type") != "physical_trot":
        raise ValueError("fast_trot type must be physical_trot")
    allowed_fields = set(FAST_TROT_PARAMETER_NAMES) | {"type", "presets"}
    unknown_fields = sorted(set(values) - allowed_fields)
    if unknown_fields:
        raise ValueError(
            "fast_trot configuration has unknown fields: %s"
            % unknown_fields
        )

    missing = [
        name for name in FAST_TROT_PARAMETER_NAMES
        if name not in values
    ]
    if missing:
        raise ValueError("fast_trot configuration is missing: %s" % missing)
    try:
        config = {
            name: float(values[name])
            for name in FAST_TROT_PARAMETER_NAMES
        }
    except (TypeError, ValueError) as exc:
        raise ValueError("fast_trot scalar parameters must be numeric") from exc
    if not all(math.isfinite(value) for value in config.values()):
        raise ValueError("fast_trot configuration values must be finite")

    positive = (
        "cycle_period",
        "simulation_cycle_period",
        "hardware_cycle_period",
        "step_length_x",
        "step_height",
        "touchdown_lead_x",
        "max_step_length_x",
        "min_step_length_x",
        "max_yaw_step",
        "stance_ratio",
        "swing_ratio",
        "maximum_body_roll",
        "maximum_body_pitch",
        "velocity_filter_alpha",
        "joint_smoothing_alpha",
        "command_acceleration",
        "joint_acceleration_limit",
        "phase_tracking_error_soft_limit",
        "phase_transition_error_limit",
        "minimum_phase_rate_scale",
        "stance_ground_tolerance",
        "startup_ramp_time",
        "shutdown_ramp_time",
        "startup_stride_fraction",
        "startup_cycle_speed_fraction",
        "stride_scale",
        "simulation_stride_scale",
        "hardware_stride_scale",
        "maximum_safe_stride_scale",
        "simulation_speed_scale",
        "hardware_speed_scale",
        "simulation_max_x",
        "hardware_max_x",
        "hardware_joint_velocity_limit_deg_s",
        "gait_frequency",
        "stride_length",
        "duty_factor",
        "body_height",
        "stance_width",
        "touchdown_blend",
        "liftoff_blend",
        "trot_ready_time",
        "joint_velocity_limit",
        "shoulder_joint_velocity_limit",
        "upper_leg_joint_velocity_limit",
        "knee_joint_velocity_limit",
        "max_joint_command_delta_deg",
        "command_smoothing",
        "physical_motion_scale",
        "contact_settle_time",
    )
    if any(config[name] <= 0.0 for name in positive):
        raise ValueError("fast_trot durations, limits, and scales must be positive")
    if config["liftoff_trail_x"] >= 0.0:
        raise ValueError("fast_trot liftoff_trail_x must be negative")
    if config["lateral_step_y"] < 0.0 or config["max_y"] < 0.0:
        raise ValueError("fast_trot lateral limits cannot be negative")
    if not 0.020 <= config["step_height"] <= 0.050:
        raise ValueError("fast_trot step_height must be in [0.020, 0.050]")
    if not (
        0.0 < config["min_step_length_x"]
        <= config["step_length_x"]
        <= config["max_step_length_x"]
        <= 0.075
    ):
        raise ValueError(
            "fast_trot step lengths must satisfy "
            "0 < min <= requested <= max <= 0.075 m"
        )
    configured_sweep = (
        config["touchdown_lead_x"] - config["liftoff_trail_x"]
    )
    if abs(configured_sweep - config["step_length_x"]) > 1e-9:
        raise ValueError(
            "fast_trot touchdown lead minus liftoff trail must equal step_length_x"
        )
    if configured_sweep > config["max_step_length_x"] + 1e-9:
        raise ValueError("fast_trot lead-to-trail sweep exceeds max_step_length_x")
    if (
        not 0.0 < config["swing_ratio"] < 0.5
        or not 0.5 < config["stance_ratio"] < 1.0
        or abs(
            config["swing_ratio"] + config["stance_ratio"] - 1.0
        ) > 1e-9
    ):
        raise ValueError(
            "fast_trot swing/stance ratios must sum to one with diagonal overlap"
        )
    if not -0.030 <= config["body_height_offset"] <= 0.0:
        raise ValueError("fast_trot body_height_offset must be in [-0.030, 0]")
    if not 0.0 <= config["forward_pitch_bias"] <= config["maximum_body_pitch"]:
        raise ValueError("fast_trot forward_pitch_bias exceeds the safe pitch limit")
    if config["body_bob"] < 0.0 or config["body_roll"] < 0.0:
        raise ValueError("fast_trot body bob/roll amplitudes cannot be negative")
    if not 1.0 <= config["joint_acceleration_limit"] <= 60.0:
        raise ValueError(
            "fast_trot joint_acceleration_limit must be in [1, 60] rad/s^2"
        )
    if not (
        0.0 < config["phase_transition_error_limit"]
        < config["phase_tracking_error_soft_limit"]
        <= math.radians(20.0)
    ):
        raise ValueError(
            "fast_trot phase error limits must satisfy "
            "0 < transition < soft <= 20 degrees"
        )
    if not 0.0 < config["minimum_phase_rate_scale"] <= 1.0:
        raise ValueError(
            "fast_trot minimum_phase_rate_scale must be in (0, 1]"
        )
    if not 0.0005 <= config["stance_ground_tolerance"] <= 0.005:
        raise ValueError(
            "fast_trot stance_ground_tolerance must be in [0.0005, 0.005] m"
        )
    for name in (
        "velocity_filter_alpha",
        "joint_smoothing_alpha",
        "startup_stride_fraction",
        "startup_cycle_speed_fraction",
        "simulation_stride_scale",
        "hardware_stride_scale",
        "simulation_speed_scale",
        "hardware_speed_scale",
    ):
        if not 0.0 < config[name] <= 1.0:
            raise ValueError("%s must be in (0, 1]" % name)
    if not 0.50 <= config["stride_scale"] <= config["maximum_safe_stride_scale"]:
        raise ValueError("fast_trot stride_scale is outside the safe tuning range")
    if not 1.0 <= config["maximum_safe_stride_scale"] <= 1.25:
        raise ValueError(
            "fast_trot maximum_safe_stride_scale must be in [1.0, 1.25]"
        )
    if not 0.50 <= config["hardware_cycle_period"] <= 0.90:
        raise ValueError("fast_trot hardware_cycle_period must be in [0.50, 0.90]")
    if not 0.20 <= config["hardware_speed_scale"] <= 0.75:
        raise ValueError("fast_trot hardware_speed_scale must be in [0.20, 0.75]")
    if not 30.0 <= config["hardware_joint_velocity_limit_deg_s"] <= 120.0:
        raise ValueError(
            "fast_trot hardware joint velocity limit must be in [30, 120] deg/s"
        )
    alias_pairs = (
        ("stride_length", config["step_length_x"]),
        ("duty_factor", config["stance_ratio"]),
        ("body_height", 0.200 + config["body_height_offset"]),
        (
            "joint_velocity_limit",
            config["hardware_joint_velocity_limit_deg_s"],
        ),
        ("command_smoothing", config["joint_smoothing_alpha"]),
        ("physical_motion_scale", config["hardware_stride_scale"]),
    )
    for name, expected in alias_pairs:
        if abs(config[name] - expected) > 1e-5:
            raise ValueError(
                "fast_trot %s must match its compatibility value %.6f"
                % (name, expected)
            )
    valid_frequencies = (
        1.0 / config["simulation_cycle_period"],
        1.0 / config["hardware_cycle_period"],
    )
    if not any(
        abs(config["gait_frequency"] - expected) <= 1e-5
        for expected in valid_frequencies
    ):
        raise ValueError(
            "fast_trot gait_frequency must match the selected simulation "
            "or hardware cycle period"
        )
    if not 1.0 <= config["gait_frequency"] <= 2.5:
        raise ValueError("fast_trot gait_frequency must be in [1.0, 2.5] Hz")
    if not 0.175 <= config["body_height"] <= 0.210:
        raise ValueError("fast_trot body_height must be in [0.175, 0.210] m")
    if not 0.080 <= config["stance_width"] <= 0.135:
        raise ValueError("fast_trot stance_width must be in [0.080, 0.135] m")
    for name in ("front_foot_offset", "rear_foot_offset"):
        if not -0.025 <= config[name] <= 0.025:
            raise ValueError("%s must be in [-0.025, 0.025] m" % name)
    if (
        not 0.05 <= config["liftoff_blend"] <= 0.35
        or not 0.05 <= config["touchdown_blend"] <= 0.35
        or (
            config["liftoff_blend"] + config["touchdown_blend"]
            > 0.70
        )
    ):
        raise ValueError(
            "fast_trot lift/touchdown blends must each be in [0.05, 0.35] "
            "and sum to at most 0.70"
        )
    if not 0.5 <= config["trot_ready_time"] <= 3.0:
        raise ValueError("fast_trot trot_ready_time must be in [0.5, 3.0] s")
    if not 0.02 <= config["contact_settle_time"] <= 0.12:
        raise ValueError(
            "fast_trot contact_settle_time must be in [0.02, 0.12] s"
        )
    velocity_limits = (
        config["shoulder_joint_velocity_limit"],
        config["upper_leg_joint_velocity_limit"],
        config["knee_joint_velocity_limit"],
    )
    if any(
        not 20.0 <= value <= config["joint_velocity_limit"]
        for value in velocity_limits
    ):
        raise ValueError(
            "fast_trot per-joint velocity limits must be in [20, "
            "joint_velocity_limit] deg/s"
        )
    if not 0.10 <= config["max_joint_command_delta_deg"] <= 3.0:
        raise ValueError(
            "fast_trot max_joint_command_delta_deg must be in [0.10, 3.0]"
        )
    maximum_simulation_stride = (
        config["step_length_x"]
        * config["maximum_safe_stride_scale"]
        * config["simulation_stride_scale"]
    )
    maximum_hardware_stride = (
        config["step_length_x"]
        * config["maximum_safe_stride_scale"]
        * config["hardware_stride_scale"]
    )
    if (
        maximum_simulation_stride > config["max_step_length_x"] + 1e-9
        or maximum_hardware_stride > config["max_step_length_x"] + 1e-9
    ):
        raise ValueError(
            "fast_trot maximum scaled stride exceeds max_step_length_x"
        )
    simulation_stride_capacity = (
        config["simulation_max_x"]
        * config["stance_ratio"]
        * config["simulation_cycle_period"]
    )
    if simulation_stride_capacity + 1e-9 < maximum_simulation_stride:
        raise ValueError(
            "fast_trot simulation command envelope cannot produce the "
            "maximum safe stride"
        )

    presets_raw = values.get("presets")
    if not isinstance(presets_raw, dict):
        raise ValueError("fast_trot presets must be a mapping")
    if set(presets_raw) != set(FAST_TROT_PRESETS):
        raise ValueError(
            "fast_trot presets must be exactly %s" % (FAST_TROT_PRESETS,)
        )
    presets = {}
    for preset_name in FAST_TROT_PRESETS:
        preset_raw = presets_raw[preset_name]
        if not isinstance(preset_raw, dict):
            raise ValueError("fast_trot preset %s must be a mapping" % preset_name)
        unknown_preset = sorted(
            set(preset_raw) - set(FAST_TROT_PRESET_PARAMETER_NAMES)
        )
        if unknown_preset:
            raise ValueError(
                "fast_trot preset %s has unknown fields: %s"
                % (preset_name, unknown_preset)
            )
        missing_preset = [
            name for name in FAST_TROT_PRESET_PARAMETER_NAMES
            if name not in preset_raw
        ]
        if missing_preset:
            raise ValueError(
                "fast_trot preset %s is missing: %s"
                % (preset_name, missing_preset)
            )
        try:
            preset = {
                name: float(preset_raw[name])
                for name in FAST_TROT_PRESET_PARAMETER_NAMES
            }
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "fast_trot preset %s values must be numeric" % preset_name
            ) from exc
        if not all(math.isfinite(value) for value in preset.values()):
            raise ValueError(
                "fast_trot preset %s values must be finite" % preset_name
            )
        for name, (lower, upper) in FAST_TROT_TUNING_BOUNDS.items():
            if not lower <= preset[name] <= upper:
                raise ValueError(
                    "fast_trot preset %s %s must be in [%.3f, %.3f]"
                    % (preset_name, name, lower, upper)
                )
        requested_stride = (
            config["step_length_x"]
            * preset["stride_scale"]
            * config["hardware_stride_scale"]
        )
        stride_capacity = (
            config["hardware_max_x"]
            * preset["hardware_speed_scale"]
            * config["stance_ratio"]
            * preset["hardware_cycle_period"]
        )
        if stride_capacity + 1e-9 < requested_stride:
            raise ValueError(
                "fast_trot preset %s command envelope cannot produce its "
                "requested stride" % preset_name
            )
        presets[preset_name] = preset

    config.update({
        "type": "physical_trot",
        "presets": presets,
        "strideLength": config["step_length_x"],
        "stepHeight": config["step_height"],
        "bodyHeight": 0.200 + config["body_height_offset"],
        "gaitFrequency": 1.0 / config["simulation_cycle_period"],
        "minGaitFrequency": 1.0 / config["hardware_cycle_period"],
        "swingRatio": config["swing_ratio"],
        "stanceRatio": config["stance_ratio"],
        "smoothingAlpha": config["velocity_filter_alpha"],
        "maxSpeed": config["simulation_max_x"],
        "acceleration": config["command_acceleration"],
        "stride_length": config["step_length_x"],
        "lateral_stride_length": config["lateral_step_y"],
        "min_gait_frequency": 1.0 / config["hardware_cycle_period"],
        "period": config["simulation_cycle_period"],
        "swing_time": (
            config["swing_ratio"] * config["simulation_cycle_period"]
        ),
        "stance_time": (
            config["stance_ratio"] * config["simulation_cycle_period"]
        ),
        "max_x": config["simulation_max_x"],
        "max_y": config["max_y"],
        "max_yaw": (
            config["max_yaw_step"]
            / (
                config["stance_ratio"]
                * config["simulation_cycle_period"]
            )
        ),
        "hardware_max_yaw": (
            config["max_yaw_step"]
            / (
                config["stance_ratio"]
                * config["hardware_cycle_period"]
            )
        ),
        "max_step_x": config["max_step_length_x"],
        "max_step_y": config["lateral_step_y"],
        "max_speed": config["simulation_max_x"],
        "min_step_height": config["step_height"],
        "step_in_place_activity": 0.55,
        "body_shift_x": 0.0,
        "body_shift_y": 0.0,
        "shift_time": 0.0,
        "smoothing_factor": config["velocity_filter_alpha"],
        "body_pitch": 0.0,
        "joint_tracking_alpha": config["joint_smoothing_alpha"],
        "settle_time": config["shutdown_ramp_time"],
    })
    return config


def validate_fast_trot_tuning(config, tuning):
    """Return a complete finite runtime tuning or raise without clamping."""
    if config.get("type") != "physical_trot":
        raise ValueError("fast-trot tuning requires a physical_trot config")
    if not isinstance(tuning, dict):
        raise ValueError("fast-trot tuning must be a mapping")
    unknown = sorted(set(tuning) - set(FAST_TROT_PRESET_PARAMETER_NAMES))
    if unknown:
        raise ValueError("unknown fast-trot tuning fields: %s" % unknown)
    missing = [
        name for name in FAST_TROT_PRESET_PARAMETER_NAMES
        if name not in tuning
    ]
    if missing:
        raise ValueError("fast-trot tuning is missing: %s" % missing)
    result = {}
    for name, (lower, upper) in FAST_TROT_TUNING_BOUNDS.items():
        try:
            value = float(tuning[name])
        except (TypeError, ValueError) as exc:
            raise ValueError("%s must be numeric" % name) from exc
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(
                "%s must be finite and in [%.3f, %.3f]"
                % (name, lower, upper)
            )
        result[name] = value
    if result["stride_scale"] > config["maximum_safe_stride_scale"] + 1e-9:
        raise ValueError("stride_scale exceeds maximum_safe_stride_scale")
    requested_stride = (
        config["step_length_x"]
        * result["stride_scale"]
        * config["hardware_stride_scale"]
    )
    stride_capacity = (
        config["hardware_max_x"]
        * result["hardware_speed_scale"]
        * config["stance_ratio"]
        * result["hardware_cycle_period"]
    )
    if stride_capacity + 1e-9 < requested_stride:
        raise ValueError(
            "fast-trot tuning command envelope cannot produce its requested "
            "stride; increase cycle period or hardware speed scale, or "
            "reduce stride scale"
        )
    return result


def trot_config(
    stride_length,
    step_height,
    gait_frequency,
    swing_ratio,
    max_x,
    max_y,
    max_yaw,
    acceleration,
    smoothing_alpha=0.30,
    min_gait_frequency=None,
    min_step_height=None,
    body_bob=0.0025,
    body_roll=0.010,
    body_pitch=0.012,
    joint_tracking_alpha=0.15,
):
    """Create a world-locked diagonal-trot configuration.

    ``gait_frequency`` is the full-command cadence.  At lower speeds the
    controller blends down toward ``min_gait_frequency`` so a small joystick
    command does not make the robot stamp rapidly in place.
    """
    stance_ratio = 1.0 - swing_ratio
    max_speed = max(max_x, max_y)
    if min_gait_frequency is None:
        min_gait_frequency = gait_frequency * 0.65
    if min_step_height is None:
        min_step_height = step_height * 0.60
    return {
        "type": "phase_trot",
        "strideLength": stride_length,
        "stepHeight": step_height,
        "bodyHeight": 0.200,
        "gaitFrequency": gait_frequency,
        "minGaitFrequency": min_gait_frequency,
        "swingRatio": swing_ratio,
        "stanceRatio": stance_ratio,
        "smoothingAlpha": smoothing_alpha,
        "maxSpeed": max_speed,
        "acceleration": acceleration,
        "stride_length": stride_length,
        "lateral_stride_length": min(0.050, stride_length * 0.45),
        "step_height": step_height,
        "body_height": 0.200,
        "gait_frequency": gait_frequency,
        "min_gait_frequency": min_gait_frequency,
        "period": 1.0 / gait_frequency,
        "swing_ratio": swing_ratio,
        "stance_ratio": stance_ratio,
        "swing_time": swing_ratio / gait_frequency,
        "stance_time": stance_ratio / gait_frequency,
        "max_x": max_x,
        "max_y": max_y,
        "max_yaw": max_yaw,
        "max_step_x": stride_length,
        "max_step_y": min(0.050, stride_length * 0.45),
        "max_speed": max_speed,
        "acceleration": acceleration,
        "min_step_height": min_step_height,
        "step_in_place_activity": 0.55,
        "body_shift_x": 0.0,
        "body_shift_y": 0.0,
        "shift_time": 0.0,
        "smoothing_factor": smoothing_alpha,
        "body_bob": body_bob,
        "body_roll": body_roll,
        "body_pitch": body_pitch,
        "joint_tracking_alpha": joint_tracking_alpha,
        "settle_time": 0.35,
    }


def diagnostic_crawl_config(tuning):
    """Create the guarded one-leg crawl from one validated real profile."""
    tuning = validate_tuning(tuning, allow_simulation=False)
    if tuning["gait"] != DIAGNOSTIC_CRAWL:
        raise ValueError("diagnostic crawl profile selected a different gait")
    config = load_spotmicro_video_walk_config()
    cycle = tuning["cycle_duration"]
    swing_duration = cycle * (1.0 - tuning["duty_factor"])
    non_swing_slot = cycle / len(LEG_ORDER) - swing_duration
    if non_swing_slot <= 0.03:
        raise ValueError("diagnostic crawl cycle leaves no three-leg support shift")

    config.update({
        "full_cycle_duration": cycle,
        "period": cycle,
        "shift_duration": non_swing_slot * 0.60,
        "support_verify_duration": non_swing_slot * 0.20,
        "swing_duration": swing_duration,
        "settle_duration": non_swing_slot * 0.20,
        "touchdown_settle_duration": non_swing_slot * 0.20,
        "stop_center_duration": max(0.20, non_swing_slot * 0.60),
        "step_height": tuning["step_height"],
        "max_step_x": tuning["stride_length"],
        "max_step_y": tuning["lateral_stride_width"],
        "maximum_step_x": tuning["stride_length"],
        "maximum_step_y": tuning["lateral_stride_width"],
        "hardware_time_scale": 1.0,
        "joint_smoothing_alpha": smoothing_alpha(tuning),
        "joint_tracking_alpha": smoothing_alpha(tuning),
        "liftoff_blend": min(0.25, tuning["touchdown_softness"] * 0.75),
        "touchdown_blend": tuning["touchdown_softness"],
        "stance_width": tuning["stance_width"],
        "real_tuning": dict(tuning),
    })
    per_leg_duration = cycle / len(LEG_ORDER)
    config.update({
        "max_x": tuning["stride_length"] / per_leg_duration,
        "max_y": tuning["lateral_stride_width"] / per_leg_duration,
        "max_yaw": config["max_yaw_step"] / per_leg_duration,
        "maximum_yaw_rate": config["max_yaw_step"] / per_leg_duration,
    })
    return config


def real_safe_trot_config(tuning):
    """Create a separately named loaded-floor trot without changing sim trot."""
    tuning = validate_tuning(tuning, allow_simulation=False)
    if tuning["gait"] != REAL_SAFE_TROT:
        raise ValueError("real-safe profile selected a different gait")
    cycle = tuning["cycle_duration"]
    duty = tuning["duty_factor"]
    swing_ratio = 1.0 - duty
    forward_limit = tuning["stride_length"] / max(duty * cycle, 1e-6)
    lateral_limit = tuning["lateral_stride_width"] / max(duty * cycle, 1e-6)
    config = trot_config(
        stride_length=tuning["stride_length"],
        step_height=tuning["step_height"],
        gait_frequency=1.0 / cycle,
        min_gait_frequency=0.70 / cycle,
        swing_ratio=swing_ratio,
        max_x=forward_limit,
        max_y=lateral_limit,
        max_yaw=0.35,
        acceleration=max(0.06, forward_limit * 1.5),
        smoothing_alpha=smoothing_alpha(tuning),
        min_step_height=tuning["step_height"] * 0.75,
        body_bob=0.0005,
        body_roll=0.002,
        body_pitch=0.003,
        joint_tracking_alpha=smoothing_alpha(tuning),
    )
    config.update({
        "type": "real_safe_trot",
        "hardware_speed_scale": 1.0,
        "simulation_speed_scale": 1.0,
        "liftoff_blend": min(0.25, tuning["touchdown_softness"] * 0.75),
        "touchdown_blend": tuning["touchdown_softness"],
        "stance_width": tuning["stance_width"],
        "maximum_body_roll": 0.08,
        "maximum_body_pitch": 0.08,
        "real_tuning": dict(tuning),
    })
    return config


def apply_real_tuning_to_configs(configs, tuning):
    """Atomically replace only the selected real-hardware gait dictionary."""
    tuning = validate_tuning(tuning, allow_simulation=False)
    updated = dict(configs)
    if tuning["gait"] == DIAGNOSTIC_CRAWL:
        updated[DIAGNOSTIC_CRAWL] = diagnostic_crawl_config(tuning)
    elif tuning["gait"] == REAL_SAFE_TROT:
        updated[REAL_SAFE_TROT] = real_safe_trot_config(tuning)
    else:  # Kept fail-closed if the validator grows new gait names later.
        raise ValueError("unsupported real-hardware tuning gait")
    return updated


_REAL_DEFAULTS = load_profiles(include_user=False)


GAITS = {
    "spot_walk": load_spot_walk_config(),
    SPOTMICRO_VIDEO_WALK: load_spotmicro_video_walk_config(),
    DIAGNOSTIC_CRAWL: diagnostic_crawl_config(
        _REAL_DEFAULTS["REAL_DIAGNOSTIC"]
    ),
    REAL_SAFE_TROT: real_safe_trot_config(_REAL_DEFAULTS["REAL_SAFE"]),
    "legacy_walk": {
        "type": "legacy",
        "period": 1.20,
        "swing_time": 0.29,
        "clearance": 0.020,
        "max_x": 0.058,
        "max_y": 0.028,
        "max_yaw": 0.32,
        "max_step_x": 0.045,
        "max_step_y": 0.026,
        "body_shift_x": 0.004,
        "body_shift_y": 0.009,
        "shift_time": 0.08,
        "smoothing_factor": 0.45,
        # Static sequence: RF -> RL -> LF -> RR.
        "swing_starts": {
            "front_right": 0.04,
            "rear_left": 0.29,
            "front_left": 0.54,
            "rear_right": 0.79,
        },
        "crab_swing_starts": {
            "front_left": 0.03,
            "rear_right": 0.03,
            "front_right": 0.53,
            "rear_left": 0.53,
        },
    },
    "amble": {
        "type": "legacy",
        "period": 0.82,
        "swing_time": 0.30,
        "clearance": 0.022,
        "max_x": 0.090,
        "max_y": 0.035,
        "max_yaw": 0.50,
        "max_step_x": 0.043,
        "max_step_y": 0.026,
        "body_shift_x": 0.003,
        "body_shift_y": 0.006,
        "shift_time": 0.06,
        "smoothing_factor": 0.50,
        # Quasi-static sequence with brief adjacent swing overlap.
        "swing_starts": {
            "front_right": 0.03,
            "rear_left": 0.25,
            "front_left": 0.53,
            "rear_right": 0.75,
        },
        "crab_swing_starts": {
            "front_left": 0.03,
            "rear_right": 0.03,
            "front_right": 0.53,
            "rear_left": 0.53,
        },
    },
    # Smooth diagonal trot modes. Pair 1 is front_left + rear_right.
    # Pair 2 is front_right + rear_left. Pair 2 is always 180 deg out of phase.
    "slow_trot": trot_config(
        stride_length=0.075,
        step_height=0.011,
        gait_frequency=1.45,
        min_gait_frequency=1.15,
        swing_ratio=0.40,
        max_x=0.090,
        max_y=0.035,
        max_yaw=0.50,
        acceleration=0.30,
        smoothing_alpha=0.38,
        body_bob=0.0014,
        body_roll=0.0055,
        body_pitch=0.0065,
        joint_tracking_alpha=0.14,
    ),
    "normal_trot": trot_config(
        stride_length=0.090,
        step_height=0.0115,
        gait_frequency=1.80,
        min_gait_frequency=1.35,
        swing_ratio=0.42,
        max_x=0.130,
        max_y=0.045,
        max_yaw=0.75,
        acceleration=0.45,
        smoothing_alpha=0.46,
        body_bob=0.0018,
        body_roll=0.0065,
        body_pitch=0.0075,
        joint_tracking_alpha=0.15,
    ),
    "fast_trot": load_fast_trot_config(),
}


def load_gait_configs(path=None, fast_trot_path=None):
    """Return independent gait dictionaries with YAML-backed motion profiles.

    ``path`` owns the shared walking/simulation profiles.  Hardware launch
    files may pass ``fast_trot_path`` to overlay only the physical fast trot,
    which prevents real-servo tuning from silently changing Gazebo behavior.
    """
    configs = {
        name: dict(config)
        for name, config in GAITS.items()
        if name not in ("spot_walk", SPOTMICRO_VIDEO_WALK, "fast_trot")
    }
    configs["spot_walk"] = load_spot_walk_config(path)
    configs[SPOTMICRO_VIDEO_WALK] = load_spotmicro_video_walk_config(path)
    configs["fast_trot"] = load_fast_trot_config(fast_trot_path or path)
    return configs


TROT_PHASE_OFFSETS = {
    "front_left": 0.0,
    "rear_right": 0.0,
    "front_right": 0.5,
    "rear_left": 0.5,
}


def copy_feet(feet):
    return {leg: tuple(feet[leg]) for leg in LEG_ORDER}


def rotate_z(point, angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    x, y, z = point
    return cosine * x - sine * y, sine * x + cosine * y, z


def rotate_xy(x, y, angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return cosine * x - sine * y, sine * x + cosine * y


def periodic_elapsed(phase, start, period):
    return (phase - start) % period


def smooth_bump(value):
    """Unit-height bump with zero velocity and acceleration at both ends."""
    value = clamp(value, 0.0, 1.0)
    return 64.0 * value ** 3 * (1.0 - value) ** 3


def _ordered_triangle(points):
    center = (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )
    ordered = sorted(
        points,
        key=lambda point: math.atan2(point[1] - center[1], point[0] - center[0]),
    )
    signed_area = sum(
        start[0] * end[1] - end[0] * start[1]
        for start, end in zip(ordered, ordered[1:] + ordered[:1])
    )
    if signed_area < 0.0:
        ordered.reverse()
    return ordered


def support_clearance(point, triangle):
    """Return the minimum signed distance from a point to a CCW triangle edge."""
    triangle = _ordered_triangle(list(triangle))
    distances = []
    for start, end in zip(triangle, triangle[1:] + triangle[:1]):
        edge_x = end[0] - start[0]
        edge_y = end[1] - start[1]
        length = max(math.hypot(edge_x, edge_y), 1e-9)
        cross = edge_x * (point[1] - start[1]) - edge_y * (
            point[0] - start[0]
        )
        distances.append(cross / length)
    return min(distances)


def project_inside_support_triangle(point, triangle, margin):
    """Move a body projection minimally toward the support centroid.

    The three stance feet form a convex support triangle. A binary search toward
    its centroid gives a deterministic, inexpensive inset point without a full
    dynamics solver.
    """
    triangle = _ordered_triangle(list(triangle))
    margin = max(0.0, float(margin))
    if support_clearance(point, triangle) >= margin:
        return tuple(point)
    centroid = (
        sum(vertex[0] for vertex in triangle) / 3.0,
        sum(vertex[1] for vertex in triangle) / 3.0,
    )
    if support_clearance(centroid, triangle) < margin:
        # An excessive configured margin cannot fit. The centroid remains the
        # maximum-clearance conservative fallback.
        return centroid
    low = 0.0
    high = 1.0
    for _ in range(32):
        blend = 0.5 * (low + high)
        candidate = (
            point[0] + (centroid[0] - point[0]) * blend,
            point[1] + (centroid[1] - point[1]) * blend,
        )
        if support_clearance(candidate, triangle) >= margin:
            high = blend
        else:
            low = blend
    return (
        point[0] + (centroid[0] - point[0]) * high,
        point[1] + (centroid[1] - point[1]) * high,
    )


def normalized_velocity_activity(config, velocity):
    """Return motion demand inside the gait's x/y/yaw command ellipsoid."""
    limits = (config["max_x"], config["max_y"], config["max_yaw"])
    squared = sum(
        (component / max(limit, 1e-6)) ** 2
        for component, limit in zip(velocity, limits)
    )
    return clamp(math.sqrt(squared), 0.0, 1.0)


def limit_velocity_command(velocity, limits):
    """Clamp a combined x/y/yaw command to one normalized gait envelope.

    Per-axis clipping alone permits full forward, lateral, and yaw commands at
    the same time.  That can demand substantially more foot travel than any
    single configured maximum.  Ellipsoidal limiting keeps the direction of a
    combined command while protecting the leg workspace.
    """
    clipped = [
        clamp(float(component), -limit, limit)
        for component, limit in zip(velocity, limits)
    ]
    demand = math.sqrt(
        sum(
            (component / max(limit, 1e-6)) ** 2
            for component, limit in zip(clipped, limits)
        )
    )
    if demand > 1.0:
        clipped = [component / demand for component in clipped]
    return clipped


def clamp_elliptical_offset(offset_x, offset_y, limit_x, limit_y):
    """Clamp an XY offset to an ellipse, including a disabled zero axis."""
    values = tuple(
        float(value) for value in (offset_x, offset_y, limit_x, limit_y)
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("workspace offset and limits must be finite")
    offset_x, offset_y, limit_x, limit_y = values
    if limit_x < 0.0 or limit_y < 0.0:
        raise ValueError("workspace limits must be non-negative")
    if limit_x <= 1e-12:
        offset_x = 0.0
    if limit_y <= 1e-12:
        offset_y = 0.0
    ratio = math.hypot(
        offset_x / limit_x if limit_x > 1e-12 else 0.0,
        offset_y / limit_y if limit_y > 1e-12 else 0.0,
    )
    if ratio > 1.0:
        offset_x /= ratio
        offset_y /= ratio
    return offset_x, offset_y


def quintic_hermite(start, end, start_velocity, end_velocity, duration, phase):
    """Interpolate position with continuous endpoint velocity/acceleration."""
    phase = clamp(phase, 0.0, 1.0)
    phase2 = phase * phase
    phase3 = phase2 * phase
    phase4 = phase3 * phase
    phase5 = phase4 * phase

    start_position = 1.0 - 10.0 * phase3 + 15.0 * phase4 - 6.0 * phase5
    start_tangent = phase - 6.0 * phase3 + 8.0 * phase4 - 3.0 * phase5
    end_position = 10.0 * phase3 - 15.0 * phase4 + 6.0 * phase5
    end_tangent = -4.0 * phase3 + 7.0 * phase4 - 3.0 * phase5
    return (
        start_position * start
        + start_tangent * duration * start_velocity
        + end_position * end
        + end_tangent * duration * end_velocity
    )


class VoltGaitController:
    """Generate foot targets for crawl/amble and smooth phase-based trots."""

    def __init__(self, gait_configs=None, hardware_mode=False):
        self.gaits = dict(gait_configs or GAITS)
        # Preserve the historical pure-controller default; the ROS motion node
        # explicitly selects the canonical WALK alias at startup.
        self.gait_name = "spot_walk"
        self.hardware_mode = bool(hardware_mode)
        self.fast_trot_tuning = self.default_fast_trot_tuning()
        self.feet = copy_feet(NOMINAL_FEET)
        self.active = False
        self.settling = False
        self.stop_requested = False
        self.forced_stop = False
        self.settled_legs = set()
        self.start_time = None
        self.body_x_world = 0.0
        self.body_y_world = 0.0
        self.body_yaw_world = 0.0
        self.world_feet = copy_feet(NOMINAL_FEET)
        self.settle_start_time = None
        self.settle_origins = copy_feet(NOMINAL_FEET)
        self.crab_mode = False
        self.was_swinging = {leg: False for leg in LEG_ORDER}
        self.swing_origins = copy_feet(NOMINAL_FEET)
        self.swing_targets = copy_feet(NOMINAL_FEET)
        self.swing_heights = {
            leg: self.config.get("min_step_height", 0.0) for leg in LEG_ORDER
        }
        self.trot_phase = 0.0
        self.current_gait_frequency = self.config.get("min_gait_frequency", 0.0)
        self.fast_trot_start_time = None
        self.fast_trot_startup_scale = 0.0
        self.fast_trot_input_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_planned_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_requested_stride = 0.0
        self.fast_trot_achieved_stride = 0.0
        self.fast_trot_cycle_period = self.fast_trot_active_cycle_period()
        self.reset_fast_trot_runtime()
        self.swing_start_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.swing_end_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.spot_phase_index = 0
        self.spot_phase_name = "shift_to_support"
        self.spot_phase_start_time = None
        self.spot_phase_progress = 0.0
        self.spot_swing_leg = SPOT_WALK_LEG_ORDER[0]
        self.spot_stop_requested = False
        self.spot_forced_stop = False
        self.spot_body_shift = (0.0, 0.0)
        self.spot_shift_origin = (0.0, 0.0)
        self.spot_shift_target = (0.0, 0.0)
        self.spot_requested_body_offset = (0.0, 0.0)
        self.spot_effective_body_offset = (0.0, 0.0)
        self.spot_body_offset_origin = (0.0, 0.0)
        self.spot_body_offset_target = (0.0, 0.0)
        self.spot_support_projection = (0.0, 0.0)
        self.video_leg_slot = 0
        self.video_step_state = "STOPPED"
        self.video_state_start_time = None
        self.video_shift_completion = 0.0
        self.video_support_valid = True
        self.video_support_clearance = 0.0
        self.video_lift_allowed = False
        self.video_warning = ""
        self.video_support_feedback = {}
        self.video_center_feet_origins = copy_feet(NOMINAL_FEET)
        self.video_height_offset = 0.0
        self.video_height_origin = 0.0
        self.debug_state = {
            "phase": 0.0,
            "phase_index": 0,
            "phase_name": "shift_to_support",
            "phase_progress": 0.0,
            "swing_legs": [],
            "stance_legs": list(LEG_ORDER),
            "body_shift": {"x": 0.0, "y": 0.0},
            "effective_body_offset": {"x": 0.0, "y": 0.0},
            "support_projection": {"x": 0.0, "y": 0.0},
            "body_shift_x": 0.0,
            "body_shift_y": 0.0,
            "body_shift_target_x": 0.0,
            "body_shift_target_y": 0.0,
            "support_target_x": 0.0,
            "support_target_y": 0.0,
            "support_margin": 0.0,
            "support_clearance": 0.0,
            "shift_completion": 0.0,
            "support_polygon_valid": True,
            "lift_allowed": False,
            "step_state": "STOPPED",
            "warning": "",
            "feet": copy_feet(NOMINAL_FEET),
            "body_motion": {
                "height": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
            },
            "cadence": self.current_gait_frequency,
        }

    @property
    def config(self):
        return self.gaits[self.gait_name]

    def nominal_foot(self, leg_name):
        """Return the active profile's canonical ground target for one leg."""
        base = NOMINAL_FEET[leg_name]
        if not (
            self.hardware_mode
            and "stance_width" in self.config
        ):
            return tuple(base)
        axle_offset = 0.0
        if self.is_physical_trot():
            axle_offset = (
                self.config["front_foot_offset"]
                if leg_name.startswith("front_")
                else self.config["rear_foot_offset"]
            )
        return (
            base[0] + axle_offset,
            math.copysign(self.config["stance_width"], base[1]),
            base[2],
        )

    def nominal_feet(self):
        return {leg: tuple(self.nominal_foot(leg)) for leg in LEG_ORDER}

    def default_fast_trot_tuning(self):
        """Return the safe backend default without mutating shared gait data."""
        config = self.gaits["fast_trot"]
        if self.hardware_mode:
            return dict(config["presets"]["bench"])
        return {
            "stride_scale": config["stride_scale"],
            "step_height": config["step_height"],
            "hardware_cycle_period": config["hardware_cycle_period"],
            "hardware_speed_scale": config["hardware_speed_scale"],
        }

    def set_fast_trot_tuning(self, tuning):
        """Install a validated stopped-state tuning selected by the ROS node."""
        self.fast_trot_tuning = validate_fast_trot_tuning(
            self.gaits["fast_trot"],
            tuning,
        )
        self.fast_trot_cycle_period = self.fast_trot_active_cycle_period()
        return dict(self.fast_trot_tuning)

    def set_real_tuning(self, tuning):
        """Install one stopped-state real profile without touching sim gaits."""
        self.gaits = apply_real_tuning_to_configs(self.gaits, tuning)
        return dict(self.gaits[tuning["gait"]]["real_tuning"])

    def real_safe_governed_dt(self, dt):
        """Slow the load-safe gait clock while conditioned joints lag IK.

        Hardware has no encoder/contact feedback, so this is deliberately
        labelled command-path governance.  It prevents a new diagonal phase
        from outrunning the post-IK velocity/acceleration limiter; it does not
        claim to measure the TD-8130MG shaft position.
        """
        dt = max(0.0, float(dt))
        feedback = self.video_support_feedback
        try:
            command_error = float(feedback.get("command_error", 0.0))
        except (TypeError, ValueError):
            command_error = float("inf")
        if not math.isfinite(command_error) or command_error < 0.0:
            self.real_safe_phase_rate_scale = 0.0
            return 0.0
        soft_limit = math.radians(8.0)
        rate_scale = clamp(1.0 - command_error / soft_limit, 0.12, 1.0)
        if feedback.get("command_ready") is False and command_error > soft_limit:
            rate_scale = 0.12
        self.real_safe_phase_rate_scale = rate_scale
        return dt * rate_scale

    def reset_fast_trot_runtime(self):
        """Clear phase-governor and timing state without changing tuning."""
        self.fast_trot_startup_elapsed = 0.0
        self.fast_trot_phase_rate_scale = 1.0
        self.fast_trot_transition_hold_phase = None
        self.fast_trot_transition_dwell_elapsed = 0.0
        self.fast_trot_observed_cycle_period = 0.0
        self.fast_trot_last_cycle_wall_time = None
        self.fast_trot_last_observed_phase = None
        self.fast_trot_shutdown_start_time = None
        self.fast_trot_shutdown_origin_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_shutdown_origin_input_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_shutdown_origin_stride = 0.0
        self.fast_trot_shutdown_origin_height = 0.0
        self.fast_trot_settle_origin_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_settle_origin_input_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_ready_start_time = None
        self.fast_trot_ready_shutdown_start_time = None
        self.fast_trot_ready_shutdown_scale = 0.0

    def plan_fast_trot_shutdown(self, now):
        """Finish an active swing while smoothly reducing body propulsion."""
        if self.fast_trot_shutdown_start_time is None:
            self.fast_trot_shutdown_start_time = now
            self.fast_trot_shutdown_origin_velocity = tuple(
                self.fast_trot_planned_velocity
            )
            self.fast_trot_shutdown_origin_input_velocity = tuple(
                self.fast_trot_input_velocity
            )
            self.fast_trot_shutdown_origin_stride = float(
                self.fast_trot_requested_stride
            )
            self.fast_trot_shutdown_origin_height = float(
                getattr(self, "fast_trot_step_height", 0.0)
            )
        progress = clamp(
            (now - self.fast_trot_shutdown_start_time)
            / self.config["shutdown_ramp_time"],
            0.0,
            1.0,
        )
        scale = 1.0 - smootherstep(progress)
        self.fast_trot_planned_velocity = tuple(
            value * scale
            for value in self.fast_trot_shutdown_origin_velocity
        )
        self.fast_trot_input_velocity = tuple(
            value * scale
            for value in self.fast_trot_shutdown_origin_input_velocity
        )
        self.fast_trot_requested_stride = (
            self.fast_trot_shutdown_origin_stride * scale
        )
        self.fast_trot_achieved_stride = self.fast_trot_requested_stride
        self.fast_trot_step_height = (
            self.fast_trot_shutdown_origin_height * scale
        )
        return self.fast_trot_planned_velocity

    def fast_trot_feedback_values(self):
        """Return finite command lag and transition readiness, if available."""
        feedback = self.video_support_feedback
        if not feedback:
            return None, True
        try:
            command_error = float(feedback.get("command_error"))
        except (TypeError, ValueError):
            return float("inf"), False
        if not math.isfinite(command_error) or command_error < 0.0:
            return float("inf"), False
        transition_ready = (
            command_error
            <= self.config["phase_transition_error_limit"]
        )
        tracking_required = feedback.get("tracking_required") is True
        tracking_available = feedback.get("tracking_available") is True
        tracking_ready = feedback.get("tracking_ready") is True
        if (
            tracking_required
            and (not tracking_available or not tracking_ready)
        ):
            transition_ready = False
        elif tracking_available and not tracking_ready:
            transition_ready = False
        return command_error, transition_ready

    def fast_trot_governed_dt(self, dt):
        """Retain the Cartesian path while slowing it to downstream capacity.

        The soft governor reduces the complete gait clock, including body
        integration. Contact transitions receive a stricter endpoint gate so
        a rate-limited foot cannot enter scheduled stance while still airborne
        or leave stance before reaching its rearward trail.
        """
        dt = max(0.0, float(dt))
        command_error, transition_ready = self.fast_trot_feedback_values()
        if command_error is None:
            self.fast_trot_phase_rate_scale = 1.0
            return dt

        config = self.config
        rate_scale = clamp(
            1.0
            - command_error
            / config["phase_tracking_error_soft_limit"],
            config["minimum_phase_rate_scale"],
            1.0,
        )
        governed_dt = dt * rate_scale
        released_transition = False
        if self.fast_trot_transition_hold_phase is not None:
            if transition_ready:
                self.fast_trot_transition_dwell_elapsed += dt
                if (
                    self.fast_trot_transition_dwell_elapsed
                    >= config["contact_settle_time"]
                ):
                    self.fast_trot_transition_hold_phase = None
                    self.fast_trot_transition_dwell_elapsed = 0.0
                    released_transition = True
                else:
                    self.fast_trot_phase_rate_scale = 0.0
                    return 0.0
            else:
                self.fast_trot_transition_dwell_elapsed = 0.0
                self.fast_trot_phase_rate_scale = 0.0
                return 0.0

        if not released_transition and governed_dt > 0.0:
            phase = self.trot_phase % 1.0
            boundaries = (
                config["swing_ratio"],
                0.5,
                0.5 + config["swing_ratio"],
                1.0,
            )
            epsilon = 1e-7
            boundary = next(
                (
                    value
                    for value in boundaries
                    if value > phase + epsilon
                ),
                1.0,
            )
            distance = boundary - phase
            period = max(self.fast_trot_cycle_period, 1e-9)
            if governed_dt / period >= distance:
                governed_dt = max(
                    0.0,
                    (distance - epsilon) * period,
                )
                self.fast_trot_transition_hold_phase = boundary
                self.fast_trot_transition_dwell_elapsed = 0.0

        self.fast_trot_phase_rate_scale = (
            governed_dt / dt if dt > 1e-12 else 0.0
        )
        return governed_dt

    def record_fast_trot_cycle_timing(self, now, phase):
        """Measure the actual wall-clock cycle after feedback retiming."""
        previous = self.fast_trot_last_observed_phase
        if (
            previous is not None
            and phase < previous
            and previous > 0.5
        ):
            if self.fast_trot_last_cycle_wall_time is not None:
                period = now - self.fast_trot_last_cycle_wall_time
                if math.isfinite(period) and period > 0.0:
                    self.fast_trot_observed_cycle_period = period
            self.fast_trot_last_cycle_wall_time = now
        self.fast_trot_last_observed_phase = phase

    def fast_trot_active_cycle_period(self):
        config = self.gaits["fast_trot"]
        if self.hardware_mode:
            tuning = getattr(
                self,
                "fast_trot_tuning",
                config["presets"]["bench"],
            )
            return float(tuning["hardware_cycle_period"])
        return float(config["simulation_cycle_period"])

    def fast_trot_backend_stride_scale(self):
        config = self.gaits["fast_trot"]
        if self.hardware_mode:
            return float(config["physical_motion_scale"])
        key = (
            "simulation_stride_scale"
        )
        return float(config[key])

    def fast_trot_speed_scale(self):
        config = self.gaits["fast_trot"]
        if self.hardware_mode:
            tuning = getattr(
                self,
                "fast_trot_tuning",
                config["presets"]["bench"],
            )
            return float(tuning["hardware_speed_scale"])
        return float(config["simulation_speed_scale"])

    def fast_trot_command_limits(self):
        config = self.gaits["fast_trot"]
        period = self.fast_trot_active_cycle_period()
        max_x = (
            config["hardware_max_x"]
            if self.hardware_mode
            else config["simulation_max_x"]
        )
        max_yaw = config["max_yaw_step"] / (
            config["stance_ratio"] * period
        )
        scale = self.fast_trot_speed_scale()
        return max_x * scale, config["max_y"] * scale, max_yaw * scale

    def is_phase_trot(self):
        return self.config.get("type") in (
            "phase_trot",
            "physical_trot",
            "real_safe_trot",
        )

    def is_physical_trot(self):
        return self.config.get("type") == "physical_trot"

    def is_spot_walk(self):
        return self.config.get("type") == "spot_walk"

    def is_video_walk(self):
        return self.config.get("type") == "stable_crawl"

    def is_stable_crawl(self):
        return self.config.get("type") in ("spot_walk", "stable_crawl")

    def gait_time_scale(self):
        if not self.is_video_walk():
            return 1.0
        key = "hardware_time_scale" if self.hardware_mode else "simulation_time_scale"
        return self.config[key]

    def set_support_feedback(self, feedback=None):
        """Store optional downstream/contact readiness for the next gait tick.

        ``contacts`` may later map leg names to booleans. Until sensors exist,
        motion passes joint tracking freshness/error information. Pure callers
        may omit feedback; that is an explicit commanded-trajectory assumption.
        """
        if feedback is None:
            self.video_support_feedback = {}
            return
        if not isinstance(feedback, dict):
            raise ValueError("support feedback must be a mapping")
        self.video_support_feedback = dict(feedback)

    def reset_spot_walk_state(self, now=None):
        self.spot_phase_index = 0
        self.spot_phase_name = "shift_to_support"
        self.spot_phase_start_time = now
        self.spot_phase_progress = 0.0
        self.spot_swing_leg = SPOT_WALK_LEG_ORDER[0]
        self.spot_stop_requested = False
        self.spot_forced_stop = False
        self.spot_body_shift = (0.0, 0.0)
        self.spot_shift_origin = (0.0, 0.0)
        self.spot_shift_target = (0.0, 0.0)
        self.spot_requested_body_offset = (0.0, 0.0)
        self.spot_effective_body_offset = (0.0, 0.0)
        self.spot_body_offset_origin = (0.0, 0.0)
        self.spot_body_offset_target = (0.0, 0.0)
        self.spot_support_projection = (0.0, 0.0)
        self.video_leg_slot = 0
        self.video_step_state = "STOPPED"
        self.video_state_start_time = now
        self.video_shift_completion = 0.0
        self.video_support_valid = True
        self.video_support_clearance = 0.0
        self.video_lift_allowed = False
        self.video_warning = ""
        self.video_center_feet_origins = copy_feet(self.feet)
        self.video_height_offset = 0.0
        self.video_height_origin = 0.0

    def reset(self, now=None):
        neutral_feet = self.nominal_feet()
        self.feet = copy_feet(neutral_feet)
        self.active = False
        self.settling = False
        self.stop_requested = False
        self.forced_stop = False
        self.body_x_world = 0.0
        self.body_y_world = 0.0
        self.body_yaw_world = 0.0
        self.world_feet = copy_feet(neutral_feet)
        self.settled_legs.clear()
        self.start_time = now
        self.settle_start_time = None
        self.settle_origins = copy_feet(neutral_feet)
        self.crab_mode = False
        self.was_swinging = {leg: False for leg in LEG_ORDER}
        self.swing_origins = copy_feet(neutral_feet)
        self.swing_targets = copy_feet(neutral_feet)
        self.swing_heights = {
            leg: self.config.get("min_step_height", 0.0) for leg in LEG_ORDER
        }
        self.trot_phase = 0.0
        self.current_gait_frequency = self.config.get(
            "min_gait_frequency",
            self.config.get("gait_frequency", 0.0),
        )
        self.fast_trot_start_time = None
        self.fast_trot_startup_scale = 0.0
        self.fast_trot_input_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_planned_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_requested_stride = 0.0
        self.fast_trot_achieved_stride = 0.0
        self.fast_trot_cycle_period = self.fast_trot_active_cycle_period()
        self.reset_fast_trot_runtime()
        self.swing_start_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.swing_end_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.reset_spot_walk_state(now)
        if self.is_video_walk():
            self.spot_phase_name = "stopped"
            self.video_step_state = "STOPPED"
            self.video_support_valid = True
            self.update_video_debug(
                0.0,
                [],
                list(LEG_ORDER),
                self.video_body_motion(),
            )
        else:
            self.update_debug(
                0.0,
                [],
                list(LEG_ORDER),
                phase_name="shift_to_support",
            )

    def set_gait(self, gait_name, now):
        gait_name = canonical_gait_name(gait_name)
        if gait_name not in self.gaits:
            raise ValueError("Unknown gait: %s" % gait_name)
        self.gait_name = gait_name
        self.start_time = now
        self.settle_start_time = None
        self.crab_mode = False
        self.was_swinging = {leg: False for leg in LEG_ORDER}
        self.swing_origins = copy_feet(self.feet)
        self.swing_targets = copy_feet(self.feet)
        self.swing_heights = {
            leg: self.config.get("min_step_height", 0.0) for leg in LEG_ORDER
        }
        self.trot_phase = 0.0
        self.current_gait_frequency = self.config.get(
            "min_gait_frequency",
            self.config.get("gait_frequency", 0.0),
        )
        self.fast_trot_start_time = None
        self.fast_trot_startup_scale = 0.0
        self.fast_trot_input_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_planned_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_requested_stride = 0.0
        self.fast_trot_achieved_stride = 0.0
        self.fast_trot_cycle_period = self.fast_trot_active_cycle_period()
        self.reset_fast_trot_runtime()
        self.swing_start_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.swing_end_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.reset_spot_walk_state(now)
        self.settling = False
        self.stop_requested = False
        self.forced_stop = False
        self.settled_legs.clear()
        if self.is_video_walk():
            self.spot_phase_name = "stopped"
            self.video_step_state = "STOPPED"
            self.video_support_valid = True
            self.update_video_debug(
                0.0,
                [],
                list(LEG_ORDER),
                self.video_body_motion(),
            )

    def body_to_world(self, point, body_x=None, body_y=None, body_yaw=None):
        """Transform a body/local foot point into the gait world frame."""
        if body_x is None:
            body_x = self.body_x_world
        if body_y is None:
            body_y = self.body_y_world
        if body_yaw is None:
            body_yaw = self.body_yaw_world
        x, y, z = point
        world_x, world_y = rotate_xy(x, y, body_yaw)
        return body_x + world_x, body_y + world_y, z

    def world_to_body(self, point):
        """Convert a locked world foothold into the current body/local frame."""
        x, y, z = point
        local_x, local_y = rotate_xy(
            x - self.body_x_world,
            y - self.body_y_world,
            -self.body_yaw_world,
        )
        return local_x, local_y, z

    def sync_world_feet_from_body(self):
        self.world_feet = {
            leg: self.body_to_world(self.feet[leg])
            for leg in LEG_ORDER
        }

    def set_current_feet(self, feet):
        """Seed gait state from the controller's current FK pose."""
        seeded = {}
        for leg in LEG_ORDER:
            if leg not in feet or len(feet[leg]) != 3:
                raise ValueError("current feet must contain every VOLT leg")
            point = tuple(float(value) for value in feet[leg])
            if not all(math.isfinite(value) for value in point):
                raise ValueError("current foot targets must be finite")
            seeded[leg] = point
        self.feet = seeded
        self.sync_world_feet_from_body()

    def hold_current_feet(self, feet, now=None):
        """Stop phase advancement while preserving the measured finite pose.

        Command ownership may be removed at any point, including mid-swing.
        Resetting to nominal feet in that situation would create a jump when
        ownership is restored.  This method instead makes the measured pose the
        next gait seed and clears every latent swing/settle request.
        """
        self.set_current_feet(feet)
        self.active = False
        self.settling = False
        self.stop_requested = False
        self.forced_stop = False
        self.settled_legs.clear()
        self.settle_start_time = None
        self.start_time = now
        self.was_swinging = {leg: False for leg in LEG_ORDER}
        self.reset_spot_walk_state(now)
        self.update_debug(
            0.0,
            [],
            list(LEG_ORDER),
            phase_name="ownership_hold",
        )

    def integrate_body_pose(self, velocity, dt):
        """Move the body through the world while stance feet remain planted."""
        self.body_x_world, self.body_y_world, self.body_yaw_world = (
            self.predict_body_pose(velocity, dt)
        )

    def predict_body_pose(self, velocity, dt):
        """Integrate a constant body-frame twist, including curved turns."""
        vx, vy, yaw_rate = velocity
        if abs(yaw_rate) < 1e-6:
            local_x = vx * dt
            local_y = vy * dt
        else:
            angle = yaw_rate * dt
            sine = math.sin(angle)
            one_minus_cosine = 1.0 - math.cos(angle)
            local_x = (
                vx * sine - vy * one_minus_cosine
            ) / yaw_rate
            local_y = (
                vx * one_minus_cosine + vy * sine
            ) / yaw_rate
        world_x, world_y = rotate_xy(local_x, local_y, self.body_yaw_world)
        return (
            self.body_x_world + world_x,
            self.body_y_world + world_y,
            self.body_yaw_world + yaw_rate * dt,
        )

    def velocity_activity(self, velocity, step_in_place=False):
        if self.is_physical_trot():
            limits = self.fast_trot_command_limits()
            activity = normalized_velocity_activity(
                {
                    "max_x": max(limits[0], 1e-9),
                    "max_y": max(limits[1], 1e-9),
                    "max_yaw": max(limits[2], 1e-9),
                },
                velocity,
            )
            if step_in_place:
                activity = max(
                    activity,
                    self.config["step_in_place_activity"],
                )
            return clamp(activity, 0.0, 1.0)
        activity = normalized_velocity_activity(self.config, velocity)
        if step_in_place:
            activity = max(activity, self.config["step_in_place_activity"])
        return clamp(activity, 0.0, 1.0)

    def fast_trot_startup_profile(self, now=None, elapsed=None):
        """Hold one stable slow cycle, then ramp stride before cadence."""
        config = self.config
        if elapsed is None:
            if now is None:
                raise ValueError("fast-trot startup needs now or elapsed")
            if self.fast_trot_start_time is None:
                self.fast_trot_start_time = now
            elapsed = max(0.0, now - self.fast_trot_start_time)
        else:
            elapsed = max(0.0, float(elapsed))
        ramp_time = config["startup_ramp_time"]
        target_period = self.fast_trot_active_cycle_period()
        hold_time = (
            target_period
            / config["startup_cycle_speed_fraction"]
        )
        ramp = clamp((elapsed - hold_time) / ramp_time, 0.0, 1.0)
        ramp = smootherstep(ramp)

        stride_fraction = config["startup_stride_fraction"] + (
            1.0 - config["startup_stride_fraction"]
        ) * ramp
        # Establish most of the Cartesian stride before increasing cadence.
        cadence_ramp = smootherstep(
            clamp((ramp - 0.70) / 0.30, 0.0, 1.0)
        )
        cycle_speed_fraction = config["startup_cycle_speed_fraction"] + (
            1.0 - config["startup_cycle_speed_fraction"]
        ) * cadence_ramp
        step_height_fraction = 0.60 + 0.40 * ramp
        self.fast_trot_startup_scale = stride_fraction
        return stride_fraction, cycle_speed_fraction, step_height_fraction

    def plan_fast_trot_motion(
        self,
        now,
        velocity,
        step_in_place=False,
        startup_elapsed=None,
    ):
        """Convert a filtered Twist fraction into a compatible Cartesian sweep."""
        config = self.config
        if self.stop_requested and any(self.was_swinging.values()):
            return self.plan_fast_trot_shutdown(now)
        self.fast_trot_shutdown_start_time = None
        vx, _vy, yaw_rate = velocity
        max_x, _max_y, max_yaw = self.fast_trot_command_limits()
        x_activity = clamp(abs(vx) / max(max_x, 1e-9), 0.0, 1.0)
        yaw_activity = clamp(abs(yaw_rate) / max(max_yaw, 1e-9), 0.0, 1.0)
        (
            startup_stride,
            startup_cycle_speed,
            startup_height,
        ) = self.fast_trot_startup_profile(
            now,
            elapsed=startup_elapsed,
        )

        target_period = self.fast_trot_active_cycle_period()
        self.fast_trot_cycle_period = target_period / max(
            startup_cycle_speed,
            1e-6,
        )
        stance_time = (
            config["stance_ratio"] * self.fast_trot_cycle_period
        )

        requested_stride = 0.0
        if x_activity > 1e-6:
            stable_stride = clamp(
                config["step_length_x"] * x_activity,
                config["min_step_length_x"],
                config["max_step_length_x"],
            )
            engagement = smootherstep(
                clamp(x_activity / 0.25, 0.0, 1.0)
            )
            base_stride = stable_stride * engagement
            requested_stride = (
                base_stride
                * self.fast_trot_tuning["stride_scale"]
                * self.fast_trot_backend_stride_scale()
                * startup_stride
            )
            requested_stride = min(
                requested_stride,
                config["max_step_length_x"],
            )
        compatible_vx = (
            math.copysign(requested_stride / max(stance_time, 1e-9), vx)
            if requested_stride > 0.0
            else 0.0
        )
        compatible_vx = clamp(compatible_vx, -max_x, max_x)
        achieved_stride = abs(compatible_vx) * stance_time

        requested_yaw_step = (
            config["max_yaw_step"]
            * yaw_activity
            * startup_stride
        )
        compatible_yaw = (
            math.copysign(
                requested_yaw_step / max(stance_time, 1e-9),
                yaw_rate,
            )
            if requested_yaw_step > 0.0
            else 0.0
        )
        compatible_yaw = clamp(compatible_yaw, -max_yaw, max_yaw)

        self.fast_trot_input_velocity = tuple(float(value) for value in velocity)
        self.fast_trot_planned_velocity = (
            compatible_vx,
            0.0,
            compatible_yaw,
        )
        self.fast_trot_requested_stride = requested_stride
        self.fast_trot_achieved_stride = achieved_stride
        self.fast_trot_step_height = (
            self.fast_trot_tuning["step_height"] * startup_height
        )
        if step_in_place and self.fast_trot_step_height <= 0.0:
            self.fast_trot_step_height = self.fast_trot_tuning["step_height"] * 0.60
        return self.fast_trot_planned_velocity

    def advance_trot_phase(self, dt, velocity, step_in_place=False):
        """Advance a continuous phase with speed-adaptive, slew-limited cadence."""
        config = self.config
        if self.is_physical_trot():
            self.current_gait_frequency = 1.0 / max(
                self.fast_trot_cycle_period,
                1e-6,
            )
            self.trot_phase = (
                self.trot_phase
                + self.current_gait_frequency * max(dt, 0.0)
            ) % 1.0
            return self.trot_phase
        activity = smootherstep(self.velocity_activity(velocity, step_in_place))
        minimum = config["min_gait_frequency"]
        maximum = config["gait_frequency"]
        target_frequency = minimum + (maximum - minimum) * activity
        cadence_step = config.get("cadence_acceleration", 2.5) * max(dt, 0.0)
        self.current_gait_frequency += clamp(
            target_frequency - self.current_gait_frequency,
            -cadence_step,
            cadence_step,
        )
        self.trot_phase = (
            self.trot_phase + self.current_gait_frequency * max(dt, 0.0)
        ) % 1.0
        return self.trot_phase

    def adaptive_step_height(self, velocity, step_in_place=False):
        config = self.config
        if self.is_physical_trot():
            return float(
                getattr(
                    self,
                    "fast_trot_step_height",
                    self.fast_trot_tuning["step_height"] * 0.60,
                )
            )
        activity = smootherstep(self.velocity_activity(velocity, step_in_place))
        return config["min_step_height"] + (
            config["step_height"] - config["min_step_height"]
        ) * activity

    def body_motion(self, phase, velocity):
        """Small natural body bob and attitude compensation for trot support."""
        config = self.config
        if self.is_physical_trot():
            activity = self.velocity_activity(
                self.fast_trot_input_velocity,
            )
            forward_activity = (
                clamp(
                    self.fast_trot_input_velocity[0]
                    / max(self.fast_trot_command_limits()[0], 1e-9),
                    0.0,
                    1.0,
                )
            )
            double_frequency = 0.5 - 0.5 * math.cos(
                4.0 * math.pi * phase
            )
            return {
                "height": (
                    config["body_height_offset"]
                    * self.fast_trot_startup_scale
                    + config["body_bob"]
                    * double_frequency
                    * activity
                ),
                "roll": (
                    config["body_roll"]
                    * math.sin(2.0 * math.pi * phase)
                    * activity
                ),
                "pitch": (
                    config["forward_pitch_bias"]
                    * smootherstep(forward_activity)
                    * self.fast_trot_startup_scale
                ),
            }
        linear_scale = clamp(
            math.hypot(
                velocity[0] / max(config["max_x"], 1e-6),
                velocity[1] / max(config["max_y"], 1e-6),
            ),
            0.0,
            1.0,
        )
        activity = normalized_velocity_activity(config, velocity)
        diagonal = math.sin(2.0 * math.pi * phase)
        double_frequency = 0.5 - 0.5 * math.cos(4.0 * math.pi * phase)
        return {
            "height": config["body_bob"] * double_frequency * activity,
            "roll": config["body_roll"] * diagonal * activity,
            "pitch": -config["body_pitch"]
            * math.cos(2.0 * math.pi * phase)
            * linear_scale,
        }

    def request_stop(self):
        if self.active:
            if self.is_stable_crawl():
                self.spot_stop_requested = True
                self.spot_forced_stop = True
                return
            self.stop_requested = True
            self.forced_stop = True

    def phase(self, now):
        if self.start_time is None:
            self.start_time = now
        return (now - self.start_time) % self.config["period"]

    def cycle_phase(self, now):
        if self.start_time is None:
            self.start_time = now
        if self.is_stable_crawl():
            return float(self.debug_state.get("phase", 0.0))
        if self.is_phase_trot():
            return self.trot_phase
        return ((now - self.start_time) / self.config["period"]) % 1.0

    def phase_starts(self):
        key = "crab_swing_starts" if self.crab_mode else "swing_starts"
        return self.config[key]

    def swing_progress(self, leg_name, phase):
        config = self.config
        start = self.phase_starts()[leg_name] * config["period"]
        elapsed = periodic_elapsed(phase, start, config["period"])
        if elapsed >= config["swing_time"]:
            return None
        return elapsed / config["swing_time"]

    def touchdown_target(self, leg_name, velocity):
        config = self.config
        nominal = self.nominal_foot(leg_name)
        vx, vy, yaw_rate = velocity
        stance_time = config["period"] - config["swing_time"]

        predicted = rotate_z(nominal, 0.5 * yaw_rate * stance_time)
        offset_x = predicted[0] - nominal[0] + 0.5 * vx * stance_time
        offset_y = predicted[1] - nominal[1] + 0.5 * vy * stance_time

        offset_x, offset_y = clamp_elliptical_offset(
            offset_x,
            offset_y,
            config["max_step_x"],
            config["max_step_y"],
        )

        return nominal[0] + offset_x, nominal[1] + offset_y, nominal[2]

    def trot_touchdown_world(self, leg_name, velocity):
        """Pick the next world foothold ahead of the body at swing touchdown."""
        config = self.config
        nominal = self.nominal_foot(leg_name)
        vx, vy, yaw_rate = velocity
        if self.is_physical_trot():
            swing_time = (
                config["swing_ratio"] * self.fast_trot_cycle_period
            )
            stance_time = (
                config["stance_ratio"] * self.fast_trot_cycle_period
            )
            stride = self.fast_trot_requested_stride
            direction = 0.0 if abs(vx) <= 1e-9 else math.copysign(1.0, vx)
            stride_ratio = (
                stride / config["step_length_x"]
                if stride > 0.0
                else 0.0
            )
            lead_x = (
                config["touchdown_lead_x"]
                * stride_ratio
                * direction
            )
            touchdown_local = (
                nominal[0] + lead_x,
                nominal[1],
                nominal[2],
            )
            future_x, future_y, touchdown_yaw = self.predict_body_pose(
                velocity,
                swing_time,
            )
            future_yaw = touchdown_yaw + yaw_rate * 0.5 * stance_time
            return self.body_to_world(
                touchdown_local,
                future_x,
                future_y,
                future_yaw,
            )
        frequency = max(self.current_gait_frequency, 1e-6)
        swing_time = config["swing_ratio"] / frequency
        stance_time = config["stance_ratio"] / frequency

        lead_x = clamp(
            0.5 * vx * stance_time,
            -0.5 * config["stride_length"],
            0.5 * config["stride_length"],
        )
        lead_y = clamp(
            0.5 * vy * stance_time,
            -0.5 * config["lateral_stride_length"],
            0.5 * config["lateral_stride_length"],
        )
        touchdown_local = (
            nominal[0] + lead_x,
            nominal[1] + lead_y,
            nominal[2],
        )

        future_x, future_y, touchdown_yaw = self.predict_body_pose(
            velocity,
            swing_time,
        )
        future_yaw = touchdown_yaw + yaw_rate * 0.5 * stance_time
        return self.body_to_world(
            touchdown_local,
            future_x,
            future_y,
            future_yaw,
        )

    def stance_step(self, location, velocity, dt):
        vx, vy, yaw_rate = velocity
        rotated = rotate_z(location, -yaw_rate * dt)
        return rotated[0] - vx * dt, rotated[1] - vy * dt, NOMINAL_FEET_Z

    def stance_foot_velocity(self, location, velocity):
        """Instantaneous planted-foot velocity in the rotating body frame."""
        vx, vy, yaw_rate = velocity
        return (
            -vx + yaw_rate * location[1],
            -vy - yaw_rate * location[0],
            0.0,
        )

    def swing_step(self, leg_name, progress, velocity):
        origin = self.swing_origins[leg_name]
        target = self.touchdown_target(leg_name, velocity)
        blend = smootherstep(progress)

        x = origin[0] + (target[0] - origin[0]) * blend
        y = origin[1] + (target[1] - origin[1]) * blend
        ground_z = origin[2] + (target[2] - origin[2]) * blend

        lift_shape = 16.0 * progress ** 2 * (1.0 - progress) ** 2
        z = ground_z + self.config["clearance"] * lift_shape
        return x, y, z

    def support_shift(self, phase):
        config = self.config
        transition = config["shift_time"]
        if transition <= 0.0:
            return 0.0, 0.0

        shift_x = 0.0
        shift_y = 0.0
        total_weight = 0.0
        period = config["period"]
        swing_time = config["swing_time"]

        for leg_name in LEG_ORDER:
            start = self.phase_starts()[leg_name] * period
            elapsed = periodic_elapsed(phase, start, period)
            if elapsed < swing_time:
                weight = 1.0
            else:
                before = (start - phase) % period
                after = (phase - (start + swing_time)) % period
                if before < transition:
                    weight = smootherstep(1.0 - before / transition)
                elif after < transition:
                    weight = smootherstep(1.0 - after / transition)
                else:
                    weight = 0.0

            nominal = NOMINAL_FEET[leg_name]
            shift_x -= math.copysign(config["body_shift_x"] * weight, nominal[0])
            shift_y -= math.copysign(config["body_shift_y"] * weight, nominal[1])
            total_weight += weight

        if total_weight > 1.0:
            shift_x /= total_weight
            shift_y /= total_weight
        return shift_x, shift_y

    def begin_trot_swing(self, leg_name, velocity):
        """Freeze a swing target so command changes cannot jerk the foot."""
        origin = self.feet[leg_name]
        target = self.touchdown_target(leg_name, velocity)
        self.swing_origins[leg_name] = origin
        self.swing_targets[leg_name] = target
        self.swing_start_velocities[leg_name] = self.stance_foot_velocity(
            origin,
            velocity,
        )
        self.swing_end_velocities[leg_name] = self.stance_foot_velocity(
            target,
            velocity,
        )

    def phase_trot_swing_step(self, leg_name, swing):
        """Return a smooth swing arc between old and new world footholds."""
        origin = self.swing_origins[leg_name]
        target = self.swing_targets[leg_name]

        swing = clamp(swing, 0.0, 1.0)
        hardware_contact_profile = (
            self.hardware_mode
            and "liftoff_blend" in self.config
            and "touchdown_blend" in self.config
        )
        if hardware_contact_profile:
            # Unload vertically before translating, then finish horizontal
            # placement before accepting load.  Quintic blends make velocity
            # and acceleration zero at every sub-phase boundary.
            liftoff = self.config["liftoff_blend"]
            touchdown = self.config["touchdown_blend"]
            transfer_end = 1.0 - touchdown
            if swing < liftoff:
                lift = smootherstep(swing / max(liftoff, 1e-9))
                transfer = 0.0
            elif swing <= transfer_end:
                lift = 1.0
                transfer = smootherstep(
                    (swing - liftoff)
                    / max(transfer_end - liftoff, 1e-9)
                )
            else:
                lift = 1.0 - smootherstep(
                    (swing - transfer_end) / max(touchdown, 1e-9)
                )
                transfer = 1.0
            x = origin[0] + (target[0] - origin[0]) * transfer
            y = origin[1] + (target[1] - origin[1]) * transfer
            ground_z = origin[2] + (target[2] - origin[2]) * transfer
        else:
            # Preserve the established simulation trajectory.
            transfer = smootherstep(swing)
            x = origin[0] + (target[0] - origin[0]) * transfer
            y = origin[1] + (target[1] - origin[1]) * transfer
            ground_z = origin[2] + (target[2] - origin[2]) * transfer
            lift = (
                math.sin(math.pi * swing) ** 2
                if self.is_physical_trot()
                else smooth_bump(swing)
            )
        z = ground_z + self.swing_heights[leg_name] * lift
        return x, y, z

    def settle_feet(self, now):
        config = self.config
        physical_loaded_hold = self.hardware_mode and self.is_physical_trot()
        if self.settle_start_time is None:
            self.settle_start_time = now
            self.settle_origins = copy_feet(self.feet)
            if self.is_physical_trot():
                self.fast_trot_settle_origin_velocity = tuple(
                    self.fast_trot_planned_velocity
                )
                self.fast_trot_settle_origin_input_velocity = tuple(
                    self.fast_trot_input_velocity
                )
                self.fast_trot_requested_stride = 0.0
                self.fast_trot_achieved_stride = 0.0
        progress = (now - self.settle_start_time) / config.get(
            "settle_time",
            0.60,
        )
        if progress >= 1.0:
            if not physical_loaded_hold:
                self.feet = copy_feet(NOMINAL_FEET)
                self.world_feet = copy_feet(NOMINAL_FEET)
            self.active = False
            self.settling = False
            self.stop_requested = False
            self.forced_stop = False
            self.crab_mode = False
            self.start_time = now
            self.trot_phase = 0.0
            self.was_swinging = {leg: False for leg in LEG_ORDER}
            if self.is_physical_trot():
                self.fast_trot_startup_scale = 0.0
                self.fast_trot_input_velocity = (0.0, 0.0, 0.0)
                self.fast_trot_planned_velocity = (0.0, 0.0, 0.0)
                self.fast_trot_requested_stride = 0.0
                self.fast_trot_achieved_stride = 0.0
                self.fast_trot_step_height = 0.0
                self.reset_fast_trot_runtime()
            self.update_debug(
                0.0,
                [],
                list(LEG_ORDER),
                {"height": 0.0, "roll": 0.0, "pitch": 0.0}
                if physical_loaded_hold
                else None,
                phase_name=(
                    "loaded_hold"
                    if physical_loaded_hold
                    else "cycle"
                ),
                step_state=(
                    "LOADED_HOLD"
                    if physical_loaded_hold
                    else "CYCLE"
                ),
            )
            return copy_feet(self.feet), (0.0, 0.0), False

        blend = smootherstep(progress)
        if not physical_loaded_hold:
            for leg_name in LEG_ORDER:
                origin = self.settle_origins[leg_name]
                target = NOMINAL_FEET[leg_name]
                self.feet[leg_name] = tuple(
                    origin[index] + (target[index] - origin[index]) * blend
                    for index in range(3)
                )
        if self.is_physical_trot():
            self.fast_trot_planned_velocity = tuple(
                value * (1.0 - blend)
                for value in self.fast_trot_settle_origin_velocity
            )
            self.fast_trot_input_velocity = tuple(
                value * (1.0 - blend)
                for value in self.fast_trot_settle_origin_input_velocity
            )
            body_motion = {
                "height": (
                    self.config["body_height_offset"]
                    * self.fast_trot_startup_scale
                    * (1.0 - blend)
                ),
                "roll": 0.0,
                "pitch": (
                    self.config["forward_pitch_bias"]
                    * self.fast_trot_startup_scale
                    * (1.0 - blend)
                    if self.fast_trot_input_velocity[0] > 0.0
                    else 0.0
                ),
            }
        else:
            body_motion = (0.0, 0.0)
        self.update_debug(
            self.cycle_phase(now),
            [],
            list(LEG_ORDER),
            body_motion if isinstance(body_motion, dict) else None,
        )
        return copy_feet(self.feet), body_motion, True

    def spot_support_plan(
        self,
        swing_leg,
        velocity,
        body_offset=(0.0, 0.0),
    ):
        """Plan support shift and a safe effective operator body offset.

        IK applies the operator's body translation and the crawl support shift
        together.  The stability check therefore has to constrain their sum,
        not the gait shift alone.  If an extreme operator offset cannot be
        countered within the configured shift bounds, only the unsafe portion
        of that offset is temporarily clipped while this foot is in swing.
        """
        config = self.config
        try:
            body_offset = tuple(float(value) for value in body_offset)
        except (TypeError, ValueError) as exc:
            raise ValueError("body_offset must contain two numeric values") from exc
        if len(body_offset) != 2 or not all(
            math.isfinite(value) for value in body_offset
        ):
            raise ValueError("body_offset must contain two finite values")
        stance_legs = [leg for leg in LEG_ORDER if leg != swing_leg]
        triangle = [
            self.world_to_body(self.world_feet[leg])[:2]
            for leg in stance_legs
        ]
        centroid = (
            sum(point[0] for point in triangle) / 3.0,
            sum(point[1] for point in triangle) / 3.0,
        )

        # Move toward the support centroid, bounded by the configured VOLT body
        # translation. Bias opposite travel so the continuing stance motion does
        # not carry the body projection toward the open corner during swing.
        shift_x = math.copysign(
            min(abs(centroid[0]), config["body_shift_x"]),
            centroid[0],
        )
        shift_y = math.copysign(
            min(abs(centroid[1]), config["body_shift_y"]),
            centroid[1],
        )
        anticipation = 0.5 * config["swing_duration"]
        shift_x += clamp(
            -velocity[0] * anticipation,
            -0.35 * config["body_shift_x"],
            0.35 * config["body_shift_x"],
        )
        shift_y += clamp(
            -velocity[1] * anticipation,
            -0.35 * config["body_shift_y"],
            0.35 * config["body_shift_y"],
        )
        shift_x = clamp(
            shift_x,
            -config["body_shift_x"],
            config["body_shift_x"],
        )
        shift_y = clamp(
            shift_y,
            -config["body_shift_y"],
            config["body_shift_y"],
        )
        requested_projection = (
            body_offset[0] + shift_x,
            body_offset[1] + shift_y,
        )
        safe_projection = project_inside_support_triangle(
            requested_projection,
            triangle,
            config["support_margin"],
        )
        safe_shift = (
            clamp(
                safe_projection[0] - body_offset[0],
                -config["body_shift_x"],
                config["body_shift_x"],
            ),
            clamp(
                safe_projection[1] - body_offset[1],
                -config["body_shift_y"],
                config["body_shift_y"],
            ),
        )
        effective_body_offset = (
            safe_projection[0] - safe_shift[0],
            safe_projection[1] - safe_shift[1],
        )
        return safe_shift, effective_body_offset, safe_projection

    def spot_support_target(
        self,
        swing_leg,
        velocity,
        body_offset=(0.0, 0.0),
    ):
        """Compatibility helper returning the bounded gait-shift component."""
        return self.spot_support_plan(
            swing_leg,
            velocity,
            body_offset,
        )[0]

    def spot_touchdown_world(self, leg_name, velocity):
        """Freeze a bounded foothold ahead of the predicted VOLT body pose."""
        config = self.config
        nominal = NOMINAL_FEET[leg_name]
        vx, vy, yaw_rate = velocity
        stance_duration = config["cycle_period"] - config["swing_duration"]
        lead_time = config["touchdown_lead"] * stance_duration
        predicted_nominal = rotate_z(nominal, yaw_rate * lead_time)
        offset_x = predicted_nominal[0] - nominal[0] + vx * lead_time
        offset_y = predicted_nominal[1] - nominal[1] + vy * lead_time
        offset_x, offset_y = clamp_elliptical_offset(
            offset_x,
            offset_y,
            config["maximum_step_x"],
            config["maximum_step_y"],
        )
        touchdown_local = (
            nominal[0] + offset_x,
            nominal[1] + offset_y,
            nominal[2],
        )
        future_x, future_y, future_yaw = self.predict_body_pose(
            velocity,
            config["swing_duration"],
        )
        return self.body_to_world(
            touchdown_local,
            future_x,
            future_y,
            future_yaw,
        )

    def begin_spot_shift(self, now, leg_slot, velocity, body_offset=(0.0, 0.0)):
        self.spot_phase_index = (leg_slot % len(SPOT_WALK_LEG_ORDER)) * 2
        self.spot_phase_name = "shift_to_support"
        self.spot_phase_start_time = now
        self.spot_phase_progress = 0.0
        self.spot_swing_leg = SPOT_WALK_LEG_ORDER[leg_slot % len(SPOT_WALK_LEG_ORDER)]
        self.spot_shift_origin = tuple(self.spot_body_shift)
        self.spot_body_offset_origin = tuple(self.spot_effective_body_offset)
        (
            self.spot_shift_target,
            self.spot_body_offset_target,
            self.spot_support_projection,
        ) = self.spot_support_plan(
            self.spot_swing_leg,
            velocity,
            body_offset,
        )

    def begin_spot_swing(self, now, velocity, step_in_place):
        leg_name = self.spot_swing_leg
        self.spot_phase_index += 1
        self.spot_phase_name = "swing_%s" % leg_name
        self.spot_phase_start_time = now
        self.spot_phase_progress = 0.0
        self.swing_origins[leg_name] = self.world_feet[leg_name]
        swing_velocity = (0.0, 0.0, 0.0) if step_in_place else velocity
        self.swing_targets[leg_name] = self.spot_touchdown_world(
            leg_name,
            swing_velocity,
        )
        self.swing_heights[leg_name] = self.config["step_height"]

    def begin_spot_settle(self, now, body_offset=(0.0, 0.0)):
        self.settling = True
        self.spot_phase_name = "settle_to_nominal"
        self.spot_phase_index = len(SPOT_WALK_LEG_ORDER) * 2
        self.spot_phase_start_time = now
        self.spot_phase_progress = 0.0
        self.settle_start_time = now
        self.settle_origins = copy_feet(self.feet)
        self.spot_shift_origin = tuple(self.spot_body_shift)
        self.spot_body_offset_origin = tuple(self.spot_effective_body_offset)
        self.spot_body_offset_target = tuple(body_offset)

    def spot_settle_step(self, now):
        duration = self.config["settle_duration"]
        progress = clamp((now - self.settle_start_time) / duration, 0.0, 1.0)
        blend = smootherstep(progress)
        for leg_name in LEG_ORDER:
            origin = self.settle_origins[leg_name]
            target = self.nominal_foot(leg_name)
            self.feet[leg_name] = tuple(
                origin[index] + (target[index] - origin[index]) * blend
                for index in range(3)
            )
        self.spot_body_shift = tuple(
            value * (1.0 - blend) for value in self.spot_shift_origin
        )
        self.spot_effective_body_offset = tuple(
            self.spot_body_offset_origin[index]
            + (
                self.spot_body_offset_target[index]
                - self.spot_body_offset_origin[index]
            )
            * blend
            for index in range(2)
        )
        self.world_feet = {
            leg: self.body_to_world(self.feet[leg])
            for leg in LEG_ORDER
        }
        self.spot_phase_progress = progress
        body_motion = {
            "x": self.spot_body_shift[0],
            "y": self.spot_body_shift[1],
            "body_x_override": self.spot_effective_body_offset[0],
            "body_y_override": self.spot_effective_body_offset[1],
            "height": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
        }
        self.update_debug(
            0.0,
            [],
            list(LEG_ORDER),
            body_motion,
            phase_index=self.spot_phase_index,
            phase_name=self.spot_phase_name,
            phase_progress=progress,
            body_shift=self.spot_body_shift,
            effective_body_offset=self.spot_effective_body_offset,
            support_projection=(
                self.spot_effective_body_offset[0] + self.spot_body_shift[0],
                self.spot_effective_body_offset[1] + self.spot_body_shift[1],
            ),
        )
        if progress >= 1.0:
            self.feet = copy_feet(NOMINAL_FEET)
            self.sync_world_feet_from_body()
            self.active = False
            self.settling = False
            self.spot_stop_requested = False
            self.spot_forced_stop = False
            self.start_time = now
            self.reset_spot_walk_state(now)
            self.update_debug(
                0.0,
                [],
                list(LEG_ORDER),
                phase_name="shift_to_support",
            )
            return copy_feet(self.feet), body_motion, False
        return copy_feet(self.feet), body_motion, True

    def video_requested_body_shift(self, swing_leg):
        """Return the VOLT-frame diagonal shift associated with one swing leg."""
        config = self.config
        support_name = VIDEO_WALK_SUPPORT_BY_LEG[swing_leg]
        forward = config["forward_body_shift"]
        rearward = config["rearward_body_shift"]
        lateral = config["lateral_body_shift"]
        targets = {
            "front_left": (forward, lateral),
            "back_left": (-rearward, lateral),
            "front_right": (forward, -lateral),
            "back_right": (-rearward, -lateral),
        }
        return targets[support_name]

    def video_support_triangle(self, swing_leg):
        stance_legs = [leg for leg in LEG_ORDER if leg != swing_leg]
        return [
            self.world_to_body(self.world_feet[leg])[:2]
            for leg in stance_legs
        ]

    @staticmethod
    def video_triangle_is_nondegenerate(triangle):
        if len(triangle) != 3 or not all(
            len(point) == 2
            and all(math.isfinite(float(value)) for value in point)
            for point in triangle
        ):
            return False
        ordered = _ordered_triangle(list(triangle))
        double_area = abs(sum(
            start[0] * end[1] - end[0] * start[1]
            for start, end in zip(ordered, ordered[1:] + ordered[:1])
        ))
        return double_area > 1e-8

    def video_validate_support_target(self, swing_leg, support_target):
        """Validate a frozen body projection against the current stance feet."""
        triangle = self.video_support_triangle(swing_leg)
        if not self.video_triangle_is_nondegenerate(triangle):
            return False, -1.0
        centroid = (
            sum(point[0] for point in triangle) / 3.0,
            sum(point[1] for point in triangle) / 3.0,
        )
        margin = self.config["support_margin"]
        centroid_clearance = support_clearance(centroid, triangle)
        target_clearance = support_clearance(support_target, triangle)
        valid = (
            math.isfinite(centroid_clearance)
            and math.isfinite(target_clearance)
            and centroid_clearance >= margin
            and target_clearance >= margin - 1e-6
        )
        return valid, target_clearance

    def video_support_plan(self, swing_leg, body_offset=(0.0, 0.0)):
        """Project the requested diagonal shift into the three-foot safe inset."""
        config = self.config
        requested_shift = self.video_requested_body_shift(swing_leg)
        triangle = self.video_support_triangle(swing_leg)
        if not self.video_triangle_is_nondegenerate(triangle):
            return (
                tuple(self.spot_body_shift),
                tuple(body_offset),
                tuple(body_offset),
                False,
                -1.0,
            )

        centroid = (
            sum(point[0] for point in triangle) / 3.0,
            sum(point[1] for point in triangle) / 3.0,
        )
        margin = config["support_margin"]
        centroid_clearance = support_clearance(centroid, triangle)
        if not math.isfinite(centroid_clearance) or centroid_clearance < margin:
            return (
                tuple(self.spot_body_shift),
                tuple(body_offset),
                tuple(body_offset),
                False,
                centroid_clearance,
            )

        requested_projection = (
            body_offset[0] + requested_shift[0],
            body_offset[1] + requested_shift[1],
        )
        safe_projection = project_inside_support_triangle(
            requested_projection,
            triangle,
            margin,
        )
        target_clearance = support_clearance(safe_projection, triangle)
        valid = (
            all(math.isfinite(value) for value in safe_projection)
            and target_clearance >= margin - 1e-6
        )
        if not valid:
            return (
                tuple(self.spot_body_shift),
                tuple(body_offset),
                tuple(body_offset),
                False,
                target_clearance,
            )

        safe_shift = (
            clamp(
                safe_projection[0] - body_offset[0],
                -config["rearward_body_shift"],
                config["forward_body_shift"],
            ),
            clamp(
                safe_projection[1] - body_offset[1],
                -config["lateral_body_shift"],
                config["lateral_body_shift"],
            ),
        )
        effective_body_offset = (
            safe_projection[0] - safe_shift[0],
            safe_projection[1] - safe_shift[1],
        )
        return (
            safe_shift,
            effective_body_offset,
            safe_projection,
            True,
            target_clearance,
        )

    def video_support_feedback_ready(self, swing_leg, touchdown=False):
        """Gate lift using optional tracking and future contact feedback."""
        feedback = self.video_support_feedback
        if feedback.get("command_ready") is False:
            return False, "support shift is still being filtered"
        if (
            feedback.get("tracking_available") is True
            and feedback.get("tracking_ready") is False
        ):
            return False, "joint tracking error is above the lift threshold"
        if (
            feedback.get("tracking_required") is True
            and feedback.get("tracking_available") is False
            and feedback.get("tracking_assumed") is not True
        ):
            return False, "fresh joint tracking feedback is unavailable"

        contacts = feedback.get("contacts")
        if contacts is not None:
            if not isinstance(contacts, dict):
                return False, "contact feedback is malformed"
            required_legs = (
                list(LEG_ORDER)
                if touchdown
                else [leg for leg in LEG_ORDER if leg != swing_leg]
            )
            if not all(contacts.get(leg) is True for leg in required_legs):
                return False, (
                    "touchdown contact is not confirmed"
                    if touchdown
                    else "one or more stance contacts are not confirmed"
                )
        return True, ""

    def video_touchdown_world(self, leg_name, velocity):
        """Freeze a workspace-bounded Cartesian foothold for the video crawl."""
        config = self.config
        nominal = self.nominal_foot(leg_name)
        vx, vy, yaw_rate = velocity
        stance_duration = (
            config["full_cycle_duration"] - config["swing_duration"]
        ) * self.gait_time_scale()
        lead_time = config["touchdown_lead"] * stance_duration
        yaw_step = clamp(
            yaw_rate * lead_time,
            -config["max_yaw_step"],
            config["max_yaw_step"],
        )
        rotated_nominal = rotate_z(nominal, yaw_step)
        offset_x = rotated_nominal[0] - nominal[0] + vx * lead_time
        offset_y = rotated_nominal[1] - nominal[1] + vy * lead_time
        offset_x, offset_y = clamp_elliptical_offset(
            offset_x,
            offset_y,
            config["max_step_x"],
            config["max_step_y"],
        )
        touchdown_local = (
            nominal[0] + offset_x,
            nominal[1] + offset_y,
            nominal[2],
        )
        future_x, future_y, future_yaw = self.predict_body_pose(
            velocity,
            config["swing_duration"] * self.gait_time_scale(),
        )
        return self.body_to_world(
            touchdown_local,
            future_x,
            future_y,
            future_yaw,
        )

    def begin_video_shift(self, now, leg_slot, body_offset=(0.0, 0.0)):
        sequence = self.config["leg_sequence"]
        self.video_leg_slot = leg_slot % len(sequence)
        self.spot_swing_leg = sequence[self.video_leg_slot]
        support_name = VIDEO_WALK_SUPPORT_BY_LEG[self.spot_swing_leg]
        self.spot_phase_index = self.video_leg_slot * 2
        self.spot_phase_name = "shift_%s" % support_name
        self.spot_phase_start_time = now
        self.video_state_start_time = now
        self.video_step_state = "SHIFT"
        self.spot_phase_progress = 0.0
        self.spot_shift_origin = tuple(self.spot_body_shift)
        self.spot_body_offset_origin = tuple(self.spot_effective_body_offset)
        self.video_height_origin = self.video_height_offset
        (
            self.spot_shift_target,
            self.spot_body_offset_target,
            self.spot_support_projection,
            self.video_support_valid,
            self.video_support_clearance,
        ) = self.video_support_plan(self.spot_swing_leg, body_offset)
        self.video_shift_completion = 0.0
        self.video_lift_allowed = False
        self.video_warning = (
            ""
            if self.video_support_valid
            else "Unsafe or degenerate support polygon; refusing foot lift."
        )

    def begin_video_swing(self, now, velocity, step_in_place):
        leg_name = self.spot_swing_leg
        self.spot_phase_index = self.video_leg_slot * 2 + 1
        self.spot_phase_name = "swing_%s" % leg_name
        self.spot_phase_start_time = now
        self.video_state_start_time = now
        self.video_step_state = "LIFT_AND_SWING"
        self.spot_phase_progress = 0.0
        self.swing_origins[leg_name] = self.world_feet[leg_name]
        swing_velocity = (0.0, 0.0, 0.0) if step_in_place else velocity
        self.swing_targets[leg_name] = self.video_touchdown_world(
            leg_name,
            swing_velocity,
        )
        self.swing_heights[leg_name] = self.config["step_height"]
        self.video_lift_allowed = True
        self.video_warning = ""

    def begin_video_settle(self, now):
        self.video_step_state = "SETTLE"
        self.video_state_start_time = now
        self.spot_phase_progress = 1.0
        self.video_lift_allowed = True

    def begin_video_centering(self, now, body_offset=(0.0, 0.0)):
        self.settling = True
        self.video_step_state = "CENTERING"
        self.spot_phase_name = "centering"
        self.spot_phase_index = 8
        self.spot_phase_start_time = now
        self.video_state_start_time = now
        self.spot_phase_progress = 0.0
        self.video_center_feet_origins = copy_feet(self.feet)
        self.spot_shift_origin = tuple(self.spot_body_shift)
        self.spot_body_offset_origin = tuple(self.spot_effective_body_offset)
        self.spot_body_offset_target = tuple(body_offset)
        self.video_height_origin = self.video_height_offset
        self.video_lift_allowed = False
        self.video_warning = ""

    def video_body_motion(self, swing_progress=None):
        config = self.config
        bob = 0.0
        roll = 0.0
        pitch = 0.0
        if swing_progress is not None:
            shape = smooth_bump(swing_progress)
            bob = config["body_bob_amplitude"] * shape
            side = 1.0 if self.spot_shift_target[1] >= 0.0 else -1.0
            fore = 1.0 if self.spot_shift_target[0] >= 0.0 else -1.0
            roll = config["body_roll_amplitude"] * side * shape
            pitch = config["body_pitch_amplitude"] * fore * shape
        return {
            "x": self.spot_body_shift[0],
            "y": self.spot_body_shift[1],
            "body_x_override": self.spot_effective_body_offset[0],
            "body_y_override": self.spot_effective_body_offset[1],
            "height": self.video_height_offset + bob,
            "roll": roll,
            "pitch": pitch,
        }

    def update_video_debug(
        self,
        phase_progress,
        swing_legs,
        stance_legs,
        body_motion,
    ):
        if self.spot_phase_index < 8:
            cycle_phase = (
                self.spot_phase_index + clamp(phase_progress, 0.0, 1.0)
            ) / 8.0
        else:
            cycle_phase = 0.0
        self.update_debug(
            cycle_phase,
            swing_legs,
            stance_legs,
            body_motion,
            phase_index=self.spot_phase_index,
            phase_name=self.spot_phase_name,
            phase_progress=phase_progress,
            body_shift=self.spot_body_shift,
            effective_body_offset=self.spot_effective_body_offset,
            support_projection=self.spot_support_projection,
            step_state=self.video_step_state,
            body_shift_target=self.spot_shift_target,
            support_target=self.spot_support_projection,
            support_margin=self.config["support_margin"],
            support_clearance_value=self.video_support_clearance,
            shift_completion=self.video_shift_completion,
            support_polygon_valid=self.video_support_valid,
            lift_allowed=self.video_lift_allowed,
            warning=self.video_warning,
        )

    def video_center_step(self, now):
        duration = self.config["stop_center_duration"] * self.gait_time_scale()
        elapsed = max(0.0, now - self.video_state_start_time)
        progress = clamp(elapsed / duration, 0.0, 1.0)
        blend = smootherstep(progress)
        for leg_name in LEG_ORDER:
            origin = self.video_center_feet_origins[leg_name]
            target = self.nominal_foot(leg_name)
            self.feet[leg_name] = tuple(
                origin[index] + (target[index] - origin[index]) * blend
                for index in range(3)
            )
        # This is a deliberate, gradual stop-recovery trajectory. Normal shift
        # and swing stance feet remain world locked; recovery avoids the abrupt
        # nominal reset that would otherwise occur on the next standing tick.
        self.world_feet = {
            leg: self.body_to_world(self.feet[leg])
            for leg in LEG_ORDER
        }
        self.spot_body_shift = tuple(
            value * (1.0 - blend) for value in self.spot_shift_origin
        )
        self.spot_effective_body_offset = tuple(
            self.spot_body_offset_origin[index]
            + (
                self.spot_body_offset_target[index]
                - self.spot_body_offset_origin[index]
            )
            * blend
            for index in range(2)
        )
        self.video_height_offset = self.video_height_origin * (1.0 - blend)
        self.video_shift_completion = 1.0 - progress
        body_motion = self.video_body_motion()
        self.update_video_debug(
            progress,
            [],
            list(LEG_ORDER),
            body_motion,
        )
        if progress < 1.0:
            return copy_feet(self.feet), body_motion, True

        self.feet = self.nominal_feet()
        self.sync_world_feet_from_body()
        self.active = False
        self.settling = False
        self.spot_stop_requested = False
        self.spot_forced_stop = False
        self.video_height_offset = 0.0
        self.spot_body_shift = (0.0, 0.0)
        self.spot_shift_target = (0.0, 0.0)
        self.spot_support_projection = tuple(self.spot_body_offset_target)
        self.spot_effective_body_offset = tuple(self.spot_body_offset_target)
        self.video_step_state = "STOPPED"
        self.spot_phase_name = "stopped"
        self.spot_phase_index = 0
        self.video_shift_completion = 0.0
        self.video_lift_allowed = False
        body_motion = self.video_body_motion()
        self.update_video_debug(
            0.0,
            [],
            list(LEG_ORDER),
            body_motion,
        )
        return copy_feet(self.feet), body_motion, False

    def spotmicro_video_walk_step(
        self,
        now,
        dt,
        velocity,
        step_in_place=False,
        body_offset=(0.0, 0.0),
    ):
        """Run the guarded five-substate/eight-principal-phase video crawl."""
        config = self.config
        try:
            velocity = tuple(float(value) for value in velocity)
            body_offset = tuple(float(value) for value in body_offset)
        except (TypeError, ValueError) as exc:
            raise ValueError("video-walk inputs must be numeric") from exc
        if (
            len(velocity) != 3
            or len(body_offset) != 2
            or not all(math.isfinite(value) for value in velocity + body_offset)
        ):
            raise ValueError("video-walk inputs must contain finite values")

        commanded_motion = (
            math.hypot(velocity[0], velocity[1])
            > config["command_deadband_linear"]
            or abs(velocity[2]) > config["command_deadband_yaw"]
        )
        if not self.active and (commanded_motion or step_in_place):
            self.active = True
            self.settling = False
            self.start_time = now
            self.sync_world_feet_from_body()
            self.reset_spot_walk_state(now)
            self.spot_requested_body_offset = body_offset
            self.spot_effective_body_offset = body_offset
            self.begin_video_shift(now, 0, body_offset)

        if not self.active:
            self.spot_phase_name = "stopped"
            self.video_step_state = "STOPPED"
            self.spot_body_shift = (0.0, 0.0)
            self.spot_shift_target = (0.0, 0.0)
            self.spot_effective_body_offset = body_offset
            self.spot_support_projection = body_offset
            self.video_shift_completion = 0.0
            self.video_support_valid = True
            self.video_lift_allowed = False
            self.video_warning = ""
            body_motion = self.video_body_motion()
            self.update_video_debug(
                0.0,
                [],
                list(LEG_ORDER),
                body_motion,
            )
            return copy_feet(self.feet), body_motion, False

        if commanded_motion or step_in_place:
            if not self.spot_forced_stop and not self.settling:
                self.spot_stop_requested = False
        else:
            self.spot_stop_requested = True

        if self.video_step_state == "CENTERING":
            return self.video_center_step(now)
        if (
            self.spot_stop_requested
            and self.video_step_state in ("SHIFT", "VERIFY_SUPPORT")
        ):
            self.feet = {
                leg: self.world_to_body(self.world_feet[leg])
                for leg in LEG_ORDER
            }
            self.begin_video_centering(now, body_offset)
            return self.video_center_step(now)

        # Once a stop is latched, no residual filtered command advances the
        # body or creates another foothold. An airborne foot still completes
        # its already-frozen Cartesian trajectory.
        integrated_velocity = (
            (0.0, 0.0, 0.0)
            if self.spot_stop_requested
            else velocity
        )
        self.integrate_body_pose(integrated_velocity, dt)
        scale = self.gait_time_scale()
        state_elapsed = max(0.0, now - self.video_state_start_time)
        swing_legs = []
        stance_legs = list(LEG_ORDER)
        phase_progress = self.spot_phase_progress

        if self.video_step_state == "SHIFT":
            duration = config["shift_duration"] * scale
            state_progress = clamp(state_elapsed / duration, 0.0, 1.0)
            blend = smootherstep(state_progress)
            self.spot_body_shift = tuple(
                self.spot_shift_origin[index]
                + (
                    self.spot_shift_target[index]
                    - self.spot_shift_origin[index]
                )
                * blend
                for index in range(2)
            )
            self.spot_effective_body_offset = tuple(
                self.spot_body_offset_origin[index]
                + (
                    self.spot_body_offset_target[index]
                    - self.spot_body_offset_origin[index]
                )
                * blend
                for index in range(2)
            )
            self.video_height_offset = (
                self.video_height_origin
                + (
                    config["gait_body_height_offset"]
                    - self.video_height_origin
                )
                * blend
            )
            target_distance = math.hypot(
                self.spot_shift_target[0] - self.spot_shift_origin[0],
                self.spot_shift_target[1] - self.spot_shift_origin[1],
            )
            remaining = math.hypot(
                self.spot_shift_target[0] - self.spot_body_shift[0],
                self.spot_shift_target[1] - self.spot_body_shift[1],
            )
            self.video_shift_completion = (
                1.0
                if target_distance <= 1e-9
                else clamp(1.0 - remaining / target_distance, 0.0, 1.0)
            )
            (
                self.video_support_valid,
                self.video_support_clearance,
            ) = self.video_validate_support_target(
                self.spot_swing_leg,
                self.spot_support_projection,
            )
            self.video_lift_allowed = False
            self.video_warning = (
                ""
                if self.video_support_valid
                else "Unsafe or degenerate support polygon; refusing foot lift."
            )
            phase_progress = (
                config["shift_duration"] * state_progress
            ) / (
                config["shift_duration"] + config["support_verify_duration"]
            )
            if state_progress >= 1.0:
                self.video_step_state = "VERIFY_SUPPORT"
                self.video_state_start_time = now
                self.video_shift_completion = 1.0

        elif self.video_step_state == "VERIFY_SUPPORT":
            duration = config["support_verify_duration"] * scale
            state_progress = clamp(state_elapsed / duration, 0.0, 1.0)
            self.spot_body_shift = tuple(self.spot_shift_target)
            self.spot_effective_body_offset = tuple(self.spot_body_offset_target)
            self.video_height_offset = config["gait_body_height_offset"]
            self.video_shift_completion = 1.0
            (
                self.video_support_valid,
                self.video_support_clearance,
            ) = self.video_validate_support_target(
                self.spot_swing_leg,
                self.spot_support_projection,
            )
            feedback_ready, feedback_warning = (
                self.video_support_feedback_ready(self.spot_swing_leg)
            )
            self.video_lift_allowed = (
                self.video_shift_completion
                >= config["shift_completion_threshold"]
                and self.video_support_valid
                and feedback_ready
            )
            if not self.video_support_valid:
                self.video_warning = (
                    "Unsafe or degenerate support polygon; refusing foot lift."
                )
            else:
                self.video_warning = feedback_warning
            phase_progress = (
                config["shift_duration"]
                + config["support_verify_duration"] * state_progress
            ) / (
                config["shift_duration"] + config["support_verify_duration"]
            )
            if state_progress >= 1.0 and self.video_lift_allowed:
                self.begin_video_swing(now, integrated_velocity, step_in_place)
                # Preserve one all-stance verification endpoint in diagnostics.
                phase_progress = 1.0

        elif self.video_step_state in ("LIFT_AND_SWING", "TOUCHDOWN"):
            duration = config["swing_duration"] * scale
            swing_progress = clamp(state_elapsed / duration, 0.0, 1.0)
            self.video_step_state = (
                "TOUCHDOWN"
                if swing_progress >= 0.75
                else "LIFT_AND_SWING"
            )
            leg_name = self.spot_swing_leg
            self.world_feet[leg_name] = self.phase_trot_swing_step(
                leg_name,
                swing_progress,
            )
            self.spot_body_shift = tuple(self.spot_shift_target)
            self.spot_effective_body_offset = tuple(self.spot_body_offset_target)
            self.video_height_offset = config["gait_body_height_offset"]
            self.video_shift_completion = 1.0
            self.video_lift_allowed = True
            (
                self.video_support_valid,
                self.video_support_clearance,
            ) = self.video_validate_support_target(
                leg_name,
                self.spot_support_projection,
            )
            self.video_warning = (
                ""
                if self.video_support_valid
                else "Support margin lost during swing; completing touchdown."
            )
            phase_progress = swing_progress
            if swing_progress < 1.0:
                swing_legs = [leg_name]
                stance_legs = [
                    leg for leg in LEG_ORDER if leg != leg_name
                ]
            else:
                self.world_feet[leg_name] = self.swing_targets[leg_name]
                self.begin_video_settle(now)
                swing_legs = []
                stance_legs = list(LEG_ORDER)

        elif self.video_step_state == "SETTLE":
            duration = config["touchdown_settle_duration"] * scale
            state_progress = clamp(state_elapsed / duration, 0.0, 1.0)
            phase_progress = 1.0
            self.spot_body_shift = tuple(self.spot_shift_target)
            self.spot_effective_body_offset = tuple(self.spot_body_offset_target)
            self.video_height_offset = config["gait_body_height_offset"]
            self.video_shift_completion = 1.0
            self.video_lift_allowed = True
            (
                self.video_support_valid,
                self.video_support_clearance,
            ) = self.video_validate_support_target(
                self.spot_swing_leg,
                self.spot_support_projection,
            )
            touchdown_ready, touchdown_warning = (
                self.video_support_feedback_ready(
                    self.spot_swing_leg,
                    touchdown=True,
                )
            )
            self.video_warning = touchdown_warning
            if state_progress >= 1.0 and touchdown_ready:
                self.feet = {
                    leg: self.world_to_body(self.world_feet[leg])
                    for leg in LEG_ORDER
                }
                if self.spot_stop_requested:
                    self.begin_video_centering(now, body_offset)
                    return self.video_center_step(now)
                self.begin_video_shift(
                    now,
                    self.video_leg_slot + 1,
                    body_offset,
                )

        self.feet = {
            leg: self.world_to_body(self.world_feet[leg])
            for leg in LEG_ORDER
        }
        self.spot_phase_progress = clamp(phase_progress, 0.0, 1.0)
        swing_progress = (
            self.spot_phase_progress
            if self.video_step_state in ("LIFT_AND_SWING", "TOUCHDOWN")
            else None
        )
        body_motion = self.video_body_motion(swing_progress)
        self.update_video_debug(
            self.spot_phase_progress,
            swing_legs,
            stance_legs,
            body_motion,
        )
        return copy_feet(self.feet), body_motion, True

    def spot_walk_step(
        self,
        now,
        dt,
        velocity,
        step_in_place=False,
        body_offset=(0.0, 0.0),
    ):
        """Run the eight-phase statically stable Spot Micro-derived crawl."""
        config = self.config
        try:
            body_offset = tuple(float(value) for value in body_offset)
        except (TypeError, ValueError) as exc:
            raise ValueError("body_offset must contain two numeric values") from exc
        if len(body_offset) != 2 or not all(
            math.isfinite(value) for value in body_offset
        ):
            raise ValueError("body_offset must contain two finite values")
        self.spot_requested_body_offset = body_offset
        # Compare one dimensionless activity value instead of applying the
        # same scalar directly to linear m/s and angular rad/s. This also keeps
        # low commands detectable after the conservative hardware speed scale.
        commanded_motion = (
            normalized_velocity_activity(config, velocity)
            > config["velocity_deadband"]
        )

        if not self.active and (commanded_motion or step_in_place):
            self.active = True
            self.settling = False
            self.settle_start_time = None
            self.start_time = now
            self.sync_world_feet_from_body()
            self.reset_spot_walk_state(now)
            self.spot_requested_body_offset = body_offset
            self.spot_effective_body_offset = body_offset
            self.begin_spot_shift(now, 0, velocity, body_offset)

        if not self.active:
            self.update_debug(
                0.0,
                [],
                list(LEG_ORDER),
                phase_index=0,
                phase_name="shift_to_support",
                phase_progress=0.0,
                body_shift=(0.0, 0.0),
                effective_body_offset=body_offset,
                support_projection=body_offset,
            )
            return copy_feet(self.feet), {
                "x": 0.0,
                "y": 0.0,
                "body_x_override": body_offset[0],
                "body_y_override": body_offset[1],
                "height": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
            }, False

        if self.settling:
            return self.spot_settle_step(now)

        if commanded_motion or step_in_place:
            if not self.spot_forced_stop:
                self.spot_stop_requested = False
        else:
            self.spot_stop_requested = True

        # Stance feet stay fixed in this world frame. Transforming them back at
        # the end of the update makes the body advance without foot skating.
        self.integrate_body_pose(velocity, dt)
        phase_elapsed = max(0.0, now - self.spot_phase_start_time)
        reported_phase_index = self.spot_phase_index
        reported_phase_name = self.spot_phase_name

        if self.spot_phase_name == "shift_to_support":
            duration = config["support_shift_duration"]
            progress = clamp(phase_elapsed / duration, 0.0, 1.0)
            (
                self.spot_shift_target,
                self.spot_body_offset_target,
                self.spot_support_projection,
            ) = self.spot_support_plan(
                self.spot_swing_leg,
                velocity,
                body_offset,
            )
            blend = smootherstep(progress)
            self.spot_body_shift = tuple(
                self.spot_shift_origin[index]
                + (
                    self.spot_shift_target[index]
                    - self.spot_shift_origin[index]
                )
                * blend
                for index in range(2)
            )
            self.spot_effective_body_offset = tuple(
                self.spot_body_offset_origin[index]
                + (
                    self.spot_body_offset_target[index]
                    - self.spot_body_offset_origin[index]
                )
                * blend
                for index in range(2)
            )
            swing_legs = []
            stance_legs = list(LEG_ORDER)
            if progress >= 1.0:
                if self.spot_stop_requested:
                    self.feet = {
                        leg: self.world_to_body(self.world_feet[leg])
                        for leg in LEG_ORDER
                    }
                    self.begin_spot_settle(now, body_offset)
                    return self.spot_settle_step(now)
                self.begin_spot_swing(now, velocity, step_in_place)
                # Report one exact, all-stance shift endpoint before classifying
                # the next control sample as airborne.
                progress = 1.0
                swing_legs = []
                stance_legs = list(LEG_ORDER)
        else:
            duration = config["swing_duration"]
            progress = clamp(phase_elapsed / duration, 0.0, 1.0)
            leg_name = self.spot_swing_leg
            self.world_feet[leg_name] = self.phase_trot_swing_step(
                leg_name,
                progress,
            )
            # Follow the slowly changing shrunken support triangle as the body
            # advances, without arbitrary joint/servo offsets.
            (
                safe_shift,
                _safe_body_offset,
                safe_projection,
            ) = self.spot_support_plan(
                leg_name,
                velocity,
                body_offset,
            )
            follow = min(1.0, max(0.0, dt) / 0.10)
            self.spot_body_shift = tuple(
                current + (target - current) * follow
                for current, target in zip(self.spot_body_shift, safe_shift)
            )
            # Constrain the actual body projection applied by IK on every
            # airborne sample.  If the bounded gait shift cannot counter the
            # full operator offset, only its unsafe remainder is clipped.
            self.spot_effective_body_offset = (
                safe_projection[0] - self.spot_body_shift[0],
                safe_projection[1] - self.spot_body_shift[1],
            )
            self.spot_support_projection = safe_projection
            swing_legs = [leg_name] if progress < 1.0 else []
            stance_legs = [
                leg for leg in LEG_ORDER if leg not in swing_legs
            ]
            if progress >= 1.0:
                self.world_feet[leg_name] = self.swing_targets[leg_name]
                self.feet = {
                    leg: self.world_to_body(self.world_feet[leg])
                    for leg in LEG_ORDER
                }
                if self.spot_stop_requested:
                    self.begin_spot_settle(now, body_offset)
                    return self.spot_settle_step(now)
                next_slot = (
                    SPOT_WALK_LEG_ORDER.index(leg_name) + 1
                ) % len(SPOT_WALK_LEG_ORDER)
                self.begin_spot_shift(
                    now,
                    next_slot,
                    velocity,
                    body_offset,
                )
                # Preserve the exact touchdown endpoint in status for this frame.
                progress = 1.0
                swing_legs = []
                stance_legs = list(LEG_ORDER)

        self.feet = {
            leg: self.world_to_body(self.world_feet[leg])
            for leg in LEG_ORDER
        }
        self.spot_phase_progress = progress
        cycle_phase = (
            min(reported_phase_index, 7) + progress
        ) / float(len(SPOT_WALK_LEG_ORDER) * 2)
        body_motion = {
            "x": self.spot_body_shift[0],
            "y": self.spot_body_shift[1],
            "body_x_override": self.spot_effective_body_offset[0],
            "body_y_override": self.spot_effective_body_offset[1],
            "height": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
        }
        self.update_debug(
            cycle_phase,
            swing_legs,
            stance_legs,
            body_motion,
            phase_index=reported_phase_index,
            phase_name=reported_phase_name,
            phase_progress=progress,
            body_shift=self.spot_body_shift,
            effective_body_offset=self.spot_effective_body_offset,
            support_projection=self.spot_support_projection,
        )
        return copy_feet(self.feet), body_motion, True

    def physical_trot_ready_step(self, now):
        """Hold four planted feet while entering the conservative ready pose."""
        if not (self.hardware_mode and self.is_physical_trot()):
            return None
        if self.fast_trot_ready_start_time is None:
            self.fast_trot_ready_start_time = now
        elapsed = max(0.0, now - self.fast_trot_ready_start_time)
        ready_time = self.config["trot_ready_time"]
        if (
            elapsed >= ready_time
            and self.fast_trot_ready_shutdown_start_time is None
        ):
            return None

        target_scale = self.config["startup_stride_fraction"]
        ready_scale = target_scale * smootherstep(
            clamp(elapsed / max(ready_time, 1e-9), 0.0, 1.0)
        )
        if self.stop_requested:
            if self.fast_trot_ready_shutdown_start_time is None:
                self.fast_trot_ready_shutdown_start_time = now
                self.fast_trot_ready_shutdown_scale = ready_scale
            shutdown_progress = clamp(
                (now - self.fast_trot_ready_shutdown_start_time)
                / self.config["shutdown_ramp_time"],
                0.0,
                1.0,
            )
            ready_scale = self.fast_trot_ready_shutdown_scale * (
                1.0 - smootherstep(shutdown_progress)
            )
            if shutdown_progress >= 1.0:
                self.active = False
                self.stop_requested = False
                self.forced_stop = False

        self.fast_trot_startup_scale = ready_scale
        self.fast_trot_input_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_planned_velocity = (0.0, 0.0, 0.0)
        self.fast_trot_requested_stride = 0.0
        self.fast_trot_achieved_stride = 0.0
        self.feet = {
            leg: self.world_to_body(self.world_feet[leg])
            for leg in LEG_ORDER
        }
        body_motion = {
            "height": self.config["body_height_offset"] * ready_scale,
            "roll": 0.0,
            "pitch": 0.0,
        }
        self.update_debug(
            0.0,
            [],
            list(LEG_ORDER),
            body_motion,
            phase_name="trot_ready",
            phase_progress=clamp(elapsed / max(ready_time, 1e-9), 0.0, 1.0),
            step_state="TROT_READY",
        )
        return copy_feet(self.feet), body_motion, self.active

    def phase_trot_step(self, now, dt, velocity, step_in_place=False):
        speed = math.hypot(velocity[0], velocity[1])
        commanded_motion = speed > 0.0015 or abs(velocity[2]) > 0.015

        if not self.active and (commanded_motion or step_in_place):
            self.active = True
            self.settling = False
            self.settle_start_time = None
            self.start_time = now
            self.trot_phase = (
                1.0 - 2e-7
                if self.is_physical_trot()
                else 0.0
            )
            self.current_gait_frequency = self.config["min_gait_frequency"]
            self.was_swinging = {leg: False for leg in LEG_ORDER}
            self.sync_world_feet_from_body()
            if self.is_physical_trot():
                self.fast_trot_start_time = now
                self.reset_fast_trot_runtime()
                self.fast_trot_ready_start_time = now
                self.fast_trot_startup_scale = 0.0
                self.fast_trot_requested_stride = 0.0
                self.fast_trot_achieved_stride = 0.0

        if not self.active:
            self.update_debug(0.0, [], list(LEG_ORDER))
            return copy_feet(self.feet), (0.0, 0.0), False

        if commanded_motion or step_in_place:
            if not self.forced_stop and not self.settling:
                self.stop_requested = False
        else:
            self.stop_requested = True

        ready_result = self.physical_trot_ready_step(now)
        if ready_result is not None:
            return ready_result

        if self.settling:
            return self.settle_feet(now)

        if self.stop_requested and not any(self.was_swinging.values()):
            self.settling = True
            self.settle_start_time = None
            self.settle_origins = copy_feet(self.feet)
            return self.settle_feet(now)

        motion_velocity = velocity
        if self.is_physical_trot():
            motion_velocity = self.plan_fast_trot_motion(
                now,
                velocity,
                step_in_place,
                startup_elapsed=self.fast_trot_startup_elapsed,
            )
            phase_dt = self.fast_trot_governed_dt(dt)
        elif self.hardware_mode and self.config.get("type") == "real_safe_trot":
            phase_dt = self.real_safe_governed_dt(dt)
        else:
            phase_dt = dt
        self.integrate_body_pose(motion_velocity, phase_dt)
        cycle_phase = self.advance_trot_phase(
            phase_dt,
            motion_velocity,
            step_in_place,
        )
        if self.is_physical_trot():
            self.fast_trot_startup_elapsed += phase_dt
            self.record_fast_trot_cycle_timing(now, cycle_phase)
            self.current_gait_frequency = (
                self.fast_trot_phase_rate_scale
                / max(self.fast_trot_cycle_period, 1e-9)
            )
        swing_ratio = self.config["swing_ratio"]
        swing_legs = []
        stance_legs = []
        for leg_name in LEG_ORDER:
            # Pair A (front_left + rear_right) uses phase offset 0.0. Pair B
            # (front_right + rear_left) uses 0.5, exactly 180 degrees later.
            leg_phase = (
                cycle_phase + TROT_PHASE_OFFSETS[leg_name]
            ) % 1.0
            scheduled_swing = leg_phase < swing_ratio
            # A stop may finish legs that were already airborne, but it may not
            # initiate the next diagonal pair.
            swinging = scheduled_swing and (
                not self.stop_requested or self.was_swinging[leg_name]
            )

            if swinging:
                if not self.was_swinging[leg_name]:
                    self.swing_origins[leg_name] = self.world_feet[leg_name]
                    self.swing_targets[leg_name] = self.trot_touchdown_world(
                        leg_name,
                        motion_velocity,
                    )
                    self.swing_heights[leg_name] = self.adaptive_step_height(
                        motion_velocity,
                        step_in_place,
                    )
                swing = leg_phase / swing_ratio
                self.world_feet[leg_name] = self.phase_trot_swing_step(
                    leg_name,
                    swing,
                )
                swing_legs.append(leg_name)
            else:
                if self.was_swinging[leg_name]:
                    self.world_feet[leg_name] = self.swing_targets[leg_name]

                # Stance phase foot locking: do not animate the planted foot in
                # the body frame. Keep its world coordinate fixed; as the body
                # integrates forward, world_to_body() makes the body move over
                # the planted support point without visible ground sliding.
                stance_legs.append(leg_name)
            self.was_swinging[leg_name] = swinging

        self.feet = {
            leg: self.world_to_body(self.world_feet[leg])
            for leg in LEG_ORDER
        }
        body_motion = self.body_motion(cycle_phase, motion_velocity)
        self.update_debug(cycle_phase, swing_legs, stance_legs, body_motion)
        if self.stop_requested and not swing_legs:
            self.settling = True
            self.settle_start_time = None
            self.settle_origins = copy_feet(self.feet)
        return copy_feet(self.feet), body_motion, True

    def legacy_step(self, now, dt, velocity, step_in_place=False):
        speed = math.hypot(velocity[0], velocity[1])
        commanded_motion = speed > 0.0015 or abs(velocity[2]) > 0.015

        if not self.active and (commanded_motion or step_in_place):
            self.active = True
            self.settling = False
            self.settled_legs.clear()
            self.start_time = now
            self.crab_mode = abs(velocity[1]) > 0.002
            self.was_swinging = {leg: False for leg in LEG_ORDER}

        if not self.active:
            self.update_debug(0.0, [], list(LEG_ORDER))
            return copy_feet(self.feet), (0.0, 0.0), False

        if commanded_motion or step_in_place:
            if not self.forced_stop and not self.settling:
                self.stop_requested = False
        else:
            self.stop_requested = True

        if self.settling:
            return self.settle_feet(now)

        if self.stop_requested and not any(self.was_swinging.values()):
            self.settling = True
            self.settle_start_time = now
            self.settle_origins = copy_feet(self.feet)
            return self.settle_feet(now)

        phase = self.phase(now)
        swing_legs = []
        stance_legs = []

        for leg_name in LEG_ORDER:
            progress = self.swing_progress(leg_name, phase)
            scheduled_swing = progress is not None
            swinging = scheduled_swing and (
                not self.stop_requested or self.was_swinging[leg_name]
            )

            if swinging:
                if not self.was_swinging[leg_name]:
                    self.swing_origins[leg_name] = self.feet[leg_name]
                swing_velocity = (
                    (0.0, 0.0, 0.0)
                    if self.stop_requested
                    else velocity
                )
                self.feet[leg_name] = self.swing_step(
                    leg_name,
                    progress,
                    swing_velocity,
                )
                swing_legs.append(leg_name)
            else:
                if self.was_swinging[leg_name]:
                    touchdown_velocity = (
                        (0.0, 0.0, 0.0)
                        if self.stop_requested
                        else velocity
                    )
                    self.feet[leg_name] = self.touchdown_target(
                        leg_name,
                        touchdown_velocity,
                    )
                self.feet[leg_name] = self.stance_step(
                    self.feet[leg_name],
                    velocity,
                    dt,
                )
                stance_legs.append(leg_name)

            self.was_swinging[leg_name] = swinging

        self.update_debug(phase / self.config["period"], swing_legs, stance_legs)

        if self.stop_requested and not swing_legs:
            self.settling = True
            self.settle_start_time = now
            self.settle_origins = copy_feet(self.feet)

        return copy_feet(self.feet), self.support_shift(phase), True

    def step(
        self,
        now,
        dt,
        velocity,
        step_in_place=False,
        body_offset=(0.0, 0.0),
    ):
        if self.is_video_walk():
            return self.spotmicro_video_walk_step(
                now,
                dt,
                velocity,
                step_in_place,
                body_offset,
            )
        if self.is_spot_walk():
            return self.spot_walk_step(
                now,
                dt,
                velocity,
                step_in_place,
                body_offset,
            )
        if self.is_phase_trot():
            return self.phase_trot_step(now, dt, velocity, step_in_place)
        return self.legacy_step(now, dt, velocity, step_in_place)

    def update_debug(
        self,
        phase,
        swing_legs,
        stance_legs,
        body_motion=None,
        phase_index=0,
        phase_name="cycle",
        phase_progress=None,
        body_shift=(0.0, 0.0),
        effective_body_offset=(0.0, 0.0),
        support_projection=(0.0, 0.0),
        step_state="CYCLE",
        body_shift_target=(0.0, 0.0),
        support_target=(0.0, 0.0),
        support_margin=0.0,
        support_clearance_value=0.0,
        shift_completion=0.0,
        support_polygon_valid=True,
        lift_allowed=False,
        warning="",
    ):
        if body_motion is None:
            body_motion = {
                "height": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
            }
        if phase_progress is None:
            phase_progress = phase % 1.0
        if self.is_physical_trot():
            requested_stride = float(self.fast_trot_requested_stride)
            achieved_stride = float(self.fast_trot_achieved_stride)
            cycle_period = float(self.fast_trot_cycle_period)
            observed_cycle_period = float(
                self.fast_trot_observed_cycle_period
            )
            tuning = dict(self.fast_trot_tuning)
            backend_stride_scale = self.fast_trot_backend_stride_scale()
            input_velocity = list(self.fast_trot_input_velocity)
            planned_velocity = list(self.fast_trot_planned_velocity)
            startup_scale = float(self.fast_trot_startup_scale)
            phase_rate_scale = float(self.fast_trot_phase_rate_scale)
            transition_hold = (
                self.fast_trot_transition_hold_phase is not None
            )
        else:
            requested_stride = 0.0
            achieved_stride = 0.0
            cycle_period = (
                1.0 / self.current_gait_frequency
                if self.current_gait_frequency > 1e-9
                else float(self.config.get("period", 0.0))
            )
            observed_cycle_period = cycle_period
            tuning = dict(self.config.get("real_tuning", {}))
            backend_stride_scale = 1.0
            input_velocity = [0.0, 0.0, 0.0]
            planned_velocity = [0.0, 0.0, 0.0]
            startup_scale = 0.0
            phase_rate_scale = (
                float(getattr(self, "real_safe_phase_rate_scale", 1.0))
                if self.config.get("type") == "real_safe_trot"
                else 1.0
            )
            transition_hold = False
        swing_pair = "+".join(swing_legs) if swing_legs else "none"
        per_leg_phase = (
            {
                leg: (
                    (phase + TROT_PHASE_OFFSETS[leg]) % 1.0
                )
                for leg in LEG_ORDER
            }
            if self.is_phase_trot()
            else {}
        )
        self.debug_state = {
            "phase": phase % 1.0,
            "phase_index": int(phase_index),
            "phase_name": str(phase_name),
            "phase_progress": clamp(phase_progress, 0.0, 1.0),
            "swing_legs": list(swing_legs),
            "stance_legs": list(stance_legs),
            "body_shift": {
                "x": float(body_shift[0]),
                "y": float(body_shift[1]),
            },
            "effective_body_offset": {
                "x": float(effective_body_offset[0]),
                "y": float(effective_body_offset[1]),
            },
            "support_projection": {
                "x": float(support_projection[0]),
                "y": float(support_projection[1]),
            },
            "body_shift_x": float(body_shift[0]),
            "body_shift_y": float(body_shift[1]),
            "body_shift_target_x": float(body_shift_target[0]),
            "body_shift_target_y": float(body_shift_target[1]),
            "support_target_x": float(support_target[0]),
            "support_target_y": float(support_target[1]),
            "support_margin": float(support_margin),
            "support_clearance": float(support_clearance_value),
            "shift_completion": clamp(shift_completion, 0.0, 1.0),
            "support_polygon_valid": bool(support_polygon_valid),
            "lift_allowed": bool(lift_allowed),
            "step_state": str(step_state),
            "warning": str(warning),
            "feet": copy_feet(self.feet),
            "body_motion": dict(body_motion),
            "cadence": self.current_gait_frequency,
            "cycle_period": cycle_period,
            "configured_cycle_period": cycle_period,
            "current_cycle_period": (
                observed_cycle_period
                if observed_cycle_period > 0.0
                else cycle_period
            ),
            "phase_rate_scale": phase_rate_scale,
            "phase_transition_hold": transition_hold,
            "current_swing_pair": swing_pair,
            "per_leg_phase": per_leg_phase,
            "requested_stride": requested_stride,
            "achieved_stride": achieved_stride,
            "stride_achievement_ratio": (
                achieved_stride / requested_stride
                if requested_stride > 1e-9
                else 1.0
            ),
            "fast_trot_tuning": tuning,
            "backend_stride_scale": backend_stride_scale,
            "startup_scale": startup_scale,
            "input_velocity": input_velocity,
            "planned_velocity": planned_velocity,
            "body_world": {
                "x": float(self.body_x_world),
                "y": float(self.body_y_world),
                "yaw": float(self.body_yaw_world),
            },
        }

    def debug_snapshot(self):
        return dict(self.debug_state)


NOMINAL_FEET_Z = next(iter(NOMINAL_FEET.values()))[2]
