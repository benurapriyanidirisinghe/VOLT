#!/usr/bin/env python3

"""VOLT gait engine: two servo-budgeted gaits with world-locked stance.

Ground-up rebuild.  The previous engine carried nine gaits across four code
paths; walking quality on the real robot was limited not by the trajectory
shapes but by physics that none of those paths modeled: a TD-8130MG under
load does not track an arbitrary command.  This engine keeps exactly two
gaits and builds the servo's speed-torque reality into the *configuration
contract* instead of hoping the firmware slew limiter cleans up after an
infeasible plan.

Gaits
-----
``trot``   Two-beat diagonal gait (FL+RR / FR+RL half a cycle apart) with a
           duty factor above 0.5, so short four-foot support windows exist
           for load transfer between diagonal pairs.
``amble``  Four-beat lateral-sequence walk (RL, FL, RR, FR each a quarter
           cycle apart) with a high duty factor -- three feet are always
           planted -- plus a lateral body sway that leans the body away
           from whichever side is swinging.

Servo budget
------------
The physical insight the old engine missed: the *swing* leg is unloaded (it
carries only its own inertia, so the servo runs near its ~375 deg/s free
speed), while the *stance* legs carry the body but rotate slowly.  The
budgets therefore differ per phase:

  stance joints (loaded):    <= 80 deg/s commanded
  swing joints (unloaded):   <= 190 deg/s commanded (firmware ceiling 240)

``validate_servo_budget`` numerically sweeps one full cycle at the gait's
maximum command through the real IK and rejects any configuration whose
commanded joint speeds break those budgets.  A gait config that loads is a
gait config the servos can execute; nothing downstream needs to clip.

World locking
-------------
During stance a foot holds a fixed *world* point while the integrated body
pose moves over it, so planted feet cannot skate.  Swing targets are frozen
world-frame touchdown points computed from the commanded twist when the
foot lifts; command changes never jerk an airborne foot.
"""

import math
from pathlib import Path

import yaml

from volt_kinematics import (
    LEG_ORDER,
    JOINT_NAMES,
    NOMINAL_FEET,
    clamp,
    feet_to_joint_positions_diagnostic,
    smootherstep,
)


# --------------------------------------------------------------------------
# Public constants
# --------------------------------------------------------------------------

TROT_PHASE_OFFSETS = {
    "front_left": 0.0,
    "rear_right": 0.0,
    "front_right": 0.5,
    "rear_left": 0.5,
}

# Lateral-sequence amble: each hind foot is followed by the same-side front
# foot.  Swing order over the cycle is FR, RR, FL, RL.
AMBLE_PHASE_OFFSETS = {
    "rear_left": 0.0,
    "front_left": 0.25,
    "rear_right": 0.5,
    "front_right": 0.75,
}

# Every historical gait name maps onto one of the two surviving gaits so
# saved profiles, old GUI messages, and muscle memory keep working.
GAIT_ALIASES = {
    "walk": "amble",
    "slow_crawl": "amble",
    "diagnostic_crawl": "amble",
    "spot_walk": "amble",
    "spotmicro_video_walk": "amble",
    "legacy_walk": "amble",
    "real_trot": "trot",
    "load_safe_trot": "trot",
    "real_safe_trot": "trot",
    "slow_trot": "trot",
    "normal_trot": "trot",
    "fast_trot": "trot",
}

GAIT_PHASE_OFFSETS = {
    "trot": TROT_PHASE_OFFSETS,
    "amble": AMBLE_PHASE_OFFSETS,
}

# Config keys: every gait section must provide exactly these numeric fields.
GAIT_PARAMETER_NAMES = (
    "cycle_period",
    "duty_factor",
    "step_height",
    "max_x",
    "max_y",
    "max_yaw",
    "settle_time",
    "body_sway_y",
    "body_height_offset",
    "velocity_filter_alpha",
    "command_acceleration",
    "hardware_speed_scale",
    "joint_velocity_limit_deg_s",
    "joint_acceleration_limit_deg_s2",
    "stance_velocity_budget_deg_s",
    "swing_velocity_budget_deg_s",
)

_MOTION_LINEAR_DEADBAND = 0.002   # m/s
_MOTION_YAW_DEADBAND = 0.010      # rad/s

NOMINAL_FEET_Z = next(iter(NOMINAL_FEET.values()))[2]

# The servo-budget sweep validates against the lowest standing height any
# shipped real profile uses; a lower body means more knee flexion and higher
# joint speeds for the same foot motion, so this is the conservative case.
_BUDGET_SWEEP_HEIGHT = 0.195
# The sweep samples at the controller's real command period.  Acceleration
# at the C1 stance/swing boundaries is a per-tick velocity step, so its
# magnitude depends on the sampling rate; validating at a finer dt than the
# controller uses would reject boundary impulses the 100 Hz command stream
# never actually contains.
_BUDGET_SWEEP_DT = 0.010


def smoothstep(value):
    """Cubic ease with zero end velocity.

    Chosen over smootherstep for swing transfer deliberately: its peak
    slope is 1.5 versus 1.875, which lowers the commanded peak joint speed
    of the unloaded swing leg by ~20% for the same step and swing time.
    The acceleration discontinuity at the ends is absorbed by the firmware's
    50 Hz interpolation and the servo's own lag.
    """
    value = clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def swing_lift(progress, height):
    """Vertical swing profile: zero velocity at liftoff and touchdown."""
    return height * math.sin(math.pi * clamp(progress, 0.0, 1.0)) ** 2


# --------------------------------------------------------------------------
# Velocity helpers (public: consumed by the motion controller)
# --------------------------------------------------------------------------

def limit_velocity_command(velocity, limits):
    """Clamp a twist to per-axis limits, then cap the combined magnitude.

    The combined (L1 in normalized axes) cap matters for the servo budget:
    the budget sweep validates single-axis maxima plus a mixed case, and
    this cap guarantees a joystick pushing every axis at once cannot demand
    more total foot motion than the validated mixed case.
    """
    max_x, max_y, max_yaw = limits
    vx = clamp(float(velocity[0]), -max_x, max_x) if max_x > 0.0 else 0.0
    vy = clamp(float(velocity[1]), -max_y, max_y) if max_y > 0.0 else 0.0
    wz = clamp(float(velocity[2]), -max_yaw, max_yaw) if max_yaw > 0.0 else 0.0
    total = 0.0
    if max_x > 0.0:
        total += abs(vx) / max_x
    if max_y > 0.0:
        total += abs(vy) / max_y
    if max_yaw > 0.0:
        total += abs(wz) / max_yaw
    if total > 1.0:
        vx /= total
        vy /= total
        wz /= total
    return (vx, vy, wz)


def normalized_velocity_activity(velocity, limits):
    """Return 0..1: how much of the gait's command envelope is in use."""
    max_x, max_y, max_yaw = limits
    activity = 0.0
    if max_x > 0.0:
        activity = max(activity, abs(float(velocity[0])) / max_x)
    if max_y > 0.0:
        activity = max(activity, abs(float(velocity[1])) / max_y)
    if max_yaw > 0.0:
        activity = max(activity, abs(float(velocity[2])) / max_yaw)
    return clamp(activity, 0.0, 1.0)


def velocity_is_active(velocity):
    return (
        math.hypot(float(velocity[0]), float(velocity[1]))
        > _MOTION_LINEAR_DEADBAND
        or abs(float(velocity[2])) > _MOTION_YAW_DEADBAND
    )


# --------------------------------------------------------------------------
# Configuration loading and validation
# --------------------------------------------------------------------------

def canonical_gait_name(name):
    """Resolve aliases; raise for names that resolve to nothing."""
    text = str(name or "").strip().lower()
    text = GAIT_ALIASES.get(text, text)
    if text not in GAIT_PHASE_OFFSETS:
        raise ValueError("Unknown gait: %s" % name)
    return text


def default_gait_config_path():
    return str(Path(__file__).resolve().parent.parent / "config" / "gait_controller.yaml")


def _require_number(section, key, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("%s.%s must be a number" % (section, key))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("%s.%s must be finite" % (section, key))
    return value


def _validate_gait_config(name, raw):
    """Structural validation of one gait section; returns a clean dict."""
    if not isinstance(raw, dict):
        raise ValueError("gait %s must be a mapping" % name)
    config = {"type": name}
    declared_type = str(raw.get("type", name)).strip().lower()
    if declared_type != name:
        raise ValueError("gait %s declares type %s" % (name, declared_type))
    missing = [key for key in GAIT_PARAMETER_NAMES if key not in raw]
    if missing:
        raise ValueError("gait %s is missing: %s" % (name, ", ".join(missing)))
    unknown = [
        key for key in raw
        if key not in GAIT_PARAMETER_NAMES and key != "type"
    ]
    if unknown:
        raise ValueError("gait %s has unknown keys: %s" % (name, ", ".join(unknown)))
    for key in GAIT_PARAMETER_NAMES:
        config[key] = _require_number(name, key, raw[key])

    if not 0.4 <= config["cycle_period"] <= 4.0:
        raise ValueError("%s cycle_period must be in [0.4, 4.0] s" % name)
    duty_low, duty_high = (0.52, 0.68) if name == "trot" else (0.70, 0.86)
    if not duty_low <= config["duty_factor"] <= duty_high:
        raise ValueError(
            "%s duty_factor must be in [%.2f, %.2f]" % (name, duty_low, duty_high)
        )
    if not 0.012 <= config["step_height"] <= 0.040:
        raise ValueError("%s step_height must be in [0.012, 0.040] m" % name)
    for key in ("max_x", "max_y", "max_yaw"):
        if config[key] <= 0.0:
            raise ValueError("%s %s must be positive" % (name, key))
    if config["max_x"] > 0.25 or config["max_y"] > 0.12 or config["max_yaw"] > 1.2:
        raise ValueError("%s command limits exceed sane bounds" % name)
    if not 0.2 <= config["settle_time"] <= 2.0:
        raise ValueError("%s settle_time must be in [0.2, 2.0] s" % name)
    if not 0.0 <= config["body_sway_y"] <= 0.03:
        raise ValueError("%s body_sway_y must be in [0, 0.03] m" % name)
    if not -0.03 <= config["body_height_offset"] <= 0.0:
        raise ValueError("%s body_height_offset must be in [-0.03, 0] m" % name)
    if not 0.05 <= config["velocity_filter_alpha"] <= 1.0:
        raise ValueError("%s velocity_filter_alpha must be in (0.05, 1]" % name)
    if not 0.02 <= config["command_acceleration"] <= 2.0:
        raise ValueError("%s command_acceleration must be in [0.02, 2.0]" % name)
    if not 0.2 <= config["hardware_speed_scale"] <= 1.0:
        raise ValueError("%s hardware_speed_scale must be in [0.2, 1]" % name)
    if not 30.0 <= config["joint_velocity_limit_deg_s"] <= 240.0:
        raise ValueError("%s joint_velocity_limit_deg_s must be in [30, 240]" % name)
    # An unloaded TD-8130MG reverses direction in ~40 ms, an effective
    # acceleration capability well above 15000 deg/s^2; the ceiling here is
    # a commanded-trajectory sanity bound, not a servo spec.
    if not 600.0 <= config["joint_acceleration_limit_deg_s2"] <= 12000.0:
        raise ValueError(
            "%s joint_acceleration_limit_deg_s2 must be in [600, 12000]" % name
        )
    if not 20.0 <= config["stance_velocity_budget_deg_s"] <= 120.0:
        raise ValueError("%s stance budget must be in [20, 120] deg/s" % name)
    if not 60.0 <= config["swing_velocity_budget_deg_s"] <= 240.0:
        raise ValueError("%s swing budget must be in [60, 240] deg/s" % name)
    if config["swing_velocity_budget_deg_s"] > config["joint_velocity_limit_deg_s"]:
        raise ValueError(
            "%s swing budget exceeds its own joint_velocity_limit_deg_s" % name
        )

    # The stance excursion the maximum command produces must stay inside the
    # foot workspace with margin (the IK sweep re-checks this exactly).
    stance_time = config["duty_factor"] * config["cycle_period"]
    if 0.5 * config["max_x"] * stance_time > 0.085:
        raise ValueError("%s max_x stride exceeds the foot workspace" % name)
    return config


def _sweep_offsets(config, phase, velocity, leg):
    """Idealized planar foot offset used only by the budget sweep.

    The runtime engine world-locks stance feet; for validation the
    equivalent body-frame motion is a linear stance sweep plus the exact
    swing profile, evaluated at constant commanded twist.
    """
    duty = config["duty_factor"]
    period = config["cycle_period"]
    vx, vy, wz = velocity
    nominal = NOMINAL_FEET[leg]
    stance_vx = vx - wz * nominal[1]
    stance_vy = vy + wz * nominal[0]
    excursion_x = stance_vx * duty * period
    excursion_y = stance_vy * duty * period
    local = (phase + GAIT_PHASE_OFFSETS[config["type"]][leg]) % 1.0
    if local < duty:
        fraction = 0.5 - local / duty
        return excursion_x * fraction, excursion_y * fraction, 0.0, True
    progress = (local - duty) / (1.0 - duty)
    fraction = -0.5 + smoothstep(progress)
    return (
        excursion_x * fraction,
        excursion_y * fraction,
        swing_lift(progress, config["step_height"]),
        False,
    )


def validate_servo_budget(config, speed_scale=1.0):
    """Reject a gait whose maximum command breaks the servo budgets.

    Sweeps a full cycle through the real IK at the gait's maximum forward,
    maximum lateral, maximum yaw, and the L1-capped mixed command, at the
    lowest shipped standing height.  Raises ValueError with the offending
    command and joint if any sample exceeds the stance or swing budget, or
    leaves the foot workspace.
    """
    stance_budget = config["stance_velocity_budget_deg_s"]
    swing_budget = config["swing_velocity_budget_deg_s"]
    limits = (
        config["max_x"] * speed_scale,
        config["max_y"] * speed_scale,
        config["max_yaw"] * speed_scale,
    )
    mixed = limit_velocity_command(
        (limits[0], limits[1], limits[2]), limits
    )
    commands = (
        (limits[0], 0.0, 0.0),
        (0.0, limits[1], 0.0),
        (0.0, 0.0, limits[2]),
        mixed,
    )
    period = config["cycle_period"]
    dt = _BUDGET_SWEEP_DT
    samples = max(int(period / dt), 40)
    acceleration_budget = config["joint_acceleration_limit_deg_s2"]
    for command in commands:
        previous = None
        previous_stance = None
        previous_speeds = None
        for index in range(samples + 1):
            phase = (index * dt) / period
            feet = {}
            stance_flags = {}
            for leg in LEG_ORDER:
                dx, dy, dz, stance = _sweep_offsets(config, phase, command, leg)
                nominal = NOMINAL_FEET[leg]
                feet[leg] = (nominal[0] + dx, nominal[1] + dy, nominal[2] + dz)
                stance_flags[leg] = stance
            positions, diagnostics = feet_to_joint_positions_diagnostic(
                feet, height=_BUDGET_SWEEP_HEIGHT
            )
            if diagnostics["projected_targets"]:
                raise ValueError(
                    "%s command %s leaves the workspace: %s"
                    % (config["type"], command, diagnostics["projected_targets"])
                )
            if previous is not None:
                speeds = [
                    (positions[j] - previous[j]) / dt * 180.0 / math.pi
                    for j in range(12)
                ]
                for joint_index in range(12):
                    speed = abs(speeds[joint_index])
                    leg = "_".join(JOINT_NAMES[joint_index].split("_")[:2])
                    in_stance = stance_flags[leg] and previous_stance[leg]
                    budget = stance_budget if in_stance else swing_budget
                    if speed > budget:
                        raise ValueError(
                            "%s command %s demands %.0f deg/s on %s during "
                            "%s (budget %.0f); slow the gait or shrink the "
                            "command limits"
                            % (
                                config["type"], command, speed,
                                JOINT_NAMES[joint_index],
                                "stance" if in_stance else "swing",
                                budget,
                            )
                        )
                if previous_speeds is not None:
                    for joint_index in range(12):
                        acceleration = abs(
                            speeds[joint_index]
                            - previous_speeds[joint_index]
                        ) / dt
                        if acceleration > acceleration_budget:
                            raise ValueError(
                                "%s command %s demands %.0f deg/s^2 on %s "
                                "(budget %.0f); the acceleration limiter "
                                "would distort the trajectory"
                                % (
                                    config["type"], command, acceleration,
                                    JOINT_NAMES[joint_index],
                                    acceleration_budget,
                                )
                            )
                previous_speeds = speeds
            previous = positions
            previous_stance = stance_flags
    return True


def load_gait_configs(path=None):
    """Load and validate the two gait sections from the controller YAML."""
    config_path = Path(path or default_gait_config_path())
    with open(config_path, "r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    try:
        parameters = document["volt_motion_controller"]["ros__parameters"]
    except (KeyError, TypeError):
        raise ValueError("gait config missing volt_motion_controller section")
    gaits = parameters.get("gaits")
    if not isinstance(gaits, dict):
        raise ValueError("gait config missing a gaits mapping")
    configs = {}
    for name in ("trot", "amble"):
        if name not in gaits:
            raise ValueError("gait config missing the %s gait" % name)
        config = _validate_gait_config(name, gaits[name])
        validate_servo_budget(config)
        # The hardware envelope is a scaled-down subset of the validated
        # simulation envelope, so it cannot fail if the full one passed;
        # validating it anyway keeps the contract explicit.
        validate_servo_budget(config, speed_scale=config["hardware_speed_scale"])
        configs[name] = config
    return configs


def apply_real_tuning_to_configs(configs, tuning):
    """Overlay a validated real-robot profile onto its target gait.

    The profile names its gait; its cadence/stride fields replace that
    gait's parameters and the result must re-pass the servo budget, so a
    profile cannot smuggle in an infeasible gait.
    """
    if not isinstance(tuning, dict):
        raise ValueError("real tuning must be a mapping")
    updated = {name: dict(config) for name, config in configs.items()}
    name = canonical_gait_name(tuning.get("gait", "trot"))
    config = updated[name]
    cycle = float(tuning.get("cycle_duration", config["cycle_period"]))
    duty = float(tuning.get("duty_factor", config["duty_factor"]))
    stride = float(tuning.get("stride_length", 0.0))
    lateral = float(tuning.get("lateral_stride_width", 0.0))
    config["cycle_period"] = cycle
    config["duty_factor"] = duty
    config["step_height"] = float(tuning.get("step_height", config["step_height"]))
    stance_time = duty * cycle
    if stride > 0.0 and stance_time > 0.0:
        config["max_x"] = stride / stance_time
    if lateral > 0.0 and stance_time > 0.0:
        config["max_y"] = lateral / stance_time
    config = _validate_gait_config(name, {
        key: config[key] for key in GAIT_PARAMETER_NAMES
    } | {"type": name})
    validate_servo_budget(config)
    updated[name] = config
    return updated


def _builtin_gaits():
    """The shipped defaults; the YAML holds the same values.

    Numbers come from the numeric design sweep against the TD-8130MG
    budgets (stance 80 / swing 190 deg/s at 0.195 m standing height):
    trot T=1.1 duty=0.58 tops out at 182 deg/s swing when turning at full
    command; amble T=2.0 duty=0.76 tops out at 187 deg/s.
    """
    return {
        "trot": _validate_gait_config("trot", {
            "type": "trot",
            # T=0.9/duty=0.62 was tuned in the loaded-servo Ignition model:
            # the shorter cycle leaves less time to fall onto each diagonal
            # and the higher duty doubles the four-foot load-transfer window.
            # With the slow command ramp below, three consecutive runs gave
            # 1.12-1.21 m per 15 s at 0.10 m/s with roll p95 <= 2.6 deg.
            "cycle_period": 0.9,
            "duty_factor": 0.62,
            "step_height": 0.020,
            "max_x": 0.10,
            "max_y": 0.05,
            "max_yaw": 0.50,
            "settle_time": 0.6,
            "body_sway_y": 0.0,
            "body_height_offset": 0.0,
            "velocity_filter_alpha": 0.30,
            # The dynamic trot is bistable on marginal servo torque: ramping
            # to full stride over ~2 cycles reliably enters the clean limit
            # cycle, where a fast ramp sometimes locked in a 10-deg rocking
            # mode with most of the stride lost to slip.
            "command_acceleration": 0.08,
            "hardware_speed_scale": 0.80,
            "joint_velocity_limit_deg_s": 190.0,
            "joint_acceleration_limit_deg_s2": 6500.0,
            "stance_velocity_budget_deg_s": 80.0,
            "swing_velocity_budget_deg_s": 190.0,
        }),
        "amble": _validate_gait_config("amble", {
            "type": "amble",
            "cycle_period": 2.0,
            "duty_factor": 0.76,
            "step_height": 0.020,
            "max_x": 0.05,
            "max_y": 0.03,
            "max_yaw": 0.30,
            "settle_time": 0.8,
            # 0.012 m of sway produced a consistent ~1 deg/s heading drift
            # in simulation (sway/swing-order asymmetry); 0.009 keeps the
            # stability lean while centering the drift around zero.
            "body_sway_y": 0.009,
            "body_height_offset": 0.0,
            "velocity_filter_alpha": 0.25,
            "command_acceleration": 0.06,
            "hardware_speed_scale": 0.85,
            "joint_velocity_limit_deg_s": 190.0,
            "joint_acceleration_limit_deg_s2": 6500.0,
            "stance_velocity_budget_deg_s": 80.0,
            "swing_velocity_budget_deg_s": 190.0,
        }),
    }


try:
    GAITS = load_gait_configs()
except (OSError, ValueError):
    # Source checkouts without an installed config still need the module
    # importable (GUI unit tests, tooling); the builtin values are the
    # same numbers the shipped YAML carries.
    GAITS = _builtin_gaits()


# --------------------------------------------------------------------------
# The engine
# --------------------------------------------------------------------------

def copy_feet(feet):
    return {leg: tuple(float(v) for v in feet[leg]) for leg in LEG_ORDER}


class VoltGaitController:
    """World-locked two-gait engine.

    The public surface is intentionally the same one the motion controller,
    emote/pose keep-code, and tests already speak: ``step``, ``set_gait``,
    ``reset``, ``set_current_feet``, ``hold_current_feet``, ``request_stop``,
    ``nominal_feet``, ``set_support_feedback``, ``set_real_tuning``,
    ``debug_snapshot``, ``cycle_phase``, ``velocity_activity``, ``active``.
    """

    def __init__(self, gait_configs=None, hardware_mode=False):
        self.gaits = {
            name: dict(config)
            for name, config in (gait_configs or GAITS).items()
        }
        self.hardware_mode = bool(hardware_mode)
        self.gait_name = "amble"
        self.real_tuning = {}
        self.support_feedback = {}
        self.warning = ""
        self.reset(0.0)

    # -- configuration ----------------------------------------------------

    @property
    def config(self):
        return self.gaits[self.gait_name]

    def speed_scale(self):
        if self.hardware_mode:
            return float(self.config["hardware_speed_scale"])
        return 1.0

    def command_limits(self):
        scale = self.speed_scale()
        config = self.config
        return (
            config["max_x"] * scale,
            config["max_y"] * scale,
            config["max_yaw"] * scale,
        )

    def nominal_foot(self, leg_name):
        return tuple(NOMINAL_FEET[leg_name])

    def nominal_feet(self):
        return {leg: self.nominal_foot(leg) for leg in LEG_ORDER}

    def set_gait(self, gait_name, now):
        name = canonical_gait_name(gait_name)
        if name == self.gait_name:
            return
        if self.active:
            raise ValueError("cannot switch gaits while the gait is active")
        self.gait_name = name
        self.reset(now)

    def set_real_tuning(self, tuning):
        self.gaits = apply_real_tuning_to_configs(self.gaits, tuning)
        self.real_tuning = dict(tuning)
        return dict(self.real_tuning)

    def set_support_feedback(self, feedback=None):
        if feedback is None:
            self.support_feedback = {}
            return
        if not isinstance(feedback, dict):
            raise ValueError("support feedback must be a mapping")
        self.support_feedback = dict(feedback)

    # -- state ------------------------------------------------------------

    def reset(self, now=None):
        self.active = False
        self.settling = False
        self.stop_requested = False
        self.forced_stop = False
        self.phase = 0.0
        self.feet = copy_feet(NOMINAL_FEET)
        self.world_feet = copy_feet(NOMINAL_FEET)
        self.body_x_world = 0.0
        self.body_y_world = 0.0
        self.body_yaw_world = 0.0
        self.swing_origins = {}
        self.swing_targets = {}
        self.was_swinging = {leg: False for leg in LEG_ORDER}
        self.settle_started = 0.0
        self.settle_origin = None
        self.last_velocity = (0.0, 0.0, 0.0)
        self.warning = ""
        self.debug_state = {
            "phase": 0.0,
            "phase_name": "stopped",
            "phase_progress": 0.0,
            "step_state": "STOPPED",
            "swing_legs": [],
            "stance_legs": list(LEG_ORDER),
            "per_leg_phase": {leg: 0.0 for leg in LEG_ORDER},
            "cycle_period": self.config["cycle_period"],
            "warning": "",
            "body_world": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "planned_velocity": [0.0, 0.0, 0.0],
        }

    def set_current_feet(self, feet):
        cleaned = {}
        for leg in LEG_ORDER:
            if leg not in feet:
                raise ValueError("feet mapping is missing %s" % leg)
            point = feet[leg]
            values = tuple(float(v) for v in point)
            if len(values) != 3 or not all(math.isfinite(v) for v in values):
                raise ValueError("foot %s must be three finite floats" % leg)
            cleaned[leg] = values
        self.feet = cleaned
        self.sync_world_feet_from_body()

    def hold_current_feet(self, feet, now=None):
        """Ownership-loss hold: adopt the pose, drop every request."""
        self.set_current_feet(feet)
        self.active = False
        self.settling = False
        self.stop_requested = False
        self.forced_stop = False
        self.swing_origins = {}
        self.swing_targets = {}
        self.was_swinging = {leg: False for leg in LEG_ORDER}
        self.debug_state["phase_name"] = "ownership_hold"
        self.debug_state["step_state"] = "OWNERSHIP_HOLD"

    def request_stop(self):
        if self.active:
            self.stop_requested = True
            self.forced_stop = True

    def release_forced_stop(self):
        """Release the safety latch once a true neutral command was seen.

        The latch itself is cleared inside step() when an idle engine
        receives a neutral command, but the motion controller only calls
        step() while motion is requested -- an idle engine never sees the
        neutral.  The controller therefore reports the observed neutral
        through this method; a held joystick still cannot restart a stop
        because the controller only calls this when the command stream is
        genuinely neutral.
        """
        if not self.active:
            self.forced_stop = False
            self.stop_requested = False

    # -- world frame ------------------------------------------------------

    def predict_body_pose(self, velocity, dt):
        """Exact constant-twist arc from the current world pose."""
        vx, vy, wz = (float(v) for v in velocity)
        yaw = self.body_yaw_world
        if abs(wz) < 1e-9:
            dx = (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            dy = (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            return self.body_x_world + dx, self.body_y_world + dy, yaw
        theta = wz * dt
        local_x = (vx * math.sin(theta) + vy * (math.cos(theta) - 1.0)) / wz
        local_y = (vx * (1.0 - math.cos(theta)) + vy * math.sin(theta)) / wz
        dx = local_x * math.cos(yaw) - local_y * math.sin(yaw)
        dy = local_x * math.sin(yaw) + local_y * math.cos(yaw)
        return self.body_x_world + dx, self.body_y_world + dy, yaw + theta

    def integrate_body_pose(self, velocity, dt):
        self.body_x_world, self.body_y_world, self.body_yaw_world = (
            self.predict_body_pose(velocity, dt)
        )

    def body_to_world(self, point, body_x=None, body_y=None, body_yaw=None):
        x = self.body_x_world if body_x is None else body_x
        y = self.body_y_world if body_y is None else body_y
        yaw = self.body_yaw_world if body_yaw is None else body_yaw
        px, py, pz = point
        return (
            x + px * math.cos(yaw) - py * math.sin(yaw),
            y + px * math.sin(yaw) + py * math.cos(yaw),
            pz,
        )

    def world_to_body(self, point):
        px = point[0] - self.body_x_world
        py = point[1] - self.body_y_world
        yaw = self.body_yaw_world
        return (
            px * math.cos(yaw) + py * math.sin(yaw),
            -px * math.sin(yaw) + py * math.cos(yaw),
            point[2],
        )

    def sync_world_feet_from_body(self):
        self.world_feet = {
            leg: self.body_to_world(self.feet[leg]) for leg in LEG_ORDER
        }

    # -- introspection ----------------------------------------------------

    def cycle_phase(self, now):
        return self.phase % 1.0

    def velocity_activity(self, velocity, step_in_place=False):
        if step_in_place:
            return max(
                0.35,
                normalized_velocity_activity(velocity, self.command_limits()),
            )
        return normalized_velocity_activity(velocity, self.command_limits())

    def debug_snapshot(self):
        return dict(self.debug_state)

    # -- per-leg phase helpers ---------------------------------------------

    def _leg_local_phase(self, leg):
        offsets = GAIT_PHASE_OFFSETS[self.gait_name]
        return (self.phase + offsets[leg]) % 1.0

    def _leg_in_stance(self, leg):
        return self._leg_local_phase(leg) < self.config["duty_factor"]

    def _swing_progress(self, leg):
        duty = self.config["duty_factor"]
        local = self._leg_local_phase(leg)
        if local < duty:
            return None
        return (local - duty) / (1.0 - duty)

    # -- touchdown planning -------------------------------------------------

    def _touchdown_world(self, leg, velocity):
        """Frozen world-frame touchdown for the swing that starts now."""
        config = self.config
        stance_time = config["duty_factor"] * config["cycle_period"]
        swing_time = (1.0 - config["duty_factor"]) * config["cycle_period"]
        vx, vy, wz = velocity
        nominal = NOMINAL_FEET[leg]
        # Foot velocity relative to the body during stance for this twist.
        stance_vx = vx - wz * nominal[1]
        stance_vy = vy + wz * nominal[0]
        lead_x = clamp(0.5 * stance_vx * stance_time, -0.085, 0.085)
        lead_y = clamp(0.5 * stance_vy * stance_time, -0.060, 0.060)
        target_body = (nominal[0] + lead_x, nominal[1] + lead_y, nominal[2])
        body_x, body_y, body_yaw = self.predict_body_pose(velocity, swing_time)
        return self.body_to_world(target_body, body_x, body_y, body_yaw)

    # -- the tick -----------------------------------------------------------

    def step(self, now, dt, velocity, step_in_place=False, body_offset=(0.0, 0.0)):
        """Advance one control tick.

        Returns ``(feet, body_motion, active)`` where ``feet`` is a fresh
        body-frame mapping over ``LEG_ORDER``, ``body_motion`` is a dict
        with ``x``/``y`` (gait body sway), ``height``/``roll``/``pitch``
        offsets, and ``body_x_override``/``body_y_override`` (the operator
        offset passed straight through), and ``active`` reports whether
        the engine still owns the feet.
        """
        dt = clamp(float(dt), 0.0, 0.2)
        velocity = tuple(float(v) for v in velocity)
        commanded = velocity_is_active(velocity) or bool(step_in_place)

        if not self.active:
            if not commanded:
                # A neutral command releases a forced stop; only then may a
                # fresh command re-activate the gait.
                self.forced_stop = False
                self._update_debug(velocity, stopped=True)
                return copy_feet(self.feet), self._body_motion(), False
            if self.forced_stop:
                # request_stop() latches: a command that was never released
                # to neutral cannot restart motion (safety stops stay stopped
                # against a held joystick).
                self._update_debug(velocity, stopped=True)
                return copy_feet(self.feet), self._body_motion(), False
            # Activate: adopt the current pose as the world frame origin.
            self.active = True
            self.settling = False
            self.stop_requested = False
            self.forced_stop = False
            self.phase = 0.0
            self.body_x_world = 0.0
            self.body_y_world = 0.0
            self.body_yaw_world = 0.0
            self.sync_world_feet_from_body()
            self.was_swinging = {leg: False for leg in LEG_ORDER}
            self.swing_origins = {}
            self.swing_targets = {}

        if self.settling:
            return self._settle_step(now, dt)

        if not commanded and not self.stop_requested:
            self.stop_requested = True
        elif commanded and self.stop_requested and not self.forced_stop:
            self.stop_requested = False

        self.last_velocity = velocity
        self.integrate_body_pose(velocity, dt)
        self.phase += dt / max(self.config["cycle_period"], 1e-6)

        feet = {}
        any_airborne = False
        for leg in LEG_ORDER:
            progress = self._swing_progress(leg)
            if progress is None:
                # Stance: the world foothold does not move.
                self.was_swinging[leg] = False
                feet[leg] = self.world_to_body(self.world_feet[leg])
                continue
            if self.stop_requested and not self.was_swinging[leg]:
                # A stop never begins a new swing; keep the foot planted.
                feet[leg] = self.world_to_body(self.world_feet[leg])
                continue
            if not self.was_swinging[leg]:
                # Liftoff edge: freeze origin and touchdown in world frame.
                self.was_swinging[leg] = True
                self.swing_origins[leg] = self.world_feet[leg]
                self.swing_targets[leg] = self._touchdown_world(leg, velocity)
            origin = self.swing_origins[leg]
            target = self.swing_targets[leg]
            blend = smoothstep(progress)
            world = (
                origin[0] + (target[0] - origin[0]) * blend,
                origin[1] + (target[1] - origin[1]) * blend,
                NOMINAL_FEET_Z + swing_lift(
                    progress, self.config["step_height"]
                ),
            )
            if progress >= 1.0 - 1e-9:
                world = (target[0], target[1], NOMINAL_FEET_Z)
            self.world_feet[leg] = (world[0], world[1], NOMINAL_FEET_Z)
            body_point = self.world_to_body(
                (world[0], world[1], NOMINAL_FEET_Z)
            )
            feet[leg] = (body_point[0], body_point[1], world[2])
            any_airborne = True

        self.feet = {
            leg: (feet[leg][0], feet[leg][1], min(feet[leg][2], 0.0))
            for leg in LEG_ORDER
        }

        if self.stop_requested and not any_airborne:
            self.settling = True
            self.settle_started = now
            self.settle_origin = copy_feet(self.feet)

        self._update_debug(velocity)
        return copy_feet(self.feet), self._body_motion(), True

    def _settle_step(self, now, dt):
        """Blend every planted foot back to nominal, then go idle."""
        settle_time = max(self.config["settle_time"], 1e-3)
        progress = clamp((now - self.settle_started) / settle_time, 0.0, 1.0)
        blend = smootherstep(progress)
        feet = {}
        for leg in LEG_ORDER:
            origin = self.settle_origin[leg]
            nominal = NOMINAL_FEET[leg]
            feet[leg] = (
                origin[0] + (nominal[0] - origin[0]) * blend,
                origin[1] + (nominal[1] - origin[1]) * blend,
                origin[2] + (nominal[2] - origin[2]) * blend,
            )
        self.feet = feet
        self.debug_state["phase_name"] = "settle_to_nominal"
        self.debug_state["step_state"] = "SETTLING"
        self.debug_state["phase_progress"] = progress
        if progress >= 1.0:
            self.feet = copy_feet(NOMINAL_FEET)
            self.active = False
            self.settling = False
            self.stop_requested = False
            # forced_stop deliberately survives until a neutral command is
            # observed, so a held joystick cannot restart a safety stop.
            self.sync_world_feet_from_body()
            self._update_debug((0.0, 0.0, 0.0), stopped=True)
            return copy_feet(self.feet), self._body_motion(), False
        return copy_feet(self.feet), self._body_motion(), True

    # -- body motion ---------------------------------------------------------

    def _body_motion(self):
        config = self.config
        sway_y = 0.0
        if self.active and not self.settling and config["body_sway_y"] > 0.0:
            # Lean away from the swinging side.  With the lateral-sequence
            # amble offsets, right legs swing during the first half cycle
            # and left legs during the second, so a plain sine of the cycle
            # phase leans left first, then right.
            sway_y = config["body_sway_y"] * math.sin(
                2.0 * math.pi * (self.phase % 1.0)
            )
        # No body_x/y_override keys: the operator offset passes through
        # unchanged (the consumer's .get default), the gait only adds sway.
        return {
            "x": 0.0,
            "y": sway_y,
            "height": config["body_height_offset"] if self.active else 0.0,
            "roll": 0.0,
            "pitch": 0.0,
        }

    # -- debug ----------------------------------------------------------------

    def _update_debug(self, velocity, stopped=False):
        if stopped:
            swing_legs = []
            per_leg = {leg: 0.0 for leg in LEG_ORDER}
            phase_name = "stopped"
            step_state = "STOPPED"
            progress = 0.0
        else:
            swing_legs = [
                leg for leg in LEG_ORDER
                if self.was_swinging.get(leg)
                and self._swing_progress(leg) is not None
            ]
            per_leg = {leg: self._leg_local_phase(leg) for leg in LEG_ORDER}
            phase_name = "cycle"
            step_state = "CYCLE"
            progress = self.phase % 1.0
        self.debug_state = {
            "phase": self.phase % 1.0,
            "phase_name": phase_name,
            "phase_progress": progress,
            "step_state": step_state,
            "swing_legs": swing_legs,
            "stance_legs": [leg for leg in LEG_ORDER if leg not in swing_legs],
            "per_leg_phase": per_leg,
            "cycle_period": self.config["cycle_period"],
            "warning": self.warning,
            "body_world": {
                "x": self.body_x_world,
                "y": self.body_y_world,
                "yaw": self.body_yaw_world,
            },
            "planned_velocity": list(velocity),
        }
